
// FBX2ITP.cpp : Reads an .fbx file and writes vertex data to a JSON file.
// Usage: FBX2ITP.exe input.fbx output.json
// Requires Autodesk FBX SDK installed and linked (libfbxsdk.lib).

#include "VertexFormat.h"
#include <fbxsdk.h>
#include <array>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>

static bool s_doBlendShapes = false;

struct VertexFormat
{
    bool hasNormal = false;
    bool hasTan = false;
    bool hasUV = false;
};

struct BlendShape
{
    std::string name;                 // channel name (and target index when multiple targets)
    VertexFormat format;
    std::vector<VertexPosNormTan> deltas;      // per-control-point delta (target - base). size == mesh->GetControlPointsCount()
};

struct Mesh
{
    struct Triangle {
        uint32_t index[3];
    };
    std::string name;
    VertexFormat format;
    std::vector<VertexPosNormTanUV> verts;
    std::vector<Triangle> indices; // assuming triangles

    // new: blendshape targets
    std::vector<BlendShape> blendShapes;
};

// Helper to fetch normal for a polygon-vertex
static bool GetNormalAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector4& outNormal)
{
    if (!mesh) return false;
    fbxsdk::FbxGeometryElementNormal* elemNormal = mesh->GetElementNormal(0);
    if (!elemNormal) return false;

    if (elemNormal->GetMappingMode() == FbxGeometryElement::eByControlPoint)
    {
        int controlPointIndex = mesh->GetPolygonVertex(polyIndex, vertIndex);
        int index = (elemNormal->GetReferenceMode() == FbxGeometryElement::eDirect) ?
            controlPointIndex :
            elemNormal->GetIndexArray().GetAt(controlPointIndex);
        outNormal = elemNormal->GetDirectArray().GetAt(index);
        return true;
    }
    else if (elemNormal->GetMappingMode() == FbxGeometryElement::eByPolygonVertex)
    {
        int indexByPolygonVertex = mesh->GetPolygonVertexIndex(polyIndex) + vertIndex;
        int index = (elemNormal->GetReferenceMode() == FbxGeometryElement::eDirect) ?
            indexByPolygonVertex :
            elemNormal->GetIndexArray().GetAt(indexByPolygonVertex);
        outNormal = elemNormal->GetDirectArray().GetAt(index);
        return true;
    }
    // other mapping modes possible, not handled here
    return false;
}

// Helper to fetch tangent for a polygon-vertex
static bool GetTangentAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector4& outTangent)
{
    if (!mesh) return false;
    fbxsdk::FbxGeometryElementTangent* elemTangent = mesh->GetElementTangent(0);
    if (!elemTangent) return false;

    if (elemTangent->GetMappingMode() == FbxGeometryElement::eByControlPoint)
    {
        int controlPointIndex = mesh->GetPolygonVertex(polyIndex, vertIndex);
        int index = (elemTangent->GetReferenceMode() == FbxGeometryElement::eDirect) ?
            controlPointIndex :
            elemTangent->GetIndexArray().GetAt(controlPointIndex);
        outTangent = elemTangent->GetDirectArray().GetAt(index);
        return true;
    }
    else if (elemTangent->GetMappingMode() == FbxGeometryElement::eByPolygonVertex)
    {
        int indexByPolygonVertex = mesh->GetPolygonVertexIndex(polyIndex) + vertIndex;
        int index = (elemTangent->GetReferenceMode() == FbxGeometryElement::eDirect) ?
            indexByPolygonVertex :
            elemTangent->GetIndexArray().GetAt(indexByPolygonVertex);
        outTangent = elemTangent->GetDirectArray().GetAt(index);
        return true;
    }
    // other mapping modes possible, not handled here
    return false;
}

// Helper to fetch UV for a polygon-vertex
static bool GetUVAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector2& outUV, const char* uvName = nullptr)
{
    if (!mesh) return false;
    FbxStringList uvSetNameList;
    mesh->GetUVSetNames(uvSetNameList);
    if (uvSetNameList.GetCount() == 0) return false;

    const char* name = uvName;
    if (!name) name = uvSetNameList.GetStringAt(0);

    bool unmapped;
    if (mesh->GetPolygonVertexUV(polyIndex, vertIndex, name, outUV, unmapped))
    {
        if (unmapped) return false;
        return true;
    }
    return false;
}

// New function: Read blendshapes (blend shape deformers) from an FbxMesh.
// For each blendshape channel + each target shape, compute per-control-point deltas
// (targetPosition - basePosition) and also compute per-control-point normals and tangents
// for the target by re-evaluating triangle normals/tangents using the target positions.
static void ReadBlendShapes(FbxMesh* mesh, Mesh* out)
{
    if (!mesh || !out)
        return;

    int deformerCount = mesh->GetDeformerCount(FbxDeformer::eBlendShape);
    if (deformerCount == 0)
        return;

    // base control points (original mesh positions)
    FbxVector4* baseControlPoints = mesh->GetControlPoints();
    int baseCount = mesh->GetControlPointsCount();
    if (baseCount == 0)
        return;

    // For each blendshape deformer
    for (int d = 0; d < deformerCount; ++d)
    {
        FbxBlendShape* blendShape = static_cast<FbxBlendShape*>(mesh->GetDeformer(d, FbxDeformer::eBlendShape));
        if (!blendShape) continue;

        int channelCount = blendShape->GetBlendShapeChannelCount();
        for (int c = 0; c < channelCount; ++c)
        {
            FbxBlendShapeChannel* channel = blendShape->GetBlendShapeChannel(c);
            if (!channel) continue;

            int targetCount = channel->GetTargetShapeCount();
            if (targetCount == 0) continue;

            for (int t = 0; t < targetCount; ++t)
            {
                FbxShape* shape = channel->GetTargetShape(t);
                if (!shape) continue;

                FbxVector4* shapeControlPoints = shape->GetControlPoints();
                int shapeCount = shape->GetControlPointsCount();

                if (shapeCount != baseCount)
                {
                    std::cerr << "Warning: blend target control point count (" << shapeCount
                        << ") != base control point count (" << baseCount << ") for channel '"
                        << channel->GetName() << "' target " << t << ". Skipping target.\n";
                    continue;
                }

                BlendShape bs;
                bs.name = std::string(channel->GetName());
                if (targetCount > 1)
                    bs.name += "_target" + std::to_string(t);

                fbxsdk::FbxGeometryElementNormal* elemNormal = shape->GetElementNormal(0);
                if (out->format.hasNormal)
                {
                    bs.format.hasNormal = elemNormal
                        && (elemNormal->GetMappingMode() == FbxGeometryElement::eByControlPoint);
                }
                fbxsdk::FbxGeometryElementTangent* elemTangent = shape->GetElementTangent(0);
                if (out->format.hasTan)
                {
                    bs.format.hasTan = elemTangent
                        && (elemTangent->GetMappingMode() == FbxGeometryElement::eByControlPoint);
                }

                bs.deltas.resize(baseCount);
                for (int i = 0; i < baseCount; ++i)
                {
                    float dx = static_cast<float>(shapeControlPoints[i][0]);
                    float dy = static_cast<float>(shapeControlPoints[i][1]);
                    float dz = static_cast<float>(shapeControlPoints[i][2]);
                    bs.deltas[i].pos = Vector3(dx, dy, dz) - out->verts[i].pos;

                    if (bs.format.hasNormal)
                    {
                        int idx = (elemNormal->GetReferenceMode() == FbxGeometryElement::eDirect) ?
                            i : elemNormal->GetIndexArray().GetAt(i);
                        FbxVector4 n = elemNormal->GetDirectArray().GetAt(idx);
                        bs.deltas[i].norm = Vector3(static_cast<float>(n[0]), static_cast<float>(n[1]), static_cast<float>(n[2]));
                        bs.deltas[i].norm -= out->verts[i].norm;
                    }
                    if (bs.format.hasTan)
                    {
                        int idx = (elemTangent->GetReferenceMode() == FbxGeometryElement::eDirect) ?
                            i : elemTangent->GetIndexArray().GetAt(i);
                        FbxVector4 n = elemTangent->GetDirectArray().GetAt(idx);
                        bs.deltas[i].tan = Vector3(static_cast<float>(n[0]), static_cast<float>(n[1]), static_cast<float>(n[2]));
                        bs.deltas[i].tan -= out->verts[i].tan;
                    }
                }

                out->blendShapes.push_back(std::move(bs));

                std::cout << "Found blendshape channel '" << channel->GetName()
                    << "' target " << t << " -> '" << out->blendShapes.back().name
                    << "' (control points: " << baseCount << ")\n";
            } // target
        } // channel
    } // deformer
}

static void ProcessMeshToItp(FbxMesh* mesh, Mesh* out, int index)
{
    if (!mesh)
        return;
    FbxNode* node = mesh->GetNode();
    out->name = node ? node->GetName() : "mesh_" + std::to_string(index);

    fbxsdk::FbxGeometryElementNormal* elemN = mesh->GetElementNormal(0);
    fbxsdk::FbxGeometryElementUV* elemUV = mesh->GetElementUV(0); // may be nullptr
    fbxsdk::FbxGeometryElementTangent* elemT = mesh->GetElementTangent(0);
    out->format.hasNormal = (elemN != nullptr);
    out->format.hasUV = (elemUV != nullptr);
    out->format.hasTan = (elemT != nullptr);

    int polygonCount = mesh->GetPolygonCount();
    out->indices.resize(polygonCount);
    out->verts.resize(mesh->GetControlPointsCount());
    for (int p = 0; p < polygonCount; ++p)
    {
        int polySize = mesh->GetPolygonSize(p);
        for (int v = 0; v < polySize; ++v)
        {
            int ctrlPointIndex = mesh->GetPolygonVertex(p, v);

            // reverse the winding order
            out->indices[p].index[2 - v] = ctrlPointIndex;

            FbxVector4 pos = mesh->GetControlPointAt(ctrlPointIndex);

            FbxVector4 normal; bool hasNormal = GetNormalAt(mesh, p, v, normal);
            FbxVector2 uv; bool hasUV = GetUVAt(mesh, p, v, uv, nullptr);
            FbxVector4 tangent; bool hasTangent = GetTangentAt(mesh, p, v, tangent);

            out->verts[ctrlPointIndex].pos = Vector3(static_cast<float>(pos[0]), static_cast<float>(pos[1]), static_cast<float>(pos[2]));
            if (hasNormal)
                out->verts[ctrlPointIndex].norm = Vector3(static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2]));
            else
                out->verts[ctrlPointIndex].norm = Vector3(0.0f, 0.0f, 0.0f); // or compute later
            if (hasTangent)
                out->verts[ctrlPointIndex].tan = Vector3(static_cast<float>(tangent[0]), static_cast<float>(tangent[1]), static_cast<float>(tangent[2]));
            else
                out->verts[ctrlPointIndex].tan = Vector3(0.0f, 0.0f, 0.0f); // or compute later
            if (hasUV)
            {
                uv[1] = 1.0 - uv[1]; // flip V
                out->verts[ctrlPointIndex].uv = Vector2(static_cast<float>(uv[0]), static_cast<float>(uv[1]));
            }
            else
                out->verts[ctrlPointIndex].uv = Vector2(0.0f, 0.0f);
        }
    }

    out->indices.reserve(polygonCount); // assuming triangles
    for (int p = 0; p < polygonCount; ++p)
    {
        uint32_t tri[3];
        for (int v = 0; v < 3; ++v)
        {
            tri[v] = mesh->GetPolygonVertex(p, v);
        }
        out->indices.push_back({ tri[0], tri[1], tri[2] });
    }

    if (s_doBlendShapes)
    {   // read blend shapes
        ReadBlendShapes(mesh, out);
    }
}

static std::string Indent(int indent)
{
    std::string indentStr = "";
    for (int i = 0; i < indent; ++i)
    {
        indentStr += "\t";
    }
    return indentStr;
}

static void WriteFormatToJson(const VertexFormat& format, int indent, std::ofstream& ofs)
{
    std::string in = Indent(indent);
    ofs << in << "\"vertexformat\": [\n";
    ofs << in << "\t{\n";
    ofs << in << "\t\t\"name\": \"position\",\n";
    ofs << in << "\t\t\"type\" : \"float\",\n";
    ofs << in << "\t\t\"count\" : 3\n";
    ofs << in << "\t}";
    if (format.hasNormal)
    {
        ofs << ",\n";
        ofs << in << "\t{\n";
        ofs << in << "\t\t\"name\": \"normal\",\n";
        ofs << in << "\t\t\"type\": \"float\",\n";
        ofs << in << "\t\t\"count\": 3\n";
        ofs << in << "\t}";
    }
    if (format.hasTan)
    {
        ofs << ",\n";
        ofs << in << "\t{\n";
        ofs << in << "\t\t\"name\": \"tangent\",\n";
        ofs << in << "\t\t\"type\": \"float\",\n";
        ofs << in << "\t\t\"count\": 3\n";
        ofs << in << "\t}";
    }
    if (format.hasUV)
    {
        ofs << ",\n";
        ofs << in << "\t{\n";
        ofs << in << "\t\t\"name\": \"texcoord\",\n";
        ofs << in << "\t\t\"type\": \"float\",\n";
        ofs << in << "\t\t\"count\": 2\n";
        ofs << in << "\t}";
    }
    ofs << "\n" << in << "], \n";
}

static void WriteVertToJson(const Mesh* mesh, const VertexPosNormTanUV& vert, std::ofstream& ofs)
{
    ofs << "\t\t[ ";
    ofs << vert.pos.x << ", " << vert.pos.y << ", " << vert.pos.z;
    if (mesh->format.hasNormal)
    {
        ofs << ", " << vert.norm.x << ", " << vert.norm.y << ", " << vert.norm.z;
    }
    if (mesh->format.hasTan)
    {
        ofs << ", " << vert.tan.x << ", " << vert.tan.y << ", " << vert.tan.z;
    }
    if (mesh->format.hasUV)
    {
        ofs << ", " << vert.uv.x << ", " << vert.uv.y;
    }

    ofs << " ]";
}

static void WriteVertsToJson(const Mesh* mesh, std::ofstream& ofs)
{
    ofs << "\t\"vertices\": [\n";
    WriteVertToJson(mesh, mesh->verts[0], ofs);
    for (size_t i = 1; i < mesh->verts.size(); ++i)
    {
        ofs << ",\n";
        WriteVertToJson(mesh, mesh->verts[i], ofs);
    }
    ofs << "\n\t],\n";
}

static void WriteTriToJson(const Mesh::Triangle& tri, std::ofstream& ofs)
{
    ofs << "\t\t[ " << tri.index[0] << ", " << tri.index[1] << ", "
        << tri.index[2] << " ]";
}

static void WriteIndicesToJson(const Mesh* mesh, std::ofstream& ofs)
{
    ofs << "\t\"indices\": [\n";
    WriteTriToJson(mesh->indices[0], ofs);
    for (size_t i = 1; i < mesh->indices.size(); ++i)
    {
        ofs << ",\n";
        WriteTriToJson(mesh->indices[i], ofs);
    }
    ofs << "\n\t]";
}

static void WriteDeltaToJson(const BlendShape& bs, const VertexPosNormTan& vert, std::ofstream& ofs)
{
    ofs << "\t\t\t\t[ ";
    ofs << vert.pos.x << ", " << vert.pos.y << ", " << vert.pos.z;
    if (bs.format.hasNormal)
    {
        ofs << ", " << vert.norm.x << ", " << vert.norm.y << ", " << vert.norm.z;
    }
    if (bs.format.hasTan)
    {
        ofs << ", " << vert.tan.x << ", " << vert.tan.y << ", " << vert.tan.z;
    }

    ofs << " ]";
}

static void WriteDeltasToJson(const Mesh* mesh, const BlendShape& bs, std::ofstream& ofs)
{
    ofs << "\t\t\t\"deltas\": [\n";
    if (!bs.deltas.empty())
    {
        WriteDeltaToJson(bs, bs.deltas[0], ofs);
        for (size_t i = 1; i < bs.deltas.size(); ++i)
        {
            ofs << ",\n";
            WriteDeltaToJson(bs, bs.deltas[i], ofs);
        }
    }
    ofs << "\n\t\t\t]\n";
}

static void WriteMeshToJson(const Mesh* mesh, std::ofstream& ofs)
{
    ofs << "{\n";
    ofs << "\t\"metadata\": {\n";
    ofs << "\t\t\"type\": \"itpmesh\",\n";
    ofs << "\t\t\"version\" : 3\n";
    ofs << "\t},\n";
    ofs << "\t\"material\" : \"Assets/Materials/" << mesh->name << ".itpmat\",\n";
    WriteFormatToJson(mesh->format, 1, ofs);
    WriteVertsToJson(mesh, ofs);
    WriteIndicesToJson(mesh, ofs);

    if (s_doBlendShapes)
    {   // write blendshapes
        if (!mesh->blendShapes.empty())
        {
            ofs << ",\n\t\"blendshapes\": [\n";
            for (size_t b = 0; b < mesh->blendShapes.size(); ++b)
            {
                const BlendShape& bs = mesh->blendShapes[b];
                ofs << "\t\t{\n";
                ofs << "\t\t\t\"name\": \"" << bs.name << "\",\n";
                WriteFormatToJson(bs.format, 3, ofs);
                // deltas
                WriteDeltasToJson(mesh, bs, ofs);

                ofs << "\t\t}";
                if (b + 1 < mesh->blendShapes.size()) ofs << ",\n";
            }
            ofs << "\n\t]";
        }
    }

    ofs << "\n}\n";
}

static void WriteMesh(FbxMesh* mesh, int index)
{
    Mesh itpMesh;
    ProcessMeshToItp(mesh, &itpMesh, index);
    std::cout << itpMesh.name << "\n";

    // Open output file
    std::string outputPath = itpMesh.name + ".itpmesh3";
    std::ofstream ofs(outputPath, std::ofstream::out | std::ofstream::trunc);
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open output file: " << outputPath << "\n";
    }
    else
    {
        ofs << std::showpoint;
        WriteMeshToJson(&itpMesh, ofs);
        ofs.close();
    }
}

static void WriteAllMesh(FbxNode* node, int& index)
{
    if (!node)
        return;
    FbxMesh* mesh = node->GetMesh();
    if (mesh)
    {
        WriteMesh(mesh, index);
        index++;
    }
    for (int i = 0; i < node->GetChildCount(); ++i)
    {
        WriteAllMesh(node->GetChild(i), index);
    }
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << (argc > 0 ? argv[0] : "FBX2ITP") << " input.fbx\n";
        return 1;
    }
    const char* inputPath = argv[1];

    // Initialize SDK manager
    FbxManager* sdkManager = FbxManager::Create();
    if (!sdkManager)
    {
        std::cerr << "Failed to create FBX Manager\n";
        return 1;
    }

    // IO settings
    FbxIOSettings* ios = FbxIOSettings::Create(sdkManager, IOSROOT);
    sdkManager->SetIOSettings(ios);

    // Create importer
    FbxImporter* importer = FbxImporter::Create(sdkManager, "");
    if (!importer->Initialize(inputPath, -1, sdkManager->GetIOSettings()))
    {
        std::cerr << "Failed to initialize importer for: " << inputPath << "\n";
        std::cerr << "Error: " << importer->GetStatus().GetErrorString() << "\n";
        importer->Destroy();
        sdkManager->Destroy();
        return 1;
    }

    // Create scene and import
    FbxScene* scene = FbxScene::Create(sdkManager, "scene");
    if (!importer->Import(scene))
    {
        std::cerr << "Failed to import scene\n";
        importer->Destroy();
        sdkManager->Destroy();
        return 1;
    }
    importer->Destroy();

    FbxGeometryConverter converter(sdkManager);
    converter.Triangulate(scene, true); // The 'true' parameter ensures original nodes are replaced.

    int index = 0;
    WriteAllMesh(scene->GetRootNode(), index);

    // Cleanup
    sdkManager->Destroy();
    return 0;
}