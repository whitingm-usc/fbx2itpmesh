// FBX2ITP.cpp : Reads an .fbx file and writes vertex data to a JSON file.
// Usage: FBX2ITP.exe input.fbx output.json
// Requires Autodesk FBX SDK installed and linked (libfbxsdk.lib).

#include "VertexFormat.h"
#include <fbxsdk.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

struct Mesh
{
    struct Triangle {
        uint32_t index[3];
    };
    std::string name;
    bool hasNormal = false;
    bool hasTan = false;
    bool hasUV = false;
    std::vector<VertexPosNormTanUV> verts;
    std::vector<Triangle> indices; // assuming triangles
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

static void ProcessMeshToItp(FbxMesh* mesh, Mesh* out, int index)
{
    if (!mesh)
        return;
    FbxNode* node = mesh->GetNode();
    out->name = node ? node->GetName() : "mesh_" + std::to_string(index);

    fbxsdk::FbxGeometryElementNormal* elemN = mesh->GetElementNormal(0);
    fbxsdk::FbxGeometryElementUV* elemUV = mesh->GetElementUV(0); // may be nullptr
    fbxsdk::FbxGeometryElementTangent* elemT = mesh->GetElementTangent(0);
    out->hasNormal = (elemN != nullptr);
    out->hasUV = (elemUV != nullptr);
    out->hasTan = (elemT != nullptr);

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
            out->indices[p].index[2-v] = ctrlPointIndex;

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
}

static void WriteFormatToJson(const Mesh* mesh, std::ofstream& ofs)
{
    ofs << "\t\"vertexformat\": [\n";
    ofs << "\t\t{\n";
    ofs << "\t\t\t\"name\": \"position\",\n";
    ofs << "\t\t\t\"type\" : \"float\",\n";
    ofs << "\t\t\t\"count\" : 3\n";
    ofs << "\t\t}";
    if (mesh->hasNormal)
    {
        ofs << ",\n";
        ofs << "\t\t{\n";
        ofs << "\t\t\t\"name\": \"normal\",\n";
        ofs << "\t\t\t\"type\": \"float\",\n";
        ofs << "\t\t\t\"count\": 3\n";
        ofs << "\t\t}";
    }
    if (mesh->hasTan)
    {
        ofs << ",\n";
        ofs << "\t\t{\n";
        ofs << "\t\t\t\"name\": \"tangent\",\n";
        ofs << "\t\t\t\"type\": \"float\",\n";
        ofs << "\t\t\t\"count\": 3\n";
        ofs << "\t\t}";
    }
    if (mesh->hasUV)
    {
        ofs << ",\n";
        ofs << "\t\t{\n";
        ofs << "\t\t\t\"name\": \"texcoord\",\n";
        ofs << "\t\t\t\"type\": \"float\",\n";
        ofs << "\t\t\t\"count\": 2\n";
        ofs << "\t\t}";
    }
    ofs << "\n\t],\n";
}

static void WriteVertToJson(const Mesh* mesh, const VertexPosNormTanUV& vert, std::ofstream& ofs)
{
    ofs << "\t\t[ ";
    ofs << vert.pos.x << ", " << vert.pos.y << ", " << vert.pos.z;
    if (mesh->hasNormal)
    {
        ofs << ", " << vert.norm.x << ", " << vert.norm.y << ", " << vert.norm.z;
    }
    if (mesh->hasTan)
    {
        ofs << ", " << vert.tan.x << ", " << vert.tan.y << ", " << vert.tan.z;
    }
    if (mesh->hasUV)
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
    ofs << "\n\t]\n";
}

static void WriteMeshToJson(const Mesh* mesh, std::ofstream& ofs)
{
    ofs << "{\n";
    ofs << "\t\"metadata\": {\n";
    ofs << "\t\t\"type\": \"itpmesh\",\n";
    ofs << "\t\t\"version\" : 3\n";
    ofs << "\t},\n";
    ofs << "\t\"material\" : \"Assets/Materials/" << mesh->name << ".itpmat\",\n";
    WriteFormatToJson(mesh, ofs);
    WriteVertsToJson(mesh, ofs);
    WriteIndicesToJson(mesh, ofs);
    ofs << "}\n";
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

static FbxMesh* FindFirstMesh(FbxNode* node)
{
    if (!node)
        return nullptr;
    FbxMesh* mesh = node->GetMesh();
    if (mesh)
    {
        return mesh;
    }
    for (int i = 0; i < node->GetChildCount(); ++i)
    {
        mesh = FindFirstMesh(node->GetChild(i));
        if (mesh)
            return mesh;
    }
    return nullptr;
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

#if 0
    FbxMesh* mesh = FindFirstMesh(scene->GetRootNode());
    if (mesh)
    {
        std::string meshName;
        Mesh itpMesh;
        ProcessMeshToItp(mesh, &itpMesh, 0);
        std::cout << meshName << "\n";

        // Open output file
        std::string outputPath = meshName + ".itpmesh3";
        std::ofstream ofs(outputPath, std::ofstream::out | std::ofstream::trunc);
        if (!ofs.is_open())
        {
            std::cerr << "Failed to open output file: " << outputPath << "\n";
        }
        else
        {
            WriteMeshToJson(&itpMesh, ofs);
            ofs.close();
        }
    }
#else
    int index = 0;
    WriteAllMesh(scene->GetRootNode(), index);
#endif

    // Cleanup
    sdkManager->Destroy();
    return 0;
}