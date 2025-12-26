
// FBX2ITP.cpp : Reads an .fbx file and writes vertex data to a JSON file.
// Usage: FBX2ITP.exe input.fbx output.json
// Requires Autodesk FBX SDK installed and linked (libfbxsdk.lib).

#include "VertexFormat.h"
#include "FbxHelper.h"
#include "ItpMesh.h"
#include <array>
#include <cmath>
#include <iomanip>
#include <string>
#include <unordered_map>
#include <vector>

static bool s_doBlendShapes = true;


// Read blendshapes (blend shape deformers) from an FbxMesh.
// For each blendshape channel + each target shape, compute per-control-point deltas
// (targetPosition - basePosition) and also compute per-control-point normals and tangents
// for the target by re-evaluating triangle normals/tangents using the target positions.
static void ReadBlendShapes(FbxMesh* mesh, ItpMesh::Mesh* out)
{
    if (!mesh || !out)
        return;

    int deformerCount = mesh->GetDeformerCount(FbxDeformer::eBlendShape);
    if (deformerCount == 0)
        return;

    // how many vertices in the base mesh
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

                ItpMesh::BlendShape bs;
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

                bs.deltas.resize(static_cast<size_t>(out->verts.size()));
                for (int i = 0; i < baseCount; ++i)
                {
                    uint32_t baseIndex = out->vertexMap[static_cast<uint32_t>(i)][0];
                    VertexPosNormTanUV& baseVert = out->verts[baseIndex];
                    VertexPosNormTan vert;
                    float dx = static_cast<float>(shapeControlPoints[i][0]);
                    float dy = static_cast<float>(shapeControlPoints[i][1]);
                    float dz = static_cast<float>(shapeControlPoints[i][2]);
                    vert.pos = Vector3(dx, dy, dz) - baseVert.pos;

                    if (bs.format.hasNormal)
                    {
                        int idx = (elemNormal->GetReferenceMode() == FbxGeometryElement::eDirect) ?
                            i : elemNormal->GetIndexArray().GetAt(i);
                        FbxVector4 n = elemNormal->GetDirectArray().GetAt(idx);
                        vert.norm = Vector3(static_cast<float>(n[0]), static_cast<float>(n[1]), static_cast<float>(n[2]));
                        vert.norm -= baseVert.norm;
                    }
                    if (bs.format.hasTan)
                    {
                        int idx = (elemTangent->GetReferenceMode() == FbxGeometryElement::eDirect) ?
                            i : elemTangent->GetIndexArray().GetAt(i);
                        FbxVector4 n = elemTangent->GetDirectArray().GetAt(idx);
                        vert.tan = Vector3(static_cast<float>(n[0]), static_cast<float>(n[1]), static_cast<float>(n[2]));
                        vert.tan -= baseVert.tan;
                    }

                    for (uint32_t vi : out->vertexMap[static_cast<uint32_t>(i)])
                    {
                        // For each duplicated vertex, add the same delta
                        bs.deltas[vi] = vert;
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

static void ProcessMeshToItp(FbxMesh* mesh, ItpMesh::Mesh* out, int index)
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
    std::unordered_map<VertexPosNormTanUV, size_t> vertexMap;
    for (int p = 0; p < polygonCount; ++p)
    {
        int polySize = mesh->GetPolygonSize(p);
        for (int v = 0; v < polySize; ++v)
        {
            int ctrlPointIndex = mesh->GetPolygonVertex(p, v);

            VertexPosNormTanUV vert;
            FbxVector4 pos = mesh->GetControlPointAt(ctrlPointIndex);

            FbxVector4 normal; bool hasNormal = FbxHelper::GetNormalAt(mesh, p, v, normal);
            FbxVector2 uv; bool hasUV = FbxHelper::GetUVAt(mesh, p, v, uv, nullptr);
            FbxVector4 tangent; bool hasTangent = FbxHelper::GetTangentAt(mesh, p, v, tangent);

            vert.pos = Vector3(static_cast<float>(pos[0]), static_cast<float>(pos[1]), static_cast<float>(pos[2]));
            if (hasNormal)
                vert.norm = Vector3(static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2]));
            else
                vert.norm = Vector3(0.0f, 0.0f, 0.0f);
            if (hasTangent)
                vert.tan = Vector3(static_cast<float>(tangent[0]), static_cast<float>(tangent[1]), static_cast<float>(tangent[2]));
            else
                vert.tan = Vector3(0.0f, 0.0f, 0.0f);
            if (hasUV)
            {
                uv[1] = 1.0 - uv[1]; // flip V
                vert.uv = Vector2(static_cast<float>(uv[0]), static_cast<float>(uv[1]));
            }
            else
                vert.uv = Vector2(0.0f, 0.0f);

            size_t index = 0;
            auto inMap = vertexMap.find(vert);
            if (inMap != vertexMap.end())
            {
                index = inMap->second;
            }
            else
            {
                index = vertexMap.size();
                vertexMap[vert] = index;
                out->verts.emplace_back(vert);
                out->vertexMap[static_cast<uint32_t>(ctrlPointIndex)].push_back(static_cast<uint32_t>(index));
            }
            // reverse the winding order
            out->indices[p].index[2 - v] = static_cast<uint32_t>(index);
        }
    }

    if (s_doBlendShapes)
    {   // read blend shapes
        ReadBlendShapes(mesh, out);
    }
}

static void WriteMesh(FbxMesh* mesh, int index)
{
    ItpMesh::Mesh itpMesh;
    ProcessMeshToItp(mesh, &itpMesh, index);
    std::cout << itpMesh.name << "\n";

    {   // Open output file
        std::string outputPath = itpMesh.name + ".itpmesh3";
        std::ofstream ofs(outputPath, std::ofstream::out | std::ofstream::trunc);
        if (!ofs.is_open())
        {
            std::cerr << "Failed to open output file: " << outputPath << "\n";
        }
        else
        {
            ofs << std::showpoint;
            itpMesh.WriteToJson(ofs);
            ofs.close();
        }
    }

    if (s_doBlendShapes && !itpMesh.blendShapes.empty())
    {
        std::cout << "  BlendShapes:\n";
        for (const auto& bs : itpMesh.blendShapes)
        {
            std::cout << "    " << bs.name << " (deltas: " << bs.deltas.size() << ")\n";

            // Open output file
            std::string outputPath = bs.name + ".itpblend";
            std::ofstream ofs(outputPath, std::ofstream::out | std::ofstream::trunc);
            if (!ofs.is_open())
            {
                std::cerr << "Failed to open output file: " << outputPath << "\n";
            }
            else
            {
                ofs << std::showpoint;
                bs.WriteToJson(ofs);
                ofs.close();
            }
        }
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