
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
#include <algorithm>

static bool s_doBlendShapes = true;
static bool s_doSkinning = true;


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
                    VertexData& baseVert = out->verts[baseIndex];
                    VertexData vert;
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

// Read skinning info and also populate the mesh bones (names + bind poses + parent indices).
// Returns true if any skinning data was found.
static bool ReadSkin(FbxMesh* mesh,
    std::vector<std::array<uint8_t, 4>>& ctrlBones,
    std::vector<std::array<uint8_t, 4>>& ctrlWeights,
    std::vector<ItpMesh::Bone>& outBones)
{
    int skinDeformerCount = mesh->GetDeformerCount(FbxDeformer::eSkin);
    int controlPointCount = mesh->GetControlPointsCount();

    // Per-control-point list of (boneIndex, weight)
    std::vector<std::vector<std::pair<uint8_t, float>>> cpInfluences;
    cpInfluences.resize(static_cast<size_t>(controlPointCount));

    // Map bone (link) name -> small integer index (uint8_t)
    std::unordered_map<std::string, uint8_t> boneNameToIndex;
    uint32_t nextBoneIndex = 0;

    // Keep arrays for node pointers and bind matrices indexed by boneIndex
    std::vector<FbxNode*> boneNodes;
    std::vector<FbxAMatrix> boneBindMatrices;

    for (int s = 0; s < skinDeformerCount; ++s)
    {
        FbxSkin* skin = static_cast<FbxSkin*>(mesh->GetDeformer(s, FbxDeformer::eSkin));
        if (!skin) continue;

        int clusterCount = skin->GetClusterCount();
        for (int c = 0; c < clusterCount; ++c)
        {
            FbxCluster* cluster = skin->GetCluster(c);
            if (!cluster) continue;

            FbxNode* linkNode = cluster->GetLink(); // bone node
            if (!linkNode) continue;

            std::string boneName = linkNode->GetName();
            uint8_t boneIndex = 0;
            auto it = boneNameToIndex.find(boneName);
            if (it == boneNameToIndex.end())
            {
                if (nextBoneIndex > 255)
                {
                    std::cerr << "Warning: too many bones - bone '" << boneName << "' ignored\n";
                    continue;
                }
                boneIndex = static_cast<uint8_t>(nextBoneIndex);
                boneNameToIndex[boneName] = boneIndex;
                ++nextBoneIndex;

                boneNodes.resize(nextBoneIndex);
                boneBindMatrices.resize(nextBoneIndex);
                boneNodes[boneIndex] = linkNode;

                // Get the link (bone) bind matrix and the mesh bind matrix from the cluster
                FbxAMatrix linkBindMat;    // transform of the link (bone) at bind pose (global)
                FbxAMatrix meshBindMat;    // transform of the mesh at bind pose (global)

                // cluster API fills these
                cluster->GetTransformLinkMatrix(linkBindMat); // link in bind pose
                cluster->GetTransformMatrix(meshBindMat);     // mesh in bind pose

                // Convert link into mesh-local bind transform:
                // localBind = linkBind * inverse(meshBind)
                FbxAMatrix localBind = linkBindMat * meshBindMat.Inverse();

                boneBindMatrices[boneIndex] = localBind;
            }
            else
            {
                boneIndex = it->second;

                // If not assigned previously, attempt to populate bind matrix similarly
                FbxAMatrix linkBindMat, meshBindMat;
                cluster->GetTransformLinkMatrix(linkBindMat);
                cluster->GetTransformMatrix(meshBindMat);
                FbxAMatrix localBind = linkBindMat * meshBindMat.Inverse();

                // naive check: only overwrite if currently identity translation
                FbxVector4 curT = boneBindMatrices[boneIndex].GetT();
                if (curT[0] == 0.0 && curT[1] == 0.0 && curT[2] == 0.0)
                    boneBindMatrices[boneIndex] = localBind;
            }

            int indexCount = cluster->GetControlPointIndicesCount();
            int* indices = cluster->GetControlPointIndices();
            double* weights = cluster->GetControlPointWeights();

            for (int k = 0; k < indexCount; ++k)
            {
                int cpIndex = indices[k];
                float w = static_cast<float>(weights[k]);
                if (w <= 0.0f) 
                    continue;
                if (cpIndex < 0 || cpIndex >= controlPointCount) 
                    continue;
                cpInfluences[static_cast<size_t>(cpIndex)].emplace_back(boneIndex, w);
            }
        }
    }

    // Pack up to 4 strongest influences per control point (unchanged existing behavior)
    ctrlBones.resize(static_cast<size_t>(controlPointCount));
    ctrlWeights.resize(static_cast<size_t>(controlPointCount));

    bool anySkin = false;
    for (int i = 0; i < controlPointCount; ++i)
    {
        auto& inf = cpInfluences[static_cast<size_t>(i)];
        std::array<uint8_t, 4> b = { 0,0,0,0 };
        std::array<uint8_t, 4> w = { 0,0,0,0 };

        if (!inf.empty())
        {
            std::sort(inf.begin(), inf.end(), [](const std::pair<uint8_t, float>& a, const std::pair<uint8_t, float>& b) {
                return a.second > b.second;
                });

            float total = 0.0f;
            size_t take = std::min<size_t>(4, inf.size());
            for (size_t j = 0; j < take; ++j)
                total += inf[j].second;

            if (total > 0.0f)
            {
                int acc = 0;
                for (size_t j = 0; j < take; ++j)
                {
                    b[j] = inf[j].first;
                    float nf = inf[j].second / total;
                    int byteVal = static_cast<int>(std::round(nf * 255.0f));
                    if (j == take - 1)
                    {
                        byteVal = 255 - acc;
                        if (byteVal < 0)
                            byteVal = 0;
                    }
                    w[j] = static_cast<uint8_t>(byteVal);
                    acc += byteVal;
                }
                anySkin = true;
            }
        }

        ctrlBones[static_cast<size_t>(i)] = b;
        ctrlWeights[static_cast<size_t>(i)] = w;
    }

    // Build outBones entries (name, parentIndex, bindPose) using boneBindMatrices
    outBones.clear();
    outBones.resize(nextBoneIndex);
    for (uint32_t bi = 0; bi < nextBoneIndex; ++bi)
    {
        ItpMesh::Bone bone;
        FbxNode* node = nullptr;
        if (bi < boneNodes.size())
            node = boneNodes[bi];

        if (node)
        {
            bone.name = node->GetName();

            // find parent in the same bone map
            FbxNode* parent = node->GetParent();
            int parentIndex = -1;
            while (parent)
            {
                std::string parentName = parent->GetName();
                auto pit = boneNameToIndex.find(parentName);
                if (pit != boneNameToIndex.end())
                {
                    parentIndex = static_cast<int>(pit->second);
                    break;
                }
                parent = parent->GetParent();
            }
            bone.parentIndex = parentIndex;

            // boneBindMatrices[bi] currently holds the bone global bind transform expressed in mesh space.
            // To get the bone's local transform (relative to its parent) compute:
            // local = inverse(parentGlobal) * boneGlobal
            FbxAMatrix boneGlobal = boneBindMatrices[bi];
            FbxAMatrix localBind;
            if (bone.parentIndex >= 0 && static_cast<size_t>(bone.parentIndex) < boneBindMatrices.size())
            {
                FbxAMatrix parentGlobal = boneBindMatrices[static_cast<size_t>(bone.parentIndex)];
                localBind = parentGlobal.Inverse() * boneGlobal;
            }
            else
            {
                // root bone: local == global (already in mesh-local)
                localBind = boneGlobal;
            }

            FbxVector4 t = localBind.GetT();
            FbxVector4 r = localBind.GetR(); // Euler angles in degrees (X, Y, Z)

            bone.bindPose.trans = Vector3(static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2]));

            auto ToRadians = [](double deg) { return static_cast<float>(deg * (3.14159265358979323846 / 180.0)); };
            float pitch = ToRadians(r[0]); // X
            float yaw = ToRadians(r[1]);   // Y
            float roll = ToRadians(r[2]);  // Z

            float cy = cosf(yaw * 0.5f);
            float sy = sinf(yaw * 0.5f);
            float cp = cosf(pitch * 0.5f);
            float sp = sinf(pitch * 0.5f);
            float cr = cosf(roll * 0.5f);
            float sr = sinf(roll * 0.5f);

            bone.bindPose.rot.x = sr * cp * cy - cr * sp * sy;
            bone.bindPose.rot.y = cr * sp * cy + sr * cp * sy;
            bone.bindPose.rot.z = cr * cp * sy - sr * sp * cy;
            bone.bindPose.rot.w = cr * cp * cy + sr * sp * sy;
        }
        else
        {
            bone.name = "bone_" + std::to_string(bi);
            bone.parentIndex = -1;
            bone.bindPose.trans = Vector3(0.0f, 0.0f, 0.0f);
            bone.bindPose.rot = Quaternion::Identity;
        }

        outBones[bi] = bone;
    }

    return anySkin;
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

    // Read skinning data
    std::vector<std::array<uint8_t, 4>> ctrlBones;
    std::vector<std::array<uint8_t, 4>> ctrlWeights;
    if (s_doSkinning)
        out->format.hasSkin = ReadSkin(mesh, ctrlBones, ctrlWeights, out->bones);

    int polygonCount = mesh->GetPolygonCount();
    out->indices.resize(polygonCount);
    std::unordered_map<VertexData, size_t> vertexMap;
    for (int p = 0; p < polygonCount; ++p)
    {
        int polySize = mesh->GetPolygonSize(p);
        for (int v = 0; v < polySize; ++v)
        {
            int ctrlPointIndex = mesh->GetPolygonVertex(p, v);

            VertexData vert;
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

            // copy skin data for this control point (if present)
            if (out->format.hasSkin && ctrlBones.size() > static_cast<size_t>(ctrlPointIndex))
            {
                auto cb = ctrlBones[static_cast<size_t>(ctrlPointIndex)];
                auto cw = ctrlWeights[static_cast<size_t>(ctrlPointIndex)];
                vert.bones[0] = cb[0];
                vert.bones[1] = cb[1];
                vert.bones[2] = cb[2];
                vert.bones[3] = cb[3];
                vert.weights[0] = cw[0]; 
                vert.weights[1] = cw[1];
                vert.weights[2] = cw[2]; 
                vert.weights[3] = cw[3];
            }
            else
            {
                vert.bones[0] = vert.bones[1] = vert.bones[2] = vert.bones[3] = 0;
                vert.weights[0] = vert.weights[1] = vert.weights[2] = vert.weights[3] = 0;
            }

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

    if (s_doSkinning && itpMesh.format.hasSkin)
    {
        std::cout << "  Skinning:\n";
        // Open output file
        std::string outputPath = itpMesh.name + ".itpskel";
        std::ofstream ofs(outputPath, std::ofstream::out | std::ofstream::trunc);
        if (!ofs.is_open())
        {
            std::cerr << "Failed to open output file: " << outputPath << "\n";
        }
        else
        {
            ofs << std::showpoint;
            itpMesh.WriteSkelToJson(ofs);
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

void ReadOptions(int argc, char** argv)
{
    s_doBlendShapes = false;
    s_doSkinning = false;
    // For simplicity, only check for flags in arguments
    for (int i = 2; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "-b")
        {
            s_doBlendShapes = true;
        }
        else if (arg == "-s")
        {
            s_doSkinning = true;
        }
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
    ReadOptions(argc, argv);

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