
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

static bool s_doBlendShapes = false;
static bool s_doSkinning = false;
static bool s_doAnimation = false;
static float s_sampleRate = 30.0f; // samples per second (default)


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
        if (!blendShape) 
            continue;

        int channelCount = blendShape->GetBlendShapeChannelCount();
        for (int c = 0; c < channelCount; ++c)
        {
            FbxBlendShapeChannel* channel = blendShape->GetBlendShapeChannel(c);
            if (!channel) 
                continue;

            int targetCount = channel->GetTargetShapeCount();
            if (targetCount == 0) 
                continue;

            for (int t = 0; t < targetCount; ++t)
            {
                FbxShape* shape = channel->GetTargetShape(t);
                if (!shape) 
                    continue;

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
                bs.name = FbxHelper::GetMeshName(mesh) + "_" + std::string(channel->GetName());
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
                    vert.pos = FbxHelper::TranformVector3(shapeControlPoints[i]);
                    vert.pos -= baseVert.pos;

                    if (bs.format.hasNormal)
                    {
                        int idx = (elemNormal->GetReferenceMode() == FbxGeometryElement::eDirect) ?
                            i : elemNormal->GetIndexArray().GetAt(i);
                        FbxVector4 n = elemNormal->GetDirectArray().GetAt(idx);
                        vert.norm = FbxHelper::TranformVector3(n);
                        vert.norm -= baseVert.norm;
                    }
                    if (bs.format.hasTan)
                    {
                        int idx = (elemTangent->GetReferenceMode() == FbxGeometryElement::eDirect) ?
                            i : elemTangent->GetIndexArray().GetAt(i);
                        FbxVector4 t = elemTangent->GetDirectArray().GetAt(idx);
                        vert.tan = FbxHelper::TranformVector3(t);
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

static void GetAnimBones(FbxMesh* mesh, 
    std::vector<FbxNode*>& boneNodes,
    std::unordered_map<std::string, uint8_t>& boneNameToIndex,
    std::vector<std::vector<std::pair<uint8_t, float>>>& cpInfluences
    )
{
    // Collect skeleton nodes in scene
    // find topmost root for this mesh and collect skeleton nodes under it
    FbxNode* root = mesh->GetNode();
    while (root->GetParent())
        root = root->GetParent();
    FbxHelper::CollectSkeletonNodes(root, boneNodes);
    // make a map of bone name to bone index
    for (size_t i = 0; i < boneNodes.size(); ++i)
    {
        FbxNode* skelNode = boneNodes[i];
        std::string skelName = skelNode->GetName();
        boneNameToIndex[skelName] = static_cast<uint8_t>(i);
    }

    // go thru all the deformers looking for skin deformers
    int skinDeformerCount = mesh->GetDeformerCount(FbxDeformer::eSkin);
    int controlPointCount = mesh->GetControlPointsCount();
    cpInfluences.resize(static_cast<size_t>(controlPointCount));
    for (int s = 0; s < skinDeformerCount; ++s)
    {
        FbxSkin* skin = static_cast<FbxSkin*>(mesh->GetDeformer(s, FbxDeformer::eSkin));
        if (!skin) 
            continue;

        int clusterCount = skin->GetClusterCount();
        for (int c = 0; c < clusterCount; ++c)
        {
            FbxCluster* cluster = skin->GetCluster(c);
            if (!cluster) 
                continue;

            FbxNode* linkNode = cluster->GetLink(); // bone node
            if (!linkNode) 
                continue;

            // this is a skinning bone
            std::string boneName = linkNode->GetName();
            uint8_t boneIndex = boneNameToIndex[boneName];

            // Get the link (bone) bind matrix and the mesh bind matrix from the cluster
            FbxAMatrix linkBindMat;    // transform of the link (bone) at bind pose (global)
            FbxAMatrix meshBindMat;    // transform of the mesh at bind pose (global)

            // cluster API fills these
            cluster->GetTransformLinkMatrix(linkBindMat); // link in bind pose
            cluster->GetTransformMatrix(meshBindMat);     // mesh in bind pose

            // get the bone indices and weights for this vertex
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
}

// Read skinning info and also populate the mesh bones (names + bind poses + parent indices).
// Returns true if any skinning data was found.
static bool ReadSkin(FbxMesh* mesh,
    std::vector<std::array<uint8_t, 4>>& ctrlBones,
    std::vector<std::array<uint8_t, 4>>& ctrlWeights,
    std::vector<ItpMesh::Bone>& outBones)
{
    int vertexCount = mesh->GetControlPointsCount();

    // Per-control-point list of (boneIndex, weight)
    std::vector<std::vector<std::pair<uint8_t, float>>> cpInfluences;
    // Map bone (link) name -> small integer index (uint8_t)
    std::unordered_map<std::string, uint8_t> boneNameToIndex;
    // Keep arrays for node pointers and bind matrices indexed by boneIndex
    std::vector<FbxNode*> boneNodes;
    GetAnimBones(mesh, boneNodes, boneNameToIndex, cpInfluences);    
    size_t numBones = boneNodes.size();

    // Pack up to 4 strongest influences per vertex
    ctrlBones.resize(static_cast<size_t>(vertexCount));
    ctrlWeights.resize(static_cast<size_t>(vertexCount));
    bool anySkin = false;
    for (int i = 0; i < vertexCount; ++i)
    {
        auto& inf = cpInfluences[static_cast<size_t>(i)];
        std::array<uint8_t, 4> b = { 0,0,0,0 };
        std::array<uint8_t, 4> w = { 0,0,0,0 };

        if (!inf.empty())
        {
            // sort influences by weight descending
            std::sort(inf.begin(), inf.end(), [](const std::pair<uint8_t, float>& a, const std::pair<uint8_t, float>& b) {
                return a.second > b.second;
                });

            // rescale weights to [0, 255] range
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
                anySkin = true; // found at least one skinned vertex
            }
        }

        ctrlBones[static_cast<size_t>(i)] = b;
        ctrlWeights[static_cast<size_t>(i)] = w;
    }

    // Build outBones entries (name, parentIndex, bindPose)
    outBones.clear();
    outBones.resize(numBones);
    std::vector<FbxAMatrix> boneBindMatrices;
    boneBindMatrices.resize(numBones);
    for (uint32_t bi = 0; bi < numBones; ++bi)
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
            if (parent)
            {
                std::string parentName = parent->GetName();
                auto pit = boneNameToIndex.find(parentName);
                if (pit != boneNameToIndex.end())
                {
                    parentIndex = static_cast<int>(pit->second);
                }
            }
            bone.parentIndex = parentIndex;

            // To get the bone's local transform (relative to its parent) compute:
            // local = inverse(parentGlobal) * boneGlobal
            boneBindMatrices[bi] = node->EvaluateGlobalTransform();
            FbxAMatrix localBind;
            if (bone.parentIndex >= 0 && static_cast<size_t>(bone.parentIndex) < boneBindMatrices.size())
            {
                FbxAMatrix parentGlobal = boneBindMatrices[static_cast<size_t>(bone.parentIndex)];
                localBind = parentGlobal.Inverse() * boneBindMatrices[bi];
            }
            else
            {
                // root bone: local == global (already in mesh-local)
                localBind = boneBindMatrices[bi];
            }

            FbxVector4 t = localBind.GetT();
            bone.bindPose.trans = FbxHelper::TranformVector3(t);
            FbxQuaternion q = localBind.GetQ();
            bone.bindPose.rot = FbxHelper::TranformQuaternion(q);
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
    out->name = FbxHelper::GetMeshName(mesh, index);

    fbxsdk::FbxGeometryElementNormal* elemN = mesh->GetElementNormal(0);
    out->format.hasNormal = (elemN != nullptr);
    fbxsdk::FbxGeometryElementUV* elemUV = mesh->GetElementUV(0);
    out->format.hasUV = (elemUV != nullptr);
    fbxsdk::FbxGeometryElementTangent* elemT = mesh->GetElementTangent(0);
    out->format.hasTan = (elemT != nullptr);

    // Read skinning data
    std::vector<std::array<uint8_t, 4>> ctrlBones;
    std::vector<std::array<uint8_t, 4>> ctrlWeights;
    if (s_doSkinning)
        out->format.hasSkin = ReadSkin(mesh, ctrlBones, ctrlWeights, out->bones);

    int polygonCount = mesh->GetPolygonCount();
    out->indices.resize(polygonCount);
    // create a map of unique vertices...
    // if ALL the vertex data is identical, then we can share the vertex
    std::unordered_map<VertexData, size_t> vertexMap;
    for (int p = 0; p < polygonCount; ++p)
    {
        int polySize = mesh->GetPolygonSize(p);
        for (int v = 0; v < polySize; ++v)
        {
            int ctrlPointIndex = mesh->GetPolygonVertex(p, v);

            VertexData vert;
            FbxVector4 pos = mesh->GetControlPointAt(ctrlPointIndex);
            FbxVector4 normal; 
            bool hasNormal = FbxHelper::GetNormalAt(mesh, p, v, normal);
            FbxVector2 uv;
            bool hasUV = FbxHelper::GetUVAt(mesh, p, v, uv, nullptr);
            FbxVector4 tangent;
            bool hasTangent = FbxHelper::GetTangentAt(mesh, p, v, tangent);

            vert.pos = FbxHelper::TranformVector3(pos);
            if (hasNormal)
                vert.norm = FbxHelper::TranformVector3(normal);
            else
                vert.norm = Vector3(0.0f, 0.0f, 0.0f);
            if (hasTangent)
                vert.tan = FbxHelper::TranformVector3(tangent);
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
            {   // found existing vertex... reuse it
                index = inMap->second;
            }
            else
            {   // this is a new vertex... add it to the map
                index = vertexMap.size();
                vertexMap[vert] = index;
                out->verts.emplace_back(vert);
                out->vertexMap[static_cast<uint32_t>(ctrlPointIndex)].push_back(static_cast<uint32_t>(index));
            }
            out->indices[p].index[v] = static_cast<uint32_t>(index);
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

/// Read a single animation stack and sample it at the provided sampleRate (samples per second).
/// Fills an ItpMesh::Anim with tracks for each skeleton node discovered in the scene.
/// Returns true on success.
static bool ReadAnimation(FbxScene* scene, FbxAnimStack* animStack, ItpMesh::Anim* outAnim, float sampleRate)
{
    if (!scene || !animStack || !outAnim || sampleRate <= 0.0f)
        return false;

    // Set this animation stack active for evaluation
    scene->SetCurrentAnimationStack(animStack);

    // Get time span for the stack
    FbxTimeSpan span = animStack->GetLocalTimeSpan();
    FbxTime startTime = span.GetStart();
    FbxTime endTime = span.GetStop();

    double startSec = startTime.GetSecondDouble();
    double endSec = endTime.GetSecondDouble();
    double length = endSec - startSec;
    if (length < 0.0)
        length = 0.0;

    // Collect skeleton nodes in scene
    std::vector<FbxNode*> skeletonNodes;
    FbxHelper::CollectSkeletonNodes(scene->GetRootNode(), skeletonNodes);
    if (skeletonNodes.empty())
        return false;

    // Prepare tracks
    outAnim->name = animStack->GetName();
    outAnim->isLoop = true; // preserving simple default; FBX doesn't directly mark loop here
    uint32_t frames = static_cast<uint32_t>(std::floor(length * sampleRate)) + 1u;
    outAnim->frames = frames;
    outAnim->length = static_cast<float>(length);
    outAnim->boneCount = static_cast<uint32_t>(skeletonNodes.size());
    outAnim->tracks.clear();
    outAnim->tracks.resize(skeletonNodes.size());

    for (size_t bi = 0; bi < skeletonNodes.size(); ++bi)
    {
        ItpMesh::Anim::Track track;
        track.boneIndex = static_cast<uint32_t>(bi);
        track.poses.reserve(frames);
        outAnim->tracks[bi] = std::move(track);
    }

    // sampling step in seconds
    double stepSec = 1.0 / static_cast<double>(sampleRate);

    for (uint32_t f = 0; f < frames; ++f)
    {
        double curSec = startSec + static_cast<double>(f) * stepSec;
        FbxTime t;
        t.SetSecondDouble(curSec);

        for (size_t bi = 0; bi < skeletonNodes.size(); ++bi)
        {
            FbxNode* node = skeletonNodes[bi];

            // Evaluate global transforms for node and its parent at time t,
            // then compute local = inverse(parentGlobal) * global.
            // This is more robust than using EvaluateLocalTransform() directly
            // when FBX files have pivots/inheritance or animations applied in global space.
            FbxAMatrix global = node->EvaluateGlobalTransform(t);

            FbxAMatrix local;
            FbxNode* parent = node->GetParent();
            if (parent)
            {
                FbxAMatrix parentGlobal = parent->EvaluateGlobalTransform(t);
                local = parentGlobal.Inverse() * global;
            }
            else
            {
                local = global;
            }

            FbxVector4 ft = local.GetT();
            FbxQuaternion fq = local.GetQ(); // quaternion rotation
            ItpMesh::Bone::Pose pose;
            pose.trans = FbxHelper::TranformVector3(ft);
            pose.rot = FbxHelper::TranformQuaternion(fq);

            outAnim->tracks[bi].poses.push_back(pose);
        }
    }

    return true;
}

void PrintOptions()
{
    std::cout << "Usage: FBX2ITP input.fbx [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -b          Read blend shapes\n";
    std::cout << "  -s          Read skinning data\n";
    std::cout << "  -a          Read animations (will not export mesh)\n";
    std::cout << "  -r <rate>   Sample rate for animations (samples per second, default 30.0)\n";
    std::cout << "  -h, -?      Show this help\n";
}

void ReadOptions(int argc, char** argv)
{
    s_doBlendShapes = false;
    s_doSkinning = false;
    s_doAnimation = false;
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
        else if (arg == "-a")
        {
            s_doAnimation = true;
        }
        else if (arg == "-r" && i + 1 < argc) // sample rate
        {
            try {
                s_sampleRate = static_cast<float>(std::stof(argv[i + 1]));
                if (s_sampleRate <= 0.0f)
                    s_sampleRate = 30.0f;
            }
            catch (...) {
                s_sampleRate = 30.0f;
            }
            ++i;
        }
        else if (arg == "-h" || arg == "-?")
        {
            PrintOptions();
            exit(0);
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Error: missing input file argument.\n";
        PrintOptions();
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

    if (s_doAnimation)
    {
        // Write animations: iterate all animation stacks and sample them at s_sampleRate
        int animStackCount = scene->GetSrcObjectCount<FbxAnimStack>();
        for (int a = 0; a < animStackCount; ++a)
        {
            FbxAnimStack* stack = scene->GetSrcObject<FbxAnimStack>(a);
            if (!stack)
                continue;

            ItpMesh::Anim anim;
            if (ReadAnimation(scene, stack, &anim, s_sampleRate))
            {
                std::cout << "Writing animation: " << anim.name << " (frames: " << anim.frames << ", length: " << anim.length << "s)\n";
                std::string outputPath = anim.name + ".itpanim2";
                std::ofstream ofs(outputPath, std::ofstream::out | std::ofstream::trunc);
                if (!ofs.is_open())
                {
                    std::cerr << "Failed to open output file: " << outputPath << "\n";
                }
                else
                {
                    ofs << std::showpoint;
                    anim.WriteToJson(ofs);
                    ofs.close();
                }
            }
            else
            {
                std::cout << "Skipping animation stack: " << (stack ? stack->GetName() : "<null>") << " (no skeleton or failed to sample)\n";
            }
        }
    }
    else
    {
        FbxGeometryConverter converter(sdkManager);
        converter.Triangulate(scene, true); // The 'true' parameter ensures original nodes are replaced.

        int index = 0;
        WriteAllMesh(scene->GetRootNode(), index);
    }

    // Cleanup
    sdkManager->Destroy();
    return 0;
}