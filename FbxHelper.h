#pragma once
#include "EngineMath.h"
#include <fbxsdk.h>
#include <string>
#include <vector>


class FbxHelper
{
public:
    // Helper to fetch normal for a polygon-vertex
    static bool GetNormalAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector4& outNormal);
    static bool GetTangentAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector4& outTangent);
    static bool GetUVAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector2& outUV, const char* uvName = nullptr);
    static void CollectSkeletonNodes(FbxNode* node, std::vector<FbxNode*>& outNodes);
    static Vector3 TranformVector3(const FbxVector4& vec);
    static Quaternion TranformQuaternion(const FbxQuaternion& quat);
    static std::string GetMeshName(FbxMesh* mesh, int index=0);
};

