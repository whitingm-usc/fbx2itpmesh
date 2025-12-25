#pragma once
#include <fbxsdk.h>

class FbxHelper
{
public:
    // Helper to fetch normal for a polygon-vertex
    static bool GetNormalAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector4& outNormal);
    static bool GetTangentAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector4& outTangent);
    static bool GetUVAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector2& outUV, const char* uvName = nullptr);
};

