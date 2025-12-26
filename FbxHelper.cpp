#include "FbxHelper.h"

// Helper to fetch normal for a polygon-vertex
/*static*/ bool FbxHelper::GetNormalAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector4& outNormal)
{
    if (!mesh)
        return false;
    fbxsdk::FbxGeometryElementNormal* elemNormal = mesh->GetElementNormal(0);
    if (!elemNormal)
        return false;

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
/*static*/ bool FbxHelper::GetTangentAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector4& outTangent)
{
    if (!mesh)
        return false;
    fbxsdk::FbxGeometryElementTangent* elemTangent = mesh->GetElementTangent(0);
    if (!elemTangent)
        return false;

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
/*static*/ bool FbxHelper::GetUVAt(FbxMesh* mesh, int polyIndex, int vertIndex, FbxVector2& outUV, const char* uvName)
{
    if (!mesh)
        return false;
    FbxStringList uvSetNameList;
    mesh->GetUVSetNames(uvSetNameList);
    if (uvSetNameList.GetCount() == 0)
        return false;

    const char* name = uvName;
    if (!name)
        name = uvSetNameList.GetStringAt(0);

    bool unmapped;
    if (mesh->GetPolygonVertexUV(polyIndex, vertIndex, name, outUV, unmapped))
    {
        if (unmapped)
            return false;
        return true;
    }
    return false;
}


