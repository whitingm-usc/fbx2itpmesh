#include "ItpMesh.h"

static std::string Indent(int indent)
{
    std::string indentStr = "";
    for (int i = 0; i < indent; ++i)
    {
        indentStr += "\t";
    }
    return indentStr;
}

void ItpMesh::VertexFormat::WriteToJson(std::ofstream& ofs, int indent) const
{
    std::string in = Indent(indent);
    ofs << in << "\"vertexformat\": [\n";
    ofs << in << "\t{\n";
    ofs << in << "\t\t\"name\": \"position\",\n";
    ofs << in << "\t\t\"type\" : \"float\",\n";
    ofs << in << "\t\t\"count\" : 3\n";
    ofs << in << "\t}";
    if (hasNormal)
    {
        ofs << ",\n";
        ofs << in << "\t{\n";
        ofs << in << "\t\t\"name\": \"normal\",\n";
        ofs << in << "\t\t\"type\": \"float\",\n";
        ofs << in << "\t\t\"count\": 3\n";
        ofs << in << "\t}";
    }
    if (hasTan)
    {
        ofs << ",\n";
        ofs << in << "\t{\n";
        ofs << in << "\t\t\"name\": \"tangent\",\n";
        ofs << in << "\t\t\"type\": \"float\",\n";
        ofs << in << "\t\t\"count\": 3\n";
        ofs << in << "\t}";
    }
    if (hasSkin)
    {
        ofs << ",\n";
        ofs << in << "\t{\n";
        ofs << in << "\t\t\"name\": \"bones\",\n";
        ofs << in << "\t\t\"type\": \"byte\",\n";
        ofs << in << "\t\t\"count\": 4\n";
        ofs << in << "\t},\n";
        ofs << in << "\t{\n";
        ofs << in << "\t\t\"name\": \"weights\",\n";
        ofs << in << "\t\t\"type\": \"byte\",\n";
        ofs << in << "\t\t\"count\": 4\n";
        ofs << in << "\t}";
    }
    if (hasUV)
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

void ItpMesh::BlendShape::WriteDeltaToJson(const VertexData& vert, std::ofstream& ofs) const
{
    ofs << "\t\t[ ";
    ofs << vert.pos.x << ", " << vert.pos.y << ", " << vert.pos.z;
    if (format.hasNormal)
    {
        ofs << ", " << vert.norm.x << ", " << vert.norm.y << ", " << vert.norm.z;
    }
    if (format.hasTan)
    {
        ofs << ", " << vert.tan.x << ", " << vert.tan.y << ", " << vert.tan.z;
    }

    ofs << " ]";
}

void ItpMesh::BlendShape::WriteDeltasToJson(std::ofstream& ofs) const
{
    ofs << "\t\"deltas\": [\n";
    if (!deltas.empty())
    {
        WriteDeltaToJson(deltas[0], ofs);
        for (size_t i = 1; i < deltas.size(); ++i)
        {
            ofs << ",\n";
            WriteDeltaToJson(deltas[i], ofs);
        }
    }
    ofs << "\n\t]";
}

void ItpMesh::BlendShape::WriteToJson(std::ofstream& ofs) const
{
    ofs << "{\n";
    ofs << "\t\"metadata\": {\n";
    ofs << "\t\t\"type\": \"itpblend\",\n";
    ofs << "\t\t\"version\" : 1\n";
    ofs << "\t},\n";

    ofs << "\t\"name\": \"" << name << "\",\n";
    format.WriteToJson(ofs, 1);
    // deltas
    WriteDeltasToJson(ofs);

    ofs << "\n}\n";
}

void ItpMesh::Bone::WriteToJson(std::ofstream& ofs) const
{
    ofs << "\t\t{\n";
    ofs << "\t\t\t\"name\": \"" << name << "\",\n";
    ofs << "\t\t\t\"parentIndex\": " << parentIndex << ",\n";
    ofs << "\t\t\t\"bindPose\": {\n";
    ofs << "\t\t\t\t\"rot\": [ " << bindPose.rot.x << ", "
        << bindPose.rot.y << ", " << bindPose.rot.z << ", "
        << bindPose.rot.w << " ],\n";
    ofs << "\t\t\t\t\"trans\": [ " << bindPose.trans.x << ", "
        << bindPose.trans.y << ", " << bindPose.trans.z << " ]\n";
    ofs << "\t\t\t}\n";
    ofs << "\t\t}";
}

void ItpMesh::Mesh::Triangle::WriteToJson(std::ofstream& ofs) const
{
    ofs << "\t\t[ " << index[0] << ", " << index[1] << ", "
        << index[2] << " ]";
}

void ItpMesh::Mesh::WriteToJson(std::ofstream& ofs) const
{
    ofs << "{\n";
    ofs << "\t\"metadata\": {\n";
    ofs << "\t\t\"type\": \"itpmesh\",\n";
    ofs << "\t\t\"version\" : 3\n";
    ofs << "\t},\n";
    ofs << "\t\"material\" : \"Assets/Materials/" << name << ".itpmat\",\n";
    format.WriteToJson(ofs, 1);
    WriteVertsToJson(ofs);
    WriteIndicesToJson(ofs);

    ofs << "\n}\n";
}

void ItpMesh::Mesh::WriteVertToJson(const VertexData& vert, std::ofstream& ofs) const
{
    ofs << "\t\t[ ";
    ofs << vert.pos.x << ", " << vert.pos.y << ", " << vert.pos.z;
    if (format.hasNormal)
    {
        ofs << ", " << vert.norm.x << ", " << vert.norm.y << ", " << vert.norm.z;
    }
    if (format.hasTan)
    {
        ofs << ", " << vert.tan.x << ", " << vert.tan.y << ", " << vert.tan.z;
    }
    if (format.hasSkin)
    {
        ofs << ", " << static_cast<uint32_t>(vert.bones[0]) << ", "
            << static_cast<uint32_t>(vert.bones[1]) << ", "
            << static_cast<uint32_t>(vert.bones[2]) << ", "
            << static_cast<uint32_t>(vert.bones[3]) << ", "
            << static_cast<uint32_t>(vert.weights[0]) << ", "
            << static_cast<uint32_t>(vert.weights[1]) << ", "
            << static_cast<uint32_t>(vert.weights[2]) << ", "
            << static_cast<uint32_t>(vert.weights[3]);
    }
    if (format.hasUV)
    {
        ofs << ", " << vert.uv.x << ", " << vert.uv.y;
    }

    ofs << " ]";
}

void ItpMesh::Mesh::WriteVertsToJson(std::ofstream& ofs) const
{
    ofs << "\t\"vertices\": [\n";
    WriteVertToJson(verts[0], ofs);
    for (size_t i = 1; i < verts.size(); ++i)
    {
        ofs << ",\n";
        WriteVertToJson(verts[i], ofs);
    }
    ofs << "\n\t],\n";
}

void ItpMesh::Mesh::WriteIndicesToJson(std::ofstream& ofs) const
{
    ofs << "\t\"indices\": [\n";
    indices[0].WriteToJson(ofs);
    for (size_t i = 1; i < indices.size(); ++i)
    {
        ofs << ",\n";
        indices[i].WriteToJson(ofs);
    }
    ofs << "\n\t]";
}

void ItpMesh::Mesh::WriteSkelToJson(std::ofstream& ofs) const
{
    ofs << "{\n";
    ofs << "\t\"metadata\": {\n";
    ofs << "\t\t\"type\": \"itpskel\",\n";
    ofs << "\t\t\"version\": 1\n";
    ofs << "\t},\n";
    ofs << "\t\"bonecount\": " << bones.size() << ",\n";
    ofs << "\t\"bones\": [\n";
    if (!bones.empty())
    {
        bones[0].WriteToJson(ofs);
        for (size_t i = 1; i < bones.size(); ++i)
        {
            ofs << ",\n";
            bones[i].WriteToJson(ofs);
        }
    }
    ofs << "\n\t]\n";
    ofs << "}\n";
}
