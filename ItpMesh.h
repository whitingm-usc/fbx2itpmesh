#pragma once
#include "VertexFormat.h"
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

class ItpMesh
{
public:

    struct VertexFormat
    {
        bool hasNormal = false;
        bool hasTan = false;
        bool hasUV = false;
        bool hasSkin = false;

        void WriteToJson(std::ofstream& ofs, int indent = 1) const;
    };

    struct BlendShape
    {
        std::string name;
        VertexFormat format;
        std::vector<VertexData> deltas;

        void WriteDeltaToJson(const VertexData& vert, std::ofstream& ofs) const;
        void WriteDeltasToJson(std::ofstream& ofs) const;
        void WriteToJson(std::ofstream& ofs) const;
    };

    struct Bone
    {
        struct BindPose
        {
            Quaternion rot;
            Vector3 trans;
        };
        std::string name;
        int32_t parentIndex = -1;
        BindPose bindPose;

        void WriteToJson(std::ofstream& ofs) const;
    };

    struct Mesh
    {
        struct Triangle {
            uint32_t index[3];

            void WriteToJson(std::ofstream& ofs) const;
        };
        std::string name;
        VertexFormat format;
        std::vector<VertexData> verts;
        std::vector<Triangle> indices;          // assuming triangles

        std::vector<Bone> bones;                // skeleton bones
        std::vector<BlendShape> blendShapes;    // blendshape targets

        std::unordered_map<uint32_t, std::vector<uint32_t>> vertexMap; // original index to new indices

        void WriteToJson(std::ofstream& ofs) const;
        void WriteVertToJson(const VertexData& vert, std::ofstream& ofs) const;
        void WriteVertsToJson(std::ofstream& ofs) const;
        void WriteIndicesToJson(std::ofstream& ofs) const;
        void WriteSkelToJson(std::ofstream& ofs) const;
    };
};

