#pragma once
#include "VertexFormat.h"
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

class ItpMesh
{
public:

    struct VertexFormat
    {
        bool hasNormal = false;
        bool hasTan = false;
        bool hasUV = false;

        void WriteToJson(std::ofstream& ofs, int indent = 1) const;
    };

    struct BlendShape
    {
        std::string name;
        VertexFormat format;
        std::vector<VertexPosNormTan> deltas;

        void WriteDeltaToJson(const VertexPosNormTan& vert, std::ofstream& ofs) const;
        void WriteDeltasToJson(std::ofstream& ofs) const;
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
        std::vector<VertexPosNormTanUV> verts;
        std::vector<Triangle> indices;          // assuming triangles

        std::vector<BlendShape> blendShapes;    // blendshape targets

        void WriteToJson(std::ofstream& ofs) const;
        void WriteVertToJson(const VertexPosNormTanUV& vert, std::ofstream& ofs) const;
        void WriteVertsToJson(std::ofstream& ofs) const;
        void WriteIndicesToJson(std::ofstream& ofs) const;
    };
};

