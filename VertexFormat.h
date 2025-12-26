#pragma once
#include "EngineMath.h"
#include <cstddef>
#include <functional> // for std::hash

// This file contains C++ structures describing the various vertex formats

// For now, I have provided the first one
// This matches the HLSL structure from Simple.hlsl:
//struct VIn
//{
//    float3 position : POSITION0;
//    float4 color : COLOR0;
//};
struct VertexPosColor
{
    Vector3 pos;
    Color4 color;
};

struct VertexPosUV
{
    Vector3 pos;
    Vector2 uv;
};

struct VertexPosColorUV
{
    Vector3 pos;
    Color4 color;
    Vector2 uv;
};

struct VertexPosNormColorUV
{
    Vector3 pos;
    Vector3 norm;
    Color4 color;
    Vector2 uv;
};

struct VertexPosNormUV
{
    Vector3 pos;
    Vector3 norm;
    Vector2 uv;
};

struct VertexPosNormTanUV
{
    Vector3 pos;
    Vector3 norm;
    Vector3 tan;
    Vector2 uv;
};

struct VertexPosNormTan
{
    Vector3 pos;
    Vector3 norm;
    Vector3 tan;
};

// equality operator for VertexPosNormTanUV (bitwise float equality)
inline bool operator==(const VertexPosNormTanUV& a, const VertexPosNormTanUV& b) noexcept
{
    return a.pos.x == b.pos.x && a.pos.y == b.pos.y && a.pos.z == b.pos.z
        && a.norm.x == b.norm.x && a.norm.y == b.norm.y && a.norm.z == b.norm.z
        && a.tan.x == b.tan.x && a.tan.y == b.tan.y && a.tan.z == b.tan.z
        && a.uv.x == b.uv.x && a.uv.y == b.uv.y;
}

namespace std
{
    template<>
    struct hash<VertexPosNormTanUV>
    {
        size_t operator()(VertexPosNormTanUV const& v) const noexcept
        {
            // combine hashes of float components using boost-like hash_combine
            auto hf = std::hash<float>{};
            size_t h = hf(v.pos.x);
            h ^= hf(v.pos.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hf(v.pos.z) + 0x9e3779b9 + (h << 6) + (h >> 2);

            h ^= hf(v.norm.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hf(v.norm.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hf(v.norm.z) + 0x9e3779b9 + (h << 6) + (h >> 2);

            h ^= hf(v.tan.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hf(v.tan.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hf(v.tan.z) + 0x9e3779b9 + (h << 6) + (h >> 2);

            h ^= hf(v.uv.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hf(v.uv.y) + 0x9e3779b9 + (h << 6) + (h >> 2);

            return h;
        }
    };
}