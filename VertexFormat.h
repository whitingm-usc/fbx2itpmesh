#pragma once
#include "EngineMath.h"
#include <cstddef>
#include <functional> // for std::hash

// The vertex data structure holds data for every possible optional attribute.
struct VertexData
{
    Vector3 pos;
    Vector3 norm;
    Vector3 tan;
    uint8_t bones[4];
    uint8_t weights[4];
    Vector2 uv;
};

// equality operator for VertexData (bitwise float equality)
inline bool operator==(const VertexData& a, const VertexData& b) noexcept
{
    return a.pos.x == b.pos.x && a.pos.y == b.pos.y && a.pos.z == b.pos.z
        && a.norm.x == b.norm.x && a.norm.y == b.norm.y && a.norm.z == b.norm.z
        && a.tan.x == b.tan.x && a.tan.y == b.tan.y && a.tan.z == b.tan.z
        && a.bones[0] == b.bones[0] && a.bones[1] == b.bones[1]
        && a.bones[2] == b.bones[2] && a.bones[3] == b.bones[3]
        && a.weights[0] == b.weights[0] && a.weights[1] == b.weights[1]
        && a.weights[2] == b.weights[2] && a.weights[3] == b.weights[3]
        && a.uv.x == b.uv.x && a.uv.y == b.uv.y;
}

namespace std
{
    template<>
    struct hash<VertexData>
    {
        size_t operator()(VertexData const& v) const noexcept
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

            // include bone indices (bytes) in the hash
            auto hb = std::hash<uint8_t>{};
            h ^= hb(v.bones[0]) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hb(v.bones[1]) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hb(v.bones[2]) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hb(v.bones[3]) + 0x9e3779b9 + (h << 6) + (h >> 2);

            // include bone weights (bytes) in the hash
            h ^= hb(v.weights[0]) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hb(v.weights[1]) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hb(v.weights[2]) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hb(v.weights[3]) + 0x9e3779b9 + (h << 6) + (h >> 2);

            h ^= hf(v.uv.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= hf(v.uv.y) + 0x9e3779b9 + (h << 6) + (h >> 2);

            return h;
        }
    };
}