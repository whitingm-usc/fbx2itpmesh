#pragma once
#include "EngineMath.h"

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