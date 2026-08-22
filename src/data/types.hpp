#pragma once

#include <array>
#include <cstdint>

// ============================================================
// VTD -> Participant : Ego GT
// ============================================================
struct EgoState
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float heading = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
};


// ============================================================
// VTD -> Participant : Object GT
// Object 1개 = 36 Byte
// ============================================================
struct ObjectState
{
    std::uint32_t id = 0;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float heading = 0.0f;
    float speed = 0.0f;

    float length = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
};


// ============================================================
// VTD -> Participant : Traffic Light GT
// ============================================================
struct TrafficLightState
{
    std::int32_t id = 0;
    std::uint8_t state = 0;
};


// ============================================================
// VTD GT Frame
//
// Ego            : 24 Byte
// Object x 30    : 1080 Byte
// Traffic Light  : 5 Byte
//
// Total          : 1109 Byte
// ============================================================
struct GtFrame
{
    EgoState ego;

    std::array<ObjectState, 30> objects;

    TrafficLightState trafficLight;
};