#pragma once

#include <array>
#include <cstdint>

struct EgoState {
    float x{};
    float y{};
    float z{};
    float heading{};
    float pitch{};
    float roll{};
};

struct ObjectState {
    std::uint32_t id{};
    float x{};
    float y{};
    float z{};
    float heading{};
    float speed{};
    float length{};
    float width{};
    float height{};
};

struct TrafficLightState {
    std::int32_t id{};
    std::uint8_t state{};
};

struct GtFrame {
    EgoState ego;
    std::array<ObjectState, 30> objects;
    TrafficLightState trafficLight;
};
