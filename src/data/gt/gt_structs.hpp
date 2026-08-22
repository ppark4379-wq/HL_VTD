#pragma once

#include <array>
#include <cstdint>

// VTD에서 수신한 Ego 차량의 GT 상태.
struct EgoState {
    float x{};
    float y{};
    float z{};
    float heading{};
    float pitch{};
    float roll{};
};

// VTD에서 수신한 주변 객체 하나의 GT 상태.
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

// VTD에서 수신한 신호등 하나의 GT 상태.
struct TrafficLightState {
    std::int32_t id{};
    std::uint8_t state{};
};

// TCP/UDP 패킷 하나를 파싱해 저장하는 전체 GT 프레임.
struct GtFrame {
    EgoState ego;
    std::array<ObjectState, 30> objects;
    TrafficLightState trafficLight;
};
