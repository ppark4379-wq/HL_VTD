#pragma once

#include "utils/geometry_structs.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

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

// TCP 패킷 하나를 파싱해 저장하는 전체 GT 프레임.
struct GtFrame {
    EgoState ego;
    std::array<ObjectState, 30> objects;
    TrafficLightState trafficLight;
};


// 판제에서 사용할 Object 상태
struct WorldObjectState {
    ObjectState raw;

    // VTD's scalar speed is a magnitude in some scenarios even when an
    // actor is driven opposite to its reported body heading.  Preserve a
    // position-derived world velocity so collision prediction follows the
    // motion that was actually observed, not an assumed sign.
    double velocityX{};
    double velocityY{};
    bool velocityValid{};

    Obb2D obb;

    bool laneValid{};
    int roadId{};
    int laneId{};
    int sectionIndex{};
    double roadS{};

    bool pathValid{};
    double pathS{};
    double pathD{};

    bool isAhead{};
    bool isOnEgoPath{};
    bool isInsideCrosswalk{};
    bool isCollisionRisk{};
    double relativeS{};
    double relativeD{};
    double ttc{std::numeric_limits<double>::infinity()};
};

inline double objectVelocityX(const WorldObjectState& object) {
    return object.velocityValid
        ? object.velocityX
        : static_cast<double>(object.raw.speed) *
              std::cos(static_cast<double>(object.raw.heading));
}

inline double objectVelocityY(const WorldObjectState& object) {
    return object.velocityValid
        ? object.velocityY
        : static_cast<double>(object.raw.speed) *
              std::sin(static_cast<double>(object.raw.heading));
}

inline double objectMotionSpeed(const WorldObjectState& object) {
    return std::hypot(objectVelocityX(object), objectVelocityY(object));
}

inline double objectMotionHeading(const WorldObjectState& object) {
    const double speed = objectMotionSpeed(object);
    return speed >= 0.3
        ? std::atan2(objectVelocityY(object), objectVelocityX(object))
        : static_cast<double>(object.raw.heading);
}

struct WorldState {
    EgoState ego;
    Obb2D egoObb;
    double receiveTime{};
    double egoSpeed{};
    bool inputValid{};
    bool respawnDetected{};
    bool egoLaneValid{};
    int egoRoadId{};
    int egoLaneId{};
    int egoSectionIndex{};
    double egoRoadS{};
    double egoLaneWidth{};
    double egoLaneCenterDistance{};
    bool egoPathValid{};
    double egoPathS{};
    double egoPathD{};
    double frontBumperOffset{};
    double frontBumperPathS{};
    std::vector<WorldObjectState> objects;
    TrafficLightState trafficLight;
};
