#pragma once

#include "data/gt/gt_structs.hpp"
#include "data/map/map_matcher.hpp"
#include "data/map/map_structs.hpp"
#include "data/vehicle/vehicle_structs.hpp"
#include "planning/global/mission_corridor.hpp"

#include <cstdint>
#include <map>

struct WorldProcessConfig {
    double speedFilterAlpha{0.25};
    double maxValidSpeed{80.0};
    // Position-derived speed must not jump faster than the vehicle can
    // physically accelerate/decelerate.  This rejects transient RDB/GT time
    // mismatches without hiding a real large-distance respawn.
    double maxSpeedIncreaseRate{6.0};
    double maxSpeedDecreaseRate{12.0};
    double respawnJumpDistance{20.0};
    double pathLateralMargin{0.5};
};

struct WorldProcessData {
    struct ObjectMotionSample {
        double x{};
        double y{};
        double velocityX{};
        double velocityY{};
        bool velocityValid{};
    };
    WorldProcessConfig config;
    WorldState state;
    double previousX{};
    double previousY{};
    double previousReceiveTime{};
    double filteredSpeed{};
    bool hasPreviousFrame{};
    MapMatchState egoMapMatch;
    std::map<std::uint32_t, ObjectMotionSample> objectMotion;
};

void runWorldProcess(WorldProcessData& data,
                     const GtFrame& gt,
                     double receiveTime,
                     double frameDt,
                     const PathGeneratorData& map,
                     const std::vector<Waypoint>& globalPath,
                     const VehicleGeometry& vehicle,
                     const MissionCorridorData* mission = nullptr);
