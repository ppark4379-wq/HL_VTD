#pragma once

#include "data/gt/gt_structs.hpp"
#include "data/map/map_matcher.hpp"
#include "data/map/map_structs.hpp"
#include "data/vehicle/vehicle_structs.hpp"

struct WorldProcessConfig {
    double speedFilterAlpha{0.25};
    double maxValidSpeed{80.0};
    double respawnJumpDistance{20.0};
    double pathLateralMargin{0.5};
};

struct WorldProcessData {
    WorldProcessConfig config;
    WorldState state;
    double previousX{};
    double previousY{};
    double previousReceiveTime{};
    double filteredSpeed{};
    bool hasPreviousFrame{};
    MapMatchState egoMapMatch;
};

void runWorldProcess(WorldProcessData& data,
                     const GtFrame& gt,
                     double receiveTime,
                     double frameDt,
                     const PathGeneratorData& map,
                     const std::vector<Waypoint>& globalPath,
                     const VehicleGeometry& vehicle);
