#pragma once

#include "data/map/map_structs.hpp"
#include "planning/behavior/behavior_structs.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

struct LaneChangeConfig {
    double obstacleTriggerMinimumDistance{20.0};
    double obstacleTriggerMaximumDistance{60.0};
    double stoppedLeadWaitTime{2.0};
    double stoppedLeadMinimumDistance{5.0};
    double stoppedEgoMaximumSpeed{2.0 / 3.6};
    double transitionLength{18.0};
    double obstacleClearance{10.0};
    double prepareTime{0.5};
    double settleTime{0.5};
    double minimumTargetAheadGap{15.0};
    double minimumTargetBehindGap{12.0};
    double minimumTargetTTC{3.0};
    double maximumLaneChangeSpeed{6.0};
    double returnWaitingSpeed{2.0};
    double maximumObstacleLaneIntrusion{1.25};
    double lateralOffsetStep{0.2};
    double maximumLateralOffset{0.8};
    double minimumLaneEdgeClearance{0.15};
};

enum class LaneChangeRouteMode {
    ReturnToSource,
    KeepTargetLane,
};

struct LaneChangeData {
    LaneChangePhase phase{LaneChangePhase::None};
    int obstacleId{-1};
    int sourceRoadId{-1};
    int sourceLaneId{};
    int sourceSectionIndex{-1};
    int targetLaneId{};
    LaneChangeRouteMode routeMode{LaneChangeRouteMode::ReturnToSource};
    LaneKey reconnectLane;
    bool reconnectLaneValid{};
    std::size_t reconnectGlobalIndex{};
    std::uint8_t turnSignal{};
    std::uint8_t outwardTurnSignal{};
    std::uint8_t returnTurnSignal{};
    double phaseElapsed{};
    double startRouteS{};
    double returnStartRouteS{};
    double rejoinRouteS{};
    bool waitingForReturn{};
    std::size_t nearestIndex{};
    std::vector<Waypoint> path;
};
