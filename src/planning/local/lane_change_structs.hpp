#pragma once

#include "data/map/map_structs.hpp"
#include "planning/behavior/behavior_structs.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

struct LaneChangeConfig {
    double obstacleTriggerMinimumDistance{5.0};
    double obstacleTriggerMaximumDistance{70.0};
    double stoppedDynamicConfirmationTime{0.75};
    // 현재 차선이 다음 road로 연결되지 않을 때만 도로 끝에서
    // 이 거리 이내에 들어오면 연결 차선으로 정렬한다.
    double roadConnectionPrepareDistance{45.0};
    double transitionLength{18.0};
    double obstacleClearance{10.0};
    double prepareTime{0.5};
    double settleTime{0.5};
    double minimumTargetAheadGap{15.0};
    double minimumTargetBehindGap{12.0};
    double minimumTargetTTC{3.0};
    double maximumLaneChangeSpeed{15.0 / 3.6};
    double returnWaitingSpeed{15.0 / 3.6};
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
    // true면 현재 회피 차선이 다음 전역경로 road와 종방향으로
    // 실제 연결됨을 확인한 것이다. false는 전역경로가 현재 road에서 끝날 때만 쓴다.
    bool awaitExactReconnect{};
    // 장애물 회피가 아니라 다음 road 연결 차선으로 정렬하는 기동.
    bool roadConnectionManeuver{};
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
