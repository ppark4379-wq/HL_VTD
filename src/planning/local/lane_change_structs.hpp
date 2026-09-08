#pragma once

#include "data/map/map_structs.hpp"
#include "planning/behavior/behavior_structs.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

struct LaneChangeConfig {
    double latticeShortDistance{12.0};
    double latticeSearchBudgetMs{15.0};
    double latticeMediumDistance{20.0};
    double latticeLongDistance{30.0};
    double latticeLaneCost{2.0};
    double latticeCurvatureCost{30.0};
    double latticeProgressCost{1.0};
    double latticeObstacleClearance{0.30};
    // Minimum tracking allowance used only after the nominal-clearance
    // search proves that a mandatory static-obstacle passage is too narrow.
    // The physical OBB must still remain collision-free.
    double latticeTightObstacleClearance{0.05};
    double gatewayReserveDistance{8.0};
    double obstacleTriggerMinimumDistance{5.0};
    double obstacleTriggerMaximumDistance{70.0};
    double obstacleMandatoryAvoidanceDistance{15.0};
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
    // This trajectory was built from an official mission-guide or from a
    // checked recovery/avoidance trajectory that rejoins that same guide.
    // Keep the authority on the trajectory itself: a lane key is not enough
    // to distinguish outgoing branches that overlap before a split.
    bool missionRouteAuthority{};
    std::uint8_t turnSignal{};
    std::uint8_t outwardTurnSignal{};
    std::uint8_t returnTurnSignal{};
    double phaseElapsed{};
    double startRouteS{};
    double returnStartRouteS{};
    double rejoinRouteS{};
    bool waitingForReturn{};
    // 첫 횡이동이 실제로 끝나는 물리 차선. 후속 회피 재계획은 Ego가
    // 이 차선에 안착한 뒤에만 허용해, 출발 차선에서 계획을 반복 초기화하지 않는다.
    LaneKey shiftCompletedLane;
    bool shiftCompletedLaneValid{};
    bool gatewayReplanRequestedAfterShift{};
    // 현재 횡이동이 끝난 직후 같은 종방향 차선 연결 위에서 만날
    // 다음 정적 장애물. Execute 중 차선 중심에 먼저 안착하면 현재
    // 기동 종료를 기다리지 않고 다음 회피 후보로 원자적으로 교체한다.
    int followupObstacleId{-1};
    double followupObstacleClearance{-1.0};
    // 모든 목표 차선에 후속 장애물이 가까운 경우에도 후보를 없애지 않고
    // 가장 멀리 진행 가능한 안전한 1단계 경로를 선택했는지 표시한다.
    bool bestProgressFallback{};
    // 같은 장애물/차선에서 모든 후보가 기하·OBB 검사에
    // 탈락하면 다음 20 Hz 프레임에서 수백 후보를 즉시 다시
    // 계산하지 않는다. Ego가 2 m 이동하거나 짧은 simulation
    // cooldown이 끝나면 재탐색한다. 이 상태는 phase=None에서도
    // 유지되어 VTD command 전송이 멈추는 것을 막는다.
    int failedObstacleId{-1};
    LaneKey failedSearchLane;
    bool failedSearchLaneValid{};
    double failedSearchRoadS{};
    double retrySearchAfter{};
    std::size_t nearestIndex{};
    std::vector<Waypoint> path;
};
