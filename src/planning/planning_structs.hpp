#pragma once

#include "data/map/map_structs.hpp"
#include "planning/behavior/behavior_structs.hpp"
#include "planning/global/mission_corridor.hpp"
#include "planning/local/lane_change_structs.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

struct PathPlannerConfig {
    double wheelbase{2.944};
    double straightLookahead{8.0};
    double curveLookahead{5.0};
    // 저속에서 Ld가 길면 목표점이 멀어 조향을 살살 꺾는다. 정지 상태에서
    // 시작하는 장애물 회피가 이것 때문에 실패한다. 반대로 고속에서 Ld가
    // 짧으면 과조향으로 흔들린다. 그래서 Ld를 속도에 비례시키고 위아래를 자른다.
    double minLookahead{3.0};
    double lookaheadSpeedGain{0.6};
    double curveThreshold{0.02};
    double sharpCurveThreshold{0.20};
    double curvatureCheckDistance{25.0};
    // 곡률에 따른 물리적 속도 상한: v <= sqrt(a_lat / curvature).
    // 단계별 curveSpeed만 쓰면 반경 5 m 급커브에서도 20 km/h가
    // 허용되어 조향 한계를 넘는다.
    double maximumLateralAcceleration{1.5};
    double straightSpeed{8.0};
    double curveSpeed{4.0};
    double sharpCurveSpeed{5.5555555556};
    double schoolZoneSpeed{8.0};
    double goalTolerance{1.5};
};

struct PlanningBehaviorConfig {
    bool ignoreTrafficLights{};
    // VTD-only, explicitly enabled by the run configuration. Path/collision
    // stops from confirmed static obstacles may be overridden after a
    // sustained standstill. Dynamic EBS, traffic lights and pedestrian/
    // crosswalk waits retain their ordinary authority.
    double simulationProgressRecovery{};
    double progressRecoveryStopTimeout{3.0};
    double progressRecoverySpeed{3.0 / 3.6};
    double progressRecoveryRetryInterval{0.5};
    double progressRecoverySearchBudgetMs{15.0};
    double comfortableDeceleration{2.5};
    double stopSafetyMargin{1.0};
    double stopHoldTime{1.0};
    double stoppedSpeedThreshold{0.15};
    double followTimeGap{1.8};
    double minimumFollowDistance{5.0};
    // 목표 간격 오차를 선행차 속도에 더하는 ACC 비례이득 [1/s].
    double followGapGain{0.35};
    // 법규 정지나 비상제동이 아닌 객체 대응은 이 속도 아래로
    // 목표속도를 내리지 않는다.
    double minimumObjectResponseSpeed{15.0 / 3.6};
    // 옆 차선 차량이 급격히 진입해도 제동 거리를 확보한다.
    double nearbyObjectCautionSpeed{5.5555555556};
    double nearbyObjectLateralDistance{7.0};
    // 바로 옆 차선에 정차한 승용차급 객체는 아직 움직임이 없어도 출발·끼어들기
    // 가능성이 있으므로 짧은 범위에서만 주의속도를 적용한다. 콘·수목 등은
    // GT bounding box 치수 범위로 제외한다.
    double stationaryAdjacentVehicleCautionSpeed{5.5555555556};
    double stationaryAdjacentVehicleMaximumDistance{25.0};
    // 교차도로에서 정차 중인 차량은 아직 speed=0이어도 다음 프레임에
    // 출발할 수 있다. 두 진행 직선의 교점까지 남은 거리를 이용해 미리
    // 주의속도로 낮춰, 출발을 확인한 뒤에도 제동/회피할 시간을 남긴다.
    double crossTrafficCautionSpeed{10.0 / 3.6};
    double crossTrafficForwardDistance{55.0};
    double crossTrafficApproachDistance{35.0};
    double crossTrafficMinimumAngle{0.6108652382}; // 35 deg
    double denseStaticSpeed{10.0 / 3.6};
    double denseStaticForwardDistance{35.0};
    double denseStaticLateralDistance{7.0};
    double denseStaticPairDistance{15.0};
    double denseStaticReleaseTime{3.0};
    double stationaryVehicleMinimumLength{2.5};
    double stationaryVehicleMaximumLength{7.5};
    double stationaryVehicleMinimumWidth{1.4};
    double stationaryVehicleMaximumWidth{2.8};
    double emergencyTTC{1.5};
    // 고정 TTC만 쓰면 고속에서 정지시간보다 늦게 비상제동한다. 현재 속도에
    // 따른 정지시간 v/a와 인지·작동 여유를 함께 계산한다.
    double emergencyBrakingDeceleration{4.0};
    double emergencyReactionTime{0.4};
    // 한 프레임의 속도/자세 잡음으로 충돌 예측이 잠깐 사라져도 제동을
    // 바로 풀지 않는다. 마지막 위험 검출 뒤 이 시간 동안 제동을 유지한다.
    double emergencyMinimumHoldTime{0.5};
    double collisionPredictionHorizon{3.0};
    double collisionPredictionStep{0.2};
    // 미래 OBB 검사가 Ego 예측 위치를 이 경로 위에서 찾는다.
    // (최대속도 x collisionPredictionHorizon)보다 짧으면 예측이 중간에 잘린다.
    double localPathHorizon{80.0};
    // Follow 해제 임계값 배수. 진입/해제를 분리해 경계 진동을 막는다.
    double followReleaseRatio{1.3};
    // Recovery 진입 후 최소 정지 유지시간. 맵매칭이 안정될 시간을 준다.
    double recoveryMinimumHoldTime{0.5};
    // 앞범퍼가 정지선을 넘은 뒤에도 정지선을 유효로 보는 거리.
    // 이 값이 짧으면 적색신호에서 살짝 넘은 순간 정지 요청이 사라진다.
    double stopLineBehindWindow{8.0};
    // 신호 정지선을 찾는 최대 전방 거리.
    // GT trafficLight.id(= XODR controller id)가 일치하는 정지선만 본다.
    double signalStopSearchDistance{80.0};
    // 횡단보도 진입/점유가 예상되는 객체가 있을 때만 적용하는 접근속도.
    // 처음부터 빈 횡단보도는 전역경로의 원래 목표속도로 통과한다.
    double crosswalkApproachSpeed{5.5};      // [m/s]
    double crosswalkApproachDistance{25.0};  // [m] 이 거리부터 감속
    double crosswalkNearbyObjectDistance{5.0}; // [m] 주변 객체 조기감속 범위
    // 한 번 정지 원인이 된 객체가 횡단보도를 완전히 비운 뒤, Ego가
    // 횡단보도를 빠져나갈 때까지 적용하는 통과속도.
    double crosswalkClearanceSpeed{6.9};     // [m/s]
    // 보행자의 전체 OBB가 횡단보도를 빠져나간 뒤에도 이 시간만큼
    // 정지를 유지해 경계 프레임 튤로 조기 출발하는 것을 막는다.
    double crosswalkClearanceHoldTime{1.0};  // [s]
    // 횡단보도 밖이라도 이 시간 뒤 예상 위치가 안이면 점유로 본다.
    double crosswalkPedestrianPrediction{4.0}; // [s]
    // 횡단보도 앞 이 거리 안의 정지선을 정지 목표로 삼는다.
    double crosswalkStopLineSearch{15.0};    // [m]
};

struct PathPlan {
    bool valid{};
    bool goalReached{};
    std::size_t nearestIndex{};
    std::size_t targetIndex{};
    double lookahead{};
    double maxCurvature{};
    double targetSpeed{};
    double steering{};
};

struct PathPlannerData {
    PathPlannerConfig config;
    std::vector<Waypoint> waypoints;
    std::size_t nearestIndex{};
    PathPlan latestPlan;
};

struct LocalPathData {
    bool valid{};
    std::size_t beginIndex{};
    std::size_t endIndex{};
    std::vector<Waypoint> points;
};

struct PathSafetyResult {
    bool valid{};
    bool collision{};  // 현재 프레임에서 실제 OBB가 겹침
    bool predictedCollision{};
    double predictedCollisionTTC{};
    double minimumTTC{};
    int riskObjectId{-1};
};

struct PlanningFrameData {
    ConstraintSet constraints;
    StopDecision stop;
    BehaviorDecision behavior;
    LocalPathData localPath;
    PathSafetyResult pathSafety;
};

struct PlanningResult {
    bool valid{};
    bool goalReached{};
    double targetSpeed{};
    StopReason stopReason{StopReason::None};
    LonMode longitudinal{LonMode::Cruise};
    SystemHealth health{SystemHealth::Ok};
    PathPlan trackingPlan;
    LocalPathData localPath;
    PathSafetyResult pathSafety;
    bool emergencyBrake{};
    std::uint8_t turnSignal{};
    // 실행 로그가 Planner가 실제로 적용한 제한과 상태를 기록하도록
    // 최종 의사결정과 함께 내보낸다.
    double currentSpeedLimit{};
    bool schoolZoneActive{};
    bool stopDistanceValid{};
    double stopDistance{};
    bool crosswalkActive{};
    bool laneChangeActive{};
    bool progressRecoveryActive{};
    bool progressRecoveryContactExpected{};
    bool progressRecoveryEmergencyOverridden{};
    StopReason progressRecoveryOverriddenStop{StopReason::None};
};

// Kept separate from ordinary, collision-approved lane-change trajectories.
// A simulation escape must never enter a cache labelled "safe".
struct ProgressRecoveryData {
    PathPlannerData planner;
    bool active{};
    bool anchorValid{};
    double anchorX{}, anchorY{};
    double blockedTime{}, noMotionTime{}, retryRemaining{}, clearTime{};
    double startX{}, startY{};
    std::size_t missionLeg{static_cast<std::size_t>(-1)};
    std::size_t searchCursor{};
    bool contactExpected{};
};

struct StationaryObjectState {
    std::uint32_t id{};
    double stationaryDuration{};
    double lastX{};
    double lastY{};
    bool hasPosition{};
    bool hasEverMoved{};
};

struct CrosswalkOccupancyState {
    int id{-1};
    bool occupied{};
    double clearanceHoldRemaining{};
};

struct PlanningProcessData {
    PathPlannerData pathPlanner;
    // pathPlanner는 checkpoint의 road corridor/구역 route_s를 보존한다.
    // roadLanePlanner는 현재 자유 차선을 추종하는 조향용 경로다.
    PathPlannerData roadLanePlanner;
    // A cached lane path belongs to exactly one mission leg.  Without this
    // ownership marker, the first frame after advancing to the next
    // intersection could accept the exhausted previous-leg path as a
    // geometric fallback and steer away from the new gateway.
    std::size_t roadLanePlannerMissionLeg{};
    bool roadLanePlannerMissionLegValid{};
    // 교차로 진출 checkpoint 사이에서는 allowedLanes를 자유롭게
    // 사용하고, 진입 checkpoint 근처에서만 targetLanes에 정렬한다.
    MissionCorridorData missionCorridor;
    PlanningBehaviorConfig config;
    BehaviorState behaviorState;
    StopManagerData stopManager;
    LaneChangeData laneChange;
    LaneChangeConfig laneChangeConfig;
    // Keep the local worker's speed contract across a cached-tail handoff;
    // finishing the manoeuvre alone is not proof of a safe global rejoin.
    // 움직이던 차량이 정지하면 짧은 확인 후 정적 회피 대상으로
    // 전환하기 위한 ID별 이력이다. 사라진 ID는 다음 프레임에 제거된다.
    std::vector<StationaryObjectState> stationaryObjects;
    PlanningFrameData frame;
    PlanningResult result;
    // 횡단보도 점유·해제 유지 상태. 점유 해제 후에도 잠시 정지하고,
    // Ego가 횡단보도를 완전히 통과할 때까지 clearance 속도를 적용한다.
    std::vector<CrosswalkOccupancyState> crosswalkStates;
    double emergencyBrakeHoldRemaining{};
    ProgressRecoveryData progressRecovery;
    // 급커브가 최근점 뒤로 넘어가자마자 가속하지 않고,
    // 조향이 풀리고 출구 차선에 안착할 시간을 보장한다.
    double sharpCurveSpeedHoldRemaining{};
    double previousRouteS{};
    bool hasPreviousRouteS{};
};
