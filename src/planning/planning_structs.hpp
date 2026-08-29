#pragma once

#include "data/map/map_structs.hpp"
#include "planning/behavior/behavior_structs.hpp"
#include "planning/local/lane_change_structs.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

struct PathPlannerConfig {
    double wheelbase{2.944};
    double straightLookahead{8.0};
    double curveLookahead{5.0};
    double curveThreshold{0.02};
    double curvatureCheckDistance{25.0};
    double straightSpeed{8.0};
    double curveSpeed{4.0};
    double schoolZoneSpeed{8.0};
    double goalTolerance{1.5};
};

struct PlanningBehaviorConfig {
    double comfortableDeceleration{2.5};
    double stopSafetyMargin{1.0};
    double stopHoldTime{1.0};
    double stoppedSpeedThreshold{0.15};
    double followTimeGap{1.8};
    double minimumFollowDistance{5.0};
    double emergencyTTC{1.5};
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
    PathPlan trackingPlan;
    LocalPathData localPath;
    PathSafetyResult pathSafety;
    bool emergencyBrake{};
    std::uint8_t turnSignal{};
};

struct PlanningProcessData {
    PathPlannerData pathPlanner;
    PlanningBehaviorConfig config;
    BehaviorState behaviorState;
    StopManagerData stopManager;
    LaneChangeData laneChange;
    LaneChangeConfig laneChangeConfig;
    PlanningFrameData frame;
    PlanningResult result;
    double previousRouteS{};
    bool hasPreviousRouteS{};
};
