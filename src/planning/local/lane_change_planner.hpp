#pragma once

#include "data/gt/gt_structs.hpp"
#include "data/map/map_structs.hpp"
#include "planning/planning_structs.hpp"

struct DecisionTraceData;

// Packaged route에서는 road ID 순서만 사용하고, 평상시 조향 경로는
// Ego가 현재 타고 있는 차선의 중앙선으로 만든다.
bool buildRoadIdTrackingPath(
    PathPlannerData& output,
    const PathPlannerConfig& plannerConfig,
    const WorldState& world,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex,
    double horizon,
    DecisionTraceData* trace = nullptr,
    const MissionCorridorData* mission = nullptr);

// 장애물 회피가 없을 때만, 도로 마지막 구간에서 다음 road ID로
// 연결되는 차선으로 정렬한다.
bool tryPrepareRoadConnectionLaneChange(
    LaneChangeData& state,
    const LaneChangeConfig& config,
    const PlanningBehaviorConfig& safetyConfig,
    const WorldState& world,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex,
    DecisionTraceData* trace = nullptr,
    const MissionCorridorData* mission = nullptr,
    const LaneAlignmentContext* alignment = nullptr);

// pendingBlockerDistance가 non-null이면, 이번 프레임에 확인된 정지 장애물의
// relativeS를 채운다(없으면 -1). 차선변경 후보를 아직 못 찾았어도(반환값이
// false여도) 이 값으로 "회피에 필요한 거리를 미리 확보하도록 감속"할 수 있다.
bool tryPrepareObstacleLaneChange(
    LaneChangeData& state,
    const LaneChangeConfig& config,
    const PlanningBehaviorConfig& safetyConfig,
    const WorldState& world,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex,
    double* pendingBlockerDistance = nullptr,
    const std::vector<int>* confirmedStationaryObjectIds = nullptr,
    DecisionTraceData* trace = nullptr,
    const MissionCorridorData* mission = nullptr,
    bool activeReferenceIsMissionGuide = false,
    const LaneAlignmentContext* alignment = nullptr);

LaneAlignmentContext laneAlignmentBeforeStopLine(
    const WorldState& world,const PathGeneratorData& map,
    const PathPlannerData& reference,double stopMargin,double searchDistance);

void advanceLaneChangePhase(LaneChangeData& state,
                            const LaneChangeConfig& config,
                            const WorldState& world,
                            double dt,
                            bool executionBlocked,
                            bool obstacleStillRequiresManeuver,
                            bool candidatePathSafe,
                            DecisionTraceData* trace = nullptr,
                            bool successorPathReady = true);

// 신호와 횡단보도 등 활성 정지 요청은 회피 기동 진입도 막는다.
bool hasLaneChangeExecutionBlocker(const StopManagerData& stopManager);
bool hasLaneChangeExecutionBlocker(const StopManagerData& stopManager,
                                   double maneuverEndRouteS);

void makeLaneChangeLocalPath(LocalPathData& output,
                             const LaneChangeData& state);

PathPlan trackLaneChangePath(LaneChangeData& state,
                             const PathPlannerConfig& plannerConfig,
                             const GtFrame& gt,
                             double egoSpeed);
