#pragma once

#include "data/gt/gt_structs.hpp"
#include "data/map/map_structs.hpp"
#include "planning/planning_structs.hpp"

struct DecisionTraceData;

bool tryPrepareObstacleLaneChange(
    LaneChangeData& state,
    const LaneChangeConfig& config,
    const PlanningBehaviorConfig& safetyConfig,
    const WorldState& world,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex,
    const std::vector<int>* persistentStoppedObjectIds = nullptr,
    DecisionTraceData* trace = nullptr);

void advanceLaneChangePhase(LaneChangeData& state,
                            const LaneChangeConfig& config,
                            const WorldState& world,
                            double dt,
                            bool executionBlocked,
                            bool obstacleStillRequiresManeuver,
                            bool candidatePathSafe,
                            DecisionTraceData* trace = nullptr);

// A verified lane-change path is collision-checked against every world object,
// so obstacle stops on the original route must not keep that maneuver in
// Prepare. Traffic rules and other non-obstacle stops still block execution.
bool hasLaneChangeExecutionBlocker(const StopManagerData& stopManager,
                                   bool candidatePathSafe);

void makeLaneChangeLocalPath(LocalPathData& output,
                             const LaneChangeData& state);

PathPlan trackLaneChangePath(LaneChangeData& state,
                             const PathPlannerConfig& plannerConfig,
                             const GtFrame& gt,
                             double speedLimit,
                             double egoSpeed);
