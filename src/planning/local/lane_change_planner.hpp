#pragma once

#include "data/gt/gt_structs.hpp"
#include "data/map/map_structs.hpp"
#include "planning/planning_structs.hpp"

bool tryPrepareObstacleLaneChange(
    LaneChangeData& state,
    const LaneChangeConfig& config,
    const PlanningBehaviorConfig& safetyConfig,
    const WorldState& world,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex);

void advanceLaneChangePhase(LaneChangeData& state,
                            const LaneChangeConfig& config,
                            const WorldState& world,
                            double dt,
                            bool executionBlocked,
                            bool candidatePathSafe);

bool isAvoidedLaneChangeObject(const LaneChangeData& state, int objectId);

void makeLaneChangeLocalPath(LocalPathData& output,
                             const LaneChangeData& state);

PathPlan trackLaneChangePath(LaneChangeData& state,
                             const PathPlannerConfig& plannerConfig,
                             const GtFrame& gt,
                             double speedLimit);
