#pragma once

#include "data/gt/gt_structs.hpp"
#include "data/map/map_structs.hpp"
#include "planning/planning_structs.hpp"

#include <string>

struct DecisionTraceData;

// A still-received actor is not necessarily still blocking the original
// driving path. Only retire an unstarted, single-object detour when the
// selected successor is safe and already continuous with the measured pose.
bool canCancelClearedPreparedAvoidance(
    const LaneChangeData& maneuver, const WorldState& world,
    const std::vector<Waypoint>& successor, bool successorPathSafe);

bool initializePlanning(PlanningProcessData& data,
                        const PathPlannerConfig& plannerConfig,
                        const PlanningBehaviorConfig& behaviorConfig,
                        const LaneChangeConfig& laneChangeConfig,
                        const std::string& referencePathCsv);

void runPlanningProcess(PlanningProcessData& data,
                        const GtFrame& gt,
                        const WorldState& world,
                        const PathGeneratorData& map,
                        double dt,
                        DecisionTraceData* trace = nullptr);
