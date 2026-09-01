#pragma once

#include "data/gt/gt_structs.hpp"
#include "data/map/map_structs.hpp"
#include "planning/planning_structs.hpp"

#include <string>

struct DecisionTraceData;

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
