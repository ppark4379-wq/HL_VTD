#pragma once

#include "control/control_structs.hpp"
#include "data/gt/gt_structs.hpp"
#include "planning/planning_structs.hpp"

void runControlProcess(ControlProcessData& data,
                       const WorldState& world,
                       const PlanningResult& planning,
                       double dt);
