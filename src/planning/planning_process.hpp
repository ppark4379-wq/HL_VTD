#pragma once

#include "data/gt/gt_structs.hpp"
#include "planning/planning_structs.hpp"

#include <string>

// 플래닝 시작 시 reference path CSV를 Waypoint 구조체 배열에 로드한다.
bool initializePlanning(PathPlannerData& planner,
                        const std::string& referencePathCsv);

// GT 한 프레임을 입력받아 한 번의 경로추종 플래닝을 수행한다.
PathPlan runPlanningProcess(PathPlannerData& planner,
                            const GtFrame& gt);
