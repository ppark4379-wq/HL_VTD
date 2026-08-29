#pragma once

#include "planning/planning_structs.hpp"

#include <string>

// 체크포인트와 이미 로드된 XODR lane graph로 전역경로를 만든다.
bool buildGlobalPath(PathPlannerData& planner,
                     PathGeneratorData& map,
                     const std::string& checkpointCsv);
