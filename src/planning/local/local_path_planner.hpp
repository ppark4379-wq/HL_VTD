#pragma once

#include "planning/planning_structs.hpp"

#include <cstddef>

// 전역경로에서 현재 위치부터 horizon만큼의 지역경로를 추출한다.
void generateLocalPath(LocalPathData& output,
                       const PathPlannerData& globalPath,
                       std::size_t nearestIndex,
                       double horizon);
