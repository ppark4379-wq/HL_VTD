#pragma once

#include "control/control_structs.hpp"
#include "data/gt/gt_structs.hpp"
#include "planning/planning_structs.hpp"

// 연속한 Ego GT 위치의 차이와 dt로 현재속도를 계산한다.
double estimateEgoSpeed(ControlProcessData& control,
                        const GtFrame& gt,
                        double dt);

// 플래닝 결과를 받아 PD 속도제어와 조향 제한을 실행한다.
ControlCommand runControlProcess(ControlProcessData& control,
                                 const GtFrame& gt,
                                 const PathPlan& plan,
                                 double dt);
