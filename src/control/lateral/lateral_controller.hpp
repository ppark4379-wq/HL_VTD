#pragma once

#include "control/control_structs.hpp"

// 조향각을 차량의 허용 범위로 제한한다.
double limitSteering(double steering, double maxSteering);

// 플래닝의 조향 목표를 VTD 조향 명령으로 변환한다.
// Read : plan.valid, plan.steering
// Write: output.steering
void runLateralControl(ControlCommand& output,
                       const PathPlan& plan,
                       double maxSteering);
