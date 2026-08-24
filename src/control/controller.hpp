#pragma once

#include "control/control_structs.hpp"
#include "planning/planning_structs.hpp"

// 조향각을 차량의 허용 범위로 제한한다.
double limitSteering(double steering, double maxSteering);

// 이전 명령과 dt를 기준으로 가속도 명령 변화량을 제한한다.
double limitAccelerationRate(ControllerData& controller,
                             double acceleration,
                             double dt);

// 현재속도와 목표속도의 오차로 PD 가속도 명령을 계산한다.
double computePdAcceleration(ControllerData& controller,
                             double currentSpeed,
                             double targetSpeed,
                             double dt);

// 플래닝 결과와 PD 가속도를 VTD 제어 명령으로 만든다.
ControlCommand makeControlCommand(ControllerData& controller,
                                  const PathPlan& plan,
                                  double currentSpeed,
                                  double dt);
