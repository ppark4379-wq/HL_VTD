#pragma once

#include "control/control_structs.hpp"

// 이전 명령과 dt를 기준으로 가속도 명령 변화량(jerk)을 제한한다.
double limitAccelerationRate(ControllerData& controller,
                             double acceleration,
                             double dt);

// 현재속도와 목표속도의 오차로 PD 가속도 명령을 계산한다.
double computePdAcceleration(ControllerData& controller,
                             double currentSpeed,
                             double targetSpeed,
                             double dt);

// 목표속도를 VTD 가속도 명령으로 변환한다.
// Read : state.config, state.previous*
// Write: state.previous*, output.targetAccel
void runLongitudinalControl(ControllerData& state,
                            ControlCommand& output,
                            double currentSpeed,
                            double targetSpeed,
                            double dt);
