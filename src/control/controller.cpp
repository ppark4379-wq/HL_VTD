#include "control/controller.hpp"

#include <algorithm>

using namespace std;

// Pure Pursuit 조향각을 차량의 최대 조향각 이내로 제한한다.
double limitSteering(double steering, double maxSteering) {
    return clamp(steering, -maxSteering, maxSteering);
}

// speed error의 비례항과 미분항을 더해 종방향 가속도를 계산한다.
double computePdAcceleration(ControllerData& controller,
                             double currentSpeed,
                             double targetSpeed,
                             double dt) {
    const double error = targetSpeed - currentSpeed;
    double derivative = 0.0;
    if (controller.hasPreviousError && dt > 1e-6) {
        derivative = (error - controller.previousSpeedError) / dt;
    }

    const double acceleration = controller.config.kp * error +
                                controller.config.kd * derivative;
    controller.previousSpeedError = error;
    controller.hasPreviousError = true;
    return clamp(acceleration,
                 controller.config.minAcceleration,
                 controller.config.maxAcceleration);
}

// 플래너의 조향각과 목표속도를 VTD 제어 명령으로 변환한다.
ControlCommand makeControlCommand(ControllerData& controller,
                                  const PathPlan& plan,
                                  double currentSpeed,
                                  double dt) {
    ControlCommand command;
    if (!plan.valid) return command;

    command.steering = static_cast<float>(
        limitSteering(plan.steering, controller.config.maxSteering));
    command.targetAccel = static_cast<float>(
        computePdAcceleration(controller, currentSpeed, plan.targetSpeed, dt));
    command.turnSignal = 0;
    return command;
}
