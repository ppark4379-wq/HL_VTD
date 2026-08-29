#include "control/longitudinal/longitudinal_controller.hpp"

#include <algorithm>

double limitAccelerationRate(ControllerData& controller,
                             double acceleration,
                             double dt) {
    const double previousAcceleration =
        controller.hasPreviousAcceleration
            ? controller.previousAcceleration
            : 0.0;
    const double maxChange = controller.config.maxAccelerationRate * dt;
    const double limitedAcceleration = std::clamp(
        acceleration,
        previousAcceleration - maxChange,
        previousAcceleration + maxChange);

    controller.previousAcceleration = limitedAcceleration;
    controller.hasPreviousAcceleration = true;
    return limitedAcceleration;
}

// speed error의 비례항과 미분항을 더해 종방향 가속도를 계산한다.
double computePdAcceleration(ControllerData& controller,
                             double currentSpeed,
                             double targetSpeed,
                             double dt) {
    const double error = targetSpeed - currentSpeed;
    double derivative = 0.0;
    if (controller.hasPreviousError && dt > 1e-6) {
        const double rawDerivative =
            (error - controller.previousSpeedError) / dt;
        derivative = std::clamp(
            rawDerivative,
            -controller.config.maxErrorDerivative,
            controller.config.maxErrorDerivative);
    }

    const double requestedAcceleration =
        controller.config.kp * error + controller.config.kd * derivative;
    const double boundedAcceleration = std::clamp(
        requestedAcceleration,
        controller.config.minAcceleration,
        controller.config.maxAcceleration);
    controller.previousSpeedError = error;
    controller.hasPreviousError = true;
    return limitAccelerationRate(controller, boundedAcceleration, dt);
}

void runLongitudinalControl(ControllerData& state,
                            ControlCommand& output,
                            double currentSpeed,
                            double targetSpeed,
                            double dt) {
    output.targetAccel = static_cast<float>(
        computePdAcceleration(state, currentSpeed, targetSpeed, dt));
}
