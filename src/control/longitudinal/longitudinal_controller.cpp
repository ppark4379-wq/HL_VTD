#include "control/longitudinal/longitudinal_controller.hpp"

#include <algorithm>
#include <cmath>

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
    // 목표속도 0에서 PD만 쓰면 가속도가 -kp*v로 속도에 비례해 줄어든다.
    // 속도는 0에 수렴하지만 거리는 v0/kp 만큼 계속 나아가므로, 정지선을
    // 몇 m 지나 서게 된다. 저속에서는 PD 대신 확실한 감속을 명령해
    // 이 꼬리를 끊는다.
    // stopHoldSpeedThreshold is the measured-speed threshold for declaring
    // the vehicle stopped.  It must not also classify every small positive
    // curve-following target as a stop request: M05 legitimately requested
    // 0.35 m/s on a sharp connector and was then held at -4 m/s^2 forever.
    // Only an actual zero-speed behavior command may enter the hold branch.
    constexpr double zeroTargetThreshold=0.05;
    const bool holdingStop =
        targetSpeed <= zeroTargetThreshold &&
        currentSpeed <= state.config.stopHoldSpeedThreshold;
    if (holdingStop) {
        state.previousSpeedError = targetSpeed - currentSpeed;
        state.hasPreviousError = true;
        output.targetAccel = static_cast<float>(limitAccelerationRate(
            state, state.config.minAcceleration, dt));
        return;
    }
    double acceleration=computePdAcceleration(
        state,currentSpeed,targetSpeed,dt);
    // A positive command used to survive for several frames after crossing a
    // newly raised cruise target because the symmetric jerk limiter could
    // only unwind +2 m/s^2 gradually.  VTD then overshot a 45 km/h target by
    // more than 5 km/h, which looked like a speed runaway.  Taper propulsion
    // in the last 1 m/s and never retain positive acceleration at/above the
    // target.  Negative braking commands are left untouched.
    constexpr double approachBand=1.0;
    if(targetSpeed>state.config.stopHoldSpeedThreshold &&
       currentSpeed>targetSpeed-approachBand) {
        const double headroom=std::max(0.,targetSpeed-currentSpeed);
        acceleration=std::min(acceleration,.6*headroom);
        state.previousAcceleration=acceleration;
    }
    output.targetAccel=static_cast<float>(acceleration);
}
