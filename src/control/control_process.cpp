#include "control/control_process.hpp"

#include "control/controller.hpp"

#include <cmath>

namespace {

double computePositionSpeed(const ControlProcessData& control,
                            const GtFrame& gt,
                            double dt) {
    const double dx = gt.ego.x - control.previousX;
    const double dy = gt.ego.y - control.previousY;
    return std::hypot(dx, dy) / dt;
}

double applySpeedFilter(ControlProcessData& control,
                        double speed,
                        double alpha) {
    if (!control.hasSpeedEstimate) {
        control.hasSpeedEstimate = true;
        return speed;
    }

    return alpha * speed + (1.0 - alpha) * control.egoSpeed;
}

void updatePreviousPosition(ControlProcessData& control,
                            const GtFrame& gt) {
    control.previousX = gt.ego.x;
    control.previousY = gt.ego.y;
    control.hasPreviousPosition = true;
}

void resetSpeedAndControllerState(ControlProcessData& control) {
    control.rawEgoSpeed = 0.0;
    control.egoSpeed = 0.0;
    control.hasSpeedEstimate = false;

    control.controller.previousSpeedError = 0.0;
    control.controller.previousAcceleration = 0.0;
    control.controller.hasPreviousError = false;
    control.controller.hasPreviousAcceleration = false;
}

}  // namespace

// GT에 Ego speed 필드가 없으므로 연속 월드 위치 차분으로 추정한다.
double estimateEgoSpeed(ControlProcessData& control,
                        const GtFrame& gt,
                        const ControlRuntimeConfig& runtime) {
    if (!control.hasPreviousPosition) {
        updatePreviousPosition(control, gt);
        control.rawEgoSpeed = 0.0;
        control.egoSpeed = 0.0;
        return control.egoSpeed;
    }

    const double positionSpeed =
        computePositionSpeed(control, gt, runtime.nominalDt);
    if (!std::isfinite(positionSpeed) ||
        positionSpeed > runtime.maxValidPositionSpeed) {
        updatePreviousPosition(control, gt);
        resetSpeedAndControllerState(control);
        return control.egoSpeed;
    }

    control.rawEgoSpeed = positionSpeed;
    control.egoSpeed = applySpeedFilter(
        control, control.rawEgoSpeed, runtime.speedFilterAlpha);
    updatePreviousPosition(control, gt);
    return control.egoSpeed;
}

// 제어 루프의 최상위 호출 흐름.
ControlCommand runControlProcess(ControlProcessData& control,
                                 const GtFrame& gt,
                                 const PathPlan& plan,
                                 const ControlRuntimeConfig& runtime) {
    const double currentSpeed = estimateEgoSpeed(control, gt, runtime);
    control.latestCommand = makeControlCommand(
        control.controller, plan, currentSpeed, runtime.nominalDt);
    return control.latestCommand;
}
