#include "control/control_process.hpp"

#include "control/controller.hpp"

#include <cmath>

// GT에 Ego speed 필드가 없으므로 연속 월드 위치 차분으로 추정한다.
double estimateEgoSpeed(ControlProcessData& control,
                        const GtFrame& gt,
                        double dt) {
    if (!control.hasPreviousPosition || dt <= 1e-6) {
        control.previousX = gt.ego.x;
        control.previousY = gt.ego.y;
        control.hasPreviousPosition = true;
        control.egoSpeed = 0.0;
        return control.egoSpeed;
    }

    const double dx = gt.ego.x - control.previousX;
    const double dy = gt.ego.y - control.previousY;
    control.egoSpeed = std::hypot(dx, dy) / dt;
    control.previousX = gt.ego.x;
    control.previousY = gt.ego.y;
    return control.egoSpeed;
}

// 제어 루프의 최상위 호출 흐름.
ControlCommand runControlProcess(ControlProcessData& control,
                                 const GtFrame& gt,
                                 const PathPlan& plan,
                                 double dt) {
    const double currentSpeed = estimateEgoSpeed(control, gt, dt);
    control.latestCommand = makeControlCommand(
        control.controller, plan, currentSpeed, dt);
    return control.latestCommand;
}
