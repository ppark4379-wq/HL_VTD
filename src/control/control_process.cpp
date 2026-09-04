#include "control/control_process.hpp"

#include "control/lateral/lateral_controller.hpp"
#include "control/longitudinal/longitudinal_controller.hpp"
#include "control/safety/safety_limiter.hpp"

#include <algorithm>

void runControlProcess(ControlProcessData& data,
                       const WorldState& world,
                       const PlanningResult& planning,
                       double dt) {
    data.egoSpeed = world.egoSpeed;
    data.nominalCommand = {};

    runLateralControl(data.nominalCommand, planning.trackingPlan,
                      data.controller.config.maxSteering);
    runLongitudinalControl(data.controller, data.nominalCommand,
                           world.egoSpeed, planning.targetSpeed,
                           std::max(dt, 1e-3));
    // 조향 변화율 제한. 경로 추종 자체는 그대로 두고 명령만 매끄럽게 만든다.
    {
        const double step = std::max(dt, 1e-3) *
                            data.controller.config.maxSteeringRate;
        const double previous = data.controller.hasPreviousSteering
                                    ? data.controller.previousSteering
                                    : 0.0;
        const double limited = std::clamp(
            static_cast<double>(data.nominalCommand.steering),
            previous - step, previous + step);
        data.controller.previousSteering = limited;
        data.controller.hasPreviousSteering = true;
        data.nominalCommand.steering = static_cast<float>(limited);
    }
    data.nominalCommand.turnSignal = planning.turnSignal;

    applySafetyLimiter(data.safeCommand, data.safety,
                       data.nominalCommand, data.controller.config,
                       world.inputValid && planning.valid,
                       planning.emergencyBrake);
    data.latestCommand = data.safeCommand;
}
