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
    const double steeringBeforeFrame = data.controller.hasPreviousSteering
        ? data.controller.previousSteering : 0.0;

    runLateralControl(data.nominalCommand, planning.trackingPlan,
                      data.controller.config.maxSteering);
    runLongitudinalControl(data.controller, data.nominalCommand,
                           world.egoSpeed, planning.targetSpeed,
                           std::max(dt, 1e-3));
    // 조향 변화율 제한. 경로 추종 자체는 그대로 두고 명령만 매끄럽게 만든다.
    {
        // A planner overrun must not turn into one large steering jump.  The
        // actuator still receives only a bounded slew interval; subsequent
        // frames catch up smoothly.  This is scenario-independent and also
        // covers simulator frame skips.
        constexpr double kMaximumSteeringSlewInterval = 0.08;
        const double slewInterval = std::clamp(
            std::max(dt, 1e-3), 1e-3, kMaximumSteeringSlewInterval);
        const double step = slewInterval *
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
    if (planning.emergencyBrake) {
        if (!data.emergencySteeringLatched) {
            data.emergencySteering = static_cast<float>(steeringBeforeFrame);
            data.emergencySteeringLatched = true;
        }
        data.nominalCommand.steering = data.emergencySteering;
        data.controller.previousSteering = data.emergencySteering;
    } else {
        data.emergencySteeringLatched = false;
    }
    data.nominalCommand.turnSignal = planning.turnSignal;

    applySafetyLimiter(data.safeCommand, data.safety,
                       data.nominalCommand, data.controller.config,
                       // Planning validity already includes the appropriate
                       // input authority.  For checkpoint missions that may
                       // be a successful projection onto the validated XODR
                       // corridor while VTD's instantaneous lane match is
                       // unavailable.  Rechecking the raw world flag here
                       // used to override a valid recovery command with
                       // permanent braking.
                       planning.valid,
                       planning.emergencyBrake);
    data.latestCommand = data.safeCommand;
}
