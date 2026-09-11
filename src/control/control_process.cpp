#include "control/control_process.hpp"

#include "control/lateral/lateral_controller.hpp"
#include "control/longitudinal/longitudinal_controller.hpp"
#include "control/safety/safety_limiter.hpp"

#include <algorithm>
#include <cmath>

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
    // After the simulation standstill timeout has expired, a retained -10
    // stop-hold state must not add another braking interval to every retry.
    // Steering slew and the final emergency/invalid-input limiter still run.
    if (planning.progressRecoveryActive && planning.valid && !planning.emergencyBrake &&
        planning.targetSpeed > .05 && std::abs(world.egoSpeed) <= .2) {
        const double launch = std::min(data.controller.config.maxAcceleration,
            std::max(.3, data.controller.config.kp * (planning.targetSpeed-world.egoSpeed)));
        data.nominalCommand.targetAccel = static_cast<float>(std::max(
            static_cast<double>(data.nominalCommand.targetAccel), launch));
        data.controller.previousAcceleration = data.nominalCommand.targetAccel;
    }
    // Speed-only PD and its jerk limiter can brake too weakly for a late
    // traffic stop. Bound the command by remaining FRONT-BUMPER distance.
    // This does not affect ordinary driving/avoidance or weaken EBS.
    const bool trafficStop = planning.stopReason == StopReason::RedLight ||
        planning.stopReason == StopReason::FlashingRed ||
        planning.stopReason == StopReason::CrosswalkBlocked;
    if (trafficStop && planning.stopDistanceValid &&
        std::isfinite(planning.stopDistance) &&
        (world.egoSpeed > planning.targetSpeed + 0.05 ||
         planning.targetSpeed <= 0.05)) {
        const double speed = std::max(0.0, world.egoSpeed);
        // Reserve 1 m at the line and 0.35 s for measurement/actuation lag.
        const double available = std::max(0.1,
            planning.stopDistance - 1.0 - speed * 0.35);
        const double required = planning.targetSpeed <= 0.05
            ? data.controller.config.minAcceleration
            : std::max(data.controller.config.minAcceleration,
                       -speed * speed / (2.0 * available));
        data.nominalCommand.targetAccel = static_cast<float>(std::min(
            static_cast<double>(data.nominalCommand.targetAccel), required));
        data.controller.previousAcceleration = data.nominalCommand.targetAccel;
    }
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
