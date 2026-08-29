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
    data.nominalCommand.turnSignal = planning.turnSignal;

    applySafetyLimiter(data.safeCommand, data.safety,
                       data.nominalCommand, data.controller.config,
                       world.inputValid && planning.valid,
                       planning.emergencyBrake);
    data.latestCommand = data.safeCommand;
}
