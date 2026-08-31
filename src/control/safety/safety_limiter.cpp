#include "control/safety/safety_limiter.hpp"

#include <algorithm>
#include <cmath>

void applySafetyLimiter(ControlCommand& safeCommand,
                        SafetyReport& report,
                        const ControlCommand& nominalCommand,
                        const ControllerConfig& limits,
                        bool inputValid,
                        bool emergencyBrakeRequested) {
    report = {};
    safeCommand = nominalCommand;
    const bool finite = std::isfinite(nominalCommand.steering) &&
                        std::isfinite(nominalCommand.targetAccel);
    if (!inputValid || !finite || emergencyBrakeRequested) {
        safeCommand.steering = finite ? nominalCommand.steering : 0.0F;
        safeCommand.targetAccel =
            static_cast<float>(limits.minAcceleration);
        report.commandOverridden = true;
        report.emergencyBrakeApplied = emergencyBrakeRequested;
        report.invalidInput = !inputValid;
        report.invalidCommand = !finite;
    }
    safeCommand.steering = static_cast<float>(std::clamp(
        static_cast<double>(safeCommand.steering),
        -limits.maxSteering, limits.maxSteering));
    safeCommand.targetAccel = static_cast<float>(std::clamp(
        static_cast<double>(safeCommand.targetAccel),
        limits.minAcceleration, limits.maxAcceleration));
}
