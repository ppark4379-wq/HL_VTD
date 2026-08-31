#pragma once

#include "control/control_structs.hpp"

void applySafetyLimiter(ControlCommand& safeCommand,
                        SafetyReport& report,
                        const ControlCommand& nominalCommand,
                        const ControllerConfig& limits,
                        bool inputValid,
                        bool emergencyBrakeRequested);
