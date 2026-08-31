#include "control/lateral/lateral_controller.hpp"

#include <algorithm>

double limitSteering(double steering, double maxSteering) {
    return std::clamp(steering, -maxSteering, maxSteering);
}

void runLateralControl(ControlCommand& output,
                       const PathPlan& plan,
                       double maxSteering) {
    output.steering = plan.valid
                          ? static_cast<float>(
                                limitSteering(plan.steering, maxSteering))
                          : 0.0F;
}
