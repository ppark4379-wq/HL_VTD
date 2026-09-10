#pragma once
#include "planning/local/lane_change_structs.hpp"
#include <algorithm>
#include <cmath>

// A speed CEILING, never a floor: traffic stops, EBS, following and the
// curvature/steering speed limits are still allowed to request less or zero.
inline double localFallbackSpeedLimit(const LaneChangeConfig& config,
                                     double validatedClearance,
                                     double curvature,
                                     double maximumLateralAcceleration) {
    const double open=std::min(config.localFallbackOpenSpeed,
                               config.maximumLaneChangeSpeed);
    const double slow=std::min(config.localFallbackSpeed,open);
    const bool certified=std::isfinite(validatedClearance) && validatedClearance>=0. &&
        validatedClearance+1e-6>=config.latticeObstacleClearance;
    const bool gentle=std::isfinite(curvature) &&
        std::abs(curvature)*open*open<=maximumLateralAcceleration;
    return certified && gentle ? open : slow;
}
