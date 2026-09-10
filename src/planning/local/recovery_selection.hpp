#pragma once

#include "planning/planning_structs.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace recovery_selection {
inline void suspendForTrafficStop(ProgressRecoveryData& state,double x,double y,bool pathBlocked) {
    state.active=false;state.clearTime=0.;state.retryRemaining=0.;
    state.noMotionTime=0.;state.anchorX=x;state.anchorY=y;
    if(!pathBlocked) state.blockedTime=0.;
}

// Keep directional families interleaved even when a frame exhausts its budget.
inline std::vector<std::size_t> balancedOrder(
    const std::array<std::vector<std::size_t>, 3>& families) {
    std::vector<std::size_t> result;
    const auto count = std::max({families[0].size(), families[1].size(), families[2].size()});
    for (std::size_t i = 0; i < count; ++i)
        for (const auto& family : families)
            if (i < family.size()) result.push_back(family[i]);
    return result;
}

struct Score {
    bool contact{};
    bool terminalContact{};
    double cost{};
};

inline bool better(const Score& candidate, const Score& current, bool currentIsIncumbent) {
    // These are priorities, not weights: even a small contact may not beat a
    // clear path because the latter has a larger curvature/route cost.
    if (candidate.contact != current.contact) return !candidate.contact;
    if (candidate.terminalContact != current.terminalContact) return !candidate.terminalContact;
    const double improvement = currentIsIncumbent ? .05 * std::max(1., current.cost) : 0.;
    return candidate.cost < current.cost - improvement;
}
} // namespace recovery_selection
