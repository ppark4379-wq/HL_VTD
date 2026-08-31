#include "data/map/map_matcher.hpp"

#include "planning/global/reference_path_generator.hpp"

void resetMapMatchState(MapMatchState& state) {
    state = {};
    state.forceGlobalSearch = true;
}

void matchWorldPosition(MapMatchState& state,
                        const PathGeneratorData& map,
                        double x,
                        double y,
                        std::optional<double> heading) {
    state.lastMatch = worldToLane(map, x, y, heading);
    state.forceGlobalSearch = !state.lastMatch.valid;
}
