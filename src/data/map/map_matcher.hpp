#pragma once

#include "data/map/map_structs.hpp"

#include <optional>

// 이전 프레임의 매칭 결과. MapData가 아니라 World process가 소유한다.
struct MapMatchState {
    LaneMatch lastMatch;
    double lastRouteS{};
    bool forceGlobalSearch{true};
};

void resetMapMatchState(MapMatchState& state);
void matchWorldPosition(MapMatchState& state,
                        const PathGeneratorData& map,
                        double x,
                        double y,
                        std::optional<double> heading = std::nullopt);
