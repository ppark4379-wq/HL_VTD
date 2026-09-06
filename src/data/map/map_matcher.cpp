#include "data/map/map_matcher.hpp"

#include "planning/global/reference_path_generator.hpp"

#include <algorithm>
#include <cmath>

namespace {

double headingError(const PathGeneratorData& map,
                    const LaneMatch& match,
                    std::optional<double> heading) {
    if (!heading.has_value()) return 0.0;
    const LaneKey key{match.roadId, match.sectionIndex, match.laneId};
    const auto segment = map.laneSegments.find(key);
    if (segment == map.laneSegments.end() || segment->second.points.empty()) {
        return 3.14159265358979323846;
    }
    const Waypoint& point = segment->second.points[
        std::min(match.pointIndex, segment->second.points.size() - 1)];
    return std::abs(std::atan2(std::sin(point.yaw - *heading),
                              std::cos(point.yaw - *heading)));
}

bool sameLane(const LaneMatch& match, const LaneKey& key) {
    return match.valid && match.roadId == key.roadId &&
           match.sectionIndex == key.sectionIndex && match.laneId == key.laneId;
}

const LaneMatch* findLane(const std::vector<LaneMatch>& candidates,
                          const LaneKey& key) {
    const auto found = std::find_if(
        candidates.begin(), candidates.end(),
        [&](const LaneMatch& match) { return sameLane(match, key); });
    return found == candidates.end() ? nullptr : &(*found);
}

bool directlyReachable(const PathGeneratorData& map,
                       const LaneKey& from,
                       const LaneKey& to) {
    if (from == to) return true;
    const auto edges = map.laneGraph.find(from);
    return edges != map.laneGraph.end() &&
           std::find(edges->second.begin(), edges->second.end(), to) !=
               edges->second.end();
}

}  // namespace

void resetMapMatchState(MapMatchState& state) {
    state = {};
    state.forceGlobalSearch = true;
}

void matchWorldPosition(MapMatchState& state,
                        const PathGeneratorData& map,
                        double x,
                        double y,
                        std::optional<double> heading,
                        std::optional<LaneMatch> routePreferred) {
    std::vector<LaneMatch> candidates = findContainingLanes(map, x, y);
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(),
                       [&](const LaneMatch& candidate) {
                           return headingError(map, candidate, heading) >
                                  1.57079632679489661923;
                       }),
        candidates.end());

    const LaneKey previous{state.lastMatch.roadId,
                           state.lastMatch.sectionIndex,
                           state.lastMatch.laneId};
    const LaneMatch* previousCandidate =
        state.lastMatch.valid ? findLane(candidates, previous) : nullptr;

    const LaneMatch* routeCandidate = nullptr;
    LaneKey routeKey;
    if (routePreferred.has_value() && routePreferred->valid) {
        routeKey = {routePreferred->roadId, routePreferred->sectionIndex,
                    routePreferred->laneId};
        routeCandidate = findLane(candidates, routeKey);
    }

    // 접속도로가 겹치는 동안에는 이전 lane을 유지한다. 다만 전역 경로가
    // 이전 lane의 실제 successor로 넘어간 프레임에는 즉시 그 connector로
    // 넘겨, 교차로 진입을 늦추지 않는다.
    if (previousCandidate != nullptr) {
        if (routeCandidate != nullptr && !(routeKey == previous) &&
            directlyReachable(map, previous, routeKey)) {
            state.lastMatch = *routeCandidate;
        } else {
            state.lastMatch = *previousCandidate;
        }
    } else if (routeCandidate != nullptr) {
        state.lastMatch = *routeCandidate;
    } else if (state.lastMatch.valid) {
        const auto edges = map.laneGraph.find(previous);
        const LaneMatch* best = nullptr;
        if (edges != map.laneGraph.end()) {
            for (const LaneKey& next : edges->second) {
                const LaneMatch* candidate = findLane(candidates, next);
                if (candidate != nullptr &&
                    (best == nullptr || candidate->distance < best->distance)) {
                    best = candidate;
                }
            }
        }
        state.lastMatch = best != nullptr
                              ? *best
                              : worldToLane(map, x, y, heading);
    } else {
        state.lastMatch = worldToLane(map, x, y, heading);
    }
    state.forceGlobalSearch = !state.lastMatch.valid;
}
