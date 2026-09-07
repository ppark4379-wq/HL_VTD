#include "data/map/map_matcher.hpp"

#include "planning/global/reference_path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kMaximumHeadingError = 1.05;

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

LaneMatch nearestContinuousLane(const PathGeneratorData& map,
                                const LaneKey& previous,
                                double x,
                                double y,
                                std::optional<double> heading) {
    std::vector<LaneKey> keys{previous};
    const auto edges = map.laneGraph.find(previous);
    if (edges != map.laneGraph.end()) {
        for (const LaneKey& next : edges->second) {
            const bool lateral = next.roadId == previous.roadId &&
                next.sectionIndex == previous.sectionIndex &&
                next.laneId != previous.laneId;
            if (!lateral) keys.push_back(next);
        }
    }

    LaneMatch best;
    best.distance = std::numeric_limits<double>::infinity();
    for (const LaneKey& key : keys) {
        const auto segment = map.laneSegments.find(key);
        const auto road = map.roads.find(key.roadId);
        if (segment == map.laneSegments.end() ||
            segment->second.points.empty() || road == map.roads.end() ||
            key.sectionIndex < 0 || key.sectionIndex >=
                static_cast<int>(road->second.sections.size())) {
            continue;
        }
        const LaneSection& section = road->second.sections[key.sectionIndex];
        const auto lane = section.lanes.find(key.laneId);
        if (lane == section.lanes.end() || lane->second.type != "driving") {
            continue;
        }
        const auto nearest = std::min_element(
            segment->second.points.begin(), segment->second.points.end(),
            [&](const Waypoint& left, const Waypoint& right) {
                return std::hypot(left.x - x, left.y - y) <
                       std::hypot(right.x - x, right.y - y);
            });
        const double distance = std::hypot(nearest->x - x, nearest->y - y);
        const double width = laneWidthAt(section, key.laneId, nearest->roadS);
        // 차선 polygon을 정상적으로 다시 잡을 때까지 단지 맵 샘플 이음재만
        // 흡수한다. 실제 차선 밖으로 빠진 경우까지 이전 lane을 유지하지
        // 않도록 반 차폭에 0.75 m만 추가한다.
        if (width <= 0.0 || distance > 0.5 * width + 0.75) continue;
        if (heading.has_value() &&
            std::abs(std::atan2(
                std::sin(nearest->yaw - *heading),
                std::cos(nearest->yaw - *heading))) >
                kMaximumHeadingError) {
            continue;
        }
        if (!best.valid || distance < best.distance) {
            best = {true, key.roadId, key.laneId, key.sectionIndex,
                    nearest->roadS, distance, width,
                    static_cast<std::size_t>(std::distance(
                        segment->second.points.begin(), nearest))};
        }
    }
    return best;
}

bool directlyReachable(const PathGeneratorData& map,
                       const LaneKey& from,
                       const LaneKey& to) {
    if (from == to) return true;
    // 횡방향 edge는 "변경 가능"을 뜻할 뿐, 동일 위치에서 현재 lane이
    // 즉시 바뀌었다는 뜻이 아니다. 두 lane polygon이 경계에서 함께
    // 포함되는 동안 route hint만으로 좌우 lane을 바꾸면 조향 목표도 같이
    // 흔들린다. 실제 차선변경은 이전 lane 영역을 벗어난 뒤 아래 후보
    // 선택에서 확정하고, 여기서는 종방향 successor/junction만 즉시 넘긴다.
    if (from.roadId == to.roadId &&
        from.sectionIndex == to.sectionIndex &&
        from.laneId != to.laneId) {
        return false;
    }
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
                                  kMaximumHeadingError;
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
        if (best != nullptr) {
            state.lastMatch = *best;
        } else {
            const LaneMatch continuous = nearestContinuousLane(
                map, previous, x, y, heading);
            LaneMatch global = worldToLane(map, x, y, heading);
            if (global.valid &&
                headingError(map, global, heading) > kMaximumHeadingError) {
                global.valid = false;
            }
            // 직전 lane/정방향 successor가 이음재 범위에 있으면 임의의
            // 겹침 road로 전역 재매칭하기보다 토폴로지 연속성을 우선한다.
            state.lastMatch = continuous.valid &&
                                      (!global.valid ||
                                       continuous.distance <=
                                           global.distance + 0.35)
                                  ? continuous
                                  : global;
        }
    } else {
        state.lastMatch = worldToLane(map, x, y, heading);
        if (state.lastMatch.valid &&
            headingError(map, state.lastMatch, heading) >
                kMaximumHeadingError) {
            state.lastMatch.valid = false;
        }
    }
    state.forceGlobalSearch = !state.lastMatch.valid;
}
