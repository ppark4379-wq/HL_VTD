#include "planning/global/mission_corridor.hpp"

#include "planning/global/reference_path_generator.hpp"
#include "utils/math_utils.hpp"
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/property_tree/json_parser.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <tuple>

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

double checkpointHeading(const std::vector<Checkpoint>& checkpoints,
                         std::size_t index) {
    if (checkpoints.size() < 2) return 0.0;
    if (index == 0) {
        return std::atan2(checkpoints[1].y - checkpoints[0].y,
                          checkpoints[1].x - checkpoints[0].x);
    }
    if (index + 1 == checkpoints.size()) {
        return std::atan2(checkpoints[index].y - checkpoints[index - 1].y,
                          checkpoints[index].x - checkpoints[index - 1].x);
    }
    // 교차로 진입/진출 checkpoint에서는 바로 다음 점만 보면 회전 후
    // 방향으로 오매칭되기 쉽다. 앞뒤 점을 잇는 중앙차분으로 접근방향을 쓴다.
    return std::atan2(checkpoints[index + 1].y - checkpoints[index - 1].y,
                      checkpoints[index + 1].x - checkpoints[index - 1].x);
}

double laneHeadingError(const PathGeneratorData& map,
                        const LaneMatch& match,
                        double heading) {
    const LaneKey key{match.roadId, match.sectionIndex, match.laneId};
    const auto segment = map.laneSegments.find(key);
    if (segment == map.laneSegments.end() || segment->second.points.empty()) {
        return 3.14159265358979323846;
    }
    const std::size_t index = std::min(
        match.pointIndex, segment->second.points.size() - 1);
    return std::abs(normalizeAngle(
        segment->second.points[index].yaw - heading));
}

bool isDrivingLane(const PathGeneratorData& map, const LaneKey& key) {
    const auto road = map.roads.find(key.roadId);
    if (road == map.roads.end() || key.sectionIndex < 0 ||
        key.sectionIndex >= static_cast<int>(road->second.sections.size())) {
        return false;
    }
    const auto lane =
        road->second.sections[key.sectionIndex].lanes.find(key.laneId);
    return key.laneId != 0 &&
           lane != road->second.sections[key.sectionIndex].lanes.end() &&
           lane->second.type == "driving";
}

MissionCheckpointData makeCheckpointData(const PathGeneratorData& map,
                                         std::size_t checkpointIndex) {
    MissionCheckpointData output;
    output.checkpoint = map.checkpoints[checkpointIndex];
    const double heading = checkpointHeading(map.checkpoints, checkpointIndex);
    std::vector<LaneMatch> candidates = findContainingLanes(
        map, output.checkpoint.x, output.checkpoint.y);
    if (candidates.empty()) {
        LaneMatch nearest = worldToLane(
            map, output.checkpoint.x, output.checkpoint.y, heading);
        const LaneKey nearestKey{
            nearest.roadId, nearest.sectionIndex, nearest.laneId};
        // 운영측 좌표는 교차로 시작/끝의 대략적인 점이라 차선 폭 바깥에
        // 놓일 수 있다. worldToLane의 엄격한 "차선 내부" valid 여부와
        // 별개로, 15m 안의 실제 driving lane이면 road 구역 힌트로는 쓴다.
        if (map.laneSegments.count(nearestKey) &&
            nearest.distance <= 15.0 && isDrivingLane(map, nearestKey)) {
            nearest.valid = true;
            candidates.push_back(nearest);
        }
    }
    if (candidates.empty()) return output;

    double bestScore = kInfinity;
    for (const LaneMatch& candidate : candidates) {
        const double error = laneHeadingError(map, candidate, heading);
        bestScore = std::min(
            bestScore, candidate.distance * 4.0 + error * error * 12.0);
    }

    // 겹치는 connector road 중 진행방향과 위치가 사실상 같은 후보는 모두
    // 남긴다. 런타임 map matching이 둘 중 어느 ID를 고르더라도 같은 mission
    // gateway로 처리할 수 있다.
    struct RoadSectionDirection {
        int roadId{};
        int sectionIndex{};
        bool positive{};
        bool operator<(const RoadSectionDirection& other) const {
            if (roadId != other.roadId) return roadId < other.roadId;
            if (sectionIndex != other.sectionIndex) {
                return sectionIndex < other.sectionIndex;
            }
            return positive < other.positive;
        }
    };
    std::set<RoadSectionDirection> selectedRoadDirections;
    for (const LaneMatch& candidate : candidates) {
        const double error = laneHeadingError(map, candidate, heading);
        const double score = candidate.distance * 4.0 + error * error * 12.0;
        if (error > 1.35 || score > bestScore + 12.0 ||
            candidate.distance > candidates.front().distance + 3.0) {
            continue;
        }
        selectedRoadDirections.insert({candidate.roadId,
                                       candidate.sectionIndex,
                                       candidate.laneId > 0});
    }
    if (selectedRoadDirections.empty()) {
        const auto best = std::min_element(
            candidates.begin(), candidates.end(),
            [&](const LaneMatch& left, const LaneMatch& right) {
                const double leftError = laneHeadingError(map, left, heading);
                const double rightError = laneHeadingError(map, right, heading);
                return left.distance * 4.0 + leftError * leftError * 12.0 <
                       right.distance * 4.0 + rightError * rightError * 12.0;
            });
        selectedRoadDirections.insert({best->roadId, best->sectionIndex,
                                       best->laneId > 0});
    }

    for (const RoadSectionDirection& selected : selectedRoadDirections) {
        output.roadIds.push_back(selected.roadId);
        for (const auto& [key, segment] : map.laneSegments) {
            (void)segment;
            if (key.roadId == selected.roadId &&
                key.sectionIndex == selected.sectionIndex &&
                key.laneId != 0 &&
                (key.laneId > 0) == selected.positive &&
                isDrivingLane(map, key)) {
                output.lanes.insert(key);
            }
        }
    }
    std::sort(output.roadIds.begin(), output.roadIds.end());
    output.roadIds.erase(
        std::unique(output.roadIds.begin(), output.roadIds.end()),
        output.roadIds.end());
    return output;
}

bool isLateralEdge(const LaneKey& from, const LaneKey& to) {
    return from.roadId == to.roadId &&
           from.sectionIndex == to.sectionIndex &&
           from.laneId != to.laneId;
}

double edgeCost(const PathGeneratorData& map,
                const LaneKey& from,
                const LaneKey& to) {
    if (isLateralEdge(from, to)) {
        // 차선변경은 가능하지만 종방향 successor보다 비싸게 두어, corridor가
        // 불필요하게 도로 전체의 왕복 lateral 조합으로 퍼지는 것을 막는다.
        return 18.0 + 4.0 * std::abs(to.laneId - from.laneId);
    }
    const auto segment = map.laneSegments.find(to);
    return segment == map.laneSegments.end()
               ? kInfinity
               : std::max(1.0, segment->second.length);
}

using DistanceMap = std::map<LaneKey, double>;

bool findRestrictedRoute(const PathGeneratorData& map,
                         const std::set<LaneKey>& allowed,
                         const std::set<LaneKey>& targets,
                         const LaneKey& start,
                         std::vector<LaneKey>& route) {
    route.clear();
    if (!map.laneSegments.count(start) ||
        (!allowed.count(start) && !targets.count(start))) {
        return false;
    }
    struct Entry {
        double cost{};
        LaneKey key;
    };
    const auto farther = [](const Entry& left, const Entry& right) {
        return left.cost > right.cost;
    };
    std::priority_queue<Entry, std::vector<Entry>, decltype(farther)> pending(
        farther);
    DistanceMap costs;
    std::map<LaneKey, LaneKey> previous;
    costs[start] = 0.0;
    pending.push({0.0, start});
    LaneKey goal;
    bool found = targets.count(start) != 0;
    if (found) goal = start;

    while (!found && !pending.empty()) {
        const Entry entry = pending.top();
        pending.pop();
        if (entry.cost > costs[entry.key] + 1e-9) continue;
        const auto edges = map.laneGraph.find(entry.key);
        if (edges == map.laneGraph.end()) continue;
        for (const LaneKey& next : edges->second) {
            if ((!allowed.count(next) && !targets.count(next)) ||
                !map.laneSegments.count(next)) {
                continue;
            }
            const double candidate = entry.cost + edgeCost(map, entry.key, next);
            const auto known = costs.find(next);
            if (known != costs.end() && candidate >= known->second - 1e-9) {
                continue;
            }
            costs[next] = candidate;
            previous[next] = entry.key;
            if (targets.count(next)) {
                goal = next;
                found = true;
                break;
            }
            pending.push({candidate, next});
        }
    }
    if (!found) return false;
    for (LaneKey current = goal;; current = previous.at(current)) {
        route.push_back(current);
        if (current == start) break;
    }
    std::reverse(route.begin(), route.end());
    return true;
}

DistanceMap shortestDistances(
    const PathGeneratorData& map,
    const std::set<LaneKey>& sources,
    bool reverse,
    std::map<LaneKey, LaneKey>* previous = nullptr) {
    struct Entry {
        double cost{};
        LaneKey key;
    };
    const auto farther = [](const Entry& left, const Entry& right) {
        return left.cost > right.cost;
    };
    std::priority_queue<Entry, std::vector<Entry>, decltype(farther)> queue(
        farther);
    DistanceMap distances;
    for (const LaneKey& source : sources) {
        if (!map.laneSegments.count(source)) continue;
        distances[source] = 0.0;
        queue.push({0.0, source});
    }

    std::map<LaneKey, std::vector<LaneKey>> reverseGraph;
    if (reverse) {
        for (const auto& [from, targets] : map.laneGraph) {
            for (const LaneKey& to : targets) reverseGraph[to].push_back(from);
        }
    }

    while (!queue.empty()) {
        const Entry entry = queue.top();
        queue.pop();
        if (entry.cost > distances[entry.key] + 1e-9) continue;
        const auto neighbors = reverse ? reverseGraph.find(entry.key)
                                       : map.laneGraph.find(entry.key);
        if (neighbors == (reverse ? reverseGraph.end()
                                  : map.laneGraph.end())) {
            continue;
        }
        for (const LaneKey& next : neighbors->second) {
            const LaneKey& originalFrom = reverse ? next : entry.key;
            const LaneKey& originalTo = reverse ? entry.key : next;
            const double step = edgeCost(map, originalFrom, originalTo);
            if (!std::isfinite(step)) continue;
            const double candidate = entry.cost + step;
            const auto known = distances.find(next);
            if (known != distances.end() &&
                candidate >= known->second - 1e-9) {
                continue;
            }
            distances[next] = candidate;
            if (previous != nullptr && !reverse) (*previous)[next] = entry.key;
            queue.push({candidate, next});
        }
    }
    return distances;
}

MissionLegData makeSimpleMissionLeg(const PathGeneratorData& map,
                                    const MissionCheckpointData& from,
                                    const MissionCheckpointData& to,
                                    std::size_t fromIndex,
                                    std::size_t gatewayIndex,
                                    std::size_t toIndex) {
    MissionLegData leg;
    leg.fromCheckpoint = fromIndex;
    leg.gatewayCheckpoint = gatewayIndex;
    leg.toCheckpoint = toIndex;
    leg.targetLanes = to.lanes;
    if (from.lanes.empty() || to.lanes.empty()) return leg;

    std::map<LaneKey, LaneKey> previous;
    const DistanceMap forward = shortestDistances(
        map, from.lanes, false, &previous);
    const DistanceMap backward = shortestDistances(map, to.lanes, true);
    LaneKey bestTarget;
    bool foundTarget = false;
    leg.bestCost = kInfinity;
    for (const LaneKey& target : to.lanes) {
        const auto distance = forward.find(target);
        if (distance != forward.end() && distance->second < leg.bestCost) {
            leg.bestCost = distance->second;
            bestTarget = target;
            foundTarget = true;
        }
    }
    if (!foundTarget) return leg;

    // 최단거리의 25% 또는 35m까지 대안을 허용한다. 장애물 때문에 기본
    // 차선을 못 쓰는 경우에도 합리적인 우회 branch는 굵은 회랑에 남지만,
    // 지도 전체를 돌아오는 큰 detour는 포함되지 않는다.
    const double allowance = std::max(35.0, leg.bestCost * 0.25);
    for (const auto& [key, fromCost] : forward) {
        const auto toCost = backward.find(key);
        if (toCost != backward.end() &&
            fromCost + toCost->second <= leg.bestCost + allowance) {
            leg.allowedLanes.insert(key);
        }
    }
    // 회랑에 선택된 road/section에서는 특정 전역경로 lane만 허용하지
    // 않는다. 같은 방향의 driving lane이고 시작에서 접근할 수 있으며
    // exit로도 나갈 수 있으면, 차선이 늘거나 줄어드는 section을 포함해
    // 전부 로컬 장애물 회피 후보로 포함한다.
    struct RoadSectionDirection {
        int roadId{};
        int sectionIndex{};
        bool positive{};
        bool operator<(const RoadSectionDirection& other) const {
            if (roadId != other.roadId) return roadId < other.roadId;
            if (sectionIndex != other.sectionIndex) {
                return sectionIndex < other.sectionIndex;
            }
            return positive < other.positive;
        }
    };
    // 경계 checkpoint의 lane도 먼저 corridor seed에 넣어, 해당 section의
    // 다른 정상 진행 차선이 roadMark 때문에 빠지지 않게 한다.
    leg.allowedLanes.insert(from.lanes.begin(), from.lanes.end());
    leg.allowedLanes.insert(to.lanes.begin(), to.lanes.end());
    std::set<RoadSectionDirection> corridorSections;
    for (const LaneKey& key : leg.allowedLanes) {
        corridorSections.insert(
            {key.roadId, key.sectionIndex, key.laneId > 0});
    }
    for (const auto& [key, segment] : map.laneSegments) {
        (void)segment;
        if (!corridorSections.count(
                {key.roadId, key.sectionIndex, key.laneId > 0}) ||
            !isDrivingLane(map, key)) {
            continue;
        }
        // corridor는 도로의 사용 가능 영역이지 지금 당장 횡이동할 수
        // 있다는 뜻이 아니다. 따라서 section 전체가 실선이어도 같은 방향
        // driving lane은 corridor에 포함한다. 실선/점선은 실제 후보가
        // 경계를 통과하는 정확한 road_s에서만 검사한다.
        leg.allowedLanes.insert(key);
    }
    for (LaneKey current = bestTarget;;) {
        leg.preferredRoute.push_back(current);
        const auto prior = previous.find(current);
        if (prior == previous.end()) break;
        current = prior->second;
    }
    std::reverse(leg.preferredRoute.begin(), leg.preferredRoute.end());
    for (const LaneKey& lane : leg.allowedLanes) {
        std::vector<LaneKey> route;
        if (findRestrictedRoute(
                map, leg.allowedLanes, leg.targetLanes, lane, route)) {
            leg.routesToTargetByLane.emplace(lane, std::move(route));
        }
    }
    leg.valid = !leg.preferredRoute.empty();
    return leg;
}

// 교영측이 제공한 쌍은 [교차로 진입, 교차로 진출]이다.
// 진입점에 다 온 뒤 진출 방향을 고르면 이미 분기가 끝날 수
// 있다. 따라서 from -> gateway -> exit를 하나의 제한된 leg로
// 묶고, gateway를 실제로 통과하는 lane chain만 미리 cache한다.
MissionLegData makeMissionLeg(const PathGeneratorData& map,
                              const MissionCheckpointData& from,
                              const MissionCheckpointData& gateway,
                              const MissionCheckpointData& to,
                              std::size_t fromIndex,
                              std::size_t gatewayIndex,
                              std::size_t toIndex) {
    if (gatewayIndex == toIndex) {
        MissionLegData leg = makeSimpleMissionLeg(
            map, from, to, fromIndex, gatewayIndex, toIndex);
        leg.gatewayBestCost = leg.bestCost;
        leg.exitBestCost = 0.0;
        return leg;
    }

    const MissionLegData approach = makeSimpleMissionLeg(
        map, from, gateway, fromIndex, gatewayIndex, gatewayIndex);
    const MissionLegData crossing = makeSimpleMissionLeg(
        map, gateway, to, gatewayIndex, toIndex, toIndex);
    MissionLegData leg;
    leg.fromCheckpoint = fromIndex;
    leg.gatewayCheckpoint = gatewayIndex;
    leg.toCheckpoint = toIndex;
    leg.targetLanes = to.lanes;
    leg.gatewayBestCost = approach.bestCost;
    leg.exitBestCost = crossing.bestCost;
    leg.bestCost = approach.bestCost + crossing.bestCost;
    if (!approach.valid || !crossing.valid ||
        !std::isfinite(leg.bestCost)) {
        return leg;
    }
    leg.allowedLanes = approach.allowedLanes;
    leg.allowedLanes.insert(crossing.allowedLanes.begin(),
                            crossing.allowedLanes.end());

    // gateway 후보 차선별로 exit suffix를 고정한다. approach의
    // 끝 차선과 서로 다른 crossing preferredRoute를 이어 붙이면
    // 교차로 안에서 경로가 순간이동하므로 반드시 동일 gateway
    // lane에서 시작하는 suffix만 사용한다.
    const auto appendSuffix = [&](std::vector<LaneKey> prefix,
                                  const std::vector<LaneKey>& suffix) {
        if (prefix.empty() || suffix.empty() ||
            !(prefix.back() == suffix.front())) {
            return std::vector<LaneKey>{};
        }
        prefix.insert(prefix.end(), std::next(suffix.begin()), suffix.end());
        return prefix;
    };
    // gateway를 이미 통과한 차선에서는 crossing suffix를 바로
    // 쓰고, gateway 이전 차선은 approach cache + 동일 gateway
    // suffix로 구성한다. 그래프상 gateway를 우회하는 경로는
    // cache에 들어올 수 없다.
    for (const auto& [lane, suffix] : crossing.routesToTargetByLane) {
        leg.routesToTargetByLane[lane] = suffix;
    }
    std::set<LaneKey> usableGateways;
    for (const LaneKey& lane : gateway.lanes) {
        if (crossing.routesToTargetByLane.count(lane)) {
            usableGateways.insert(lane);
        }
    }
    if (usableGateways.empty()) return leg;
    // approach의 단순 최단 gateway가 진출로 연결되지 않을 수
    // 있다. 반드시 exit suffix가 있는 gateway lane만 목표로 다시
    // 탐색해 진입과 진출이 동일 chain이 되게 한다.
    for (const LaneKey& lane : approach.allowedLanes) {
        std::vector<LaneKey> prefix;
        if (!findRestrictedRoute(map, approach.allowedLanes,
                                 usableGateways, lane, prefix) ||
            prefix.empty()) {
            continue;
        }
        const auto suffix = crossing.routesToTargetByLane.find(prefix.back());
        if (suffix == crossing.routesToTargetByLane.end()) continue;
        std::vector<LaneKey> route = appendSuffix(prefix, suffix->second);
        if (!route.empty()) {
            leg.routesToTargetByLane[lane] = std::move(route);
        }
    }
    double preferredCost = kInfinity;
    for (const LaneKey& lane : from.lanes) {
        const auto route = leg.routesToTargetByLane.find(lane);
        if (route == leg.routesToTargetByLane.end() || route->second.empty()) {
            continue;
        }
        double cost = 0.0;
        for (std::size_t index = 1; index < route->second.size(); ++index) {
            cost += edgeCost(map, route->second[index - 1],
                             route->second[index]);
        }
        if (cost < preferredCost) {
            preferredCost = cost;
            leg.preferredRoute = route->second;
        }
    }
    leg.valid = !leg.preferredRoute.empty() &&
        leg.targetLanes.count(leg.preferredRoute.back()) != 0;
    return leg;
}

bool pruneCheckpointLanesToContinuousMission(
    const PathGeneratorData& map,
    std::vector<MissionCheckpointData>& checkpoints) {
    if (checkpoints.size() < 2) return false;
    std::vector<std::set<LaneKey>> forward(checkpoints.size());
    std::vector<std::set<LaneKey>> backward(checkpoints.size());
    forward.front() = checkpoints.front().lanes;
    for (std::size_t index = 1; index < checkpoints.size(); ++index) {
        const DistanceMap reachable = shortestDistances(
            map, forward[index - 1], false);
        for (const LaneKey& lane : checkpoints[index].lanes) {
            if (reachable.count(lane)) forward[index].insert(lane);
        }
        if (forward[index].empty()) return false;
    }

    backward.back() = checkpoints.back().lanes;
    for (std::size_t reverseIndex = checkpoints.size() - 1;
         reverseIndex > 0; --reverseIndex) {
        const std::size_t index = reverseIndex - 1;
        const DistanceMap canReachNext = shortestDistances(
            map, backward[index + 1], true);
        for (const LaneKey& lane : checkpoints[index].lanes) {
            if (canReachNext.count(lane)) backward[index].insert(lane);
        }
        if (backward[index].empty()) return false;
    }

    for (std::size_t index = 0; index < checkpoints.size(); ++index) {
        std::set<LaneKey> viable;
        std::set_intersection(
            forward[index].begin(), forward[index].end(),
            backward[index].begin(), backward[index].end(),
            std::inserter(viable, viable.end()));
        if (viable.empty()) return false;
        checkpoints[index].lanes = std::move(viable);
        checkpoints[index].roadIds.clear();
        for (const LaneKey& lane : checkpoints[index].lanes) {
            checkpoints[index].roadIds.push_back(lane.roadId);
        }
        std::sort(checkpoints[index].roadIds.begin(),
                  checkpoints[index].roadIds.end());
        checkpoints[index].roadIds.erase(
            std::unique(checkpoints[index].roadIds.begin(),
                        checkpoints[index].roadIds.end()),
            checkpoints[index].roadIds.end());
    }
    return true;
}

// 대회 checkpoint는 시작, [교차로 진입, 진출] 반복, 종료로
// 주어진다. 진입/진출을 각각 별도 leg로 두면 진입점에
// 다른 후에야 회전 방향을 선택하게 된다. 진출 checkpoint를
// anchor로 두되, 사이 진입 checkpoint는 makeMissionLeg에서
// 반드시 거치는 gateway로 제약한다.
std::vector<std::size_t> missionAnchorIndices(std::size_t checkpointCount) {
    std::vector<std::size_t> result;
    if (checkpointCount == 0) return result;
    result.push_back(0);
    for (std::size_t index = 2; index < checkpointCount; index += 2) {
        result.push_back(index);
    }
    if (result.back() != checkpointCount - 1) {
        result.push_back(checkpointCount - 1);
    }
    return result;
}

bool guideLaneKey(const PathGeneratorData& map, const Waypoint& point,
                  LaneKey& key) {
    const auto road = map.roads.find(point.roadId);
    if (road == map.roads.end() || road->second.sections.empty() ||
        point.laneId == 0 || point.roadS < -1e-6 ||
        point.roadS > road->second.length + 1e-6) {
        return false;
    }
    int sectionIndex = 0;
    for (std::size_t index = 0;
         index < road->second.sections.size(); ++index) {
        if (road->second.sections[index].s <= point.roadS + 1e-6) {
            sectionIndex = static_cast<int>(index);
        } else {
            break;
        }
    }
    key = LaneKey{point.roadId, sectionIndex, point.laneId};
    return map.laneSegments.count(key) != 0 && isDrivingLane(map, key);
}

std::vector<std::size_t> matchCheckpointsToGuide(
    const MissionCorridorData& mission,
    const std::vector<Waypoint>& guide) {
    std::vector<std::size_t> result;
    if (guide.empty()) return result;
    result.reserve(mission.checkpoints.size());
    std::size_t begin = 0;
    for (const auto& checkpoint : mission.checkpoints) {
        std::size_t best = begin;
        double bestDistance = kInfinity;
        bool foundMatchingRoad = false;
        for (std::size_t index = begin; index < guide.size(); ++index) {
            const bool matchingRoad = std::find(
                checkpoint.roadIds.begin(), checkpoint.roadIds.end(),
                guide[index].roadId) != checkpoint.roadIds.end();
            if (foundMatchingRoad && !matchingRoad) continue;
            const double distance = std::hypot(
                guide[index].x - checkpoint.checkpoint.x,
                guide[index].y - checkpoint.checkpoint.y);
            if ((matchingRoad && !foundMatchingRoad) ||
                distance < bestDistance) {
                foundMatchingRoad = matchingRoad;
                bestDistance = distance;
                best = index;
            }
        }
        result.push_back(best);
        begin = best;
    }
    return result;
}

MissionLegData makeGuideConstrainedMissionLeg(
    const PathGeneratorData& map,
    const MissionCheckpointData& from,
    const MissionCheckpointData& gateway,
    const MissionCheckpointData& to,
    std::size_t fromIndex,
    std::size_t gatewayIndex,
    std::size_t toIndex,
    const std::vector<Waypoint>& guide,
    const std::vector<std::size_t>& checkpointGuideIndices) {
    MissionLegData leg;
    (void)from;
    (void)gateway;
    leg.fromCheckpoint = fromIndex;
    leg.gatewayCheckpoint = gatewayIndex;
    leg.toCheckpoint = toIndex;
    leg.guideRouteAuthority = true;
    if (checkpointGuideIndices.size() <= toIndex || guide.empty()) return leg;

    const std::size_t begin = std::min(
        checkpointGuideIndices[fromIndex], guide.size() - 1);
    const std::size_t gatewayPoint = std::min(
        checkpointGuideIndices[gatewayIndex], guide.size() - 1);
    const std::size_t end = std::min(
        checkpointGuideIndices[toIndex], guide.size() - 1);
    if (begin > gatewayPoint || gatewayPoint > end) return leg;

    std::set<std::tuple<int, int, bool>> guideSections;
    LaneKey previous;
    bool havePrevious = false;
    for (std::size_t index = begin; index <= end; ++index) {
        LaneKey key;
        if (!guideLaneKey(map, guide[index], key)) continue;
        guideSections.emplace(key.roadId, key.sectionIndex, key.laneId > 0);
        if (!havePrevious || !(previous == key)) {
            leg.preferredRoute.push_back(key);
            previous = key;
            havePrevious = true;
        }
    }
    if (leg.preferredRoute.empty()) return leg;

    // A corridor is the set of same-direction driving lanes alongside the
    // official route, not every graph node that can eventually reach a
    // checkpoint.  This removes graph-error detours while retaining all
    // parallel lanes for local obstacle avoidance.
    for (const auto& [key, segment] : map.laneSegments) {
        (void)segment;
        if (guideSections.count(
                {key.roadId, key.sectionIndex, key.laneId > 0}) &&
            isDrivingLane(map, key)) {
            leg.allowedLanes.insert(key);
        }
    }

    for (const LaneKey& lane : to.lanes) {
        if (leg.allowedLanes.count(lane)) leg.targetLanes.insert(lane);
    }
    if (leg.targetLanes.empty()) {
        const LaneKey finalGuideLane = leg.preferredRoute.back();
        for (const LaneKey& lane : leg.allowedLanes) {
            if (lane.roadId == finalGuideLane.roadId &&
                lane.sectionIndex == finalGuideLane.sectionIndex &&
                (lane.laneId > 0) == (finalGuideLane.laneId > 0)) {
                leg.targetLanes.insert(lane);
            }
        }
    }
    if (leg.targetLanes.empty()) {
        leg.targetLanes.insert(leg.preferredRoute.back());
        leg.allowedLanes.insert(leg.preferredRoute.back());
    }

    const auto arcDistance = [&](std::size_t first, std::size_t last) {
        double distance = 0.0;
        for (std::size_t index = first; index < last; ++index) {
            distance += std::hypot(guide[index + 1].x - guide[index].x,
                                   guide[index + 1].y - guide[index].y);
        }
        return distance;
    };
    leg.gatewayBestCost = arcDistance(begin, gatewayPoint);
    leg.exitBestCost = arcDistance(gatewayPoint, end);
    leg.bestCost = leg.gatewayBestCost + leg.exitBestCost;

    for (const LaneKey& lane : leg.allowedLanes) {
        std::vector<LaneKey> route;
        if (findRestrictedRoute(map, leg.allowedLanes,
                                leg.targetLanes, lane, route)) {
            leg.routesToTargetByLane.emplace(lane, std::move(route));
        }
    }
    leg.valid = true;
    return leg;
}

void overlapMissionLegHandoffs(const PathGeneratorData& map,
                               std::vector<MissionLegData>& legs) {
    // Checkpoint coordinates are approximate road gates.  A vehicle may be
    // accepted on the exit road several metres before the sampled guide
    // changes its road id.  Make adjacent legs share the complete directed
    // lane surface at that gate so advancing the mission can never remove the
    // lane currently under the vehicle.
    for (std::size_t index = 1; index < legs.size(); ++index) {
        const MissionLegData& previous = legs[index - 1];
        MissionLegData& current = legs[index];
        std::set<std::tuple<int, int, bool>> targetSections;
        for (const LaneKey& lane : previous.targetLanes) {
            targetSections.emplace(lane.roadId, lane.sectionIndex,
                                   lane.laneId > 0);
            current.allowedLanes.insert(lane);
        }
        for (const LaneKey& lane : previous.allowedLanes) {
            if (targetSections.count(
                    {lane.roadId, lane.sectionIndex, lane.laneId > 0})) {
                current.allowedLanes.insert(lane);
            }
        }

        // The added handoff lanes are useful only when the restricted XODR
        // graph can reach this leg's gateway. Cache those routes exactly like
        // the rest of the precomputed corridor. A genuine topology gap stays
        // uncached and is handled by the guide-continuity connector.
        current.routesToTargetByLane.clear();
        for (const LaneKey& lane : current.allowedLanes) {
            std::vector<LaneKey> route;
            if (findRestrictedRoute(map, current.allowedLanes,
                                    current.targetLanes, lane, route)) {
                current.routesToTargetByLane.emplace(lane, std::move(route));
            }
        }
    }
}

bool containsRoad(const MissionCheckpointData& checkpoint, int roadId) {
    return std::find(checkpoint.roadIds.begin(), checkpoint.roadIds.end(),
                     roadId) != checkpoint.roadIds.end();
}

bool laneInSet(const std::set<LaneKey>& lanes, const LaneKey& key) {
    return lanes.find(key) != lanes.end();
}

}  // namespace

bool initializeMissionCorridor(MissionCorridorData& output,
                               PathGeneratorData& map,
                               const std::string& checkpointCsv,
                               const std::vector<Waypoint>* guidePath) {
    output = {};
    output.initialized = true;
    output.minimumTargetDistance = kInfinity;
    if (!loadCheckpointsCsv(map, checkpointCsv) || map.checkpoints.size() < 2) {
        return false;
    }

    output.checkpoints.reserve(map.checkpoints.size());
    for (std::size_t index = 0; index < map.checkpoints.size(); ++index) {
        MissionCheckpointData checkpoint = makeCheckpointData(map, index);
        if (checkpoint.roadIds.empty() || checkpoint.lanes.empty()) {
            return false;
        }
        output.checkpoints.push_back(std::move(checkpoint));
    }
    // 좌표는 임의 피팅이므로 정확한 점/차로를 강제하지 않는다. 다만 각
    // seq가 나타내는 directed road 자체는 생략할 수 없으므로 모든 anchor를
    // 앞뒤 reachability로 줄여 전체 mission을 연속적으로 연결한다.
    const std::vector<std::size_t> anchors = missionAnchorIndices(
        output.checkpoints.size());
    std::vector<MissionCheckpointData> anchorCheckpoints;
    anchorCheckpoints.reserve(anchors.size());
    for (const std::size_t index : anchors) {
        anchorCheckpoints.push_back(output.checkpoints[index]);
    }
    if (!pruneCheckpointLanesToContinuousMission(map, anchorCheckpoints)) {
        std::cerr << "[mission] checkpoint road candidates do not form one "
                     "continuous directed mission\n";
        return false;
    }
    for (std::size_t index = 0; index < anchors.size(); ++index) {
        output.checkpoints[anchors[index]].roadIds =
            std::move(anchorCheckpoints[index].roadIds);
        output.checkpoints[anchors[index]].lanes =
            std::move(anchorCheckpoints[index].lanes);
    }

    for (const MissionCheckpointData& checkpoint : output.checkpoints) {
        std::ostringstream roads;
        for (std::size_t index = 0; index < checkpoint.roadIds.size(); ++index) {
            if (index > 0) roads << '/';
            roads << checkpoint.roadIds[index];
        }
        std::cout << "[mission] seq=" << checkpoint.checkpoint.sequence
                  << " road=" << roads.str()
                  << " lane_candidates=" << checkpoint.lanes.size() << '\n';
    }

    output.legs.reserve(anchors.size() - 1);
    const std::vector<std::size_t> checkpointGuideIndices =
        guidePath != nullptr && !guidePath->empty()
            ? matchCheckpointsToGuide(output, *guidePath)
            : std::vector<std::size_t>{};
    for (std::size_t index = 0; index + 1 < anchors.size(); ++index) {
        const std::size_t fromIndex = anchors[index];
        const std::size_t toIndex = anchors[index + 1];
        const std::size_t gatewayIndex = fromIndex + 1 < toIndex
            ? fromIndex + 1 : toIndex;
        MissionLegData leg = guidePath != nullptr &&
                                     !checkpointGuideIndices.empty()
            ? makeGuideConstrainedMissionLeg(
                  map, output.checkpoints[fromIndex],
                  output.checkpoints[gatewayIndex],
                  output.checkpoints[toIndex], fromIndex, gatewayIndex,
                  toIndex, *guidePath, checkpointGuideIndices)
            : makeMissionLeg(
                  map, output.checkpoints[fromIndex],
                  output.checkpoints[gatewayIndex],
                  output.checkpoints[toIndex], fromIndex, gatewayIndex,
                  toIndex);
        if (!leg.valid) {
            std::cerr << "[mission] checkpoint "
                      << output.checkpoints[fromIndex].checkpoint.sequence
                      << " -> "
                      << output.checkpoints[toIndex].checkpoint.sequence
                      << " has no directed lane-graph corridor\n";
            return false;
        }
        std::cout << "[mission] corridor "
                  << output.checkpoints[fromIndex].checkpoint.sequence
                  << " -> gateway "
                  << output.checkpoints[gatewayIndex].checkpoint.sequence
                  << " -> exit "
                  << output.checkpoints[toIndex].checkpoint.sequence
                  << " free_lanes=" << leg.allowedLanes.size()
                  << " shortest_m=" << leg.bestCost << '\n';
        output.legs.push_back(std::move(leg));
    }
    overlapMissionLegHandoffs(map, output.legs);
    output.valid = !output.legs.empty();
    return output.valid;
}

bool missionPointIsTarget(const MissionLegData& leg, const Waypoint& point) {
    const LaneKey key{point.roadId, point.sectionIndex, point.laneId};
    if (!leg.targetLanes.count(key)) return false;
    const auto station = leg.targetRoadS.find(key);
    if (station == leg.targetRoadS.end()) return !leg.topologyAuthority;
    if (!std::isfinite(point.roadS)) return false;
    return key.laneId < 0 ? point.roadS >= station->second - .25
                          : point.roadS <= station->second + .25;
}

bool loadPrecomputedMission(MissionCorridorData& output,
                            const PathGeneratorData& map,
                            const std::string& artifact, std::string* error) {
    output = {};
    try {
        using boost::property_tree::ptree;
        ptree document;
        boost::property_tree::read_json(artifact, document);
        if (document.get<int>("runtime_schema_version") != 1 ||
            document.get<std::string>("mode") != "whole_mission_xodr_precomputed" ||
            !document.get<bool>("initialized_ok"))
            throw std::runtime_error("unsupported/uninitialized mission artifact");
        for (const auto* field : {"checkpoint_rules_valid", "ordered_lane_graph_connected",
                                  "gateway_interfaces_connected", "surface_interfaces_connected"})
            if (!document.get<bool>(std::string("validation.") + field))
                throw std::runtime_error(std::string("mission validation failed: ") + field);
        const auto key = [&](const ptree& item) {
            if (item.size() != 3) throw std::runtime_error("invalid lane key length");
            auto it = item.begin();
            const int road = (it++)->second.get_value<int>();
            const int section = (it++)->second.get_value<int>();
            const int lane = it->second.get_value<int>();
            LaneKey k{road, section, lane};
            if (!isDrivingLane(map, k) || !map.laneSegments.count(k))
                throw std::runtime_error("artifact lane is absent from loaded XODR");
            return k;
        };
        MissionCorridorData loaded;
        for (const auto& item : document.get_child("checkpoints")) {
            MissionCheckpointData cp;
            cp.checkpoint.sequence = item.second.get<int>("seq");
            cp.checkpoint.x = item.second.get<double>("x");
            cp.checkpoint.y = item.second.get<double>("y");
            if (cp.checkpoint.sequence != static_cast<int>(loaded.checkpoints.size()+1) ||
                !std::isfinite(cp.checkpoint.x) || !std::isfinite(cp.checkpoint.y))
                throw std::runtime_error("invalid checkpoint sequence/coordinate");
            loaded.checkpoints.push_back(cp);
        }
        if (loaded.checkpoints.size() < 2 || loaded.checkpoints.size()%2)
            throw std::runtime_error("mission needs start, entry/exit pairs, finish");
        for (const auto& item : document.get_child("checkpoint_matches")) {
            const auto& match = item.second;
            auto& cp = loaded.checkpoints.at(match.get<std::size_t>("seq")-1);
            const auto matched = key(match.get_child("nearest_lane"));
            cp.roadIds.push_back(matched.roadId);
            for (const auto& [k, segment] : map.laneSegments) {
                (void)segment;
                if (k.roadId == matched.roadId && k.sectionIndex == matched.sectionIndex &&
                    (k.laneId > 0) == (matched.laneId > 0)) cp.lanes.insert(k);
            }
        }
        for (const auto& item : document.get_child("runtime_legs")) {
            const auto& record = item.second;
            MissionLegData leg;
            leg.topologyAuthority = true;
            leg.finalLeg = record.get<bool>("final");
            leg.fromCheckpoint = record.get<std::size_t>("from_checkpoint");
            leg.gatewayCheckpoint = record.get<std::size_t>("gateway_checkpoint");
            leg.toCheckpoint = record.get<std::size_t>("to_checkpoint");
            if (leg.fromCheckpoint != (loaded.legs.empty()?0:loaded.legs.back().toCheckpoint) ||
                leg.gatewayCheckpoint != leg.fromCheckpoint+1 ||
                leg.toCheckpoint != leg.gatewayCheckpoint+(leg.finalLeg?0:1) ||
                leg.toCheckpoint >= loaded.checkpoints.size())
                throw std::runtime_error("runtime leg breaks checkpoint ordering");
            for (const auto& row : record.get_child("allowed")) leg.allowedLanes.insert(key(row.second));
            for (const auto& row : record.get_child("gateway_lanes")) leg.gatewayLanes.insert(key(row.second));
            for (const auto& row : record.get_child("targets")) {
                const auto k = key(row.second.get_child("lane"));
                const auto station = row.second.get<double>("road_s");
                const auto& segment = map.laneSegments.at(k);
                if (!leg.allowedLanes.count(k) || !std::isfinite(station) ||
                    station < segment.startS-1e-3 || station > segment.endS+1e-3)
                    throw std::runtime_error("gateway target is outside its XODR lane");
                leg.targetLanes.insert(k); leg.targetRoadS[k] = station;
            }
            if (leg.targetLanes.empty() || (!leg.finalLeg && leg.gatewayLanes.empty()))
                throw std::runtime_error("missing actual gateway lanes");
            for (const auto& row : record.get_child("witness_route")) {
                const auto k = key(row.second);
                if (!leg.allowedLanes.count(k)) throw std::runtime_error("witness leaves corridor");
                if (!leg.preferredRoute.empty()) {
                    const auto edges = map.laneGraph.find(leg.preferredRoute.back());
                    if (edges == map.laneGraph.end() ||
                        std::find(edges->second.begin(), edges->second.end(), k) == edges->second.end())
                        throw std::runtime_error("witness has a non-XODR lane link");
                }
                leg.preferredRoute.push_back(k);
            }
            for (const auto& lane : leg.allowedLanes) {
                std::vector<LaneKey> route;
                if (findRestrictedRoute(map, leg.allowedLanes, leg.targetLanes, lane, route))
                    leg.routesToTargetByLane.emplace(lane, std::move(route));
            }
            if (leg.preferredRoute.empty() ||
                !leg.routesToTargetByLane.count(leg.preferredRoute.front()))
                throw std::runtime_error("no directed corridor route reaches gateway");
            if (!loaded.legs.empty()) for (const auto& lane : loaded.legs.back().targetLanes)
                if (!leg.routesToTargetByLane.count(lane))
                    throw std::runtime_error("gateway alternative cannot reach next gateway");
            leg.valid = true;
            loaded.legs.push_back(std::move(leg));
        }
        if (loaded.legs.empty() || !loaded.legs.back().finalLeg ||
            loaded.legs.back().toCheckpoint+1 != loaded.checkpoints.size())
            throw std::runtime_error("mission does not reach finish");
        loaded.initialized = loaded.valid = loaded.topologyAuthority = true;
        loaded.minimumTargetDistance = kInfinity;
        output = std::move(loaded);
        return true;
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }
}

const MissionLegData* activeMissionLeg(const MissionCorridorData& mission) {
    if (!mission.valid || mission.activeLegIndex >= mission.legs.size()) {
        return nullptr;
    }
    return &mission.legs[mission.activeLegIndex];
}

bool missionLegRequiresGuideContinuity(const MissionCorridorData& mission) {
    if (mission.topologyAuthority) return false;
    const MissionLegData* leg=activeMissionLeg(mission);
    if(!leg || leg->fromCheckpoint>=mission.checkpoints.size() ||
       leg->toCheckpoint>=mission.checkpoints.size() ||
       !std::isfinite(leg->bestCost)) return false;
    const auto& from=mission.checkpoints[leg->fromCheckpoint].checkpoint;
    const auto& gateway=
        mission.checkpoints[leg->gatewayCheckpoint].checkpoint;
    const auto& to=mission.checkpoints[leg->toCheckpoint].checkpoint;
    const auto excessive=[](double cost,const Checkpoint& begin,
                            const Checkpoint& end) {
        const double direct=std::hypot(end.x-begin.x,end.y-begin.y);
        return cost>std::max(150.0,3.0*direct+30.0);
    };
    // 150 m absolute slack avoids classifying an ordinary curved block as a
    // topology gap.  The 3x bound still catches the 20--40 m intersection
    // pairs whose lane graph incorrectly proposes a kilometre-scale loop.
    return excessive(leg->gatewayBestCost,from,gateway) ||
           excessive(leg->exitBestCost,gateway,to);
}

bool missionLaneRequiresGuideContinuity(const MissionCorridorData& mission,
                                        const LaneKey& currentLane) {
    if (mission.topologyAuthority) return false;
    const MissionLegData* leg=activeMissionLeg(mission);
    if(!leg) return false;
    if(missionLegRequiresGuideContinuity(mission)) return true;
    return leg->routesToTargetByLane.find(currentLane)==
           leg->routesToTargetByLane.end();
}

void updateMissionCorridorProgress(MissionCorridorData& mission,
                                   const WorldState& world) {
    const MissionLegData* leg = activeMissionLeg(mission);
    if (leg == nullptr) return;
    if (mission.topologyAuthority) {
        Waypoint pose;
        pose.roadId = world.egoRoadId;
        pose.sectionIndex = world.egoSectionIndex;
        pose.laneId = world.egoLaneId;
        pose.roadS = world.egoRoadS;
        // Actual directed exit lane/station, across its full width. Never a
        // radius around an approximate checkpoint or a nearest route.csv point.
        if (world.egoLaneValid && missionPointIsTarget(*leg, pose))
            ++mission.targetRoadStableFrames;
        else mission.targetRoadStableFrames = 0;
        if (mission.targetRoadStableFrames >= 2) {
            ++mission.activeLegIndex;
            mission.targetRoadStableFrames = 0;
            mission.minimumTargetDistance = kInfinity;
            mission.targetRoadObserved = false;
        }
        return;
    }
    const MissionCheckpointData& target =
        mission.checkpoints[leg->toCheckpoint];
    const double distance = std::hypot(
        world.ego.x - target.checkpoint.x,
        world.ego.y - target.checkpoint.y);
    mission.minimumTargetDistance = std::min(
        mission.minimumTargetDistance, distance);
    if (world.egoLaneValid && containsRoad(target, world.egoRoadId)) {
        ++mission.targetRoadStableFrames;
        mission.targetRoadObserved = true;
    } else {
        mission.targetRoadStableFrames = 0;
    }

    // 좌표 자체는 의무 통과점이 아니므로 세 가지 일반 조건 중 하나면 다음
    // 구간으로 전진한다. activeLegIndex는 절대 뒤로 가지 않아 접속도로의
    // road/lane map-match 흔들림이 목표를 프레임마다 되돌리지 못한다.
    const bool closeEnough = mission.targetRoadObserved && distance <= 10.0;
    const bool stableTargetRoad =
        mission.targetRoadStableFrames >= 2 && distance <= 30.0;
    // 직은 checkpoint 좌표를 지나쳤다는 이유만으로는 다음 leg로
    // 넘어가지 않는다. 목표 road를 한 번이라도 실제로 맵매칭한 후에만
    // 근처를 통과한 것을 인정해, 회전을 놓친 상태에서 mission이 앞으로
    // 건너뛰는 것을 막는다.
    const bool clearlyPassed = mission.targetRoadObserved &&
                               mission.minimumTargetDistance <= 22.0 &&
                               distance >= mission.minimumTargetDistance + 5.0;
    if (!closeEnough && !stableTargetRoad && !clearlyPassed) return;

    ++mission.activeLegIndex;
    mission.minimumTargetDistance = kInfinity;
    mission.targetRoadStableFrames = 0;
    mission.targetRoadObserved = false;
}

bool findMissionForwardRoute(const PathGeneratorData& map,
                             const MissionCorridorData& mission,
                             const LaneKey& start,
                             std::vector<LaneKey>& route,
                             LaneKey& reachedLane) {
    route.clear();
    const MissionLegData* leg = activeMissionLeg(mission);
    if (leg == nullptr || !map.laneSegments.count(start)) return false;

    std::queue<LaneKey> pending;
    std::map<LaneKey, LaneKey> previous;
    std::set<LaneKey> visited;
    pending.push(start);
    visited.insert(start);
    LaneKey goal;
    bool found = laneInSet(leg->targetLanes, start);
    if (found) goal = start;

    while (!found && !pending.empty()) {
        const LaneKey current = pending.front();
        pending.pop();
        const auto edges = map.laneGraph.find(current);
        if (edges == map.laneGraph.end()) continue;
        for (const LaneKey& next : edges->second) {
            if (isLateralEdge(current, next) || visited.count(next)) continue;
            // 정상 corridor 안에서는 잘못된 분기로 빠지지 않는다. 현재 차가
            // map-match 경계 때문에 한 프레임 회랑 밖에 있으면, target까지
            // 복구할 수 있도록 첫 진입 전까지는 필터를 강제하지 않는다.
            if (laneInSet(leg->allowedLanes, current) &&
                !laneInSet(leg->allowedLanes, next) &&
                !laneInSet(leg->targetLanes, next)) {
                continue;
            }
            visited.insert(next);
            previous[next] = current;
            if (laneInSet(leg->targetLanes, next)) {
                goal = next;
                found = true;
                break;
            }
            pending.push(next);
        }
    }
    if (!found) return false;
    for (LaneKey current = goal;; current = previous.at(current)) {
        route.push_back(current);
        if (current == start) break;
    }
    std::reverse(route.begin(), route.end());
    reachedLane = goal;
    return true;
}

bool findMissionLaneRoute(const PathGeneratorData& map,
                          const MissionCorridorData& mission,
                          const LaneKey& start,
                          std::vector<LaneKey>& route) {
    route.clear();
    const MissionLegData* leg = activeMissionLeg(mission);
    if (leg == nullptr || !map.laneSegments.count(start)) return false;
    const auto cached = leg->routesToTargetByLane.find(start);
    if (cached != leg->routesToTargetByLane.end()) {
        route = cached->second;
        return !route.empty();
    }
    struct Entry {
        double cost{};
        LaneKey key;
    };
    const auto farther = [](const Entry& left, const Entry& right) {
        return left.cost > right.cost;
    };
    std::priority_queue<Entry, std::vector<Entry>, decltype(farther)> pending(
        farther);
    DistanceMap costs;
    std::map<LaneKey, LaneKey> previous;
    costs[start] = 0.0;
    pending.push({0.0, start});
    LaneKey goal;
    bool found = laneInSet(leg->targetLanes, start);
    if (found) goal = start;

    while (!found && !pending.empty()) {
        const Entry entry = pending.top();
        pending.pop();
        if (entry.cost > costs[entry.key] + 1e-9) continue;
        if (laneInSet(leg->targetLanes, entry.key)) {
            goal = entry.key;
            found = true;
            break;
        }
        const auto edges = map.laneGraph.find(entry.key);
        if (edges == map.laneGraph.end()) continue;
        for (const LaneKey& next : edges->second) {
            if (laneInSet(leg->allowedLanes, entry.key) &&
                !laneInSet(leg->allowedLanes, next) &&
                !laneInSet(leg->targetLanes, next)) {
                continue;
            }
            const double step = edgeCost(map, entry.key, next);
            if (!std::isfinite(step)) continue;
            const double candidate = entry.cost + step;
            const auto known = costs.find(next);
            if (known != costs.end() &&
                candidate >= known->second - 1e-9) {
                continue;
            }
            costs[next] = candidate;
            previous[next] = entry.key;
            pending.push({candidate, next});
        }
    }
    if (!found) return false;
    for (LaneKey current = goal;; current = previous.at(current)) {
        route.push_back(current);
        if (current == start) break;
    }
    std::reverse(route.begin(), route.end());
    return true;
}

bool saveMissionCorridorArtifacts(const MissionCorridorData& mission,
                                  const PathGeneratorData& map,
                                  const std::vector<Waypoint>& guidePath,
                                  const std::string& svgPath,
                                  const std::string& csvPath) {
    if (!mission.valid) return false;
    std::ofstream csv(csvPath);
    if (!csv) return false;
    csv << "leg_index,role,road_id,section_index,lane_id,point_index,x,y\n";
    csv << std::fixed << std::setprecision(8);
    std::set<LaneKey> allAllowed;
    std::set<LaneKey> allGateways;
    std::set<LaneKey> allJunctionConnectors;
    for (std::size_t legIndex = 0; legIndex < mission.legs.size(); ++legIndex) {
        const MissionLegData& leg = mission.legs[legIndex];
        const std::set<LaneKey> preferred(
            leg.preferredRoute.begin(), leg.preferredRoute.end());
        allAllowed.insert(leg.allowedLanes.begin(), leg.allowedLanes.end());
        allGateways.insert(leg.targetLanes.begin(), leg.targetLanes.end());
        for (const LaneKey& key : leg.preferredRoute) {
            const auto road = map.roads.find(key.roadId);
            if (road != map.roads.end() && road->second.junction >= 0) {
                allJunctionConnectors.insert(key);
            }
        }
        for (const LaneKey& key : leg.allowedLanes) {
            const auto segment = map.laneSegments.find(key);
            if (segment == map.laneSegments.end()) continue;
            const char* role = laneInSet(leg.targetLanes, key)
                                   ? "gateway"
                                   : (laneInSet(preferred, key)
                                          ? "preferred" : "corridor");
            for (std::size_t index = 0;
                 index < segment->second.points.size(); ++index) {
                const Waypoint& point = segment->second.points[index];
                csv << legIndex << ',' << role << ',' << key.roadId << ','
                    << key.sectionIndex << ',' << key.laneId << ',' << index
                    << ',' << point.x << ',' << point.y << '\n';
            }
        }
    }
    csv.close();

    std::vector<std::pair<double, double>> focus;
    for (const LaneKey& key : allAllowed) {
        const auto segment = map.laneSegments.find(key);
        if (segment == map.laneSegments.end()) continue;
        for (const Waypoint& point : segment->second.points) {
            focus.emplace_back(point.x, point.y);
        }
    }
    for (const MissionCheckpointData& checkpoint : mission.checkpoints) {
        focus.emplace_back(checkpoint.checkpoint.x, checkpoint.checkpoint.y);
    }
    if (focus.empty()) return false;

    constexpr double width = 1200.0;
    constexpr double height = 800.0;
    constexpr double margin = 45.0;
    double minX = focus.front().first;
    double maxX = minX;
    double minY = focus.front().second;
    double maxY = minY;
    for (const auto& point : focus) {
        minX = std::min(minX, point.first);
        maxX = std::max(maxX, point.first);
        minY = std::min(minY, point.second);
        maxY = std::max(maxY, point.second);
    }
    constexpr double worldPadding = 20.0;
    minX -= worldPadding;
    maxX += worldPadding;
    minY -= worldPadding;
    maxY += worldPadding;
    const double scale = std::min(
        (width - 2.0 * margin) / std::max(1.0, maxX - minX),
        (height - 2.0 * margin) / std::max(1.0, maxY - minY));
    const auto screen = [&](double x, double y) {
        return std::pair<double, double>{
            margin + (x - minX) * scale,
            height - margin - (y - minY) * scale};
    };
    const auto visible = [&](const LaneSegment& segment) {
        return std::any_of(
            segment.points.begin(), segment.points.end(),
            [&](const Waypoint& point) {
                return point.x >= minX && point.x <= maxX &&
                       point.y >= minY && point.y <= maxY;
            });
    };
    const auto polyline = [&](const LaneSegment& segment) {
        std::ostringstream points;
        points << std::fixed << std::setprecision(2);
        for (const Waypoint& point : segment.points) {
            const auto [x, y] = screen(point.x, point.y);
            points << x << ',' << y << ' ';
        }
        return points.str();
    };

    std::ofstream svg(svgPath);
    if (!svg) return false;
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1200\" "
           "height=\"800\" viewBox=\"0 0 1200 800\">\n"
           "<rect width=\"100%\" height=\"100%\" fill=\"#075f22\"/>\n";
    for (const auto& [key, segment] : map.laneSegments) {
        (void)key;
        if (!visible(segment)) continue;
        svg << "<polyline points=\"" << polyline(segment)
            << "\" fill=\"none\" stroke=\"#aeb4b2\" stroke-width=\""
            << std::max(1.0, 2.4 * scale)
            << "\" stroke-linecap=\"round\" opacity=\"0.72\"/>\n";
    }
    // 굵은 노란색: checkpoint 사이에서 장애물 회피에 자유롭게 쓸 수 있는
    // 모든 lane. 차선 폭과 비슷한 stroke로 road 영역처럼 보이게 한다.
    for (const LaneKey& key : allAllowed) {
        const auto segment = map.laneSegments.find(key);
        if (segment == map.laneSegments.end()) continue;
        const auto road = map.roads.find(key.roadId);
        if (road != map.roads.end() && road->second.junction >= 0) continue;
        svg << "<polyline points=\"" << polyline(segment->second)
            << "\" fill=\"none\" stroke=\"#ffe91f\" stroke-width=\""
            << std::max(2.0, 3.0 * scale)
            << "\" stroke-linecap=\"round\" opacity=\"0.68\"/>\n";
    }
    // 얇은 주황선: 교차로 진입점에서만 선택하는 connector와
    // 다음 구역의 gateway lane. 도로 구역 전체를 차선 하나로 묶지
    // 않는다는 구분이 이미지에서도 보이게 한다.
    allJunctionConnectors.insert(allGateways.begin(), allGateways.end());
    for (const LaneKey& key : allJunctionConnectors) {
        const auto segment = map.laneSegments.find(key);
        if (segment == map.laneSegments.end()) continue;
        svg << "<polyline points=\"" << polyline(segment->second)
            << "\" fill=\"none\" stroke=\"#ffb300\" stroke-width=\""
            << std::max(1.3, 0.65 * scale)
            << "\" stroke-linecap=\"round\" opacity=\"0.95\"/>\n";
    }
    if (guidePath.size() >= 2) {
        svg << "<polyline points=\"";
        svg << std::fixed << std::setprecision(2);
        for (const Waypoint& point : guidePath) {
            const auto [x, y] = screen(point.x, point.y);
            svg << x << ',' << y << ' ';
        }
        svg << "\" fill=\"none\" stroke=\"#1769e0\" stroke-width=\"1.2\" "
               "stroke-dasharray=\"5 4\" opacity=\"0.8\"/>\n";
    }
    for (const MissionCheckpointData& checkpoint : mission.checkpoints) {
        const auto [x, y] = screen(checkpoint.checkpoint.x,
                                   checkpoint.checkpoint.y);
        svg << "<circle cx=\"" << x << "\" cy=\"" << y
            << "\" r=\"5\" fill=\"#e53935\"/>\n"
            << "<text x=\"" << x + 7.0 << "\" y=\"" << y - 7.0
            << "\" fill=\"#d50000\" font-family=\"sans-serif\" "
               "font-size=\"14\">"
            << checkpoint.checkpoint.sequence << "</text>\n";
    }
    svg << "<text x=\"45\" y=\"28\" fill=\"white\" "
           "font-family=\"sans-serif\" font-size=\"16\">"
           "yellow: free road corridor / orange: final connector / blue: guide"
           "</text>\n</svg>\n";
    return true;
}
