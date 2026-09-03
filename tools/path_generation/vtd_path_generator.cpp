#include "path_generation/vtd_path_generator.hpp"

#include "planning/global/reference_path_generator.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kRoadSEpsilon = 1e-6;

bool containsRoadS(const LaneSegment& segment, double roadS) {
    return roadS + kRoadSEpsilon >= segment.startS &&
           roadS <= segment.endS + kRoadSEpsilon;
}

std::size_t nearestPointIndex(const LaneSegment& segment, double roadS) {
    std::size_t selected = 0;
    double bestDelta = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < segment.points.size(); ++index) {
        const double delta = std::abs(segment.points[index].roadS - roadS);
        if (delta < bestDelta) {
            bestDelta = delta;
            selected = index;
        }
    }
    return selected;
}

std::vector<LaneMatch> candidatesAtRoadS(
    const PathGeneratorData& data,
    const VtdPathPoint& point,
    std::optional<int> requiredLane) {
    std::vector<LaneMatch> candidates;
    for (const auto& [key, segment] : data.laneSegments) {
        if (key.roadId != point.roadId || segment.points.empty() ||
            !containsRoadS(segment, point.roadS)) {
            continue;
        }
        if (requiredLane && key.laneId != *requiredLane) continue;

        const std::size_t pointIndex = nearestPointIndex(segment, point.roadS);
        const Road& road = data.roads.at(key.roadId);
        const LaneDefinition& lane =
            road.sections[key.sectionIndex].lanes.at(key.laneId);
        const double width =
            evaluatePiecewise(lane.widths, point.roadS, 0.0);
        candidates.push_back({true, key.roadId, key.laneId, key.sectionIndex,
                              point.roadS, 0.0, width, pointIndex});
    }
    return candidates;
}

double transitionDistance(const PathGeneratorData& data,
                          const LaneMatch& start,
                          const LaneMatch& goal) {
    const LaneKey startKey{start.roadId, start.sectionIndex, start.laneId};
    const LaneKey goalKey{goal.roadId, goal.sectionIndex, goal.laneId};
    if (startKey == goalKey && goal.pointIndex < start.pointIndex) {
        return std::numeric_limits<double>::infinity();
    }
    const std::vector<LaneKey> route = connectLanes(data, startKey, goalKey);
    if (route.empty()) return std::numeric_limits<double>::infinity();
    std::vector<Waypoint> candidate;
    if (!appendRoute(data, route, start, goal, candidate)) {
        return std::numeric_limits<double>::infinity();
    }
    removeDuplicates(candidate);
    updateYawAndS(candidate);
    if (!validateGeneratedPath(candidate)) {
        return std::numeric_limits<double>::infinity();
    }
    return candidate.back().s;
}

std::vector<LaneMatch> selectConnectedMatches(
    const PathGeneratorData& data,
    const std::vector<std::vector<LaneMatch>>& candidateSets) {
    if (candidateSets.empty()) return {};
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> costs(candidateSets.size());
    std::vector<std::vector<int>> previous(candidateSets.size());
    costs.front().assign(candidateSets.front().size(), 0.0);
    previous.front().assign(candidateSets.front().size(), -1);

    for (std::size_t pathIndex = 1; pathIndex < candidateSets.size();
         ++pathIndex) {
        costs[pathIndex].assign(candidateSets[pathIndex].size(), infinity);
        previous[pathIndex].assign(candidateSets[pathIndex].size(), -1);
        for (std::size_t current = 0;
             current < candidateSets[pathIndex].size(); ++current) {
            for (std::size_t prior = 0;
                 prior < candidateSets[pathIndex - 1].size(); ++prior) {
                if (!std::isfinite(costs[pathIndex - 1][prior])) continue;
                const double distance = transitionDistance(
                    data, candidateSets[pathIndex - 1][prior],
                    candidateSets[pathIndex][current]);
                if (!std::isfinite(distance)) continue;
                const double candidateCost =
                    costs[pathIndex - 1][prior] + distance;
                if (candidateCost < costs[pathIndex][current]) {
                    costs[pathIndex][current] = candidateCost;
                    previous[pathIndex][current] = static_cast<int>(prior);
                }
            }
        }
    }

    int selected = -1;
    double bestCost = infinity;
    for (std::size_t index = 0; index < costs.back().size(); ++index) {
        if (costs.back()[index] < bestCost) {
            bestCost = costs.back()[index];
            selected = static_cast<int>(index);
        }
    }
    if (selected < 0) return {};

    std::vector<LaneMatch> matches(candidateSets.size());
    for (std::size_t reverse = candidateSets.size(); reverse > 0; --reverse) {
        const std::size_t pathIndex = reverse - 1;
        matches[pathIndex] = candidateSets[pathIndex][selected];
        selected = previous[pathIndex][selected];
    }
    return matches;
}

}  // namespace

Waypoint roadLaneToWaypoint(const PathGeneratorData& data,
                            int roadId, int laneId, double roadS) {
    const auto roadEntry = data.roads.find(roadId);
    if (roadEntry == data.roads.end()) {
        throw std::invalid_argument("unknown XODR road id: " +
                                    std::to_string(roadId));
    }
    const Road& road = roadEntry->second;
    if (roadS < -kRoadSEpsilon || roadS > road.length + kRoadSEpsilon) {
        throw std::invalid_argument("road s is outside road " +
                                    std::to_string(roadId));
    }
    roadS = std::clamp(roadS, 0.0, road.length);

    int sectionIndex = 0;
    for (std::size_t index = 0; index < road.sections.size(); ++index) {
        if (road.sections[index].s <= roadS + kRoadSEpsilon) {
            sectionIndex = static_cast<int>(index);
        } else {
            break;
        }
    }
    const LaneSection& section = road.sections.at(sectionIndex);
    const auto laneEntry = section.lanes.find(laneId);
    if (laneEntry == section.lanes.end() ||
        laneEntry->second.type != "driving") {
        throw std::invalid_argument(
            "road " + std::to_string(roadId) + " lane " +
            std::to_string(laneId) + " is not driving at s=" +
            std::to_string(roadS));
    }

    const RoadPose reference = roadToWorld(road, roadS);
    const double offset = laneOffset(road, section, laneId, roadS);
    Waypoint point;
    point.x = reference.x - offset * std::sin(reference.yaw);
    point.y = reference.y + offset * std::cos(reference.yaw);
    point.yaw = laneId < 0
                    ? reference.yaw
                    : normalizeAngle(reference.yaw + 3.14159265358979323846);
    point.roadS = roadS;
    point.roadId = roadId;
    point.laneId = laneId;
    point.sectionIndex = sectionIndex;
    return point;
}

VtdRouteResult makePathFromVtdRoute(
    const PathGeneratorData& data,
    const std::vector<VtdPathPoint>& vtdPath,
    int startLane) {
    VtdRouteResult result;
    if (vtdPath.size() < 2) {
        std::cerr << "[vtd-path] at least two VTD waypoints are required\n";
        return result;
    }

    std::vector<std::vector<LaneMatch>> candidateSets;
    candidateSets.reserve(vtdPath.size());
    for (std::size_t index = 0; index < vtdPath.size(); ++index) {
        const std::optional<int> requiredLane =
            index == 0 ? std::optional<int>(startLane) : std::nullopt;
        candidateSets.push_back(
            candidatesAtRoadS(data, vtdPath[index], requiredLane));
        if (candidateSets.back().empty()) {
            std::cerr << "[vtd-path] no driving lane at waypoint "
                      << vtdPath[index].sequence << " road="
                      << vtdPath[index].roadId << " s="
                      << vtdPath[index].roadS;
            if (requiredLane) std::cerr << " lane=" << *requiredLane;
            std::cerr << '\n';
            return result;
        }
    }

    result.matches = selectConnectedMatches(data, candidateSets);
    if (result.matches.size() != vtdPath.size()) {
        std::cerr << "[vtd-path] no legal lane combination connects all "
                     "VTD waypoints\n";
        result.matches.clear();
        return result;
    }

    for (std::size_t index = 0; index < result.matches.size(); ++index) {
        const LaneMatch& match = result.matches[index];
        Waypoint checkpoint = roadLaneToWaypoint(
            data, match.roadId, match.laneId, vtdPath[index].roadS);
        result.checkpoints.push_back(checkpoint);
        std::cout << "[vtd-path] seq=" << vtdPath[index].sequence
                  << " road=" << match.roadId << " lane=" << match.laneId
                  << " s=" << vtdPath[index].roadS << " -> ("
                  << checkpoint.x << ", " << checkpoint.y << ")\n";
    }

    for (std::size_t index = 0; index + 1 < result.matches.size(); ++index) {
        const LaneMatch& start = result.matches[index];
        const LaneMatch& goal = result.matches[index + 1];
        const LaneKey startKey{start.roadId, start.sectionIndex, start.laneId};
        const LaneKey goalKey{goal.roadId, goal.sectionIndex, goal.laneId};
        const std::vector<LaneKey> laneRoute =
            connectLanes(data, startKey, goalKey);
        if (laneRoute.empty()) {
            std::cerr << "[vtd-path] route disappeared while assembling path\n";
            result = {};
            return result;
        }
        if (!appendRoute(data, laneRoute, start, goal, result.path)) {
            std::cerr << "[vtd-path] lane route is not physically traversable\n";
            result = {};
            return result;
        }
    }
    removeDuplicates(result.path);
    if (!result.path.empty()) {
        result.path.front() = result.checkpoints.front();
        result.path.back() = result.checkpoints.back();
    }
    updateYawAndS(result.path);
    if (!result.path.empty() && !validateGeneratedPath(result.path)) {
        std::cerr << "[vtd-path] generated path failed geometry validation\n";
        result = {};
    }
    return result;
}
