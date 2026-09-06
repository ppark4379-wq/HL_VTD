#include "planning/global/reference_path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

double normalizeAngle(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

LaneMatch nearestLaneCenter(const PathGeneratorData& data, double x, double y,
                            double heading) {
    LaneMatch best;
    double bestScore = std::numeric_limits<double>::infinity();
    for (const auto& [key, segment] : data.laneSegments) {
        for (std::size_t index = 0; index < segment.points.size(); ++index) {
            const Waypoint& point = segment.points[index];
            const double distance = std::hypot(x - point.x, y - point.y);
            const double headingError =
                std::abs(normalizeAngle(heading - point.yaw));
            const double score = distance * distance +
                                 4.0 * headingError * headingError;
            if (score >= bestScore) continue;
            bestScore = score;
            best = {true, key.roadId, key.laneId, key.sectionIndex,
                    point.roadS, distance, 0.0, index};
        }
    }
    return best;
}

std::vector<Waypoint> interpolatePathShape(PathGeneratorData& data,
                                           double interval) {
    std::vector<Waypoint> route;
    double cumulativeS = 0.0;
    for (std::size_t segmentIndex = 0;
         segmentIndex + 1 < data.checkpoints.size(); ++segmentIndex) {
        const Checkpoint& start = data.checkpoints[segmentIndex];
        const Checkpoint& end = data.checkpoints[segmentIndex + 1];
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double length = std::hypot(dx, dy);
        if (length <= 1e-9) continue;
        const int steps = std::max(
            1, static_cast<int>(std::ceil(length / interval)));
        for (int step = segmentIndex == 0 ? 0 : 1; step <= steps; ++step) {
            const double ratio = static_cast<double>(step) / steps;
            Waypoint point;
            point.x = start.x + ratio * dx;
            point.y = start.y + ratio * dy;
            point.yaw = std::atan2(dy, dx);
            if (!route.empty()) {
                cumulativeS += std::hypot(point.x - route.back().x,
                                          point.y - route.back().y);
            }
            point.s = cumulativeS;
            const LaneMatch lane = worldToLane(data, point.x, point.y);
            if (lane.valid) {
                point.roadId = lane.roadId;
                point.laneId = lane.laneId;
                point.roadS = lane.roadS;
            }
            route.push_back(point);
        }
    }
    return route;
}

void smoothLaneTransitions(std::vector<Waypoint>& route,
                           std::size_t radius) {
    if (route.size() < 3 || radius == 0) return;
    const std::vector<Waypoint> original = route;
    for (std::size_t index = 1; index + 1 < route.size(); ++index) {
        const std::size_t begin = index > radius ? index - radius : 0;
        const std::size_t end = std::min(route.size() - 1, index + radius);
        double weightedX = 0.0;
        double weightedY = 0.0;
        double totalWeight = 0.0;
        for (std::size_t sample = begin; sample <= end; ++sample) {
            const std::size_t separation = sample > index
                ? sample - index : index - sample;
            const double weight = static_cast<double>(radius + 1 - separation);
            weightedX += original[sample].x * weight;
            weightedY += original[sample].y * weight;
            totalWeight += weight;
        }
        const double endpointTaper = std::min(
            1.0,
            static_cast<double>(std::min(index, route.size() - 1 - index)) /
                static_cast<double>(radius));
        route[index].x = original[index].x * (1.0 - endpointTaper) +
                         weightedX / totalWeight * endpointTaper;
        route[index].y = original[index].y * (1.0 - endpointTaper) +
                         weightedY / totalWeight * endpointTaper;
    }
}

bool excessiveCurvatureNearEnd(const std::vector<Waypoint>& route,
                               std::size_t radius) {
    if (route.size() < 3) return false;
    constexpr double kMaximumCurvature = 0.20;
    const std::size_t begin = route.size() > radius + 1
        ? route.size() - radius - 1 : 0;
    for (std::size_t index = begin; index + 1 < route.size(); ++index) {
        const double spacing = std::hypot(
            route[index + 1].x - route[index].x,
            route[index + 1].y - route[index].y);
        if (spacing <= 1e-9) continue;
        const double curvature = std::abs(normalizeAngle(
            route[index + 1].yaw - route[index].yaw)) / spacing;
        if (curvature > kMaximumCurvature) return true;
    }
    return false;
}

void smoothEndpointTransition(std::vector<Waypoint>& route,
                              double targetHeading,
                              std::size_t radius) {
    if (route.size() < 3 || radius < 2 ||
        !excessiveCurvatureNearEnd(route, radius)) {
        return;
    }
    const std::size_t begin = route.size() > radius + 1
        ? route.size() - radius - 1 : 0;
    const std::size_t end = route.size() - 1;
    const Waypoint start = route[begin];
    const Waypoint goal = route[end];
    const double chord = std::hypot(goal.x - start.x, goal.y - start.y);
    if (chord <= 1e-9) return;

    const double handle = chord / 3.0;
    const double control1X = start.x + handle * std::cos(start.yaw);
    const double control1Y = start.y + handle * std::sin(start.yaw);
    const double control2X = goal.x - handle * std::cos(targetHeading);
    const double control2Y = goal.y - handle * std::sin(targetHeading);
    const double denominator = static_cast<double>(end - begin);
    for (std::size_t index = begin + 1; index < end; ++index) {
        const double progress = static_cast<double>(index - begin) /
                                denominator;
        const double inverse = 1.0 - progress;
        route[index].x =
            inverse * inverse * inverse * start.x +
            3.0 * inverse * inverse * progress * control1X +
            3.0 * inverse * progress * progress * control2X +
            progress * progress * progress * goal.x;
        route[index].y =
            inverse * inverse * inverse * start.y +
            3.0 * inverse * inverse * progress * control1Y +
            3.0 * inverse * progress * progress * control2Y +
            progress * progress * progress * goal.y;
    }
    updateYawAndS(route);
}

std::vector<Waypoint> snapPathShapeToLaneCenters(PathGeneratorData& data) {
    std::vector<Waypoint> route;
    constexpr double kGuideInterval = 0.5;

    for (std::size_t segmentIndex = 0;
         segmentIndex + 1 < data.checkpoints.size(); ++segmentIndex) {
        const Checkpoint& start = data.checkpoints[segmentIndex];
        const Checkpoint& end = data.checkpoints[segmentIndex + 1];
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double length = std::hypot(dx, dy);
        if (length <= 1e-9) continue;
        const double heading = std::atan2(dy, dx);
        const int steps = std::max(
            1, static_cast<int>(std::ceil(length / kGuideInterval)));
        for (int step = segmentIndex == 0 ? 0 : 1; step <= steps; ++step) {
            const double ratio = static_cast<double>(step) / steps;
            const double x = start.x + ratio * dx;
            const double y = start.y + ratio * dy;
            const LaneMatch lane = nearestLaneCenter(data, x, y, heading);
            if (!lane.valid) return {};
            const LaneKey key{lane.roadId, lane.sectionIndex, lane.laneId};
            const auto laneSegment = data.laneSegments.find(key);
            if (laneSegment == data.laneSegments.end() ||
                lane.pointIndex >= laneSegment->second.points.size()) {
                return {};
            }
            route.push_back(laneSegment->second.points[lane.pointIndex]);
        }
    }

    // PathShape의 긴 구간은 lane center로 유지하고, 독립적인 최근접 매칭이
    // 평행 차선을 바꾸는 짧은 전환부만 약 8m 범위에서 부드럽게 만든다.
    smoothLaneTransitions(route, 16);
    removeDuplicates(route);
    updateYawAndS(route);
    const Checkpoint& previousCheckpoint =
        data.checkpoints[data.checkpoints.size() - 2];
    const Checkpoint& finalCheckpoint = data.checkpoints.back();
    smoothEndpointTransition(
        route,
        std::atan2(finalCheckpoint.y - previousCheckpoint.y,
                   finalCheckpoint.x - previousCheckpoint.x),
        60);

    double maximumLaneCenterDistance = 0.0;
    for (std::size_t index = 0; index < route.size(); ++index) {
        const double heading = index + 1 < route.size()
            ? std::atan2(route[index + 1].y - route[index].y,
                         route[index + 1].x - route[index].x)
            : route[index - 1].yaw;
        const LaneMatch lane = nearestLaneCenter(
            data, route[index].x, route[index].y, heading);
        route[index].roadId = lane.roadId;
        route[index].sectionIndex = lane.sectionIndex;
        route[index].laneId = lane.laneId;
        route[index].roadS = lane.roadS;
        maximumLaneCenterDistance =
            std::max(maximumLaneCenterDistance, lane.distance);
    }
    updateYawAndS(route);
    std::cout << "[path-shape] maximum lane-center deviation="
              << maximumLaneCenterDistance << "m\n";
    return route;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4 && argc != 5) {
        std::cerr << "Usage: " << argv[0]
                  << " MAP.xodr CHECKPOINTS.csv ROUTE.csv [--lane-snap]\n";
        return 1;
    }
    const bool laneSnap = argc == 5 && std::string(argv[4]) == "--lane-snap";
    if (argc == 5 && !laneSnap) {
        std::cerr << "[path-shape] unknown option: " << argv[4] << '\n';
        return 1;
    }

    PathGeneratorData data;
    if (!loadMap(data, argv[1], 0.5) ||
        !loadCheckpointsCsv(data, argv[2])) {
        return 2;
    }

    const std::vector<Waypoint> route = laneSnap
        ? snapPathShapeToLaneCenters(data)
        : interpolatePathShape(data, data.sampleInterval);
    if (route.size() < 2) return 3;
    if (laneSnap) {
        std::string validationReason;
        if (!validateGeneratedPath(route, &validationReason)) {
            std::cerr << "[path-shape] invalid lane-snap route: "
                      << validationReason << '\n';
            return 3;
        }
    }
    if (!savePath(route, argv[3])) return 4;

    std::cout << "[path-shape] generated " << route.size()
              << " route points, length=" << route.back().s << "m\n";
    return 0;
}
