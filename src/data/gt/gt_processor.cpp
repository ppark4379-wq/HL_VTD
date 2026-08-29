#include "data/gt/gt_processor.hpp"

#include "utils/geometry.hpp"
#include "utils/math_utils.hpp"

#include <array>
#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>

namespace {

bool validObject(const ObjectState& object) {
    constexpr double kMinimumDimension = 0.05;
    constexpr double kMaximumDimension = 50.0;
    return std::isfinite(object.x) && std::isfinite(object.y) &&
           std::isfinite(object.heading) && std::isfinite(object.speed) &&
           std::isfinite(object.length) && std::isfinite(object.width) &&
           std::isfinite(object.height) &&
           object.length >= kMinimumDimension &&
           object.width >= kMinimumDimension &&
           object.height >= kMinimumDimension &&
           object.length <= kMaximumDimension &&
           object.width <= kMaximumDimension &&
           object.height <= kMaximumDimension;
}

}  // namespace

Obb2D makeObjectObb(const ObjectState& object) {
    return makeObb2D(object.x, object.y, object.heading,
                     object.length, object.width);
}

void worldToRoute(
    double pointX,
    double pointY,
    bool& pathValid,
    double& pathS,
    double& pathD,
    const std::vector<Waypoint>& path,
    std::optional<double> expectedS = std::nullopt,
    std::optional<double> heading = std::nullopt) {

    if (path.size() < 2) {
        pathValid = false;
        return;
    }

    double bestDistance = std::numeric_limits<double>::max();
    double bestScore = std::numeric_limits<double>::max();
    double bestS = 0.0;
    double bestD = 0.0;

    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Waypoint& p0 = path[i];
        const Waypoint& p1 = path[i + 1];
        if (expectedS && std::abs(p0.s - *expectedS) > 60.0) continue;

        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;

        const double lengthSquared = dx * dx + dy * dy;

        if (lengthSquared < 1e-9) {
            continue;
        }

        // Object를 현재 path segment에 정사영
        double t =
            ((pointX - p0.x) * dx +
             (pointY - p0.y) * dy)
            / lengthSquared;

        // Segment 범위를 벗어나지 않도록 제한
        t = std::clamp(t, 0.0, 1.0);

        const double projectedX = p0.x + t * dx;
        const double projectedY = p0.y + t * dy;

        const double offsetX = pointX - projectedX;
        const double offsetY = pointY - projectedY;

        const double distance =
            std::hypot(offsetX, offsetY);

        double score = distance;
        if (heading) {
            const double segmentHeading = std::atan2(dy, dx);
            score += 2.0 * std::abs(
                normalizeAngle(*heading - segmentHeading));
        }
        if (score < bestScore) {
            bestScore = score;
            bestDistance = distance;

            const double segmentLength =
                std::sqrt(lengthSquared);

            // Global path 시작점부터의 종방향 거리
            bestS =
                p0.s + t * segmentLength;

            // Path 진행방향 기준 좌/우 판별
            const double cross =
                dx * offsetY - dy * offsetX;

            bestD =
                (cross >= 0.0 ? 1.0 : -1.0)
                * distance;
        }
    }

    if (bestDistance <= 10.0) {
        pathValid = true;
        pathS = bestS;
        pathD = bestD;
    }
}

void objectToRoute(
    WorldObjectState& state,
    const std::vector<Waypoint>& path) {
    worldToRoute(
        state.raw.x,
        state.raw.y,
        state.pathValid,
        state.pathS,
        state.pathD,
        path, std::nullopt, state.raw.heading);
}

WorldObjectState makeWorldObjectState(
    const ObjectState& object,
    const PathGeneratorData& mapData,
    const std::vector<Waypoint>& globalPath) {

    WorldObjectState state;

    // Raw GT
    state.raw = object;

    // OBB
    state.obb = makeObjectObb(object);

    // Road / Lane Matching
    const LaneMatch match = worldToLane(
        mapData,
        object.x,
        object.y,
        static_cast<double>(object.heading));

    state.laneValid = match.valid;

    if (match.valid) {
        state.roadId = match.roadId;
        state.laneId = match.laneId;
        state.sectionIndex = match.sectionIndex;
        state.roadS = match.roadS;
    }

    // Global Path 기준 Frenet 좌표
    objectToRoute(state, globalPath);

    return state;
}

WorldState makeWorldState(
    const GtFrame& gt,
    const PathGeneratorData& mapData,
    const std::vector<Waypoint>& globalPath,
    std::optional<double> expectedEgoRouteS) {

    WorldState world;
    world.ego = gt.ego;
    world.trafficLight = gt.trafficLight;

    const LaneMatch egoMatch = worldToLane(
        mapData,
        gt.ego.x,
        gt.ego.y,
        static_cast<double>(gt.ego.heading));
    world.egoLaneValid = egoMatch.valid;
    if (egoMatch.valid) {
        world.egoRoadId = egoMatch.roadId;
        world.egoLaneId = egoMatch.laneId;
        world.egoSectionIndex = egoMatch.sectionIndex;
        world.egoRoadS = egoMatch.roadS;
        world.egoLaneWidth = egoMatch.laneWidth;
        world.egoLaneCenterDistance = egoMatch.distance;
    }
    worldToRoute(
        gt.ego.x,
        gt.ego.y,
        world.egoPathValid,
        world.egoPathS,
        world.egoPathD,
        globalPath, expectedEgoRouteS, gt.ego.heading);

    for (const auto& object : gt.objects) {
        if (object.id == 0 || !validObject(object)) {
            continue;
        }

        world.objects.push_back(
            makeWorldObjectState(
                object,
                mapData,
                globalPath));
    }

    return world;
}
