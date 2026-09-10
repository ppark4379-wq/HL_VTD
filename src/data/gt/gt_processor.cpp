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
    std::optional<double> expectedS,
    std::optional<double> heading) {

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

    // Object는 기준경로 주변의 좁은 충돌 회랑만 보면 되므로 기존 10m를
    // 유지한다. Ego는 직전 route_s(expectedS)가 있고 checkpoint corridor
    // 안에서 여러 차선을 자유롭게 쓸 수 있으므로, 4~5차로 폭에서도 경로
    // 좌표를 잃지 않게 허용 폭을 넓힌다. expectedS의 +/-60m 창과 heading
    // 점수가 함께 적용되어 겹치는 먼 경로로 순간 점프하지는 않는다.
    const double maximumProjectionDistance = expectedS ? 18.0 : 10.0;
    if (bestDistance <= maximumProjectionDistance) {
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
    std::optional<double> expectedEgoRouteS,
    bool preferGlobalRouteLane) {

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

    // 교차로 안에서는 여러 connector road의 lane이 거의 겹쳐 있고 진행방향도
    // 비슷해서, 순수 최근접 탐색인 worldToLane이 기준 경로와 다른 connector를
    // 고르는 일이 생긴다. 그러면 Ego의 road/lane과 경로 웨이포인트의 road/lane이
    // 어긋나 차선변경·회피 계획이 전부 실패한다.
    // 기준 경로는 어느 road/lane을 지나는지 이미 알고 있으므로, Ego가 실제로
    // 그 lane 안에 들어 있을 때는 경로 쪽 판정을 우선한다.
    if (preferGlobalRouteLane && world.egoPathValid && !globalPath.empty()) {
        const auto hint = std::lower_bound(
            globalPath.begin(), globalPath.end(), world.egoPathS,
            [](const Waypoint& point, double value) { return point.s < value; });
        const Waypoint* routePoint =
            hint == globalPath.end() ? &globalPath.back() : &(*hint);
        if (hint != globalPath.begin() && hint != globalPath.end()) {
            const Waypoint& previous = *(hint - 1);
            if (std::abs(previous.s - world.egoPathS) <
                std::abs(routePoint->s - world.egoPathS)) {
                routePoint = &previous;
            }
        }
        if (!world.egoLaneValid || routePoint->roadId != world.egoRoadId ||
            routePoint->laneId != world.egoLaneId) {
            for (const LaneMatch& candidate :
                 findContainingLanes(mapData, gt.ego.x, gt.ego.y)) {
                if (!candidate.valid ||
                    candidate.roadId != routePoint->roadId ||
                    candidate.laneId != routePoint->laneId) {
                    continue;
                }
                world.egoLaneValid = true;
                world.egoRoadId = candidate.roadId;
                world.egoLaneId = candidate.laneId;
                world.egoSectionIndex = candidate.sectionIndex;
                world.egoRoadS = candidate.roadS;
                world.egoLaneWidth = candidate.laneWidth;
                world.egoLaneCenterDistance = candidate.distance;
                break;
            }
        }
    }

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

bool projectWorldToActivePath(WorldState& world,
                              const std::vector<Waypoint>& activePath,
                              double frontBumperOffset,
                              double lateralMargin) {
    world.egoPathValid = false;
    world.egoPathS = 0.0;
    world.egoPathD = 0.0;
    worldToRoute(world.ego.x, world.ego.y,
                 world.egoPathValid, world.egoPathS, world.egoPathD,
                 activePath, std::nullopt, world.ego.heading);
    if (!world.egoPathValid) {
        world.frontBumperPathS = 0.0;
        return false;
    }
    world.frontBumperPathS = world.egoPathS + frontBumperOffset;

    for (WorldObjectState& object : world.objects) {
        object.pathValid = false;
        object.pathS = 0.0;
        object.pathD = 0.0;
        object.relativeS = 0.0;
        object.relativeD = 0.0;
        object.isAhead = false;
        object.isOnEgoPath = false;
        object.ttc = std::numeric_limits<double>::infinity();
        objectToRoute(object, activePath);
        if (!object.pathValid) continue;

        object.relativeS = object.pathS - world.egoPathS;
        object.relativeD = object.pathD - world.egoPathD;
        object.isAhead = object.relativeS > 0.0;
        // 객체 중심이 아니라 OBB의 경로 법선 방향 반폭까지 포함한다.
        // 차선 경계에 조금 걸친 장애물도 해당 주행 회랑의 장애물이다.
        const double headingDelta = object.raw.heading - world.ego.heading;
        const double objectLateralHalfExtent = 0.5 * (
            std::abs(std::sin(headingDelta)) * object.raw.length +
            std::abs(std::cos(headingDelta)) * object.raw.width);
        const double allowedLateral =
            0.5 * std::max(world.egoLaneWidth, 0.0) +
            objectLateralHalfExtent + lateralMargin;
        object.isOnEgoPath = std::abs(object.relativeD) <= allowedLateral;
        const double longitudinalSpeed =
            objectVelocityX(object) * std::cos(world.ego.heading) +
            objectVelocityY(object) * std::sin(world.ego.heading);
        const double closingSpeed = world.egoSpeed - longitudinalSpeed;
        if (object.relativeS <= 0.0) {
            object.ttc = 0.0;
        } else if (closingSpeed > 1e-6) {
            object.ttc = object.relativeS / closingSpeed;
        }
    }
    return true;
}
