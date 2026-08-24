#include "data/gt/gt_processor.hpp"

#include <array>
#include <cmath>
#include <algorithm>
#include <limits>

Obb2D makeObjectObb(const ObjectState& object) {
    const double halfLength = object.length * 0.5;
    const double halfWidth = object.width * 0.5;

    // Object 중심 기준 로컬 좌표
    const std::array<Point2D, 4> localCorners{{
        { halfLength,  halfWidth},
        { halfLength, -halfWidth},
        {-halfLength, -halfWidth},
        {-halfLength,  halfWidth}
    }};

    const double cosHeading = std::cos(object.heading);
    const double sinHeading = std::sin(object.heading);

    Obb2D obb;

    for (std::size_t i = 0; i < localCorners.size(); ++i) {
        const double localX = localCorners[i].x;
        const double localY = localCorners[i].y;

        obb.corners[i].x =
            object.x
            + localX * cosHeading
            - localY * sinHeading;

        obb.corners[i].y =
            object.y
            + localX * sinHeading
            + localY * cosHeading;
    }

    return obb;
}

void projectObjectToPath(
    WorldObjectState& state,
    const std::vector<Waypoint>& path) {

    if (path.size() < 2) {
        state.pathValid = false;
        return;
    }

    double bestDistance = std::numeric_limits<double>::max();
    double bestS = 0.0;
    double bestD = 0.0;

    const double objectX = state.raw.x;
    const double objectY = state.raw.y;

    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Waypoint& p0 = path[i];
        const Waypoint& p1 = path[i + 1];

        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;

        const double lengthSquared = dx * dx + dy * dy;

        if (lengthSquared < 1e-9) {
            continue;
        }

        // Object를 현재 path segment에 정사영
        double t =
            ((objectX - p0.x) * dx +
             (objectY - p0.y) * dy)
            / lengthSquared;

        // Segment 범위를 벗어나지 않도록 제한
        t = std::clamp(t, 0.0, 1.0);

        const double projectedX = p0.x + t * dx;
        const double projectedY = p0.y + t * dy;

        const double offsetX = objectX - projectedX;
        const double offsetY = objectY - projectedY;

        const double distance =
            std::hypot(offsetX, offsetY);

        if (distance < bestDistance) {
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

    if (bestDistance < std::numeric_limits<double>::max()) {
        state.pathValid = true;
        state.pathS = bestS;
        state.pathD = bestD;
    }
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
    const LaneMatch match = matchWorldToLane(
        mapData,
        object.x,
        object.y,
        static_cast<double>(object.heading));

    state.laneValid = match.valid;

    if (match.valid) {
        state.roadId = match.roadId;
        state.laneId = match.laneId;
    }

    // Global Path 기준 Frenet 좌표
    projectObjectToPath(state, globalPath);

    return state;
}

WorldState makeWorldState(
    const GtFrame& gt,
    const PathGeneratorData& mapData,
    const std::vector<Waypoint>& globalPath) {

    WorldState world;
    world.ego = gt.ego;
    world.trafficLight = gt.trafficLight;

    for (const auto& object : gt.objects) {
        if (object.id == 0) {
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