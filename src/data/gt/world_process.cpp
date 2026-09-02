#include "data/gt/world_process.hpp"

#include "data/gt/gt_processor.hpp"
#include "planning/collision/collision_checker.hpp"
#include "utils/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace {

bool finiteEgo(const EgoState& ego) {
    return std::isfinite(ego.x) && std::isfinite(ego.y) &&
           std::isfinite(ego.heading);
}

bool objectInsideAnyCrosswalk(const WorldObjectState& object,
                              const StaticZoneData& zones) {
    for (const auto& crosswalk : zones.crosswalks) {
        if (containsPoint(crosswalk.polygon,
                          {object.raw.x, object.raw.y})) {
            return true;
        }
        for (const auto& corner : object.obb.corners) {
            if (containsPoint(crosswalk.polygon, corner)) return true;
        }
    }
    return false;
}

void updateEgoMotion(WorldProcessData& data,
                     const GtFrame& gt,
                     double receiveTime,
                     double frameDt) {
    data.state.receiveTime = receiveTime;
    data.state.respawnDetected = false;
    if (!data.hasPreviousFrame) {
        data.filteredSpeed = 0.0;
        return;
    }
    const double dt = frameDt;
    const double dx = gt.ego.x - data.previousX;
    const double dy = gt.ego.y - data.previousY;
    const double jumpDistance = std::hypot(dx, dy);
    // 동일한 VTD simTime의 중복 GT는 물리 시간이 흐르지 않았다. 기존 속도를
    // 유지하고, 다음 실제 프레임에서 누적 simTime으로 이동량을 나눈다.
    if (dt <= 1e-6) {
        return;
    }
    if (jumpDistance > data.config.respawnJumpDistance) {
        data.filteredSpeed = 0.0;
        data.state.respawnDetected = jumpDistance > data.config.respawnJumpDistance;
        return;
    }
    const double rawSpeed =
        (dx * std::cos(gt.ego.heading) + dy * std::sin(gt.ego.heading)) / dt;
    if (!std::isfinite(rawSpeed) ||
        std::abs(rawSpeed) > data.config.maxValidSpeed) {
        data.filteredSpeed = 0.0;
        data.state.respawnDetected = true;
        return;
    }
    const double alpha = std::clamp(data.config.speedFilterAlpha, 0.0, 1.0);
    data.filteredSpeed = alpha * rawSpeed +
                         (1.0 - alpha) * data.filteredSpeed;
}

void updateObjectRelations(WorldState& world,
                           const StaticZoneData& zones,
                           double lateralMargin) {
    for (auto& object : world.objects) {
        if (world.egoPathValid && object.pathValid) {
            object.relativeS = object.pathS - world.egoPathS;
            object.relativeD = object.pathD - world.egoPathD;
            object.isAhead = object.relativeS > 0.0;
            const double allowedLateral =
                0.5 * std::max(world.egoLaneWidth, 0.0) + lateralMargin;
            object.isOnEgoPath = std::abs(object.relativeD) <= allowedLateral;
            const double objectLongitudinalSpeed =
                static_cast<double>(object.raw.speed) *
                std::cos(object.raw.heading - world.ego.heading);
            object.ttc = calculateTTC(
                object.relativeS,
                world.egoSpeed - objectLongitudinalSpeed);
        }
        object.isInsideCrosswalk = objectInsideAnyCrosswalk(object, zones);
        object.isCollisionRisk = intersects(world.egoObb, object.obb);
    }
}

}  // namespace

void runWorldProcess(WorldProcessData& data,
                     const GtFrame& gt,
                     double receiveTime,
                     double frameDt,
                     const PathGeneratorData& map,
                     const std::vector<Waypoint>& globalPath,
                     const VehicleGeometry& vehicle) {
    std::optional<double> expectedRouteS;
    if (data.hasPreviousFrame && data.state.egoPathValid &&
        std::hypot(gt.ego.x - data.previousX,
                   gt.ego.y - data.previousY) <=
            data.config.respawnJumpDistance) {
        expectedRouteS = data.state.egoPathS;
    }
    data.state = makeWorldState(gt, map, globalPath, expectedRouteS);
    updateEgoMotion(data, gt, receiveTime, frameDt);
    data.state.egoSpeed = data.filteredSpeed;
    data.state.inputValid = finiteEgo(gt.ego) && data.state.egoLaneValid &&
                                data.state.egoPathValid;

    const double centerOffset = 0.5 *
        (vehicle.rearAxleToFrontBumper - vehicle.rearAxleToRearBumper);
    const double centerX = gt.ego.x + centerOffset * std::cos(gt.ego.heading);
    const double centerY = gt.ego.y + centerOffset * std::sin(gt.ego.heading);
    data.state.egoObb = makeObb2D(
        centerX, centerY, gt.ego.heading,
        vehicle.rearAxleToFrontBumper + vehicle.rearAxleToRearBumper,
        vehicle.width);
    data.state.frontBumperPathS = data.state.egoPathValid
                                      ? data.state.egoPathS +
                                            vehicle.rearAxleToFrontBumper
                                      : 0.0;
    updateObjectRelations(data.state, map.staticZones,
                          data.config.pathLateralMargin);

    data.previousX = gt.ego.x;
    data.previousY = gt.ego.y;
    data.previousReceiveTime = receiveTime;
    data.hasPreviousFrame = true;
}
