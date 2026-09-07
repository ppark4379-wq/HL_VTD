#include "data/gt/world_process.hpp"

#include "data/gt/gt_processor.hpp"
#include "planning/collision/collision_checker.hpp"
#include "utils/geometry.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace {

constexpr double kMaximumMissionLaneHeadingError = 1.05;

bool finiteEgo(const EgoState& ego) {
    return std::isfinite(ego.x) && std::isfinite(ego.y) &&
           std::isfinite(ego.heading);
}

std::optional<LaneMatch> missionPreferredLane(
    const PathGeneratorData& map,
    const MissionCorridorData& mission,
    double x,
    double y,
    double heading) {
    const MissionLegData* leg = activeMissionLeg(mission);
    if (leg == nullptr) return std::nullopt;
    const std::vector<LaneMatch> candidates = findContainingLanes(map, x, y);
    const LaneMatch* best = nullptr;
    double bestScore = std::numeric_limits<double>::infinity();
    for (const LaneMatch& candidate : candidates) {
        const LaneKey key{candidate.roadId, candidate.sectionIndex,
                         candidate.laneId};
        if (!leg->allowedLanes.count(key) &&
            !leg->targetLanes.count(key)) {
            continue;
        }
        const auto segment = map.laneSegments.find(key);
        if (segment == map.laneSegments.end() ||
            segment->second.points.empty()) {
            continue;
        }
        const Waypoint& point = segment->second.points[std::min(
            candidate.pointIndex, segment->second.points.size() - 1)];
        const double headingError = std::abs(normalizeAngle(
            point.yaw - heading));
        // 겹치는 접속도로에서 약 90도 틀어진 lane을 단순히 polygon 안에
        // 있다는 이유로 선택하면, 정상 주행 중 local path가 옆 도로로
        // 순간 전환된다. 실제 차량 진행방향과 60도 이상 다른 lane은
        // corridor 안이어도 현재 lane 후보가 아니다.
        if (headingError > kMaximumMissionLaneHeadingError) continue;
        const double score = candidate.distance + 2.0 * headingError;
        if (score < bestScore) {
            bestScore = score;
            best = &candidate;
        }
    }
    return best == nullptr ? std::nullopt
                           : std::optional<LaneMatch>(*best);
}

bool objectInsideAnyCrosswalk(const WorldObjectState& object,
                              const StaticZoneData& zones) {
    for (const auto& crosswalk : zones.crosswalks) {
        if (!crosswalk.routeRelevant) continue;
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
        // A single invalid derivative is normally a clock/packet alignment
        // anomaly.  Keep the last physical estimate; only the explicit
        // distance test above is allowed to classify a real respawn.
        return;
    }
    const double alpha = std::clamp(data.config.speedFilterAlpha, 0.0, 1.0);
    const double filteredCandidate = alpha * std::max(0.0, rawSpeed) +
                                     (1.0 - alpha) * data.filteredSpeed;
    const double maximumIncrease =
        std::max(0.0, data.config.maxSpeedIncreaseRate) * dt;
    const double maximumDecrease =
        std::max(0.0, data.config.maxSpeedDecreaseRate) * dt;
    data.filteredSpeed = std::clamp(
        filteredCandidate,
        std::max(0.0, data.filteredSpeed - maximumDecrease),
        data.filteredSpeed + maximumIncrease);
}

void updateObjectRelations(WorldState& world,
                           const StaticZoneData& zones,
                           double lateralMargin) {
    for (auto& object : world.objects) {
        if (world.egoPathValid && object.pathValid) {
            object.relativeS = object.pathS - world.egoPathS;
            object.relativeD = object.pathD - world.egoPathD;
            object.isAhead = object.relativeS > 0.0;
            const double headingDelta =
                object.raw.heading - world.ego.heading;
            const double objectLateralHalfExtent = 0.5 * (
                std::abs(std::sin(headingDelta)) * object.raw.length +
                std::abs(std::cos(headingDelta)) * object.raw.width);
            const double allowedLateral =
                0.5 * std::max(world.egoLaneWidth, 0.0) +
                objectLateralHalfExtent + lateralMargin;
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
                     const VehicleGeometry& vehicle,
                     const MissionCorridorData* mission) {
    std::optional<double> expectedRouteS;
    if (data.hasPreviousFrame && data.state.egoPathValid &&
        std::hypot(gt.ego.x - data.previousX,
                   gt.ego.y - data.previousY) <=
            data.config.respawnJumpDistance) {
        expectedRouteS = data.state.egoPathS;
    }
    const bool respawnJump = data.hasPreviousFrame &&
        std::hypot(gt.ego.x - data.previousX, gt.ego.y - data.previousY) >
            data.config.respawnJumpDistance;
    if (respawnJump) resetMapMatchState(data.egoMapMatch);

    const bool missionActive = mission != nullptr &&
        activeMissionLeg(*mission) != nullptr;
    data.state = makeWorldState(gt, map, globalPath, expectedRouteS,
                                !missionActive);
    std::optional<LaneMatch> routePreferred;
    if (missionActive) {
        routePreferred = missionPreferredLane(
            map, *mission, gt.ego.x, gt.ego.y, gt.ego.heading);
    } else {
        LaneMatch preferred;
        preferred.valid = data.state.egoLaneValid;
        preferred.roadId = data.state.egoRoadId;
        preferred.laneId = data.state.egoLaneId;
        preferred.sectionIndex = data.state.egoSectionIndex;
        preferred.roadS = data.state.egoRoadS;
        preferred.distance = data.state.egoLaneCenterDistance;
        preferred.laneWidth = data.state.egoLaneWidth;
        routePreferred = preferred;
    }
    matchWorldPosition(data.egoMapMatch, map, gt.ego.x, gt.ego.y,
                       static_cast<double>(gt.ego.heading), routePreferred);
    const LaneMatch& stableMatch = data.egoMapMatch.lastMatch;
    data.state.egoLaneValid = stableMatch.valid;
    if (stableMatch.valid) {
        data.state.egoRoadId = stableMatch.roadId;
        data.state.egoLaneId = stableMatch.laneId;
        data.state.egoSectionIndex = stableMatch.sectionIndex;
        data.state.egoRoadS = stableMatch.roadS;
        data.state.egoLaneWidth = stableMatch.laneWidth;
        data.state.egoLaneCenterDistance = stableMatch.distance;
    }
    updateEgoMotion(data, gt, receiveTime, frameDt);
    data.state.egoSpeed = data.filteredSpeed;
    // checkpoint mission에서 guide route는 정지선/구역의 순서를 붙이는
    // 보조자료일 뿐이다. 자유 corridor의 다른 차선을 주행해 guide route
    // 투영이 실패해도 센서 입력이나 주행 자체를 invalid로 만들지 않는다.
    // 실제 추종경로 투영 유효성은 planning 단계에서 별도로 확인한다.
    data.state.inputValid = finiteEgo(gt.ego) && data.state.egoLaneValid &&
                            (missionActive || data.state.egoPathValid);

    const double centerOffset = 0.5 *
        (vehicle.rearAxleToFrontBumper - vehicle.rearAxleToRearBumper);
    data.state.frontBumperOffset = vehicle.rearAxleToFrontBumper;
    const double centerX = gt.ego.x + centerOffset * std::cos(gt.ego.heading);
    const double centerY = gt.ego.y + centerOffset * std::sin(gt.ego.heading);
    data.state.egoObb = makeObb2D(
        centerX, centerY, gt.ego.heading,
        vehicle.rearAxleToFrontBumper + vehicle.rearAxleToRearBumper,
        vehicle.width);
    data.state.frontBumperPathS = data.state.egoPathValid
                                      ? data.state.egoPathS +
                                            data.state.frontBumperOffset
                                      : 0.0;
    updateObjectRelations(data.state, map.staticZones,
                          data.config.pathLateralMargin);

    data.previousX = gt.ego.x;
    data.previousY = gt.ego.y;
    data.previousReceiveTime = receiveTime;
    data.hasPreviousFrame = true;
}
