#include "planning/behavior/behavior_planner.hpp"
#include "data/gt/gt_structs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

// 현재 차선 전방에서 가장 가까운 객체를 찾는다.
const WorldObjectState* findLeadObject(const WorldState& world,
                                      bool staticObstacleAvoidanceActive,
                                      int safelyAvoidedObjectId) {
    const WorldObjectState* lead = nullptr;
    for (const auto& object : world.objects) {
        if (!object.isAhead || !object.isOnEgoPath) continue;
        if (staticObstacleAvoidanceActive &&
            std::abs(object.raw.speed) < 0.3) {
            continue;
        }
        if (static_cast<int>(object.raw.id) == safelyAvoidedObjectId) continue;
        if (lead == nullptr || object.relativeS < lead->relativeS) {
            lead = &object;
        }
    }
    return lead;
}

// 차량 중심이 아직 Ego 차선 안에 안 들어왔더라도 옆 차선에서
// 가까이 나란히 달리면 끼어들기 후 제동거리를 미리 확보한다.
const WorldObjectState* findNearbyForwardObject(
    const WorldState& world,
    double maximumDistance,
    double lateralDistance,
    int safelyAvoidedObjectId) {
    const WorldObjectState* nearest = nullptr;
    for (const auto& object : world.objects) {
        // 이 로직은 끼어드는 이동 차량을 위한 사전 감속이다.
        // 콘·표지물 같은 정지 객체를 옆 차선 차량으로 오인하지 않는다.
        if (std::abs(object.raw.speed) < 0.3 ||
            !object.pathValid || !object.isAhead ||
            object.relativeS >= maximumDistance ||
            std::abs(object.relativeD) > lateralDistance) {
            continue;
        }
        if (static_cast<int>(object.raw.id) == safelyAvoidedObjectId) continue;
        if (nearest == nullptr || object.relativeS < nearest->relativeS) {
            nearest = &object;
        }
    }
    return nearest;
}

bool hasVehicleSizedBoundingBox(const WorldObjectState& object,
                                const PlanningBehaviorConfig& config) {
    const double length = static_cast<double>(object.raw.length);
    const double width = static_cast<double>(object.raw.width);
    return length >= config.stationaryVehicleMinimumLength &&
           length <= config.stationaryVehicleMaximumLength &&
           width >= config.stationaryVehicleMinimumWidth &&
           width <= config.stationaryVehicleMaximumWidth &&
           length >= width * 1.4;
}

// 움직임만으로는 끼어들기 의도를 알 수 없는 정차 차량을 별도로 찾는다.
// 모든 정적 물체를 감속 대상으로 만들지 않도록 차량형 OBB, 같은 방향의 바로
// 옆 차선, 짧은 전방 거리 조건을 모두 만족해야 한다.
const WorldObjectState* findStationaryAdjacentVehicle(
    const WorldState& world,
    const PlanningBehaviorConfig& config,
    int safelyAvoidedObjectId) {
    if (!world.egoLaneValid || world.egoLaneId == 0) return nullptr;

    const WorldObjectState* nearest = nullptr;
    for (const auto& object : world.objects) {
        const bool sameLaneDirection =
            object.laneId != 0 &&
            ((object.laneId > 0) == (world.egoLaneId > 0));
        const bool immediatelyAdjacent =
            std::abs(object.laneId - world.egoLaneId) == 1;
        const bool alignedWithEgo =
            std::cos(static_cast<double>(object.raw.heading) -
                     static_cast<double>(world.ego.heading)) > 0.7;
        if (std::abs(object.raw.speed) >= 0.3 ||
            !object.laneValid || !object.pathValid || !object.isAhead ||
            object.isOnEgoPath ||
            object.roadId != world.egoRoadId ||
            object.sectionIndex != world.egoSectionIndex ||
            !sameLaneDirection || !immediatelyAdjacent || !alignedWithEgo ||
            object.relativeS >
                config.stationaryAdjacentVehicleMaximumDistance ||
            std::abs(object.relativeD) >
                config.nearbyObjectLateralDistance ||
            !hasVehicleSizedBoundingBox(object, config)) {
            continue;
        }
        if (static_cast<int>(object.raw.id) == safelyAvoidedObjectId) continue;
        if (nearest == nullptr || object.relativeS < nearest->relativeS) {
            nearest = &object;
        }
    }
    return nearest;
}

// 앞차 속도를 Ego 진행방향 성분으로 환산한다.
double longitudinalSpeedOf(const WorldObjectState& object,
                           const WorldState& world) {
    return static_cast<double>(object.raw.speed) *
           std::cos(static_cast<double>(object.raw.heading) -
                    static_cast<double>(world.ego.heading));
}

}  // namespace

void selectStopDecision(StopDecision& output,
                        const StopManagerData& manager,
                        double egoRouteS,
                        bool ignoreObstacleStops) {
    output = {};
    double nearestS = std::numeric_limits<double>::infinity();
    for (const auto& request : manager.requests) {
        if (!request.active ||
            (ignoreObstacleStops &&
             request.reason == StopReason::Obstacle) ||
            request.stopRouteS >= nearestS) {
            continue;
        }
        nearestS = request.stopRouteS;
        output.valid = true;
        output.reason = request.reason;
        output.stopRouteS = request.stopRouteS;
        // 이미 정지점을 지났으면 음수가 되고, 목표속도는 0으로 유지된다.
        output.distanceToStop = request.stopRouteS - egoRouteS;
        output.sourceId = request.sourceId;
    }
}

void updateBehavior(BehaviorState& state,
                    BehaviorDecision& output,
                    const ConstraintSet& constraints,
                    const StopDecision& stop,
                    const WorldState& world,
                    double cruiseSpeed,
                    const PlanningBehaviorConfig& config,
                    double dt) {
    output = {};

    // Recovery는 진입 즉시 해제되면 맵매칭이 안정될 시간이 없으므로
    // 최소 유지시간 동안은 정지 상태를 유지한다.
    if (state.health == SystemHealth::Recovery) {
        state.recoveryHoldTime += std::max(0.0, dt);
        if (state.recoveryHoldTime < config.recoveryMinimumHoldTime) {
            state.longitudinal = LonMode::Stop;
            output.longitudinal = LonMode::Stop;
            output.health = SystemHealth::Recovery;
            output.targetSpeed = 0.0;
            return;
        }
        state.health = SystemHealth::Ok;
    }
    state.recoveryHoldTime = 0.0;
    output.health = state.health;

    const double limitedCruise =
        constraints.speedLimit > 0.0
            ? std::min(cruiseSpeed, constraints.speedLimit)
            : cruiseSpeed;

    double targetSpeed = limitedCruise;
    LonMode selectedMode = stop.valid ? LonMode::Stop : LonMode::Cruise;

    // 정지 요청은 다른 종방향 위험 검사를 끝내는 우선순위가 아니다.
    // 모든 속도 제약을 계산한 뒤 가장 보수적인(가장 낮은) 값을 적용한다.
    // 예: 먼 적색신호 정지계획이 있어도 가까운 끼어들기 차량은 즉시 감속한다.
    if (stop.valid) {
        const double distance =
            std::max(0.0, stop.distanceToStop - config.stopSafetyMargin);
        targetSpeed = std::min(
            limitedCruise,
            std::sqrt(2.0 * config.comfortableDeceleration * distance));
    }

    // 현재 속도에 비례해 탐색거리를 늘린다. 50 km/h에서 기본값은
    // 5 m + 1.8 s * 13.9 m/s = 약 30 m이다.
    const double cautionDistance =
        config.minimumFollowDistance +
        config.followTimeGap * std::max(0.0, world.egoSpeed);
    const WorldObjectState* nearby = findNearbyForwardObject(
        world, cautionDistance,
        config.nearbyObjectLateralDistance,
        state.safelyAvoidedObjectId);
    if (nearby != nullptr) {
        const double nearbyTarget = std::min(
            {limitedCruise, config.nearbyObjectCautionSpeed,
             std::max(0.0, longitudinalSpeedOf(*nearby, world))});
        if (nearbyTarget < targetSpeed) {
            targetSpeed = nearbyTarget;
            selectedMode = LonMode::Follow;
        }
    }

    const WorldObjectState* stationaryAdjacent =
        findStationaryAdjacentVehicle(
            world, config, state.safelyAvoidedObjectId);
    if (stationaryAdjacent != nullptr &&
        config.stationaryAdjacentVehicleCautionSpeed < targetSpeed) {
        targetSpeed = config.stationaryAdjacentVehicleCautionSpeed;
        selectedMode = LonMode::Follow;
    }

    // Follow 진입과 해제에 서로 다른 임계값을 써서 경계에서의 진동을 막는다.
    const WorldObjectState* lead =
        findLeadObject(world, state.staticObstacleAvoidanceActive,
                       state.safelyAvoidedObjectId);
    if (lead != nullptr) {
        const double egoSpeed = std::max(0.0, world.egoSpeed);
        const double leadSpeed = std::clamp(
            longitudinalSpeedOf(*lead, world), 0.0, limitedCruise);
        const double desiredGap =
            config.minimumFollowDistance +
            config.followTimeGap * egoSpeed;
        // 시간간격만 보면 급제동하는 선행차가 desiredGap 안에 들어올 때까지
        // 반응하지 않는다. 두 차량의 정지거리 차이도 탐색 문턱에 더해,
        // 선행차 속도가 빠르게 떨어지는 순간부터 미리 감속한다.
        const double relativeStoppingDistance = std::max(
            0.0, (egoSpeed * egoSpeed - leadSpeed * leadSpeed) /
                     (2.0 * config.comfortableDeceleration));
        const double brakingAwareGap = desiredGap + relativeStoppingDistance;
        const double threshold =
            state.longitudinal == LonMode::Follow
                ? brakingAwareGap * config.followReleaseRatio
                : brakingAwareGap;
        if (lead->relativeS < threshold) {
            // 선행차 속도로 한 번에 떨어뜨리지 않고 간격 여유만큼 속도를
            // 허용한다. 목표 간격보다 가까우면 자연스럽게 선행차 속도 이하가
            // 되고, 멀면 점진적으로 순항속도까지 허용된다.
            const double gapError = lead->relativeS - desiredGap;
            const double leadTarget = std::clamp(
                leadSpeed + config.followGapGain * gapError,
                0.0, limitedCruise);
            if (leadTarget < targetSpeed) {
                targetSpeed = leadTarget;
                selectedMode = LonMode::Follow;
            }
        }
    }

    state.longitudinal = selectedMode;
    output.longitudinal = selectedMode;
    output.targetSpeed = targetSpeed;
}
