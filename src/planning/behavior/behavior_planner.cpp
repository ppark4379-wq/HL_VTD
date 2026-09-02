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

    if (stop.valid) {
        state.longitudinal = LonMode::Stop;
        output.longitudinal = LonMode::Stop;
        const double distance =
            std::max(0.0, stop.distanceToStop - config.stopSafetyMargin);
        output.targetSpeed = std::min(
            limitedCruise,
            std::sqrt(2.0 * config.comfortableDeceleration * distance));
        return;
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
        state.longitudinal = LonMode::Follow;
        output.longitudinal = LonMode::Follow;
        output.targetSpeed = std::min(
            {limitedCruise, config.nearbyObjectCautionSpeed,
             std::max(0.0, longitudinalSpeedOf(*nearby, world))});
        return;
    }

    // Follow 진입과 해제에 서로 다른 임계값을 써서 경계에서의 진동을 막는다.
    const WorldObjectState* lead =
        findLeadObject(world, state.staticObstacleAvoidanceActive,
                       state.safelyAvoidedObjectId);
    if (lead != nullptr) {
        const double desiredGap =
            config.minimumFollowDistance +
            config.followTimeGap * std::max(0.0, world.egoSpeed);
        const double threshold =
            state.longitudinal == LonMode::Follow
                ? desiredGap * config.followReleaseRatio
                : desiredGap;
        if (lead->relativeS < threshold) {
            state.longitudinal = LonMode::Follow;
            output.longitudinal = LonMode::Follow;
            output.targetSpeed = std::clamp(
                longitudinalSpeedOf(*lead, world), 0.0, limitedCruise);
            return;
        }
    }

    state.longitudinal = LonMode::Cruise;
    output.longitudinal = LonMode::Cruise;
    output.targetSpeed = limitedCruise;
}
