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

bool hasCrossTrafficVehicleBoundingBox(const WorldObjectState& object,
                                       const PlanningBehaviorConfig& config) {
    const double length = static_cast<double>(object.raw.length);
    const double width = static_cast<double>(object.raw.width);
    // The parked-adjacent filter intentionally stops at passenger-car size,
    // but cross traffic also includes vans, buses and trucks.  Reusing the
    // 7.5 m passenger-car ceiling excluded an 8.7 m crossing truck in M13.
    constexpr double kMaximumRoadVehicleLength = 20.0;
    constexpr double kMaximumRoadVehicleWidth = 4.0;
    return length >= config.stationaryVehicleMinimumLength &&
           width >= config.stationaryVehicleMinimumWidth &&
           length <= kMaximumRoadVehicleLength &&
           width <= kMaximumRoadVehicleWidth &&
           length >= width * 1.2;
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

// 교차도로의 정차 차량은 현재 속도가 0이라는 이유만으로 안전하지 않다.
// Ego와 객체의 heading 직선 교점을 구해, 둘 다 교점에 접근하는 방향이고
// 차량형 OBB일 때만 주의속도를 적용한다. 같은 방향 주차차량과 이미 교차점을
// 지나 멀어지는 차량에는 적용되지 않는다.
const WorldObjectState* findCrossTrafficThreat(
    const WorldState& world,
    const PlanningBehaviorConfig& config,
    int safelyAvoidedObjectId) {
    const double egoX=std::cos(world.ego.heading);
    const double egoY=std::sin(world.ego.heading);
    const WorldObjectState* nearest=nullptr;
    double nearestIntersection=std::numeric_limits<double>::infinity();
    for(const auto& object:world.objects) {
        if(static_cast<int>(object.raw.id)==safelyAvoidedObjectId ||
           !hasCrossTrafficVehicleBoundingBox(object,config) ||
           std::abs(object.raw.z-world.ego.z)>3.0) continue;
        const double motionHeading=objectMotionHeading(object);
        const double objectX=std::cos(motionHeading);
        const double objectY=std::sin(motionHeading);
        const double determinant=egoX*objectY-egoY*objectX;
        if(std::abs(determinant)<
           std::sin(config.crossTrafficMinimumAngle)) continue;
        const double relativeX=object.raw.x-world.ego.x;
        const double relativeY=object.raw.y-world.ego.y;
        const double egoToIntersection=
            (relativeX*objectY-relativeY*objectX)/determinant;
        const double objectToIntersection=
            (relativeX*egoY-relativeY*egoX)/determinant;
        const double passedAllowance=.5*double(object.raw.length)+1.0;
        if(egoToIntersection<0.0 ||
           egoToIntersection>config.crossTrafficForwardDistance ||
           objectToIntersection< -passedAllowance ||
           objectToIntersection>config.crossTrafficApproachDistance) continue;
        if(egoToIntersection<nearestIntersection) {
            nearestIntersection=egoToIntersection;
            nearest=&object;
        }
    }
    return nearest;
}

// 앞차 속도를 Ego 진행방향 성분으로 환산한다.
double longitudinalSpeedOf(const WorldObjectState& object,
                           const WorldState& world) {
    return objectVelocityX(object) * std::cos(world.ego.heading) +
           objectVelocityY(object) * std::sin(world.ego.heading);
}

}  // namespace

void selectStopDecision(StopDecision& output,
                        const StopManagerData& manager,
                        double egoRouteS) {
    output = {};
    double nearestS = std::numeric_limits<double>::infinity();
    for (const auto& request : manager.requests) {
        if (!request.active || request.stopRouteS >= nearestS) {
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

    // Recovery는 정지 상태가 아니라 이전 프레임의 경로를 새로운
    // road/section 기준 경로로 교체하는 내부 상태다. 맵매칭 경계에서
    // 0.5초씩 속도 0을 내면 같은 경계를 못 넘고 매 프레임 재진입한다.
    // 법규 정지와 비상제동은 각각 StopManager/Safety에서 별도로
    // 보장하므로, Recovery 자체는 즉시 해제하고 주행을 지속한다.
    if (state.health == SystemHealth::Recovery) {
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
        const bool trafficStop = stop.reason == StopReason::RedLight ||
            stop.reason == StopReason::FlashingRed ||
            stop.reason == StopReason::CrosswalkBlocked;
        const double margin = trafficStop ? config.trafficStopPositionMargin
                                          : config.stopSafetyMargin;
        const double distance =
            std::max(0.0, stop.distanceToStop - margin);
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
        const double objectResponseFloor = std::min(
            limitedCruise, config.minimumObjectResponseSpeed);
        const double nearbyTarget = std::min(
            {limitedCruise, config.nearbyObjectCautionSpeed,
             std::max(objectResponseFloor,
                      longitudinalSpeedOf(*nearby, world))});
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

    const WorldObjectState* crossTraffic=findCrossTrafficThreat(
        world,config,state.safelyAvoidedObjectId);
    if(crossTraffic!=nullptr && config.crossTrafficCautionSpeed<targetSpeed) {
        targetSpeed=config.crossTrafficCautionSpeed;
        selectedMode=LonMode::Follow;
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
            double leadTarget = std::clamp(
                leadSpeed + config.followGapGain * gapError,
                0.0, limitedCruise);
            // 법규 정지는 위에서 계산한 stop 속도가 그대로 우선한다.
            // 선행 객체로 인한 ACC 제약만은 비상제동 전까지 15 km/h
            // 아래로 내리지 않는다.
            leadTarget = std::min(
                limitedCruise,
                std::max(leadTarget, config.minimumObjectResponseSpeed));
            if (leadTarget < targetSpeed) {
                targetSpeed = leadTarget;
                selectedMode = LonMode::Follow;
            }
        }
    }

    // Do not exempt the currently avoided object: dense-scene speed must
    // survive a manoeuvre replacement/completion and crosswalk cap release.
    std::vector<const WorldObjectState*> closeStatic;
    bool aheadStatic=false, dense=false;
    const double c=std::cos(world.ego.heading),s=std::sin(world.ego.heading);
    for(const auto& object:world.objects) {
        if(!std::isfinite(object.raw.speed) || std::abs(object.raw.speed)>=0.3) continue;
        const double dx=object.raw.x-world.ego.x,dy=object.raw.y-world.ego.y;
        const double along=dx*c+dy*s, lateral=-dx*s+dy*c;
        if(!std::isfinite(along) || !std::isfinite(lateral) || along < -5.0 ||
           along>config.denseStaticForwardDistance ||
           std::abs(lateral)>config.denseStaticLateralDistance) continue;
        aheadStatic|=along>=0.0;
        for(const auto* other:closeStatic)
            if(other->raw.id!=object.raw.id &&
               std::hypot(object.raw.x-other->raw.x,object.raw.y-other->raw.y)<=config.denseStaticPairDistance)
                dense=true;
        closeStatic.push_back(&object);
    }
    if(aheadStatic && (dense || state.denseStaticHoldRemaining>0.0))
        state.denseStaticHoldRemaining=config.denseStaticReleaseTime;
    else
        state.denseStaticHoldRemaining=std::max(0.0,state.denseStaticHoldRemaining-std::max(0.0,dt));
    if(state.denseStaticHoldRemaining>0.0)
        targetSpeed=std::min(targetSpeed,config.denseStaticSpeed);

    state.longitudinal = selectedMode;
    output.longitudinal = selectedMode;
    output.targetSpeed = targetSpeed;
}
