#include "planning/planning_process.hpp"
#include "planning/local/recovery_selection.hpp"
#include "planning/local/local_fallback_speed.hpp"

#include "data/gt/gt_processor.hpp"
#include "logger/decision_trace.hpp"
#include "planning/behavior/behavior_planner.hpp"
#include "planning/collision/collision_checker.hpp"
#include "planning/local/local_path_planner.hpp"
#include "planning/local/lane_change_planner.hpp"
#include "planning/local/corridor_lattice.hpp"
#include "planning/local/path_planner.hpp"
#include "utils/geometry.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

namespace {

double localManeuverSpeedLimit(const LaneChangeData& maneuver,
                              const LaneChangeConfig& config,
                              const PathPlannerConfig& planner,const GtFrame& gt) {
    if(maneuver.path.size()<2) return config.localFallbackSpeed;
    // Match this frame's tracking projection, not last frame's index: passing
    // the last sharp sample must not unlock 15 after predicting only at 5.
    const auto nearest=findNearestWaypoint(maneuver.path,gt,maneuver.nearestIndex);
    return localFallbackSpeedLimit(config,maneuver.validatedObstacleClearance,
        findMaxCurvature(maneuver.path,nearest,planner.curvatureCheckDistance),
        planner.maximumLateralAcceleration);
}

bool longitudinallyAdjacent(const PathGeneratorData& map,
                            const LaneKey& first,
                            const LaneKey& second) {
    if (first == second) return true;
    const auto hasForwardEdge = [&](const LaneKey& from,
                                    const LaneKey& to) {
        // 같은 section의 옆 차선 edge는 실제 section/road 전환이 아니다.
        if (from.roadId == to.roadId &&
            from.sectionIndex == to.sectionIndex &&
            from.laneId != to.laneId) {
            return false;
        }
        const auto found = map.laneGraph.find(from);
        return found != map.laneGraph.end() &&
            std::find(found->second.begin(), found->second.end(), to) !=
                found->second.end();
    };
    // 경계 샘플에서 맵매칭과 경로 투영의 전환 타이밍이 한 프레임
    // 어긋나도 버티도록 정방향과 역방향 직접 successor를 모두 인정한다.
    return hasForwardEdge(first, second) || hasForwardEdge(second, first);
}

bool trackingPathUsable(const PathPlannerData& planner,
                        const WorldState& sensedWorld,
                        const PathGeneratorData& map,
                        double& lateralDistance,
                        double& remainingDistance,
                        bool requireForwardHorizon = true) {
    lateralDistance = std::numeric_limits<double>::infinity();
    remainingDistance = 0.0;
    if (planner.waypoints.size() < 2 ||
        !std::isfinite(sensedWorld.ego.x) ||
        !std::isfinite(sensedWorld.ego.y) ||
        !std::isfinite(sensedWorld.ego.heading)) {
        return false;
    }

    WorldState projected = sensedWorld;
    if (!projectWorldToActivePath(
            projected, planner.waypoints,
            sensedWorld.frontBumperOffset)) {
        return false;
    }
    lateralDistance = std::abs(projected.egoPathD);
    // A lane-ID match does not connect a displaced pose to this trajectory.
    // Let the checked reentry planner bridge larger offsets before tracking.
    const double maximumLateralDistance = std::max(
        1.25, 0.5 * sensedWorld.egoLaneWidth);
    if (lateralDistance > maximumLateralDistance) return false;

    const auto nearest = std::lower_bound(
        planner.waypoints.begin(), planner.waypoints.end(),
        projected.egoPathS,
        [](const Waypoint& point, double pathS) {
            return point.s < pathS;
        });
    const Waypoint& pathPoint = nearest == planner.waypoints.end()
        ? planner.waypoints.back() : *nearest;
    if (std::abs(normalizeAngle(
            sensedWorld.ego.heading - pathPoint.yaw)) > 1.35) {
        return false;
    }

    if (sensedWorld.egoLaneValid && pathPoint.roadId != 0 &&
        pathPoint.laneId != 0) {
        const LaneKey sensedLane{
            sensedWorld.egoRoadId, sensedWorld.egoSectionIndex,
            sensedWorld.egoLaneId};
        const LaneKey pathLane{
            pathPoint.roadId, pathPoint.sectionIndex, pathPoint.laneId};
        if (!longitudinallyAdjacent(map, sensedLane, pathLane) &&
            !corridorHasLongitudinalConnection(map, sensedLane, pathLane) &&
            !corridorCommittedRouteConnects(map, sensedLane, pathLane)) {
            return false;
        }
    }

    remainingDistance = planner.waypoints.back().s - projected.egoPathS;
    const auto& end = planner.waypoints.back();
    if (remainingDistance <= .25 &&
        (end.x-sensedWorld.ego.x)*std::cos(end.yaw) +
        (end.y-sensedWorld.ego.y)*std::sin(end.yaw) <= 0.) return false;
    // 새 경로도 종점 직전의 짧은 tail이면 채택하지 않는다. gateway/section
    // 전환 경로가 미리 준비돼야만 교체되고, 그 전까지는 기존 정상 경로를
    // 유지한다. 이렇게 해야 짧은 끝점을 쫓는 급조향과 local-path 공백을
    // 동시에 막을 수 있다.
    const double requiredAhead =
        std::max(2.0, std::abs(sensedWorld.egoSpeed) * 0.35);
    if (!requireForwardHorizon) {
        // section 끝에서 successor/Gateway 경로가 아직 준비되지
        // 않았다고 현재 차선의 짧은 tail까지 폐기하면 계획
        // 자체가 무효화되고 다음 경로를 만들 입력도 사라진다.
        // 차량 위치가 아직 경로 종점을 지나지 않았다면
        // 연속성 fallback으로 사용한다.
        return remainingDistance >= -0.25;
    }
    return remainingDistance >= requiredAhead;
}

// Metadata/topology disagreement must not erase a physically continuous path
// that was valid in the previous frame.  This fallback deliberately checks
// geometry only; it is used only for paths produced from the precomputed XODR
// corridor, whose lateral curves already passed road-mark and swept-surface
// validation.  It prevents a one-frame lane-ID lead/lag from becoming an
// indefinite `plan_valid=0` stop.
bool corridorGeometryContinuityUsable(const PathPlannerData& planner,
                                      const WorldState& sensedWorld,
                                      double& lateralDistance,
                                      double& remainingDistance) {
    lateralDistance=std::numeric_limits<double>::infinity();
    remainingDistance=0.;
    if(planner.waypoints.size()<2 ||
       !std::isfinite(sensedWorld.ego.x) ||
       !std::isfinite(sensedWorld.ego.y) ||
       !std::isfinite(sensedWorld.ego.heading)) return false;
    WorldState projected=sensedWorld;
    if(!projectWorldToActivePath(
            projected,planner.waypoints,sensedWorld.frontBumperOffset))
        return false;
    lateralDistance=std::abs(projected.egoPathD);
    const auto nearest=std::lower_bound(
        planner.waypoints.begin(),planner.waypoints.end(),projected.egoPathS,
        [](const Waypoint& point,double pathS){return point.s<pathS;});
    const auto& point=nearest==planner.waypoints.end()
        ? planner.waypoints.back():*nearest;
    const double headingError=std::abs(normalizeAngle(
        sensedWorld.ego.heading-point.yaw));
    remainingDistance=planner.waypoints.back().s-projected.egoPathS;
    const auto& end=planner.waypoints.back();
    const double endpointAhead=(end.x-sensedWorld.ego.x)*std::cos(end.yaw)+
        (end.y-sensedWorld.ego.y)*std::sin(end.yaw);
    return lateralDistance<=std::max(1.25,.5*sensedWorld.egoLaneWidth) &&
           headingError<=.8 && remainingDistance>=0. &&
           (remainingDistance>.25 || endpointAhead>0.);
}

// GT trafficLight.id는 XODR <controller> id다. 개별 signal id가 아니다.
//   <controller id="11"> <control signalId="54"/> <control signalId="55"/>
//                        <control signalId="58"/> </controller>
// signal 54/55/58은 적·황·녹 개별 램프이고, 패킷이 보내는 11이 논리 신호등이다.
//
// 신호 상태로 정지를 만들 정지선은 반드시 controller id가 일치해야 한다.
// road/lane만으로 보조 매칭하면, 무관한 신호등이 적색일 때 같은 도로의
// 무신호 정지선(일시정지선 등)을 붙잡고 멈추는 오검출이 생긴다.
const StopLine* findActiveStopLine(const WorldState& world,
                                   const StaticZoneData& zones,
                                   double behindWindow,
                                   double aheadWindow) {
    // 같은 road/lane에 있는 정지선을 우선하되, 못 찾으면 controller만 맞는
    // 정지선을 쓴다.
    //
    // 정지선 바로 앞은 교차로 진입부라서 맵매칭이 현재 road와 연결 road
    // 사이에서 흔들린다. 실측에서 route_s 109.1 지점에 정지한 순간 매칭이
    // road 418/lane 4 -> road 3340/lane -3 으로 넘어갔고, road/lane 조건만
    // 쓰면 그 프레임에 정지 요청이 사라져 적색신호에 그대로 출발했다.
    // controller id는 패킷이 준 값과 정확히 대응하므로 이것만으로도
    // "지금 지켜야 할 신호"는 확정된다.
    const StopLine* laneMatched = nullptr;
    const StopLine* controllerMatched = nullptr;
    double laneMatchedDistance = std::numeric_limits<double>::infinity();
    double controllerMatchedDistance = std::numeric_limits<double>::infinity();

    for (const auto& line : zones.stopLines) {
        if (!line.routeRelevant) continue;
        if (line.controllerId < 0) continue;
        if (line.controllerId != world.trafficLight.id) continue;
        const double distance = line.routeS - world.frontBumperPathS;
        if (distance < -behindWindow || distance > aheadWindow) continue;

        if (distance < controllerMatchedDistance) {
            controllerMatched = &line;
            controllerMatchedDistance = distance;
        }

        const bool roadMatches =
            line.roadId < 0 || line.roadId == world.egoRoadId;
        const bool laneMatches =
            line.laneIds.empty() ||
            std::find(line.laneIds.begin(), line.laneIds.end(),
                      world.egoLaneId) != line.laneIds.end();
        if (roadMatches && laneMatches && distance < laneMatchedDistance) {
            laneMatched = &line;
            laneMatchedDistance = distance;
        }
    }
    return laneMatched != nullptr ? laneMatched : controllerMatched;
}

void addStopRequest(StopManagerData& manager, StopReason reason,
                    double routeS, int sourceId, double holdTime = 0.0) {
    manager.requests.push_back({reason, true, routeS, holdTime, sourceId});
}

// 처음부터 멈춰 있는 물체는 고정 장애물로 보고 즉시 회피한다.
// 반면 한 번이라도 움직이던 차량은 끼어들기/급정거에 대비해 먼저
// 추종·긴급제동으로 반응하고, 짧은 시간 계속 정지했을 때만 회피
// 대상으로 전환한다.
std::vector<int> updateConfirmedStationaryObjects(
    PlanningProcessData& data, const WorldState& world, double dt) {
    std::vector<StationaryObjectState> current;
    std::vector<int> confirmed;
    current.reserve(world.objects.size());

    const double observationDt = std::isfinite(dt) ? std::max(0., dt) : 0.;
    const double elapsed = std::min(.1, observationDt);
    if (world.respawnDetected) data.stationaryObjects.clear();
    for (const auto& object : world.objects) {
        const auto previous = std::find_if(
            data.stationaryObjects.begin(), data.stationaryObjects.end(),
            [&](const StationaryObjectState& state) {
                return state.id == object.raw.id;
            });
        const bool wasTracked = previous != data.stationaryObjects.end();
        const double displacementSpeed =
            wasTracked && previous->hasPosition && observationDt > 1e-3
                ? std::hypot(object.raw.x - previous->lastX,
                             object.raw.y - previous->lastY) / observationDt
                : 0.0;
        const bool smallActor = object.raw.length > 0. && object.raw.width > 0. &&
            std::max(object.raw.length, object.raw.width) <= 2.5;
        const double movingThreshold = smallActor ? .1 : .5;
        const double stationaryThreshold = smallActor ? .1 : .3;
        const double motion = std::max(std::abs(static_cast<double>(object.raw.speed)),
                                       objectMotionSpeed(object));
        const bool moving = motion >= movingThreshold || displacementSpeed >= movingThreshold;
        const bool stationary = motion < stationaryThreshold &&
                                (!wasTracked || displacementSpeed < stationaryThreshold);
        const bool hasEverMoved = moving ||
            (wasTracked && previous->hasEverMoved);
        const double stationaryDuration = stationary
            ? (wasTracked ? previous->stationaryDuration : 0.0) + elapsed
            : 0.0;

        current.push_back({object.raw.id, stationaryDuration,
                           static_cast<double>(object.raw.x),
                           static_cast<double>(object.raw.y), true,
                           hasEverMoved});
        if (stationary &&
            (!hasEverMoved || stationaryDuration >=
                                  data.laneChangeConfig
                                      .stoppedDynamicConfirmationTime)) {
            confirmed.push_back(static_cast<int>(object.raw.id));
        }
    }
    for (auto previous : data.stationaryObjects) {
        if (std::any_of(current.begin(), current.end(),
            [&](const auto& state) { return state.id == previous.id; })) continue;
        previous.missingDuration += elapsed;
        if (previous.missingDuration > data.config.emergencyMinimumHoldTime) continue;
        previous.stationaryDuration = 0.; // missing telemetry is not stationary evidence
        previous.hasPosition = false;
        current.push_back(previous);
    }
    data.stationaryObjects = std::move(current);
    return confirmed;
}

bool updateCrossingObjectYields(PlanningProcessData& data, const WorldState& world,
    const std::vector<Waypoint>& route, std::size_t nearestIndex,
    const PathGeneratorData& map, double dt, DecisionTraceData* trace) {
    auto& waits = data.crossingObjectYields;
    if (world.respawnDetected) { waits.clear(); return false; }
    const double elapsed = std::isfinite(dt) ? std::clamp(dt, 0., .1) : 0.;
    waits.erase(std::remove_if(waits.begin(), waits.end(), [&](auto& wait) {
        const auto object = std::find_if(world.objects.begin(), world.objects.end(),
            [&](const auto& o) { return o.raw.id == wait.id; });
        if (object == world.objects.end()) {
            wait.missingDuration += elapsed;
            return wait.missingDuration > data.config.emergencyMinimumHoldTime;
        }
        wait.missingDuration = 0.;
        const auto history = std::find_if(data.stationaryObjects.begin(), data.stationaryObjects.end(),
            [&](const auto& s) { return s.id == wait.id; });
        if (history != data.stationaryObjects.end() && history->hasPosition &&
            history->stationaryDuration + 1e-9 >= data.laneChangeConfig.stoppedDynamicConfirmationTime) {
            traceDecision(trace, "crossing_yield", "stationary_avoidance_allowed",
                          "object=" + std::to_string(wait.id));
            return true;
        }
        const double c = std::cos(wait.heading), s = std::sin(wait.heading);
        double low = std::numeric_limits<double>::infinity(), high = -low;
        for (const auto& p : object->obb.corners) {
            const double d = -(p.x-wait.x)*s + (p.y-wait.y)*c;
            low = std::min(low,d); high = std::max(high,d);
        }
        const bool crossed = wait.entrySide > 0. ? high < -wait.halfWidth : low > wait.halfWidth;
        const double center = .5*(low+high);
        const double lateralSpeed = -objectVelocityX(*object)*s + objectVelocityY(*object)*c;
        const bool returned = (low > wait.halfWidth || high < -wait.halfWidth) &&
            center*lateralSpeed > .01;
        if (crossed || returned)
            traceDecision(trace, "crossing_yield", "body_cleared", "object=" + std::to_string(wait.id));
        return crossed || returned;
    }), waits.end());

    const double speed = std::max(0., world.egoSpeed);
    const double forwardLimit = std::max(data.config.crosswalkApproachDistance,
        speed*data.config.emergencyReactionTime + speed*speed/(2.*data.config.emergencyBrakingDeceleration) +
        data.config.minimumFollowDistance);
    const auto body = vehicleFootprintFromObb(world.egoObb,
        {world.ego.x, world.ego.y, world.ego.heading});
    const double halfWidth = .5*std::max({3., world.egoLaneWidth, body.width});
    const std::size_t start = route.empty() ? 0 : std::min(nearestIndex,route.size()-1);
    const double origin = route.empty() ? 0. : route[start].s;
    for (const auto& object : world.objects) {
        if (object.raw.length <= 0. || object.raw.width <= 0. ||
            std::max(object.raw.length,object.raw.width) > 2.5 || objectMotionSpeed(object) < .1 ||
            std::any_of(waits.begin(), waits.end(), [&](const auto& w) { return w.id == object.raw.id; })) continue;
        // Use a bounded window of the already selected reference. No new
        // global search, lane match or dependency on a marked crosswalk.
        Waypoint nearest;
        nearest.x=world.ego.x; nearest.y=world.ego.y; nearest.yaw=world.ego.heading;
        nearest.roadId=world.egoRoadId; nearest.laneId=world.egoLaneId;
        double along=(object.raw.x-world.ego.x)*std::cos(nearest.yaw)+
                     (object.raw.y-world.ego.y)*std::sin(nearest.yaw);
        if (!route.empty()) {
            double best=std::numeric_limits<double>::infinity();
            for (std::size_t i=start; i<route.size() && route[i].s-origin<=forwardLimit; ++i) {
                const double distance=std::hypot(route[i].x-object.raw.x,route[i].y-object.raw.y);
                if (distance<best) {best=distance; nearest=route[i];}
            }
            along=nearest.s-origin+(object.raw.x-nearest.x)*std::cos(nearest.yaw)+
                                  (object.raw.y-nearest.y)*std::sin(nearest.yaw);
        }
        if (along <= 0. || along > forwardLimit ||
            !roadPointSharesObjectLevel(map,nearest,object.raw.x,object.raw.y,object.raw.z)) continue;
        const double c=std::cos(nearest.yaw), s=std::sin(nearest.yaw);
        const double lateral=-(object.raw.x-nearest.x)*s+(object.raw.y-nearest.y)*c;
        const double lateralSpeed=-objectVelocityX(object)*s+objectVelocityY(object)*c;
        double low=std::numeric_limits<double>::infinity(), high=-low;
        for (const auto& p:object.obb.corners) {
            const double d=-(p.x-nearest.x)*s+(p.y-nearest.y)*c;
            low=std::min(low,d); high=std::max(high,d);
        }
        const bool overlapping=low<=halfWidth && high>=-halfWidth;
        const bool approaching=lateral*lateralSpeed < -.01 &&
            std::abs(lateral)<=halfWidth+data.config.crosswalkNearbyObjectDistance;
        if (!overlapping && !approaching) continue;
        const double entrySide=std::abs(lateral)>.01 ? lateral : -lateralSpeed;
        waits.push_back({object.raw.id,nearest.x,nearest.y,nearest.yaw,halfWidth,entrySide});
        traceDecision(trace, "crossing_yield", "motion_started",
                      "object=" + std::to_string(object.raw.id));
    }
    return !waits.empty();
}

}  // namespace

double settledStraightManeuverSpeedCap(const LaneChangeData& maneuver,
    const WorldState& world, const std::vector<Waypoint>& path,
    double cruiseSpeed, double clearance, const LaneChangeConfig& laneConfig,
    const PlanningBehaviorConfig& config) {
    const double slow=laneConfig.maximumLaneChangeSpeed;
    if ((maneuver.phase!=LaneChangePhase::Execute && maneuver.phase!=LaneChangePhase::Settle) ||
        maneuver.waitingForReturn || maneuver.bestProgressFallback ||
        !maneuver.shiftCompletedLaneValid || !world.egoLaneValid ||
        !(LaneKey{world.egoRoadId,world.egoSectionIndex,world.egoLaneId}==maneuver.shiftCompletedLane) ||
        !std::isfinite(world.egoLaneCenterDistance) || std::abs(world.egoLaneCenterDistance)>.35 ||
        !std::isfinite(clearance) || clearance+1e-6<laneConfig.latticeObstacleClearance ||
        path.size()<2 || !std::isfinite(cruiseSpeed) || cruiseSpeed<=slow) return slow;
    const double c=std::cos(world.ego.heading),s=std::sin(world.ego.heading);
    const double reserve=8.+std::max(cruiseSpeed,world.egoSpeed)*.5;
    const double preview=std::max(20.,reserve+
        (cruiseSpeed*cruiseSpeed-slow*slow)/(2.*config.comfortableDeceleration));
    std::size_t nearest=0;double best=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<path.size();++i) {
        const double d=std::hypot(path[i].x-world.ego.x,path[i].y-world.ego.y);
        if(d<best){best=d;nearest=i;}
    }
    if(best>1.) return slow;
    double available=0.;
    for(std::size_t i=nearest;i<path.size();++i) {
        const auto& p=path[i];
        const double dx=p.x-world.ego.x,dy=p.y-world.ego.y;
        const double ahead=dx*c+dy*s, lateral=-dx*s+dy*c;
        const double yawError=std::atan2(std::sin(p.yaw-world.ego.heading),
                                         std::cos(p.yaw-world.ego.heading));
        if (p.roadId!=world.egoRoadId || p.sectionIndex!=world.egoSectionIndex ||
            p.laneId!=world.egoLaneId || !std::isfinite(ahead) ||
            std::abs(lateral)>.35 || std::abs(yawError)>.05 || ahead<available-1.) break;
        available=std::max(available,ahead);
        if(available>=preview)break;
    }
    // Any nearby body restricts this optional speed relaxation. Existing ACC,
    // cross-traffic, dense-obstacle, crossing and emergency caps remain below.
    for(const auto& object:world.objects) {
        const double dx=object.raw.x-world.ego.x,dy=object.raw.y-world.ego.y;
        const double ahead=dx*c+dy*s,lateral=-dx*s+dy*c;
        const double radius=.5*std::hypot(object.raw.length,object.raw.width);
        if(ahead+radius>=-5. && std::abs(lateral)<=config.nearbyObjectLateralDistance+radius)
            available=std::min(available,std::max(0.,ahead-radius));
    }
    return std::min(cruiseSpeed,std::sqrt(slow*slow+2.*config.comfortableDeceleration*
                                        std::max(0.,available-reserve)));
}

bool queuedGatewayBypassIsComplete(const std::vector<Waypoint>& path,
    const MissionCorridorData& mission) {
    const auto* leg=activeMissionLeg(mission);
    if(!leg || path.size()<2) return false;
    bool gatewayPassed=leg->gatewayLanes.empty();
    for(const auto& p:path) {
        gatewayPassed=gatewayPassed || leg->gatewayLanes.count(
            LaneKey{p.roadId,p.sectionIndex,p.laneId})!=0;
        if(gatewayPassed && missionPointIsTarget(*leg,p)) return true;
    }
    return false;
}

double soleGatewayQueueDistance(const WorldState& world,
    const PathGeneratorData& map, const MissionCorridorData& mission) {
    const double none=std::numeric_limits<double>::infinity();
    const auto* leg=activeMissionLeg(mission);
    if(!map.corridorRuntime || !leg || leg->gatewayLanes.empty()) return none;
    const auto& r=*map.corridorRuntime;
    if(r.activeLeg!=mission.activeLegIndex || r.gatewayQueueBlockers.empty() ||
       r.activeGatewayObserved) return none;
    // After actual entrance, ordinary following/contact recovery owns the
    // vehicle. A stale entrance-queue latch must not demand the old signal
    // again when VTD stops reporting it inside the connector.
    // These IDs come from a completed full-route search with all stationary
    // obstacles and the vehicle footprint. Pending work is not a failure;
    // stop-line distance is not a separate wait/bypass decision.
    if(r.committedGlobalPathReachesGateway && r.soleGatewayApproachPath.empty() &&
       corridorTrackingPathIsSafe(r.committedGlobalPath,world,
           r.globalPathClearance(),nullptr,nullptr,&map)) return none;
    const auto body=vehicleFootprintFromObb(world.egoObb,
        {world.ego.x,world.ego.y,world.ego.heading});
    double closestQueue=none;
    for(const auto& o:world.objects) {
        if(std::find(r.gatewayQueueBlockers.begin(),r.gatewayQueueBlockers.end(),
                     static_cast<int>(o.raw.id))==r.gatewayQueueBlockers.end() ||
           objectMotionSpeed(o)>=.3 || std::max(o.raw.length,o.raw.width)<=2.5) continue;
        const double yaw=normalizeAngle(o.raw.heading-world.ego.heading);
        if(std::abs(yaw)>.7) continue;
        const double dx=o.raw.x-world.ego.x,dy=o.raw.y-world.ego.y;
        const double forward=dx*std::cos(world.ego.heading)+dy*std::sin(world.ego.heading);
        const double side=-dx*std::sin(world.ego.heading)+dy*std::cos(world.ego.heading);
        // A passed vehicle cannot stop Ego; a still-adjacent entrance queue
        // retains its mission constraint. This is following geometry only.
        if(forward+.5*o.raw.length < -body.frontExtent() ||
           std::abs(side)>std::max(8.,2.*world.egoLaneWidth)) continue;
        closestQueue=std::min(closestQueue,std::max(0.,forward-body.frontExtent()-
            .5*(o.raw.length*std::abs(std::cos(yaw))+o.raw.width*std::abs(std::sin(yaw)))));
    }
    return closestQueue;
}

std::vector<int> updateCrosswalkStationaryRelease(const Crosswalk& crossing,
    const WorldState& world, const StopDecision& previousStop,
    const std::vector<StationaryObjectState>& history,
    const PlanningBehaviorConfig& config, double dt,
    std::vector<CrosswalkObjectWaitState>& waits) {
    std::vector<int> result;
    if (config.simulationCrosswalkStaticRelease != 1.) { waits.clear(); return result; }
    const bool waitingHere = world.inputValid && world.egoSpeed <= config.stoppedSpeedThreshold &&
        previousStop.valid && previousStop.reason == StopReason::CrosswalkBlocked &&
        previousStop.sourceId == crossing.id &&
        std::isfinite(previousStop.distanceToStop) &&
        previousStop.distanceToStop <= config.trafficStopPositionMargin + .5;
    // A dropped frame/large dt must not instantly complete a three-second wait.
    const double elapsed = std::isfinite(dt) ? std::clamp(dt,0.,.1) : 0.;
    // The 30-object stream may temporarily omit a cone. Absence is not
    // motion and must not erase a completed yield or a pedestrian's history.
    // These records are scoped to this crossing and removed after passing it.
    const bool crossingYieldCompleted=std::any_of(waits.begin(),waits.end(),
        [&](const auto& wait){return wait.crosswalkId==crossing.id && wait.released;});
    for (const auto& object : world.objects) {
        const bool inside=intersects(crossing.polygon,object.obb);
        auto wait=std::find_if(waits.begin(),waits.end(),[&](const CrosswalkObjectWaitState& s){
            return s.crosswalkId==crossing.id && s.objectId==object.raw.id;});
        if (wait==waits.end()) {
            waits.push_back({crossing.id,object.raw.id});wait=waits.end()-1;
        }
        const auto tracked = std::find_if(history.begin(), history.end(),
            [&](const StationaryObjectState& state) { return state.id == object.raw.id; });
        const bool stationary=std::abs(object.raw.speed)<.1 && objectMotionSpeed(object)<.1;
        const bool displaced=wait->hasPosition &&
            std::hypot(object.raw.x-wait->lastX,object.raw.y-wait->lastY)>.15;
        wait->hasEverMoved = wait->hasEverMoved || !stationary || displaced ||
            (tracked!=history.end() && tracked->hasEverMoved);
        wait->hasPosition=true;
        wait->lastX=object.raw.x;wait->lastY=object.raw.y;
        wait->stationaryObservation=stationary && !wait->hasEverMoved
            ? wait->stationaryObservation+elapsed : 0.;
        if (!inside) continue; // remember approach motion, but never time out an approaching actor
        if (wait->hasEverMoved || tracked==history.end() || !tracked->hasPosition) {
            wait->released=false;wait->stoppedWait=0.;continue;
        }
        if (!wait->released) {
            wait->stoppedWait=waitingHere ? wait->stoppedWait+elapsed : 0.;
            // After one completed stop at this crossing, later static IDs
            // need observation confirmation, not another complete stop.
            // Never apply this to an actor that has moved, even if now still.
            wait->released=wait->stoppedWait>=config.crosswalkStaticConfirmationTime ||
                (crossingYieldCompleted &&
                 wait->stationaryObservation>=config.crosswalkStaticConfirmationTime);
        }
        if (wait->released) result.push_back(static_cast<int>(object.raw.id));
    }
    return result;
}

bool crosswalkHasRiskObject(const Crosswalk& crosswalk,
                            const WorldState& world,
                            double predictionHorizon,
                            const std::vector<int>& staticPropIds) {
    return std::any_of(
        world.objects.begin(), world.objects.end(),
        [&](const WorldObjectState& object) {
            if (std::find(staticPropIds.begin(), staticPropIds.end(),
                          static_cast<int>(object.raw.id)) != staticPropIds.end()) return false;
            // Do not turn a normally moving lead vehicle into a pedestrian
            // yield request after Ego has already entered the crossing.
            // Vehicle following/prediction/EBS remain independent and active.
            // GT lacks semantic class: retain the existing conservative size
            // boundary for pedestrians, cyclists and other small road users.
            constexpr double kVulnerableRoadUserMaximumSize = 2.5;
            const double maximumSize = std::max(
                static_cast<double>(object.raw.length),
                static_cast<double>(object.raw.width));
            if (maximumSize > kVulnerableRoadUserMaximumSize) return false;
            // 중심점이 횡단보도 밖으로 나가도 보행자 체적의 뒤쪽이
            // 남아 있을 수 있다. polygon과 전체 OBB가 한 부분이라도
            // 겹치는 동안은 점유로 판정한다.
            if (intersects(crosswalk.polygon, object.obb)) return true;
            if (predictionHorizon <= 0.0 ||
                objectMotionSpeed(object) < 0.1) return false;
            // horizon 끝점 하나만 보면 그 전에 횡단보도를 통과한 빠른 객체를
            // 놓친다. 0.25초 간격으로 궤적 전체를 검사한다.
            constexpr double kPredictionStep = 0.25;
            for (double time = kPredictionStep;
                 time <= predictionHorizon + 1e-9;
                 time += kPredictionStep) {
                const double predictedX =
                    object.raw.x + objectVelocityX(object) * time;
                const double predictedY =
                    object.raw.y + objectVelocityY(object) * time;
                const Obb2D predictedObb = makeObb2D(
                    predictedX, predictedY, object.raw.heading,
                    object.raw.length, object.raw.width);
                if (intersects(crosswalk.polygon, predictedObb)) {
                    return true;
                }
            }
            return false;
        });
}

double crosswalkStopRouteS(const Crosswalk& crossing, const WorldState& world,
    const StaticZoneData& zones, const std::vector<Waypoint>& route, double searchDistance) {
    double best=-std::numeric_limits<double>::infinity();
    for (const auto& line:zones.stopLines) {
        if (!line.routeRelevant || line.routeS>crossing.entryRouteS+1e-3 ||
            crossing.entryRouteS-line.routeS>searchDistance) continue;
        auto point=std::lower_bound(route.begin(),route.end(),line.routeS,
            [](const Waypoint& p,double s){return p.s<s;});
        if (point==route.end() && !route.empty()) --point;
        if (point!=route.begin() && point!=route.end() &&
            std::abs((point-1)->s-line.routeS)<std::abs(point->s-line.routeS)) --point;
        const bool onCurrentRoad=line.roadId<0 || line.roadId==world.egoRoadId;
        const bool onApproachRoute=point!=route.end() && point->roadId==line.roadId;
        // A crosswalk is often owned by the connecting road, while its stop
        // line belongs to the PRECEDING approach road (M09/M14, among others).
        // Match the approach, not equality with the crosswalk's road ID.
        if (!onCurrentRoad && !onApproachRoute) continue;
        const int lane=onCurrentRoad && world.egoLaneValid ? world.egoLaneId
            : onApproachRoute ? point->laneId : 0;
        if (!line.laneIds.empty() && lane!=0 &&
            std::find(line.laneIds.begin(),line.laneIds.end(),lane)==line.laneIds.end()) continue;
        best=std::max(best,line.routeS); // nearest valid line BEFORE this crossing
    }
    return std::isfinite(best)?best:crossing.entryRouteS;
}

namespace {

CrosswalkOccupancyState* findCrosswalkState(
    PlanningProcessData& data, int crosswalkId) {
    const auto found = std::find_if(
        data.crosswalkStates.begin(), data.crosswalkStates.end(),
        [&](const CrosswalkOccupancyState& state) {
            return state.id == crosswalkId;
        });
    return found == data.crosswalkStates.end() ? nullptr : &*found;
}

void updateCrosswalkStates(PlanningProcessData& data,
                           const WorldState& world,
                           const StaticZoneData& zones,
                           double dt,
                           const StopDecision& previousStop,
                           DecisionTraceData* trace) {
    data.crosswalkObjectWaits.erase(std::remove_if(data.crosswalkObjectWaits.begin(),
        data.crosswalkObjectWaits.end(),[&](const CrosswalkObjectWaitState& wait){
            const auto crossing=std::find_if(zones.crosswalks.begin(),zones.crosswalks.end(),
                [&](const Crosswalk& c){return c.id==wait.crosswalkId;});
            return world.respawnDetected || crossing==zones.crosswalks.end() ||
                !crossing->routeRelevant || (world.egoPathValid && crossing->exitRouteS<world.egoPathS);
        }),data.crosswalkObjectWaits.end());
    data.crosswalkStates.erase(
        std::remove_if(
            data.crosswalkStates.begin(), data.crosswalkStates.end(),
            [&](const CrosswalkOccupancyState& state) {
                const auto crosswalk = std::find_if(
                    zones.crosswalks.begin(), zones.crosswalks.end(),
                    [&](const Crosswalk& candidate) {
                        return candidate.id == state.id;
                    });
                return crosswalk == zones.crosswalks.end() ||
                       !crosswalk->routeRelevant ||
                       (world.egoPathValid &&
                        crosswalk->exitRouteS < world.egoPathS);
            }),
        data.crosswalkStates.end());

    for (const auto& crosswalk : zones.crosswalks) {
        if (!crosswalk.routeRelevant ||
            (world.egoPathValid &&
             crosswalk.exitRouteS < world.egoPathS)) {
            continue;
        }
        CrosswalkOccupancyState* state = findCrosswalkState(data, crosswalk.id);
        const bool requireClearCrosswalk =
            world.trafficLight.state == 6 ||
            (state != nullptr && state->waitUntilClear);
        // A completed flashing-red stop never authorizes passing through a
        // pedestrian who is still on the crossing, even if initially still.
        const auto staticProps = requireClearCrosswalk ? std::vector<int>{} :
            updateCrosswalkStationaryRelease(crosswalk,world,previousStop,
                data.stationaryObjects,data.config,dt,data.crosswalkObjectWaits);
        const bool occupied = crosswalkHasRiskObject(
            crosswalk, world, data.config.crosswalkPedestrianPrediction, staticProps);
        if (occupied) {
            if (state == nullptr) {
                data.crosswalkStates.push_back(
                    {crosswalk.id, true,
                     data.config.crosswalkClearanceHoldTime, requireClearCrosswalk});
                traceDecision(trace, "crosswalk", "occupancy_started",
                              "full_object_obb_overlaps_or_is_predicted_to_overlap");
            } else {
                state->occupied = true;
                state->waitUntilClear = requireClearCrosswalk;
                state->clearanceHoldRemaining =
                    data.config.crosswalkClearanceHoldTime;
            }
            continue;
        }
        if (state == nullptr) continue;
        if (state->occupied) {
            state->occupied = false;
            state->clearanceHoldRemaining =
                data.config.crosswalkClearanceHoldTime;
            std::ostringstream detail;
            detail << "crosswalk=" << crosswalk.id
                   << " stationary_yield_released=" << staticProps.size()
                   << " hold_seconds="
                   << data.config.crosswalkClearanceHoldTime;
            traceDecision(trace, "crosswalk", "clearance_hold_started",
                          detail.str());
        } else {
            state->clearanceHoldRemaining = std::max(
                0.0, state->clearanceHoldRemaining - std::max(0.0, dt));
        }
    }
}

double pointToSegmentDistance(const Point2D& point,
                              const Point2D& begin,
                              const Point2D& end) {
    const double dx = end.x - begin.x;
    const double dy = end.y - begin.y;
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 1e-9) {
        return std::hypot(point.x - begin.x, point.y - begin.y);
    }
    const double t = std::clamp(
        ((point.x - begin.x) * dx + (point.y - begin.y) * dy) /
            lengthSquared,
        0.0, 1.0);
    return std::hypot(point.x - (begin.x + t * dx),
                      point.y - (begin.y + t * dy));
}

bool pointNearCrosswalk(const Crosswalk& crosswalk,
                        const Point2D& point,
                        double margin) {
    if (containsPoint(crosswalk.polygon, point)) return true;
    if (margin <= 0.0 || crosswalk.polygon.size() < 2) return false;
    for (std::size_t index = 0; index < crosswalk.polygon.size(); ++index) {
        const Point2D& begin = crosswalk.polygon[index];
        const Point2D& end =
            crosswalk.polygon[(index + 1) % crosswalk.polygon.size()];
        if (pointToSegmentDistance(point, begin, end) <= margin) return true;
    }
    return false;
}

bool crosswalkHasNearbyObject(const Crosswalk& crosswalk,
                              const WorldState& world,
                              double margin) {
    return std::any_of(
        world.objects.begin(), world.objects.end(),
        [&](const WorldObjectState& object) {
            if (pointNearCrosswalk(crosswalk,
                                   {object.raw.x, object.raw.y}, margin)) {
                return true;
            }
            return std::any_of(
                object.obb.corners.begin(), object.obb.corners.end(),
                [&](const Point2D& corner) {
                    return pointNearCrosswalk(crosswalk, corner, margin);
                });
        });
}

double routeHeadingAtS(const std::vector<Waypoint>& route, double routeS) {
    if (route.empty()) return 0.0;
    const auto found = std::lower_bound(
        route.begin(), route.end(), routeS,
        [](const Waypoint& waypoint, double value) {
            return waypoint.s < value;
        });
    if (found == route.end()) return route.back().yaw;
    return found->yaw;
}

// GT에는 semantic object type이 없으므로 보행자/자전거라는 이름에 의존하지
// 않는다. 대신 취약 도로사용자 크기의 OBB가 경로 쪽으로 횡이동하는 물리적
// 충돌 상황을 잡는다. 지도 횡단보도 매칭이 없거나 객체가 막 움직이기 시작한
// 프레임에도 jerk 제한을 거치지 않고 제동할 수 있는 마지막 안전망이다.
bool crossingObjectRequiresEmergency(
    const WorldState& world,
    const std::vector<Waypoint>& route,
    const PlanningBehaviorConfig& config,
    int& riskObjectId) {
    const double egoSpeed = std::max(0.0, world.egoSpeed);
    const double stoppingEnvelope =
        egoSpeed * config.emergencyReactionTime +
        egoSpeed * egoSpeed /
            (2.0 * config.emergencyBrakingDeceleration) +
        config.minimumFollowDistance;
    for (const auto& object : world.objects) {
        const double length = static_cast<double>(object.raw.length);
        const double width = static_cast<double>(object.raw.width);
        if (!object.pathValid || !object.isAhead ||
            object.relativeS > stoppingEnvelope ||
            std::abs(object.relativeD) >
                0.5 * std::max(3.0, world.egoLaneWidth) + 5.0 ||
            std::abs(object.raw.speed) < 0.3 ||
            length <= 0.0 || width <= 0.0 ||
            // 보행자뿐 아니라 자전거 크기도 포함한다. 승용차급 객체의
            // 끼어들기는 별도 OBB 예측과 선행차 제동 로직이 담당한다.
            std::max(length, width) > 2.5) {
            continue;
        }
        const double routeHeading =
            routeHeadingAtS(route, object.pathS);
        const double lateralVelocity =
            -objectVelocityX(object) * std::sin(routeHeading) +
             objectVelocityY(object) * std::cos(routeHeading);
        const bool movingTowardPath =
            object.relativeD * lateralVelocity < -0.05;
        if (!movingTowardPath || std::abs(lateralVelocity) < 0.25) continue;
        riskObjectId = static_cast<int>(object.raw.id);
        return true;
    }
    return false;
}

// 선행차가 급제동하면 등속 OBB 충돌 TTC가 갱신되기 전에 상대 정지거리가
// 위험해질 수 있다. 반응시간 동안 진행할 거리까지 빼고 필요한 감속도를
// 계산해, 가용 비상제동의 75%를 넘으면 즉시 최종 safety brake를 요청한다.
bool leadObjectRequiresEmergency(const WorldState& world,
                                 const LocalPathData& selectedPath,
                                 const PlanningBehaviorConfig& config,
                                 int& riskObjectId,
                                 bool includeStationary = false) {
    if (!selectedPath.valid || selectedPath.points.empty()) return false;
    const double egoSpeed = std::max(0.0, world.egoSpeed);
    for (const auto& object : world.objects) {
        if (!includeStationary && std::abs(object.raw.speed) < 0.3) {
            continue;
        }

        const Waypoint* nearest = nullptr;
        double nearestDistance = std::numeric_limits<double>::infinity();
        for (const Waypoint& point : selectedPath.points) {
            const double distance = std::hypot(
                point.x - static_cast<double>(object.raw.x),
                point.y - static_cast<double>(object.raw.y));
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = &point;
            }
        }
        const double pathHalfWidth =
            0.5 * std::max(3.0, world.egoLaneWidth) +
            0.5 * std::max(0.0F, object.raw.width);
        const double relativeS = nearest != nullptr
            ? nearest->s - world.egoPathS
            : -std::numeric_limits<double>::infinity();
        if (nearest == nullptr || relativeS <= 0.0 ||
            nearestDistance > pathHalfWidth) {
            continue;
        }
        const double leadSpeed = std::max(
            0.0, objectVelocityX(object) * std::cos(world.ego.heading) +
                     objectVelocityY(object) * std::sin(world.ego.heading));
        const double speedEnergy =
            egoSpeed * egoSpeed - leadSpeed * leadSpeed;
        if (speedEnergy <= 0.0) continue;
        const double usableDistance =
            relativeS - config.minimumFollowDistance -
            egoSpeed * config.emergencyReactionTime;
        if (usableDistance <= 0.0 ||
            speedEnergy / (2.0 * usableDistance) >=
                0.75 * config.emergencyBrakingDeceleration) {
            riskObjectId = static_cast<int>(object.raw.id);
            return true;
        }
    }
    return false;
}

bool flashingRedHoldComplete(StopManagerData& manager,
                             const WorldState& world,
                             const StopLine& line,
                             const PlanningBehaviorConfig& config,
                             double dt) {
    auto& hold = manager.hold;
    if (hold.stopLineId != line.id) {
        hold = {};
        hold.stopLineId = line.id;
    }
    if (hold.holdSatisfied) return true;
    const double distance = line.routeS - world.frontBumperPathS;
    if (world.inputValid && world.egoPathValid && !world.respawnDetected &&
        distance >= 0.0 && distance <= 2.0 &&
        std::abs(world.egoSpeed) <= std::min(.05, config.stoppedSpeedThreshold)) {
        hold.stoppedTime += std::isfinite(dt) ? std::clamp(dt,0.,.1) : 0.;
        hold.holdSatisfied = hold.stoppedTime + 1e-9 >= std::max(.5,config.stopHoldTime);
    } else {
        // Separate short pauses, rolling, and stops beyond the line cannot
        // be accumulated into the required continuous stationary interval.
        hold.stoppedTime = 0.;
    }
    return hold.holdSatisfied;
}

void updateStopRequests(PlanningProcessData& data,
                        const WorldState& world,
                        const StaticZoneData& zones,
                        double dt,
                        DecisionTraceData* trace) {
    data.stopManager.requests.clear();
    const StopLine* line =
        findActiveStopLine(world, zones, data.config.stopLineBehindWindow,
                           data.config.signalStopSearchDistance);
    const int signalState = static_cast<int>(world.trafficLight.state);
    if ((signalState != 6 && signalState != 0) || world.respawnDetected)
        data.stopManager.hold = {};
    else if (signalState == 0 && !data.stopManager.hold.holdSatisfied)
        data.stopManager.hold.stoppedTime = 0.;
    // A transient unassigned packet must not demand a second stop after the
    // same flashing line was already cleared. A new line resets in the hold
    // checker; explicit red/yellow/green and respawn reset the whole latch.

    if (!data.config.ignoreTrafficLights && line != nullptr) {
        if (signalState == 1 || signalState == 2) {
            // Requested policy: yellow has the same stop authority as red.
            addStopRequest(data.stopManager, StopReason::RedLight,
                           line->routeS, line->id);
        } else if (signalState == 6 &&
                   !flashingRedHoldComplete(data.stopManager, world, *line,
                                             data.config, dt)) {
            // GT state 6 carries no blink color. Unmapped flashing signals
            // use the same conservative stop-and-yield policy as flashing red.
            addStopRequest(data.stopManager, StopReason::FlashingRed,
                           line->routeS, line->id,
                           data.config.stopHoldTime);
        }
    }

    for (const auto& crosswalk : zones.crosswalks) {
        if (!crosswalk.routeRelevant) continue;
        // 앞범퍼가 진입한 뒤에도 후륜축이 횡단보도를 완전히 빠져나갈 때까지
        // 점유 검사를 유지한다. 진입점을 넘었다는 이유로 보행자 정지를
        // 해제하면 횡단 중인 보행자 앞으로 다시 출발할 수 있다.
        if (crosswalk.exitRouteS < world.egoPathS) continue;
        const CrosswalkOccupancyState* state =
            findCrosswalkState(data, crosswalk.id);
        const bool stopForOccupancy =
            state != nullptr &&
            (state->occupied || state->clearanceHoldRemaining > 1e-6);
        if (stopForOccupancy) {
            // 횡단보도 위에 1 km/h 이하로 3초 이상 서 있으면 감점이다.
            // 횡단보도 입구를 목표로 삼으면 제동 오차만큼 밀려 횡단보도
            // 위에 서게 되므로, 그 앞 정지선이 있으면 정지선을 목표로 한다.
            // 여유는 정지 프로파일의 stop_safety_margin이 이미 적용한다.
            const double target=crosswalkStopRouteS(crosswalk,world,zones,
                data.pathPlanner.waypoints,data.config.crosswalkStopLineSearch);
            addStopRequest(data.stopManager, StopReason::CrosswalkBlocked,
                           std::max(target, world.frontBumperPathS),
                           crosswalk.id);
            std::ostringstream detail;
            detail << "crosswalk=" << crosswalk.id
                   << " road=" << crosswalk.roadId
                   << " entry_route_s=" << crosswalk.entryRouteS
                   << " stop_route_s=" << target
                   << " front_bumper_path_s=" << world.frontBumperPathS
                   << " occupied=" << state->occupied
                   << " clearance_hold_remaining="
                   << state->clearanceHoldRemaining;
            traceDecision(trace, "crosswalk_stop", "stop_request_created",
                          detail.str());
        }
    }

    // 정지 목표가 횡단보도 안이면, 아직 진입하지 않은 경우에 한해 앞범퍼
    // 목표를 횡단보도 입구로 당긴다. 이미 진입했다면 안전 정지를 우선한다.
    for (auto& request : data.stopManager.requests) {
        for (const auto& crosswalk : zones.crosswalks) {
            if (!crosswalk.routeRelevant) continue;
            const bool targetInside =
                request.stopRouteS > crosswalk.entryRouteS &&
                request.stopRouteS < crosswalk.exitRouteS;
            if (targetInside &&
                world.frontBumperPathS < crosswalk.entryRouteS) {
                request.stopRouteS = crosswalk.entryRouteS;
            }
        }
    }
}

bool evaluateLaneChangeCandidate(PathSafetyResult& safety,
                                 const LaneChangeData& laneChange,
                                 const WorldState& world,
                                 const PlanningBehaviorConfig& config,
                                 double minimumPredictionSpeed,
                                 const PathGeneratorData& map) {
    LocalPathData candidate;
    makeLaneChangeLocalPath(candidate, laneChange);
    resetPathSafety(safety);
    if (!candidate.valid) return false;

    // KeepTarget 경로에는 현재 횡이동 뒤의 successor/connector tail도
    // 포함된다. 그 tail 끝의 다음 장애물이 안전한 첫 회피까지 매 프레임
    // 취소하지 못하도록, 안전 재검사는 현재 횡이동 완료점 뒤 2 m까지만
    // 수행한다. committed path는 자르지 않으며 다음 장애물은 안착 직전의
    // 원자적 재계획이 담당한다.
    if (!laneChange.localCorridorAvoidance &&
        laneChange.routeMode == LaneChangeRouteMode::KeepTargetLane &&
        candidate.points.size() >= 3) {
        const auto end = std::upper_bound(
            candidate.points.begin(), candidate.points.end(),
            laneChange.rejoinRouteS + 2.0,
            [](double value, const Waypoint& point) {
                return value < point.s;
            });
        if (end != candidate.points.end()) {
            const auto keepEnd = std::min(candidate.points.end(), end + 1);
            if (keepEnd >= candidate.points.begin() + 3) {
                candidate.points.erase(keepEnd, candidate.points.end());
            }
        }
    }
    WorldState predictionWorld = world;
    predictionWorld.egoSpeed = std::max(
        {1.0, std::abs(world.egoSpeed), minimumPredictionSpeed});
    const double remainingLength =
        std::max(0.0, candidate.points.back().s - world.egoPathS);
    const double fullManeuverHorizon =
        remainingLength / predictionWorld.egoSpeed +
        config.collisionPredictionStep;
    checkCurrentCollisions(safety, world.egoObb, world.objects,-1,&world,&map);
    if (safety.collision) return false;

    // 등속 예측을 수십 초까지 늘리면 움직이는 차량이 먼 미래에도 같은
    // 속도·방향을 유지한다고 가정해 안전한 회피를 과도하게 막는다. 동적
    // 객체는 설정된 근거리 horizon만, 실제로 경로를 막는 정적 객체는 남은
    // 기동 전체를 검사한다.
    WorldState dynamicWorld = predictionWorld;
    dynamicWorld.objects.erase(
        std::remove_if(dynamicWorld.objects.begin(),
                       dynamicWorld.objects.end(),
                       [](const WorldObjectState& object) {
                           return std::abs(object.raw.speed) < 0.3;
                       }),
        dynamicWorld.objects.end());
    checkPredictedCollisions(safety, dynamicWorld, candidate,
                             config.collisionPredictionHorizon,
                             config.collisionPredictionStep,false,&map);
    if (safety.predictedCollision) return false;

    WorldState staticWorld = predictionWorld;
    staticWorld.objects.erase(
        std::remove_if(staticWorld.objects.begin(),
                       staticWorld.objects.end(),
                       [](const WorldObjectState& object) {
                           return std::abs(object.raw.speed) >= 0.3;
                       }),
        staticWorld.objects.end());
    checkPredictedCollisions(safety, staticWorld, candidate,
                             std::max(config.collisionPredictionHorizon,
                                      fullManeuverHorizon),
                             config.collisionPredictionStep, true,&map);
    return !safety.collision && !safety.predictedCollision;
}

bool crossedFinishLine(PlanningProcessData& data,
                       const WorldState& world,
                       const StaticZoneData& zones,
                       bool pathGoalReached) {
    bool crossed = pathGoalReached;
    if (zones.finish.valid && world.egoPathValid) {
        crossed = data.hasPreviousRouteS &&
                  data.previousRouteS < zones.finish.routeS &&
                  world.egoPathS >= zones.finish.routeS;
    } else if (world.egoPathValid &&
               !data.pathPlanner.waypoints.empty()) {
        // 도로 안 자유 차선 주행에서는 최종 checkpoint와 같은 종방향
        // 위치에 도착해도 옆 차선 중심이라 좌표 거리가 차선 폭만큼
        // 남는다. 마지막 route_s에 도달했으면 목적지를 통과한 것으로
        // 판정해, 끝점을 지나친 뒤 recovery 정지에 빠지지 않게 한다.
        crossed = world.egoPathS >=
            data.pathPlanner.waypoints.back().s -
                data.pathPlanner.config.goalTolerance;
    }
    if (world.egoPathValid) {
        data.previousRouteS = world.egoPathS;
        data.hasPreviousRouteS = true;
    }
    return crossed;
}

// 프레임 중간값을 0이 아니라 NaN/-1로 초기화한다.
// 0으로 두면 "쓰기 전에 읽음" 버그가 그럴듯한 값(정지거리 0 등)으로 조용히
// 통과하지만, NaN이면 즉시 드러난다.
void resetPlanningFrame(PlanningProcessData& data) {
    data.frame = {};
    data.result = {};
    const double poison = std::numeric_limits<double>::quiet_NaN();
    data.frame.constraints.speedLimit = poison;
    data.frame.stop.stopRouteS = poison;
    data.frame.stop.distanceToStop = poison;
    data.frame.behavior.targetSpeed = poison;
    data.frame.pathSafety.minimumTTC = poison;
    data.frame.pathSafety.riskObjectId = -1;
}

// 실제로 추종할 경로가 확정된 뒤 그 경로의 곡률속도를 기본 제한으로 삼는다.
// 회피 중 Ego는 원래 전역경로에서 수 m 떨어지므로, 전역경로의 route-recovery
// 속도를 여기 사용하면 안전한 목표 차선에 안착한 뒤에도 약 7 km/h로 묶인다.
bool updateSpeedConstraints(PlanningProcessData& data,
                            const WorldState& world,
                            const StaticZoneData& zones,
                            double selectedPathSpeed,
                            bool& schoolZoneActive, DecisionTraceData* trace) {
    data.frame.constraints.speedLimit = findZoneSpeedLimit(
        zones, {world.ego.x, world.ego.y},
        world.egoPathValid ? world.egoPathS : -1.0,
        selectedPathSpeed);
    schoolZoneActive = isSchoolZoneActive(
        zones, {world.ego.x, world.ego.y},
        world.egoPathValid ? world.egoPathS : -1.0);

    if (world.egoPathValid) {
        for (const auto& zone : zones.speedZones) {
            if (!zone.routeRelevant || !zone.schoolZone ||
                !zone.hasRouteRange ||
                zone.routeSEnd < world.frontBumperPathS) {
                continue;
            }
            const double distanceToEntry = std::max(
                0.0, zone.routeSBegin - world.frontBumperPathS);
            const double approachLimit = schoolZoneApproachSpeedLimit(
                data.pathPlanner.config, data.config, distanceToEntry, world.egoSpeed);
            if (approachLimit < data.frame.constraints.speedLimit) {
                std::ostringstream detail;
                detail << "distance_to_entry=" << distanceToEntry
                       << " cap_kph=" << approachLimit * 3.6
                       << " approach_target_kph=" << data.pathPlanner.config.schoolZoneApproachSpeed * 3.6
                       << " target_before_entry_m=" << data.config.schoolZoneApproachDistance;
                traceDecision(trace, "speed", "school_zone_prebrake", detail.str());
            }
            data.frame.constraints.speedLimit = std::min(
                data.frame.constraints.speedLimit, approachLimit);
        }
    }
    if (schoolZoneActive) {
        data.frame.constraints.speedLimit = std::min(
            data.frame.constraints.speedLimit,
            data.pathPlanner.config.schoolZoneSpeed);
    }

    bool crosswalkControlActive = false;
    for (const auto& crosswalk : zones.crosswalks) {
        if (!crosswalk.routeRelevant) continue;
        if (crosswalk.exitRouteS < world.egoPathS) {
            continue;
        }
        const CrosswalkOccupancyState* state =
            findCrosswalkState(data, crosswalk.id);
        const bool risk = state != nullptr && state->occupied;
        const bool previouslyActive = state != nullptr;
        const bool nearby = crosswalkHasNearbyObject(
            crosswalk, world, data.config.crosswalkNearbyObjectDistance);
        if (!risk && !nearby && !previouslyActive) {
            continue;
        }
        const double distance = crosswalk.entryRouteS - world.frontBumperPathS;
        if ((risk || nearby) &&
            distance > data.config.crosswalkApproachDistance) {
            continue;
        }
        const double limit = (risk || nearby)
                                 ? data.config.crosswalkApproachSpeed
                                 : data.config.crosswalkClearanceSpeed;
        crosswalkControlActive = true;
        data.frame.constraints.speedLimit =
            data.frame.constraints.speedLimit > 0.0
                ? std::min(data.frame.constraints.speedLimit, limit)
                : limit;
    }
    return crosswalkControlActive;
}

bool recoveryProtectedObject(const WorldObjectState& object,
                             const std::vector<int>& confirmedStationaryObjectIds) {
    const bool vulnerable = object.raw.length <= 2.5 && object.raw.width <= 1.5 &&
        object.raw.height >= 1.;
    return vulnerable || std::find(confirmedStationaryObjectIds.begin(),
        confirmedStationaryObjectIds.end(), static_cast<int>(object.raw.id)) ==
        confirmedStationaryObjectIds.end();
}

// A separate veto on simulation recovery. The ordinary cut-in/lead EBS
// calculations below are unchanged. Include recently stopped actors until
// the existing motion-history confirmation classifies them as stationary.
bool recoveryHasDynamicRisk(const WorldState& world, const LocalPathData& path,
                            const PathGeneratorData& map,
                            const PlanningBehaviorConfig& config,
                            const std::vector<int>& confirmedStationaryObjectIds,
                            int& riskObjectId) {
    // Most static-obstacle recovery frames have no protected actor. Avoid
    // copying the whole object vector just to erase every element from it.
    if (std::none_of(world.objects.begin(), world.objects.end(), [&](const auto& object) {
            return recoveryProtectedObject(object, confirmedStationaryObjectIds);
        })) return false;
    WorldState protectedWorld = world;
    protectedWorld.objects.erase(std::remove_if(protectedWorld.objects.begin(),
        protectedWorld.objects.end(), [&](const auto& object) {
            return !recoveryProtectedObject(object, confirmedStationaryObjectIds);
        }), protectedWorld.objects.end());
    if (protectedWorld.objects.empty()) return false;
    PathSafetyResult safety;
    resetPathSafety(safety);
    checkCurrentCollisions(safety, world.egoObb, protectedWorld.objects, -1, &world, &map);
    updateMinimumTTC(safety, protectedWorld.objects, false, -1, &path, &map);
    checkPredictedCollisions(safety, protectedWorld, path,
        config.collisionPredictionHorizon, config.collisionPredictionStep, true, &map);
    const double threshold = std::max(config.emergencyTTC,
        config.emergencyReactionTime + config.collisionPredictionStep +
        std::max(0., world.egoSpeed) / config.emergencyBrakingDeceleration);
    if (requiresEmergencyBrake(safety, threshold)) {
        riskObjectId = safety.riskObjectId;
        return true;
    }
    return crossingObjectRequiresEmergency(protectedWorld, path.points, config, riskObjectId) ||
        leadObjectRequiresEmergency(protectedWorld, path, config, riskObjectId, true);
}

bool applySimulationProgressRecovery(
    PlanningProcessData& data, const GtFrame& gt, const WorldState& sensedWorld,
    WorldState& world, const PathGeneratorData& map, PathPlan& tracking,
    bool& emergency, bool stationaryPathStop, int stationaryBlocker,
    double selectedPathClearance, bool goalReached, bool gatewayQueue, bool queueGreen,
    double gatewayQueueSpeedCap,
    const std::vector<int>& confirmedStationaryObjectIds,
    double dt, DecisionTraceData* trace) {
    auto& state = data.progressRecovery;
    if (data.config.simulationProgressRecovery != 1. || goalReached ||
        sensedWorld.respawnDetected ||
        !std::isfinite(sensedWorld.ego.x) || !std::isfinite(sensedWorld.ego.y) ||
        !std::isfinite(sensedWorld.ego.z) ||
        !std::isfinite(sensedWorld.ego.heading) || !std::isfinite(sensedWorld.egoSpeed)) {
        state = {};
        return false;
    }
    if (state.missionLeg != data.missionCorridor.activeLegIndex) {
        state = {};
        state.missionLeg = data.missionCorridor.activeLegIndex;
    }
    int dynamicRiskObject = -1;
    const bool unconfirmedBlocker = stationaryPathStop && std::any_of(
        world.objects.begin(), world.objects.end(), [&](const auto& object) {
            return static_cast<int>(object.raw.id) == stationaryBlocker &&
                recoveryProtectedObject(object, confirmedStationaryObjectIds);
        });
    const bool dynamicRisk = unconfirmedBlocker || recoveryHasDynamicRisk(world,
        data.frame.localPath, map, data.config, confirmedStationaryObjectIds, dynamicRiskObject);
    // Recovery addresses a vehicle that remains stuck AFTER danger clears.
    // A real EBS request (including its release hold) or a protected actor
    // must cancel an active escape as well as prevent a new one. Time spent
    // yielding is never authority to override a moving vehicle's collision.
    if (emergency || dynamicRisk) {
        if (state.active || state.blockedTime > 0.)
            traceDecision(trace, "progress_recovery", "waiting_for_dynamic_risk",
                "object=" + std::to_string(unconfirmedBlocker ? stationaryBlocker : dynamicRiskObject) +
                " recovery_timer_reset=1 emergency_preserved=1");
        state = {};
        return false;
    }
    // A paused simulator or one delayed frame must not expire the timer.
    const double elapsed = std::isfinite(dt) ? std::clamp(dt, 0., .1) : 0.;
    const bool stopped = std::abs(sensedWorld.egoSpeed) <= data.config.stoppedSpeedThreshold;
    // Match final command validity. A stored PP plan can be valid while it
    // no longer projects under Ego; the limiter then brakes for invalid input.
    // Treat that geometric failure as a recovery request, not as progress.
    // The independently sensed input guard above still rejects bad telemetry.
    const bool blocked = stationaryPathStop || emergency || dynamicRisk || !tracking.valid || !world.inputValid ||
        (!data.frame.stop.valid && data.frame.behavior.targetSpeed <= .05) || data.frame.pathSafety.collision;
    if (!state.anchorValid || std::hypot(sensedWorld.ego.x - state.anchorX,
                                        sensedWorld.ego.y - state.anchorY) >= .5) {
        state.anchorX = sensedWorld.ego.x; state.anchorY = sensedWorld.ego.y;
        state.anchorValid = true; state.noMotionTime = 0.; state.blockedTime = 0.;
    } else {
        state.noMotionTime += elapsed;
    }
    // Count observed standstill as well as planner stop requests: a positive
    // target with a latched actuator brake is also a failure to make progress.
    if (stopped) state.blockedTime += elapsed;
    else if (!stopped) state.blockedTime = 0.;

    // A signal request is a stop POSITION, not an immediate standstill.
    // Keep recovery geometry available on approach when the ordinary path
    // is exhausted. Its speed is capped below by the same bumper-distance
    // profile as ordinary tracking; the signal remains in the final result.
    // At the line, and for pedestrian/other stops, retain stop authority.
    const bool signalApproach = data.frame.stop.valid &&
        (data.frame.stop.reason == StopReason::RedLight ||
         data.frame.stop.reason == StopReason::FlashingRed) &&
        std::isfinite(data.frame.stop.distanceToStop) &&
        data.frame.stop.distanceToStop > data.config.trafficStopPositionMargin &&
        gatewayQueueSpeedCap > .05;
    if (data.frame.stop.valid && !signalApproach) {
        if (state.active)
            traceDecision(trace, "progress_recovery", "suspended_for_traffic_stop",
                          "commands_blocked=1 static_blockage_history_preserved=1");
        recovery_selection::suspendForTrafficStop(state,sensedWorld.ego.x,sensedWorld.ego.y,blocked);
        return false;
    }

    if (!state.active) {
        if (state.blockedTime < data.config.progressRecoveryStopTimeout) return false;
        state.active = true;
        state.startX = sensedWorld.ego.x; state.startY = sensedWorld.ego.y;
        state.retryRemaining = 0.;
        traceDecision(trace, "progress_recovery", "activated",
            "blocked_seconds=" + std::to_string(state.blockedTime) +
            " emergency=" + std::to_string(emergency));
    }
    if(gatewayQueue && queueGreen) state.gatewayQueueGreenAuthorized=true;

    // A currently certified ordinary trajectory outranks recovery immediately.
    // Waiting for 2m of escape motion and another second can carry Ego past
    // its gateway even though the planned connection is already usable.
    // Match the selected trajectory's actual clearance contract: using the
    // smaller tight margin here released onto a nominally blocked path and
    // caused stop/recovery cycles without any change in the scene.
    const bool ordinaryReady = canResumeOrdinaryPathAfterRecovery(
        sensedWorld, data.frame.localPath.points,
        tracking.valid && world.inputValid && data.frame.localPath.valid && !stationaryPathStop &&
        !emergency && data.frame.behavior.targetSpeed > .05 &&
        !data.frame.pathSafety.collision && !data.frame.pathSafety.predictedCollision) &&
        corridorTrackingPathIsSafe(data.frame.localPath.points, sensedWorld,
            selectedPathClearance, nullptr, &data.config, &map);
    if (ordinaryReady) {
        traceDecision(trace, "progress_recovery", "released",
                      "attached_collision_free_ordinary_path=1");
        state = {};
        return false;
    }

    WorldState recoveryWorld = sensedWorld;
    const auto attachmentUsable = [&]() {
        if (state.planner.waypoints.size() < 3 ||
            std::abs(recoveryWorld.egoPathD) > .75) return false;
        const auto near = std::lower_bound(state.planner.waypoints.begin(),
            state.planner.waypoints.end(), recoveryWorld.egoPathS,
            [](const auto& p, double s) { return p.s < s; });
        return near != state.planner.waypoints.end() &&
            std::abs(normalizeAngle(near->yaw - sensedWorld.ego.heading)) <= .6 &&
            state.planner.waypoints.back().s - recoveryWorld.egoPathS > 5.;
    };
    bool usable = state.planner.waypoints.size() >= 3 &&
        projectWorldToActivePath(recoveryWorld, state.planner.waypoints,
                                 sensedWorld.frontBumperOffset) && attachmentUsable();
    const bool stalled = state.noMotionTime >= data.config.progressRecoveryStopTimeout;
    const bool shortTail = usable &&
        state.planner.waypoints.back().s - recoveryWorld.egoPathS < 12.;
    state.retryRemaining = std::max(0., state.retryRemaining - elapsed);
    if (state.geometryFallback && state.retryRemaining <= 0.) usable = false;
    // Contact motion is not successful escape. Continue the balanced search
    // every retry interval even while the body is making longitudinal progress.
    const bool contactRecheck = state.contactExpected || data.frame.pathSafety.collision;
    if (state.retryRemaining <= 0. && (!usable || stalled || shortTail || contactRecheck)) {
        double failedHeading = std::numeric_limits<double>::quiet_NaN();
        if (usable && stalled) {
            const auto preview = std::lower_bound(state.planner.waypoints.begin(),
                state.planner.waypoints.end(), recoveryWorld.egoPathS + 3.,
                [](const auto& p, double s) { return p.s < s; });
            if (preview != state.planner.waypoints.end())
                failedHeading = normalizeAngle(preview->yaw - sensedWorld.ego.heading);
        }
        PathPlannerData replacement;
        replacement.config = data.pathPlanner.config;
        // The 3 m ordinary lookahead smooths away a near-standstill escape.
        // Use a shorter preview only for this low-speed recovery controller.
        replacement.config.minLookahead = std::min(2., replacement.config.minLookahead);
        bool contact = false;
        bool retainedReference = false;
        const auto& reference = usable ? state.planner.waypoints : data.frame.localPath.points;
        if (prepareCorridorProgressRecovery(replacement, contact, state.searchCursor,
                sensedWorld, map, data.missionCorridor, data.laneChangeConfig,
                data.config, reference, confirmedStationaryObjectIds, trace, failedHeading,
                &retainedReference)) {
            WorldState candidateWorld = sensedWorld;
            if (projectWorldToActivePath(candidateWorld, replacement.waypoints,
                                         sensedWorld.frontBumperOffset)) {
                state.planner = std::move(replacement);
                state.contactExpected = contact;
                state.geometryFallback = false;
                // Rechecking the same path must not postpone the stalled-path
                // timer forever. Only a replacement starts a new escape trial.
                if (!retainedReference) {
                    state.noMotionTime = 0.;
                    state.anchorX = sensedWorld.ego.x; state.anchorY = sensedWorld.ego.y;
                }
                recoveryWorld = std::move(candidateWorld);
                usable = attachmentUsable();
            }
        }
        state.retryRemaining = data.config.progressRecoveryRetryInterval;
    }
    if (!usable) {
        prepareSimulationCreepFallback(state.planner, sensedWorld, map,
            data.missionCorridor, data.pathPlanner.waypoints, trace);
        state.geometryFallback = true;
        state.contactExpected = true;
        recoveryWorld = sensedWorld;
        projectWorldToActivePath(recoveryWorld, state.planner.waypoints,
                                 sensedWorld.frontBumperOffset);
    }
    // Input validity for this simulation escape comes from this frame's
    // finite pose, not the failed lane/path match that caused the stop.
    recoveryWorld.inputValid = true;
    auto recoveryTracking = runPurePursuit(state.planner, gt, sensedWorld.egoSpeed);
    if (!recoveryTracking.valid || recoveryTracking.goalReached ||
        !std::isfinite(recoveryTracking.steering)) return false;

    // Check the trajectory actually about to be commanded, including a
    // retained escape if its blocker starts moving again. Its risk can differ
    // from that of the ordinary path evaluated earlier in this frame.
    LocalPathData recoveryPath;
    generateLocalPath(recoveryPath, state.planner, recoveryTracking.nearestIndex,
                      data.config.localPathHorizon);
    WorldState movingRecoveryWorld = recoveryWorld;
    movingRecoveryWorld.egoSpeed = std::max(std::abs(recoveryWorld.egoSpeed),
                                           data.config.progressRecoverySpeed);
    if (recoveryHasDynamicRisk(movingRecoveryWorld, recoveryPath, map, data.config,
            confirmedStationaryObjectIds, dynamicRiskObject)) {
        traceDecision(trace, "progress_recovery", "cancelled_for_dynamic_risk",
            "object=" + std::to_string(dynamicRiskObject) + " recovery_timer_reset=1");
        state = {};
        emergency = true;
        data.emergencyBrakeHoldRemaining = data.config.emergencyMinimumHoldTime;
        return false;
    }

    data.result.progressRecoveryEmergencyOverridden = emergency;
    data.result.progressRecoveryOverriddenStop = stationaryPathStop
        ? StopReason::StationaryPathBlocked : StopReason::None;
    // This final arbitration happens after all stationary caps and EBS
    // requests. Otherwise those same requests reapply zero in the next tick.
    world = std::move(recoveryWorld);
    tracking = recoveryTracking;
    tracking.goalReached = false;
    data.frame.behavior.targetSpeed = std::min(
        data.config.progressRecoverySpeed, recoveryTracking.targetSpeed);
    if (signalApproach) {
        // frame.stop was selected in the sensed guide coordinates. Do not
        // recompute this distance using the recovery path's station origin.
        const double available = std::max(0., data.frame.stop.distanceToStop -
                                               data.config.trafficStopPositionMargin);
        // Approaching a red light is not permission to push through its
        // waiting gateway queue or the certified queue approach endpoint.
        data.frame.behavior.targetSpeed = std::min({data.frame.behavior.targetSpeed,
            std::sqrt(2. * data.config.comfortableDeceleration * available),
            gatewayQueueSpeedCap});
    }
    data.frame.behavior.longitudinal = signalApproach ? LonMode::Stop : LonMode::Cruise;
    data.frame.behavior.health = SystemHealth::Ok;
    const auto& signalPoint = state.planner.waypoints[recoveryTracking.targetIndex];
    const double lateral = -(signalPoint.x - sensedWorld.ego.x) * std::sin(sensedWorld.ego.heading) +
                            (signalPoint.y - sensedWorld.ego.y) * std::cos(sensedWorld.ego.heading);
    data.frame.behavior.turnSignal = std::abs(lateral) < .2 ? 0U : lateral > 0. ? 1U : 2U;
    data.frame.localPath = std::move(recoveryPath);
    // Do not hide contacts from RViz or scoring when commanding progress.
    resetPathSafety(data.frame.pathSafety);
    checkCurrentCollisions(data.frame.pathSafety, world.egoObb, world.objects, -1, &world, &map);
    updateMinimumTTC(data.frame.pathSafety, world.objects, false, -1, &data.frame.localPath, &map);
    WorldState predictionWorld = std::move(movingRecoveryWorld);
    predictionWorld.egoSpeed = data.frame.behavior.targetSpeed;
    checkPredictedCollisions(data.frame.pathSafety, predictionWorld, data.frame.localPath,
        data.config.collisionPredictionHorizon, data.config.collisionPredictionStep, false, &map);
    emergency = false;
    data.emergencyBrakeHoldRemaining = 0.;
    data.result.progressRecoveryActive = true;
    data.result.progressRecoveryGeometryFallback = state.geometryFallback;
    data.result.progressRecoveryContactExpected = state.contactExpected ||
        data.frame.pathSafety.collision || data.frame.pathSafety.predictedCollision;
    traceDecision(trace, "progress_recovery", "command_override",
        "speed=" + std::to_string(data.frame.behavior.targetSpeed) +
        " signal_approach=" + std::to_string(signalApproach) +
        " stop_distance=" + std::to_string(data.frame.stop.distanceToStop) +
        " emergency_overridden=" + std::to_string(data.result.progressRecoveryEmergencyOverridden) +
        " stop_overridden=" + std::to_string(static_cast<std::uint32_t>(data.result.progressRecoveryOverriddenStop)) +
        " contact_expected=" + std::to_string(data.result.progressRecoveryContactExpected));
    return true;
}

}  // namespace

double schoolZoneApproachSpeedLimit(const PathPlannerConfig& planner,
    const PlanningBehaviorConfig& behavior, double distanceToEntry, double egoSpeed) {
    const double reaction = std::abs(egoSpeed) * behavior.schoolZoneApproachReactionTime;
    const double available = std::max(0.0, distanceToEntry - reaction);
    const double entry = std::sqrt(planner.schoolZoneSpeed * planner.schoolZoneSpeed +
        2.0 * behavior.comfortableDeceleration * available);
    const double approach = std::sqrt(planner.schoolZoneApproachSpeed * planner.schoolZoneApproachSpeed +
        2.0 * behavior.comfortableDeceleration *
            std::max(0.0, available - behavior.schoolZoneApproachDistance));
    // Neither this preview nor the zone target can raise a lower obstacle,
    // curvature, signal or emergency limit: the caller always takes min().
    return std::min(entry, approach);
}

std::uint8_t previewTrajectoryTurnSignal(const WorldState& world,
    const PathGeneratorData& map, const std::vector<Waypoint>& path) {
    if (!world.egoLaneValid || path.size() < 2) return 0;
    const auto roadIt = map.roads.find(world.egoRoadId);
    if (roadIt == map.roads.end()) return 0;
    const auto& road = roadIt->second;
    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double d = std::hypot(path[i].x-world.ego.x, path[i].y-world.ego.y);
        if (d < best) { best=d; nearest=i; }
    }
    if (best > 3.0) return 0;
    // Preview ~3 s, including at crawl speed. Road-relative displacement
    // removes ordinary road curvature from the indicator decision.
    const double horizon = std::clamp(std::abs(world.egoSpeed)*3.0, 8.0, 35.0);
    double firstOffset=0.0, firstCenter=0.0, direction=1.0;
    bool initialized=false;
    for (std::size_t i=nearest; i<path.size(); ++i) {
        const auto& p=path[i];
        if (p.s-path[nearest].s > horizon) break;
        if (p.roadId != world.egoRoadId || p.sectionIndex<0 ||
            static_cast<std::size_t>(p.sectionIndex)>=road.sections.size()) break;
        const auto& section=road.sections[p.sectionIndex];
        if (!section.lanes.count(world.egoLaneId)) break;
        const auto pose=roadToWorld(road,p.roadS);
        const double center=laneOffset(road,section,world.egoLaneId,p.roadS);
        const double offset=-(p.x-pose.x)*std::sin(pose.yaw)+
                             (p.y-pose.y)*std::cos(pose.yaw)-center;
        if (!initialized) {
            firstOffset=offset; firstCenter=center;
            direction=std::cos(world.ego.heading-pose.yaw)>=0.0?1.0:-1.0;
            initialized=true;
        }
        // A lane renumbering/offset discontinuity is not a lateral manoeuvre.
        if (std::abs(center-firstCenter)>0.5) break;
        const double change=(offset-firstOffset)*direction;
        if (std::abs(change)>=0.5) return change>0.0?1U:2U;
    }
    return 0;
}

void maintainTrajectoryTurnSignal(PlanningProcessData& data,
    const WorldState& world, std::uint8_t previewSignal, double dt) {
    if (world.respawnDetected) {
        data.trajectoryTurnSignal=0;
        data.trajectoryTurnSignalHoldRemaining=0.0;
        data.trajectoryTurnSignalRoadId=-1;
    }
    if (previewSignal!=0) {
        data.trajectoryTurnSignal=previewSignal;
        data.trajectoryTurnSignalHoldRemaining=1.0;
        data.trajectoryTurnSignalRoadId=world.egoRoadId;
    } else {
        const auto body=vehicleFootprintFromObb(world.egoObb,
            {world.ego.x,world.ego.y,world.ego.heading});
        // A multistep path may travel parallel while straddling the boundary.
        // No future lateral slope does NOT mean that this shift has finished.
        const bool stillStraddling=world.egoLaneValid && body.width>0.0 &&
            world.egoLaneWidth>0.0 &&
            data.trajectoryTurnSignalRoadId==world.egoRoadId &&
            std::abs(world.egoLaneCenterDistance)+body.width*.5>
                world.egoLaneWidth*.5-.05;
        if (stillStraddling && data.trajectoryTurnSignal!=0) {
            data.trajectoryTurnSignalHoldRemaining=1.0;
        } else {
            data.trajectoryTurnSignalHoldRemaining=std::max(0.0,
                data.trajectoryTurnSignalHoldRemaining-std::max(0.0,dt));
            if (data.trajectoryTurnSignalHoldRemaining<=0.0) data.trajectoryTurnSignal=0;
        }
    }
}

bool canCancelClearedPreparedAvoidance(
    const LaneChangeData& maneuver, const WorldState& world,
    const std::vector<Waypoint>& successor, bool successorPathSafe) {
    if (maneuver.phase != LaneChangePhase::Prepare ||
        maneuver.roadConnectionManeuver || maneuver.localCorridorAvoidance ||
        maneuver.obstacleId < 0 || !successorPathSafe || successor.size() < 2) {
        return false;
    }
    WorldState projected = world;
    if (!projectWorldToActivePath(projected, successor, world.frontBumperOffset) ||
        std::abs(projected.egoPathD) > 0.2) return false;
    const auto nearest = std::lower_bound(successor.begin(), successor.end(),
        projected.egoPathS,
        [](const Waypoint& point, double s) { return point.s < s; });
    const Waypoint& pose = nearest == successor.end() ? successor.back() : *nearest;
    if (std::abs(normalizeAngle(pose.yaw - world.ego.heading)) > 0.15) return false;

    const auto blocker = std::find_if(projected.objects.begin(), projected.objects.end(),
        [&](const WorldObjectState& object) {
            return static_cast<int>(object.raw.id) == maneuver.obstacleId;
        });
    // Do not infer clearance merely from motion: a moving lead/cut-in can
    // still occupy the successor. Live predictive safety is a separate
    // prerequisite above. An unprojectable actor is conservatively retained.
    return blocker == projected.objects.end() ||
        (blocker->pathValid && (!blocker->isAhead || !blocker->isOnEgoPath));
}

bool canResumeOrdinaryPathAfterRecovery(
    const WorldState& world, const std::vector<Waypoint>& path, bool pathSafe) {
    if (!pathSafe || world.respawnDetected || path.size() < 2) return false;
    bool projected = false;
    double station = 0., lateral = 0.;
    worldToRoute(world.ego.x, world.ego.y, projected, station, lateral,
                 path, std::nullopt, world.ego.heading);
    if (!projected || std::abs(lateral) > .3 || path.back().s-station < 8.) return false;
    const auto near = std::lower_bound(path.begin(), path.end(), station,
        [](const Waypoint& p, double s) { return p.s < s; });
    return near != path.end() &&
        std::abs(normalizeAngle(near->yaw-world.ego.heading)) <= .2;
}

bool buildGatewayHandoffContinuation(
    std::vector<Waypoint>& output, const std::vector<Waypoint>& successor,
    const std::vector<Waypoint>& gatewayReference) {
    output.clear();
    if (successor.size() < 2 || gatewayReference.size() < 2) return false;
    const auto& tail = successor.back();
    const auto join = std::find_if(gatewayReference.begin(), gatewayReference.end(),
        [&](const Waypoint& p) {
            return p.roadId == tail.roadId && p.sectionIndex == tail.sectionIndex &&
                p.laneId == tail.laneId && std::hypot(p.x-tail.x, p.y-tail.y) <= .05 &&
                std::abs(normalizeAngle(p.yaw-tail.yaw)) <= .05;
        });
    if (join == gatewayReference.end()) return false;
    output = successor;
    for (auto point = join + 1; point != gatewayReference.end(); ++point) {
        auto next = *point;
        next.s = output.back().s + std::hypot(next.x-output.back().x,
                                             next.y-output.back().y);
        output.push_back(next);
    }
    return true;
}

bool canResumeCommittedGatewayRoute(
    const LaneChangeData& maneuver, const WorldState& world,
    const std::vector<Waypoint>& successor, bool successorSafe) {
    if (!successorSafe || world.respawnDetected ||
        maneuver.phase == LaneChangePhase::None ||
        maneuver.roadConnectionManeuver || maneuver.path.size() < 2 ||
        successor.size() < 2) return false;
    WorldState projected = world;
    if (!projectWorldToActivePath(projected, successor, world.frontBumperOffset) ||
        std::abs(projected.egoPathD) > .35 ||
        successor.back().s - projected.egoPathS < 8.) return false;
    const auto first = std::lower_bound(successor.begin(), successor.end(),
        projected.egoPathS, [](const Waypoint& p, double s) { return p.s < s; });
    if (first == successor.end() ||
        std::abs(normalizeAngle(first->yaw - world.ego.heading)) > .12) return false;
    // A new route may branch later, but the steering being executed now
    // must join continuously. Never jump sideways to a nearby gateway line.
    bool samePrefix=true;
    for (auto point = first; point != successor.end() &&
         point->s <= projected.egoPathS + 3.; ++point) {
        WorldState sample = world;
        sample.ego.x = point->x; sample.ego.y = point->y;
        sample.ego.heading = point->yaw;
        if (!projectWorldToActivePath(sample, maneuver.path, world.frontBumperOffset) ||
            std::abs(sample.egoPathD) > .35) {samePrefix=false;break;}
        const auto old = std::lower_bound(maneuver.path.begin(), maneuver.path.end(),
            sample.egoPathS, [](const Waypoint& p, double s) { return p.s < s; });
        if (old == maneuver.path.end() ||
            std::abs(normalizeAngle(point->yaw - old->yaw)) > .12) {samePrefix=false;break;}
    }
    if(samePrefix)return true;
    // A measured-pose C2 bridge is allowed to diverge from the old local
    // prefix. Demanding identical geometry for 3 m postponed such a bridge
    // until the required fork was already missed. Require actual attachment,
    // consistent sample tangents, bounded curvature and initial continuity;
    // the caller separately verifies the complete gateway tail and collision.
    if(std::hypot(first->x-world.ego.x,first->y-world.ego.y)>.05 ||
       std::abs(normalizeAngle(first->yaw-world.ego.heading))>.02)return false;
    WorldState oldProjection=world;
    if(!projectWorldToActivePath(oldProjection,maneuver.path,world.frontBumperOffset))return false;
    const auto old=std::lower_bound(maneuver.path.begin(),maneuver.path.end(),oldProjection.egoPathS,
        [](const Waypoint& p,double s){return p.s<s;});
    if(old==maneuver.path.end() || old+1==maneuver.path.end())return false;
    const double oldDs=std::hypot((old+1)->x-old->x,(old+1)->y-old->y);
    if(oldDs<.01)return false;
    const double oldK=normalizeAngle((old+1)->yaw-old->yaw)/oldDs;
    for(auto p=first;p+1!=successor.end() && p->s<first->s+8.;++p) {
        const auto& next=*(p+1);
        const double dx=next.x-p->x,dy=next.y-p->y,ds=std::hypot(dx,dy);
        if(ds<.01 || ds>.51)return false;
        const double dyaw=normalizeAngle(next.yaw-p->yaw),k=dyaw/ds;
        if(std::abs(k)>.17785 ||
           std::abs(normalizeAngle(std::atan2(dy,dx)-(p->yaw+.5*dyaw)))>.03 ||
           (p==first && std::abs(k-oldK)>.03))return false;
    }
    return true;
}

bool initializePlanning(PlanningProcessData& data,
                        const PathPlannerConfig& plannerConfig,
                        const PlanningBehaviorConfig& behaviorConfig,
                        const LaneChangeConfig& laneChangeConfig,
                        const std::string& referencePathCsv) {
    data = {};
    data.pathPlanner.config = plannerConfig;
    data.roadLanePlanner.config = plannerConfig;
    data.config = behaviorConfig;
    data.laneChangeConfig = laneChangeConfig;
    return loadWaypointsCsv(data.pathPlanner.waypoints, referencePathCsv);
}

void runPlanningProcess(PlanningProcessData& data,
                        const GtFrame& gt,
                        const WorldState& sensedWorld,
                        const PathGeneratorData& map,
                        double dt,
                        DecisionTraceData* trace) {
    if (map.corridorRuntime) beginCorridorSearchFrame(*map.corridorRuntime);
    const StopDecision previousStop=data.frame.stop;
    resetPlanningFrame(data);
    updateMissionCorridorProgress(data.missionCorridor, sensedWorld);

    // sensedWorld의 path 좌표는 traffic-rule annotation용 guide route다.
    // checkpoint mission의 주행/장애물 판단은 아래에서 실제 추종할
    // lane-center 또는 보관된 maneuver path로 다시 투영한 복사본을 쓴다.
    WorldState world = sensedWorld;
    const bool missionActive =
        activeMissionLeg(data.missionCorridor) != nullptr;
    const std::size_t activeMissionLegIndex =
        data.missionCorridor.activeLegIndex;
    const auto stampRoadLanePlannerMissionLeg = [&]() {
        if (missionActive) {
            data.roadLanePlannerMissionLeg = activeMissionLegIndex;
            data.roadLanePlannerMissionLegValid = true;
        } else {
            data.roadLanePlannerMissionLegValid = false;
        }
    };
    const auto roadLanePlannerMatchesActiveLeg = [&]() {
        return !missionActive ||
               (data.roadLanePlannerMissionLegValid &&
                data.roadLanePlannerMissionLeg == activeMissionLegIndex);
    };
    const LaneKey sensedLane{sensedWorld.egoRoadId,
                             sensedWorld.egoSectionIndex,
                             sensedWorld.egoLaneId};
    const bool guideTopologyRequired=missionActive &&
        (missionLegRequiresGuideContinuity(data.missionCorridor) ||
         (sensedWorld.egoLaneValid &&
          missionLaneRequiresGuideContinuity(
              data.missionCorridor,sensedLane)));

    if(missionActive && !guideTopologyRequired && !data.progressRecovery.active &&
       data.laneChange.phase!=LaneChangePhase::None &&
       !data.laneChange.roadConnectionManeuver &&
       advanceClearLaneCenterReturn(data.laneChange.path,sensedWorld,map,
                                    data.missionCorridor,data.config,trace))
        data.laneChange.nearestIndex=0;
    if(missionActive && !guideTopologyRequired && !data.progressRecovery.active &&
       data.laneChange.phase==LaneChangePhase::None && map.corridorRuntime) {
        auto& runtime=*map.corridorRuntime;
        if(runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested &&
           advanceClearLaneCenterReturn(runtime.committedGlobalPath,sensedWorld,map,
                                        data.missionCorridor,data.config,trace))
            runtime.committedGlobalPathProgress=0;
        if(runtime.recoveryTrackingPathLeg==activeMissionLegIndex &&
           advanceClearLaneCenterReturn(runtime.recoveryTrackingPath,sensedWorld,map,
                                        data.missionCorridor,data.config,trace))
            runtime.recoveryTrackingPathProgress=0;
    }

    PathPlan globalTracking;
    std::size_t nearestGlobalIndex = data.pathPlanner.nearestIndex;
    if (!missionActive) {
        globalTracking = runPurePursuit(
            data.pathPlanner, gt, sensedWorld.egoSpeed);
        nearestGlobalIndex = globalTracking.nearestIndex;
    }
    // 새 맵매칭 section으로 경로를 만들다가 경계 한 프레임에
    // 실패해도 이전 정상 경로를 먼저 지우지 않는다. 임시 객체에 성공한
    // 경로만 만든 뒤, Ego 위치·헤딩·lane successor·잔여거리 검사를
    // 통과했을 때 원자적으로 교체한다.
    PathPlannerData roadLaneCandidate;
    const bool roadLaneCandidateBuilt = !guideTopologyRequired &&
        buildRoadIdTrackingPath(
            roadLaneCandidate, data.pathPlanner.config, world, map,
            data.pathPlanner, nearestGlobalIndex,
            data.config.localPathHorizon, trace,
            &data.missionCorridor);
    double candidateLateralDistance{};
    double candidateRemainingDistance{};
    const bool roadLaneCandidateUsable = roadLaneCandidateBuilt &&
        trackingPathUsable(roadLaneCandidate, sensedWorld, map,
                           candidateLateralDistance,
                           candidateRemainingDistance);
    bool retainedRoadLanePath = false;
    bool shortTailContinuityPath = false;
    bool geometricContinuityPath = false;
    // An excessive XODR detour is an explicit topology-gap diagnosis, not a
    // suggestion.  In that case the competition-provided, map-derived
    // shortest route is the global continuity source; accepting the corridor
    // candidate first let a route that eventually reached the same gateway
    // wander kilometres through unrelated roads (M11 3450->2756 instead of
    // the required 3450->2939 connector).  Local obstacle manoeuvres still
    // override either source below and may use every legal corridor lane.
    if (roadLaneCandidateUsable && !guideTopologyRequired) {
        data.roadLanePlanner = std::move(roadLaneCandidate);
        stampRoadLanePlannerMissionLeg();
    } else if (missionActive && map.corridorRuntime &&
               roadLaneCandidateBuilt &&
               !guideTopologyRequired &&
               corridorGeometryContinuityUsable(
                   roadLaneCandidate, sensedWorld,
                   candidateLateralDistance,
                   candidateRemainingDistance)) {
        // The candidate is geometrically attached to Ego and came from the
        // checked corridor, so a temporary lane-ID mismatch is not grounds
        // for deleting it.
        data.roadLanePlanner = std::move(roadLaneCandidate);
        stampRoadLanePlannerMissionLeg();
        geometricContinuityPath = true;
        std::ostringstream detail;
        detail << "source=new_corridor_candidate"
               << " lateral_distance=" << candidateLateralDistance
               << " remaining_distance=" << candidateRemainingDistance
               << " sensed_lane=" << sensedWorld.egoRoadId << ':'
               << sensedWorld.egoSectionIndex << ':'
               << sensedWorld.egoLaneId;
        traceDecision(trace, "road_corridor",
                      "geometric_continuity_path_accepted", detail.str());
    } else {
        double retainedLateralDistance{};
        double retainedRemainingDistance{};
        retainedRoadLanePath = roadLanePlannerMatchesActiveLeg() &&
            !guideTopologyRequired && trackingPathUsable(
            data.roadLanePlanner, sensedWorld, map,
            retainedLateralDistance, retainedRemainingDistance);
        if (!retainedRoadLanePath && roadLanePlannerMatchesActiveLeg() &&
            missionActive && !guideTopologyRequired &&
            map.corridorRuntime &&
            corridorGeometryContinuityUsable(
                data.roadLanePlanner, sensedWorld,
                retainedLateralDistance, retainedRemainingDistance)) {
            retainedRoadLanePath = true;
            geometricContinuityPath = true;
            std::ostringstream detail;
            detail << "source=previous_corridor_path"
                   << " lateral_distance=" << retainedLateralDistance
                   << " remaining_distance=" << retainedRemainingDistance
                   << " reason=metadata_or_topology_lag";
            traceDecision(trace, "road_corridor",
                          "geometric_continuity_path_retained",
                          detail.str());
        }
        if (retainedRoadLanePath) {
            std::ostringstream detail;
            detail << "reason="
                   << (roadLaneCandidateBuilt
                           ? "new_path_failed_pose_or_topology_validation"
                           : "new_path_build_failed")
                   << " lateral_distance=" << retainedLateralDistance
                   << " remaining_distance=" << retainedRemainingDistance
                   << " sensed_road=" << sensedWorld.egoRoadId
                   << " sensed_section=" << sensedWorld.egoSectionIndex
                   << " sensed_lane=" << sensedWorld.egoLaneId;
            traceDecision(trace, "road_corridor",
                          "previous_tracking_path_retained", detail.str());
        } else {
            std::ostringstream detail;
            detail << "candidate_built=" << roadLaneCandidateBuilt
                   << " candidate_lateral=" << candidateLateralDistance
                   << " candidate_remaining=" << candidateRemainingDistance
                   << " retained_lateral=" << retainedLateralDistance
                   << " retained_remaining=" << retainedRemainingDistance
                   << " sensed_input_valid=" << sensedWorld.inputValid
                   << " sensed_lane_valid=" << sensedWorld.egoLaneValid
                   << " sensed_road=" << sensedWorld.egoRoadId
                   << " sensed_section=" << sensedWorld.egoSectionIndex
                   << " sensed_lane=" << sensedWorld.egoLaneId;
            traceDecision(trace, "road_corridor",
                          "tracking_path_unavailable", detail.str());
        }
    }
    if (!guideTopologyRequired && !roadLaneCandidateUsable && !retainedRoadLanePath &&
        !geometricContinuityPath && roadLaneCandidateBuilt) {
        double shortTailLateralDistance{};
        double shortTailRemainingDistance{};
        shortTailContinuityPath = trackingPathUsable(
            roadLaneCandidate, sensedWorld, map,
            shortTailLateralDistance, shortTailRemainingDistance, false);
        if (shortTailContinuityPath) {
            data.roadLanePlanner = std::move(roadLaneCandidate);
            stampRoadLanePlannerMissionLeg();
            std::ostringstream detail;
            detail << "remaining_distance=" << shortTailRemainingDistance
                   << " lateral_distance=" << shortTailLateralDistance
                   << " reason=no_full_horizon_replacement_available";
            traceDecision(trace, "road_corridor",
                          "short_tail_continuity_path_retained",
                          detail.str());
        }
    }
    const bool roadLanePathValid = !guideTopologyRequired &&
        (roadLaneCandidateUsable || retainedRoadLanePath ||
         shortTailContinuityPath || geometricContinuityPath);
    PathPlan tracking;
    bool guideContinuityPathValid = false;
    bool guideReferencePhysicallyAttached = false;
    bool storedCorridorContinuityPathValid = false;
    bool referenceRecoveryPathValid = false;
    bool physicalMobilityFallbackPathValid = false;
    if (roadLanePathValid) {
        tracking = runPurePursuit(
            data.roadLanePlanner, gt, sensedWorld.egoSpeed);
    } else if (!missionActive) {
        tracking = globalTracking;
    } else {
        // The packaged reference path is the final precomputed continuity
        // source.  Corridor graph/search failure must not make a frame
        // pathless while this map-derived route is physically underneath the
        // vehicle.  An exhaustive rematch is used only on this exceptional
        // branch, so the ordinary 20 Hz path never pays its cost.
        double bestScore = std::numeric_limits<double>::infinity();
        std::size_t bestIndex = data.pathPlanner.nearestIndex;
        std::size_t guideBegin=0,guideEnd=data.pathPlanner.waypoints.size();
        if(guideTopologyRequired && !data.pathPlanner.waypoints.empty()) {
            const auto* leg=activeMissionLeg(data.missionCorridor);
            const auto nearestCheckpointSample=[&](std::size_t checkpoint) {
                const auto& target=
                    data.missionCorridor.checkpoints[checkpoint].checkpoint;
                std::size_t selected=0;double distance=
                    std::numeric_limits<double>::infinity();
                for(std::size_t index=0;
                    index<data.pathPlanner.waypoints.size();++index) {
                    const auto& point=data.pathPlanner.waypoints[index];
                    const double candidate=std::hypot(
                        point.x-target.x,point.y-target.y);
                    if(candidate<distance) {distance=candidate;selected=index;}
                }
                return selected;
            };
            const auto from=nearestCheckpointSample(leg->fromCheckpoint);
            const auto to=nearestCheckpointSample(leg->toCheckpoint);
            if(from<=to) {
                guideBegin=from>80?from-80:0;
                guideEnd=std::min(data.pathPlanner.waypoints.size(),to+81);
            }
        }
        for (std::size_t index = guideBegin;
             index < guideEnd; ++index) {
            const auto& point = data.pathPlanner.waypoints[index];
            const double distance = std::hypot(
                point.x - sensedWorld.ego.x,
                point.y - sensedWorld.ego.y);
            const double yaw = std::abs(normalizeAngle(
                point.yaw - sensedWorld.ego.heading));
            if (yaw > 1.75) continue;
            const double score = distance + 1.5 * yaw;
            if (score < bestScore) {
                bestScore = score;
                bestIndex = index;
            }
        }
        double guideLateral{};
        double guideRemaining{};
        if (!data.missionCorridor.topologyAuthority &&
            std::isfinite(bestScore) && bestScore <= 12.0) {
            data.pathPlanner.nearestIndex = bestIndex;
            guideContinuityPathValid = corridorGeometryContinuityUsable(
                data.pathPlanner, sensedWorld,
                guideLateral, guideRemaining);
        }
        // Use the actual lane envelope, not a fixed 1.25 m cutoff. A small
        // lateral impulse near a split must not hand route authority to the
        // other XODR successor while Ego is still inside the same lane.
        const double guideAttachmentDistance = std::max(
            1.25, 0.5 * std::max(sensedWorld.egoLaneWidth, 1.88));
        const bool guidePhysicallyAttached = !data.missionCorridor.topologyAuthority &&
            guideContinuityPathValid &&
            guideLateral <= guideAttachmentDistance &&
            bestIndex < data.pathPlanner.waypoints.size() &&
            std::abs(normalizeAngle(
                data.pathPlanner.waypoints[bestIndex].yaw -
                sensedWorld.ego.heading)) <= 0.8;
        guideReferencePhysicallyAttached = guidePhysicallyAttached;
        if (guidePhysicallyAttached) {
            tracking = runPurePursuit(
                data.pathPlanner, gt, sensedWorld.egoSpeed);
            std::ostringstream detail;
            detail << "nearest_index=" << bestIndex
                   << " lateral_distance=" << guideLateral
                   << " remaining_distance=" << guideRemaining
                   << " reason=corridor_paths_temporarily_unavailable";
            traceDecision(trace, "road_corridor",
                          "precomputed_guide_continuity_selected",
                          detail.str());
        } else if (guideTopologyRequired && map.corridorRuntime) {
            // Local obstacle avoidance may legally finish several lanes away
            // from the packaged mission route.  Rejecting that route left an
            // exhausted manoeuvre committed forever; tracking it directly
            // would cut sideways.  Join the measured pose to a forward guide
            // sample with a checked, cached C2 bridge.
            PathPlannerData recovered;
            if (buildReferenceRecoveryTrackingPath(
                    recovered, data.pathPlanner.config,
                    data.pathPlanner.waypoints, bestIndex, sensedWorld, map,
                    data.missionCorridor, data.config.localPathHorizon,
                    trace)) {
                data.roadLanePlanner = std::move(recovered);
                stampRoadLanePlannerMissionLeg();
                tracking = runPurePursuit(
                    data.roadLanePlanner, gt, sensedWorld.egoSpeed);
                referenceRecoveryPathValid = tracking.valid;
                traceDecision(trace, "road_corridor",
                              "guide_recovery_tracking_selected",
                              referenceRecoveryPathValid
                                  ? "checked_c2_bridge=1"
                                  : "checked_c2_bridge_tracking_invalid=1");
            }
        }
        if (!guidePhysicallyAttached && !referenceRecoveryPathValid &&
            guideTopologyRequired && map.corridorRuntime) {
            // Never point Pure Pursuit directly at a guide several metres
            // away.  That produced the large steering/wobble seen after an
            // avoidance and still did not reconnect.  If a checked C2 bridge
            // is not currently legal, keep moving on the XODR lane physically
            // under Ego.  The guide bridge is retried on the next frame and,
            // once legal, becomes the smooth return to the gateway route.
            double physicalLateral{};
            double physicalRemaining{};
            // Reuse the already checked XODR continuation while Ego remains
            // attached and it still has a useful forward suffix. Rebuilding
            // the same graph path every 50 ms reduced the effective loop to
            // about 4 Hz and caused stale dynamic-object observations.
            bool physicalAttached = roadLanePlannerMatchesActiveLeg() &&
                corridorGeometryContinuityUsable(
                    data.roadLanePlanner, sensedWorld, physicalLateral,
                    physicalRemaining) &&
                physicalLateral <= 1.25 && physicalRemaining >= 5.0;
            bool physicalRebuilt = false;
            if (!physicalAttached) {
                PathPlannerData physical;
                const bool physicalBuilt = buildRoadIdTrackingPath(
                    physical, data.pathPlanner.config, sensedWorld, map,
                    data.pathPlanner, bestIndex, data.config.localPathHorizon,
                    trace, &data.missionCorridor);
                physicalAttached = physicalBuilt &&
                    corridorGeometryContinuityUsable(
                        physical, sensedWorld, physicalLateral,
                        physicalRemaining) &&
                    physicalLateral <= 1.25;
                if (physicalAttached) {
                    data.roadLanePlanner = std::move(physical);
                    stampRoadLanePlannerMissionLeg();
                    physicalRebuilt = true;
                }
            }
            if (physicalAttached) {
                tracking = runPurePursuit(
                    data.roadLanePlanner, gt, sensedWorld.egoSpeed);
                physicalMobilityFallbackPathValid = tracking.valid;
                std::ostringstream detail;
                detail << "checked_xodr_lane=1"
                       << " lateral_distance=" << physicalLateral
                       << " remaining_distance=" << physicalRemaining
                       << " rebuilt=" << physicalRebuilt
                       << " guide_bridge_retry=1";
                traceDecision(trace, "road_corridor",
                              "physical_mobility_fallback_selected",
                              detail.str());
            }
        }
    }
    if (missionActive && !tracking.valid &&
        roadLanePlannerMatchesActiveLeg() &&
        data.roadLanePlanner.waypoints.size() >= 2 &&
        corridorGeometryContinuityUsable(data.roadLanePlanner,sensedWorld,
                                         candidateLateralDistance,candidateRemainingDistance)) {
        // Retain a still-attached forward corridor when metadata lags.
        // An exhausted/detached prefix belongs to reentry planning; it is
        // no longer a steering target merely because it was once accepted.
        tracking = runPurePursuit(
            data.roadLanePlanner, gt, sensedWorld.egoSpeed);
        if (tracking.valid) {
            if (tracking.goalReached) {
                tracking.goalReached = false;
                tracking.targetSpeed = 0.;
            }
            storedCorridorContinuityPathValid = true;
            traceDecision(trace, "road_corridor",
                          "stored_precomputed_path_forced",
                          "reason=attached_forward_path_retained");
        }
    }
    bool corridorTrackingPathValid =
        roadLanePathValid || storedCorridorContinuityPathValid ||
        referenceRecoveryPathValid || physicalMobilityFallbackPathValid;

    // A temporary local detour must not hide a completed gateway route until
    // its old Execute tail expires. Resume the certified route as soon as
    // the entry is continuous and the current observations still approve it.
    if (missionActive && corridorTrackingPathValid && tracking.valid &&
        map.corridorRuntime && data.laneChange.phase != LaneChangePhase::None) {
        const auto& runtime = *map.corridorRuntime;
        const auto& successor = data.roadLanePlanner.waypoints;
        const bool approachOnly = runtime.soleGatewayApproachSelected;
        const auto& gatewayReference = approachOnly
            ? runtime.soleGatewayApproachPath : runtime.committedGlobalPath;
        const bool gatewayCertified = runtime.committedGlobalPathReachesGateway &&
            runtime.committedGlobalPathLeg == activeMissionLegIndex &&
            !runtime.globalRouteReplanRequested && runtime.trackingPathApproved &&
            !successor.empty() && !runtime.committedGlobalPath.empty();
        // A recovery/physical fallback may coexist with a cached global path.
        // Verify that this actual tracking suffix rejoins that precise route.
        std::vector<Waypoint> gatewayContinuation;
        const bool followsGateway = (gatewayCertified ||
            (approachOnly && runtime.trackingPathApproved && !successor.empty())) &&
            canResumeCommittedGatewayRoute(data.laneChange, sensedWorld, successor, true) &&
            buildGatewayHandoffContinuation(gatewayContinuation, successor, gatewayReference);
        if (followsGateway && corridorTrackingPathIsSafe(gatewayContinuation, sensedWorld,
                approachOnly?data.laneChangeConfig.latticeObstacleClearance:runtime.globalPathClearance(),
                nullptr, &data.config, &map)) {
            // The trajectory indicator preview below retains the actual turn
            // signal; traffic stops and emergency braking are evaluated later.
            data.laneChange = {};
            map.corridorRuntime->localFallbackContinuationRequested = false;
            map.corridorRuntime->localFallbackEngaged = false;
            traceDecision(trace, "corridor_global_route", "safe_gateway_tracking_resumed",
                          approachOnly
                              ? "continuous_prefix=1 live_collision_checks=1 approach_only=1 gateway_completed=0"
                              : "continuous_prefix=1 live_collision_checks=1 temporary_maneuver_released=1");
        }
    }

    // Whether an already committed lateral manoeuvre still has a geometric
    // projection under Ego.  The phase enum alone is not sufficient: at a
    // merge/section boundary the manoeuvre tail can be exhausted while the
    // map matcher is already on a valid successor lane.  Keeping Execute in
    // that state used to make planning.valid false and the safety limiter
    // brake indefinitely despite a usable precomputed successor path.
    bool committedActivePathProjectionValid =
        data.laneChange.phase == LaneChangePhase::None;
    bool successorActivePathFallbackUsed = false;
    if (missionActive) {
        const std::vector<Waypoint>* activePath = nullptr;
        // Projection is transactional.  projectWorldToActivePath clears path
        // coordinates before searching, so calling it directly on `world`
        // used to destroy the still-valid sensed/precomputed projection when
        // a replacement path missed for one frame.
        WorldState projectedWorld = world;
        bool activePathValid = false;
        const auto tryActivePath = [&](const std::vector<Waypoint>& path) {
            WorldState candidate = world;
            if (path.size() < 2 || !projectWorldToActivePath(
                    candidate, path, sensedWorld.frontBumperOffset)) {
                return false;
            }
            projectedWorld = std::move(candidate);
            activePath = &path;
            return true;
        };
        if (data.laneChange.phase != LaneChangePhase::None) {
            activePathValid = tryActivePath(data.laneChange.path);
            committedActivePathProjectionValid = activePathValid;
        }
        // An exhausted or detached manoeuvre tail must not mask a physically
        // attached successor, regardless of whether the phase transition to
        // Settle has happened yet.  The projection check prevents an early
        // hand-off: a successor is accepted only when it is actually under
        // Ego with compatible heading.
        if (!activePathValid) {
            if (corridorTrackingPathValid) {
                activePathValid = tryActivePath(
                    data.roadLanePlanner.waypoints);
                if (activePathValid &&
                    data.laneChange.phase != LaneChangePhase::None) {
                    tracking = runPurePursuit(
                        data.roadLanePlanner, gt, sensedWorld.egoSpeed);
                    successorActivePathFallbackUsed = tracking.valid;
                }
            }
            if (!activePathValid && guideContinuityPathValid) {
                activePathValid = tryActivePath(data.pathPlanner.waypoints);
                if (activePathValid &&
                    data.laneChange.phase != LaneChangePhase::None) {
                    tracking = runPurePursuit(
                        data.pathPlanner, gt, sensedWorld.egoSpeed);
                    successorActivePathFallbackUsed = tracking.valid;
                }
            }
            if (activePathValid &&
                data.laneChange.phase != LaneChangePhase::None) {
                traceDecision(trace, "road_corridor",
                              "exhausted_maneuver_handoff",
                              "reason=committed_path_no_longer_projects successor_selected=1");
            }
        }
        if (activePathValid) world = std::move(projectedWorld);
        // guide route를 잃은 것은 정상일 수 있지만, 지도 lane이나 실제
        // 추종경로까지 잃으면 안전한 조향/충돌예측이 불가능하다.
        // During a merge/split or a recovery curve VTD can temporarily lose
        // its lane-centre match even though Ego is finite and projects cleanly
        // onto the already validated XODR trajectory.  Treat that successful
        // active-path projection as the mission input authority.  Requiring
        // the raw lane match as well made the safety limiter hold -4 m/s^2
        // forever at M11 road 3450 while a valid path and positive target
        // speed were both present.
        world.inputValid = activePathValid;
        if (!world.inputValid) {
            std::ostringstream detail;
            detail << "sensed_input_valid=" << sensedWorld.inputValid
                   << " active_path_present=" << (activePath != nullptr)
                   << " active_path_projected=" << activePathValid
                   << " road_lane_path_valid=" << roadLanePathValid
                   << " stored_corridor_continuity_valid="
                   << storedCorridorContinuityPathValid
                   << " guide_continuity_valid="
                   << guideContinuityPathValid
                   << " lane_change_phase="
                   << static_cast<int>(data.laneChange.phase);
            traceDecision(trace, "road_corridor",
                          "active_path_input_invalid", detail.str());
        }
    }
    if ((corridorTrackingPathValid || guideContinuityPathValid) &&
        tracking.goalReached) {
        // A local endpoint is not mission completion. Preserve its zero
        // target until a forward successor or checked recovery owns control;
        // restoring cruise here drove beyond exhausted paths into the island.
        tracking.goalReached = false;
    }
    // checkpoint mission에서는 이후 회피 생성/객체 rematch에도 dense
    // guide route를 전달하지 않는다. 현재 corridor lane chain을 유일한
    // 주행 기준경로로 사용한다.
    PathPlannerData committedManeuverPlanner;
    const bool committedManeuverActive =
        missionActive &&
        data.laneChange.phase != LaneChangePhase::None &&
        data.laneChange.path.size() >= 2;
    if (committedManeuverActive) {
        committedManeuverPlanner.config = data.pathPlanner.config;
        committedManeuverPlanner.waypoints = data.laneChange.path;
        committedManeuverPlanner.nearestIndex =
            data.laneChange.nearestIndex;
    }
    // 이미 확정한 기동 중에는 객체 재투영과 후속 회피 생성도 같은
    // committed path를 기준으로 수행한다. 일반 차선 경로를 다시
    // 참조하면 객체의 전후/차선 판정이 바뀌어 후보가 프레임마다
    // 흔들릴 수 있다.
    // When the packaged mission guide is physically under Ego, keep it as
    // the obstacle/lattice reference even if a cached physical continuation
    // also exists.  At a split both paths initially overlap; allowing the
    // physical fallback to become the reference made the planner regard the
    // wrong outgoing branch as an equally valid lane and miss the gateway.
    // Once avoidance has actually detached Ego from the guide, the attached
    // XODR continuation remains the reference so obstacle avoidance never
    // attempts a lateral jump back to a distant guide.
    const bool useAttachedMissionGuide =
        missionActive && guideTopologyRequired &&
        guideReferencePhysicallyAttached;
    // Route authority must follow the actual trajectory, not only the
    // currently attached raw guide.  A checked C2 recovery path contains an
    // official-guide suffix, and a committed avoidance path can be replanned
    // again before it finishes.  Dropping this bit in either case let a nested
    // replan choose the other physical branch at a split even though its lane
    // metadata was identical (M12 exposed this generic failure mode).
    const bool maneuverReferenceIsMissionGuide =
        committedManeuverActive
            ? data.laneChange.missionRouteAuthority
            : (useAttachedMissionGuide || referenceRecoveryPathValid);
    const PathPlannerData& maneuverReference = committedManeuverActive
        ? committedManeuverPlanner
        : (useAttachedMissionGuide
               ? data.pathPlanner
               : (missionActive && corridorTrackingPathValid
                      ? data.roadLanePlanner : data.pathPlanner));
    const std::size_t nearestManeuverIndex = committedManeuverActive
        ? data.laneChange.nearestIndex
        : (useAttachedMissionGuide
               ? data.pathPlanner.nearestIndex
               : (missionActive &&
                  (corridorTrackingPathValid || guideContinuityPathValid)
                      ? tracking.nearestIndex : nearestGlobalIndex));
    const auto laneAlignment=laneAlignmentBeforeStopLine(
        sensedWorld,map,maneuverReference,data.config.stopSafetyMargin,
        std::max(data.laneChangeConfig.obstacleTriggerMaximumDistance,
                 data.laneChangeConfig.roadConnectionPrepareDistance));
    data.frame.constraints.mapMatchValid = world.inputValid;
    bool schoolZoneActive = false;
    bool crosswalkControlActive = false;
    const std::vector<int> confirmedStationaryObjectIds =
        updateConfirmedStationaryObjects(data, world, dt);
    const bool crossingYield = updateCrossingObjectYields(data, world,
        maneuverReference.waypoints, nearestManeuverIndex, map, dt, trace);

    // 횡단보도 점유는 여러 속도/정지 판단에서 공유한다. 한 프레임에
    // 한 번만 갱신해 점유 해제 타이머가 중복으로 감소하지 않게 한다.
    updateCrosswalkStates(data, sensedWorld, map.staticZones, dt, previousStop, trace);

    // 신호/횡단보도 정지 여부를 먼저 판정한다. 빨간불 전에 만든
    // 회피 경로를 정지한 동안 보관했다가 녹색불에서 중간부터
    // 추종하면 정지선에서 바로 크게 조향한다.
    updateStopRequests(data, sensedWorld, map.staticZones, dt, trace);

    // A blocked whole-corridor route is not a reason to wait until the final
    // stationary speed guard fires. Poll the bounded local worker before
    // legacy one-shift planning; adoption is transactional and revalidates
    // the measured pose and current scene. Signal/pedestrian requests above
    // still gate execution below, including while a local job is running.
    if (missionActive && map.corridorRuntime) {
        map.corridorRuntime->localFallbackContinuationRequested =
            data.laneChange.phase != LaneChangePhase::None &&
            data.laneChange.localCorridorAvoidance &&
            !data.laneChange.path.empty() &&
            data.laneChange.path.back().s-world.egoPathS <
                // Obstacle activation (50 m) is not a path-renewal distance:
                // renewing a 54 m local path with 50 m left repeatedly replaces
                // its already checked return before the vehicle can follow it.
                // Retain five seconds of planning headroom, at least 20 m;
                // new hazards still invalidate the incumbent independently.
                std::max(20.0,
                         std::abs(world.egoSpeed)*
                    data.laneChangeConfig.localFallbackTimeHeadway+8.0);
        LaneChangeData localReplacement;
        // A tight but validated detour must not be reclassified as blocked
        // using the nominal global margin on every frame. Keep it until its
        // own clearance becomes unsafe or the rolling horizon needs renewal.
        // Pending jobs are still polled so no asynchronous result is stranded.
        bool retainSafeLocal = false;
        if (data.laneChange.phase != LaneChangePhase::None &&
            data.laneChange.localCorridorAvoidance &&
            !map.corridorRuntime->localFallbackContinuationRequested &&
            !map.corridorRuntime->localFallbackPending) {
            PathSafetyResult localSafety;
            retainSafeLocal = evaluateLaneChangeCandidate(
                localSafety, data.laneChange, world, data.config,
                std::max(std::abs(world.egoSpeed),
                         localManeuverSpeedLimit(data.laneChange,data.laneChangeConfig,
                                                 data.pathPlanner.config,gt)), map);
            const double localClearance = data.laneChange.validatedObstacleClearance >= 0.
                ? data.laneChange.validatedObstacleClearance
                : data.laneChangeConfig.latticeObstacleClearance;
            retainSafeLocal = retainSafeLocal && std::isinf(
                corridorStationaryFreeDistance(data.laneChange.path, world,
                                               localClearance, &map));
            if (retainSafeLocal) map.corridorRuntime->localFallbackEngaged = false;
        }
        if (!retainSafeLocal && prepareLocalCorridorFallback(
                localReplacement, data.laneChangeConfig, data.config, world,
                map, data.missionCorridor, maneuverReference.waypoints, trace)) {
            WorldState replacementWorld = world;
            const bool queueNeedsCompleteBypass=std::isfinite(
                soleGatewayQueueDistance(world,map,data.missionCorridor));
            if(queueNeedsCompleteBypass && !queuedGatewayBypassIsComplete(
                    localReplacement.path,data.missionCorridor)) {
                traceDecision(trace,"sole_gateway","partial_bypass_not_adopted",
                    "queue_lane_retained=1 complete_gateway_passage_required=1");
            } else if (projectWorldToActivePath(replacementWorld, localReplacement.path,
                                         sensedWorld.frontBumperOffset)) {
                world = std::move(replacementWorld);
                data.laneChange = std::move(localReplacement);
                traceDecision(trace, "lane_change", "local_fallback_adopted",
                              "static_safe_multistage_path_replaces_reference_atomically=1");
            } else {
                traceDecision(trace, "lane_change", "local_fallback_not_attached",
                              "previous_trajectory_retained=1");
            }
        }
    }
    bool localPathUnsafe = false;
    if (data.laneChange.phase != LaneChangePhase::None &&
        data.laneChange.localCorridorAvoidance) {
        PathSafetyResult risk;
        localPathUnsafe = !evaluateLaneChangeCandidate(risk, data.laneChange, world, data.config,
                std::max(std::abs(world.egoSpeed),
                         localManeuverSpeedLimit(data.laneChange,data.laneChangeConfig,
                                                 data.pathPlanner.config,gt)), map);
        const double clearance = data.laneChange.validatedObstacleClearance >= 0.
            ? data.laneChange.validatedObstacleClearance
            : data.laneChangeConfig.latticeObstacleClearance;
        localPathUnsafe = localPathUnsafe || !std::isinf(corridorStationaryFreeDistance(
            data.laneChange.path, world, clearance, &map));
    }
    // Only an actually adopted, safe local path owns the manoeuvre. Merely
    // searching (or failing) must NEVER suppress the existing local planner.
    // This applies to static blockers as well as unexpected moving objects.
    const bool localFallbackOwnsPlanning = !localPathUnsafe &&
        data.laneChange.phase != LaneChangePhase::None &&
        data.laneChange.localCorridorAvoidance;

    double pendingBlockerDistance = -1.0;
    bool mandatoryCommittedAvoidance = false;
    if (!localFallbackOwnsPlanning &&
        data.laneChange.phase != LaneChangePhase::None && map.corridorRuntime) {
        const auto footprint=vehicleFootprintFromObb(world.egoObb,
            {world.ego.x,world.ego.y,world.ego.heading});
        // A retained partial manoeuvre is not a certificate for the next
        // obstacle cluster. Search a successor with the ordinary planning
        // preview, not only after entering the 15 m emergency-avoidance zone.
        // This probes the actual swept trajectory, so a clear neighbouring
        // object cannot trigger cosmetic lane changes. Replacement is atomic
        // and remains contingent on all existing safety/mission checks.
        const double successorPreview = std::max(
            data.laneChangeConfig.obstacleMandatoryAvoidanceDistance,
            std::min(data.laneChangeConfig.localFallbackHorizon-12.,
                std::max(data.laneChangeConfig.localFallbackActivationDistance,
                    std::abs(world.egoSpeed)*data.laneChangeConfig.localFallbackTimeHeadway)));
        for (const auto& point : data.laneChange.path) {
            const double ahead = point.s - world.egoPathS;
            if (ahead < 0.0) continue;
            if (ahead > successorPreview) break;
            const auto box=vehicleFootprintAt(footprint,{point.x,point.y,point.yaw});
            for (const auto& object : world.objects) {
                if(!corridorObstacleAtPathLevel(map,point,object)) continue;
                if (std::find(confirmedStationaryObjectIds.begin(),
                              confirmedStationaryObjectIds.end(), int(object.raw.id)) ==
                    confirmedStationaryObjectIds.end()) continue;
                if (intersects(box, object.obb)) {
                    mandatoryCommittedAvoidance = true;
                    pendingBlockerDistance = ahead;
                    break;
                }
            }
            if (mandatoryCommittedAvoidance) break;
        }
        if (mandatoryCommittedAvoidance) {
            LaneChangeData replacement;
            // Do not wait for Execute/Settle or a lane-centre match when a
            // committed trajectory now heads into a nearby static object.
            const bool ready = tryPrepareObstacleLaneChange(
                replacement, data.laneChangeConfig, data.config, world, map,
                maneuverReference, nearestManeuverIndex, nullptr,
                &confirmedStationaryObjectIds, trace, &data.missionCorridor,
                maneuverReferenceIsMissionGuide,&laneAlignment);
            traceDecision(trace, "lane_change", "mandatory_avoidance_replan",
                          ready ? "replacement_ready=1" : "replacement_ready=0 retry_required=1");
            if (ready) data.laneChange = std::move(replacement);
        }
    }
    // A gateway path used to be replaced every frame by a merely visible
    // (often 25--40 m distant) static obstacle.  That discarded the committed
    // gateway branch before the car entered it and made the selected lane
    // oscillate away from the exit.  The block above already preempts any
    // active path that actually intersects a confirmed static obstacle inside
    // obstacleMandatoryAvoidanceDistance.  Do not perform a second, broader
    // preemption here: distant objects remain planned candidates, but cannot
    // erase a currently safe gateway commitment.
    // Gateway route에 서로 다른 section의 횡이동이 연속하면
    // 이전 단계의 settle 종료를 기다리는 동안 다음 점선 구간을
    // 지나칠 수 있다. 이전 기동은 Gateway 정렬뿐 아니라 장애물
    // 회피일 수도 있다. 기동의 종류와 무관하게 현재 목표 차선
    // 중심에 안착했으면 다음 Gateway 단계를 즉시 계획해
    // committed path를 원자적으로 이어 간다. 이 조건은 road/lane
    // ID를 특정하지 않고 모든 corridor leg에 동일하게 적용된다.
    if (!localFallbackOwnsPlanning &&
        data.laneChange.phase != LaneChangePhase::None &&
        !data.missionCorridor.topologyAuthority &&
        data.laneChange.shiftCompletedLaneValid &&
        !mandatoryCommittedAvoidance &&
        (data.laneChange.phase == LaneChangePhase::Settle ||
         world.egoPathS >= data.laneChange.rejoinRouteS - 1.0) &&
        world.egoLaneValid &&
        world.egoRoadId == data.laneChange.shiftCompletedLane.roadId &&
        world.egoSectionIndex ==
            data.laneChange.shiftCompletedLane.sectionIndex &&
        world.egoLaneId == data.laneChange.shiftCompletedLane.laneId &&
        world.egoLaneCenterDistance <= 0.75) {
        LaneChangeData chainedGateway;
        const bool gatewayReady = tryPrepareRoadConnectionLaneChange(
                chainedGateway, data.laneChangeConfig, data.config,
                world, map, maneuverReference, nearestManeuverIndex,
                trace, &data.missionCorridor,&laneAlignment);
        // Gateway 후보 자체가 전체 기동과 settling tail의 장애물
        // 여유를 통과해야 한다. 따라서 회피 기동 도중이라도 목표
        // 차선 중심에 안착했고 안전한 Gateway 단계가 있으면 방향과
        // 무관하게 바로 연결한다. 막힌 차선으로의 복귀는 후보 안전
        // 검사에서 탈락하므로 별도의 방향 휴리스틱을 두지 않는다.
        if (gatewayReady) {
            traceDecision(trace, "road_corridor",
                          "gateway_stage_chained",
                          "next_lateral_stage_committed_before_settle");
            data.laneChange = std::move(chainedGateway);
        }
    }
    // 보관한 회피 경로가 실행 중 새 정적 장애물에 막혀도 기존 경로부터
    // 비우지 않는다. 빈 상태로 만든 뒤 후보 탐색에 실패하면 프레임마다
    // 좌우 후보가 뒤집히고 결국 추종할 지역경로까지 사라진다. 현재 차가
    // 차로 중심에 안착한 경우에만 별도 상태에서 새 후보를 만들고, 안전한
    // 후보 생성이 성공했을 때 원자적으로 교체한다.
    if (!localFallbackOwnsPlanning &&
        (data.laneChange.phase == LaneChangePhase::Prepare ||
         data.laneChange.phase == LaneChangePhase::Execute ||
         data.laneChange.phase == LaneChangePhase::Settle) &&
        map.corridorRuntime) {
        PathSafetyResult activeSafety;
        const bool activePathSafe = evaluateLaneChangeCandidate(
            activeSafety, data.laneChange, world, data.config,
            data.laneChangeConfig.maximumLaneChangeSpeed,map);
        const bool staticRisk = activeSafety.riskObjectId >= 0 &&
            std::any_of(world.objects.begin(), world.objects.end(),
                        [&](const WorldObjectState& object) {
                            return static_cast<int>(object.raw.id) ==
                                       activeSafety.riskObjectId &&
                                   std::abs(object.raw.speed) < 0.3;
                        });
        const bool dynamicRisk = activeSafety.riskObjectId >= 0 &&
            std::any_of(world.objects.begin(), world.objects.end(),
                        [&](const WorldObjectState& object) {
                            return static_cast<int>(object.raw.id) ==
                                       activeSafety.riskObjectId &&
                                   std::abs(object.raw.speed) >= 0.3;
                        });
        // 후보 생성 시 안착 차선의 종방향 연결에서 확인한 다음 장애물은
        // 현재 경로 종점 뒤에 있어 activePathSafe만으로는 잡히지 않는다.
        // 목표 차선 중심에 먼저 안착한 순간 다음 회피를 준비해, 첫 기동의
        // rejoinRouteS를 모두 지난 뒤에야 재계획하던 한 프레임/한 기동
        // 지연을 없앤다. 2~3단계를 포함한 연속 장애물도 각 안전한 차선
        // 중심 구간에서 경로를 원자적으로 계속 이어 간다.
        const double followupDistance =
            data.laneChange.followupObstacleClearance >= 0.0
                ? std::max(0.0, data.laneChange.rejoinRouteS - world.egoPathS) +
                      data.laneChange.followupObstacleClearance
                : std::numeric_limits<double>::infinity();
        const bool anticipatedFollowupReady =
            !data.laneChange.roadConnectionManeuver &&
            data.laneChange.followupObstacleId >= 0 &&
            data.laneChange.shiftCompletedLaneValid &&
            (data.laneChange.phase == LaneChangePhase::Settle ||
             world.egoPathS >= data.laneChange.rejoinRouteS - 1.0) &&
            world.egoLaneValid &&
            world.egoRoadId == data.laneChange.shiftCompletedLane.roadId &&
            world.egoSectionIndex ==
                data.laneChange.shiftCompletedLane.sectionIndex &&
            world.egoLaneId == data.laneChange.shiftCompletedLane.laneId &&
            world.egoLaneCenterDistance <= 0.75 &&
            followupDistance <=
                data.laneChangeConfig.obstacleTriggerMaximumDistance &&
            std::any_of(world.objects.begin(), world.objects.end(),
                        [&](const WorldObjectState& object) {
                            return static_cast<int>(object.raw.id) ==
                                       data.laneChange.followupObstacleId &&
                                   std::abs(object.raw.speed) < 0.3 &&
                                   object.isAhead &&
                                   object.relativeS >=
                                       data.laneChangeConfig
                                           .obstacleTriggerMinimumDistance &&
                                   object.relativeS <=
                                       data.laneChangeConfig
                                           .obstacleTriggerMaximumDistance;
                        });
        // 실제로 이미 접촉한(collision=true) 정적 장애물도 재계획 대상이다.
        // 예전에는 !activeSafety.collision을 요구해서, 일단 닿아버리면
        // 비상제동만 걸리고 목표점(reconnected)에 영원히 못 도달해
        // phase가 절대 None으로 안 돌아가 다음 회피 탐색 자체가 막혔다.
        // 정적 장애물은 접촉 여부와 무관하게 반드시 회피해야 한다.
        // A new cut-in can invalidate a committed detour too. In the new
        // topology-authority mode, try a predictive-collision-checked escape
        // from the actual pose. Replace atomically ONLY on success; otherwise
        // keep the path and independent predictive braking/yielding active.
        const bool unsafeActivePathReplan =
            !activePathSafe && (staticRisk ||
                (data.missionCorridor.topologyAuthority && dynamicRisk)) &&
            (activeSafety.collision || activeSafety.predictedCollision);
        if (anticipatedFollowupReady || unsafeActivePathReplan) {
            std::ostringstream detail;
            detail << "reason="
                   << (anticipatedFollowupReady
                           ? "anticipated_followup"
                           : "active_path_became_unsafe")
                   << " followup_object="
                   << data.laneChange.followupObstacleId
                   << " risk_object=" << activeSafety.riskObjectId
                   << " followup_distance=" << followupDistance
                   << " predicted_ttc="
                   << activeSafety.predictedCollisionTTC
                   << " dynamic_risk=" << dynamicRisk
                   << " route_invalidated=" << unsafeActivePathReplan;
            LaneChangeData replacement;
            double replacementBlockerDistance = -1.0;
            const bool replacementReady = tryPrepareObstacleLaneChange(
                replacement, data.laneChangeConfig, data.config, world, map,
                maneuverReference, nearestManeuverIndex,
                &replacementBlockerDistance,
                &confirmedStationaryObjectIds, trace,
                &data.missionCorridor,
                maneuverReferenceIsMissionGuide,&laneAlignment);
            detail << " replacement_ready=" << replacementReady;
            traceDecision(trace, "lane_change",
                          replacementReady
                              ? "active_path_replaced"
                              : "active_path_retained",
                          detail.str());
            if (replacementReady) {
                data.laneChange = std::move(replacement);
                pendingBlockerDistance = replacementBlockerDistance;
            }
        }
    }
    // 먼 신호 정지 요청보다 현재 도로의 장애물 회피를 먼저 계획한다.
    // 아래에서 경로 종점이 정지선을 침범할 때만 실행을 막는다.
    if (!localFallbackOwnsPlanning) {
        tryPrepareObstacleLaneChange(
            data.laneChange, data.laneChangeConfig, data.config, world, map,
            maneuverReference, nearestManeuverIndex, &pendingBlockerDistance,
            &confirmedStationaryObjectIds, trace, &data.missionCorridor,
            maneuverReferenceIsMissionGuide,&laneAlignment);
    }
    // 아직 확정 기동이 없을 때는 눈앞 장애물 후보를 먼저 검색한다.
    // 장애물이 mandatory 거리 밖이면 안전 검증된 gateway 정렬도
    // 계속 허용한다. 멀리 보이는 장애물 하나가 분기 진입을 막아
    // 버리는 것을 방지하고, 15 m 안에서는 회피 실패를 다른 경로로
    // 덮지 않는다.
    if (!localFallbackOwnsPlanning &&
        data.laneChange.phase == LaneChangePhase::None &&
        (pendingBlockerDistance < 0.0 ||
         pendingBlockerDistance >
             data.laneChangeConfig.obstacleMandatoryAvoidanceDistance)) {
        tryPrepareRoadConnectionLaneChange(
            data.laneChange, data.laneChangeConfig, data.config, world,
            map, maneuverReference, nearestManeuverIndex, trace,
            &data.missionCorridor,&laneAlignment);
    }
    // gateway target/path는 한 번 선택하면 정지나 감속 중에도 고정한다.
    // Pure Pursuit가 실행 시점의 최근접점을 다시 찾으므로 prepare 동안
    // 프레임마다 경로를 재생성할 필요가 없고, 재생성은 오히려 분기 선택을
    // 흔들거나 순간적인 실선 판정으로 정상 경로를 잃게 만든다.
    bool laneChangeExecutionBlocked =
        data.laneChange.phase == LaneChangePhase::None
            ? hasLaneChangeExecutionBlocker(data.stopManager)
            : hasLaneChangeExecutionBlocker(
                  data.stopManager, data.laneChange.rejoinRouteS);

    const auto successorPlannerAttachedToEgo =
        [&](const PathPlannerData& planner,double clearance) {
            if (planner.waypoints.size() < 2) return false;
            WorldState projected = sensedWorld;
            if (!projectWorldToActivePath(
                    projected, planner.waypoints,
                    sensedWorld.frontBumperOffset)) {
                return false;
            }
            const auto nearest = std::lower_bound(
                planner.waypoints.begin(), planner.waypoints.end(),
                projected.egoPathS,
                [](const Waypoint& point, double pathS) {
                    return point.s < pathS;
                });
            const Waypoint& pose = nearest == planner.waypoints.end()
                ? planner.waypoints.back() : *nearest;
            return planner.waypoints.back().s-projected.egoPathS>=5. &&
                std::abs(projected.egoPathD) <= 1.25 &&
                std::abs(normalizeAngle(
                    pose.yaw - sensedWorld.ego.heading)) <= 0.8 &&
                corridorTrackingPathIsSafe(planner.waypoints,sensedWorld,
                    clearance,nullptr,&data.config,&map);
        };
    const bool needSuccessorCheck=data.laneChange.phase!=LaneChangePhase::None &&
        (data.laneChange.phase==LaneChangePhase::Prepare ||
         data.laneChange.phase==LaneChangePhase::Settle ||
         successorActivePathFallbackUsed ||
         world.egoPathS>=data.laneChange.rejoinRouteS-1.);
    // Validate the exact successor selected below, not a different safe
    // guide while an unsafe corridor path still owns tracking.
    bool successorPathReady=needSuccessorCheck &&
        (corridorTrackingPathValid
            ?successorPlannerAttachedToEgo(data.roadLanePlanner,
                data.laneChangeConfig.latticeObstacleClearance)
            :(guideContinuityPathValid &&
              successorPlannerAttachedToEgo(data.pathPlanner,
                data.laneChangeConfig.latticeObstacleClearance)));
    if(needSuccessorCheck && !successorPathReady &&
       !data.laneChange.roadConnectionManeuver && map.corridorRuntime &&
       (data.laneChange.phase==LaneChangePhase::Settle ||
        world.egoPathS>=data.laneChange.rejoinRouteS-1.)) {
        PathPlannerData tail;
        tail.config=data.pathPlanner.config;
        tail.waypoints=data.laneChange.path;
        tail.nearestIndex=data.laneChange.nearestIndex;
        const double clearance=data.laneChange.validatedObstacleClearance>=0.
            ?data.laneChange.validatedObstacleClearance
            :data.laneChangeConfig.latticeObstacleClearance;
        if(successorPlannerAttachedToEgo(tail,clearance) &&
           retainCorridorAvoidanceTail(map,data.missionCorridor,sensedWorld,
                                      tail.waypoints,trace,clearance)) {
            data.roadLanePlanner=std::move(tail);
            stampRoadLanePlannerMissionLeg();
            tracking=runPurePursuit(data.roadLanePlanner,gt,sensedWorld.egoSpeed);
            corridorTrackingPathValid=tracking.valid;
            successorPathReady=tracking.valid;
            map.corridorRuntime->trackingPathApproved=tracking.valid;
            map.corridorRuntime->trackingPathValidatedClearance=
                tracking.valid?clearance:-1.;
            traceDecision(trace,"lane_change","safe_avoidance_tail_handoff",
                          "unsafe_successor_not_selected=1");
        }
    }
    if (data.missionCorridor.topologyAuthority && map.corridorRuntime &&
        data.laneChange.phase != LaneChangePhase::None &&
        !data.laneChange.roadConnectionManeuver &&
        !data.laneChange.gatewayReplanRequestedAfterShift &&
        data.laneChange.shiftCompletedLaneValid && world.egoLaneValid &&
        LaneKey{world.egoRoadId,world.egoSectionIndex,world.egoLaneId} ==
            data.laneChange.shiftCompletedLane &&
        world.egoLaneCenterDistance <= 0.75 &&
        (data.laneChange.phase == LaneChangePhase::Settle ||
         world.egoPathS >= data.laneChange.rejoinRouteS-1.0)) {
        requestCorridorGlobalReplan(*map.corridorRuntime);
        data.laneChange.gatewayReplanRequestedAfterShift = true;
        traceDecision(trace,"corridor_global_route","post_avoidance_replan_requested",
                      "target_lane_reached=1 keep_current_trajectory_until_replacement=1");
    }
    if (!data.missionCorridor.topologyAuthority &&
        data.laneChange.phase == LaneChangePhase::Settle &&
        !successorPathReady && map.corridorRuntime) {
        // A topology-valid route can still be the pre-detour route several
        // metres away.  Keeping that route marked committed makes the next
        // obstacle curve start on the wrong lane; dropping it here triggers
        // a bounded rebuild from the measured post-manoeuvre pose next frame.
        auto& runtime = *map.corridorRuntime;
        if (runtime.committedGlobalPathReachesGateway) {
            // Do not rebuild the same detached thousand-point route at 20 Hz.
            // Keep the route itself: buildCorridorTrackingPath reconnects the
            // measured pose to a safe forward point on this exact gateway
            // route.  Throwing it away and recovering to an arbitrary nearby
            // XODR successor skipped required connector roads in M11.
            runtime.globalRouteSearchFailed = true;
            runtime.globalRouteFailureLane = {
                world.egoRoadId, world.egoSectionIndex, world.egoLaneId};
            runtime.globalRouteFailureRoadS = world.egoRoadS;
            runtime.globalRouteRetryAfter = world.receiveTime + 1.0;
            traceDecision(trace, "corridor_global_route",
                          "post_maneuver_route_invalidated",
                          "reason=successor_not_physically_attached_to_ego route_retained_for_c2_reconnect=1");
        }
    }

    PathSafetyResult maneuverSpeedSafety;
    const bool candidateSafeAtManeuverSpeed =
        data.laneChange.phase == LaneChangePhase::None ||
        evaluateLaneChangeCandidate(
            maneuverSpeedSafety, data.laneChange, world, data.config,
            data.laneChange.localCorridorAvoidance
                ? std::max(std::abs(world.egoSpeed),
                           localManeuverSpeedLimit(data.laneChange,data.laneChangeConfig,
                                                   data.pathPlanner.config,gt))
                : data.laneChangeConfig.maximumLaneChangeSpeed,map);
    if (data.laneChange.phase != LaneChangePhase::None && trace != nullptr &&
        trace->enabled) {
        std::ostringstream detail;
        detail << "phase=" << static_cast<int>(data.laneChange.phase)
               << " route_mode=" << static_cast<int>(data.laneChange.routeMode)
               << " safe=" << candidateSafeAtManeuverSpeed
               << " collision=" << maneuverSpeedSafety.collision
               << " predicted_collision=" << maneuverSpeedSafety.predictedCollision
               << " predicted_ttc=" << maneuverSpeedSafety.predictedCollisionTTC
               << " risk_object=" << maneuverSpeedSafety.riskObjectId;
        traceDecision(trace, "lane_change", "maneuver_speed_safety_checked",
                      detail.str());
    }

    const auto& successorPath = corridorTrackingPathValid
        ? data.roadLanePlanner.waypoints : data.pathPlanner.waypoints;
    const bool maneuverObstacleStillConfirmed = !canCancelClearedPreparedAvoidance(
        data.laneChange, sensedWorld, successorPath, successorPathReady);
    const bool finishingAvoidance = data.laneChange.phase != LaneChangePhase::None &&
        !data.laneChange.roadConnectionManeuver &&
        !data.laneChange.gatewayReplanRequestedAfterShift;
    advanceLaneChangePhase(data.laneChange, data.laneChangeConfig, world, dt,
                           laneChangeExecutionBlocked,
                           maneuverObstacleStillConfirmed,
                           candidateSafeAtManeuverSpeed,
                           trace, successorPathReady);
    if (finishingAvoidance && data.laneChange.phase == LaneChangePhase::None &&
        data.missionCorridor.topologyAuthority && map.corridorRuntime) {
        requestCorridorGlobalReplan(*map.corridorRuntime);
        traceDecision(trace,"corridor_global_route","post_avoidance_replan_requested",
                      "avoidance_finished=1 keep_current_trajectory_until_replacement=1");
    }

    if (successorActivePathFallbackUsed &&
        !committedActivePathProjectionValid &&
        data.laneChange.phase != LaneChangePhase::None && successorPathReady) {
        // Complete the transactional hand-off in the state machine too.  On
        // the next frame normal gateway/obstacle planning may commit a new
        // manoeuvre from the measured successor lane; the stale path can no
        // longer force invalid-input braking or overwrite that choice.
        data.laneChange.phase = LaneChangePhase::Settle;
        data.laneChange.phaseElapsed = 0.0;
        data.laneChange.waitingForReturn = false;
        data.laneChange.turnSignal = 0;
        traceDecision(trace, "lane_change", "phase_transition",
                      "detached_committed_path->settle successor_physically_attached=1");
    }

    // 기동에 진입한 뒤에는 정적 물체의 종방향 판정을 전역경로
    // isOnEgoPath로 다시 하지 않는다. 복귀 구간이 멀리서 잠시 막혀도
    // 현재 회피 차선은 안전할 수 있다. 실제 선택 경로의 정적 충돌은
    // 아래 OBB 예측이 계속 검사한다.
    const bool safelyAvoidingObstacle =
        data.laneChange.phase != LaneChangePhase::None &&
        !data.laneChange.roadConnectionManeuver;
    selectStopDecision(data.frame.stop, data.stopManager,
                       sensedWorld.frontBumperPathS);


    // LaneChangeData::path가 선택된 순간부터 기동과 settle이
    // 끝날 때까지 유일한 committed trajectory다. Prepare에서
    // 일반 차선 경로로 되돌아갔다 Execute에서 다시 곡선으로
    // 바꾸거나, Settle에서 짧은 lane tail로 되돌아가면 급조향과
    // 경로 공백이 생긴다. 후보 경로에는 prepare 시간 동안
    // 직진할 prefix가 이미 포함되어 있으므로 전 상태에서
    // 같은 경로를 지속 추종한다.
    double selectedPathClearance=data.laneChangeConfig.latticeObstacleClearance;
    const auto corridorClearance=[&] {
        return corridorTrackingPathValid && map.corridorRuntime &&
               map.corridorRuntime->trackingPathApproved &&
               map.corridorRuntime->trackingPathValidatedClearance>=0.
            ?map.corridorRuntime->trackingPathValidatedClearance
            :data.laneChangeConfig.latticeObstacleClearance;
    };
    if (data.laneChange.phase != LaneChangePhase::None) {
        const PathPlan successorTracking = tracking;
        PathPlan maneuverTracking = trackLaneChangePath(
            data.laneChange, data.pathPlanner.config, gt, world.egoSpeed);
        const bool useSuccessorTracking = successorTracking.valid && successorPathReady &&
            (!maneuverTracking.valid || successorActivePathFallbackUsed);
        if (!useSuccessorTracking) {
            tracking = maneuverTracking;
            makeLaneChangeLocalPath(data.frame.localPath, data.laneChange);
            if(data.laneChange.validatedObstacleClearance>=0.)
                selectedPathClearance=data.laneChange.validatedObstacleClearance;
        } else {
            tracking = successorTracking;
            const PathPlannerData& selectedPlanner = corridorTrackingPathValid
                ? data.roadLanePlanner : data.pathPlanner;
            generateLocalPath(data.frame.localPath, selectedPlanner,
                              tracking.nearestIndex,
                              data.config.localPathHorizon);
            selectedPathClearance=corridorClearance();
            traceDecision(trace, "road_corridor",
                          "exhausted_maneuver_tracking_handoff",
                          "reason=settle_path_finished successor_tracking_valid=1");
        }
    } else {
        const PathPlannerData& selectedPlanner = corridorTrackingPathValid
            ? data.roadLanePlanner : data.pathPlanner;
        generateLocalPath(data.frame.localPath, selectedPlanner,
                          tracking.nearestIndex,
                          data.config.localPathHorizon);
        selectedPathClearance=corridorClearance();
    }
    // The competition checkpoints are approximate road anchors, not points
    // that Ego must touch.  Nevertheless every paired gateway denotes an
    // upcoming intersection.  A corridor path can be perfectly straight up
    // to that road boundary and expose the connector curvature only after
    // crossing it; curvature-only speed control then discovers a 90-degree
    // turn one frame too late.  Enter a common manoeuvre-speed envelope while
    // approaching (and crossing) every gateway.  This uses no scenario ID or
    // checkpoint lane and leaves Ego free to use every legal corridor lane.
    if (tracking.valid && missionActive) {
        const MissionLegData* leg = activeMissionLeg(data.missionCorridor);
        if (leg != nullptr &&
            leg->gatewayCheckpoint < data.missionCorridor.checkpoints.size()) {
            const Checkpoint& gateway = data.missionCorridor
                .checkpoints[leg->gatewayCheckpoint].checkpoint;
            const double gatewayDistance = std::hypot(
                static_cast<double>(world.ego.x) - gateway.x,
                static_cast<double>(world.ego.y) - gateway.y);
            if (gatewayDistance <=
                data.laneChangeConfig.roadConnectionPrepareDistance) {
                const double before = tracking.targetSpeed;
                tracking.targetSpeed = std::min(
                    tracking.targetSpeed,
                    data.laneChangeConfig.maximumLaneChangeSpeed);
                if (before > tracking.targetSpeed + 1e-6) {
                    std::ostringstream detail;
                    detail << "gateway_distance=" << gatewayDistance
                           << " speed_cap=" << tracking.targetSpeed
                           << " reason=intersection_connector_preview";
                    traceDecision(trace, "road_corridor",
                                  "gateway_approach_speed_limited",
                                  detail.str());
                }
            }
        }
    }
    constexpr double kSharpCurveReleaseHoldTime = 4.0;
    if (tracking.valid &&
        tracking.maxCurvature >= data.pathPlanner.config.sharpCurveThreshold) {
        data.sharpCurveSpeedHoldRemaining = kSharpCurveReleaseHoldTime;
    } else {
        data.sharpCurveSpeedHoldRemaining = std::max(
            0.0, data.sharpCurveSpeedHoldRemaining - std::max(0.0, dt));
    }
    if (tracking.valid && data.sharpCurveSpeedHoldRemaining > 0.0) {
        tracking.targetSpeed = std::min(
            tracking.targetSpeed, data.pathPlanner.config.sharpCurveSpeed);
    }
    // 전역경로 또는 회피 지역경로 중 이번 프레임에 실제 선택된 경로를 기준으로
    // 곡률/구역 제한속도를 계산한다.
    crosswalkControlActive = updateSpeedConstraints(
        data, sensedWorld, map.staticZones, tracking.targetSpeed,
        schoolZoneActive,trace);
    if (!world.inputValid || world.respawnDetected) {
        data.behaviorState.health = SystemHealth::Recovery;
    }
    data.behaviorState.staticObstacleAvoidanceActive =
        safelyAvoidingObstacle;
    data.behaviorState.safelyAvoidedObjectId = safelyAvoidingObstacle
        ? data.laneChange.obstacleId
        : -1;
    updateBehavior(data.behaviorState, data.frame.behavior,
                   data.frame.constraints, data.frame.stop, world,
                   tracking.targetSpeed, data.config, dt);
    data.behaviorState.laneChange = data.laneChange.phase;
    data.frame.behavior.laneChange = data.laneChange.phase;
    data.frame.behavior.turnSignal = data.laneChange.turnSignal;

    // Gateway 횡이동 edge가 다음 section에 있으면 현재 section에서는
    // 아직 차선변경 후보를 만들 수 없다. 이 때 일반 주행속도를
    // 유지하면 lateral edge가 실제로 열린 순간에는 이미 너무
    // 빨라 정지선 앞에서 경로가 뒤로 밀린다. 전역경로가 아니라
    // 미리 계산된 mission lane graph의 첫 lateral edge까지의 실제
    // local-path 잔여거리를 사용해, edge 진입 전에 횡이동 속도로
    // 감속을 완료한다.
    if (missionActive &&
        (corridorTrackingPathValid || guideContinuityPathValid) &&
        data.laneChange.phase == LaneChangePhase::None &&
        world.egoLaneValid) {
        const LaneKey currentLane{
            world.egoRoadId, world.egoSectionIndex, world.egoLaneId};
        std::vector<LaneKey> routeToGateway;
        if (findMissionLaneRoute(
                map, data.missionCorridor, currentLane, routeToGateway) &&
            std::adjacent_find(
                routeToGateway.begin(), routeToGateway.end(),
                [](const LaneKey& from, const LaneKey& to) {
                    return from.roadId == to.roadId &&
                           from.sectionIndex == to.sectionIndex &&
                           from.laneId != to.laneId;
                }) != routeToGateway.end()) {
            const double distanceToGatewayAction = std::max(
                0.0,
                (corridorTrackingPathValid
                     ? data.roadLanePlanner.waypoints.back().s
                     : data.pathPlanner.waypoints.back().s) -
                    world.egoPathS);
            if (distanceToGatewayAction <=
                data.laneChangeConfig.roadConnectionPrepareDistance) {
                const double before = data.frame.behavior.targetSpeed;
                data.frame.behavior.targetSpeed = std::min(
                    before, data.laneChangeConfig.maximumLaneChangeSpeed);
                if (before > data.frame.behavior.targetSpeed + 1e-6) {
                    std::ostringstream detail;
                    detail << "distance_to_lateral_edge="
                           << distanceToGatewayAction
                           << " speed_cap="
                           << data.laneChangeConfig.maximumLaneChangeSpeed;
                    traceDecision(trace, "road_corridor",
                                  "gateway_approach_speed_limited",
                                  detail.str());
                }
            }
        }
    }

    // 확인된 정지 장애물이 있는데도 아직 안전한 차선변경 후보를 못 찾았으면
    // (신호 대기, 도로표시 제약, 옆 차선 간격 부족 등으로 이번 프레임엔 회피가
    // 안 되는 경우) 일반 Follow 감속만으로는 장애물에 도달했을 때 옆으로
    // 옮길 거리(전환거리+통과여유)가 남아있는다는 보장이 없다. 회피가 실제로
    // 가능해지는 순간 곧바로 완주할 수 있도록, 장애물까지 그 거리가 남을
    // 때 회피 기동 속도(maximumLaneChangeSpeed)에 도달하게 미리 감속한다.
    if (data.laneChange.phase == LaneChangePhase::None &&
        pendingBlockerDistance >= 0.0) {
        const double reserveDistance = data.laneChangeConfig.transitionLength +
                                       data.laneChangeConfig.obstacleClearance;
        const double distanceToSlow =
            std::max(0.0, pendingBlockerDistance - reserveDistance);
        const double maneuverSpeed = data.laneChangeConfig.maximumLaneChangeSpeed;
        const double approachCap = std::sqrt(
            maneuverSpeed * maneuverSpeed +
            2.0 * data.config.comfortableDeceleration * distanceToSlow);
        data.frame.behavior.targetSpeed =
            std::min(data.frame.behavior.targetSpeed, approachCap);
    }

    // Keep the committed route/phase intact. Only relax its speed contract on
    // a measured, settled straight portion, with braking room before its end.
    double maneuverCap=data.laneChangeConfig.maximumLaneChangeSpeed;
    const double straightCap=settledStraightManeuverSpeedCap(data.laneChange,world,
        data.frame.localPath.points,data.frame.behavior.targetSpeed,selectedPathClearance,
        data.laneChangeConfig,data.config);
    if(straightCap>maneuverCap+.05 && data.frame.localPath.valid && !data.frame.stop.valid &&
       !sensedWorld.respawnDetected && !data.progressRecovery.active) {
        data.settledStraightCruiseTime+=std::clamp(dt,0.,.1);
        if(data.settledStraightCruiseTime>=data.laneChangeConfig.settleTime) {
            PathSafetyResult cruiseSafety;
            if(evaluateLaneChangeCandidate(cruiseSafety,data.laneChange,world,data.config,
                std::max(world.egoSpeed,straightCap),map)) maneuverCap=straightCap;
        }
    } else data.settledStraightCruiseTime=0.;
    const bool lateralShiftActive =
        data.laneChange.phase != LaneChangePhase::None;
    if (lateralShiftActive) {
        data.frame.behavior.targetSpeed = std::min(
            data.frame.behavior.targetSpeed,
            maneuverCap);
    }
    if (data.laneChange.phase == LaneChangePhase::Execute) {
        if (data.laneChange.waitingForReturn) {
            data.frame.behavior.targetSpeed = std::min(
                data.frame.behavior.targetSpeed,
                data.laneChangeConfig.returnWaitingSpeed);
        }
    }

    // Only certified, gentle local detours get the open-passage cap. Tight
    // tails retain the old low speed, including after manoeuvre handoff.
    if ((data.laneChange.localCorridorAvoidance && maneuverCap<=data.laneChangeConfig.maximumLaneChangeSpeed &&
         data.laneChange.phase != LaneChangePhase::None) ||
        (selectedPathClearance >= 0.0 && selectedPathClearance + 1e-6 <
             data.laneChangeConfig.latticeObstacleClearance)) {
        data.frame.behavior.targetSpeed = std::min(
            data.frame.behavior.targetSpeed,
            localFallbackSpeedLimit(data.laneChangeConfig,
                data.laneChange.localCorridorAvoidance &&
                data.laneChange.phase!=LaneChangePhase::None &&
                data.laneChange.validatedObstacleClearance<0. ? -1. : selectedPathClearance,
                tracking.maxCurvature,data.pathPlanner.config.maximumLateralAcceleration));
    }

    // A stored trajectory may remain useful for replanning even when a new
    // stationary body blocks it. Minimum-progress rules must not authorize
    // driving through that body. Check the actual selected path and its
    // certified clearance after all speed floors; retain its geometry.
    int stationaryBlocker=-1;
    const double freeDistance=data.frame.localPath.valid
        ?corridorStationaryFreeDistance(data.frame.localPath.points,world,
            selectedPathClearance,&map,&stationaryBlocker)
        :std::numeric_limits<double>::infinity();
    bool stationaryPathStop=false;
    if(stationaryBlocker>=0 && std::isfinite(freeDistance)) {
        const double usable=std::max(0.,freeDistance-data.config.stopSafetyMargin-
            std::max(0.,world.egoSpeed)*data.config.emergencyReactionTime);
        const double cap=std::sqrt(2.*data.config.comfortableDeceleration*usable);
        data.frame.behavior.targetSpeed=std::min(data.frame.behavior.targetSpeed,cap);
        stationaryPathStop=cap<=0.05;
        if(!data.frame.stop.valid)
            data.frame.behavior.longitudinal=stationaryPathStop?LonMode::Stop:LonMode::Follow;
        bool globalReplanRequired = false;
        if (map.corridorRuntime) {
            auto& runtime = *map.corridorRuntime;
            // This speed guard checks the SELECTED local path. A blocked
            // temporary manoeuvre is not proof that the cached gateway route
            // is blocked too. Invalidating both caused a successful global
            // worker to restart repeatedly instead of ever being followed.
            globalReplanRequired = !runtime.committedGlobalPathReachesGateway ||
                runtime.committedGlobalPathLeg != data.missionCorridor.activeLegIndex ||
                !std::isinf(corridorStationaryFreeDistance(
                    runtime.committedGlobalPath, world,
                    runtime.globalPathClearance(), &map));
            if (globalReplanRequired && !runtime.globalRouteReplanRequested)
                requestCorridorGlobalReplan(runtime);
        }
        std::ostringstream detail;
        detail<<"object="<<stationaryBlocker<<" free_distance="<<freeDistance
              <<" speed_cap="<<cap<<" clearance="<<selectedPathClearance
              <<" geometry_retained=1 replan_required=1 global_replan_required="
              <<globalReplanRequired;
        traceDecision(trace,"planning","selected_path_stationary_clearance_limit",detail.str());
    }

    const double gatewayQueueGap=soleGatewayQueueDistance(world,map,data.missionCorridor);
    if(map.corridorRuntime && activeMissionLeg(data.missionCorridor) &&
       !activeMissionLeg(data.missionCorridor)->gatewayLanes.empty() &&
       !data.progressRecovery.active && data.laneChange.phase==LaneChangePhase::None &&
       std::isfinite(map.corridorRuntime->trackingGatewayStopDistance)) {
        const double remaining=map.corridorRuntime->trackingGatewayStopDistance;
        const double usable=std::max(0.,remaining-data.config.stopSafetyMargin-
            std::max(0.,world.egoSpeed)*data.config.emergencyReactionTime);
        data.frame.behavior.targetSpeed=std::min(data.frame.behavior.targetSpeed,
            std::sqrt(2.*data.config.comfortableDeceleration*usable));
        traceDecision(trace,"sole_gateway","alignment_endpoint_speed_limit",
            "remaining="+std::to_string(remaining)+" unreachable_straight_suffix_forbidden=1");
    }
    // A newly certified local bypass owns tracking, not the old approach
    // prefix underneath it. Do not pin its speed to the obsolete stop point.
    const bool safeLocalBypass=localFallbackOwnsPlanning &&
        data.laneChange.phase!=LaneChangePhase::None && tracking.valid &&
        data.frame.localPath.valid && std::isinf(freeDistance) &&
        queuedGatewayBypassIsComplete(data.laneChange.path,data.missionCorridor);
    const bool gatewayQueue=std::isfinite(gatewayQueueGap) && !safeLocalBypass;
    // This API reports the signal relevant to Ego. Map controller IDs may
    // be unavailable/mismatched; do not turn a received green into an
    // indefinite queue. Actual regulatory stop requests retain authority;
    // an unassigned signal on an unsignalized road is not a stop command.
    const bool queueGreen=world.trafficLight.id>=0 &&
        (world.trafficLight.state==3 || world.trafficLight.state==5);
    double gatewayQueueSpeedCap=std::numeric_limits<double>::infinity();
    // A collision-free APPROACH PREFIX ends before its blocker. Its lack of
    // collisions does not authorize driving beyond its end at cruise speed.
    // Brake for that endpoint while it is the selected tracking authority.
    if(map.corridorRuntime && map.corridorRuntime->soleGatewayApproachSelected && !safeLocalBypass) {
        const auto& approach=map.corridorRuntime->soleGatewayApproachPath;
        if(approach.size()>=2) {
            const auto near=std::min_element(approach.begin(),approach.end(),
                [&](const auto& a,const auto& b){return
                    std::hypot(a.x-world.ego.x,a.y-world.ego.y)<
                    std::hypot(b.x-world.ego.x,b.y-world.ego.y);});
            const double remaining=std::max(0.,approach.back().s-near->s);
            const double usable=std::max(0.,remaining-
                std::max(0.,world.egoSpeed)*data.config.emergencyReactionTime);
            const double cap=std::sqrt(2.*data.config.comfortableDeceleration*usable);
            gatewayQueueSpeedCap=std::min(gatewayQueueSpeedCap,cap);
            data.frame.behavior.targetSpeed=std::min(data.frame.behavior.targetSpeed,cap);
            stationaryPathStop=stationaryPathStop || cap<=.05;
            traceDecision(trace,"sole_gateway","approach_endpoint_speed_limit",
                "remaining="+std::to_string(remaining)+" speed_cap="+std::to_string(cap));
        }
    }
    if(gatewayQueue) {
        const double usable=std::max(0.,gatewayQueueGap-data.config.stopSafetyMargin-
            selectedPathClearance-std::max(0.,world.egoSpeed)*data.config.emergencyReactionTime);
        const double cap=std::sqrt(2.*data.config.comfortableDeceleration*usable);
        gatewayQueueSpeedCap=std::min(gatewayQueueSpeedCap,cap);
        data.frame.behavior.targetSpeed=std::min(data.frame.behavior.targetSpeed,cap);
        stationaryPathStop=stationaryPathStop || cap<=.05;
        if(!data.frame.stop.valid)
            data.frame.behavior.longitudinal=stationaryPathStop?LonMode::Stop:LonMode::Follow;
        traceDecision(trace,"sole_gateway","entrance_vehicle_queue",
            "gap="+std::to_string(gatewayQueueGap)+
            " green_static_timeout="+std::to_string(queueGreen));
    }

    // 미래 충돌검사가 선택된 지역경로를 사용한다.
    resetPathSafety(data.frame.pathSafety);
    checkCurrentCollisions(data.frame.pathSafety, world.egoObb,
                           world.objects,-1,&world,&map);
    updateMinimumTTC(data.frame.pathSafety, world.objects,
                     safelyAvoidingObstacle,
                     safelyAvoidingObstacle
                         ? data.laneChange.obstacleId
                         : -1,&data.frame.localPath,&map);
    checkPredictedCollisions(data.frame.pathSafety, world,
                             data.frame.localPath,
                             data.config.collisionPredictionHorizon,
                             data.config.collisionPredictionStep,false,&map);
    // 비상제동은 급정거·끼어들기 같은 동적 위험과 현재 실제
    // 충돌에만 적용한다. 앞에 멈춰 있는 정적 장애물의 미래
    // 충돌은 속도 0 명령으로 해결할 대상이 아니라 차선회피 대상이다.
    WorldState dynamicEmergencyWorld = world;
    dynamicEmergencyWorld.objects.erase(
        std::remove_if(dynamicEmergencyWorld.objects.begin(),
                       dynamicEmergencyWorld.objects.end(),
                       [](const WorldObjectState& object) {
                           return std::abs(object.raw.speed) < 0.3;
                       }),
        dynamicEmergencyWorld.objects.end());
    PathSafetyResult dynamicEmergencySafety;
    resetPathSafety(dynamicEmergencySafety);
    checkCurrentCollisions(dynamicEmergencySafety, world.egoObb,
                           dynamicEmergencyWorld.objects,-1,&world,&map);
    updateMinimumTTC(dynamicEmergencySafety,
                     dynamicEmergencyWorld.objects, false,-1,
                     &data.frame.localPath,&map);
    checkPredictedCollisions(dynamicEmergencySafety, dynamicEmergencyWorld,
                             data.frame.localPath,
                             data.config.collisionPredictionHorizon,
                             data.config.collisionPredictionStep,false,&map);
    // 복귀 경로가 멀리서 막혀 있을 때는 검증된 저속 복귀를 하거나
    // 위의 대기 지점 속도로 감속한다. 충돌 예측이 있다는
    // 사실만으로 즉시 -4 m/s^2를 명령하지 않고, 현재 충돌이거나
    // TTC가 임계 이내로 들어온 실제 긴급 상황에서만 비상제동한다.
    const double speedAwareEmergencyTTC = std::max(
        data.config.emergencyTTC,
        data.config.emergencyReactionTime +
            data.config.collisionPredictionStep +
            std::max(0.0, world.egoSpeed) /
                data.config.emergencyBrakingDeceleration);
    int physicalRiskObjectId = -1;
    const bool crossingEmergency = crossingObjectRequiresEmergency(
        world, data.frame.localPath.points, data.config,
        physicalRiskObjectId);
    // 급제동 선행차는 차선변경 중에도 검사하되, 원래 전역경로의
    // isOnEgoPath가 아니라 이번 프레임에 실제 추종하는 지역경로를 쓴다.
    // 따라서 이미 옆으로 피한 정적 장애물은 비상제동을 만들지 않는다.
    // Stationary obstacles remain avoidance targets even after a failed
    // candidate search. Do not convert that failure into a stationary-lead
    // emergency-brake request; the lattice retries with expanded candidates.
    const bool leadEmergency = leadObjectRequiresEmergency(
        world, data.frame.localPath, data.config, physicalRiskObjectId,
        false);
    // A confirmed static road obstacle is an avoidance/replanning problem,
    // not an EBS target.  Objects that have just stopped after moving remain
    // dynamic until the existing confirmation timer expires.  VTD does not
    // transmit a semantic class, so keep pedestrian/cyclist-sized standing
    // objects under the original emergency protection using their GT box.
    bool confirmedStaticCollision = false;
    if (data.frame.pathSafety.collision &&
        data.frame.pathSafety.riskObjectId >= 0) {
        const auto risk = std::find_if(
            world.objects.begin(), world.objects.end(),
            [&](const WorldObjectState& object) {
                return static_cast<int>(object.raw.id) ==
                       data.frame.pathSafety.riskObjectId;
            });
        if (risk != world.objects.end()) {
            const bool vulnerableRoadUserSized =
                risk->raw.length <= 2.5 && risk->raw.width <= 1.5 &&
                risk->raw.height >= 1.0;
            confirmedStaticCollision = !vulnerableRoadUserSized &&
                std::find(confirmedStationaryObjectIds.begin(),
                          confirmedStationaryObjectIds.end(),
                          static_cast<int>(risk->raw.id)) !=
                    confirmedStationaryObjectIds.end();
        }
    }
    const bool collisionEmergency =
        data.frame.pathSafety.collision && !confirmedStaticCollision;
    const bool rawEmergency = collisionEmergency || crossingYield ||
        requiresEmergencyBrake(
            dynamicEmergencySafety, speedAwareEmergencyTTC) ||
        crossingEmergency || leadEmergency;
    if (rawEmergency) {
        data.emergencyBrakeHoldRemaining =
            data.config.emergencyMinimumHoldTime;
    } else {
        data.emergencyBrakeHoldRemaining = std::max(
            0.0, data.emergencyBrakeHoldRemaining - std::max(0.0, dt));
    }
    bool emergency = rawEmergency ||
        data.emergencyBrakeHoldRemaining > 0.0;
    const bool missionComplete = data.missionCorridor.valid &&
        activeMissionLeg(data.missionCorridor) == nullptr;
    const bool finalGoalReached = missionComplete ||
        (!data.missionCorridor.topologyAuthority && crossedFinishLine(
            data, sensedWorld, map.staticZones,
            !data.missionCorridor.valid && globalTracking.goalReached));
    const bool progressOverride = applySimulationProgressRecovery(
        data, gt, sensedWorld, world, map, tracking, emergency,
        stationaryPathStop, stationaryBlocker, selectedPathClearance, finalGoalReached,
        gatewayQueue, queueGreen, gatewayQueueSpeedCap,
        confirmedStationaryObjectIds, dt, trace);
    // Cover global-path lane changes and EVERY shift in a multistep local
    // path, not just the first candidate's outward direction. Only lamps
    // change here: no path, steering, speed or safety decision is modified.
    const auto previewSignal=previewTrajectoryTurnSignal(
        sensedWorld,map,data.frame.localPath.points);
    maintainTrajectoryTurnSignal(data,sensedWorld,previewSignal,dt);
    if (data.trajectoryTurnSignal!=0 || data.laneChange.localCorridorAvoidance) {
        data.frame.behavior.turnSignal=data.trajectoryTurnSignal;
    }
    if (trace != nullptr && trace->enabled) {
        std::ostringstream detail;
        detail << "ego_road=" << world.egoRoadId
               << " ego_section=" << world.egoSectionIndex
               << " ego_lane=" << world.egoLaneId
               << " ego_path_s=" << world.egoPathS
               << " ego_path_d=" << world.egoPathD
               << " ego_speed=" << world.egoSpeed
               << " school_zone=" << schoolZoneActive
               << " crosswalk_control=" << crosswalkControlActive
               << " lane_change_phase="
               << static_cast<int>(data.laneChange.phase)
               << " target_lane=" << data.laneChange.targetLaneId
               << " maneuver_path_safe="
               << candidateSafeAtManeuverSpeed
               << " stop_valid=" << data.frame.stop.valid
               << " stop_distance=" << data.frame.stop.distanceToStop
               << " min_ttc=" << data.frame.pathSafety.minimumTTC
               << " collision=" << data.frame.pathSafety.collision
               << " predicted_collision="
               << data.frame.pathSafety.predictedCollision
               << " predicted_ttc="
               << data.frame.pathSafety.predictedCollisionTTC
               << " risk_object=" << data.frame.pathSafety.riskObjectId
               << " emergency=" << emergency
               << " raw_emergency=" << rawEmergency
               << " collision_emergency=" << collisionEmergency
               << " confirmed_static_collision="
               << confirmedStaticCollision
               << " emergency_hold_remaining="
               << data.emergencyBrakeHoldRemaining
               << " emergency_ttc_threshold=" << speedAwareEmergencyTTC
               << " physical_risk_object=" << physicalRiskObjectId
               << " dense_static_hold=" << data.behaviorState.denseStaticHoldRemaining
               << " progress_recovery=" << progressOverride
               << " target_speed=" << data.frame.behavior.targetSpeed;
        traceDecision(trace, "planning", "frame_result", detail.str());
    }
    data.frame.behavior.emergencyBrake = emergency;
    if (emergency) data.frame.behavior.targetSpeed = 0.0;

    tracking.targetSpeed = data.frame.behavior.targetSpeed;
    tracking.goalReached = finalGoalReached;
    data.pathPlanner.latestPlan = tracking;

    data.result.valid = tracking.valid && world.inputValid;
    data.result.goalReached = tracking.goalReached;
    data.result.targetSpeed = tracking.targetSpeed;
    data.result.stopReason = data.frame.stop.valid ? data.frame.stop.reason :
        !progressOverride && stationaryPathStop
            ? StopReason::StationaryPathBlocked : StopReason::None;
    data.result.longitudinal = data.frame.behavior.longitudinal;
    data.result.health = data.frame.behavior.health;
    data.result.trackingPlan = tracking;
    data.result.localPath = data.frame.localPath;
    data.result.pathSafety = data.frame.pathSafety;
    data.result.emergencyBrake = emergency;
    data.result.turnSignal = data.frame.behavior.turnSignal;
    data.result.currentSpeedLimit = data.frame.constraints.speedLimit;
    data.result.schoolZoneActive = schoolZoneActive;
    data.result.stopDistanceValid = data.frame.stop.valid;
    data.result.stopDistance = data.frame.stop.distanceToStop;
    data.result.crosswalkActive = crosswalkControlActive;
    data.result.laneChangeActive =
        progressOverride || data.laneChange.phase != LaneChangePhase::None;
}
