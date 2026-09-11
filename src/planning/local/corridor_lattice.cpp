#include "planning/local/corridor_lattice.hpp"
#include "planning/local/recovery_selection.hpp"
#include "planning/local/corridor_surface_query.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/collision/collision_checker.hpp"
#include "logger/decision_trace.hpp"
#include "utils/geometry.hpp"
#include "utils/math_utils.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <future>

bool connectToReferencePath(std::vector<Waypoint>& output,
    const std::vector<Waypoint>& reference,std::size_t referenceProgress,
    CorridorRuntime& runtime,const PathGeneratorData& map,const WorldState& world,
    double horizon,DecisionTraceData* trace,bool corridorOnly,double clearance=-1.);

bool corridorObstacleAtPathLevel(const PathGeneratorData& map,
                                const Waypoint& point,
                                const WorldObjectState& object) {
    return roadPointSharesObjectLevel(map,point,object.raw.x,object.raw.y,object.raw.z);
}

// Own immutable map/graph snapshots. The control loop only polls the result;
// it never waits for global A* or shares mutable planning state with it.
struct CorridorGlobalRouteWorker {
    struct Result {
        std::size_t leg{};
        std::uint64_t revision{}, staticSignature{};
        LaneKey startLane;
        double startRoadS{}, receiveTime{};
        double searchMilliseconds{};
        double clearance{-1.};
        bool success{};
        bool exitPreview{};
        bool approachOnly{};
        bool approachInEntranceLane{};
        int approachBlocker{-1};
        std::vector<int> queueBlockers;
        bool exception{};
        std::vector<Waypoint> path;
        std::vector<std::string> diagnostics;
    };
    std::shared_ptr<CorridorRuntime> runtime;
    std::shared_ptr<const PathGeneratorData> map;
    std::future<Result> pending;
};

struct CorridorLocalRouteWorker {
    struct Result {
        std::size_t leg{};
        std::uint64_t epoch{}, signature{};
        double clearance{}, elapsedMs{}, receiveTime{};
        int blocker{-1}, searchTier{};
        bool success{}, timedOut{}, exception{};
        std::size_t expanded{};
        std::vector<Waypoint> path;
        std::vector<std::string> diagnostics;
        std::shared_ptr<CorridorReentryScope> reentryScope;
    };
    std::shared_ptr<CorridorRuntime> runtime;
    std::shared_ptr<const PathGeneratorData> map;
    std::future<Result> pending;
};

namespace {
constexpr double inf=std::numeric_limits<double>::infinity();
constexpr double maximumCurvature=0.176848;
// XODR one-sided derivative probes occur a few micrometres to millimetres
// apart and can carry a deliberately discontinuous yaw.  They describe the
// source derivative, not a drivable controller sample.  The ordinary network
// spacing is about 0.5 m, so removing sub-2 cm duplicates preserves all road
// geometry while preventing a fake section-boundary steering jump.
constexpr double minimumTrackingSampleSpacing=0.02;
LaneKey keyOf(const Waypoint& p) { return {p.roadId,p.sectionIndex,p.laneId}; }
double gap(const Waypoint& a,const Waypoint& b) { return std::hypot(a.x-b.x,a.y-b.y); }
VehicleFootprint2D egoFootprint(const WorldState& world) {
    return vehicleFootprintFromObb(
        world.egoObb,{world.ego.x,world.ego.y,world.ego.heading});
}
bool visible(const PrecomputedLaneNetwork& n,std::int64_t id) {
    if(id<0 || id>=static_cast<std::int64_t>(n.points.size())) return false;
    const auto& p=n.points[id];
    const auto lane=n.lanes.find(keyOf(p.pose));
    return p.replacedByTransition<0 && lane!=n.lanes.end() && lane->second.topologyStatus!="excluded_by_review";
}
std::int64_t containing(const CorridorRuntime& r,double x,double y,double yaw,
                        bool corridorOnly=true,const std::vector<bool>* surfaceOverride=nullptr);
void addEdge(CorridorRuntime& r,std::int64_t from,CorridorEdge e) {
    if(!visible(r.network,from) || !visible(r.network,e.to)) return;
    r.forward[from].push_back(e);
    r.reverse[e.to].push_back({from,e.distance,e.transition,e.lateral});
}
void activate(CorridorRuntime& r,const MissionCorridorData& mission) {
    if(r.activeLeg==mission.activeLegIndex) return;
    r.pendingGatewayHandoffPath.clear();
    r.pendingGatewayHandoffClearance=r.globalPathClearance();
    if(r.activeLeg!=static_cast<std::size_t>(-1) &&
       r.activeLeg+1==mission.activeLegIndex &&
       r.committedGlobalPathHasExitPreview &&
       r.committedGlobalPathProgress<r.committedGlobalPath.size()) {
        r.pendingGatewayHandoffPath.assign(
            r.committedGlobalPath.begin()+r.committedGlobalPathProgress,
            r.committedGlobalPath.end());
    }
    r.activeLeg=mission.activeLegIndex;
    r.committedTransition=-1;
    r.committedTransitionLeg=mission.activeLegIndex;
    r.committedTransitionProgress=0;
    r.committedTransitionEntered=false;
    r.committedGlobalPath.clear();
    r.committedGlobalPathClearance=-1.;
    r.committedGlobalPathLeg=mission.activeLegIndex;
    r.committedGlobalPathProgress=0;
    r.committedGlobalPathReachesGateway=false;
    r.committedGlobalPathHasExitPreview=false;
    r.activeGatewayObserved=false;
    r.soleGatewayApproachPath.clear();r.soleGatewayApproachSelected=false;
    r.soleGatewayApproachInEntranceLane=false;
    r.gatewayQueueBlockers.clear();
    r.recoveryTrackingPath.clear();
    r.recoveryTrackingPathLeg=mission.activeLegIndex;
    r.recoveryTrackingPathProgress=0;
    r.recoveryTrackingPathClearance=-1.;
    r.recoveryTrackingPathReachesReference=false;
    r.recoveryTrackingPathRelocalized=false;r.trackingPathRelocalized=false;
    r.localFallbackRelocalizationRequested=false;
    r.referenceRecoveryFailureLane={};
    r.referenceRecoveryFailureValid=false;
    r.referenceRecoveryFailureRoadS=0.;
    r.referenceRecoveryRetryAfter=0.;
    r.globalRouteSearchFailed=false;
    r.globalRouteReplanRequested=false;
    ++r.globalRouteRequestRevision;
    r.globalRouteFailureLane={};
    r.globalRouteFailureStaticCount=0;
    r.globalRouteFailureRoadS=0.;
    r.globalRouteRetryAfter=0.;
    r.searchCursor={};
    r.searchLeg=mission.activeLegIndex;
    r.obstacleSearchObject=-1;
    const auto* leg=activeMissionLeg(mission);
    r.allowed.assign(r.network.points.size(),false);
    r.trackingAllowed.assign(r.network.points.size(),false);
    r.surfaceAllowed.assign(r.network.points.size(),false);
    r.globalSearchAllowed.assign(r.network.points.size(),false);
    r.globalSearchGoal.assign(r.network.points.size(),false);
    r.globalExitPreviewActive=false;
    r.gatewayCost.assign(r.network.points.size(),inf);
    r.gatewayNext.assign(r.network.points.size(),-1);
    r.gatewaySpine.assign(r.network.points.size(),false);
    r.handoffCost.assign(r.network.points.size(),inf);
    r.gatewayLateralSteps.assign(r.network.points.size(),
                                 std::numeric_limits<int>::max());
    ++r.routeRevision;
    if(!leg) return;
    using Item=std::pair<double,std::int64_t>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    for(const auto& p:r.network.points) {
        const auto key=keyOf(p.pose);
        r.allowed[p.id]=visible(r.network,p.id) && (leg->allowedLanes.count(key)||leg->targetLanes.count(key));
        r.trackingAllowed[p.id]=r.allowed[p.id];
        r.surfaceAllowed[p.id]=(leg->allowedLanes.count(key)||leg->targetLanes.count(key)) &&
            r.network.lanes.at(key).topologyStatus!="excluded_by_review";
        if(r.allowed[p.id] && missionPointIsTarget(*leg,p.pose)) {
            r.gatewayCost[p.id]=0.;
            r.gatewayLateralSteps[p.id]=0;
            queue.push({0.,p.id});
        }
    }
    // A bounded exit apron is physical clearance, not another route/goal.
    const auto addExitSurfaces=[&](const MissionLegData& source) {
        for(const auto& span:source.surfaceOnlySpans) {
            if(!leg->allowedLanes.count(span.outgoingLane) &&
               !leg->targetLanes.count(span.outgoingLane)) continue;
            const auto lane=r.network.lanes.find(span.lane);
            if(lane==r.network.lanes.end() ||
               lane->second.topologyStatus=="excluded_by_review") continue;
            for(const auto id:lane->second.pointIds) {
                const auto& p=r.network.points[id];
                if(p.pose.roadS>=span.roadSMin-1e-6 &&
                   p.pose.roadS<=span.roadSMax+1e-6) r.surfaceAllowed[id]=true;
            }
        }
    };
    addExitSurfaces(*leg);
    if(mission.activeLegIndex>0)
        addExitSurfaces(mission.legs[mission.activeLegIndex-1]);
    while(!queue.empty()) {
        const auto [cost,id]=queue.top();queue.pop();
        if(cost>r.gatewayCost[id]+1e-8) continue;
        for(const auto& e:r.reverse[id]) {
            if(!r.allowed[e.to]) continue;
            const double next=cost+e.distance+(e.lateral?8.:0.);
            if(next+1e-8<r.gatewayCost[e.to]) {
                r.gatewayCost[e.to]=next;
                r.gatewayNext[e.to]=id;
                queue.push({next,e.to});
            }
        }
    }
    // A separate 0-1 shortest path records how many lane changes remain.
    // Distance alone cannot tell a 30 m straight continuation from one legal
    // lateral stage, which previously rejected safe intermediate lanes.
    std::deque<std::int64_t> stepQueue;
    for(const auto& p:r.network.points) if(r.allowed[p.id] &&
        missionPointIsTarget(*leg,p.pose)) stepQueue.push_back(p.id);
    while(!stepQueue.empty()) {
        const auto id=stepQueue.front();stepQueue.pop_front();
        for(const auto& e:r.reverse[id]) {
            const auto predecessor=e.to;
            if(!r.allowed[predecessor]) continue;
            const int next=r.gatewayLateralSteps[id]+(e.lateral?1:0);
            if(next>=r.gatewayLateralSteps[predecessor]) continue;
            r.gatewayLateralSteps[predecessor]=next;
            if(e.lateral) stepQueue.push_back(predecessor);
            else stepQueue.push_front(predecessor);
        }
    }
    for(const auto& p:r.network.points) {
        r.gatewaySpine[p.id]=r.allowed[p.id] &&
            r.gatewayLateralSteps[p.id]==0 &&
            std::isfinite(r.gatewayCost[p.id]);
    }

    // A checkpoint is a handoff between two precomputed corridors, not a
    // physical dead end.  Search the union of the current and next leg over
    // every forward lane/merge/split edge and cache a route to the next
    // gateway.  Normal tracking still prefers the current leg; this cache is
    // used only after its target lane has no current-leg continuation.
    if(mission.activeLegIndex+1<mission.legs.size()) {
        const auto& nextLeg=mission.legs[mission.activeLegIndex+1];
        std::priority_queue<Item,std::vector<Item>,std::greater<Item>> handoffQueue;
        for(const auto& p:r.network.points) {
            const auto key=keyOf(p.pose);
            if(nextLeg.allowedLanes.count(key)||nextLeg.targetLanes.count(key)) {
                if(visible(r.network,p.id)) r.trackingAllowed[p.id]=true;
                // Join smoothing validates the physical swept surface.  The
                // next leg is already a prevalidated road corridor, so its
                // surface must be visible during this atomic handoff too.
                // Transition-replaced raw samples are still real pavement,
                // even though they must not become ordinary graph nodes.
                if(r.network.lanes.at(key).topologyStatus!="excluded_by_review")
                    r.surfaceAllowed[p.id]=true;
            }
            if(r.trackingAllowed[p.id] && missionPointIsTarget(nextLeg,p.pose)) {
                r.handoffCost[p.id]=0.;handoffQueue.push({0.,p.id});
            }
        }
        while(!handoffQueue.empty()) {
            const auto [cost,id]=handoffQueue.top();handoffQueue.pop();
            if(cost>r.handoffCost[id]+1e-8) continue;
            for(const auto& e:r.reverse[id]) {
                if(!r.trackingAllowed[e.to]) continue;
                const double next=cost+e.distance+(e.lateral?8.:0.);
                if(next+1e-8<r.handoffCost[e.to]) {
                    r.handoffCost[e.to]=next;
                    handoffQueue.push({next,e.to});
                }
            }
        }
    }
    r.globalSearchAllowed=r.allowed;
    for(const auto& p:r.network.points)
        r.globalSearchGoal[p.id]=r.allowed[p.id] && missionPointIsTarget(*leg,p.pose);
    // Continue through the CURRENT exit into the next ordinary-road corridor.
    // Select frontier samples using directed network distance, not a radius
    // around a checkpoint. Do not open the next junction or the whole map.
    if(mission.topologyAuthority && !leg->finalLeg &&
       mission.activeLegIndex+1<mission.legs.size()) {
        const auto& nextLeg=mission.legs[mission.activeLegIndex+1];
        // An avoidance may need to hold its side until a whole vehicle group
        // has passed. A lane-change length (12/20/30 m) is not an adequate
        // deadline for returning to the required exit lane. Cover the same
        // bounded obstacle horizon as local planning, plus a settled tail.
        // The frontier still stops before the next gateway/intersection.
        const double preview=std::min(120.,std::max({
            r.laneConfig.latticeShortDistance,r.laneConfig.latticeMediumDistance,
            r.laneConfig.latticeLongDistance,r.laneConfig.localFallbackHorizon+
                std::max(8.,r.laneConfig.gatewayReserveDistance)}));
        std::vector<double> distance(r.network.points.size(),inf);
        std::priority_queue<Item,std::vector<Item>,std::greater<Item>> frontier;
        const auto eligible=[&](std::int64_t id) {
            if(!visible(r.network,id) || !r.ordinaryRoadPoint[id]) return false;
            const auto key=keyOf(r.network.points[id].pose);
            return nextLeg.allowedLanes.count(key) && !nextLeg.gatewayLanes.count(key);
        };
        for(const auto& [key,lane]:r.network.lanes) {
            (void)key;
            // One seed per directed exit lane, at the actual exit cross
            // section. Seeding every sample beyond it would extend forever.
            for(const auto id:lane.pointIds) {
                if(!r.globalSearchGoal[id] || !eligible(id)) continue;
                const auto& p=r.network.points[id].pose;
                double anchorDistance=inf;
                for(const auto& [target,station]:leg->targetRoadS)
                    if(target.roadId==p.roadId && target.laneId*p.laneId>0)
                        anchorDistance=std::min(anchorDistance,std::abs(p.roadS-station));
                if(anchorDistance<=1.) {
                    distance[id]=0.;frontier.push({0.,id});
                }
                break;
            }
        }
        std::vector<bool> goals(r.network.points.size(),false);
        while(!frontier.empty()) {
            const auto [d,id]=frontier.top();frontier.pop();
            if(d>distance[id]+1e-8) continue;
            r.globalSearchAllowed[id]=true;
            const auto& point=r.network.points[id];
            if(d>=preview || (nextLeg.finalLeg && missionPointIsTarget(nextLeg,point.pose))) {
                // Lane-level corridor membership is insufficient after the
                // last legal merge window. A free frontier on a dead-end
                // lane must not sacrifice the following mandatory gateway.
                if(std::isfinite(r.handoffCost[id])) goals[id]=true;
                continue;
            }
            bool forward=false,atNextGateway=false;
            const auto visit=[&](std::int64_t to,double step) {
                if(!eligible(to)) return;
                const double value=d+std::max(0.,step);
                if(value+1e-8<distance[to]) {distance[to]=value;frontier.push({value,to});}
            };
            for(const auto& edge:r.forward[id]) {
                if(edge.lateral) continue;
                if(edge.to>=0 && nextLeg.gatewayLanes.count(keyOf(r.network.points[edge.to].pose)))
                    atNextGateway=true;
                if(!eligible(edge.to)) continue;
                forward=true;visit(edge.to,edge.distance);
            }
            visit(point.leftPoint,0.);visit(point.rightPoint,0.);
            // A shorter corridor may end at the next junction before the
            // requested preview length. Its physical entry is the limit.
            if(!forward && atNextGateway && d>1. &&
               std::isfinite(r.handoffCost[id])) goals[id]=true;
        }
        if(std::any_of(goals.begin(),goals.end(),[](bool value){return value;})) {
            r.globalSearchGoal=std::move(goals);
            r.globalExitPreviewActive=true;
        }
    }
}
std::int64_t nearest(const CorridorRuntime& r,const WorldState& world,bool tracking=false) {
    std::int64_t result=-1;double score=inf;
    const int gx=static_cast<int>(std::floor(world.ego.x/10.));
    const int gy=static_cast<int>(std::floor(world.ego.y/10.));
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
        auto cell=r.spatial.find({gx+dx,gy+dy});if(cell==r.spatial.end()) continue;
        for(auto id:cell->second) {
            if(!(tracking?r.trackingAllowed[id]:r.allowed[id])) continue;
            const auto& p=r.network.points[id].pose;
            const double d=std::hypot(p.x-world.ego.x,p.y-world.ego.y);
            const double yaw=std::abs(normalizeAngle(p.yaw-world.ego.heading));
            if(d>8. || yaw>1.2) continue;
            const bool same=keyOf(p)==LaneKey{world.egoRoadId,world.egoSectionIndex,world.egoLaneId};
            const double value=d+2.*yaw+(same?0.:2.);
            if(value<score) {score=value;result=id;}
        }
    }
    return result;
}
const CorridorEdge* forwardChoice(const CorridorRuntime& r,std::int64_t id,bool requireGateway=true,
                                  bool allowAnyContinuation=false) {
    // Follow the cached corridor-wide route exactly whenever its next edge is
    // longitudinal.  A generated merge/split is a real longitudinal road
    // connection, not an optional detour, so it must not lose to a straight
    // branch merely because it carries a transition id.
    if(requireGateway && id>=0 &&
       id<static_cast<std::int64_t>(r.gatewayNext.size())) {
        const auto selected=r.gatewayNext[id];
        if(selected>=0) {
            for(const auto& e:r.forward[id]) {
                if(e.to==selected && !e.lateral && r.allowed[e.to]) return &e;
            }
        }
    }
    const CorridorEdge* best=nullptr;double score=inf;
    for(const auto& e:r.forward[id]) {
        if(e.lateral || !r.allowed[e.to] || (requireGateway&&!std::isfinite(r.gatewayCost[e.to]))) continue;
        const double value=r.gatewayCost[e.to]+(e.transition>=0?1e-6:0.);
        if(!best || value<score) {best=&e;score=value;}
    }
    // The current target lane can physically continue through a precomputed
    // merge into the next mission leg even though that lane is intentionally
    // outside the current leg's allowed set.  If the ordinary choice is
    // exhausted, use the all-lane handoff search instead of returning an
    // empty path.  Lateral edges remain committed maneuvers, never implicit
    // tracking jumps.
    if(!best && requireGateway &&
       (!r.allowed[id] || r.gatewayCost[id]<=1e-8) &&
       std::isfinite(r.handoffCost[id])) {
        for(const auto& e:r.forward[id]) {
            if(e.lateral || !r.trackingAllowed[e.to] ||
               !std::isfinite(r.handoffCost[e.to])) continue;
            const double value=(e.transition>=0?1e-6:0.)+r.handoffCost[e.to];
            if(!best || value<score) {best=&e;score=value;}
        }
    }
    // Last resort, tracking only: the precomputed network always knows where
    // this physical lane continues, so a mission-bookkeeping filter must never
    // be the reason the vehicle has no path at all.  Follow the plain forward
    // continuation of the lane the vehicle is physically in, ignoring the
    // corridor mask and the gateway/handoff costs.  A lane successor keeps the
    // driving direction, so this can only ever mean "keep going straight in
    // this lane" - never a jump into oncoming traffic, never an implicit lane
    // change.  Candidate planning must NOT use this: letting a candidate tail
    // run past the corridor end inflates the remaining distance and the
    // gateway alignment deadline then never fires.
    if(!best && allowAnyContinuation) {
        for(const auto& e:r.forward[id]) {
            if(e.lateral || !visible(r.network,e.to)) continue;
            const double value=(e.transition>=0?1e-6:0.);
            if(!best || value<score) {best=&e;score=value;}
        }
    }
    return best;
}
void append(std::vector<Waypoint>& out,Waypoint p) {
    if(!out.empty()) {
        // XODR one-sided derivative probes are micrometres apart. They are
        // diagnostic samples, not controller steps: dyaw/ds would explode.
        const double d=gap(out.back(),p);
        if(d<minimumTrackingSampleSpacing) return;
        p.s=out.back().s+d;
    }
    out.push_back(p);
}
void repairTrackingTangents(std::vector<Waypoint>& path) {
    if(path.size()<2) return;
    for(std::size_t i=0;i<path.size();++i) {
        const std::size_t a=i==0?0:i-1;
        const std::size_t b=i+1<path.size()?i+1:i;
        const double dx=path[b].x-path[a].x;
        const double dy=path[b].y-path[a].y;
        if(std::hypot(dx,dy)<minimumTrackingSampleSpacing) continue;
        const double geometricYaw=std::atan2(dy,dx);
        // Preserve genuine curves: only replace metadata yaw that disagrees
        // sharply with the direction of the surrounding physical points.
        if(std::abs(normalizeAngle(path[i].yaw-geometricYaw))>0.2)
            path[i].yaw=geometricYaw;
    }
}
std::vector<Waypoint> forwardPath(const CorridorRuntime& r,std::int64_t start,double horizon,double startS,
                                  bool allowAnyContinuation=false,int* firstTransition=nullptr) {
    std::vector<Waypoint> out;
    auto first=r.network.points[start].pose;first.s=startS;out.push_back(first);
    std::set<std::int64_t> seen;
    for(auto id=start;seen.insert(id).second && out.back().s-startS<horizon;) {
        // Keep the directed suffix to the required gateway. At the last
        // lane-change opportunity a straight edge may still be inside the
        // corridor yet already have infinite gateway cost. End the prefix
        // there; only explicit physical occupancy probes may continue it.
        const auto* e=forwardChoice(r,id);
        if(!e && allowAnyContinuation) e=forwardChoice(r,id,false,true);
        if(!e) break;
        if(e->transition>=0) {
            if(firstTransition && *firstTransition<0) *firstTransition=e->transition;
            const auto& t=r.network.transitions[e->transition];
            for(const auto& p:t.points) {
                Waypoint q=r.network.points[t.toPoint].pose;
                const auto containingId=containing(r,p.x,p.y,p.yaw);
                if(containingId>=0) q=r.network.points[containingId].pose;
                q.x=p.x;q.y=p.y;q.yaw=p.yaw;q.roadS=p.roadS;
                append(out,q);
            }
        } else append(out,r.network.points[e->to].pose);
        id=e->to;
    }
    repairTrackingTangents(out);
    return out;
}

bool committedTransitionPath(std::vector<Waypoint>& out,CorridorRuntime& r,
                             const WorldState& world,double horizon,
                             DecisionTraceData* trace) {
    if(r.committedTransition<0 || r.committedTransitionLeg!=r.activeLeg ||
       r.committedTransition>=static_cast<int>(r.network.transitions.size())) return false;
    const auto& transition=r.network.transitions[r.committedTransition];
    if(transition.points.size()<2) {r.committedTransition=-1;return false;}
    const double c=std::cos(world.ego.heading),s=std::sin(world.ego.heading);
    std::size_t closest=r.committedTransitionProgress;double best=inf;
    const std::size_t searchBegin=r.committedTransitionProgress>8
        ? r.committedTransitionProgress-8 : 0;
    for(std::size_t i=searchBegin;i<transition.points.size();++i) {
        const auto& p=transition.points[i];
        const double dx=p.x-world.ego.x,dy=p.y-world.ego.y;
        const double along=dx*c+dy*s;
        const double distance=std::hypot(dx,dy);
        const double yaw=std::abs(normalizeAngle(p.yaw-world.ego.heading));
        if(along < -2.5 || yaw>1.0) continue;
        const double score=distance+.25*std::abs(along)+yaw;
        if(score<best) {best=score;closest=i;}
    }
    if(!std::isfinite(best) || best>6.0) {
        const auto& start=transition.points.front();
        const double beforeStart=(start.x-world.ego.x)*std::cos(start.yaw)+
                                 (start.y-world.ego.y)*std::sin(start.yaw);
        // Long transitions may be committed well before their entry. Keep
        // them armed while the start remains ahead; endpoint distance must
        // never release a route that has not even been entered yet.
        if(!r.committedTransitionEntered && beforeStart>=-2.0) return false;
        const auto& end=transition.points.back();
        const double past=(world.ego.x-end.x)*std::cos(end.yaw)+
                          (world.ego.y-end.y)*std::sin(end.yaw);
        if(past>2.0 || std::hypot(world.ego.x-end.x,world.ego.y-end.y)>12.0)
            r.committedTransition=-1;
        return false;
    }
    r.committedTransitionEntered=true;
    r.committedTransitionProgress=std::max(r.committedTransitionProgress,closest);
    const std::size_t begin=closest>2?closest-2:0;
    out.clear();
    for(std::size_t i=begin;i<transition.points.size();++i) {
        const auto& p=transition.points[i];
        Waypoint q=r.network.points[transition.toPoint].pose;
        const auto containingId=containing(r,p.x,p.y,p.yaw);
        if(containingId>=0) q=r.network.points[containingId].pose;
        q.x=p.x;q.y=p.y;q.yaw=p.yaw;q.roadS=p.roadS;
        if(out.empty()) q.s=world.egoPathS-std::hypot(q.x-world.ego.x,q.y-world.ego.y);
        append(out,q);
    }
    auto tail=forwardPath(r,transition.toPoint,horizon,
                          out.empty()?world.egoPathS:out.back().s);
    for(const auto& p:tail) append(out,p);
    repairTrackingTangents(out);
    traceDecision(trace,"corridor_lattice","transition_commitment_retained",
                  "transition="+std::to_string(r.committedTransition)+
                  " nearest_sample="+std::to_string(closest));
    return out.size()>=2;
}
// Enumerate the whole forward corridor, including lateral targets before any
// legality filtering. Road IDs identify points, never truncate the horizon.
std::vector<std::pair<double,std::int64_t>> targets(const CorridorRuntime& r,std::int64_t start,
                                                  const std::array<double,3>& distances,double egoWidth) {
    using Item=std::pair<double,std::int64_t>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> q;
    std::map<std::int64_t,double> costs;costs[start]=0.;q.push({0.,start});
    std::map<std::pair<LaneKey,int>,Item> selected;
    while(!q.empty()) {
        const auto [d,id]=q.top();q.pop();if(d>costs[id]+1e-8 || d>distances[2]+1.) continue;
        const auto& p=r.network.points[id];
        for(int i=0;i<3;++i) if(std::abs(d-distances[i])<=.75 && p.width>=egoWidth) {
            auto key=std::make_pair(keyOf(p.pose),i);
            auto old=selected.find(key);
            if(old==selected.end() || std::abs(d-distances[i])<std::abs(old->second.first-distances[i])) selected[key]={d,id};
        }
        const auto visit=[&](std::int64_t to,double step) {
            if(!visible(r.network,to) || !r.allowed[to]) return;
            const double next=d+step;
            auto found=costs.find(to);
            if(found==costs.end() || next+1e-8<found->second) {costs[to]=next;q.push({next,to});}
        };
        for(const auto& e:r.forward[id]) if(!e.lateral) visit(e.to,e.distance);
        visit(p.leftPoint,0.);visit(p.rightPoint,0.);
    }
    std::vector<Item> out;for(const auto& [key,item]:selected) out.push_back(item);
    return out;
}
std::array<double,6> polynomial(double p,double v,double a,double end,double endV,double endA) {
    const double c0=p,c1=v,c2=a/2.;
    const double A=end-c0-c1-c2,B=endV-c1-2*c2,C=endA-2*c2;
    return {c0,c1,c2,10*A-4*B+C/2.,-15*A+7*B-C,6*A-3*B+C/2.};
}
std::array<double,3> jet(const std::array<double,6>& c,double u) {
    double p=c[5],v=5*c[5],a=20*c[5];
    for(int i=4;i>=0;--i) p=p*u+c[i];
    for(int i=4;i>=1;--i) v=v*u+i*c[i];
    for(int i=4;i>=2;--i) a=a*u+i*(i-1)*c[i];
    return {p,v,a};
}
bool curve(std::vector<Waypoint>& out,const Waypoint& start,double startK,
           const PrecomputedLanePoint& target,double* maximum,
           double tangentScale=1.,double headingLimit=1.4) {
    const auto& end=target.pose;const double length=gap(start,end);
    if(length<3. || std::abs(normalizeAngle(end.yaw-start.yaw))>headingLimit) return false;
    const double tangent=length*tangentScale;
    const auto x=polynomial(start.x,tangent*std::cos(start.yaw),-tangent*tangent*startK*std::sin(start.yaw),
                            end.x,tangent*std::cos(end.yaw),-tangent*tangent*target.curvature*std::sin(end.yaw));
    const auto y=polynomial(start.y,tangent*std::sin(start.yaw),tangent*tangent*startK*std::cos(start.yaw),
                            end.y,tangent*std::sin(end.yaw),tangent*tangent*target.curvature*std::cos(end.yaw));
    const int count=static_cast<int>(std::ceil(std::max(length,tangent)/.2));*maximum=0.;
    for(int i=0;i<=count;++i) {
        const auto a=jet(x,double(i)/count),b=jet(y,double(i)/count);
        const double speed=std::hypot(a[1],b[1]);if(speed<.1) return false;
        const double k=(a[1]*b[2]-b[1]*a[2])/(speed*speed*speed);
        *maximum=std::max(*maximum,std::abs(k));if(*maximum>maximumCurvature) return false;
        Waypoint p=i<count/2?start:end;p.x=a[0];p.y=b[0];p.yaw=std::atan2(b[1],a[1]);
        p.roadS=start.roadS+(end.roadS-start.roadS)*double(i)/count;
        if(!out.empty() && gap(out.back(),p)>.51) return false;
        append(out,p);
    }
    return out.size()>2;
}
std::int64_t containing(const CorridorRuntime& r,double x,double y,double yaw,
                        bool corridorOnly,const std::vector<bool>* surfaceOverride) {
    const int gx=static_cast<int>(std::floor(x/10.)),gy=static_cast<int>(std::floor(y/10.));
    std::int64_t result=-1;double best=inf;
    const double c=std::cos(yaw),s=std::sin(yaw);
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
        const auto cell=r.spatial.find({gx+dx,gy+dy});if(cell==r.spatial.end()) continue;
        for(auto id:cell->second) {
            if((corridorOnly && !(surfaceOverride ? (*surfaceOverride)[id] : r.surfaceAllowed[id])) ||
               (!corridorOnly && !visible(r.network,id))) continue;
            const auto& p=r.network.points[id];
            const double dx=x-p.pose.x,dy=y-p.pose.y,radius=p.width/2.+.6;
            if(dx*dx+dy*dy>radius*radius) continue;
            const auto [pc,ps]=r.tangent[id];
            if(c*pc+s*ps<.36235775) continue;
            const double a=dx*pc+dy*ps;
            const double b=-dx*ps+dy*pc;
            if(std::abs(a)>.55 || std::abs(b)>p.width/2.+.03) continue;
            const double score=std::abs(b)+.1*std::abs(a);
            if(score<best) {best=score;result=id;}
        }
    }
    return result;
}
bool boundaryAllowed(const PathGeneratorData& map,const Waypoint& a,const Waypoint& b) {
    if(a.roadId!=b.roadId || a.sectionIndex!=b.sectionIndex) return true;
    if(a.laneId==b.laneId) return true;
    if(a.laneId*b.laneId<=0) return false;
    const auto& section=map.roads.at(a.roadId).sections.at(a.sectionIndex);
    const int step=b.laneId>a.laneId?1:-1;
    for(int lane=a.laneId;lane!=b.laneId;lane+=step)
        if(!laneChangeAllowed(section,lane,lane+step,b.roadS)) return false;
    return true;
}
bool surfaceAndMarks(const CorridorRuntime& r,const PathGeneratorData& map,
                     std::vector<Waypoint>& path,const WorldState& world,std::string& reason,
                     bool corridorOnly=true,Waypoint* rejectedPoint=nullptr,
                     const std::vector<bool>* surfaceOverride=nullptr) {
    const auto ego=egoFootprint(world);
    const int bodySteps=std::max(1,int(std::ceil(ego.length/.5)));
    std::int64_t previous=-1;
    corridor_detail::SurfaceQuery surfaceQuery(r,corridorOnly,surfaceOverride);
    for(auto& p:path) {
        const double c=std::cos(p.yaw),s=std::sin(p.yaw);
        const auto id=surfaceQuery.containing(p.x,p.y,c,s);
        if(id<0) {
            if(rejectedPoint) *rejectedPoint=p;
            reason=corridorOnly?"center_outside_corridor":"center_outside_driving_surface";return false;
        }
        const auto& center=r.network.points[id].pose;
        if(previous>=0 && !boundaryAllowed(map,r.network.points[previous].pose,center)) {
            reason="solid_or_forbidden_lane_change";return false;
        }
        previous=id;
        p.roadId=center.roadId;p.sectionIndex=center.sectionIndex;p.laneId=center.laneId;p.roadS=center.roadS;
        for(int i=0;i<=bodySteps;++i) for(double side:{-ego.width/2.,0.,ego.width/2.}) {
            const double along=-ego.rearExtent()+ego.length*double(i)/bodySteps;
            const auto corner=surfaceQuery.containing(p.x+along*c-side*s,p.y+along*s+side*c,c,s);
            if(corner<0) {
                if(rejectedPoint) {
                    *rejectedPoint=p;rejectedPoint->x=p.x+along*c-side*s;
                    rejectedPoint->y=p.y+along*s+side*c;
                }
                reason=corridorOnly?"footprint_outside_corridor":"footprint_outside_driving_surface";return false;
            }
            if(!boundaryAllowed(map,center,r.network.points[corner].pose)) {
                reason="footprint_crosses_solid";return false;
            }
        }
    }
    return true;
}
double staticClearance(const std::vector<Waypoint>& path,const WorldState& world,
                       double margin=0.,int* objectId=nullptr,
                       const PathGeneratorData* map=nullptr,
                       bool reserveCrossTrafficMotion=true) {
    if(objectId) *objectId=-1;
    const auto ego=egoFootprint(world);
    for(const auto& p:path) {
        for(const auto& o:world.objects) {
            if(std::abs(o.raw.speed)>=.3) continue;
            const double headingDifference=std::abs(normalizeAngle(
                double(o.raw.heading)-p.yaw));
            const bool vehicleSized=o.raw.length>=2.5F &&
                o.raw.length<=7.5F && o.raw.width>=1.4F &&
                o.raw.width<=2.8F;
            const bool crossingOrientation=headingDifference>=0.6108652382 &&
                headingDifference<=2.5307274154;
            // A stopped cross-traffic vehicle can start between two 20 Hz
            // samples.  M14 reproduced a launch while Ego passed only 2.54 m
            // from its centre: the old 0.60 m allowance declared that path
            // clear by about ten centimetres.  Reserve 1.5 m only for
            // vehicle-sized, cross-oriented objects; cones and parallel
            // parked vehicles retain the ordinary narrow-gap clearance.
            const double startUncertainty=
                reserveCrossTrafficMotion&&vehicleSized&&crossingOrientation?1.50:0.;
            const auto box=vehicleFootprintAt(ego,{p.x,p.y,p.yaw},margin+startUncertainty);
            if(intersects(box,o.obb) && (!map || corridorObstacleAtPathLevel(*map,p,o))) {
                if(objectId) *objectId=static_cast<int>(o.raw.id);
                return p.s-path.front().s;
            }
        }
    }
    return inf;
}
bool collisionFree(const std::vector<Waypoint>& path,const WorldState& world,
                   const PlanningBehaviorConfig& config,const LaneChangeConfig& laneConfig,
                   const PathGeneratorData* map=nullptr) {
    // Check every stationary object, including the one that triggered this
    // search, over the complete manoeuvre. The moving prediction below only
    // contains moving objects and cannot replace this test.
    if(std::isfinite(staticClearance(path,world,laneConfig.latticeObstacleClearance,nullptr,map))) return false;
    WorldState moving=world;
    moving.egoSpeed=std::max({1.,std::abs(world.egoSpeed),laneConfig.maximumLaneChangeSpeed});
    moving.objects.erase(std::remove_if(moving.objects.begin(),moving.objects.end(),
        [](const auto& o){return std::abs(o.raw.speed)<.3;}),moving.objects.end());
    LocalPathData local;local.valid=true;local.points=path;
    PathSafetyResult safety;resetPathSafety(safety);
    checkCurrentCollisions(safety,world.egoObb,world.objects,-1,&world,map);
    checkPredictedCollisions(safety,moving,local,config.collisionPredictionHorizon,config.collisionPredictionStep,true,map);
    return !safety.collision&&!safety.predictedCollision;
}
bool stationaryPathClear(const std::vector<Waypoint>& path,
                         const WorldState& world,double margin,
                         std::size_t begin=0,int* objectId=nullptr,
                         const PathGeneratorData* map=nullptr,
                         Waypoint* collisionPoint=nullptr);
std::shared_ptr<CorridorReentryScope> makeReentryScope(
    const CorridorRuntime&,const PathGeneratorData&,const WorldState&,bool,bool);
bool smoothSourceJoins(std::vector<Waypoint>& path,const CorridorRuntime& r,
                       const PathGeneratorData& map,const WorldState& world,
                       std::string* failure=nullptr,bool corridorOnly=true,
                       double obstacleClearance=-1.,Waypoint* failurePoint=nullptr) {
    std::shared_ptr<CorridorReentryScope> cornerSurface;
    for(std::size_t i=1;i<path.size();++i) {
        const double distance=gap(path[i-1],path[i]);
        const double yawDelta=normalizeAngle(path[i].yaw-path[i-1].yaw);
        const double segmentCurvature=distance>1e-6?yawDelta/distance:0.;
        bool sustainedMapCurve=false;
        if(i+1<path.size()) {
            const double nextDistance=gap(path[i],path[i+1]);
            if(nextDistance>minimumTrackingSampleSpacing) {
                const double nextCurvature=normalizeAngle(
                    path[i+1].yaw-path[i].yaw)/nextDistance;
                sustainedMapCurve=segmentCurvature*nextCurvature>0. &&
                    std::abs(segmentCurvature-nextCurvature)<0.04;
            }
        }
        if(!sustainedMapCurve && i>=2) {
            const double previousDistance=gap(path[i-2],path[i-1]);
            if(previousDistance>minimumTrackingSampleSpacing) {
                const double previousCurvature=normalizeAngle(
                    path[i-1].yaw-path[i-2].yaw)/previousDistance;
                sustainedMapCurve=segmentCurvature*previousCurvature>0. &&
                    std::abs(segmentCurvature-previousCurvature)<0.04;
            }
        }
        // Slowing down reduces lateral acceleration, not the steering angle
        // required by a geometric radius. Repair over-limit map arcs too,
        // before entering them, with a wider C2 join on the legal surface.
        // Small derivative noise retains the existing isolated-join rule.
        const bool infeasibleMapCurve = sustainedMapCurve &&
            std::abs(segmentCurvature)>maximumCurvature+1e-3;
        if(!infeasibleMapCurve &&
           (std::abs(yawDelta)<=.19*distance+1e-6 || sustainedMapCurve)) continue;
        bool fixed=false;
        std::ostringstream repairFailures;
        // A short source seam can border a tight mapped arc even when its
        // adjacent derivatives do not classify it as a sustained curve.
        // Search the same radius/tangent family for both; the footprint,
        // road marks, curvature and obstacle checks still decide feasibility.
        for(double radius:{3.,5.,8.,12.,16.,24.}) {
            std::size_t first=i-1,last=i;
            while(first>0 && path[i].s-path[first].s<radius) --first;
            while(last+1<path.size() && path[last].s-path[i-1].s<radius) ++last;
            // At a freshly rebuilt path the first point can itself be the
            // section-boundary source.  It is still a valid C2 curve start;
            // requiring an additional point behind it made a zero-gap XODR
            // heading knot impossible to smooth and emptied the path.
            if((first==i-1 && first!=0) || last==i) continue;
            const auto a=containing(r,path[first].x,path[first].y,path[first].yaw,corridorOnly);
            const auto b=containing(r,path[last].x,path[last].y,path[last].yaw,corridorOnly);
            if(a<0 || b<0) continue;
            auto end=r.network.points[b];end.pose=path[last];
            std::vector<Waypoint> replacement;double maxK{};std::string reason;
            bool feasible=false;
            // The ordinary lane-change primitive deliberately limits heading
            // change to 80 degrees. A mapped right-angle corner needs its own
            // bounded tangent family, with the SAME curvature/body checks.
            for(double tangentScale:{1.,1.2,1.4,1.6}) {
                replacement.clear();
                reason.clear();
                if(!curve(replacement,path[first],r.network.points[a].curvature,end,&maxK,
                          tangentScale,1.75)) continue;
                Waypoint rejected;
                bool onSurface=surfaceAndMarks(r,map,replacement,world,reason,corridorOnly,&rejected);
                if(!onSurface && infeasibleMapCurve && corridorOnly &&
                   reason=="footprint_outside_corridor") {
                    // Keep the centre trajectory on the official corridor;
                    // only the rotating body's envelope can borrow reviewed,
                    // same-direction, same-level adjacent driving pavement.
                    // No new graph edge, goal, centre-line or solid crossing.
                    auto originalMetadata=replacement;
                    const bool centresInside=std::all_of(originalMetadata.begin(),originalMetadata.end(),
                        [&](Waypoint& p) {
                            const auto id=containing(r,p.x,p.y,p.yaw,true);
                            if(id<0)return false;
                            const auto& centre=r.network.points[id].pose;
                            p.roadId=centre.roadId;p.sectionIndex=centre.sectionIndex;
                            p.laneId=centre.laneId;p.roadS=centre.roadS;return true;
                        });
                    if(centresInside) {
                        if(!cornerSurface)cornerSurface=makeReentryScope(r,map,world,true,false);
                        onSurface=surfaceAndMarks(r,map,replacement,world,reason,true,&rejected,&cornerSurface->surface);
                        if(onSurface)replacement=std::move(originalMetadata);
                    }
                }
                if(!onSurface && failure) {
                    repairFailures<<" [radius="<<radius<<" tangent="<<tangentScale
                        <<" rejected="<<reason<<" xy="<<rejected.x<<':'<<rejected.y<<']';
                }
                if(onSurface &&
                   (obstacleClearance<0. || stationaryPathClear(
                       replacement,world,obstacleClearance,0,nullptr,&map))) {
                    feasible=true;break;
                }
            }
            if(!feasible) {
                if(failure) repairFailures<<" [radius="<<radius<<" max_k="<<maxK
                    <<" reason="<<(reason.empty()?"curve":reason)<<']';
                continue;
            }
            // Global materialization must try another joining radius if the
            // first geometrically valid smoothing cuts across an obstacle.
            // Other callers keep their established validation unchanged.
            if(obstacleClearance>=0. && !stationaryPathClear(
                   replacement,world,obstacleClearance,0,nullptr,&map)) continue;
            std::vector<Waypoint> joined(path.begin(),path.begin()+first);
            for(const auto& p:replacement) append(joined,p);
            for(auto j=last+1;j<path.size();++j) append(joined,path[j]);
            path=std::move(joined);i=first+replacement.size()-1;fixed=true;break;
        }
        if(!fixed) {
            // Do not turn an existing low-speed map continuation into an
            // empty path solely because this bounded repair family failed.
            // This is NOT a curvature-feasibility certificate: retain the
            // original curvature and its speed cap, and expose the failure.
            if(infeasibleMapCurve) {
                if(failure)*failure="mapped_curve_repair_unavailable curvature="+
                    std::to_string(std::abs(segmentCurvature))+repairFailures.str();
                // One unsuccessful attempt per contiguous over-limit arc,
                // not the same radius family at every 0.5 m source sample.
                while(i+1<path.size()) {
                    const double d=gap(path[i],path[i+1]);
                    const double k=d>minimumTrackingSampleSpacing
                        ?normalizeAngle(path[i+1].yaw-path[i].yaw)/d:0.;
                    if(k*segmentCurvature<=0. || std::abs(k)<=maximumCurvature+1e-3)break;
                    ++i;
                }
                continue;
            }
            if(failurePoint) *failurePoint=path[i];
            if(failure) {
                std::ostringstream detail;
                detail<<"index="<<i<<" distance="<<distance
                      <<" yaw_delta="
                      <<std::abs(normalizeAngle(path[i].yaw-path[i-1].yaw))
                      <<" from="<<path[i-1].roadId<<':'
                      <<path[i-1].sectionIndex<<':'<<path[i-1].laneId
                      <<" to="<<path[i].roadId<<':'
                      <<path[i].sectionIndex<<':'<<path[i].laneId
                      <<repairFailures.str();
                *failure=detail.str();
            }
            return false;
        }
    }
    return true;
}

bool isStationaryRouteObstacle(const WorldObjectState& object) {
    // Route occupancy must not silently omit small received actors. Traffic
    // and pedestrian yielding retain their separate, higher-priority logic.
    return std::isfinite(object.raw.speed) && std::abs(object.raw.speed)<.3 &&
           object.raw.length>0. && object.raw.width>0.;
}

std::uint64_t stationarySceneSignature(const WorldState& world) {
    // Slot order is not object identity. A 30-slot update can replace one
    // obstacle with another without changing the count; moved/stopped actors
    // also invalidate a previous failed search. Quantization ignores noise.
    std::vector<std::array<std::int64_t,7>> objects;
    for (const auto& object : world.objects) {
        if (!isStationaryRouteObstacle(object)) continue;
        const auto& o = object.raw;
        objects.push_back({o.id, std::llround(o.x*4.), std::llround(o.y*4.),
            std::llround(o.z*4.), std::llround(o.heading*16.),
            std::llround(o.length*10.), std::llround(o.width*10.)});
    }
    std::sort(objects.begin(), objects.end());
    std::uint64_t signature = 14695981039346656037ULL;
    for (const auto& object : objects) for (const auto value : object) {
        signature ^= static_cast<std::uint64_t>(value);
        signature *= 1099511628211ULL;
    }
    return signature;
}

bool stationaryPathClear(const std::vector<Waypoint>& path,
                         const WorldState& world,double margin,
                         std::size_t begin,int* objectId,
                         const PathGeneratorData* map,Waypoint* collisionPoint) {
    if(objectId) *objectId=-1;
    if(path.empty()) return false;
    begin=std::min(begin,path.size()-1);
    const auto ego=egoFootprint(world);
    for(std::size_t i=begin;i<path.size();++i) {
        const auto& p=path[i];
        const auto box=vehicleFootprintAt(ego,{p.x,p.y,p.yaw},margin);
        for(const auto& object:world.objects) {
            if(!isStationaryRouteObstacle(object)) continue;
            if(!intersects(box,object.obb)) continue;
            if(map && !corridorObstacleAtPathLevel(*map,p,object)) continue;
            if(objectId) *objectId=static_cast<int>(object.raw.id);
            if(collisionPoint) *collisionPoint=p;
            return false;
        }
    }
    return true;
}

std::vector<Waypoint> corridorEdgePath(const CorridorRuntime& r,
                                       std::int64_t from,
                                       const CorridorEdge& edge) {
    std::vector<Waypoint> path;
    if(from<0 || edge.to<0 ||
       from>=static_cast<std::int64_t>(r.network.points.size()) ||
       edge.to>=static_cast<std::int64_t>(r.network.points.size())) return path;
    append(path,r.network.points[from].pose);
    if(edge.lateral) {
        double maximum{};
        path.clear();
        curve(path,r.network.points[from].pose,
              r.network.points[from].curvature,
              r.network.points[edge.to],&maximum);
        return path;
    }
    if(edge.transition>=0 &&
       edge.transition<static_cast<int>(r.network.transitions.size())) {
        const auto& transition=r.network.transitions[edge.transition];
        for(const auto& point:transition.points) {
            Waypoint sample=r.network.points[edge.to].pose;
            const auto containingId=containing(
                r,point.x,point.y,point.yaw);
            if(containingId>=0)
                sample=r.network.points[containingId].pose;
            sample.x=point.x;sample.y=point.y;sample.yaw=point.yaw;
            sample.roadS=point.roadS;
            append(path,sample);
        }
    }
    append(path,r.network.points[edge.to].pose);
    return path;
}

// Search the sampled XODR graph with every stationary obstacle already
// removed, then materialize every selected lateral edge as a checked quintic.
// The terminal nodes are the points physically nearest the checkpoint, not
// merely the first sample of a lane whose ID happens to be a gateway lane.
bool materializeObstacleAwareGlobalRoute(
    std::vector<Waypoint>& output,CorridorRuntime& r,
    const PathGeneratorData& map,const WorldState& world,
    const MissionCorridorData& mission,std::int64_t start,
    DecisionTraceData* trace) {
    output.clear();
    const auto* leg=activeMissionLeg(mission);
    if(!leg || start<0 || !r.allowed[start] ||
       leg->toCheckpoint>=mission.checkpoints.size()) return false;

    const auto& checkpoint=mission.checkpoints[leg->toCheckpoint].checkpoint;
    std::vector<bool> goal(r.network.points.size(),false);
    std::size_t goalCount=0;
    for(const auto& key:leg->targetLanes) {
        const auto lane=r.network.lanes.find(key);
        if(lane==r.network.lanes.end() || lane->second.pointIds.empty()) continue;
        std::int64_t closest=-1;double best=inf;
        for(const auto id:lane->second.pointIds) {
            if(id<0 || !r.allowed[id]) continue;
            const auto& p=r.network.points[id].pose;
            const double distance=leg->topologyAuthority
                ? std::abs(p.roadS-leg->targetRoadS.at(key))
                : std::hypot(p.x-checkpoint.x,p.y-checkpoint.y);
            if(distance<best) {best=distance;closest=id;}
        }
        if(closest>=0 && !goal[closest]) {goal[closest]=true;++goalCount;}
    }
    if(goalCount==0) return false;

    // Three states per XODR centre point: near the right edge, centre, and
    // near the left edge.  A centre-line-only graph cannot represent the
    // staggered narrow passage in M11 even though the vehicle physically fits
    // between the obstacles.  Offsets never exceed the lane width minus the
    // ego half-width and edge clearance.
    constexpr int offsetStateCount=3;
    const auto stateId=[](std::int64_t point,int offset) {
        return point*offsetStateCount+offset;
    };
    const auto statePoint=[](std::int64_t state) {
        return state/offsetStateCount;
    };
    const auto stateOffset=[](std::int64_t state) {
        return int(state%offsetStateCount);
    };
    const auto offsetPose=[&](std::int64_t point,int offset) {
        Waypoint pose=r.network.points[point].pose;
        const double room=std::max(0.,std::min(
            r.laneConfig.maximumLateralOffset,
            r.network.points[point].width/2.-egoFootprint(world).width/2.-
                r.laneConfig.minimumLaneEdgeClearance));
        const double lateral=double(offset-1)*room;
        pose.x-=lateral*std::sin(pose.yaw);
        pose.y+=lateral*std::cos(pose.yaw);
        return pose;
    };
    const auto offsetCurve=[&](std::vector<Waypoint>& physical,
                               std::int64_t from,int fromOffset,
                               std::int64_t to,int toOffset) {
        physical.clear();
        auto target=r.network.points[to];
        target.pose=offsetPose(to,toOffset);
        double maximum{};
        return curve(physical,offsetPose(from,fromOffset),
                     r.network.points[from].curvature,target,&maximum);
    };
    struct Parent {
        std::int64_t fromState{-1};
        CorridorEdge edge;
        int targetOffset{1};
        bool generatedCurve{};
        bool valid{};
    };
    using Item=std::pair<double,std::int64_t>;
    const std::size_t stateCount=r.network.points.size()*offsetStateCount;
    std::vector<double> distance(stateCount,inf);
    std::vector<Parent> parent(stateCount);
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    const auto startState=stateId(start,1);
    distance[startState]=0.;queue.push({0.,startState});
    std::int64_t reachedState=-1;
    std::size_t expanded=0,rejectedStatic=0,rejectedGeometry=0;
    while(!queue.empty()) {
        const auto [cost,currentState]=queue.top();queue.pop();
        if(cost>distance[currentState]+1e-8) continue;
        const auto id=statePoint(currentState);
        const int currentOffset=stateOffset(currentState);
        ++expanded;
        if(goal[id] && currentOffset==1) {reachedState=currentState;break;}
        const auto relax=[&](const CorridorEdge& edge,int targetOffset,
                             bool generatedCurve) {
            if(edge.to<0 || !r.allowed[edge.to]) return;
            std::vector<Waypoint> physical;
            double step=std::max(.0001,edge.distance);
            if(generatedCurve) {
                if(!offsetCurve(physical,id,currentOffset,
                                edge.to,targetOffset)) {
                    ++rejectedGeometry;return;
                }
                std::string reason;
                if(!surfaceAndMarks(r,map,physical,world,reason)) {
                    ++rejectedGeometry;return;
                }
                step=physical.back().s-physical.front().s;
            } else if(edge.transition>=0) {
                physical=corridorEdgePath(r,id,edge);
                if(physical.size()<2) {++rejectedGeometry;return;}
                step=physical.back().s-physical.front().s;
            } else {
                physical={offsetPose(edge.to,targetOffset)};
            }
            if(!stationaryPathClear(
                    physical,world,r.laneConfig.latticeObstacleClearance,0,nullptr,&map)) {
                ++rejectedStatic;return;
            }
            const bool changesLane=!(keyOf(r.network.points[id].pose)==
                keyOf(r.network.points[edge.to].pose));
            const double candidate=cost+step+
                (changesLane?4.:(generatedCurve?.35:0.));
            const auto targetState=stateId(edge.to,targetOffset);
            if(candidate+1e-8>=distance[targetState]) return;
            distance[targetState]=candidate;
            parent[targetState]={currentState,edge,targetOffset,
                                 generatedCurve,true};
            queue.push({candidate,targetState});
        };
        for(const auto& edge:r.forward[id]) {
            if(edge.lateral) {
                for(int targetOffset=0;targetOffset<offsetStateCount;
                    ++targetOffset)
                    relax(edge,targetOffset,true);
            } else if(edge.transition>=0) {
                // Generated merge/split curves were validated on their
                // centreline.  Settle to centre before entering them.
                if(currentOffset==1) relax(edge,1,false);
            } else {
                relax(edge,currentOffset,false);
            }
        }
        // Small within-lane shifts let the global route use the free side of
        // a lane between staggered objects.  They are full C2 curves, never a
        // lateral teleport.  Two longitudinal lengths cover tight and smooth
        // passages while keeping the graph bounded.
        const std::array<double,2> offsetShiftLengths{
            std::max(6.,r.laneConfig.latticeShortDistance*2./3.),
            r.laneConfig.latticeShortDistance};
        if(currentOffset>=0) for(double desired:offsetShiftLengths) {
            auto target=id;double travelled=0.;std::set<std::int64_t> seen;
            while(travelled<desired && seen.insert(target).second) {
                const CorridorEdge* next=nullptr;
                for(const auto& edge:r.forward[target]) {
                    if(edge.lateral || edge.transition>=0 ||
                       !r.allowed[edge.to] ||
                       !(keyOf(r.network.points[edge.to].pose)==
                           keyOf(r.network.points[id].pose))) continue;
                    if(next) {next=nullptr;break;}
                    next=&edge;
                }
                if(!next) {target=-1;break;}
                travelled+=next->distance;target=next->to;
            }
            if(target<0 || travelled<desired-.75) continue;
            for(int targetOffset=0;targetOffset<offsetStateCount;
                ++targetOffset) {
                if(targetOffset==currentOffset) continue;
                relax({target,travelled,-1,true},targetOffset,true);
            }
        }
    }
    if(reachedState<0) {
        std::ostringstream detail;
        detail<<"start="<<start<<" expanded="<<expanded
              <<" goals="<<goalCount
              <<" static_rejections="<<rejectedStatic
              <<" geometry_rejections="<<rejectedGeometry;
        traceDecision(trace,"corridor_global_route","search_exhausted",
                      detail.str());
        return false;
    }

    std::vector<Parent> reversed;
    for(auto current=reachedState;current!=startState;) {
        if(current<0 || !parent[current].valid) return false;
        reversed.push_back(parent[current]);
        current=parent[current].fromState;
    }
    std::reverse(reversed.begin(),reversed.end());
    Waypoint first=offsetPose(start,1);first.s=0.;
    append(output,first);
    for(const auto& step:reversed) {
        const auto from=statePoint(step.fromState);
        const int fromOffset=stateOffset(step.fromState);
        const int targetOffset=step.targetOffset;
        std::vector<Waypoint> physical;
        if(step.generatedCurve) {
            if(!offsetCurve(physical,from,fromOffset,
                            step.edge.to,targetOffset)) return false;
            std::string reason;
            if(!surfaceAndMarks(r,map,physical,world,reason)) return false;
        } else if(step.edge.transition>=0) {
            physical=corridorEdgePath(r,from,step.edge);
        } else {
            physical={offsetPose(from,fromOffset),
                      offsetPose(step.edge.to,targetOffset)};
        }
        if(physical.empty()) return false;
        for(const auto& point:physical) append(output,point);
    }
    repairTrackingTangents(output);
    std::string failure;
    if(output.size()<2 ||
       !smoothSourceJoins(output,r,map,world,&failure)) {
        traceDecision(trace,"corridor_global_route","materialization_failed",
                      failure);
        output.clear();return false;
    }
    if(!stationaryPathClear(
            output,world,r.laneConfig.latticeObstacleClearance,0,nullptr,&map)) {
        traceDecision(trace,"corridor_global_route","materialization_failed",
                      "assembled_path_intersects_stationary_object");
        output.clear();return false;
    }
    std::ostringstream detail;
    detail<<"revision="<<(r.routeRevision+1)
          <<" start="<<r.network.points[start].pose.roadId<<':'
          <<r.network.points[start].pose.sectionIndex<<':'
          <<r.network.points[start].pose.laneId
          <<" goal="<<r.network.points[statePoint(reachedState)].pose.roadId<<':'
          <<r.network.points[statePoint(reachedState)].pose.sectionIndex<<':'
          <<r.network.points[statePoint(reachedState)].pose.laneId
          <<" points="<<output.size()<<" length="
          <<(output.back().s-output.front().s)
          <<" expanded="<<expanded
          <<" static_rejections="<<rejectedStatic
          <<" geometry_rejections="<<rejectedGeometry;
    traceDecision(trace,"corridor_global_route","materialized",detail.str());
    return true;
}

// Real-time global route: search the already sampled XODR network after
// removing poses occupied by stationary obstacles.  The terminal condition
// is any directed driving-lane sample on the required outgoing road.  The
// competition checkpoints are approximate road hints, not points (or lane
// centres) that Ego must touch.
//
// The old exhaustive implementation generated and swept every possible
// quintic while expanding the graph.  That blocked the 20 Hz command thread
// for seconds.  Here graph expansion checks only the three sampled lateral
// poses (right/centre/left).  Quintics are generated for the selected route
// only; a bad selected edge is blacklisted and the cheap A* is retried.  This
// keeps whole-corridor obstacle awareness without moving expensive work into
// every control frame.
bool materializePrecomputedGatewayRouteStage(
    std::vector<Waypoint>& output,CorridorRuntime& r,
    const PathGeneratorData& map,const WorldState& world,
    const MissionCorridorData& mission,std::int64_t start,
    DecisionTraceData* trace,bool allowOffsets,bool queueApproach=false,
    bool expandedOffsets=false,double clearance=-1.) {
    output.clear();
    if(clearance<0.) clearance=r.laneConfig.latticeObstacleClearance;
    const auto* leg=activeMissionLeg(mission);
    if(!leg || start<0 || !r.allowed[start] ||
       leg->toCheckpoint>=mission.checkpoints.size()) return false;

    const auto& goal=r.globalSearchGoal;
    const auto& searchAllowed=r.globalSearchAllowed;
    const auto goalCount=std::count(goal.begin(),goal.end(),true);
    if(goalCount==0) return false;

    const int offsetStateCount=expandedOffsets?13:3;
    const auto body=egoFootprint(world);
    // Allocate search labels only for this corridor, not the entire map.
    // Sorted point IDs retain the existing A* tie-breaking order.
    std::vector<std::int64_t> activePoints;
    std::vector<std::int64_t> compact(r.network.points.size(),-1);
    for(std::size_t i=0;i<searchAllowed.size();++i) if(searchAllowed[i]) {
        compact[i]=activePoints.size();activePoints.push_back(i);
    }
    if(compact[start]<0) return false;
    const char* stage=expandedOffsets?"expanded_offsets":
        allowOffsets?"offset_fallback":"lane_centers";
    const double extent=std::max(0.,r.laneConfig.maximumLateralOffset);
    std::map<LaneKey,std::array<double,2>> gapOffsets;
    if(expandedOffsets) {
        std::map<LaneKey,std::vector<std::int64_t>> lanePoints;
        for(const auto id:activePoints) lanePoints[keyOf(r.network.points[id].pose)].push_back(id);
        for(const auto& [key,ids]:lanePoints) {
            std::vector<std::pair<double,double>> blockedIntervals;
            for(const auto& object:world.objects) {
                if(!isStationaryRouteObstacle(object)) continue;
                double best=inf;const Waypoint* near=nullptr;
                for(const auto id:ids) {
                    const auto& pose=r.network.points[id].pose;
                    const double d=std::hypot(pose.x-object.raw.x,pose.y-object.raw.y);
                    if(d<best) {best=d;near=&pose;}
                }
                if(!near || best>extent+body.length+object.raw.length ||
                   !corridorObstacleAtPathLevel(map,*near,object)) continue;
                const double c=std::cos(near->yaw),sn=std::sin(near->yaw);
                double lo=inf,hi=-inf;
                for(const auto& corner:object.obb.corners) {
                    const double lateral=-(corner.x-near->x)*sn+(corner.y-near->y)*c;
                    lo=std::min(lo,lateral);hi=std::max(hi,lateral);
                }
                blockedIntervals.push_back({lo-body.width/2.-clearance,
                                            hi+body.width/2.+clearance});
            }
            std::sort(blockedIntervals.begin(),blockedIntervals.end());
            std::array<double,2> candidates{0.,0.},widths{0.,0.};
            const auto consider=[&](double lo,double hi) {
                for(int side=0;side<2;++side) {
                    const double l=std::max(lo,side==0?-extent:0.);
                    const double h=std::min(hi,side==0?0.:extent);
                    if(h-l>widths[side]) {widths[side]=h-l;candidates[side]=(h+l)/2.;}
                }
            };
            double cursor=-extent;
            for(const auto& interval:blockedIntervals) {
                consider(cursor,std::min(extent,interval.first));
                cursor=std::max(cursor,interval.second);
            }
            consider(cursor,extent);gapOffsets[key]=candidates;
        }
    }
    const auto lateralOffset=[&](std::int64_t point,int offset) {
        const double room=std::max(0.,std::min(extent,
            r.network.points[point].width/2.-body.width/2.-
                r.laneConfig.minimumLaneEdgeClearance));
        if(offset<3) return double(offset-1)*room;
        if(offset>=11) return gapOffsets.at(keyOf(r.network.points[point].pose))[offset-11];
        return (offset%2? -1.:1.)*extent*(1.+(offset-3)/2)/4.;
    };
    const auto offsetAllowed=[&](std::int64_t point,int offset) {
        if(offset<3) return true;
        const auto& p=r.network.points[point];
        const double lateral=lateralOffset(point,offset);
        if(std::abs(lateral)<1e-5) return false;
        for(int prior=0;prior<offset;++prior)
            if(std::abs(lateral-lateralOffset(point,prior))<1e-5) return false;
        const double room=std::max(0.,p.width/2.-body.width/2.-r.laneConfig.minimumLaneEdgeClearance);
        if(std::abs(lateral)<=room+1e-6) return true;
        const auto adjacent=lateral<0.?p.rightPoint:p.leftPoint;
        return (lateral<0.?p.rightChangeAllowed:p.leftChangeAllowed) &&
            adjacent>=0 && searchAllowed[adjacent] && visible(r.network,adjacent) &&
            std::abs(normalizeAngle(p.pose.yaw-r.network.points[adjacent].pose.yaw))<.5;
    };
    // A narrow passage can start inside a merge/taper. The original in-lane
    // endpoints stop at lane IDs and cannot express an offset beginning
    // before that seam. Only the expanded tier follows the unique declared
    // longitudinal successor across it; every resulting curve is still swept.
    std::vector<std::array<CorridorEdge,4>> expandedShifts;
    if(expandedOffsets) {
        expandedShifts.resize(activePoints.size());
        const std::array<double,4> lengths{6.,8.,r.laneConfig.latticeShortDistance,
                                                r.laneConfig.latticeMediumDistance};
        for(std::size_t i=0;i<activePoints.size();++i) {
            const auto origin=activePoints[i];
            auto cursor=origin;double travelled=0.;std::size_t lengthIndex=0;
            for(int steps=0;steps<200 && lengthIndex<lengths.size();++steps) {
                const CorridorEdge* next=nullptr;
                for(const auto& edge:r.forward[cursor]) {
                    if(edge.lateral || edge.to<0 || !searchAllowed[edge.to]) continue;
                    if(next) {next=nullptr;break;}
                    next=&edge;
                }
                if(!next || next->to==cursor) break;
                cursor=next->to;travelled+=next->distance;
                if(std::abs(normalizeAngle(r.network.points[cursor].pose.yaw-
                                          r.network.points[origin].pose.yaw))>.5) break;
                while(lengthIndex<lengths.size() && travelled>=lengths[lengthIndex])
                    expandedShifts[i][lengthIndex++]={cursor,travelled,-1,true};
            }
        }
    }
    // With a farther return frontier, reaching a neighbouring free lane is
    // not enough: the chosen route must also visit this leg's actual target.
    // Keep that path-dependent fact in A*, rather than rejecting the cheapest
    // assembled path and accidentally declaring every alternative blocked.
    const auto stateId=[&](std::int64_t point,int offset,bool targetVisited=false) {
        return compact[point]*offsetStateCount*2+(targetVisited?offsetStateCount:0)+offset;
    };
    const auto statePoint=[&](std::int64_t state) {
        return activePoints[state/(offsetStateCount*2)];
    };
    const auto stateOffset=[&](std::int64_t state) {
        return int(state%offsetStateCount);
    };
    const auto targetVisited=[&](std::int64_t state) {
        return (state/offsetStateCount)%2!=0;
    };
    const auto offsetPose=[&](std::int64_t point,int offset) {
        Waypoint pose=r.network.points[point].pose;
        const double lateral=lateralOffset(point,offset);
        pose.x-=lateral*std::sin(pose.yaw);
        pose.y+=lateral*std::cos(pose.yaw);
        return pose;
    };
    const auto offsetCurve=[&](std::vector<Waypoint>& physical,
                               std::int64_t from,int fromOffset,
                               std::int64_t to,int toOffset) {
        physical.clear();
        auto target=r.network.points[to];
        target.pose=offsetPose(to,toOffset);
        double maximum{};
        return curve(physical,offsetPose(from,fromOffset),
                     r.network.points[from].curvature,target,&maximum);
    };
    struct Parent {
        std::int64_t fromState{-1};
        CorridorEdge edge;
        int targetOffset{1};
        bool generatedCurve{};
        bool valid{};
    };
    using Item=std::pair<double,std::int64_t>;
    const std::size_t stateCount=
        activePoints.size()*offsetStateCount*2;
    std::vector<signed char> blocked(stateCount,-1);
    const auto stateBlocked=[&](std::int64_t state) {
        auto& cached=blocked[state];
        if(cached>=0) return cached!=0;
        const auto pose=offsetPose(statePoint(state),stateOffset(state));
        const auto box=vehicleFootprintAt(egoFootprint(world),
            {pose.x,pose.y,pose.yaw},clearance);
        cached=0;
        for(const auto& object:world.objects) {
            if(!isStationaryRouteObstacle(object)) continue;
            if(intersects(box,object.obb) && corridorObstacleAtPathLevel(map,pose,object)) {
                cached=1;break;
            }
        }
        return cached!=0;
    };
    const auto heuristic=[&](std::int64_t state) {
        // Queue endpoints are BEFORE the gateway. Its downstream distance
        // is not an admissible heuristic for these temporary goals.
        if(queueApproach) return 0.;
        const double value=r.gatewayCost[statePoint(state)];
        return std::isfinite(value)?value:0.;
    };
    const auto edgeKey=[&](const Parent& step) {
        return std::pair<std::int64_t,std::int64_t>{
            step.fromState,
            step.edge.to*offsetStateCount+step.targetOffset};
    };
    std::map<std::pair<std::int64_t,std::int64_t>,bool>
        generatedCollisionCache;
    const auto generatedEdgeClear=[&](std::int64_t fromState,
                                      std::int64_t toPoint,
                                      int toOffset) {
        const auto toState=stateId(toPoint,toOffset);
        const auto key=std::make_pair(fromState,toState);
        const auto cached=generatedCollisionCache.find(key);
        if(cached!=generatedCollisionCache.end()) return cached->second;
        std::vector<Waypoint> physical;
        if(!offsetCurve(physical,statePoint(fromState),stateOffset(fromState),
                        toPoint,toOffset)) {
            generatedCollisionCache[key]=false;return false;
        }
        // Cheap necessary (not sufficient) body-in-road test. Without it,
        // many nearly identical curves cutting an intersection corner can
        // each consume an entire A* retry and hide an existing safe route.
        // Probe actual quintic samples, never an approximate chord. The full
        // fine-grained body, markings and collision sweep still approves the
        // selected route below. Cache these probes for this immutable scene.
        const auto coarseBody=egoFootprint(world);
        for(const double fraction:{.2,.5,.8}) {
            const auto& pose=physical[std::min(physical.size()-1,
                std::size_t(fraction*double(physical.size()-1)))];
            const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
            bool inside=containing(r,pose.x,pose.y,pose.yaw)>=0;
            for(const double along:{-coarseBody.rearExtent(),coarseBody.frontExtent()})
                for(const double side:{-coarseBody.width/2.,coarseBody.width/2.})
                    if(inside && containing(r,pose.x+along*c-side*s,
                                              pose.y+along*s+side*c,pose.yaw)<0)
                        inside=false;
            if(!inside) {
                generatedCollisionCache[key]=false;return false;
            }
        }
        const auto a=offsetPose(statePoint(fromState),stateOffset(fromState));
        const auto b=offsetPose(toPoint,toOffset);
        const double length=gap(a,b),lengthSquared=length*length;
        const double k0=r.network.points[statePoint(fromState)].curvature;
        const double k1=r.network.points[toPoint].curvature;
        // A quintic lies in its six Bezier control points' convex hull.
        // Unlike a straight-chord shortcut, this bounds bowed lane changes
        // too. Inflate by the whole ego body before rejecting distant boxes.
        const auto bounds=[&](double p0,double p1,double d0,double d1,double a0,double a1) {
            const std::array<double,6> control{p0,p0+d0/5.,p0+2.*d0/5.+a0/20.,
                p1-2.*d1/5.+a1/20.,p1-d1/5.,p1};
            const auto extent=std::minmax_element(control.begin(),control.end());
            return std::make_pair(*extent.first,*extent.second);
        };
        const auto bx=bounds(a.x,b.x,length*std::cos(a.yaw),length*std::cos(b.yaw),
            -lengthSquared*k0*std::sin(a.yaw),-lengthSquared*k1*std::sin(b.yaw));
        const auto by=bounds(a.y,b.y,length*std::sin(a.yaw),length*std::sin(b.yaw),
            lengthSquared*k0*std::cos(a.yaw),lengthSquared*k1*std::cos(b.yaw));
        const auto body=egoFootprint(world);
        const double radius=std::hypot(std::max(body.frontExtent(),body.rearExtent()),body.width/2.)+
            std::sqrt(2.)*clearance;
        bool nearStationary=false;
        for(const auto& object:world.objects) {
            if(!isStationaryRouteObstacle(object)) continue;
            const double objectRadius=.5*std::hypot(object.raw.length,object.raw.width);
            if(object.raw.x+objectRadius>=bx.first-radius &&
               object.raw.x-objectRadius<=bx.second+radius &&
               object.raw.y+objectRadius>=by.first-radius &&
               object.raw.y-objectRadius<=by.second+radius) {
                nearStationary=true;break;
            }
        }
        if(!nearStationary) {
            generatedCollisionCache[key]=true;
            return true;
        }
        const bool clear=stationaryPathClear(
                physical,world,clearance,0,nullptr,&map);
        generatedCollisionCache[key]=clear;
        return clear;
    };

    std::set<std::pair<std::int64_t,std::int64_t>> rejectedEdges;
    std::int64_t reachedState=-1;
    std::size_t expandedTotal=0,rejectedStatic=0,rejectedSelected=0;
    // One physical lane edge can appear as nine offset combinations.  Keep
    // enough retries to reject that entire early taper and let A* choose the
    // next dashed/full-width window, without sweeping unrelated graph edges.
    constexpr int maximumSelectedRouteRetries=32;
    for(int attempt=0;attempt<maximumSelectedRouteRetries;++attempt) {
        std::vector<double> distance(stateCount,inf);
        std::vector<Parent> parent(stateCount);
        std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
        const auto startState=stateId(start,1,
            missionPointIsTarget(*leg,r.network.points[start].pose));
        distance[startState]=0.;
        queue.push({heuristic(startState),startState});
        reachedState=-1;
        while(!queue.empty()) {
            const auto [priority,currentState]=queue.top();queue.pop();
            if(priority>distance[currentState]+heuristic(currentState)+1e-8)
                continue;
            const auto id=statePoint(currentState);
            const int currentOffset=stateOffset(currentState);
            ++expandedTotal;
            // A gateway is a road/lane target, not permission to finish on a
            // temporary within-lane offset.  Ending the committed global path
            // at offset state 0/2 leaves Ego between sampled centrelines and
            // makes the next mission leg physically impossible to attach.
            if(goal[id] && currentOffset==1 &&
               (queueApproach || !r.globalExitPreviewActive || targetVisited(currentState))) {
                reachedState=currentState;break;
            }
            const auto relax=[&](const CorridorEdge& edge,int targetOffset,
                                 bool generatedCurve,double penalty) {
                if(edge.to<0 || !searchAllowed[edge.to] || !offsetAllowed(edge.to,targetOffset)) return;
                // A lane/section change must not teleport an adaptive offset.
                if(expandedOffsets && !generatedCurve && currentOffset>=3 &&
                   std::abs(lateralOffset(id,currentOffset)-lateralOffset(edge.to,targetOffset))>1e-5) return;
                const auto targetState=stateId(edge.to,targetOffset,
                    targetVisited(currentState) ||
                    missionPointIsTarget(*leg,r.network.points[edge.to].pose));
                // Curvature proxy for a quintic lateral manoeuvre. Long and
                // short edges replace the same forward travel, so a pure
                // length cost otherwise repeatedly selects the sharp 12m
                // version even when the vehicle nose cannot fit on the road.
                if(generatedCurve) {
                    const auto a=offsetPose(id,currentOffset);
                    const auto b=offsetPose(edge.to,targetOffset);
                    const double lateral=std::abs(-(b.x-a.x)*std::sin(a.yaw)+
                                                 (b.y-a.y)*std::cos(a.yaw));
                    penalty+=r.laneConfig.latticeCurvatureCost*5.77*lateral/
                        std::max(9.,edge.distance*edge.distance);
                }
                const double candidate=distance[currentState]+
                    std::max(.0001,edge.distance)+penalty;
                // Dominated edges need neither collision curves nor a cache
                // lookup. This is important when several lengths share a
                // reachable target and while lazy validation retries A*.
                if(candidate+1e-8>=distance[targetState]) return;
                const Parent step{currentState,edge,targetOffset,
                                  generatedCurve,true};
                if(rejectedEdges.count(edgeKey(step))) return;
                if(targetState!=startState && stateBlocked(targetState)) {
                    ++rejectedStatic;return;
                }
                if(generatedCurve && !generatedEdgeClear(
                        currentState,edge.to,targetOffset)) {
                    ++rejectedStatic;return;
                }
                distance[targetState]=candidate;
                parent[targetState]=step;
                queue.push({candidate+heuristic(targetState),targetState});
            };
            const int firstOffset=allowOffsets?0:1;
            const int lastOffset=allowOffsets?offsetStateCount:2;
            for(const auto& edge:r.forward[id]) {
                if(edge.lateral) {
                    for(int targetOffset=firstOffset;targetOffset<lastOffset;
                        ++targetOffset)
                        relax(edge,targetOffset,true,8.);
                } else if(edge.transition>=0) {
                    // Precomputed merge/split geometry is centred.  A small
                    // in-lane shift below settles an offset route beforehand.
                    if(currentOffset==1) relax(edge,1,false,0.);
                } else {
                    relax(edge,currentOffset,false,0.);
                }
            }
            for(const auto& edge:r.globalLateral[id])
                for(int targetOffset=firstOffset;targetOffset<lastOffset;++targetOffset)
                    relax(edge,targetOffset,true,8.);
            // Shifts within one lane are necessary for staggered narrow
            // obstacles.  The graph assumes the shift, then the selected
            // quintic is validated exactly below.
            // The same immutable lane walk used to allocate a set and scan
            // dozens of points for every expanded offset state and retry.
            // Its endpoints are now calculated once with the map at startup.
            if(allowOffsets) for(const auto& shift:expandedOffsets
                    ?expandedShifts[compact[id]]:r.globalInLaneShift[id]) {
                if(shift.to<0) continue;
                for(int targetOffset=0;targetOffset<offsetStateCount;
                    ++targetOffset) {
                    if(targetOffset==currentOffset) continue;
                    relax(shift,targetOffset,true,.35);
                }
            }
        }
        if(reachedState<0) break;

        std::vector<Parent> reversed;
        for(auto current=reachedState;current!=startState;) {
            if(current<0 || !parent[current].valid) {
                reachedState=-1;break;
            }
            reversed.push_back(parent[current]);
            current=parent[current].fromState;
        }
        if(reachedState<0) break;
        std::reverse(reversed.begin(),reversed.end());

        output.clear();
        Waypoint first=offsetPose(start,1);first.s=0.;append(output,first);
        bool selectedRouteValid=true;
        Parent failedStep;
        std::string failure;
        Waypoint rejectedPoint;
        std::vector<std::size_t> sampleSteps(1,0);
        for(std::size_t stepIndex=0;stepIndex<reversed.size();++stepIndex) {
            const auto& step=reversed[stepIndex];
            const auto from=statePoint(step.fromState);
            const int fromOffset=stateOffset(step.fromState);
            std::vector<Waypoint> physical;
            if(step.generatedCurve) {
                if(!offsetCurve(physical,from,fromOffset,
                                step.edge.to,step.targetOffset) ||
                   !surfaceAndMarks(r,map,physical,world,failure,true,&rejectedPoint)) {
                    selectedRouteValid=false;failedStep=step;break;
                }
            } else if(step.edge.transition>=0) {
                physical=corridorEdgePath(r,from,step.edge);
            } else {
                physical={offsetPose(from,fromOffset),
                          offsetPose(step.edge.to,step.targetOffset)};
            }
            if(expandedOffsets && !step.generatedCurve && fromOffset!=1 &&
               !surfaceAndMarks(r,map,physical,world,failure,true,&rejectedPoint)) {
                selectedRouteValid=false;failedStep=step;break;
            }
            if(physical.empty() || !stationaryPathClear(
                    physical,world,clearance,0,nullptr,&map)) {
                failure="selected_edge_intersects_stationary_object";
                selectedRouteValid=false;failedStep=step;break;
            }
            for(const auto& point:physical) {
                const auto count=output.size();
                append(output,point);
                if(output.size()>count) sampleSteps.push_back(stepIndex);
            }
        }
        if(!selectedRouteValid) {
            ++rejectedSelected;
            std::ostringstream detail;
            detail<<"attempt="<<attempt
                  <<" stage="<<stage
                  <<" reason="<<failure
                  <<" from="
                  <<r.network.points[statePoint(failedStep.fromState)].pose.roadId
                  <<':'
                  <<r.network.points[statePoint(failedStep.fromState)].pose.sectionIndex
                  <<':'
                  <<r.network.points[statePoint(failedStep.fromState)].pose.laneId
                  <<" to="<<r.network.points[failedStep.edge.to].pose.roadId
                  <<':'<<r.network.points[failedStep.edge.to].pose.sectionIndex
                  <<':'<<r.network.points[failedStep.edge.to].pose.laneId
                  <<" generated="<<failedStep.generatedCurve
                  <<" from_s="<<r.network.points[statePoint(failedStep.fromState)].pose.roadS
                  <<" to_s="<<r.network.points[failedStep.edge.to].pose.roadS
                  <<" from_offset="<<stateOffset(failedStep.fromState)
                  <<" to_offset="<<failedStep.targetOffset
                  <<" failed_x="<<rejectedPoint.x<<" failed_y="<<rejectedPoint.y
                  <<" failed_yaw="<<rejectedPoint.yaw;
            traceDecision(trace,"corridor_global_route",
                          "fast_selected_edge_rejected",detail.str());
            rejectedEdges.insert(edgeKey(failedStep));
            output.clear();
            continue;
        }
        if(output.size()<2 || reversed.empty()) {
            traceDecision(trace,"corridor_global_route",
                          "fast_materialization_failed","assembled_path_too_short");
            output.clear();return false;
        }
        // Keep the selected-edge provenance before source-join smoothing can
        // insert/remove samples. A bad assembled join rejects this selected
        // edge, not every other route through the full corridor.
        const auto beforeSmoothing=output;
        repairTrackingTangents(output);
        const bool joined=smoothSourceJoins(output,r,map,world,&failure,true,
            clearance,&rejectedPoint);
        int assembledBlocker=-1;
        const bool assembledClear=joined && stationaryPathClear(output,world,
            clearance,0,&assembledBlocker,&map,&rejectedPoint);
        if(!assembledClear) {
            std::size_t nearestSample=0;double closest=inf;
            for(std::size_t i=0;i<beforeSmoothing.size();++i) {
                const double d=gap(beforeSmoothing[i],rejectedPoint);
                if(d<closest) {closest=d;nearestSample=i;}
            }
            const auto& invalid=reversed[sampleSteps[nearestSample]];
            const auto key=edgeKey(invalid);
            std::ostringstream detail;
            detail<<"attempt="<<attempt
                  <<" stage="<<stage
                  <<" reason="<<(joined?"assembled_path_intersects_stationary_object":"source_join_unavailable")
                  <<" blocker="<<assembledBlocker<<" failed_x="<<rejectedPoint.x
                  <<" failed_y="<<rejectedPoint.y<<" nearest_sample_distance="<<closest
                  <<" from_point="<<statePoint(invalid.fromState)
                  <<" to_point="<<invalid.edge.to<<" join_detail="<<failure;
            traceDecision(trace,"corridor_global_route","assembled_edge_rejected",detail.str());
            if(!rejectedEdges.insert(key).second) {output.clear();return false;}
            ++rejectedSelected;output.clear();continue;
        }
        // A farther goal is not permission to bypass the required turn. A
        // route that starts before the exit must actually include a directed
        // sample on the selected (including proven same-exit) gateway lanes.
        const bool startsPastGateway=r.activeGatewayObserved ||
            missionPointIsTarget(*leg,r.network.points[start].pose);
        if(!queueApproach && r.globalExitPreviewActive && !startsPastGateway &&
           !std::any_of(output.begin(),output.end(),[&](const Waypoint& p) {
               return leg->gatewayLanes.count(keyOf(p))!=0;
           })) {
            traceDecision(trace,"corridor_global_route","gateway_passage_missing",
                          "exit_preview_cannot_skip_required_gateway=1");
            output.clear();return false;
        }
        break;
    }
    if(reachedState<0 || output.size()<2) {
        traceDecision(trace,"corridor_global_route",
                      "precomputed_gateway_unreachable",
                      std::string("stage=")+stage+
                      " expanded="+std::to_string(expandedTotal)+
                      " goals="+std::to_string(goalCount)+
                      " blocked_samples="+std::to_string(rejectedStatic)+
                      " rejected_selected_edges="+
                          std::to_string(rejectedSelected)+
                      " selected_route_retry_limit="+
                          std::to_string(rejectedSelected>=maximumSelectedRouteRetries));
        output.clear();return false;
    }
    std::ostringstream detail;
    detail<<"clearance="<<clearance<<" stage="<<stage
          <<" revision="<<(r.routeRevision+1)
          <<" start="<<r.network.points[start].pose.roadId<<':'
          <<r.network.points[start].pose.sectionIndex<<':'
          <<r.network.points[start].pose.laneId
          <<" goal="<<r.network.points[statePoint(reachedState)].pose.roadId<<':'
          <<r.network.points[statePoint(reachedState)].pose.sectionIndex<<':'
          <<r.network.points[statePoint(reachedState)].pose.laneId
          <<" points="<<output.size()
          <<" expanded="<<expandedTotal
          <<" blocked_samples="<<rejectedStatic
          <<" rejected_selected_edges="<<rejectedSelected
          <<" exit_preview="<<r.globalExitPreviewActive
          <<" rule=stationary_clear_first_directed_sample_on_required_exit_road";
    traceDecision(trace,"corridor_global_route",
                  "precomputed_gateway_materialized",detail.str());
    return true;
}

bool materializeGatewaySearchTier(
    std::vector<Waypoint>& output,CorridorRuntime& r,
    const PathGeneratorData& map,const WorldState& world,
    const MissionCorridorData& mission,std::int64_t start,
    DecisionTraceData* trace,bool expanded,double clearance) {
    // First branch repeatedly through physical lane centres all the way to
    // the mandatory exit. Only an exhausted centre search opens offset
    // samples for narrow/staggered gaps. This is a feasibility priority,
    // not a centre attraction cost that can change the selected gateway.
    if(!expanded && materializePrecomputedGatewayRouteStage(
           output,r,map,world,mission,start,trace,false,false,false,clearance)) return true;
    traceDecision(trace,"corridor_global_route","offset_search_started",
                  "lane_center_gateway_route_unavailable=1");
    if(materializePrecomputedGatewayRouteStage(
           output,r,map,world,mission,start,trace,true,false,expanded,clearance)) return true;

    const auto* leg=activeMissionLeg(mission);
    if(!mission.topologyAuthority || !leg || leg->finalLeg ||
       mission.activeLegIndex+1>=mission.legs.size()) return false;
    const auto& nextLeg=mission.legs[mission.activeLegIndex+1];
    // A return curve can already cross the required exit lane before the
    // ordinary-road frontier, but settle only inside its declared successor
    // connector. Cutting its GRAPH endpoint at the frontier discards that
    // otherwise collision-free curve. As a last full-route search, complete
    // exactly the following declared leg, never an arbitrary neighbouring
    // road. Keep the CURRENT mission and its target/gateway certificates.
    auto ordinaryAllowed=r.globalSearchAllowed;
    auto ordinaryGoals=r.globalSearchGoal;
    const bool ordinaryPreview=r.globalExitPreviewActive;
    r.globalSearchAllowed=r.trackingAllowed;
    r.globalSearchGoal.assign(r.network.points.size(),false);
    for(const auto& point:r.network.points)
        if(r.trackingAllowed[point.id] &&
           missionPointIsTarget(nextLeg,point.pose))
            r.globalSearchGoal[point.id]=true;
    r.globalExitPreviewActive=true;
    traceDecision(trace,"corridor_global_route","declared_next_gateway_search_started",
                  "ordinary_frontier_unreachable=1 current_mission_unchanged=1");
    bool success=false;
    for(const bool offsets:{false,true}) {
        if(expanded && !offsets) continue;
        if(!materializePrecomputedGatewayRouteStage(
               output,r,map,world,mission,start,trace,offsets,false,expanded,clearance)) continue;
        const bool nextGatewayPassed=nextLeg.gatewayLanes.empty() ||
            std::any_of(output.begin(),output.end(),[&](const Waypoint& point) {
                return nextLeg.gatewayLanes.count(keyOf(point))!=0;
            });
        if(nextGatewayPassed) {success=true;break;}
        traceDecision(trace,"corridor_global_route","next_gateway_passage_missing",
                      "extended_search_cannot_skip_declared_next_gateway=1");
        output.clear();
    }
    r.globalSearchAllowed=std::move(ordinaryAllowed);
    r.globalSearchGoal=std::move(ordinaryGoals);
    r.globalExitPreviewActive=ordinaryPreview || success;
    return success;
}

bool materializePrecomputedGatewayRoute(
    std::vector<Waypoint>& output,CorridorRuntime& r,
    const PathGeneratorData& map,const WorldState& world,
    const MissionCorridorData& mission,std::int64_t start,
    DecisionTraceData* trace,double* validatedClearance=nullptr) {
    const double nominal=r.laneConfig.latticeObstacleClearance;
    const double tight=std::min(nominal,r.laneConfig.latticeTightObstacleClearance);
    for(int tier=0;tier<3;++tier) {
        if(tier==2 && tight>=nominal) break;
        const double clearance=tier==2?tight:nominal;
        if(tier>0) traceDecision(trace,"corridor_global_route","expanded_search_started",
            "tier="+std::to_string(tier)+" clearance="+std::to_string(clearance)+
            " previous_tier_exhausted=1");
        if(materializeGatewaySearchTier(output,r,map,world,mission,start,trace,tier>0,clearance)) {
            if(validatedClearance) *validatedClearance=clearance;
            return true;
        }
    }
    return false;
}

bool soleGatewayApproach(std::vector<Waypoint>& output,int& blocker,
    CorridorRuntime& r,const PathGeneratorData& map,const WorldState& world,
    const MissionCorridorData& mission,std::int64_t start,DecisionTraceData* trace,
    std::vector<int>& queueBlockers,bool& inEntranceLane) {
    queueBlockers.clear();
    inEntranceLane=false;
    const auto* leg=activeMissionLeg(mission);
    if(!leg || leg->gatewayLanes.empty() ||
       leg->gatewayCheckpoint>=mission.checkpoints.size()) return false;
    auto approachLanes=leg->gatewayLanes;
    // This fallback runs only after all full collision-free gateway searches.
    // With multiple blocked gateways, retain their alternatives in the same
    // graph: ordinary lateral-change/distance cost chooses the approach.
    // An empty parallel dead end is never a gateway alternative.
    for(std::size_t pass=0;pass<leg->allowedLanes.size();++pass) {
        const auto previous=approachLanes;
        for(const auto& entry:map.laneGraph) {
            if(!leg->allowedLanes.count(entry.first)) continue;
            for(const auto& to:entry.second) {
                const bool lateral=entry.first.roadId==to.roadId &&
                    entry.first.sectionIndex==to.sectionIndex && entry.first.laneId!=to.laneId;
                if(!lateral && previous.count(to)) {approachLanes.insert(entry.first);break;}
            }
        }
        if(previous.size()==approachLanes.size()) break;
    }
    WorldState probe=world;std::vector<int> gateBlockers;
    probe.objects.erase(std::remove_if(probe.objects.begin(),probe.objects.end(),
        [&](const WorldObjectState& object) {
            if(!isStationaryRouteObstacle(object) || objectMotionSpeed(object)>=.1) return false;
            // The outgoing gateway may turn 90 degrees from Ego. Testing
            // its object with Ego's heading would miss exactly that blocker.
            // Use occupancy on the directed approach samples instead.
            const auto body=egoFootprint(world);
            for(const auto& key:approachLanes) {
                const auto lane=r.network.lanes.find(key);
                if(lane==r.network.lanes.end()) continue;
                for(const auto id:lane->second.pointIds) {
                    if(id<0 || !r.globalSearchAllowed[id]) continue;
                    const auto& p=r.network.points[id].pose;
                    if(!intersects(vehicleFootprintAt(body,{p.x,p.y,p.yaw},
                              r.laneConfig.latticeObstacleClearance),object.obb) ||
                       !corridorObstacleAtPathLevel(map,p,object)) continue;
                    gateBlockers.push_back(static_cast<int>(object.raw.id));return true;
                }
            }
            return false;
        }),probe.objects.end());
    if(gateBlockers.empty()) return false;
    // The full vehicle-footprint search has already failed. Distance to a
    // stop line/checkpoint cannot override that feasibility result.
    queueBlockers=gateBlockers;
    // Search TO the back of a queue on a longitudinal predecessor of the
    // declared gateway, with EVERY real obstacle present. Truncating a route
    // searched with blockers erased can stop before an unfinished lane change
    // and strand Ego beside the actual entrance queue.
    auto originalGoals=r.globalSearchGoal;
    r.globalSearchGoal.assign(r.network.points.size(),false);
    std::map<std::int64_t,int> queueGoals;
    const auto body=egoFootprint(world);
    for(const auto& key:approachLanes) {
        const auto lane=r.network.lanes.find(key);
        if(lane==r.network.lanes.end() || lane->second.pointIds.size()<2) continue;
        const auto& ids=lane->second.pointIds;
        std::size_t tail=ids.size();int tailBlocker=-1;
        double lastBlockedS=inf;
        for(std::size_t j=ids.size();j-->0;) {
            const auto id=ids[j];const auto& p=r.network.points[id].pose;
            if(!r.globalSearchAllowed[id]) continue;
            if(tail<ids.size() && lastBlockedS-p.s>8.) break;
            int occupant=-1;
            const auto box=vehicleFootprintAt(body,{p.x,p.y,p.yaw},
                r.laneConfig.latticeObstacleClearance);
            for(const auto& object:world.objects) {
                if(std::find(gateBlockers.begin(),gateBlockers.end(),
                    static_cast<int>(object.raw.id))==gateBlockers.end()) continue;
                if(intersects(box,object.obb) && corridorObstacleAtPathLevel(map,p,object)) {
                    occupant=static_cast<int>(object.raw.id);break;
                }
            }
            if(occupant<0) continue;
            tail=j;tailBlocker=occupant;lastBlockedS=p.s;
        }
        if(tail==ids.size()) continue;
        const double endS=r.network.points[ids[tail]].pose.s-
            PlanningBehaviorConfig{}.stopSafetyMargin;
        while(tail>0 && r.network.points[ids[tail]].pose.s>endS) --tail;
        const auto goalId=ids[tail];const auto& p=r.network.points[goalId].pose;
        if(!r.globalSearchAllowed[goalId] ||
           r.network.points[goalId].pose.s>endS ||
           gap(p,r.network.points[start].pose)<2.) continue;
        queueGoals[goalId]=tailBlocker;r.globalSearchGoal[goalId]=true;
    }
    bool queueFound=false;
    if(!queueGoals.empty()) for(const bool offsets:{false,true}) {
        if(!materializePrecomputedGatewayRouteStage(output,r,map,world,mission,
                start,trace,offsets,true)) continue;
        auto selected=queueGoals.begin();double best=inf;
        for(auto it=queueGoals.begin();it!=queueGoals.end();++it) {
            const double d=gap(output.back(),r.network.points[it->first].pose);
            if(d<best) {best=d;selected=it;}
        }
        blocker=selected->second;queueFound=true;break;
    }
    r.globalSearchGoal=std::move(originalGoals);
    if(queueFound) {
        inEntranceLane=true;
        traceDecision(trace,"sole_gateway","entrance_lane_queue_approach_built",
            "all_real_obstacles_checked=1 gateway_completed=0 blocker="+
            std::to_string(blocker)+" target_lane="+
            std::to_string(output.back().roadId)+":"+
            std::to_string(output.back().sectionIndex)+":"+
            std::to_string(output.back().laneId));
        return true;
    }
    // Only if none of the entrance queue endpoints is safely reachable may
    // the old partial approach wait in another lane. It is not a completion
    // certificate and does not disable the existing green static timeout.
    std::vector<Waypoint> route;
    if(!materializePrecomputedGatewayRoute(route,r,map,probe,mission,start,trace)) return false;
    const double free=corridorStationaryFreeDistance(route,world,
        r.laneConfig.latticeObstacleClearance,&map,&blocker);
    if(!std::isfinite(free) || std::find(gateBlockers.begin(),gateBlockers.end(),blocker)==gateBlockers.end())
        return false;
    const double length=free-PlanningBehaviorConfig{}.stopSafetyMargin;
    if(length<2.) return false;
    const double limit=route.front().s+length;
    route.erase(std::upper_bound(route.begin(),route.end(),limit,
        [](double s,const Waypoint& p){return s<p.s;}),route.end());
    if(route.size()<2 || !corridorTrackingPathIsSafe(route,world,
           r.laneConfig.latticeObstacleClearance,nullptr,nullptr,&map)) return false;
    output=std::move(route);
    traceDecision(trace,"sole_gateway","safe_approach_built",
        "full_collision_free_search_failed=1 gateway_retained=1 blocker="+std::to_string(blocker)+
        " length="+std::to_string(length)+" gateway_completed=0");
    return true;
}

bool globalRouteSlice(std::vector<Waypoint>& output,CorridorRuntime& r,
                      const WorldState& world,double horizon,
                      bool updateProgress=true) {
    output.clear();
    if(!r.committedGlobalPathReachesGateway ||
       r.committedGlobalPathLeg!=r.activeLeg ||
       r.committedGlobalPath.size()<2) return false;
    const auto& route=r.committedGlobalPath;
    const std::size_t searchBegin=r.committedGlobalPathProgress>20
        ? r.committedGlobalPathProgress-20 : 0;
    std::size_t nearestIndex=searchBegin;double best=inf;
    double bestDistance=inf,bestYaw=inf;
    for(std::size_t i=searchBegin;i<route.size();++i) {
        const double d=std::hypot(route[i].x-world.ego.x,
                                  route[i].y-world.ego.y);
        const double yaw=std::abs(normalizeAngle(
            route[i].yaw-world.ego.heading));
        if(yaw>1.35) continue;
        const double backward=i<r.committedGlobalPathProgress
            ? .02*double(r.committedGlobalPathProgress-i) : 0.;
        const double score=d+1.5*yaw+backward;
        if(score<best) {
            best=score;bestDistance=d;bestYaw=yaw;nearestIndex=i;
        }
    }
    // A completed dynamic/local detour may leave Ego on another lane.  Do
    // not pull it several metres sideways toward the stale pre-detour route;
    // force a new current-position-to-gateway search instead.
    if(!std::isfinite(best) || bestDistance>1.25 || bestYaw>0.8)
        return false;
    if(updateProgress)
        r.committedGlobalPathProgress=std::max(
            r.committedGlobalPathProgress,nearestIndex);
    const std::size_t begin=nearestIndex>4?nearestIndex-4:0;
    const double offset=world.egoPathS-route[nearestIndex].s;
    for(std::size_t i=begin;i<route.size();++i) {
        auto point=route[i];point.s+=offset;append(output,point);
        if(i>nearestIndex && point.s>=world.egoPathS+horizon) break;
    }
    return output.size()>=2;
}

bool recoveryRouteSlice(std::vector<Waypoint>& output,CorridorRuntime& r,
                        const WorldState& world,double horizon,const PathGeneratorData& map) {
    output.clear();
    if(r.recoveryTrackingPathLeg!=r.activeLeg ||
       r.recoveryTrackingPath.size()<2) return false;
    const auto& route=r.recoveryTrackingPath;
    const std::size_t searchBegin=r.recoveryTrackingPathProgress>12
        ? r.recoveryTrackingPathProgress-12 : 0;
    std::size_t nearestIndex=searchBegin;
    double best=inf,bestDistance=inf,bestYaw=inf;
    for(std::size_t i=searchBegin;i<route.size();++i) {
        const double distance=std::hypot(
            route[i].x-world.ego.x,route[i].y-world.ego.y);
        const double yaw=std::abs(normalizeAngle(
            route[i].yaw-world.ego.heading));
        if(yaw>0.8) continue;
        const double backward=i<r.recoveryTrackingPathProgress
            ? .03*double(r.recoveryTrackingPathProgress-i) : 0.;
        const double score=distance+.5*yaw+backward;
        if(score>=best) continue;
        best=score;bestDistance=distance;bestYaw=yaw;nearestIndex=i;
    }
    if(!std::isfinite(best) || bestDistance>1.25 || bestYaw>0.8 ||
       nearestIndex+2>=route.size() ||
       !stationaryPathClear(route,world,r.recoveryTrackingPathClearance>=0.
                                ?r.recoveryTrackingPathClearance:r.laneConfig.latticeObstacleClearance,
                            nearestIndex,nullptr,&map)) {
        r.recoveryTrackingPath.clear();
        r.recoveryTrackingPathProgress=0;
        return false;
    }
    r.recoveryTrackingPathProgress=std::max(
        r.recoveryTrackingPathProgress,nearestIndex);
    const std::size_t begin=nearestIndex>4?nearestIndex-4:0;
    const double offset=world.egoPathS-route[nearestIndex].s;
    for(std::size_t i=begin;i<route.size();++i) {
        auto point=route[i];point.s+=offset;append(output,point);
        if(i>nearestIndex && point.s>=world.egoPathS+horizon) break;
    }
    return output.size()>=2;
}

bool queueApproachSlice(std::vector<Waypoint>& output,const CorridorRuntime& r,
                        const WorldState& world,double horizon) {
    output.clear();
    const auto& route=r.soleGatewayApproachPath;
    std::size_t near=0;double best=inf;
    for(std::size_t i=0;i<route.size();++i) {
        if(std::abs(normalizeAngle(route[i].yaw-world.ego.heading))>.8) continue;
        const double distance=std::hypot(route[i].x-world.ego.x,route[i].y-world.ego.y);
        if(distance<best) {best=distance;near=i;}
    }
    if(best>1.25 || near+1>=route.size()) return false;
    // The caller creates a fresh Pure Pursuit planner each frame. Its bounded
    // nearest-point search starts at index zero: returning the entire cached
    // approach traps tracking in the first 35 m after Ego has driven past it.
    // Keep the committed curve, but hand off only the current tracking slice.
    const double offset=world.egoPathS-route[near].s;
    for(std::size_t i=near>4?near-4:0;i<route.size();++i) {
        auto point=route[i];point.s+=offset;append(output,point);
        if(i>near && point.s>=world.egoPathS+horizon) break;
    }
    return output.size()>=2;
}

bool ensureObstacleAwareGlobalRoute(
    std::vector<Waypoint>& output,CorridorRuntime& r,
    const PathGeneratorData& map,const WorldState& world,
    const MissionCorridorData& mission,double horizon,
    DecisionTraceData* trace) {
    std::vector<Waypoint> current;
    const bool sliceAvailable=globalRouteSlice(current,r,world,horizon);
    bool retained=sliceAvailable && !r.globalRouteReplanRequested;
    if(retained) {
        // Check only the unconsumed suffix.  An obstacle already passed must
        // not invalidate a route merely because its old point is still stored.
        retained=stationaryPathClear(
            r.committedGlobalPath,world,r.globalPathClearance(),
            r.committedGlobalPathProgress,nullptr,&map);
    }
    if(retained) {output=std::move(current);r.trackingPathValidatedClearance=r.globalPathClearance();return true;}

    if (r.budgetedSearch && mission.topologyAuthority && r.globalRouteWorker) {
        auto& worker = *r.globalRouteWorker;
        if (worker.pending.valid() &&
            worker.pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto result = worker.pending.get();
            for(const auto& event:result.diagnostics)
                traceDecision(trace,"corridor_global_route","worker_detail",event);
            {
                std::ostringstream detail;
                detail<<"success="<<result.success<<" search_ms="<<result.searchMilliseconds
                      <<" snapshot_age="<<(world.receiveTime-result.receiveTime)
                      <<" result_leg="<<result.leg<<" active_leg="<<r.activeLeg
                      <<" result_revision="<<result.revision
                      <<" current_revision="<<r.globalRouteRequestRevision
                      <<" start="<<result.startLane.roadId<<':'<<result.startLane.sectionIndex
                      <<':'<<result.startLane.laneId<<" start_s="<<result.startRoadS
                      <<" points="<<result.path.size();
                traceDecision(trace,"corridor_global_route","worker_completed",detail.str());
            }
            if (result.leg == r.activeLeg && result.revision == r.globalRouteRequestRevision) {
                if(!result.exception && result.staticSignature==stationarySceneSignature(world))
                    r.gatewayQueueBlockers=result.queueBlockers;
                // The vehicle and observations have advanced while A* ran.
                // Validate attachment and the remaining static-clear suffix
                // against THIS frame before adopting the asynchronous result.
                std::size_t nearestIndex = 0;
                double best = inf;
                for (std::size_t i=0; i<result.path.size(); ++i) {
                    const auto& p = result.path[i];
                    if (std::abs(normalizeAngle(p.yaw-world.ego.heading)) > .8) continue;
                    const double d = std::hypot(p.x-world.ego.x,p.y-world.ego.y);
                    if (d < best) {best=d;nearestIndex=i;}
                }
                int blocker=-1;
                const bool observedSuffixClear=result.success &&
                    stationaryPathClear(result.path,world,result.clearance,
                                        nearestIndex,&blocker,&map);
                if(result.success && (best>1.25 || nearestIndex+1>=result.path.size() || !observedSuffixClear)) {
                    std::ostringstream detail;
                    detail<<"attachment="<<best<<" nearest="<<nearestIndex
                          <<" suffix_clear="<<observedSuffixClear<<" blocker="<<blocker;
                    traceDecision(trace,"corridor_global_route","successful_route_not_adopted",detail.str());
                }
                if (result.success && best <= 1.25 && nearestIndex+1 < result.path.size() && observedSuffixClear) {
                    if(result.approachOnly) {
                        // A retry exists to discover an OPEN through-route.
                        // Replacing an attached queue approach every second
                        // repeatedly postpones its already chosen lane change.
                        // Upgrade a parallel partial wait to the entrance lane,
                        // but otherwise keep the same safe queue trajectory.
                        const bool retainApproach=
                            r.soleGatewayApproachSignature==result.staticSignature &&
                            r.soleGatewayApproachBlocker==result.approachBlocker &&
                            (r.soleGatewayApproachInEntranceLane || !result.approachInEntranceLane) &&
                            corridorTrackingPathIsSafe(r.soleGatewayApproachPath,world,
                                r.laneConfig.latticeObstacleClearance,nullptr,nullptr,&map);
                        if(!retainApproach) {
                            r.soleGatewayApproachPath=std::move(result.path);
                            r.soleGatewayApproachBlocker=result.approachBlocker;
                            r.soleGatewayApproachSignature=result.staticSignature;
                            r.soleGatewayApproachInEntranceLane=result.approachInEntranceLane;
                        }
                        r.globalRouteSearchFailed=true;r.globalRouteFailureLane=result.startLane;
                        r.globalRouteFailureRoadS=result.startRoadS;
                        r.globalRouteFailureStaticSignature=result.staticSignature;
                        r.globalRouteRetryAfter=world.receiveTime+1.;
                        traceDecision(trace,"sole_gateway",retainApproach?
                            "safe_approach_retained":"safe_approach_adopted",
                            "gateway_completed=0 actual_pose_and_obstacles_rechecked=1 entrance_lane="+
                            std::to_string(r.soleGatewayApproachInEntranceLane));
                    } else {
                    r.committedGlobalPath = std::move(result.path);
                    r.committedGlobalPathClearance=result.clearance;
                    r.soleGatewayApproachPath.clear();
                    r.gatewayQueueBlockers.clear();
                    r.committedGlobalPathLeg = r.activeLeg;
                    r.committedGlobalPathProgress = nearestIndex;
                    r.committedGlobalPathReachesGateway = true;
                    r.committedGlobalPathHasExitPreview = result.exitPreview;
                    r.globalRouteSearchFailed = false;
                    r.globalRouteReplanRequested = false;
                    r.recoveryTrackingPath.clear();
                    ++r.routeRevision;
                    traceDecision(trace,"corridor_global_route","async_route_adopted",
                                  "current_pose_and_observed_static_suffix_revalidated=1");
                    if (globalRouteSlice(output,r,world,horizon)) {r.trackingPathValidatedClearance=r.globalPathClearance();return true;}
                    }
                } else if (!result.success) {
                    r.globalRouteSearchFailed = true;
                    r.globalRouteFailureLane = result.startLane;
                    r.globalRouteFailureRoadS = result.startRoadS;
                    r.globalRouteFailureStaticSignature = result.staticSignature;
                    r.globalRouteRetryAfter = world.receiveTime+5.;
                    traceDecision(trace,"corridor_global_route",
                                  result.exception?"async_search_exception":"async_search_exhausted",
                                  "keep_local_avoidance_and_retry_on_scene_change=1");
                } else {
                    // A successful but stale plan is not evidence that the
                    // current pose has no route; request a fresh snapshot.
                    r.globalRouteSearchFailed = false;
                    traceDecision(trace,"corridor_global_route","async_stale_route_discarded",
                                  "ego_or_observations_changed=1");
                }
            }
        }
        const LaneKey currentLane{world.egoRoadId,world.egoSectionIndex,world.egoLaneId};
        const auto signature = stationarySceneSignature(world);
        const bool suppressed = r.globalRouteSearchFailed &&
            r.globalRouteFailureLane == currentLane &&
            r.globalRouteFailureStaticSignature == signature &&
            std::abs(world.egoRoadS-r.globalRouteFailureRoadS) < 8. &&
            world.receiveTime < r.globalRouteRetryAfter;
        if (!worker.pending.valid() && !suppressed) {
            const auto snapshot = worker.runtime;
            const auto mapSnapshot = worker.map;
            const auto revision = r.globalRouteRequestRevision;
            const bool gatewayObserved = r.activeGatewayObserved;
            const bool diagnostics=trace && trace->enabled;
            try {
              worker.pending = std::async(std::launch::async,
                [snapshot,mapSnapshot,world,mission,signature,revision,diagnostics,gatewayObserved]() {
                    const auto started=std::chrono::steady_clock::now();
                    CorridorGlobalRouteWorker::Result result;
                    result.leg = mission.activeLegIndex;
                    result.revision = revision;
                    result.staticSignature = signature;
                    result.startLane = {world.egoRoadId,world.egoSectionIndex,world.egoLaneId};
                    result.startRoadS = world.egoRoadS;
                    result.receiveTime = world.receiveTime;
                    DecisionTraceData workerTrace;workerTrace.enabled=diagnostics;
                    try {
                        activate(*snapshot,mission);
                        snapshot->activeGatewayObserved=gatewayObserved;
                        const auto start = nearest(*snapshot,world);
                        const auto attachCandidate=[&]() {
                            if(result.path.empty()) return false;
                            if(std::hypot(result.path.front().x-world.ego.x,
                                          result.path.front().y-world.ego.y)<=.35 &&
                               std::abs(normalizeAngle(result.path.front().yaw-world.ego.heading))<=.1)
                                return true;
                            std::vector<Waypoint> attached;
                            bool valid=connectToReferencePath(attached,result.path,0,
                                *snapshot,*mapSnapshot,world,inf,&workerTrace,true,result.clearance);
                            if(valid) {
                                const auto* leg=activeMissionLeg(mission);
                                Waypoint actual;actual.roadId=world.egoRoadId;
                                actual.sectionIndex=world.egoSectionIndex;
                                actual.laneId=world.egoLaneId;actual.roadS=world.egoRoadS;
                                if(!result.approachOnly && leg && !leg->gatewayLanes.empty() &&
                                   !gatewayObserved && !missionPointIsTarget(*leg,actual) &&
                                   std::none_of(attached.begin(),attached.end(),[&](const auto& p){
                                       return leg->gatewayLanes.count(keyOf(p))!=0;
                                   })) valid=false;
                            }
                            if(valid) result.path=std::move(attached);
                            else {
                                result.path.clear();
                                traceDecision(&workerTrace,"corridor_global_route","snapshot_attachment_failed",
                                    "measured_pose_requires_checked_reentry=1");
                            }
                            return valid;
                        };
                        // A full route includes attachment from the measured
                        // pose. An unattached map route must not suppress an
                        // otherwise reachable waiting approach.
                        if(start>=0) result.success=materializePrecomputedGatewayRoute(
                            result.path,*snapshot,*mapSnapshot,world,mission,start,&workerTrace,&result.clearance) &&
                            attachCandidate();
                        if(start>=0 && !result.success) {
                            result.approachOnly=true;
                            result.clearance=snapshot->laneConfig.latticeObstacleClearance;
                            result.success=soleGatewayApproach(result.path,result.approachBlocker,
                                *snapshot,*mapSnapshot,world,mission,start,&workerTrace,result.queueBlockers,
                                result.approachInEntranceLane) &&
                                attachCandidate();
                        }
                        result.exitPreview=result.success && !result.approachOnly && snapshot->globalExitPreviewActive;
                    } catch (...) {result.success=false;result.exception=true;}
                    result.searchMilliseconds=std::chrono::duration<double,std::milli>(
                        std::chrono::steady_clock::now()-started).count();
                    result.diagnostics=std::move(workerTrace.frameEvents);
                    return result;
                });
              r.globalRouteReplanRequested = true;
              traceDecision(trace,"corridor_global_route","async_search_started",
                          std::string("scope=")+(r.globalExitPreviewActive?
                              "active_gateway_and_next_corridor_entry":"whole_active_corridor")+
                              " received_objects="+
                              std::to_string(world.objects.size())+" unseen_space_not_verified=1");
            } catch (const std::exception& ex) {
                r.globalRouteSearchFailed=true;
                r.globalRouteFailureLane=currentLane;
                r.globalRouteFailureRoadS=world.egoRoadS;
                r.globalRouteFailureStaticSignature=signature;
                r.globalRouteRetryAfter=world.receiveTime+1.;
                traceDecision(trace,"corridor_global_route","async_launch_failed",ex.what());
            }
        }
        // A newly checked approach must not be hidden by a blocked old route.
        if(sliceAvailable && stationaryPathClear(r.committedGlobalPath,world,
               r.globalPathClearance(),r.committedGlobalPathProgress,nullptr,&map) &&
           corridorTrackingPathIsSafe(current,world,r.globalPathClearance(),
                                      nullptr,nullptr,&map)) {
            output=std::move(current);r.trackingPathValidatedClearance=r.globalPathClearance();return true;
        }
        if(queueApproachSlice(output,r,world,horizon) &&
           corridorTrackingPathIsSafe(output,world,r.laneConfig.latticeObstacleClearance,
                                      nullptr,nullptr,&map)) {
            r.soleGatewayApproachSelected=true;return true;
        }
        if (sliceAvailable) {output=std::move(current);r.trackingPathValidatedClearance=r.globalPathClearance();return true;}
        // The ordinary gateway-aware sampled path and bounded local avoidance
        // continue sending commands while the global worker is busy.
        return false;
    }

    // Never run the exhaustive, whole-corridor obstacle search from the
    // real-time control frame.  It evaluates thousands of swept quintics and
    // used to block command transmission for 4--13 wall-clock seconds in
    // M11 even though the nominal control period is 50 ms.  The live planner
    // already has two bounded layers for this situation:
    //   1. forwardPath() follows the precomputed XODR/gateway graph, and
    //   2. prepareCorridorLattice() searches all corridor lanes under the
    //      shared lattice_search_budget_ms deadline.
    // If a committed route has just become obstructed, retain its slice so
    // the bounded local planner can replace the blocked part atomically.  If
    // no committed route exists, return false and let buildCorridorTrackingPath
    // use the precomputed gateway-aware forward path.  Offline callers do not
    // call beginCorridorSearchFrame(), so they may still materialize and audit
    // a complete obstacle-aware route exhaustively.
    if(r.budgetedSearch) {
        const auto start=nearest(r,world);
        const LaneKey failureLane=start>=0
            ? keyOf(r.network.points[start].pose)
            : LaneKey{world.egoRoadId,world.egoSectionIndex,world.egoLaneId};
        const std::size_t staticCount=std::count_if(
            world.objects.begin(),world.objects.end(),
            [](const auto& object){
                return isStationaryRouteObstacle(object);
            });
        // A physically blocked corridor can remain impossible for many
        // frames.  Repeating even the fast whole-leg search at 20 Hz steals
        // time from command transmission.  Retry when the object set changes,
        // Ego advances materially, or five simulation seconds pass.
        const bool retrySuppressed=r.globalRouteSearchFailed &&
            r.globalRouteFailureLane==failureLane &&
            r.globalRouteFailureStaticSignature==stationarySceneSignature(world) &&
            std::abs(world.egoRoadS-r.globalRouteFailureRoadS)<8.0 &&
            world.receiveTime<r.globalRouteRetryAfter;
        std::vector<Waypoint> rebuilt;
        double routeClearance=r.laneConfig.latticeObstacleClearance;
        if(!retrySuppressed && start>=0 && materializePrecomputedGatewayRoute(
                rebuilt,r,map,world,mission,start,trace,&routeClearance)) {
            r.committedGlobalPath=std::move(rebuilt);
            r.committedGlobalPathClearance=routeClearance;
            r.committedGlobalPathLeg=r.activeLeg;
            r.committedGlobalPathProgress=0;
            r.committedGlobalPathReachesGateway=true;
            r.committedGlobalPathHasExitPreview=r.globalExitPreviewActive;
            r.globalRouteSearchFailed=false;
            r.globalRouteReplanRequested=false;
            r.recoveryTrackingPath.clear();
            ++r.routeRevision;
            if(globalRouteSlice(output,r,world,horizon)) {r.trackingPathValidatedClearance=r.globalPathClearance();return true;}
        }
        if(!retrySuppressed) {
            r.globalRouteSearchFailed=true;
            r.globalRouteFailureLane=failureLane;
            r.globalRouteFailureStaticCount=staticCount;
            r.globalRouteFailureStaticSignature=stationarySceneSignature(world);
            r.globalRouteFailureRoadS=world.egoRoadS;
            r.globalRouteRetryAfter=world.receiveTime+5.0;
        }
        if(sliceAvailable) {
            output=std::move(current);
            traceDecision(trace,"corridor_global_route",
                          "blocked_route_retained_for_local_avoidance",
                          retrySuppressed
                              ? "complete_replan_retry_throttled=1"
                              : "complete_replan_unavailable=1");
            return true;
        }
        traceDecision(trace,"corridor_global_route",
                      "realtime_precomputed_fallback",
                      retrySuppressed
                          ? "gateway_route_retry_throttled=1"
                          : "gateway_route_unavailable=1");
        return false;
    }

    const auto start=nearest(r,world);
    const LaneKey failureLane{world.egoRoadId,world.egoSectionIndex,
                              world.egoLaneId};
    const std::size_t staticCount=std::count_if(
        world.objects.begin(),world.objects.end(),
        [](const auto& object){return isStationaryRouteObstacle(object);});
    const bool unchangedFailedSearch=r.globalRouteSearchFailed &&
        r.globalRouteFailureLane==failureLane &&
        r.globalRouteFailureStaticCount==staticCount &&
        r.globalRouteFailureStaticSignature==stationarySceneSignature(world) &&
        std::abs(world.egoRoadS-r.globalRouteFailureRoadS)<8. &&
        world.receiveTime<r.globalRouteRetryAfter;
    std::vector<Waypoint> rebuilt;
    if(!unchangedFailedSearch && start>=0 &&
       materializeObstacleAwareGlobalRoute(
            rebuilt,r,map,world,mission,start,trace)) {
        r.committedGlobalPath=std::move(rebuilt);
        r.committedGlobalPathClearance=r.laneConfig.latticeObstacleClearance;
        r.committedGlobalPathLeg=r.activeLeg;
        r.committedGlobalPathProgress=0;
        r.committedGlobalPathReachesGateway=true;
        r.globalRouteSearchFailed=false;
        r.globalRouteReplanRequested=false;
        r.recoveryTrackingPath.clear();
        ++r.routeRevision;
        if(globalRouteSlice(output,r,world,horizon)) {r.trackingPathValidatedClearance=r.globalPathClearance();return true;}
    }
    if(!unchangedFailedSearch) {
        r.globalRouteSearchFailed=true;
        r.globalRouteFailureLane=failureLane;
        r.globalRouteFailureStaticCount=staticCount;
        r.globalRouteFailureStaticSignature=stationarySceneSignature(world);
        r.globalRouteFailureRoadS=world.egoRoadS;
        r.globalRouteRetryAfter=world.receiveTime+5.;
    }
    // Search failure is not path failure.  Keep the existing sampled route
    // so prepareCorridorLattice can see its blocker and atomically replace it
    // with a local avoidance path.  Clearing it here was the direct cause of
    // plan_valid=0 and the -4 m/s^2 stop in M11 frame 759.
    if(sliceAvailable) {
        output=std::move(current);
        traceDecision(trace,"corridor_global_route",
                      "blocked_route_retained_for_local_avoidance",
                      "complete_replan_unavailable=1");
        return true;
    }
    return false;
}

// A failed nearest-node chain is not proof that there is no drivable path.
// Search all nearby corridor lanes and construct a checked connection from
// the measured pose to their precomputed samples. This also handles landing
// on a raw sample whose forward link was replaced by a merge/split curve.
bool recoverTrackingPath(std::vector<Waypoint>& output,const CorridorRuntime& r,
                         const PathGeneratorData& map,const WorldState& world,
                         double horizon,DecisionTraceData* trace,
                         const CorridorReentryScope* scope=nullptr,
                         const MissionCorridorData* mission=nullptr) {
    using Choice=std::pair<double,std::int64_t>;
    const int gx=int(std::floor(world.ego.x/10.));
    const int gy=int(std::floor(world.ego.y/10.));
    const double c=std::cos(world.ego.heading),s=std::sin(world.ego.heading);
    const std::vector<double> recoveryLengths=scope
        ?std::vector<double>{6.,8.,10.,12.,14.,16.,20.,24.,28.,32.,35.}
        :std::vector<double>{12.,21.,30.};
    auto attempt=[&](bool physicalNetwork,std::size_t& tested) {
        std::map<std::pair<LaneKey,int>,Choice> choices;
        for(int dx=-4;dx<=4;++dx) for(int dy=-4;dy<=4;++dy) {
            const auto cell=r.spatial.find({gx+dx,gy+dy});
            if(cell==r.spatial.end()) continue;
            for(const auto id:cell->second) {
                if(physicalNetwork?!visible(r.network,id):!r.trackingAllowed[id]) continue;
                if(scope && (!r.allowed[id] || !std::isfinite(r.gatewayCost[id]) ||
                             r.gatewayLateralSteps[id]!=0)) continue;
                const auto& p=r.network.points[id];
                const double px=p.pose.x-world.ego.x,py=p.pose.y-world.ego.y;
                const double ahead=px*c+py*s;
                const double heading=std::abs(normalizeAngle(p.pose.yaw-world.ego.heading));
                const double reach=scope?std::hypot(px,py):ahead;
                // A missed turning connector can be beside the vehicle. Its
                // return endpoint need not be six metres along Ego's old
                // straight heading. Curvature and the complete body sweep,
                // rather than that projection, decide physical feasibility.
                if(ahead<(scope?0.:6.) || reach<6. || reach>35. ||
                   std::abs(-px*s+py*c)>(scope?25.:10.) ||
                   p.width<egoFootprint(world).width || heading>(scope?2.2:1.)) continue;
                for(int bucket=0;bucket<int(recoveryLengths.size());++bucket) {
                    const double score=std::abs(reach-recoveryLengths[bucket]);
                    const auto key=std::make_pair(keyOf(p.pose),bucket);
                    const auto old=choices.find(key);
                    if(old==choices.end() || score<old->second.first) choices[key]={score,id};
                }
            }
        }
        double best=inf;
        for(const auto& item:choices) {
            ++tested;const auto id=item.second.second;
            const auto& target=r.network.points[id];
            Waypoint start=target.pose;start.x=world.ego.x;start.y=world.ego.y;
            start.yaw=world.ego.heading;start.s=world.egoPathS;
            start.roadId=world.egoRoadId;start.sectionIndex=world.egoSectionIndex;
            start.laneId=world.egoLaneId;start.roadS=world.egoRoadS;
            std::vector<Waypoint> candidate;double maxK{};std::string reason;
            if(!curve(candidate,start,r.curvature,target,&maxK,1.,scope?2.2:1.4)) continue;
            Waypoint rejected;
            if(!surfaceAndMarks(r,map,candidate,world,reason,!physicalNetwork,&rejected,
                                scope?&scope->surface:nullptr)) {
                if(scope)traceDecision(trace,"corridor_relocalization","bridge_rejected",
                    "target="+std::to_string(target.pose.roadId)+":"+std::to_string(target.pose.laneId)+
                    " reason="+reason+" point="+std::to_string(rejected.x)+":"+std::to_string(rejected.y));
                continue;
            }
            const double completion=candidate.back().s;
            auto tail=forwardPath(r,id,horizon,completion,physicalNetwork);
            if(!smoothSourceJoins(tail,r,map,world,nullptr,!physicalNetwork)) continue;
            auto safety=candidate;
            for(const auto& p:tail) {if(p.s>completion+2.) break;append(safety,p);}
            if(!collisionFree(safety,world,PlanningBehaviorConfig{},r.laneConfig,&map)) continue;
            const double score=std::abs(-(target.pose.x-start.x)*s+(target.pose.y-start.y)*c)+
                               20.*maxK+(std::isfinite(r.gatewayCost[id])?0.:20.);
            if(score>=best) continue;
            for(const auto& p:tail) append(candidate,p);
            // The consumer needs a settled, usable return. A cheaper short
            // prefix in the same wrong entrance lane must not repeatedly
            // win here and then fail adoption, starving the actual return.
            if(scope && candidate.back().s-candidate.front().s<
                std::max(8.,r.laneConfig.gatewayReserveDistance)+2.) continue;
            if(scope && (!mission || !corridorReentryPreservesMission(candidate,r,*mission,world,*scope))) continue;
            if(scope && !corridorTrackingPathIsSafe(candidate,world,r.laneConfig.latticeObstacleClearance,
                                                   nullptr,nullptr,&map)) continue;
            output=std::move(candidate);best=score;
        }
        return std::isfinite(best);
    };
    std::size_t corridorTested=0;
    if(attempt(false,corridorTested)) {
        std::ostringstream detail;detail<<"tested="<<corridorTested<<" points="<<output.size();
        traceDecision(trace,"corridor_lattice","tracking_recovered_all_lanes",detail.str());
        return true;
    }
    // A missed fork is not permission to adopt an arbitrary XODR road. The
    // former second pass searched the whole physical network and was the
    // direct source of M12's 3405 -> 126 departure. Recovery may use every
    // lane in the active/next official corridor, but never a lane outside it.
    output.clear();
    traceDecision(trace,"corridor_lattice","tracking_recovery_exhausted",
                  "corridor_tested="+std::to_string(corridorTested)+
                  " outside_corridor_fallback=blocked");
    return false;
}
}

bool initializeCorridorRuntime(PathGeneratorData& map,const std::string& directory,
                              const LaneChangeConfig& config,std::string* error) {
    PrecomputedLaneNetwork network;
    if(!loadPrecomputedLaneNetwork(directory,network,error)) return false;
    return initializeCorridorRuntime(map,std::move(network),config,error);
}
bool initializeCorridorRuntime(PathGeneratorData& map,PrecomputedLaneNetwork network,
                              const LaneChangeConfig& config,std::string* error) {
    auto r=std::make_shared<CorridorRuntime>();
    r->network=std::move(network);
    r->laneConfig=config;
    r->lateralLength=config.latticeShortDistance;
    r->forward.resize(r->network.points.size());r->reverse.resize(r->network.points.size());
    r->globalLateral.resize(r->network.points.size());
    r->globalInLaneShift.resize(r->network.points.size());
    r->ordinaryRoadPoint.resize(r->network.points.size());
    r->tangent.resize(r->network.points.size());
    for(const auto& p:r->network.points) {
        if(!map.roads.count(p.pose.roadId)) {if(error)*error="precomputed road absent from active XODR";return false;}
        r->ordinaryRoadPoint[p.id]=map.roads.at(p.pose.roadId).junction<0;
        r->tangent[p.id]={std::cos(p.pose.yaw),std::sin(p.pose.yaw)};
        r->spatial[{int(std::floor(p.pose.x/10.)),int(std::floor(p.pose.y/10.))}].push_back(p.id);
        if(p.nextPoint>=0) addEdge(*r,p.id,{p.nextPoint,gap(p.pose,r->network.points[p.nextPoint].pose),-1,false});
    }
    for(const auto& c:r->network.connections) if(c.toPoint>=0 && c.gap<=.05 && c.status!="wrong_contact_direction")
        addEdge(*r,c.fromPoint,{c.toPoint,std::max(.0001,c.gap),-1,false});
    // Some source derivative knots deliberately have nextPoint=-1 although
    // their one-sided positions coincide. Preserve reachability through this
    // declared sample order; smooth and validate the actual geometry below.
    for(const auto& [key,lane]:r->network.lanes) for(std::size_t i=1;i<lane.pointIds.size();++i) {
        const auto a=lane.pointIds[i-1],b=lane.pointIds[i];
        const auto& p=r->network.points[a];
        const double d=gap(p.pose,r->network.points[b].pose);
        if(p.nextPoint<0 && d<=.05) addEdge(*r,a,{b,std::max(.0001,d),-1,false});
    }
    for(std::size_t i=0;i<r->network.transitions.size();++i) {
        const auto& t=r->network.transitions[i];
        if(t.vehicleFootprintValid&&!t.crossesSolid&&!t.violatesLaneChangePermission)
            addEdge(*r,t.fromPoint,{t.toPoint,t.points.back().s,int(i),false});
    }
    // Legal lateral reachability consumes real forward distance; it cannot
    // teleport across lanes at the last sample before a solid line/gateway.
    const auto successor=[&](std::int64_t id) -> std::int64_t {
        const auto next=r->network.points[id].nextPoint;
        if(visible(r->network,next)) return next;
        std::int64_t result=-1;
        for(const auto& e:r->forward[id]) if(!e.lateral && e.transition<0) {
            if(result>=0) return -1;
            result=e.to;
        }
        return result;
    };
    std::vector<double> lateralLengths{r->lateralLength,
        config.latticeMediumDistance,config.latticeLongDistance};
    std::sort(lateralLengths.begin(),lateralLengths.end());
    lateralLengths.erase(std::unique(lateralLengths.begin(),lateralLengths.end()),
                         lateralLengths.end());
    for(const auto& p:r->network.points) if(visible(r->network,p.id)) {
        for(int direction:{1,-1}) {
          auto lateralStart=p.id;
          std::set<std::int64_t> lateralSeen;
          int laneSteps=0;
          while(lateralSeen.insert(lateralStart).second) {
            const auto& sidePoint=r->network.points[lateralStart];
            const auto adjacent=direction<0?sidePoint.rightPoint:sidePoint.leftPoint;
            const bool changeAllowed=direction<0?sidePoint.rightChangeAllowed:sidePoint.leftChangeAllowed;
            if(!changeAllowed || !visible(r->network,adjacent)) break;
            lateralStart=adjacent;++laneSteps;
            auto id=lateralStart,source=p.id;double distance=0.,sourceDistance=0.;bool legal=true;
            for(const double desired:lateralLengths) {
              if(!std::isfinite(desired) || desired<=0.) continue;
              while(distance<desired) {
                const auto& target=r->network.points[id];
                if(!boundaryAllowed(map,r->network.points[source].pose,target.pose)) {legal=false;break;}
                const auto next=successor(id);if(next<0) {legal=false;break;}
                distance+=std::max(.0001,gap(target.pose,r->network.points[next].pose));id=next;
                while(sourceDistance+.25<distance) {
                    const auto nextSource=successor(source);if(nextSource<0) {legal=false;break;}
                    sourceDistance+=std::max(.0001,gap(r->network.points[source].pose,r->network.points[nextSource].pose));source=nextSource;
                }
                if(!legal) break;
              }
              if(!legal) break;
              const CorridorEdge edge{id,distance,-1,true};
              if(laneSteps==1 && desired==r->lateralLength) addEdge(*r,p.id,edge);
              else r->globalLateral[p.id].push_back(edge);
            }
          }
        }
    }
    // A junction connector often has no lateral neighbours of its own.
    // Waiting until its very last point to change to an outgoing lane loses
    // the physical apron exactly where a stopped truck may overhang it.
    // Add early connections only to adjacent lanes of this connector's own
    // longitudinal exit. This adds no other junction movement or road goal;
    // the whole generated body still has to pass surface/mark/collision tests.
    for(const auto& p:r->network.points) {
        if(!visible(r->network,p.id) || map.roads.at(p.pose.roadId).junction<0) continue;
        auto next=p.id;double travelled=0.;
        std::size_t horizon=0;
        std::set<std::int64_t> seen;
        while(horizon<lateralLengths.size() && seen.insert(next).second) {
            const auto following=successor(next);
            if(following<0) break;
            travelled+=std::max(.0001,gap(r->network.points[next].pose,
                                         r->network.points[following].pose));
            next=following;
            if(travelled<lateralLengths[horizon]) continue;
            ++horizon;
            const auto& exitPoint=r->network.points[next];
            if(exitPoint.pose.roadId==p.pose.roadId ||
               map.roads.at(exitPoint.pose.roadId).junction>=0) continue;
            for(int side:{-1,1}) {
                auto target=next;
                std::set<std::int64_t> neighbours;
                while(neighbours.insert(target).second) {
                    const auto& current=r->network.points[target];
                    const auto adjacent=side<0?current.rightPoint:current.leftPoint;
                    if(!visible(r->network,adjacent)) break;
                    const auto& endpoint=r->network.points[adjacent];
                    if(endpoint.pose.roadId!=exitPoint.pose.roadId ||
                       endpoint.pose.laneId*exitPoint.pose.laneId<=0 ||
                       !boundaryAllowed(map,current.pose,endpoint.pose)) break;
                    target=adjacent;
                    if(std::abs(normalizeAngle(endpoint.pose.yaw-p.pose.yaw))>1.4) continue;
                    r->globalLateral[p.id].push_back({target,travelled,-1,true});
                }
            }
        }
    }
    const std::array<double,4> shiftLengths{
        6.,8.,config.latticeShortDistance,config.latticeMediumDistance};
    for(const auto& p:r->network.points) if(visible(r->network,p.id)) {
        for(std::size_t i=0;i<shiftLengths.size();++i) {
            auto target=p.id;double travelled=0.;
            std::set<std::int64_t> seen;
            while(travelled<shiftLengths[i] && seen.insert(target).second) {
                const CorridorEdge* next=nullptr;
                for(const auto& edge:r->forward[target]) {
                    if(edge.lateral || edge.transition>=0 ||
                       !(keyOf(r->network.points[edge.to].pose)==keyOf(p.pose))) continue;
                    if(next) {next=nullptr;break;}
                    next=&edge;
                }
                if(!next) {target=-1;break;}
                travelled+=next->distance;target=next->to;
            }
            if(target>=0 && travelled>=shiftLengths[i]-.75)
                r->globalInLaneShift[p.id][i]={target,travelled,-1,true};
        }
    }
    // Snapshot once at startup, not on a control frame or at each replan.
    // The worker owns its graph/masks and cannot race activation/map matching.
    auto worker=std::make_shared<CorridorGlobalRouteWorker>();
    worker->runtime=std::make_shared<CorridorRuntime>(*r);
    auto frozenMap=std::make_shared<PathGeneratorData>(map);
    frozenMap->corridorRuntime.reset();
    worker->map=std::move(frozenMap);
    auto localWorker=std::make_shared<CorridorLocalRouteWorker>();
    localWorker->runtime=std::make_shared<CorridorRuntime>(*r);
    localWorker->map=worker->map;
    r->localRouteWorker=std::move(localWorker);
    r->globalRouteWorker=std::move(worker);
    map.corridorRuntime=std::move(r);return true;
}

bool corridorHasLongitudinalConnection(const PathGeneratorData& map,
                                       const LaneKey& from,
                                       const LaneKey& to) {
    if(from==to) return true;
    if(!map.corridorRuntime) return false;
    const auto& r=*map.corridorRuntime;
    const auto lane=r.network.lanes.find(from);
    if(lane==r.network.lanes.end()) return false;
    for(const auto id:lane->second.pointIds) {
        if(id<0 || id>=static_cast<std::int64_t>(r.forward.size())) continue;
        for(const auto& edge:r.forward[id]) {
            if(!edge.lateral && keyOf(r.network.points[edge.to].pose)==to)
                return true;
        }
    }
    return false;
}

bool connectToReferencePath(std::vector<Waypoint>& output,
                            const std::vector<Waypoint>& reference,
                            std::size_t referenceProgress,
                            CorridorRuntime& r,
                            const PathGeneratorData& map,
                            const WorldState& world,double horizon,
                            DecisionTraceData* trace,
                            bool corridorOnly=true,double clearance) {
    auto collisionConfig=r.laneConfig;
    if(clearance<0.) clearance=&reference==&r.committedGlobalPath
        ?r.globalPathClearance():r.laneConfig.latticeObstacleClearance;
    collisionConfig.latticeObstacleClearance=clearance;
    if(clearance<r.laneConfig.latticeObstacleClearance)
        collisionConfig.maximumLaneChangeSpeed=std::min(
            collisionConfig.maximumLaneChangeSpeed,collisionConfig.localFallbackSpeed);
    output.clear();
    if(reference.size()<2) return false;
    // A failed bridge to an unchanged, blocked global route used to consume
    // tens of milliseconds on every control tick. Cache FAILURE briefly;
    // successful recovery paths and their per-frame safety checks are separate.
    auto& failed=r.committedReconnectFailure;
    bool cacheable=r.budgetedSearch && corridorOnly &&
        &reference==&r.committedGlobalPath && !world.respawnDetected &&
        std::isfinite(world.receiveTime);
    std::vector<std::array<double,20>> observations;
    const auto footprint=egoFootprint(world);
    if(cacheable) {
        observations.reserve(world.objects.size());
        for(const auto& object:world.objects) {
            const auto& o=object.raw;
            std::array<double,20> row{{double(o.id),o.x,o.y,o.z,o.heading,
                o.speed,o.length,o.width,o.height,object.velocityX,
                object.velocityY,double(object.velocityValid)}};
            for(std::size_t i=0;i<object.obb.corners.size();++i) {
                row[12+2*i]=object.obb.corners[i].x;
                row[13+2*i]=object.obb.corners[i].y;
            }
            if(!std::all_of(row.begin(),row.end(),
                           [](double value){return std::isfinite(value);})) {
                cacheable=false;break;
            }
            observations.push_back(row);
        }
        if(cacheable) std::sort(observations.begin(),observations.end());
    }
    const LaneKey lane{world.egoRoadId,world.egoSectionIndex,world.egoLaneId};
    const double age=world.receiveTime-failed.failedAt;
    if(cacheable && failed.valid && age>=0. && age<.5 &&
       failed.leg==r.activeLeg && failed.routeRevision==r.routeRevision &&
       failed.requestRevision==r.globalRouteRequestRevision &&
       failed.progress==referenceProgress && failed.pointCount==reference.size() &&
       failed.lane==lane && failed.horizon==horizon &&
       failed.clearance==clearance &&
       failed.maneuverSpeed==collisionConfig.maximumLaneChangeSpeed &&
       std::hypot(world.ego.x-failed.x,world.ego.y-failed.y)<.25 &&
       std::abs(world.ego.z-failed.z)<.1 &&
       std::abs(normalizeAngle(world.ego.heading-failed.yaw))<.02 &&
       std::abs(world.egoSpeed-failed.speed)<.1 &&
       std::abs(r.curvature-failed.curvature)<1e-4 &&
       std::abs(footprint.length-failed.footprint.length)<1e-6 &&
       std::abs(footprint.width-failed.footprint.width)<1e-6 &&
       std::abs(footprint.centerOffset-failed.footprint.centerOffset)<1e-6 &&
       failed.observations==observations) {
        traceDecision(trace,"corridor_lattice","tracking_reconnect_retry_throttled",
                      "unchanged_failed_bridge=1 safety_checks_remain_active=1");
        return false;
    }
    if(&reference==&r.committedGlobalPath) failed.valid=false;
    const double c=std::cos(world.ego.heading);
    const double s=std::sin(world.ego.heading);
    double bestScore=inf;
    std::size_t tested=0;
    // Reference paths are sampled densely.  Testing every fourth point keeps
    // this bounded while still providing several C2 lengths in 6--35 m.
    const std::size_t begin=referenceProgress>80
        ? referenceProgress-80 : 1;
    const std::size_t end=std::min(
        reference.size(),referenceProgress+300);
    for(std::size_t i=std::max<std::size_t>(1,begin);i<end;i+=4) {
        const double dx=reference[i].x-world.ego.x;
        const double dy=reference[i].y-world.ego.y;
        const double ahead=dx*c+dy*s;
        const double lateral=-dx*s+dy*c;
        if(ahead<6. || ahead>50. || std::abs(lateral)>14. ||
           std::abs(normalizeAngle(
               reference[i].yaw-world.ego.heading))>0.8) continue;
        ++tested;
        Waypoint start=reference[i];
        start.x=world.ego.x;start.y=world.ego.y;
        start.yaw=world.ego.heading;start.s=world.egoPathS;
        start.roadId=world.egoRoadId;
        start.sectionIndex=world.egoSectionIndex;
        start.laneId=world.egoLaneId;start.roadS=world.egoRoadS;
        PrecomputedLanePoint target;
        target.pose=reference[i];
        if(i+1<reference.size()) {
            const double ds=gap(reference[i],reference[i+1]);
            if(ds>minimumTrackingSampleSpacing)
                target.curvature=normalizeAngle(
                    reference[i+1].yaw-reference[i].yaw)/ds;
        }
        std::vector<Waypoint> candidate;
        double maxK{};std::string reason;
        if(!curve(candidate,start,r.curvature,target,&maxK) ||
           !surfaceAndMarks(r,map,candidate,world,reason,corridorOnly))
            continue;
        for(std::size_t j=i+1;j<reference.size();++j) {
            append(candidate,reference[j]);
            if(candidate.back().s>=world.egoPathS+horizon) break;
        }
        if(candidate.size()<2 ||
           !collisionFree(candidate,world,PlanningBehaviorConfig{},
                          collisionConfig,&map)) continue;
        const double score=std::abs(lateral)+20.*maxK+.01*ahead;
        if(score>=bestScore) continue;
        output=std::move(candidate);bestScore=score;
    }
    if(output.size()<2) {
        if(cacheable) {
            failed.valid=true;failed.leg=r.activeLeg;
            failed.routeRevision=r.routeRevision;
            failed.requestRevision=r.globalRouteRequestRevision;
            failed.progress=referenceProgress;failed.pointCount=reference.size();
            failed.lane=lane;failed.x=world.ego.x;failed.y=world.ego.y;
            failed.z=world.ego.z;failed.yaw=world.ego.heading;
            failed.speed=world.egoSpeed;failed.curvature=r.curvature;
            failed.horizon=horizon;failed.failedAt=world.receiveTime;
            failed.clearance=clearance;
            failed.maneuverSpeed=collisionConfig.maximumLaneChangeSpeed;
            failed.footprint=footprint;failed.observations=std::move(observations);
        }
        return false;
    }
    std::ostringstream detail;
    detail<<"tested="<<tested<<" points="<<output.size()
          <<" rule=measured_pose_to_"
          <<(corridorOnly?"committed_gateway_route":"mission_reference");
    traceDecision(trace,"corridor_lattice",
                  "tracking_reconnected_to_gateway_route",detail.str());
    return true;
}

bool corridorCommittedRouteConnects(const PathGeneratorData& map,
                                    const LaneKey& from,
                                    const LaneKey& to) {
    if(from==to) return true;
    if(!map.corridorRuntime) return false;
    const auto& runtime=*map.corridorRuntime;
    if(!runtime.committedGlobalPathReachesGateway ||
       runtime.committedGlobalPath.size()<2) return false;
    LaneKey previous=keyOf(runtime.committedGlobalPath.front());
    for(std::size_t i=1;i<runtime.committedGlobalPath.size();++i) {
        const LaneKey current=keyOf(runtime.committedGlobalPath[i]);
        if(current==previous) continue;
        if((previous==from && current==to) ||
           (previous==to && current==from)) return true;
        previous=current;
    }
    return false;
}

void requestCorridorGlobalReplan(CorridorRuntime& runtime) {
    runtime.globalRouteReplanRequested = true;
    ++runtime.globalRouteRequestRevision;
    runtime.globalRouteSearchFailed = false;
    runtime.globalRouteRetryAfter = 0.;
}

namespace {
bool remainingAttachedPath(std::vector<Waypoint>& suffix,
                           const std::vector<Waypoint>& path,
                           const WorldState& world,bool requireAttached=true) {
    suffix.clear();
    if(path.size()<2) return false;
    std::size_t nearest=0;double best=inf;
    for(std::size_t i=0;i<path.size();++i) {
        if(std::abs(normalizeAngle(path[i].yaw-world.ego.heading))>.8) continue;
        const double distance=std::hypot(path[i].x-world.ego.x,path[i].y-world.ego.y);
        if(distance<best) {best=distance;nearest=i;}
    }
    if(!std::isfinite(best) || (requireAttached && best>1.25) || nearest+1>=path.size()) return false;
    suffix.assign(path.begin()+nearest,path.end());
    return true;
}

}

bool corridorTrackingPathIsSafe(const std::vector<Waypoint>& path,const WorldState& world,
                               double margin,int* blockerId,
                               const PlanningBehaviorConfig* behaviorConfig,
                               const PathGeneratorData* map) {
    if(blockerId) *blockerId=-1;
    std::vector<Waypoint> suffix;
    if(!remainingAttachedPath(suffix,path,world)) return false;
    if(!stationaryPathClear(suffix,world,margin,0,blockerId,map)) return false;
    PathSafetyResult safety;resetPathSafety(safety);
    checkCurrentCollisions(safety,world.egoObb,world.objects,-1,&world,map);
    if(safety.collision) return false;
    const PlanningBehaviorConfig defaults;
    const auto& behavior=behaviorConfig?*behaviorConfig:defaults;
    if(std::any_of(world.objects.begin(),world.objects.end(),
                   [](const auto& o){return std::abs(o.raw.speed)>=.3;})) {
        WorldState moving=world;
        moving.egoSpeed=std::max({1.,std::abs(world.egoSpeed),
                                 LaneChangeConfig{}.maximumLaneChangeSpeed});
        moving.objects.erase(std::remove_if(moving.objects.begin(),moving.objects.end(),
            [](const auto& o){return std::abs(o.raw.speed)<.3;}),moving.objects.end());
        LocalPathData local;local.valid=true;local.points=std::move(suffix);
        checkPredictedCollisions(safety,moving,local,behavior.collisionPredictionHorizon,
                                behavior.collisionPredictionStep,true,map);
    }
    return !safety.predictedCollision;
}

double corridorStationaryFreeDistance(const std::vector<Waypoint>& path,const WorldState& world,
                                     double margin,const PathGeneratorData* map,int* blockerId) {
    if(blockerId) *blockerId=-1;
    std::vector<Waypoint> suffix;
    if(!remainingAttachedPath(suffix,path,world,false)) return 0.;
    Waypoint current=suffix.front();current.x=world.ego.x;current.y=world.ego.y;
    current.yaw=world.ego.heading;current.roadId=world.egoRoadId;
    current.sectionIndex=world.egoSectionIndex;current.laneId=world.egoLaneId;current.roadS=world.egoRoadS;
    for(const auto& object:world.objects) {
        if(!isStationaryRouteObstacle(object) || !intersects(world.egoObb,object.obb) ||
           (map && !corridorObstacleAtPathLevel(*map,current,object))) continue;
        if(blockerId) *blockerId=static_cast<int>(object.raw.id);
        return 0.;
    }
    // Match the envelope used to approve the generated stationary-clear
    // path. Turning away changes the PATH heading, not a parked object's
    // motion: the optional cross-traffic reserve would otherwise add 1.5m
    // here alone and stop a route already certified with the requested
    // clearance. Actual body contact above and the chosen margin remain;
    // dynamic collision prediction / emergency braking are separate.
    return staticClearance(suffix,world,std::max(0.,margin),blockerId,map,false);
}

namespace {
using LocalClock=std::chrono::steady_clock;

bool localTrajectorySafe(const std::vector<Waypoint>& path,const WorldState& world,
                         const PathGeneratorData& map,const LaneChangeConfig& config,
                         const PlanningBehaviorConfig& behavior,double margin) {
    if(path.size()<2 || !stationaryPathClear(path,world,margin,0,nullptr,&map)) return false;
    PathSafetyResult safety;resetPathSafety(safety);
    checkCurrentCollisions(safety,world.egoObb,world.objects,-1,&world,&map);
    if(safety.collision) return false;
    WorldState moving=world;
    moving.objects.erase(std::remove_if(moving.objects.begin(),moving.objects.end(),
        [](const auto& o){return isStationaryRouteObstacle(o);}),moving.objects.end());
    if(!moving.objects.empty()) {
        // Worker checks the possible open-passage speed, not the former
        // universal 5 km/h cap. The command thread also rechecks current
        // observations at the curvature/clearance-selected manoeuvre speed.
        const double speed=margin+1e-6>=config.latticeObstacleClearance
            ?std::min(config.localFallbackOpenSpeed,config.maximumLaneChangeSpeed)
            :std::min(config.localFallbackSpeed,config.localFallbackOpenSpeed);
        moving.egoSpeed=std::max({.5,std::abs(world.egoSpeed),speed});
        LocalPathData local;local.valid=true;local.points=path;
        checkPredictedCollisions(safety,moving,local,behavior.collisionPredictionHorizon,
                                behavior.collisionPredictionStep,true,&map);
    }
    return !safety.predictedCollision;
}

// This search owns a separate startup snapshot and never modifies a global
// goal, route, margin, revision or failure flag. Only its bounded local mask
// and local centre/offset states are searched.
std::shared_ptr<CorridorReentryScope> makeReentryScope(
    const CorridorRuntime& r,const PathGeneratorData& map,const WorldState& world,
    bool outsideCorridor,bool relocalizing=false) {
    auto scope=std::make_shared<CorridorReentryScope>();
    scope->outsideCorridor=outsideCorridor;
    scope->allowed=r.globalSearchAllowed;
    scope->surface=r.surfaceAllowed;
    scope->preview.resize(r.allowed.size(),false);
    for(std::size_t id=0;id<r.allowed.size();++id) {
        scope->allowed[id]=scope->allowed[id] || r.allowed[id];
        scope->preview[id]=scope->allowed[id] && !r.allowed[id];
    }
    if(!outsideCorridor) return scope;
    // This is a local surface/bridge permission, not a change to the route
    // graph, gateway costs or mission targets. Only same-direction pavement
    // within roughly one lane of an official lane can enter the local graph.
    for(const auto& point:r.network.points) {
        if(scope->allowed[point.id] || !visible(r.network,point.id) ||
           std::hypot(point.pose.x-world.ego.x,point.pose.y-world.ego.y)>120.) continue;
        // A displaced car can already be on a connector which diverges more
        // than one lane from the corridor. Admit only a short origin-lane
        // apron; the endpoint/whole return certificate still requires the
        // official gateway. Never add that lane to global mission authority.
        if(relocalizing && point.pose.roadId==world.egoRoadId &&
           point.pose.sectionIndex==world.egoSectionIndex &&
           point.pose.laneId*world.egoLaneId>0 &&
           std::abs(point.pose.laneId-world.egoLaneId)<=1) {
            const double dx=point.pose.x-world.ego.x,dy=point.pose.y-world.ego.y;
            const double ahead=dx*std::cos(world.ego.heading)+dy*std::sin(world.ego.heading);
            if(ahead>=-egoFootprint(world).length && ahead<=40. && std::hypot(dx,dy)<=45. &&
               std::abs(normalizeAngle(point.pose.yaw-world.ego.heading))<.8) {
                scope->allowed[point.id]=scope->surface[point.id]=true;
                continue;
            }
        }
        const int gx=int(std::floor(point.pose.x/10.)),gy=int(std::floor(point.pose.y/10.));
        bool permitted=false;
        for(int dx=-1;dx<=1 && !permitted;++dx) for(int dy=-1;dy<=1 && !permitted;++dy) {
            const auto cell=r.spatial.find({gx+dx,gy+dy});if(cell==r.spatial.end()) continue;
            for(const auto id:cell->second) {
                if(!r.allowed[id] && !scope->preview[id]) continue;
                const auto& official=r.network.points[id];
                if(gap(point.pose,official.pose)>4.0 ||
                   std::abs(normalizeAngle(point.pose.yaw-official.pose.yaw))>.35) continue;
                double a{},b{};
                if(sampleRoadElevation(a,map,point.pose.roadId,point.pose.roadS) &&
                   sampleRoadElevation(b,map,official.pose.roadId,official.pose.roadS) &&
                   std::abs(a-b)>1.) continue;
                if(!boundaryAllowed(map,point.pose,official.pose)) continue;
                permitted=true;break;
            }
        }
        if(permitted) scope->allowed[point.id]=scope->surface[point.id]=true;
    }
    return scope;
}

} // namespace

bool corridorReentryPreservesMission(const std::vector<Waypoint>& path,const CorridorRuntime& r,
    const MissionCorridorData& mission,const WorldState& world,
    const CorridorReentryScope& scope) {
    if(path.size()<2 || scope.preview.size()!=r.allowed.size()) return false;
    const auto* leg=activeMissionLeg(mission);if(!leg) return false;
    // Geometric recovery may temporarily leave the corridor, but must end on
    // an official centreline with a gateway-reaching directed suffix.
    const auto end=containing(r,path.back().x,path.back().y,path.back().yaw);
    if(end<0 || (!std::isfinite(r.gatewayCost[end]) && !scope.preview[end]) ||
       gap(path.back(),r.network.points[end].pose)>.75) return false;
    double outside=0.;bool visitedGateway=leg->gatewayLanes.empty();
    bool visitedTarget=false, crossedTarget=false;
    corridor_detail::SurfaceQuery surfaceQuery(r);
    for(std::size_t i=0;i<path.size();++i) {
        const auto& p=path[i];
        if(leg->gatewayLanes.count(keyOf(p))) visitedGateway=true;
        if(missionPointIsTarget(*leg,p)) visitedTarget=true;
        const auto station=leg->targetRoadS.find(keyOf(p));
        if(station!=leg->targetRoadS.end() && std::abs(p.roadS-station->second)<=.75)
            crossedTarget=true;
        if(i && surfaceQuery.containing(p.x,p.y,std::cos(p.yaw),std::sin(p.yaw))<0)
            outside+=gap(path[i-1],p);
    }
    if(outside>40.) return false;
    Waypoint start=path.front();start.roadId=world.egoRoadId;
    start.sectionIndex=world.egoSectionIndex;start.laneId=world.egoLaneId;start.roadS=world.egoRoadS;
    const bool startedPastTarget=missionPointIsTarget(*leg,start);
    if(!startedPastTarget && (scope.preview[end] || missionPointIsTarget(*leg,path.back())) &&
       !leg->targetRoadS.empty() && !crossedTarget) return false;
    if(scope.preview[end] && !startedPastTarget && (!visitedTarget || !visitedGateway)) return false;
    if(leg->targetLanes.count(keyOf(path.back())) &&
       !leg->targetLanes.count(keyOf(start)) && !visitedGateway) return false;
    return true;
}

namespace {

bool searchLocalCorridor(std::vector<Waypoint>& output,CorridorRuntime& r,
    const PathGeneratorData& map,const WorldState& world,const MissionCorridorData& mission,
    const LaneChangeConfig& config,const PlanningBehaviorConfig& behavior,
    double clearance,double minimumDistance,LocalClock::time_point deadline,
    std::size_t& expanded,bool& timedOut,DecisionTraceData* trace,
    const CorridorReentryScope* scope=nullptr) {
    output.clear();
    auto start=nearest(r,world);
    const auto& allowed=scope?scope->allowed:r.allowed;
    const auto* surface=scope?&scope->surface:nullptr;
    corridor_detail::SurfaceQuery surfaceQuery(r,true,surface);
    if(scope) {
        double best=inf;
        for(const auto& point:r.network.points) {
            if(!allowed[point.id]) continue;
            const double d=std::hypot(point.pose.x-world.ego.x,point.pose.y-world.ego.y);
            const double yaw=std::abs(normalizeAngle(point.pose.yaw-world.ego.heading));
            if(d>4. || yaw>.8) continue;
            const bool same=keyOf(point.pose)==LaneKey{world.egoRoadId,world.egoSectionIndex,world.egoLaneId};
            const double score=d+2.*yaw+(same?0.:2.);
            if(score<best) {best=score;start=point.id;}
        }
    }
    if(start<0 || !allowed[start]) return false;
    auto searchDeadline=deadline;
    const auto expired=[&](){return LocalClock::now()>=searchDeadline || expanded>=12000;};
    using Item=std::pair<double,std::int64_t>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> frontier;
    std::vector<double> along(r.network.points.size(),inf);
    std::vector<std::int64_t> nodes;
    along[start]=0.;frontier.push({0.,start});
    const double horizon=std::clamp(config.localFallbackHorizon+(scope?30.:0.),15.,120.);
    std::map<std::int64_t,std::vector<CorridorEdge>> reconnects;
    const auto bridges=[&](std::int64_t id)->const std::vector<CorridorEdge>& {
        auto [entry,inserted]=reconnects.emplace(id,std::vector<CorridorEdge>{});
        if(!inserted || !scope || scope->preview[id] ||
           (r.allowed[id] && r.gatewayLateralSteps[id]==0)) return entry->second;
        const auto& from=r.network.points[id].pose;
        const int gx=int(std::floor(from.x/10.)),gy=int(std::floor(from.y/10.));
        std::map<std::pair<LaneKey,int>,std::pair<double,std::int64_t>> choices;
        for(int dx=-4;dx<=4;++dx) for(int dy=-4;dy<=4;++dy) {
            const auto cell=r.spatial.find({gx+dx,gy+dy});if(cell==r.spatial.end()) continue;
            for(const auto to:cell->second) {
                if((!r.allowed[to] && !scope->preview[to]) || !visible(r.network,to)) continue;
                const auto& p=r.network.points[to].pose;
                // Ordinary same-road lateral edges already cover this move.
                // These extra curves exist only for overlapping connector
                // roads that have no direct lateral edge in the XODR graph.
                if(p.roadId==from.roadId && p.sectionIndex==from.sectionIndex) continue;
                const double px=p.x-from.x,py=p.y-from.y;
                const double along=px*std::cos(from.yaw)+py*std::sin(from.yaw);
                const double lateral=-px*std::sin(from.yaw)+py*std::cos(from.yaw);
                if(along<6. || along>35. || std::abs(lateral)>4. ||
                   std::abs(normalizeAngle(p.yaw-from.yaw))>.6) continue;
                const std::array<double,6> lengths{{6.,8.,12.,16.,24.,32.}};
                for(int bucket=0;bucket<int(lengths.size());++bucket) {
                    const double score=std::abs(along-lengths[bucket]);
                    const auto key=std::make_pair(keyOf(p),bucket);
                    const auto old=choices.find(key);
                    if(old==choices.end() || score<old->second.first) choices[key]={score,to};
                }
            }
        }
        for(const auto& item:choices) entry->second.push_back({item.second.second,
            gap(from,r.network.points[item.second.second].pose),-1,true});
        return entry->second;
    };
    while(!frontier.empty()) {
        if(expired()) {timedOut=true;return false;}
        const auto [distance,id]=frontier.top();frontier.pop();
        if(distance>along[id]+1e-8) continue;
        nodes.push_back(id);
        const auto visit=[&](const CorridorEdge& edge) {
            if(edge.to<0 || !allowed[edge.to] || (!scope && !std::isfinite(r.gatewayCost[edge.to]))) return;
            const double next=distance+std::max(.0001,edge.distance);
            if(next<=horizon && next+1e-8<along[edge.to]) {
                along[edge.to]=next;frontier.push({next,edge.to});
            }
        };
        for(const auto& edge:r.forward[id]) visit(edge);
        for(const auto& edge:r.globalLateral[id]) visit(edge);
        for(const auto& edge:bridges(id)) visit(edge);
    }
    if(nodes.empty() || minimumDistance>horizon) return false;
    std::vector<int> localIndex(r.network.points.size(),-1);
    for(std::size_t i=0;i<nodes.size();++i) localIndex[nodes[i]]=static_cast<int>(i);
    // Local gap states may straddle a legally changeable shared boundary.
    // Limiting every state to one lane's inset centre leaves real narrow
    // channels between parked vehicles unrepresentable even with a clear
    // swept footprint. Global states and their margins remain unchanged.
    constexpr int offsets=5,centre=2;
    const auto state=[&](std::int64_t point,int offset){return localIndex[point]*offsets+offset;};
    const auto point=[&](int id){return nodes[std::size_t(id/offsets)];};
    const int firstState=state(start,centre);
    const auto body=egoFootprint(world);
    // Join nearby blockers into one local task instead of completing the
    // maneuver immediately behind the first parked car. Distances follow
    // the directed corridor graph, so bends and road-id boundaries do not
    // truncate the group. A group beyond the bounded horizon is rolled
    // forward later; it is never declared cleared by this local result.
    std::vector<std::pair<double,double>> obstacleSpans;
    for(const auto& object:world.objects) {
        if(!isStationaryRouteObstacle(object)) continue;
        double best=inf,station=0.,longitudinalHalf=0.;
        for(const auto id:nodes) {
            const auto& p=r.network.points[id];
            const double dx=object.raw.x-p.pose.x,dy=object.raw.y-p.pose.y;
            const double d2=dx*dx+dy*dy;
            if(d2>=best || !corridorObstacleAtPathLevel(map,p.pose,object)) continue;
            const double angle=normalizeAngle(object.raw.heading-p.pose.yaw);
            const double halfAlong=.5*(std::abs(std::cos(angle))*object.raw.length+
                                       std::abs(std::sin(angle))*object.raw.width);
            const double halfAcross=.5*(std::abs(std::sin(angle))*object.raw.length+
                                        std::abs(std::cos(angle))*object.raw.width);
            const double across=-dx*std::sin(p.pose.yaw)+dy*std::cos(p.pose.yaw);
            if(std::abs(across)>p.width/2.+halfAcross) continue;
            best=d2;station=along[id]+dx*std::cos(p.pose.yaw)+dy*std::sin(p.pose.yaw);
            longitudinalHalf=halfAlong;
        }
        if(std::isfinite(best) && station+longitudinalHalf>=0.)
            obstacleSpans.push_back({station-longitudinalHalf,station+longitudinalHalf});
    }
    std::sort(obstacleSpans.begin(),obstacleSpans.end());
    const double tailReserve=std::max(8.,config.gatewayReserveDistance);
    double clusterEnd=std::max(0.,minimumDistance-tailReserve-body.rearExtent()-clearance);
    for(const auto& span:obstacleSpans) {
        if(span.first>clusterEnd+std::max(12.,2.*body.length)) break;
        if(span.second+body.rearExtent()+clearance+tailReserve>horizon) break;
        clusterEnd=std::max(clusterEnd,span.second);
    }
    minimumDistance=std::max(minimumDistance,clusterEnd+body.rearExtent()+clearance+tailReserve);
    const auto offsetAllowed=[&](int id) {
        const int slot=id%offsets;
        if(slot!=0 && slot!=offsets-1) return true;
        const auto& p=r.network.points[point(id)];
        const auto adjacent=slot==0?p.rightPoint:p.leftPoint;
        const bool permitted=slot==0?p.rightChangeAllowed:p.leftChangeAllowed;
        return permitted && adjacent>=0 && allowed[adjacent] &&
            (scope || std::isfinite(r.gatewayCost[adjacent]));
    };
    const auto pose=[&](int id) {
        auto p=r.network.points[point(id)].pose;
        const double room=std::max(0.,std::min(config.maximumLateralOffset,
            r.network.points[point(id)].width/2.-body.width/2.-config.minimumLaneEdgeClearance));
        const int slot=id%offsets;
        const double offset=slot==0?-r.network.points[point(id)].width/2.:
            slot==offsets-1?r.network.points[point(id)].width/2.:(slot-centre)*room;
        p.x-=offset*std::sin(p.yaw);p.y+=offset*std::cos(p.yaw);
        if(id==firstState) {p.x=world.ego.x;p.y=world.ego.y;p.yaw=world.ego.heading;p.roadS=world.egoRoadS;}
        return p;
    };
    const std::size_t count=nodes.size()*offsets;
    std::vector<signed char> blocked(count,-1);
    const auto occupied=[&](int id) {
        auto& result=blocked[id];if(result>=0) return result!=0;
        if(!offsetAllowed(id)) {result=1;return true;}
        const auto p=pose(id);const auto box=vehicleFootprintAt(body,{p.x,p.y,p.yaw},clearance);
        result=0;
        for(const auto& object:world.objects)
            if(isStationaryRouteObstacle(object) && intersects(box,object.obb) &&
               corridorObstacleAtPathLevel(map,p,object)) {result=1;break;}
        return result!=0;
    };
    struct Parent {int from{-1};CorridorEdge edge;bool generated{};};
    std::map<std::pair<int,int>,bool> checked;
    std::set<std::pair<int,int>> forbidden;
    const auto geometry=[&](std::vector<Waypoint>& path,int from,int to,
                             const CorridorEdge& edge,bool generated) {
        path.clear();
        if(generated) {
            auto target=r.network.points[edge.to];target.pose=pose(to);
            double maximum{};
            return curve(path,pose(from),from==firstState?r.curvature:r.network.points[point(from)].curvature,target,&maximum);
        }
        if(edge.transition>=0) path=corridorEdgePath(r,point(from),edge);
        else {append(path,pose(from));append(path,pose(to));}
        return path.size()>=2;
    };
    std::size_t geometryRejected=0;
    // Prefer a complete, checked lane-centre solution, not a continuously
    // changing centre cost. Reuse graph/edge checks in the unrestricted pass;
    // a failed centre probe must not remove narrow-gap states or gateway exits.
    // A long rolling horizon is not an obstacle: with an empty forward scene
    // it must not postpone settling by seeding clusterEnd from minimumDistance.
    // Try a route-coherent settled tail first. This is a candidate preference,
    // not a new legality rule: the original unrestricted search remains last.
    const double obstacleClearedAt=obstacleSpans.empty()?0.:
        clusterEnd+body.rearExtent()+clearance;
    // Edges end at the next sampled road point, not at exactly 12.000m.
    // Match the graph's 0.75m sampling tolerance, otherwise every nominal
    // 12m lane-centre attachment can be rejected at 12.xm mid-manoeuvre.
    const double preferredSettleEnd=obstacleClearedAt+12.+.75;
    const auto search=[&](bool centresOnly,bool settledPreference) {
    for(int attempt=0;attempt<(scope?16:8);++attempt) {
        if(expired()) {timedOut=true;return false;}
        std::vector<double> distance(count,inf);
        std::vector<Parent> parent(count);
        std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
        distance[firstState]=0.;queue.push({minimumDistance,firstState});
        int reached=-1;
        const auto heuristic=[&](int id){return std::max(0.,minimumDistance-along[point(id)]);};
        while(!queue.empty()) {
            if(expired()) {timedOut=true;return false;}
            const auto [priority,raw]=queue.top();queue.pop();const int current=static_cast<int>(raw);
            if(priority>distance[current]+heuristic(current)+1e-8) continue;
            ++expanded;const auto id=point(current);
            double settledTail=0.;
            if(current%offsets==centre && along[id]>=minimumDistance) {
                for(int at=current;at!=firstState && parent[at].from>=0;) {
                    const auto& step=parent[at];
                    if(step.generated || step.edge.lateral || step.edge.transition>=0 ||
                       at%offsets!=centre || step.from%offsets!=centre) break;
                    settledTail+=step.edge.distance;at=step.from;
                    if(settledTail>=std::max(8.,config.gatewayReserveDistance)) break;
                }
            }
            if(current%offsets==centre && along[id]>=minimumDistance-(scope?tailReserve:0.) &&
               (scope || settledTail>=std::max(8.,config.gatewayReserveDistance)) &&
               (std::isfinite(r.gatewayCost[id]) || (scope && scope->preview[id])) && !occupied(current) &&
               (r.ordinaryRoadPoint[id] || missionPointIsTarget(*activeMissionLeg(mission),pose(current)))) {
                reached=current;break;
            }
            const auto relax=[&](const CorridorEdge& edge,int targetOffset,bool generated) {
                if(centresOnly && targetOffset!=centre) return;
                if(edge.to<0 || localIndex[edge.to]<0 || expired()) return;
                if(settledPreference) {
                    // Give a physically clear offset up to 12m to settle. A
                    // narrow passage or a required gateway alignment can still
                    // use every original sample in the unrestricted fallback.
                    if(along[edge.to]>=preferredSettleEnd && targetOffset!=centre)
                        return;
                    const auto& fromPose=r.network.points[id].pose;
                    const auto& toPose=r.network.points[edge.to].pose;
                    const auto actualFrom=pose(current);
                    const double fromOffset=std::abs(
                        -(actualFrom.x-fromPose.x)*std::sin(fromPose.yaw)+
                         (actualFrom.y-fromPose.y)*std::cos(fromPose.yaw));
                    // A centre endpoint 20m away is still a 20m off-centre
                    // return curve. Limit the return itself, not only its
                    // discrete endpoint's offset slot, in this preferred pass.
                    if(generated && fromOffset>.05 &&
                       along[edge.to]>preferredSettleEnd) return;
                    const bool sameRoad=fromPose.roadId==toPose.roadId &&
                        fromPose.sectionIndex==toPose.sectionIndex;
                    const bool changesLane=sameRoad && fromPose.laneId!=toPose.laneId;
                    const bool gatewayAlignment=r.gatewayLateralSteps[edge.to]<r.gatewayLateralSteps[id];
                    if(along[id]>=obstacleClearedAt && changesLane && !gatewayAlignment)
                        return;
                }
                const int next=state(edge.to,targetOffset);
                if(forbidden.count({current,next}) || occupied(next)) return;
                const auto a=pose(current),b=pose(next);
                const double lateral=std::abs(-(b.x-a.x)*std::sin(a.yaw)+(b.y-a.y)*std::cos(a.yaw));
                const double cost=distance[current]+std::max(.0001,edge.distance)+
                    (edge.lateral?4.:generated?.2:0.)+
                    (generated?config.latticeCurvatureCost*5.77*lateral/std::max(9.,edge.distance*edge.distance):0.);
                if(cost+1e-8>=distance[next]) return;
                const auto key=std::make_pair(current,next);
                auto old=checked.find(key);
                if(old==checked.end()) {
                    std::vector<Waypoint> path;
                    bool valid=geometry(path,current,next,edge,generated);
                    if(valid && generated) {
                        for(const double fraction:{.2,.5,.8}) {
                            const auto& p=path[std::size_t(fraction*(path.size()-1))];
                            const auto box=vehicleFootprintAt(body,{p.x,p.y,p.yaw});
                            const double c=std::cos(p.yaw),s=std::sin(p.yaw);
                            if(surfaceQuery.containing(p.x,p.y,c,s)<0) {valid=false;break;}
                            for(const auto& corner:box.corners)
                                if(surfaceQuery.containing(corner.x,corner.y,c,s)<0) {valid=false;break;}
                            if(!valid) break;
                        }
                    }
                    if(valid) valid=stationaryPathClear(path,world,clearance,0,nullptr,&map);
                    old=checked.emplace(key,valid).first;
                }
                if(!old->second) return;
                distance[next]=cost;parent[next]={current,edge,generated};
                queue.push({cost+heuristic(next),next});
            };
            for(const auto& edge:r.forward[id]) {
                if(edge.lateral) for(int o=0;o<offsets;++o) relax(edge,o,true);
                else if(edge.transition>=0) {if(current%offsets==centre) relax(edge,centre,false);}
                else {
                    const bool attach=current==firstState &&
                        (gap(pose(current),r.network.points[id].pose)>.15 ||
                         std::abs(normalizeAngle(world.ego.heading-r.network.points[id].pose.yaw))>.07);
                    relax(edge,current%offsets,attach);
                }
            }
            for(const auto& edge:r.globalLateral[id]) for(int o=0;o<offsets;++o) relax(edge,o,true);
            for(const auto& edge:bridges(id)) for(int o=0;o<offsets;++o) relax(edge,o,true);
            for(const auto& edge:r.globalInLaneShift[id])
                for(int o=0;o<offsets;++o)
                    if(o!=current%offsets || current==firstState) relax(edge,o,true);
        }
        if(reached<0) break;
        std::vector<int> order;
        for(int at=reached;at!=firstState;at=parent[at].from) {
            if(at<0 || parent[at].from<0) return false;
            order.push_back(at);
        }
        std::reverse(order.begin(),order.end());
        bool valid=true;output.clear();
        for(const auto next:order) {
            if(expired()) {timedOut=true;output.clear();return false;}
            const auto& step=parent[next];std::vector<Waypoint> part;std::string reason;
            if(!geometry(part,step.from,next,step.edge,step.generated) ||
               !surfaceAndMarks(r,map,part,world,reason,true,nullptr,surface) ||
               !stationaryPathClear(part,world,clearance,0,nullptr,&map)) {
                forbidden.insert({step.from,next});++geometryRejected;valid=false;break;
            }
            for(const auto& p:part) append(output,p);
        }
        if(!valid) {output.clear();continue;}
        if(scope) {
            // Certify the settled tail as geometry after choosing the return
            // node. Requiring it in a point/offset-only A* parent chain loses
            // feasible paths when the cheapest parent keeps delaying rejoin.
            const double joinedAt=output.back().s;
            const auto tail=forwardPath(r,point(reached),tailReserve,joinedAt);
            if(tail.empty() || tail.back().s-joinedAt<tailReserve-.5) {
                forbidden.insert({parent[reached].from,reached});output.clear();continue;
            }
            for(const auto& p:tail) append(output,p);
        }
        repairTrackingTangents(output);
        std::string reason;
        if(expired()) {timedOut=true;output.clear();return false;}
        // The expanded surface has already been checked for every selected
        // edge. Recheck the assembled path and its return certificate rather
        // than handing a surface-only permission to the ordinary tracker.
        bool geometryOkay=scope
            ? surfaceAndMarks(r,map,output,world,reason,true,nullptr,surface) &&
                corridorReentryPreservesMission(output,r,mission,world,*scope)
            : smoothSourceJoins(output,r,map,world,&reason);
        if(scope && !geometryOkay) traceDecision(trace,"corridor_local_fallback","return_certificate_rejected",
            "surface_reason="+reason+" end="+std::to_string(output.back().x)+":"+std::to_string(output.back().y));
        if(scope && geometryOkay) for(std::size_t i=1;i<output.size();++i) {
            const double ds=gap(output[i-1],output[i]);
            if(ds<1e-6 || ds>.75 || std::abs(normalizeAngle(output[i].yaw-output[i-1].yaw))/ds>
               maximumCurvature+.002) {
                geometryOkay=false;
                traceDecision(trace,"corridor_local_fallback","return_join_rejected",
                    "ds="+std::to_string(ds)+" yaw_step="+std::to_string(normalizeAngle(output[i].yaw-output[i-1].yaw)));
                break;
            }
        }
        if(geometryOkay &&
           localTrajectorySafe(output,world,map,config,behavior,clearance)) {
            traceDecision(trace,"corridor_local_fallback","local_route_built",
                "points="+std::to_string(output.size())+" clearance="+std::to_string(clearance)+
                " expanded="+std::to_string(expanded)+" geometry_rejected="+std::to_string(geometryRejected)+
                " sampling="+std::string(centresOnly?"lane_centres":"full_offsets")+
                " settled_tail_preference="+std::to_string(settledPreference)+
                " endpoint_gateway_reachable=1 claims_gateway_reached=0 tier="+
                std::to_string(scope?(scope->outsideCorridor?2:1):0));
            return true;
        }
        if(scope && !geometryOkay) {
            // A rejected return/goal is one candidate, not proof that every
            // earlier return point also misses the gate. Try another bridge.
            int rejected=reached;
            for(auto it=order.rbegin();it!=order.rend();++it)
                if(parent[*it].generated) {rejected=*it;break;}
            forbidden.insert({parent[rejected].from,rejected});
            ++geometryRejected;output.clear();continue;
        }
        output.clear();return false;
    }
    output.clear();return false;
    };
    const auto probeStarted=LocalClock::now();
    // Spend at most a quarter of the remaining search time (8 ms maximum).
    // The nominal/tight clearance budgets and the command-thread work do not
    // increase. Geometry checks already made here remain useful to full A*.
    searchDeadline=std::min(deadline,probeStarted+std::min(
        deadline>probeStarted?(deadline-probeStarted)/4:LocalClock::duration::zero(),
        std::chrono::duration_cast<LocalClock::duration>(std::chrono::milliseconds(8))));
    if(search(true,true)) return true;
    traceDecision(trace,"corridor_local_fallback","centre_probe_fallback",
                  "full_offset_samples_retained=1");
    // Preserve dense offset sampling before the complete blocker group, while
    // preferring an ordinary centred continuation afterwards. Both probes run
    // inside the existing worker deadline; no synchronous replanning is added.
    const auto settleProbeStarted=LocalClock::now();
    searchDeadline=std::min(deadline,settleProbeStarted+std::min(
        deadline>settleProbeStarted?(deadline-settleProbeStarted)/4:LocalClock::duration::zero(),
        std::chrono::duration_cast<LocalClock::duration>(std::chrono::milliseconds(8))));
    timedOut=false;
    forbidden.clear();
    if(search(false,true)) return true;
    searchDeadline=deadline;
    timedOut=false;
    forbidden.clear(); // assembled-path failures can depend on the parent chain
    return search(false,false);
}

bool attachLocalResult(std::vector<Waypoint>& path,const std::vector<Waypoint>& candidate,
    CorridorRuntime& r,const PathGeneratorData& map,const WorldState& world,
    const LaneChangeConfig& config,const PlanningBehaviorConfig& behavior,double margin,
    const MissionCorridorData& mission,const CorridorReentryScope* scope=nullptr) {
    if(candidate.size()<2) return false;
    std::size_t nearestIndex=0;double best=inf;
    for(std::size_t i=0;i<candidate.size();++i) {
        if(std::abs(normalizeAngle(candidate[i].yaw-world.ego.heading))>.25) continue;
        const double d=std::hypot(candidate[i].x-world.ego.x,candidate[i].y-world.ego.y);
        if(d<best) {best=d;nearestIndex=i;}
    }
    if(best>.75 || nearestIndex+1>=candidate.size() ||
       candidate.back().s-candidate[nearestIndex].s<std::max(8.,config.gatewayReserveDistance)+2.) return false;
    path.assign(candidate.begin()+nearestIndex,candidate.end());
    if(best>.001 || std::abs(normalizeAngle(path.front().yaw-world.ego.heading))>.001) {
        // At most three short C2 probes; never map/graph recovery on the
        // command thread. Unattachable asynchronous results are discarded.
        bool joined=false;const auto deadline=LocalClock::now()+std::chrono::milliseconds(4);
        Waypoint actual=path.front();actual.x=world.ego.x;actual.y=world.ego.y;actual.yaw=world.ego.heading;
        for(const double reach:{3.,5.,8.}) {
            if(LocalClock::now()>=deadline) break;
            const auto end=std::lower_bound(path.begin(),path.end(),path.front().s+reach,
                [](const auto& p,double s){return p.s<s;});
            if(end==path.end()) continue;
            PrecomputedLanePoint target;target.pose=*end;
            const auto index=static_cast<std::size_t>(end-path.begin());
            if(index>0 && index+1<path.size()) target.curvature=normalizeAngle(path[index+1].yaw-path[index-1].yaw)/
                std::max(.1,gap(path[index-1],path[index+1]));
            std::vector<Waypoint> bridge;double maximum{};std::string reason;
            if(!curve(bridge,actual,r.curvature,target,&maximum) ||
               !surfaceAndMarks(r,map,bridge,world,reason,true,nullptr,scope?&scope->surface:nullptr)) continue;
            for(auto it=end+1;it!=path.end();++it) append(bridge,*it);
            if(!localTrajectorySafe(bridge,world,map,config,behavior,margin)) continue;
            path=std::move(bridge);joined=true;break;
        }
        if(!joined) return false;
    }
    if(!localTrajectorySafe(path,world,map,config,behavior,margin)) return false;
    if(scope) {
        // The unchanged suffix was swept in the worker; any newly attached
        // prefix was swept above with the same permission. Do not repeat that
        // full-map body sweep on the real-time command thread.
        if(!corridorReentryPreservesMission(path,r,mission,world,*scope)) return false;
    }
    double station=world.egoPathS;
    for(std::size_t i=0;i<path.size();++i) {if(i) station+=gap(path[i-1],path[i]);path[i].s=station;}
    return true;
}
} // namespace

namespace {
struct RecoveryBounds {
    double minX{inf}, maxX{-inf}, minY{inf}, maxY{-inf};
};

RecoveryBounds recoveryBounds(const Obb2D& box) {
    RecoveryBounds bounds;
    for (const auto& p : box.corners) {
        bounds.minX = std::min(bounds.minX, p.x);
        bounds.maxX = std::max(bounds.maxX, p.x);
        bounds.minY = std::min(bounds.minY, p.y);
        bounds.maxY = std::max(bounds.maxY, p.y);
    }
    return bounds;
}

bool recoveryBoundsOverlap(const RecoveryBounds& a, const RecoveryBounds& b) {
    // Strict separation only: touching/overlapping bounds still use the
    // original SAT computation. Keep near-boundary floating-point cases in
    // SAT too; this padding is only on the cheap rejection test.
    constexpr double roundoffGuard = 1e-6;
    return !(a.maxX + roundoffGuard < b.minX || b.maxX + roundoffGuard < a.minX ||
             a.maxY + roundoffGuard < b.minY || b.maxY + roundoffGuard < a.minY);
}

// SAT minimum translation depth, including containment. Zero denotes no
// overlap. Recovery prices contact; ordinary collision checks remain binary.
double recoveryOverlapDepth(const Obb2D& a, const Obb2D& b) {
    double depth = inf;
    for (const auto* box : {&a, &b}) {
        for (std::size_t i = 0; i < 2; ++i) {
            const auto& p = box->corners[i];
            const auto& q = box->corners[i + 1];
            const double length = std::hypot(q.x - p.x, q.y - p.y);
            if (length < 1e-6) return 0.;
            const double nx = -(q.y - p.y) / length, ny = (q.x - p.x) / length;
            double amin = inf, amax = -inf, bmin = inf, bmax = -inf;
            for (const auto& corner : a.corners) {
                const double s = corner.x * nx + corner.y * ny;
                amin = std::min(amin, s); amax = std::max(amax, s);
            }
            for (const auto& corner : b.corners) {
                const double s = corner.x * nx + corner.y * ny;
                bmin = std::min(bmin, s); bmax = std::max(bmax, s);
            }
            if (amax <= bmin || bmax <= amin) return 0.;
            depth = std::min(depth, std::min(amax - bmin, bmax - amin));
        }
    }
    return std::isfinite(depth) ? depth : 0.;
}
}

bool prepareCorridorProgressRecovery(
    PathPlannerData& output, bool& contactExpected, std::size_t& searchCursor,
    const WorldState& world, const PathGeneratorData& map,
    const MissionCorridorData& mission, const LaneChangeConfig& laneConfig,
    const PlanningBehaviorConfig& behavior,
    const std::vector<Waypoint>& reference,
    const std::vector<int>& confirmedStationaryObjectIds, DecisionTraceData* trace,
    double failedHeadingChange, bool* retainedReference) {
    if (retainedReference) *retainedReference = false;
    if (behavior.simulationProgressRecovery != 1. || !map.corridorRuntime ||
        !world.inputValid || world.respawnDetected || !activeMissionLeg(mission)) return false;
    auto& r = *map.corridorRuntime;
    activate(r, mission);
    const auto startId = nearest(r, world);
    if (startId < 0) return false;
    const auto body = egoFootprint(world);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(behavior.progressRecoverySearchBudgetMs));
    Waypoint start = r.network.points[startId].pose;
    start.x = world.ego.x; start.y = world.ego.y; start.yaw = world.ego.heading;
    start.s = world.egoPathValid ? world.egoPathS : 0.;
    std::vector<Waypoint> best;
    double bestCost = inf;
    bool bestContact = false;
    bool bestTerminalContact = false, bestIsIncumbent = false;
    std::size_t checked = 0;
    struct RecoveryObject {
        const WorldObjectState* object;
        bool moving, protectedActor;
        double vx, vy;
        RecoveryBounds bounds;
        Obb2D horizonBox;
        RecoveryBounds horizonBounds;
    };
    std::vector<RecoveryObject> objects;
    objects.reserve(world.objects.size());
    for (const auto& object : world.objects) {
        const bool confirmed = std::find(confirmedStationaryObjectIds.begin(),
            confirmedStationaryObjectIds.end(), static_cast<int>(object.raw.id)) !=
            confirmedStationaryObjectIds.end();
        const bool moving = std::abs(object.raw.speed) >= .3;
        const double vx = moving ? objectVelocityX(object) : 0.;
        const double vy = moving ? objectVelocityY(object) : 0.;
        const auto horizonBox = moving ? makeObb2D(
            object.raw.x + vx * behavior.collisionPredictionHorizon,
            object.raw.y + vy * behavior.collisionPredictionHorizon,
            object.raw.heading, object.raw.length, object.raw.width) : object.obb;
        // Immutable for this search invocation only; no stale-scene cache.
        const bool baselineVulnerable = !behavior.changedDefaultModel &&
            object.raw.length <= 2.5 && object.raw.width <= 1.5 && object.raw.height >= 1.;
        objects.push_back({&object, moving, moving || !confirmed || baselineVulnerable, vx, vy,
            recoveryBounds(object.obb), horizonBox, recoveryBounds(horizonBox)});
    }
    const auto consider = [&](std::vector<Waypoint> path, const char* source, bool incumbent = false) {
        if (path.size() < 3 || gap(path.front(), start) > .4 ||
            std::abs(normalizeAngle(path.front().yaw - start.yaw)) > .2) return;
        double length = 0., maxK = 0.;
        path.front().s = start.s;
        for (std::size_t i = 1; i < path.size(); ++i) {
            const double ds = gap(path[i - 1], path[i]);
            if (!std::isfinite(ds) || ds > .75 || ds < 1e-6) return;
            const double k = std::abs(normalizeAngle(path[i].yaw - path[i - 1].yaw)) / ds;
            if (!std::isfinite(k) || k > maximumCurvature + .002) return;
            maxK = std::max(maxK, k); length += ds;
            path[i].s = start.s + length;
            if (length >= 40.) { path.resize(i + 1); break; }
        }
        if (length < 8.) return;
        if (std::isfinite(failedHeadingChange)) {
            const auto preview = std::lower_bound(path.begin(), path.end(), start.s + 3.,
                [](const auto& p, double station) { return p.s < station; });
            if (preview != path.end() && std::abs(normalizeAngle(
                    preview->yaw - start.yaw - failedHeadingChange)) < .06) return;
        }
        std::string reason;
        // May leave the mission corridor to use another physical driving
        // lane, but never manufacture a path across missing road geometry.
        if (!surfaceAndMarks(r, map, path, world, reason, false)) {
            traceCandidatePath(trace, keyOf(path.back()), path, "rejected", reason);
            return;
        }
        double contactLength = 0., depthIntegral = 0., maximumDepth = 0., lastContactDistance = 0.;
        bool terminalContact = false;
        for (std::size_t i = 0; i < path.size(); ++i) {
            const auto& p = path[i];
            const auto box = vehicleFootprintAt(body, {p.x, p.y, p.yaw});
            const auto bounds = recoveryBounds(box);
            double depth = 0.;
            const double time = std::min(behavior.collisionPredictionHorizon,
                (p.s - start.s) / std::max(.2, behavior.progressRecoverySpeed));
            for (const auto& cached : objects) {
                const auto& object = *cached.object;
                if (!corridorObstacleAtPathLevel(map, p, object)) continue;
                // Keep current occupancy as well as the short-term moving
                // prediction: a stopped vehicle can start during the escape.
                double objectDepth = recoveryBoundsOverlap(bounds, cached.bounds)
                    ? recoveryOverlapDepth(box, object.obb) : 0.;
                if (cached.moving) {
                    if (time == behavior.collisionPredictionHorizon) {
                        if (recoveryBoundsOverlap(bounds, cached.horizonBounds))
                            objectDepth = std::max(objectDepth,
                                recoveryOverlapDepth(box, cached.horizonBox));
                    } else {
                        const auto predicted = makeObb2D(
                            object.raw.x + cached.vx * time,
                            object.raw.y + cached.vy * time,
                            object.raw.heading, object.raw.length, object.raw.width);
                        if (recoveryBoundsOverlap(bounds, recoveryBounds(predicted)))
                            objectDepth = std::max(objectDepth, recoveryOverlapDepth(box, predicted));
                    }
                }
                // Contact is negotiable only for confirmed static obstacles.
                // A recently stopped actor retains dynamic protection until
                // the ordinary stationary-confirmation timer has expired.
                if (objectDepth > 0. && cached.protectedActor) return;
                depth = std::max(depth, objectDepth);
            }
            const double ds = i ? p.s - path[i - 1].s : .2;
            if (depth > 0.) {
                contactLength += ds; depthIntegral += ds * depth;
                maximumDepth = std::max(maximumDepth, depth);
                lastContactDistance = p.s - start.s;
                if (path.back().s - p.s < 2.) terminalContact = true;
            }
        }
        const auto endId = containing(r, path.back().x, path.back().y, path.back().yaw, false);
        const bool reachable = endId >= 0 && std::isfinite(r.gatewayCost[endId]);
        // Recovery retains the required exit even before a queue approach
        // exists. A timeout must not authorize an unreachable straight road.
        if(!activeMissionLeg(mission)->gatewayLanes.empty() &&
           !r.activeGatewayObserved && !reachable) return;
        // Clear exits dominate paths that finish inside a body. Among paths
        // that escape, minimize penetration and contact distance. An initial
        // overlap is not a rejection, so driving out of it remains possible.
        // Contact/terminal classes are compared separately below. Keeping a
        // huge fixed terminal penalty here would also inflate the 5% switch
        // threshold and hide substantial improvements in penetration depth.
        const double cost = 1000. * maximumDepth + 100. * depthIntegral + 100. * contactLength +
            25. * lastContactDistance +
            (reachable ? 0. : 20.) + 5. * maxK + 2. / length;
        ++checked;
        if (!best.empty() && !recovery_selection::better(
                {contactLength > 0., terminalContact, cost},
                {bestContact, bestTerminalContact, bestCost}, bestIsIncumbent)) return;
        bestCost = cost; bestContact = contactLength > 0.; best = std::move(path);
        bestTerminalContact = terminalContact; bestIsIncumbent = incumbent;
        traceCandidatePath(trace, keyOf(best.back()), best, "viable", source);
    };

    // Revalidate and rescore the remaining incumbent FIRST in today's scene.
    // This retains the best discovered path across budgeted searches without
    // reusing stale collision costs. Similar alternatives do not cause weaving.
    std::vector<Waypoint> suffix;
    if (remainingAttachedPath(suffix, reference, world))
        consider(std::move(suffix), "simulation_contact_cost_stored_path", true);

    struct Endpoint { std::int64_t id; double offset; };
    std::vector<Endpoint> specs;
    for (const auto& distances : std::array<std::array<double, 3>, 3>{{
            {{6., 8., 10.}}, {{12., 16., 20.}}, {{24., 30., 40.}}}}) {
        for (const auto& entry : targets(r, startId, distances, body.width)) {
            const double room = std::max(0., r.network.points[entry.second].width / 2. -
                body.width / 2. - laneConfig.minimumLaneEdgeClearance);
            for (const double offset : {0., -room, room}) specs.push_back({entry.second, offset});
        }
    }
    // Short steering primitives cover a gap between sampled endpoints. They
    // ramp curvature from the measured value and undergo the same footprint
    // and curvature checks as map-connected quintics.
    constexpr std::array<double, 7> bends{{0., -.33, .33, -.67, .67, -1., 1.}};
    const std::size_t total = specs.size() + bends.size();
    std::array<std::vector<std::size_t>, 3> families;
    const auto familyOf = [](double lateral) -> std::size_t {
        return lateral < -.25 ? 0 : lateral > .25 ? 2 : 1;
    };
    for (std::size_t i = 0; i < specs.size(); ++i) {
        auto p = r.network.points[specs[i].id].pose;
        p.x -= specs[i].offset * std::sin(p.yaw);
        p.y += specs[i].offset * std::cos(p.yaw);
        const double lateral = -(p.x-start.x)*std::sin(start.yaw) +
                                 (p.y-start.y)*std::cos(start.yaw);
        families[familyOf(lateral)].push_back(i);
    }
    // Insert a steering primitive near the front of each family as well, so
    // map-endpoint ordering cannot starve the non-map escape shapes.
    for (std::size_t i = 0; i < bends.size(); ++i) {
        auto& family = families[familyOf(bends[i])];
        const auto position = std::min<std::size_t>(1 + i / 3, family.size());
        family.insert(family.begin() + position, specs.size() + i);
    }
    const auto order = recovery_selection::balancedOrder(families);
    std::array<std::size_t, 3> attempted{};
    std::size_t searched = 0;
    for (std::size_t count = 0; count < total; ++count) {
        const std::size_t index = order[searchCursor++ % total];
        ++searched;
        for (std::size_t family = 0; family < families.size(); ++family)
            if (std::find(families[family].begin(), families[family].end(), index) != families[family].end())
                ++attempted[family];
        std::vector<Waypoint> path;
        if (index < specs.size()) {
            const auto spec = specs[index];
            auto target = r.network.points[spec.id];
            target.pose.x -= spec.offset * std::sin(target.pose.yaw);
            target.pose.y += spec.offset * std::cos(target.pose.yaw);
            double maxK{};
            if (curve(path, start, r.curvature, target, &maxK)) {
                const auto tail = forwardPath(r, spec.id, 12., path.back().s);
                for (auto p : tail) {
                    p.x -= spec.offset * std::sin(p.yaw);
                    p.y += spec.offset * std::cos(p.yaw);
                    append(path, p);
                }
                consider(std::move(path), "simulation_contact_cost_quintic");
            }
        } else {
            Waypoint p = start;
            append(path, p);
            const double targetK = bends[index - specs.size()] * maximumCurvature;
            for (int step = 1; step <= 80; ++step) {
                const double u = std::min(1., (step - .5) * .2 / 3.);
                const double k = (1. - u) * std::clamp(r.curvature, -maximumCurvature, maximumCurvature) + u * targetK;
                const double midYaw = p.yaw + .1 * k;
                p.x += .2 * std::cos(midYaw); p.y += .2 * std::sin(midYaw);
                p.yaw = normalizeAngle(p.yaw + .2 * k);
                append(path, p);
            }
            consider(std::move(path), "simulation_contact_cost_steering_arc");
        }
        // A directional round is the smallest budget unit (at most three
        // candidates). Budget exhaustion must not select just the first side.
        if (searched % 3 == 0 && std::chrono::steady_clock::now() >= deadline) break;
    }
    traceDecision(trace, "progress_recovery", "balanced_search",
        "right=" + std::to_string(attempted[0]) + " straight=" + std::to_string(attempted[1]) +
        " left=" + std::to_string(attempted[2]) + " attempted=" + std::to_string(searched) +
        " total=" + std::to_string(total) + " incumbent_retained=" + std::to_string(bestIsIncumbent));
    if (best.empty()) {
        traceDecision(trace, "progress_recovery", "no_drivable_geometry",
                      "retry_cursor=" + std::to_string(searchCursor));
        return false;
    }
    output.waypoints = std::move(best); output.nearestIndex = 0;
    if (retainedReference) *retainedReference = bestIsIncumbent;
    contactExpected = bestContact;
    traceCandidatePath(trace, keyOf(output.waypoints.back()), output.waypoints,
                       "selected", "simulation_progress_recovery_contact_allowed");
    traceDecision(trace, "progress_recovery", "path_selected",
        "contact_expected=" + std::to_string(bestContact) +
        " cost=" + std::to_string(bestCost) + " checked=" + std::to_string(checked));
    return true;
}

void prepareSimulationCreepFallback(PathPlannerData& output, const WorldState& world,
    const PathGeneratorData& map, const MissionCorridorData& mission,
    const std::vector<Waypoint>& reference, DecisionTraceData* trace) {
    std::vector<Waypoint> guide;
    double initialCurvature = 0.;
    if (map.corridorRuntime && activeMissionLeg(mission)) {
        auto& r = *map.corridorRuntime;
        activate(r, mission);
        auto id = nearest(r, world);
        if (id < 0 || !std::isfinite(r.gatewayCost[id])) {
            double best = inf;
            for (const auto& point : r.network.points) {
                if (!r.allowed[point.id] || !std::isfinite(r.gatewayCost[point.id])) continue;
                const double cost = std::hypot(point.pose.x-world.ego.x, point.pose.y-world.ego.y) +
                    3. * std::abs(normalizeAngle(point.pose.yaw-world.ego.heading));
                if (cost < best) { best = cost; id = point.id; }
            }
        }
        if (id >= 0) guide = forwardPath(r, id, 30., 0.);
        if (std::isfinite(r.curvature)) initialCurvature = std::clamp(
            r.curvature, -maximumCurvature, maximumCurvature);
    }
    if (guide.size() < 2) guide = reference;
    double targetCurvature = 0.;
    Waypoint start;
    start.roadId = world.egoRoadId; start.sectionIndex = world.egoSectionIndex;
    start.laneId = world.egoLaneId; start.roadS = world.egoRoadS;
    if (!guide.empty()) {
        const auto near = std::min_element(guide.begin(), guide.end(), [&](const auto& a, const auto& b) {
            return std::hypot(a.x-world.ego.x,a.y-world.ego.y) <
                   std::hypot(b.x-world.ego.x,b.y-world.ego.y);
        });
        const double preview = std::max(6., std::hypot(near->x-world.ego.x,near->y-world.ego.y));
        auto aim = near;
        double distance = 0.;
        while (aim+1 != guide.end() && distance < preview) {
            distance += gap(*aim, *(aim+1)); ++aim;
        }
        const double dx = aim->x-world.ego.x, dy = aim->y-world.ego.y;
        const double forward = dx*std::cos(world.ego.heading)+dy*std::sin(world.ego.heading);
        if (forward > .1) {
            const double lateral = -dx*std::sin(world.ego.heading)+dy*std::cos(world.ego.heading);
            targetCurvature = std::clamp(2.*lateral/std::max(1.,dx*dx+dy*dy),
                                         -maximumCurvature, maximumCurvature);
            start = *near;
        }
    }
    start.x=world.ego.x; start.y=world.ego.y; start.yaw=world.ego.heading; start.s=0.;
    output.waypoints.clear(); output.nearestIndex=0;
    output.config.minLookahead=std::min(2.,output.config.minLookahead);
    auto p=start; append(output.waypoints,p);
    for (int step=1; step<=80; ++step) {
        const double blend=std::min(1.,(step-.5)*.2/3.);
        const double k=(1.-blend)*initialCurvature+blend*targetCurvature;
        const double middle=p.yaw+.1*k;
        p.x+=.2*std::cos(middle); p.y+=.2*std::sin(middle);
        p.yaw=normalizeAngle(p.yaw+.2*k); append(output.waypoints,p);
    }
    traceDecision(trace,"progress_recovery","unchecked_creep_geometry",
        "all_checked_candidates_failed=1 curvature="+std::to_string(targetCurvature)+
        " collision_clearance_certified=0 surface_certified=0");
}

bool prepareLocalCorridorFallback(LaneChangeData& replacement,const LaneChangeConfig& config,
    const PlanningBehaviorConfig& safety,const WorldState& world,const PathGeneratorData& map,
    const MissionCorridorData& mission,const std::vector<Waypoint>& reference,DecisionTraceData* trace) {
    if(!map.corridorRuntime || !mission.valid || !world.inputValid || !world.egoLaneValid ||
       (!world.egoPathValid && !map.corridorRuntime->localFallbackRelocalizationRequested))
        return false;
    auto& r=*map.corridorRuntime;
    if(!r.localRouteWorker || !activeMissionLeg(mission)) return false;
    activate(r,mission);
    if(r.localFallbackLeg!=mission.activeLegIndex || world.respawnDetected) {
        r.localFallbackLeg=mission.activeLegIndex;++r.localFallbackEpoch;
        r.localFallbackRetryAt={};r.localFallbackIssued=false;
        r.localFallbackSearchTier=0;r.localFallbackSearchBlocker=-1;
    }
    auto& worker=*r.localRouteWorker;
    const auto now=LocalClock::now();
    const auto signature=stationarySceneSignature(world);
    const double activation=std::min(std::max(5.,config.localFallbackHorizon-12.),
        std::max(config.localFallbackActivationDistance,std::abs(world.egoSpeed)*config.localFallbackTimeHeadway));
    // Continuation is requested only for an ACTIVE local manoeuvre. A past
    // local result plus a short global reference does not activate fallback.
    const bool rolling=r.localFallbackContinuationRequested;
    int blocker=-1;
    bool referenceUsable=false;
    const auto nearbyFreeDistance=[&](const std::vector<Waypoint>& source) {
        std::vector<Waypoint> nearby;
        if(!remainingAttachedPath(nearby,source,world)) return inf;
        referenceUsable=nearby.size()>=2 &&
            nearby.back().s-nearby.front().s>=8.;
        const auto end=std::upper_bound(nearby.begin(),nearby.end(),nearby.front().s+activation+1.,
            [](double station,const auto& p){return station<p.s;});
        nearby.erase(end,nearby.end());
        return corridorStationaryFreeDistance(nearby,world,config.latticeObstacleClearance,&map,&blocker);
    };
    double freeDistance=nearbyFreeDistance(reference);
    // A detached/exhausted reference is not evidence that the road ahead is
    // clear. Scan a physical continuation solely to trigger checked recovery;
    // it is never adopted as a route to an arbitrary successor. Infinite
    // free distance on a usable detour means CLEAR, not missing: scanning its
    // old lane would rediscover the obstacle that the detour already avoids.
    // A safe queue approach intentionally ends before its blocker. Looking
    // past that endpoint on the old physical lane launches an unrelated
    // local bypass while the global worker is already checking passage.
    // Actual obstacles on the selected path and relocalization remain active.
    if(!referenceUsable && !r.soleGatewayApproachSelected && !std::isfinite(freeDistance)) {
        const auto physical=containing(r,world.ego.x,world.ego.y,world.ego.heading,false);
        if(physical>=0) freeDistance=nearbyFreeDistance(
            forwardPath(r,physical,activation+1.,world.egoPathS,true));
    }
    // Judge the reference actually being selected, not a stale stored global
    // path or global failure flags. A blocked reference needs local help even
    // if the global producer has mistakenly kept its approved flag set.
    const bool relocalizing=r.localFallbackRelocalizationRequested;
    const bool obstacleNeeded=blocker>=0 && freeDistance<=activation;
    const bool needed=obstacleNeeded || rolling || relocalizing;
    if(needed && blocker!=r.localFallbackSearchBlocker) {
        r.localFallbackSearchBlocker=blocker;r.localFallbackSearchTier=0;
    }
    r.localFallbackEngaged=needed;
    if(worker.pending.valid() && worker.pending.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        auto result=worker.pending.get();r.localFallbackPending=false;
        for(const auto& detail:result.diagnostics)
            traceDecision(trace,"corridor_local_fallback","worker_detail",detail);
        traceDecision(trace,"corridor_local_fallback","worker_completed",
            "success="+std::to_string(result.success)+" ms="+std::to_string(result.elapsedMs)+
            " expanded="+std::to_string(result.expanded)+" timeout="+std::to_string(result.timedOut)+
            " clearance="+std::to_string(result.clearance)+" tier="+std::to_string(result.searchTier));
        std::vector<Waypoint> attached;
        if(needed && !world.respawnDetected && result.success && result.leg==mission.activeLegIndex &&
           result.epoch==r.localFallbackEpoch &&
           attachLocalResult(attached,result.path,r,map,world,config,safety,result.clearance,
                             mission,result.reentryScope.get())) {
            const auto& end=attached.back();
            replacement={};replacement.phase=LaneChangePhase::Prepare;
            replacement.routeMode=LaneChangeRouteMode::KeepTargetLane;
            replacement.localCorridorAvoidance=true;
            replacement.obstacleId=result.blocker;
            replacement.sourceRoadId=world.egoRoadId;replacement.sourceSectionIndex=world.egoSectionIndex;
            replacement.sourceLaneId=world.egoLaneId;replacement.targetLaneId=end.laneId;
            replacement.reconnectLane=keyOf(end);replacement.reconnectLaneValid=true;
            replacement.shiftCompletedLane=keyOf(end);replacement.shiftCompletedLaneValid=true;
            replacement.startRouteS=world.egoPathS;
            replacement.rejoinRouteS=std::max(world.egoPathS+2.,end.s-std::max(8.,config.gatewayReserveDistance));
            replacement.validatedObstacleClearance=result.clearance;
            replacement.gatewayReplanRequestedAfterShift=false;
            const auto& direction=attached[std::min<std::size_t>(20,attached.size()-1)];
            const double lateral=-(direction.x-world.ego.x)*std::sin(world.ego.heading)+
                                  (direction.y-world.ego.y)*std::cos(world.ego.heading);
            replacement.turnSignal=std::abs(lateral)<.2?0U:lateral>0.?1U:2U;
            replacement.outwardTurnSignal=replacement.turnSignal;
            replacement.path=std::move(attached);r.localFallbackIssued=true;
            r.localFallbackSearchTier=0;
            r.localFallbackRetryAt=now+std::chrono::duration_cast<LocalClock::duration>(
                std::chrono::duration<double>(config.localFallbackRetryInterval));
            traceCandidatePath(trace,replacement.reconnectLane,replacement.path,"selected","bounded_local_corridor_multistep");
            traceDecision(trace,"corridor_local_fallback","replacement_ready",
                "current_pose_and_all_obstacles_checked=1 global_route_unchanged=1 points="+
                std::to_string(replacement.path.size()));
            return true;
        }
        if(!result.success && result.leg==mission.activeLegIndex &&
           result.epoch==r.localFallbackEpoch && result.blocker==blocker)
            r.localFallbackSearchTier=std::min(2,result.searchTier+1);
        r.localFallbackRetryAt=now+std::chrono::duration_cast<LocalClock::duration>(
            std::chrono::duration<double>(config.localFallbackRetryInterval));
        traceDecision(trace,"corridor_local_fallback",result.success?"stale_or_unattached_result":"no_local_route",
                      "keep_current_path_and_safety_checks=1");
    }
    r.localFallbackPending=worker.pending.valid();
    if(!needed || world.respawnDetected || r.localFallbackPending) return false;
    const bool changed=signature!=r.localFallbackSignature ||
        std::hypot(world.ego.x-r.localFallbackLastX,world.ego.y-r.localFallbackLastY)>2.;
    if(!changed && now<r.localFallbackRetryAt) return false;
    r.localFallbackSignature=signature;r.localFallbackLastX=world.ego.x;r.localFallbackLastY=world.ego.y;
    const auto snapshot=worker.runtime;
    const auto mapSnapshot=worker.map;
    const auto epoch=r.localFallbackEpoch;
    const double initialCurvature=r.curvature;
    const int searchTier=r.localFallbackSearchTier;
    double obstacleLength=0.;
    for(const auto& object:world.objects) if(static_cast<int>(object.raw.id)==blocker) obstacleLength=object.raw.length;
    const double minimumDistance=(relocalizing && !obstacleNeeded)?25.:rolling?std::max(20.,config.localFallbackHorizon*.6):
        std::max(15.,std::max(0.,freeDistance)+obstacleLength+egoFootprint(world).length+std::max(8.,config.gatewayReserveDistance));
    // A horizon renewal is not permission to redraw a safe committed return.
    // Keep its centre-bound suffix and extend only its actual longitudinal
    // successor first. New hazards still reject the complete extended path.
    const auto continuationReference=rolling && !obstacleNeeded && !relocalizing
        ?reference:std::vector<Waypoint>{};
    try {
        worker.pending=std::async(std::launch::async,[snapshot,mapSnapshot,world,mission,config,safety,
            signature,epoch,blocker,minimumDistance,initialCurvature,searchTier,relocalizing,continuationReference]() {
            CorridorLocalRouteWorker::Result result;result.leg=mission.activeLegIndex;
            result.epoch=epoch;result.signature=signature;result.receiveTime=world.receiveTime;result.blocker=blocker;
            result.searchTier=searchTier;
            const auto started=LocalClock::now();
            const double budget=searchTier>0?config.localReentrySearchBudgetMs:config.localFallbackSearchBudgetMs;
            const auto deadline=started+std::chrono::duration_cast<LocalClock::duration>(
                std::chrono::duration<double,std::milli>(std::max(1.,budget)));
            DecisionTraceData diagnostics;diagnostics.enabled=true;
            try {
                activate(*snapshot,mission);
                snapshot->curvature=initialCurvature;
                if(searchTier>0) result.reentryScope=makeReentryScope(
                    *snapshot,*mapSnapshot,world,searchTier>1,relocalizing);
                const auto nominalDeadline=started+std::chrono::duration_cast<LocalClock::duration>(
                    std::chrono::duration<double,std::milli>(std::max(1.,budget*.4)));
                const auto extendCommittedReturn=[&]() {
                    if(continuationReference.empty() || result.reentryScope || LocalClock::now()>=deadline)
                        return false;
                    std::vector<Waypoint> candidate;
                    if(!remainingAttachedPath(candidate,continuationReference,world) ||
                       candidate.back().s-candidate.front().s<8.) return false;
                    // Match adoption's tighter attachment envelope. A stale
                    // but loosely projected reference must fall through to A*
                    // here, not repeatedly produce an unadoptable extension.
                    if(std::hypot(candidate.front().x-world.ego.x,candidate.front().y-world.ego.y)>.75 ||
                       std::abs(normalizeAngle(candidate.front().yaw-world.ego.heading))>.25) return false;
                    const auto end=candidate.back();
                    const auto id=containing(*snapshot,end.x,end.y,end.yaw);
                    if(id<0 || !snapshot->allowed[id] || !std::isfinite(snapshot->gatewayCost[id])) return false;
                    const auto& centre=snapshot->network.points[id].pose;
                    const double lateral=-(end.x-centre.x)*std::sin(centre.yaw)+
                                          (end.y-centre.y)*std::cos(centre.yaw);
                    if(std::abs(lateral)>.05 || std::abs(normalizeAngle(end.yaw-centre.yaw))>.1)
                        return false;
                    const auto tail=forwardPath(*snapshot,id,
                        std::max(8.,minimumDistance-(end.s-candidate.front().s)+1.),end.s);
                    for(std::size_t i=1;i<tail.size();++i) append(candidate,tail[i]);
                    if(candidate.back().s-candidate.front().s<minimumDistance) return false;
                    const auto goal=containing(*snapshot,candidate.back().x,candidate.back().y,candidate.back().yaw);
                    if(goal<0 || !snapshot->allowed[goal] || !std::isfinite(snapshot->gatewayCost[goal])) return false;
                    std::string reason;
                    if(!smoothSourceJoins(candidate,*snapshot,*mapSnapshot,world,&reason) ||
                       !surfaceAndMarks(*snapshot,*mapSnapshot,candidate,world,reason,true) ||
                       !localTrajectorySafe(candidate,world,*mapSnapshot,config,safety,result.clearance)) return false;
                    result.path=std::move(candidate);
                    traceDecision(&diagnostics,"corridor_local_fallback","committed_return_extended",
                        "existing_return_preserved=1 longitudinal_successor_only=1 all_obstacles_rechecked=1");
                    return true;
                };
                result.clearance=config.latticeObstacleClearance;
                // Pose reattachment is not necessarily an obstacle detour:
                // first connect directly to a nearby official gateway-reaching
                // lane, with the expanded mask confined to the checked bridge.
                if(relocalizing && searchTier==2) {
                    result.success=recoverTrackingPath(result.path,*snapshot,*mapSnapshot,world,
                        std::max(10.,config.gatewayReserveDistance),&diagnostics,result.reentryScope.get(),&mission) &&
                        localTrajectorySafe(result.path,world,*mapSnapshot,config,safety,result.clearance);
                    if(!result.success)result.path.clear();
                }
                if(!result.success)result.success=extendCommittedReturn();
                if(!result.success)result.success=searchLocalCorridor(result.path,*snapshot,*mapSnapshot,world,mission,config,safety,
                    result.clearance,minimumDistance,nominalDeadline,result.expanded,result.timedOut,&diagnostics,
                    result.reentryScope.get());
                if(!result.success && LocalClock::now()<deadline &&
                   config.latticeTightObstacleClearance<config.latticeObstacleClearance) {
                    result.clearance=std::max(0.,config.latticeTightObstacleClearance);result.timedOut=false;
                    result.success=extendCommittedReturn();
                    if(!result.success)result.success=searchLocalCorridor(result.path,*snapshot,*mapSnapshot,world,mission,config,safety,
                        result.clearance,minimumDistance,deadline,result.expanded,result.timedOut,&diagnostics,
                        result.reentryScope.get());
                }
            } catch(...) {result.exception=true;result.success=false;result.path.clear();}
            result.elapsedMs=std::chrono::duration<double,std::milli>(LocalClock::now()-started).count();
            result.diagnostics=std::move(diagnostics.frameEvents);return result;
        });
        r.localFallbackPending=true;
        traceDecision(trace,"corridor_local_fallback","worker_started",
            "blocker="+std::to_string(blocker)+" free_distance="+std::to_string(freeDistance)+
            " horizon="+std::to_string(config.localFallbackHorizon)+" minimum_endpoint="+std::to_string(minimumDistance)+
            " budget_ms="+std::to_string(searchTier>0?config.localReentrySearchBudgetMs:config.localFallbackSearchBudgetMs)+
            " tier="+std::to_string(searchTier)+" relocalizing="+std::to_string(relocalizing));
    } catch(const std::exception& error) {
        r.localFallbackPending=false;r.localFallbackRetryAt=now+std::chrono::milliseconds(500);
        traceDecision(trace,"corridor_local_fallback","launch_failed",error.what());
    }
    return false;
}

bool retainCorridorAvoidanceTail(const PathGeneratorData& map,const MissionCorridorData& mission,
                                const WorldState& world,const std::vector<Waypoint>& path,
                                DecisionTraceData* trace,double validatedClearance) {
    if(!map.corridorRuntime) return false;
    auto& r=*map.corridorRuntime;activate(r,mission);
    const double clearance=validatedClearance>=0.?validatedClearance:r.laneConfig.latticeObstacleClearance;
    if(!corridorTrackingPathIsSafe(path,world,clearance,nullptr,nullptr,&map)) return false;
    std::vector<Waypoint> suffix;
    if(!remainingAttachedPath(suffix,path,world)) return false;
    r.recoveryTrackingPath=std::move(suffix);r.recoveryTrackingPathProgress=0;
    r.recoveryTrackingPathRelocalized=false;
    r.recoveryTrackingPathLeg=r.activeLeg;r.recoveryTrackingPathClearance=clearance;
    r.recoveryTrackingPathReachesReference=false;
    traceDecision(trace,"corridor_lattice","safe_avoidance_tail_retained",
                  "points="+std::to_string(r.recoveryTrackingPath.size()));
    return true;
}

bool advanceClearLaneCenterReturn(std::vector<Waypoint>& path,
    const WorldState& world,const PathGeneratorData& map,
    const MissionCorridorData& mission,const PlanningBehaviorConfig& safety,
    DecisionTraceData* trace) {
    if(!map.corridorRuntime || !world.inputValid || !world.egoLaneValid ||
       world.respawnDetected || std::abs(world.egoSpeed)<.3) return false;
    auto& r=*map.corridorRuntime;const auto* leg=activeMissionLeg(mission);
    if(!leg || r.activeLeg!=mission.activeLegIndex || r.soleGatewayApproachSelected ||
       world.receiveTime<r.laneCenterRetryAfter) return false;
    std::vector<Waypoint> old;
    if(!remainingAttachedPath(old,path,world) || old.size()<3) return false;
    // Find where THIS path already intended to settle. Do not select a new
    // lane using an unstable boundary map match or a new gateway cost.
    std::size_t join=0;std::int64_t joinId=-1;
    for(std::size_t i=0;i<old.size();++i) {
        const auto& p=old[i];const double ahead=p.s-old.front().s;
        if(p.roadId!=world.egoRoadId || p.sectionIndex!=world.egoSectionIndex || ahead>80.) break;
        const auto lane=r.network.lanes.find(keyOf(p));
        if(lane==r.network.lanes.end()) break;
        const auto& samples=lane->second.pointIds;
        if(samples.empty()) break;
        const double direction=p.laneId<0?1.:-1.;
        auto near=std::lower_bound(samples.begin(),samples.end(),direction*p.roadS,
            [&](auto id,double station){return direction*r.network.points[id].pose.roadS<station;});
        if(near==samples.end()) --near;
        else if(near!=samples.begin() && gap(p,r.network.points[*(near-1)].pose)<gap(p,r.network.points[*near].pose)) --near;
        const auto& centre=r.network.points[*near].pose;
        const double offset=-(p.x-centre.x)*std::sin(centre.yaw)+(p.y-centre.y)*std::cos(centre.yaw);
        if(std::abs(offset)>.12 || std::abs(normalizeAngle(p.yaw-centre.yaw))>.04) continue;
        if(ahead<22.) return false; // already centred / already returning soon
        join=i;joinId=*near;break;
    }
    if(joinId<0 || !r.allowed[joinId] || !std::isfinite(r.gatewayCost[joinId])) return false;
    const auto key=keyOf(r.network.points[joinId].pose);
    if(leg->gatewayLanes.count(key)) return false;
    const auto& ids=r.network.lanes.at(key).pointIds;
    auto source=std::min_element(ids.begin(),ids.end(),[&](auto a,auto b) {
        return gap(old.front(),r.network.points[a].pose)<gap(old.front(),r.network.points[b].pose);
    });
    if(source==ids.end()) return false;
    const auto& startCentre=r.network.points[*source].pose;
    const auto lateral=[&](const Waypoint& p) {
        return -(p.x-startCentre.x)*std::sin(startCentre.yaw)+
                (p.y-startCentre.y)*std::cos(startCentre.yaw);
    };
    const double initial=lateral(old.front());
    if(std::abs(initial)<.45 || std::abs(initial)>r.network.points[*source].width*.55 ||
       std::abs(normalizeAngle(world.ego.heading-startCentre.yaw))>.08) return false;
    // Only a long parallel offset is eligible. An ongoing lane change,
    // bend or junction entry is not a request to change the driver's mind.
    for(const auto& p:old) {
        if(p.s-old.front().s>8.) break;
        if(std::abs(lateral(p)-initial)>.15 ||
           std::abs(normalizeAngle(p.yaw-startCentre.yaw))>.06) return false;
    }
    r.laneCenterRetryAfter=world.receiveTime+1.;
    const auto& cfg=r.laneConfig;
    const auto body=egoFootprint(world);
    const double settle=std::clamp(std::abs(world.egoSpeed)*3.,12.,30.);
    for(const double length:{settle,settle+6.}) {
        auto end=source;
        while(end!=ids.end() && r.network.points[*end].pose.s-startCentre.s<length) ++end;
        if(end==ids.end() || r.network.points[*end].pose.s+5.>=r.network.points[joinId].pose.s) continue;
        // Dense/passing obstacles are a reason to retain the fine offset,
        // even when a hairline centre trajectory technically fits. Include
        // moving objects here; detailed prediction is checked again below.
        bool busy=false;
        for(auto it=source;it!=ids.end() &&
            r.network.points[*it].pose.s<=r.network.points[*end].pose.s+8.;++it) {
            const auto& p=r.network.points[*it].pose;
            const auto box=vehicleFootprintAt(body,{p.x,p.y,p.yaw},cfg.latticeObstacleClearance+.5);
            for(const auto& object:world.objects)
                if(intersects(box,object.obb) && corridorObstacleAtPathLevel(map,p,object)) {busy=true;break;}
            if(busy) break;
        }
        if(busy) return false;
        Waypoint start=old.front();start.x=world.ego.x;start.y=world.ego.y;start.yaw=world.ego.heading;
        std::vector<Waypoint> candidate;double maximum{};
        if(!curve(candidate,start,r.curvature,r.network.points[*end],&maximum) ||
           maximum*std::max(1.,world.egoSpeed*world.egoSpeed)>1.5) continue;
        for(auto it=end+1;it!=ids.end() &&
            r.network.points[*it].pose.s<r.network.points[joinId].pose.s;++it)
            append(candidate,r.network.points[*it].pose);
        append(candidate,old[join]);
        std::string reason;
        if(!surfaceAndMarks(r,map,candidate,world,reason,true) ||
           !localTrajectorySafe(candidate,world,map,cfg,safety,cfg.latticeObstacleClearance)) continue;
        const double shift=old[join].s-candidate.back().s;
        for(auto& p:candidate) p.s+=shift;
        // Keep the exact original stations, geometry and gateway tail.
        candidate.insert(candidate.end(),old.begin()+join+1,old.end());
        path=std::move(candidate);
        traceDecision(trace,"lane_center","clear_return_advanced",
            "same_lane_chain=1 original_gateway_tail_preserved=1 settle_m="+
            std::to_string(length)+" original_join_m="+std::to_string(old[join].s-old.front().s));
        return true;
    }
    return false;
}

static bool buildCorridorTrackingGeometry(PathPlannerData& output,const PathPlannerConfig& config,
                              const WorldState& world,const PathGeneratorData& map,
                              const MissionCorridorData& mission,double horizon,DecisionTraceData* trace) {
    auto& r=*map.corridorRuntime;activate(r,mission);
    if(world.respawnDetected && r.activeGatewayObserved) {
        r.activeGatewayObserved=false;
        requestCorridorGlobalReplan(r);
    }
    if(const auto* leg=activeMissionLeg(mission);leg && world.egoLaneValid &&
       leg->gatewayLanes.count({world.egoRoadId,world.egoSectionIndex,world.egoLaneId})) {
        if(!r.activeGatewayObserved)
            traceDecision(trace,"corridor_global_route","required_gateway_observed",
                          "actual_ego_lane=1 mission_progress_unchanged=1");
        r.activeGatewayObserved=true;
    }
    output.config=config;
    if(!r.pendingGatewayHandoffPath.empty()) {
        std::vector<Waypoint> tail;
        std::string reason;
        const bool keep=!world.respawnDetected &&
            remainingAttachedPath(tail,r.pendingGatewayHandoffPath,world) &&
            surfaceAndMarks(r,map,tail,world,reason) &&
            corridorTrackingPathIsSafe(tail,world,r.pendingGatewayHandoffClearance,
                                      nullptr,nullptr,&map);
        r.pendingGatewayHandoffPath.clear();
        if(keep) {
            r.recoveryTrackingPath=std::move(tail);
            r.recoveryTrackingPathRelocalized=false;
            r.recoveryTrackingPathLeg=r.activeLeg;
            r.recoveryTrackingPathProgress=0;
            r.recoveryTrackingPathClearance=r.pendingGatewayHandoffClearance;
            // Poll the next-leg global search while following this certified
            // overlap. It is not a claim to reach the NEXT gateway already.
            r.globalRouteReplanRequested=true;
        }
        traceDecision(trace,"corridor_global_route",
                      keep?"exit_preview_handoff_retained":"exit_preview_handoff_rejected",
                      "current_pose_surface_and_obstacles_revalidated=1");
    }
    const auto recoverAndCache=[&]() {
        if(!recoverTrackingPath(output.waypoints,r,map,world,horizon,trace))
            return false;
        r.recoveryTrackingPathRelocalized=false;
        r.recoveryTrackingPath=output.waypoints;
        r.recoveryTrackingPathClearance=r.laneConfig.latticeObstacleClearance;
        r.recoveryTrackingPathLeg=r.activeLeg;
        r.recoveryTrackingPathProgress=0;
        r.recoveryTrackingPathReachesReference=false;
        traceDecision(trace,"corridor_lattice","tracking_recovery_cached",
                      "points="+std::to_string(output.waypoints.size()));
        return true;
    };
    const auto cacheCurrentOutput=[&]() {
        r.recoveryTrackingPathRelocalized=false;
        r.recoveryTrackingPath=output.waypoints;
        r.recoveryTrackingPathClearance=r.laneConfig.latticeObstacleClearance;
        r.recoveryTrackingPathLeg=r.activeLeg;
        r.recoveryTrackingPathProgress=0;
        r.recoveryTrackingPathReachesReference=true;
        traceDecision(trace,"corridor_lattice","tracking_recovery_cached",
                      "points="+std::to_string(output.waypoints.size()));
    };
    // Primary tracker: a single obstacle-aware route from the current
    // corridor position to the physical checkpoint gateway.  It includes
    // legal lateral quintics, so neither the greedy obstacle selector nor the
    // gateway selector has to rediscover the same decision every frame.
    // Poll/rebuild before returning a cached short recovery bridge. Otherwise
    // that bridge can indefinitely hide a completed global worker result.
    // While the worker is busy, keep using the bridge instead of rebuilding
    // its expensive swept geometry on every frame.
    // Even an idle committed global path must advance against the current
    // pose before a cached recovery tail can return. Otherwise that tail can
    // hide a completed global route for its entire length with no replan
    // flag ever set, including past the required lane-change window.
    if(ensureObstacleAwareGlobalRoute(
            output.waypoints,r,map,world,mission,horizon,trace) &&
       corridorTrackingPathIsSafe(output.waypoints,world,
            r.trackingPathValidatedClearance>=0.?r.trackingPathValidatedClearance:r.laneConfig.latticeObstacleClearance,
                                 nullptr,nullptr,&map)) return true;
    if(recoveryRouteSlice(output.waypoints,r,world,horizon,map)) {
        r.trackingPathValidatedClearance=r.recoveryTrackingPathClearance;
        r.trackingPathRelocalized=r.recoveryTrackingPathRelocalized;
        return true;
    }
    traceDecision(trace,"corridor_global_route","fallback_selected",
                  "reason=no_complete_static_obstacle_aware_route");
    if(r.committedGlobalPathReachesGateway &&
       connectToReferencePath(output.waypoints,r.committedGlobalPath,
                              r.committedGlobalPathProgress,r,map,world,
                              horizon,trace)) {
        cacheCurrentOutput();
        r.recoveryTrackingPathClearance=r.globalPathClearance();
        r.trackingPathValidatedClearance=r.globalPathClearance();
        return true;
    }
    r.trackingPathValidatedClearance=-1.;
    if(committedTransitionPath(output.waypoints,r,world,horizon,trace)) return true;
    const auto id=nearest(r,world,true);
    if(id<0) {
        output.config=config;
        return recoverAndCache();
    }
    if(r.poseValid && !world.respawnDetected) {
        const double d=std::hypot(world.ego.x-r.previousX,world.ego.y-r.previousY);
        if(d>.15 && d<3.) r.curvature=.7*r.curvature+.3*std::clamp(normalizeAngle(world.ego.heading-r.previousYaw)/d,-maximumCurvature,maximumCurvature);
        if(d>.15) {r.previousX=world.ego.x;r.previousY=world.ego.y;r.previousYaw=world.ego.heading;}
    } else {r.previousX=world.ego.x;r.previousY=world.ego.y;r.previousYaw=world.ego.heading;r.curvature=r.network.points[id].curvature;r.poseValid=true;}
    // Tracking may keep a plain physical continuation only while it remains
    // inside the active/next official corridor.  The former
    // allowAnyContinuation tier ignored that mask; at an overlapping split it
    // kept moving, but followed a perfectly valid XODR successor on the wrong
    // road and therefore missed the gateway.  An empty strict suffix is now a
    // recovery request, never permission to leave the mission corridor.
    int selectedTransition=-1;
    output.waypoints=forwardPath(r,id,horizon,world.egoPathS,false,
                                 &selectedTransition);
    if(!output.waypoints.empty() &&
       output.waypoints.back().s-world.egoPathS<horizon-.5 &&
       r.gatewayCost[id]>0.) {
        r.trackingGatewayStopDistance=std::max(0.,output.waypoints.back().s-world.egoPathS);
        traceDecision(trace,"corridor_lattice","gateway_prefix_endpoint",
            "remaining="+std::to_string(r.trackingGatewayStopDistance)+
            " start_cost="+std::to_string(r.gatewayCost[id]));
    }
    if(selectedTransition>=0) {
        if(r.committedTransition!=selectedTransition) {
            r.committedTransitionProgress=0;
            r.committedTransitionEntered=false;
        }
        r.committedTransition=selectedTransition;
        r.committedTransitionLeg=r.activeLeg;
        traceDecision(trace,"corridor_lattice","transition_committed",
                      "transition="+std::to_string(selectedTransition));
    }
    if(!output.waypoints.empty() &&
       std::hypot(output.waypoints.front().x-world.ego.x,
                  output.waypoints.front().y-world.ego.y)<1.0 &&
       std::abs(normalizeAngle(
           output.waypoints.front().yaw-world.ego.heading))>0.2) {
        // A source derivative probe may be the nearest stored point even
        // though its one-sided yaw is not the vehicle's drivable tangent.
        // Anchor only the first sample to the measured pose; every following
        // sample still comes unchanged from the precomputed XODR network.
        output.waypoints.front().x=world.ego.x;
        output.waypoints.front().y=world.ego.y;
        output.waypoints.front().yaw=world.ego.heading;
    }
    std::string joinFailure;
    if(!smoothSourceJoins(output.waypoints,r,map,world,&joinFailure)) {
        traceDecision(trace,"corridor_lattice","tracking_unavailable",
                      "source_join_not_smoothable_inside_corridor "+joinFailure);
        output.waypoints.clear();
        return recoverAndCache();
    }
    if(!joinFailure.empty())traceDecision(trace,"corridor_lattice",
        "mapped_curve_repair_unavailable",joinFailure);
    // A nearest XODR sample can be on the correct lane graph yet still be a
    // lane-width away after an avoidance manoeuvre.  Such a path is not a
    // valid successor for Settle: Pure Pursuit cuts sideways toward it, while
    // retaining the exhausted manoeuvre path makes its local goal command
    // zero speed forever.  Require a physically attached segment and use the
    // existing all-lane C2 recovery to bridge from the measured Ego pose.
    double attachment=inf;
    double attachmentYaw=inf;
    for(std::size_t i=1;i<output.waypoints.size();++i) {
        const auto& a=output.waypoints[i-1];
        const auto& b=output.waypoints[i];
        const double dx=b.x-a.x,dy=b.y-a.y;
        const double lengthSquared=dx*dx+dy*dy;
        if(lengthSquared<1e-8) continue;
        const double t=std::clamp(
            ((world.ego.x-a.x)*dx+(world.ego.y-a.y)*dy)/lengthSquared,
            0.,1.);
        const double distance=std::hypot(
            world.ego.x-(a.x+t*dx),world.ego.y-(a.y+t*dy));
        const double yaw=std::abs(normalizeAngle(
            std::atan2(dy,dx)-world.ego.heading));
        if(distance+.25*yaw>=attachment+.25*attachmentYaw) continue;
        attachment=distance;attachmentYaw=yaw;
    }
    if(attachment>1.25 || attachmentYaw>0.8) {
        std::ostringstream detail;
        detail<<"distance="<<attachment<<" yaw="<<attachmentYaw
              <<" action=connect_measured_pose_to_precomputed_lane";
        traceDecision(trace,"corridor_lattice",
                      "detached_tracking_path_recovery",detail.str());
        output.waypoints.clear();
        return recoverAndCache();
    }
    // Behind samples prevent nearest/PP from selecting an unrelated branch;
    // preserving physical arc distance across road changes avoids tail loss.
    if(output.waypoints.size()<2) {
        // The precomputed network always contains the points, so a dead stop
        // here is always an edge/filter problem. Record exactly which filter
        // removed every outgoing edge instead of a generic failure message.
        std::size_t edges=0,lateralEdges=0,outsideCorridor=0,gatewayUnreachable=0;
        for(const auto& e:r.forward[id]) {
            ++edges;
            if(e.lateral) {++lateralEdges;continue;}
            if(!r.allowed[e.to]) {++outsideCorridor;continue;}
            if(!std::isfinite(r.gatewayCost[e.to])) ++gatewayUnreachable;
        }
        std::ostringstream detail;
        detail<<"point="<<id<<" lane="<<world.egoRoadId<<':'
              <<world.egoSectionIndex<<':'<<world.egoLaneId
              <<" forward_edges="<<edges<<" lateral="<<lateralEdges
              <<" outside_corridor="<<outsideCorridor
              <<" gateway_unreachable="<<gatewayUnreachable
              <<" handoff_finite="<<std::isfinite(r.handoffCost[id])
              <<" active_leg="<<r.activeLeg;
        traceDecision(trace,"corridor_lattice","tracking_unavailable",detail.str());
        output.waypoints.clear();
        return recoverAndCache();
    }
    return true;
}

bool buildCorridorTrackingPath(PathPlannerData& output,const PathPlannerConfig& config,
                              const WorldState& world,const PathGeneratorData& map,
                              const MissionCorridorData& mission,double horizon,DecisionTraceData* trace) {
    auto& r=*map.corridorRuntime;
    r.trackingPathApproved=false;r.trackingPathBlockingObject=-1;
    r.trackingGatewayStopDistance=inf;
    r.soleGatewayApproachSelected=false;
    r.trackingPathRelocalized=false;
    r.trackingPathValidatedClearance=-1.;
    bool geometry=buildCorridorTrackingGeometry(output,config,world,map,mission,horizon,trace);
    std::vector<Waypoint> attached;
    // Recovery must run before the caller invalidates its active-path world.
    // Missing trajectory is itself a trigger, even with zero nearby objects.
    r.localFallbackRelocalizationRequested=!geometry ||
        !remainingAttachedPath(attached,output.waypoints,world) ||
        (std::isfinite(r.trackingGatewayStopDistance) &&
         r.trackingGatewayStopDistance<std::max(20.,std::abs(world.egoSpeed)*
             r.laneConfig.localFallbackTimeHeadway));
    if(r.localFallbackRelocalizationRequested) {
        LaneChangeData recovered;
        if(prepareLocalCorridorFallback(recovered,r.laneConfig,PlanningBehaviorConfig{},
            world,map,mission,output.waypoints,trace)) {
            output.waypoints=std::move(recovered.path);output.config=config;
            r.recoveryTrackingPath=output.waypoints;
            r.recoveryTrackingPathLeg=r.activeLeg;r.recoveryTrackingPathProgress=0;
            r.recoveryTrackingPathClearance=recovered.validatedObstacleClearance;
            r.recoveryTrackingPathReachesReference=false;
            r.recoveryTrackingPathRelocalized=true;r.trackingPathRelocalized=true;
            r.trackingPathValidatedClearance=recovered.validatedObstacleClearance;
            r.localFallbackRelocalizationRequested=false;geometry=true;
            r.trackingGatewayStopDistance=inf;
            traceDecision(trace,"corridor_lattice","lost_path_rejoined",
                "checked_current_pose_bridge=1 official_gateway_reachable=1 mission_not_skipped=1");
        }
    }
    if(!geometry) r.trackingGatewayStopDistance=0.;
    if(geometry) {
        const double clearance=r.trackingPathValidatedClearance>=0.
            ?r.trackingPathValidatedClearance:r.laneConfig.latticeObstacleClearance;
        r.trackingPathApproved=corridorTrackingPathIsSafe(output.waypoints,world,clearance,
            &r.trackingPathBlockingObject,nullptr,&map);
        r.trackingPathValidatedClearance=r.trackingPathApproved?clearance:-1.;
        if(!r.trackingPathApproved)
            traceDecision(trace,"corridor_global_route","tracking_geometry_not_approved",
                "blocker="+std::to_string(r.trackingPathBlockingObject)+
                " points="+std::to_string(output.waypoints.size())+
                " geometry_is_reference_only=1");
    }
    if(r.trackingPathRelocalized ||
       (r.trackingPathApproved && r.trackingPathValidatedClearance<r.laneConfig.latticeObstacleClearance)) {
        // Low-speed return, not a new general cruising-speed policy.
        const double speed=r.laneConfig.localFallbackSpeed;
        output.config.straightSpeed=std::min(output.config.straightSpeed,speed);
        output.config.curveSpeed=std::min(output.config.curveSpeed,speed);
        output.config.sharpCurveSpeed=std::min(output.config.sharpCurveSpeed,speed);
    }
    // Geometry remains available to local avoidance; approval is independent.
    return geometry;
}

bool buildReferenceRecoveryTrackingPath(
    PathPlannerData& output,const PathPlannerConfig& config,
    const std::vector<Waypoint>& reference,std::size_t referenceProgress,
    const WorldState& world,const PathGeneratorData& map,
    const MissionCorridorData& mission,double horizon,
    DecisionTraceData* trace) {
    output={};
    output.config=config;
    if(!map.corridorRuntime || reference.size()<2) return false;
    auto& r=*map.corridorRuntime;
    activate(r,mission);
    // Reuse the checked bridge while Ego remains attached to it.  This keeps
    // expensive curve/surface testing out of the ordinary 20 Hz loop.
    if(r.recoveryTrackingPathReachesReference &&
       recoveryRouteSlice(output.waypoints,r,world,horizon,map)) {
        traceDecision(trace,"corridor_lattice",
                      "mission_reference_recovery_retained",
                      "points="+std::to_string(output.waypoints.size()));
        return true;
    }
    const LaneKey currentLane{world.egoRoadId,world.egoSectionIndex,
                              world.egoLaneId};
    const bool retrySuppressed=r.referenceRecoveryFailureValid &&
        r.referenceRecoveryFailureLane==currentLane &&
        std::abs(world.egoRoadS-r.referenceRecoveryFailureRoadS)<2.0 &&
        world.receiveTime<r.referenceRecoveryRetryAfter;
    if(retrySuppressed) {
        traceDecision(trace,"corridor_lattice",
                      "mission_reference_recovery_retry_throttled",
                      "physical_lane_tracking_remains_active=1");
        return false;
    }
    if(!connectToReferencePath(output.waypoints,reference,referenceProgress,
                               r,map,world,horizon,trace,true)) {
        r.referenceRecoveryFailureLane=currentLane;
        r.referenceRecoveryFailureValid=true;
        r.referenceRecoveryFailureRoadS=world.egoRoadS;
        r.referenceRecoveryRetryAfter=world.receiveTime+1.0;
        return false;
    }
    r.referenceRecoveryFailureValid=false;
    r.recoveryTrackingPathRelocalized=false;
    r.recoveryTrackingPath=output.waypoints;
    r.recoveryTrackingPathClearance=r.laneConfig.latticeObstacleClearance;
    r.recoveryTrackingPathLeg=r.activeLeg;
    r.recoveryTrackingPathProgress=0;
    r.recoveryTrackingPathReachesReference=true;
    traceDecision(trace,"corridor_lattice",
                  "mission_reference_recovery_cached",
                  "points="+std::to_string(output.waypoints.size()));
    return true;
}

void beginCorridorSearchFrame(CorridorRuntime& runtime) {
    runtime.budgetedSearch=true;
    runtime.searchStarted=false;
    runtime.obstacleSearchCalled=false;
    runtime.gatewaySearchCalled=false;
}

bool prepareCorridorLattice(LaneChangeData& state,const LaneChangeConfig& config,
                           const PlanningBehaviorConfig& safetyConfig,const WorldState& world,
                           const PathGeneratorData& map,const MissionCorridorData& mission,
                           const std::vector<int>* confirmed,double* blockerDistance,
                           bool gatewayOnly,DecisionTraceData* trace,
                           const PathPlannerData* activeReference,
                           bool activeReferenceIsMissionGuide,
                           const LaneAlignmentContext* alignment) {
    const bool alignmentDeadline=alignment && std::isfinite(alignment->maximumCompletionDistance);
    if(blockerDistance) *blockerDistance=-1.;
    if(state.phase!=LaneChangePhase::None || !world.egoLaneValid || !world.egoPathValid) return false;
    auto& r=*map.corridorRuntime;activate(r,mission);
    auto startId=nearest(r,world);if(startId<0) return false;
    std::vector<Waypoint> committedBase;
    bool committedGlobalRoute=globalRouteSlice(
        committedBase,r,world,safetyConfig.localPathHorizon,false);
    const auto projectToPath=[&](const std::vector<Waypoint>& path,
                                 double& routeS,double& lateral,
                                 double& pathYaw) {
        if(path.size()<2) return false;
        double best=inf;
        bool found=false;
        for(std::size_t i=1;i<path.size();++i) {
            const double dx=path[i].x-path[i-1].x;
            const double dy=path[i].y-path[i-1].y;
            const double lengthSquared=dx*dx+dy*dy;
            if(lengthSquared<1e-8) continue;
            const double yaw=std::atan2(dy,dx);
            const double headingError=std::abs(normalizeAngle(
                yaw-world.ego.heading));
            if(headingError>1.2) continue;
            const double t=std::clamp(
                ((world.ego.x-path[i-1].x)*dx+
                 (world.ego.y-path[i-1].y)*dy)/lengthSquared,0.,1.);
            const double px=path[i-1].x+t*dx;
            const double py=path[i-1].y+t*dy;
            const double distance=std::hypot(world.ego.x-px,
                                              world.ego.y-py);
            const double score=distance+.25*headingError;
            if(score>=best) continue;
            best=score;found=true;pathYaw=yaw;
            routeS=path[i-1].s+t*(path[i].s-path[i-1].s);
            lateral=-(world.ego.x-px)*std::sin(yaw)+
                       (world.ego.y-py)*std::cos(yaw);
        }
        return found;
    };
    const auto pathAttachedToEgo=[&](const std::vector<Waypoint>& path,
                                     double maximumLateral) {
        double routeS{},lateral{},pathYaw{};
        return projectToPath(path,routeS,lateral,pathYaw) &&
            std::abs(lateral)<=maximumLateral &&
            std::abs(normalizeAngle(pathYaw-world.ego.heading))<=0.8;
    };
    // A corridor-wide route remains topologically valid after a local detour,
    // but it is not a valid *starting curve* when Ego is already one lane away
    // from it.  Candidate collision checks only protect the geometry that is
    // supplied to them; allowing a 3--5 m detached prefix made Pure Pursuit
    // cut diagonally through the very obstacle that the candidate avoided.
    if(activeReference && committedGlobalRoute &&
       !pathAttachedToEgo(committedBase,1.25)) {
        committedGlobalRoute=false;
        committedBase.clear();
        traceDecision(trace,"corridor_lattice","stale_global_prefix_rejected",
                      "reason=not_physically_attached_to_ego");
    }
    // The committed path already ends at the exact gateway and contains all
    // required legal lane changes.  A second gateway manoeuvre would be the
    // old failure mode: it can overwrite the globally selected branch with a
    // short greedy candidate.
    if(gatewayOnly && committedGlobalRoute) return false;
    std::vector<Waypoint> base;
    bool activeReferenceUsed=false;
    // A route-authority reference can legitimately be about half a lane away
    // after a collision impulse or a local avoidance.  Derive its attachment
    // envelope from the measured lane width instead of a brittle fixed 1.25 m
    // cutoff.  Ordinary non-authority paths retain the strict cutoff.
    const double authorityAttachmentDistance = std::max(
        1.25, 0.5 * std::max(world.egoLaneWidth, egoFootprint(world).width));
    if(activeReference &&
       pathAttachedToEgo(
           activeReference->waypoints,
           activeReferenceIsMissionGuide
               ? authorityAttachmentDistance : 1.25)) {
        double projectedS{},projectedD{},projectedYaw{};
        projectToPath(activeReference->waypoints,projectedS,
                      projectedD,projectedYaw);
        const auto nearest=std::lower_bound(
            activeReference->waypoints.begin(),
            activeReference->waypoints.end(),projectedS,
            [](const Waypoint& point,double pathS){return point.s<pathS;});
        const auto nearestIndex=nearest==activeReference->waypoints.end()
            ? activeReference->waypoints.size()-1
            : static_cast<std::size_t>(
                  nearest-activeReference->waypoints.begin());
        const auto begin=nearestIndex>4
            ? activeReference->waypoints.begin()+nearestIndex-4
            : activeReference->waypoints.begin();
        const double endS=projectedS+safetyConfig.localPathHorizon;
        for(auto it=begin;it!=activeReference->waypoints.end();++it) {
            append(base,*it);
            if(it->s>=endS) break;
        }
        activeReferenceUsed=base.size()>=2;
    }
    if(activeReferenceIsMissionGuide && activeReference != nullptr &&
       !activeReferenceUsed) {
        // Never synthesize an avoidance from an unrelated physical suffix.
        // The caller retains the checked guide/recovery trajectory and retries
        // after the temporary actor or pose displacement clears.
        traceDecision(trace,"corridor_lattice",
                      "route_authority_reference_detached",
                      "candidate_not_committed corridor_recovery_retained=1");
        return false;
    }
    if(activeReferenceUsed) {
        // Replanning an active avoidance must start on that exact committed
        // trajectory.  The corridor global route can legitimately be stale
        // until the current manoeuvre finishes.
    } else if(committedGlobalRoute) {
        base=std::move(committedBase);
        // Keep the complete route slice as the obstacle scan/candidate prefix.
    } else if(!gatewayOnly && committedTransitionPath(
            base,r,world,safetyConfig.localPathHorizon,trace)) {
        startId=r.network.transitions[r.committedTransition].toPoint;
    } else {
        base=forwardPath(r,startId,safetyConfig.localPathHorizon,world.egoPathS);
    }
    if(!smoothSourceJoins(base,r,map,world)) return false;
    if(base.size()<2) return false;
    // Path sources do not necessarily share an s origin.  A manoeuvre tail,
    // a committed global route and a freshly recovered XODR lane can all be
    // physically attached to Ego while carrying different absolute s values.
    // Using p.s-world.egoPathS directly then turned a physically 8--10 m
    // blocker into a reported 27 m blocker and delayed mandatory avoidance.
    // Rebase the selected physical path so Ego's projection is exactly the
    // current world.egoPathS before any trigger, horizon or completion test.
    std::vector<double> baseArc(base.size(),0.);
    for(std::size_t i=1;i<base.size();++i)
        baseArc[i]=baseArc[i-1]+gap(base[i-1],base[i]);
    double egoArc=0.,egoProjectionScore=inf;
    for(std::size_t i=1;i<base.size();++i) {
        const double dx=base[i].x-base[i-1].x;
        const double dy=base[i].y-base[i-1].y;
        const double lengthSquared=dx*dx+dy*dy;
        if(lengthSquared<1e-8) continue;
        const double t=std::clamp(
            ((world.ego.x-base[i-1].x)*dx+
             (world.ego.y-base[i-1].y)*dy)/lengthSquared,0.,1.);
        const double px=base[i-1].x+t*dx;
        const double py=base[i-1].y+t*dy;
        const double yaw=std::atan2(dy,dx);
        const double score=std::hypot(world.ego.x-px,world.ego.y-py)+
            .25*std::abs(normalizeAngle(yaw-world.ego.heading));
        if(score>=egoProjectionScore) continue;
        egoProjectionScore=score;
        egoArc=baseArc[i-1]+t*std::sqrt(lengthSquared);
    }
    for(std::size_t i=0;i<base.size();++i)
        base[i].s=world.egoPathS+baseArc[i]-egoArc;
    // On a diagnosed topology gap the packaged, map-derived reference is the
    // only route authority that is known to cross the requested gateway.  Its
    // lane keys therefore act as a reachability witness for local avoidance;
    // the incomplete corridor graph may report infinity even on the correct
    // outgoing lane.  This is generic to every missing-link leg and contains
    // no scenario, road or coordinate special case.
    const bool referenceIsGatewayAuthority=activeReferenceUsed &&
        (activeReferenceIsMissionGuide ||
         (!mission.topologyAuthority && !std::isfinite(r.gatewayCost[startId])));
    // Lane IDs are not branch identities.  Before a split, the lane that
    // continues straight and the lane that enters the official connector can
    // share the same road/section/lane metadata.  A candidate is therefore
    // considered route-preserving only when its physical centre endpoint
    // actually meets a *future* official-guide sample with compatible
    // tangent.  This prevents a long candidate from stepping past the fork
    // while still claiming reachability from a stale lane key.
    const auto referenceJoinIndex = [&](const Waypoint& target,
                                        double minimumRouteS) {
        std::size_t selected = base.size();
        double best = inf;
        if (!referenceIsGatewayAuthority) return selected;
        for (std::size_t index = 0; index < base.size(); ++index) {
            const auto& guidePoint = base[index];
            if (guidePoint.s + 1e-6 < minimumRouteS ||
                !(keyOf(guidePoint) == keyOf(target))) {
                continue;
            }
            const double positionError = gap(guidePoint, target);
            const double headingError = std::abs(normalizeAngle(
                guidePoint.yaw - target.yaw));
            if (positionError > 1.25 || headingError > 0.35) continue;
            const double score = positionError + headingError;
            if (score >= best) continue;
            best = score;
            selected = index;
        }
        return selected;
    };
    const WorldObjectState* blocker=nullptr;double blockerGap=inf;
    bool blockerIsDynamic=false;
    bool blockerFromDirectProjection=false;
    for(const auto& o:world.objects) {
        const bool moving=std::abs(o.raw.speed)>=.3;
        const bool confirmedStatic=!moving &&
            (!confirmed || std::find(confirmed->begin(),confirmed->end(),
                                     int(o.raw.id))!=confirmed->end());
        if(!moving && !confirmedStatic) continue;
        for(const auto& p:base) {
            const double along=p.s-world.egoPathS;
            if(along<0.) continue;
            if(along>config.obstacleTriggerMaximumDistance || along>=blockerGap) break;
            const auto box=vehicleFootprintAt(egoFootprint(world),{p.x,p.y,p.yaw});
            Obb2D obstacleBox=o.obb;
            if(moving) {
                // A moving object is a route invalidator only when its
                // predicted pose actually blocks the committed corridor path.
                // This avoids changing lanes for harmless parallel traffic,
                // while still replanning immediately for a cut-in.  EBS remains
                // the independent fallback when TTC is already critical.
                const double travelSpeed=std::max(
                    {1.,std::abs(world.egoSpeed),config.maximumLaneChangeSpeed});
                const double time=along/travelSpeed;
                if(time>safetyConfig.collisionPredictionHorizon+
                        safetyConfig.collisionPredictionStep) continue;
                obstacleBox=makeObb2D(
                    o.raw.x+objectVelocityX(o)*time,
                    o.raw.y+objectVelocityY(o)*time,
                    o.raw.heading,o.raw.length,o.raw.width);
            }
            if(intersects(box,obstacleBox)) {
                blocker=&o;blockerGap=along;blockerIsDynamic=moving;
                blockerFromDirectProjection=false;break;
            }
        }
    }
    // A merge/split tail can remain on a parallel sampled branch until only
    // centimetres before the branches meet. Path-OBB scanning then discovers
    // a compact obstacle directly in front of Ego far too late, although its
    // measured pose has been visible for many frames. Add a conservative
    // current-heading projection for confirmed, non-pedestrian stationary
    // objects inside the mandatory envelope. This is only a trigger: every
    // resulting path still passes the swept-OBB, road-mark, surface and
    // curvature checks below.
    const double egoC=std::cos(world.ego.heading);
    const double egoS=std::sin(world.ego.heading);
    for(const auto& o:world.objects) {
        if(std::abs(o.raw.speed)>=.3) continue;
        if(confirmed && std::find(confirmed->begin(),confirmed->end(),
                                  int(o.raw.id))==confirmed->end()) continue;
        // Full-size vehicles are already swept against the complete active
        // path above. A heading-only projection is too crude for a vehicle
        // in the neighbouring lane and caused false blockers in M11. This
        // supplement exists for compact low road furniture whose tiny OBB
        // can fall between the sampled path poses (cones/barrels/blocks).
        const bool compactLowRoadObject=
            o.raw.height>0.05F && o.raw.height<1.0F &&
            o.raw.length>=0.2F && o.raw.width>=0.2F;
        if(!compactLowRoadObject) continue;
        const bool vulnerableRoadUserSized=
            o.raw.length<=2.5F && o.raw.width<=1.5F &&
            o.raw.height>=1.0F;
        if(vulnerableRoadUserSized) continue;
        const double dx=o.raw.x-world.ego.x;
        const double dy=o.raw.y-world.ego.y;
        const double along=dx*egoC+dy*egoS;
        if(along<=0.) continue;
        const double lateral=-dx*egoS+dy*egoC;
        const double headingDelta=o.raw.heading-world.ego.heading;
        const double obstacleLongitudinalHalf=.5*(
            std::abs(std::cos(headingDelta))*o.raw.length+
            std::abs(std::sin(headingDelta))*o.raw.width);
        const double obstacleLateralHalf=.5*(
            std::abs(std::sin(headingDelta))*o.raw.length+
            std::abs(std::cos(headingDelta))*o.raw.width);
        // This fallback answers only "is it on the path directly ahead?".
        // Using the whole lane envelope also classified a vehicle safely in
        // the adjacent lane as a blocker, provoking an unnecessary move into
        // M11's parked-vehicle row. Keep a small tracking allowance around
        // the actual swept body instead.
        const double pathEnvelope=.5*egoFootprint(world).width+obstacleLateralHalf+
            config.latticeObstacleClearance+.15;
        if(std::abs(lateral)>pathEnvelope) continue;
        // Ego reference is the rear axle: 1.3725 m to OBB centre plus half
        // the 4.855 m body gives 3.8 m to the front bumper.
        const double contactGap=std::max(
            0.,along-egoFootprint(world).frontExtent()-obstacleLongitudinalHalf);
        if(contactGap>config.obstacleMandatoryAvoidanceDistance ||
           contactGap>=blockerGap) continue;
        blocker=&o;blockerGap=contactGap;blockerIsDynamic=false;
        blockerFromDirectProjection=true;
    }
    if(blockerDistance&&blocker) *blockerDistance=blockerGap;
    const bool mandatoryAvoidance=blocker &&
        blockerGap<=config.obstacleMandatoryAvoidanceDistance;
    const auto* leg=activeMissionLeg(mission);
    const bool atGateway=leg&&leg->targetLanes.count(keyOf(base.back()));
    const int currentGatewaySteps=r.gatewayLateralSteps[startId];
    const double speed=std::abs(world.egoSpeed);
    const double prefix=speed*config.prepareTime+std::max(0.,(speed*speed-config.maximumLaneChangeSpeed*config.maximumLaneChangeSpeed)/4.);
    // Search every gateway-reaching lane immediately when a legal lateral
    // stage remains. Surface/road-mark and collision validation rejects a
    // shift until the adjacent lane has enough physical width, so this picks
    // the first safe dashed window instead of waiting for a late deadline.
    const bool gatewayRequired=!atGateway && currentGatewaySteps>0 &&
        currentGatewaySteps<std::numeric_limits<int>::max();
    if(gatewayOnly ? !gatewayRequired : !blocker) {
        r.searchCursor[gatewayOnly?1:0]=0;
        if(!gatewayOnly) {
            r.obstacleSearchObject=-1;
            r.obstacleTightClearanceMode=false;
        }
        return false;
    }
    if(!gatewayOnly) {
        const int blockerId=static_cast<int>(blocker->raw.id);
        if(r.obstacleSearchObject!=blockerId) {
            r.searchCursor[0]=0;
            r.obstacleSearchObject=blockerId;
            r.obstacleTightClearanceMode=false;
        }
    }
    auto& alreadyCalled=gatewayOnly?r.gatewaySearchCalled:r.obstacleSearchCalled;
    if(r.budgetedSearch && alreadyCalled) {
        traceDecision(trace,"corridor_lattice","search_deferred","reason=already_searched_this_frame");
        return false;
    }
    alreadyCalled=true;
    if(r.searchLeg!=r.activeLeg) {r.searchCursor={};r.searchLeg=r.activeLeg;}
    if(r.budgetedSearch && !r.searchStarted) {
        r.searchStarted=true;
        r.searchDeadline=std::chrono::steady_clock::now()+
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double,std::milli>(config.latticeSearchBudgetMs));
    }
    // Start the C2 maneuver after preparation/deceleration, using the ordinary
    // lane points as prefix. No early high-speed diagonal steering command.
    auto begin=std::lower_bound(base.begin(),base.end(),world.egoPathS+prefix,[](const auto& p,double s){return p.s<s;});
    if(begin==base.end()) begin=base.begin();
    const double prefixLength=begin->s-world.egoPathS;
    // Gateway stages need a third horizon beyond the 20 m candidate, but using
    // the ordinary 30 m obstacle horizon can consume the remaining taper. Use
    // a 24 m default (medium + half the reserved exit distance) for gateway
    // alignment while retaining 12/20/30 m for obstacle exploration.
    const double thirdDistance=gatewayOnly
        ? std::min(config.latticeLongDistance,
                   config.latticeMediumDistance+
                       0.5*config.gatewayReserveDistance)
        : config.latticeLongDistance;
    const std::array<double,3> horizons{prefixLength+config.latticeShortDistance,
        prefixLength+config.latticeMediumDistance,prefixLength+thirdDistance};
    const bool compactBlocker=blocker && blocker->raw.height>0.05F &&
        blocker->raw.height<1.0F && blocker->raw.length>=0.2F &&
        blocker->raw.width>=0.2F;
    // Once a confirmed stationary blocker is inside the mandatory envelope,
    // examine the narrow but non-overlapping passages immediately.  Waiting
    // for an entire 0.30 m-clearance search to exhaust first consumed dozens
    // of control frames in dense parked-vehicle rows; the 0.05 m pass then
    // started only after the front bumper had reached the obstacle. Dynamic
    // actors retain the wider margin because their predicted pose can move.
    const bool tightClearance=!gatewayOnly && mandatoryAvoidance &&
        (!blockerIsDynamic || compactBlocker ||
         r.obstacleTightClearanceMode);
    const double candidateClearance=tightClearance
        ? config.latticeTightObstacleClearance
        : config.latticeObstacleClearance;
    auto collisionConfig=config;
    collisionConfig.latticeObstacleClearance=candidateClearance;
    auto endpoints=targets(r,startId,horizons,egoFootprint(world).width);
    if(!gatewayOnly) {
        // Densely spaced obstacles and the last dashed window need shorter
        // manoeuvres than the nominal 12/20/30 m choices. Every added endpoint
        // still passes the same curvature, swept-footprint and collision tests.
        for(const auto& extra:std::array<std::array<double,3>,2>{{
                {{prefixLength+6.,prefixLength+8.,prefixLength+10.}},
                {{prefixLength+16.,prefixLength+24.,prefixLength+40.}}}}) {
            for(const auto& endpoint:targets(r,startId,extra,egoFootprint(world).width)) {
                if(std::none_of(endpoints.begin(),endpoints.end(),[&](const auto& old){return old.second==endpoint.second;}))
                    endpoints.push_back(endpoint);
            }
        }
    }
    double bestCost=inf,bestClearance=-1.,bestDistance=inf;
    bool bestGatewayReachable=false,bestFree=false;
    int bestGatewaySteps=std::numeric_limits<int>::max();
    LaneChangeData best;bool found=false;
    double bestStartOffset=0.,bestLaneOffset=0.;
    std::map<std::string,int> rejected;
    std::size_t tried=0;
    std::size_t ordinal=0;
    auto& cursor=r.searchCursor[gatewayOnly?1:0];
    const std::size_t resume=r.budgetedSearch?cursor:0;
    bool deferred=false;
    struct SearchSpec {
        double distance{};
        std::int64_t id{-1};
        double startOffset{};
        double laneOffset{};
        bool missionReachable{};
        int missionLateralSteps{std::numeric_limits<int>::max()};
        int tier{};
        std::size_t stableOrder{};
    };
    std::vector<SearchSpec> searchSpecs;
    std::size_t stableOrder=0;
    for(const auto& [distance,id]:endpoints) {
      std::vector<double> starts{prefixLength};
      if(!gatewayOnly)
        for(double shiftLength:{6.,10.,16.,24.}) {
          const double offset=distance-shiftLength;
          if(offset>prefixLength+2.) starts.push_back(offset);
        }
      const double halfLane=std::min(1.5,r.network.points[id].width/2.);
      const std::vector<double> offsets=gatewayOnly
          ? std::vector<double>{0.}
          : std::vector<double>{0.,-halfLane,halfLane,
                                -halfLane*.5,halfLane*.5,
                                -halfLane*(2./3.),halfLane*(2./3.)};
      for(std::size_t startIndex=0;startIndex<starts.size();++startIndex) {
        for(const double laneOffset:offsets) {
          const bool centre=std::abs(laneOffset)<.01;
          const bool fullOffset=std::abs(laneOffset)>=halfLane*.60;
          // Do not exhaust every refinement of one lane before looking at the
          // next lane.  Rank common candidates across the whole corridor:
          // centre/ordinary start, full offset/ordinary start, then delayed
          // starts and finally half-offset refinements.  All candidates remain
          // present, including the short late shifts needed in cone clusters.
          int tier=0;
          if(startIndex==0) tier=centre?0:(fullOffset?1:4);
          else tier=centre?2:(fullOffset?3:5);
          const bool guideReachable=referenceIsGatewayAuthority &&
              referenceJoinIndex(
                  r.network.points[id].pose,
                  world.egoPathS+std::max(0.,distance-3.))<base.size();
          const bool missionReachable=
              std::isfinite(r.gatewayCost[id]) || guideReachable;
          searchSpecs.push_back({distance,id,starts[startIndex],laneOffset,
                                 missionReachable,
                                 guideReachable?0:r.gatewayLateralSteps[id],
                                 tier,stableOrder++});
        }
      }
    }
    std::stable_sort(searchSpecs.begin(),searchSpecs.end(),
                     [](const auto& a,const auto& b) {
        // Inspect gateway/reference-preserving lanes first.  A time-budgeted
        // scan must not commit an unrelated branch merely because it happened
        // to precede the correct lane in point-id order.
        if(a.missionReachable!=b.missionReachable)
            return a.missionReachable>b.missionReachable;
        if(a.missionLateralSteps!=b.missionLateralSteps)
            return a.missionLateralSteps<b.missionLateralSteps;
        if(a.tier!=b.tier) return a.tier<b.tier;
        return a.stableOrder<b.stableOrder;
    });
    for(const auto& spec:searchSpecs) {
      const double distance=spec.distance;
      const auto id=spec.id;
        const double startOffset=spec.startOffset;
        const double laneOffset=spec.laneOffset;
        const auto begin=std::lower_bound(base.begin(),base.end(),world.egoPathS+startOffset,
                                         [](const auto& p,double value){return p.s<value;});
        if(begin==base.end()) continue;
        const auto current=ordinal++;
        if(current<resume) continue;
        if(r.budgetedSearch && std::chrono::steady_clock::now()>=r.searchDeadline) {
            cursor=current;deferred=true;goto search_finished;
        }
        ++tried;
        const Waypoint centreTarget=r.network.points[id].pose;
        auto target=r.network.points[id];
        target.pose.x-=laneOffset*std::sin(target.pose.yaw);
        target.pose.y+=laneOffset*std::cos(target.pose.yaw);
        traceCandidateLane(trace,keyOf(target.pose));
        const bool gatewayReachable=std::isfinite(r.gatewayCost[id]);
        const std::size_t guideJoin = referenceJoinIndex(
            centreTarget,
            world.egoPathS+std::max(0.,distance-3.));
        const bool guideReachable=guideJoin<base.size();
        const bool missionReachable=gatewayReachable || guideReachable;
        const int targetGatewaySteps=r.gatewayLateralSteps[id];
        // Replacing an attached official guide with a zero-offset C2 curve
        // back to the very same lane cannot avoid anything.  Near a split it
        // is actively harmful: a tiny geometric difference can cross the
        // attachment threshold and hand tracking to the other successor.
        // Keep the exact guide until a candidate makes a real lateral move.
        if(!gatewayOnly && referenceIsGatewayAuthority && guideReachable &&
           std::abs(laneOffset)<.01 &&
           keyOf(target.pose)==keyOf(r.network.points[startId].pose)) {
            ++rejected["official_guide_noop"];
            continue;
        }
        // The official route remains the branch authority at every urgency
        // level.  A mandatory blocker may change the local geometry, but it
        // may not authorize a candidate whose endpoint cannot physically
        // rejoin the future guide.  Search failure keeps the current route
        // latched so safety control or a later route-preserving candidate can
        // handle the obstacle; it never turns into an arbitrary branch.
        if(!gatewayOnly && referenceIsGatewayAuthority && !guideReachable) {
            ++rejected["official_route_not_rejoined"];
            continue;
        }
        // Urgency changes clearance/search effort, not the mandatory exit.
        // If the current position can still reach this gateway, an ordinary
        // detour may not terminate on an irreversible different successor.
        // After crossing the gateway, the next-leg handoff may legitimately
        // have no finite current-gateway cost; retain that existing fallback.
        if(!gatewayOnly && !missionReachable &&
           (!mandatoryAvoidance || (!r.activeGatewayObserved &&
                                    std::isfinite(r.gatewayCost[startId])))) {
            ++rejected["required_gateway_unreachable"];
            continue;
        }
        // A distant blocker may justify preparing an escape, but it may not
        // move the car farther away from the required exit.  "Eventually
        // reachable" is too weak near a split: in M12 it selected a clear
        // lane requiring three later changes while Ego's current branch
        // required only two, and the remaining dashed length was then gone.
        // A blocker inside the mandatory envelope still overrides this rule
        // so immediate collision avoidance remains possible.
        if(!gatewayOnly && !mandatoryAvoidance &&
           currentGatewaySteps<std::numeric_limits<int>::max() &&
           targetGatewaySteps>currentGatewaySteps) {
            ++rejected["distant_gateway_regression"];
            continue;
        }
        if(gatewayOnly && !gatewayReachable) {
            ++rejected["gateway_unreachable"];continue;
        }
        if(gatewayOnly) {
            // Accept one safe intermediate lane at a time. It only has to
            // reduce the number of legal lateral stages and remain connected
            // to the gateway; it need not reach the gateway in this quintic.
            // This preserves obstacle clearance before final alignment.
            if(r.gatewayLateralSteps[id]>=currentGatewaySteps) {
                ++rejected["gateway_lateral_stage_not_reduced"];continue;
            }
        }
        std::vector<Waypoint> candidate(base.begin(),begin);
        Waypoint start=*begin;double startK=r.network.points[startId].curvature;
        if(begin==base.begin()) {start.x=world.ego.x;start.y=world.ego.y;start.yaw=world.ego.heading;start.s=world.egoPathS;startK=r.curvature;}
        else {
            WorldState probe=world;probe.ego.x=start.x;probe.ego.y=start.y;probe.ego.heading=start.yaw;
            const auto pid=nearest(r,probe);if(pid>=0) startK=r.network.points[pid].curvature;
        }
        double maxK{};
        if(!curve(candidate,start,startK,target,&maxK)) {++rejected["curvature_or_heading"];continue;}
        std::string reason;
        if(!surfaceAndMarks(r,map,candidate,world,reason)) {++rejected[reason];traceCandidatePath(trace,keyOf(target.pose),candidate,"rejected",reason);continue;}
        double completion=candidate.back().s;
        if(gatewayOnly && alignmentDeadline &&
           completion-world.egoPathS>alignment->maximumCompletionDistance) {
            ++rejected["alignment_after_stop_line"];continue;
        }
        auto tail=forwardPath(r,id,safetyConfig.localPathHorizon,completion);
        if(referenceIsGatewayAuthority && guideReachable) {
            // A lane key does not identify a unique successor at a split.
            // If this candidate lands back on the attached official guide,
            // its suffix must come from that guide as well.  Calling
            // forwardPath() here used point-id ordering and could append the
            // other branch even for a zero-offset, same-lane candidate.
            if(guideJoin<base.size() &&
               gap(base[guideJoin],centreTarget)<=1.25 &&
               std::abs(normalizeAngle(base[guideJoin].yaw-
                                       centreTarget.yaw))<=0.35) {
                tail.clear();
                const double routeShift=completion-base[guideJoin].s;
                for(std::size_t guideIndex=guideJoin;
                    guideIndex<base.size();++guideIndex) {
                    auto guidePoint=base[guideIndex];
                    guidePoint.s+=routeShift;
                    append(tail,guidePoint);
                }
            }
        }
        if(std::abs(laneOffset)>.01) {
            const auto centreTail=tail;
            std::vector<Waypoint> shifted;
            for(auto p:tail) {
                p.x-=laneOffset*std::sin(p.yaw);p.y+=laneOffset*std::cos(p.yaw);
                append(shifted,p);
            }
            // Keep the checked offset until the triggering obstacle has been
            // passed. Releasing at the first lateral target would steer back
            // into an obstacle sitting on the original lane centre.
            double holdUntil=completion;
            if(blocker) holdUntil=std::max(
                holdUntil,world.egoPathS+blockerGap+
                              double(blocker->raw.length)+6.);
            const auto holdIt=std::lower_bound(
                shifted.begin(),shifted.end(),holdUntil,
                [](const auto& point,double routeS){return point.s<routeS;});
            if(holdIt==shifted.end()) {
                ++rejected["offset_hold_tail_too_short"];continue;
            }
            const auto holdIndex=static_cast<std::size_t>(
                holdIt-shifted.begin());

            // An offset is only a temporary narrow-passage state.  Build the
            // complete manoeuvre here: ingress -> obstacle-clear hold -> C2
            // return to the exact XODR centre -> ordinary precomputed tail.
            // Previously the shifted tail was infinite and rejoinRouteS
            // pointed at an offset pose.  Settle then retained that detached
            // path until Pure Pursuit ran out of points and commanded a stop.
            bool recentered=false;
            for(const double returnLength:{8.,12.,16.,24.}) {
                const auto returnIt=std::lower_bound(
                    centreTail.begin()+std::min(holdIndex,centreTail.size()),
                    centreTail.end(),shifted[holdIndex].s+returnLength,
                    [](const auto& point,double routeS){return point.s<routeS;});
                if(returnIt==centreTail.end()) continue;
                const auto returnIndex=static_cast<std::size_t>(
                    returnIt-centreTail.begin());
                const auto centreId=containing(
                    r,returnIt->x,returnIt->y,returnIt->yaw);
                const auto holdCentreId=containing(
                    r,centreTail[holdIndex].x,centreTail[holdIndex].y,
                    centreTail[holdIndex].yaw);
                if(centreId<0 || holdCentreId<0) continue;
                auto returnTarget=r.network.points[centreId];
                returnTarget.pose=*returnIt;
                std::vector<Waypoint> returnCurve;
                double returnMaximum{};
                if(!curve(returnCurve,shifted[holdIndex],
                          r.network.points[holdCentreId].curvature,
                          returnTarget,&returnMaximum)) continue;
                std::string returnReason;
                if(!surfaceAndMarks(r,map,returnCurve,world,returnReason))
                    continue;

                std::vector<Waypoint> settledTail;
                for(std::size_t i=0;i<=holdIndex;++i)
                    append(settledTail,shifted[i]);
                for(const auto& point:returnCurve)
                    append(settledTail,point);
                // rejoinRouteS is now the actual lane-centre attachment,
                // never the end of the temporary offset hold.
                completion=settledTail.back().s;
                for(std::size_t i=returnIndex+1;i<centreTail.size();++i)
                    append(settledTail,centreTail[i]);
                tail=std::move(settledTail);
                maxK=std::max(maxK,returnMaximum);
                recentered=true;
                break;
            }
            if(!recentered) {
                ++rejected["offset_not_recentered"];continue;
            }
            if(tail.empty() || tail.back().s<completion+2.) {
                ++rejected["recenter_tail_too_short"];continue;
            }
            auto checked=tail;
            checked.erase(std::upper_bound(checked.begin(),checked.end(),completion+2.,
                          [](double v,const auto& p){return v<p.s;}),checked.end());
            if(!surfaceAndMarks(r,map,checked,world,reason)) {
                ++rejected[reason];continue;
            }
        }
        const auto collisionTail=tail;
        const bool terminalGatewayTarget=
            gatewayOnly && r.gatewayLateralSteps[id]==0;
        if(!smoothSourceJoins(tail,r,map,world)) {
            if(!terminalGatewayTarget) {
                ++rejected["tracking_tail_not_smoothable"];continue;
            }
            // The current corridor deliberately ends at the gateway. A join
            // beyond it belongs to the next mission leg and can be absent or
            // discontinuous in the current-leg graph. Keep only the smooth
            // prefix needed to cross the gateway; the mission advances as
            // soon as the target road is observed and supplies the next tail.
            tail=collisionTail;
            std::size_t keep=tail.size();
            for(std::size_t i=1;i<tail.size();++i) {
                const double ds=gap(tail[i-1],tail[i]);
                if(ds<1e-3 || std::abs(normalizeAngle(
                       tail[i].yaw-tail[i-1].yaw))>.19*ds+1e-6) {
                    keep=i;break;
                }
            }
            tail.resize(keep);
            traceCandidatePath(trace,keyOf(target.pose),candidate,
                               "viable","terminal_gateway_tail_trimmed");
        }
        // Inspect the maneuver plus a short settling tail; retain the complete
        // tracking tail, and rank its future obstacles separately.
        auto safety=candidate;
        for(const auto& p:tail) {if(p.s>completion+2.) break;append(safety,p);}
        if(!collisionFree(safety,world,safetyConfig,collisionConfig,&map)) {++rejected["collision"];traceCandidatePath(trace,keyOf(target.pose),candidate,"rejected","collision");continue;}
        // A stationary-object search is not successful merely because a
        // short candidate stops before the object.  The controller never
        // stops at that candidate endpoint; it keeps following the appended
        // lane tail, so accepting such a candidate only postpones the same
        // collision while locking out a better manoeuvre.  Every selected
        // static-obstacle path must therefore clear the triggering object,
        // regardless of whether it is already inside the mandatory-distance
        // window.  The mandatory distance controls urgency, not correctness.
        if(blocker && !blockerIsDynamic) {
            auto passage=candidate;
            for(const auto& p:tail) append(passage,p);
            WorldState triggeringObstacle=world;
            triggeringObstacle.objects={*blocker};
            const auto& end=passage.back();
            const double beyond=(end.x-blocker->raw.x)*std::cos(end.yaw)+
                                (end.y-blocker->raw.y)*std::sin(end.yaw);
            if(std::isfinite(staticClearance(passage,triggeringObstacle,
                                             candidateClearance,nullptr,&map)) ||
               beyond<egoFootprint(world).rearExtent()+0.5*std::hypot(blocker->raw.length,blocker->raw.width)+
                      candidateClearance) {
                ++rejected["static_obstacle_not_passed"];
                traceCandidatePath(trace,keyOf(target.pose),candidate,"rejected",
                                   "static_obstacle_not_passed");
                continue;
            }
        }
        int followupObject=-1;
        // Obstacle clearance is evaluated on the untrimmed physical lane
        // continuation even when the controller tail stops at a mission-leg
        // boundary, so a terminal gateway never hides an obstacle.
        const double clearance=staticClearance(
            collisionTail,world,candidateClearance,
            &followupObject,&map);
        const double hops=std::abs(-(target.pose.x-start.x)*std::sin(start.yaw)+(target.pose.y-start.y)*std::cos(start.yaw))/3.5;
        const double cost=config.latticeLaneCost*hops+config.latticeCurvatureCost*maxK+
            config.latticeProgressCost/(1.+std::min(80.,clearance))+.2*std::abs(laneOffset);
        const double tailLength=tail.empty()?0.:tail.back().s-tail.front().s;
        const bool settlingClear=clearance>=config.latticeShortDistance+config.gatewayReserveDistance;
        // A known obstacle 25 m ahead is not equivalent to an unobstructed
        // lane. Otherwise the cheap straight candidate wins until the last
        // legal lane-change window has passed. Rank a fully checked clear
        // continuation first, retaining finite-clearance paths as fallback.
        const bool free=!std::isfinite(clearance) &&
            tailLength>=config.gatewayReserveDistance;
        // A gateway manoeuvre is never allowed to point back into an obstacle
        // waiting just beyond its polynomial endpoint. Obstacle-mode planning
        // can still keep such a best-progress fallback and chain another
        // avoidance, but gateway alignment may use only a clear settling tail.
        if(gatewayOnly && !settlingClear) {
            ++rejected["gateway_tail_blocked"];continue;
        }
        // For gateway stages, select the shortest safe longitudinal horizon.
        // Curvature is already hard-limited above; preferring the smoothest
        // 30 m curve consumed the road needed by the following gateway stage.
        // Among equally unobstructed escapes preserve a way to complete the
        // mission. An unreachable lane remains eligible when it is the only
        // clear escape; reachability is not a collision-filter override.
        const auto gatewayBetter=[&]() {
            if(missionReachable!=bestGatewayReachable)
                return missionReachable;
            if(missionReachable && targetGatewaySteps!=bestGatewaySteps)
                return targetGatewaySteps<bestGatewaySteps;
            return cost<bestCost;
        };
        const bool better=gatewayOnly
            ? (!found || (alignmentDeadline && targetGatewaySteps<bestGatewaySteps) ||
               ((!alignmentDeadline || targetGatewaySteps==bestGatewaySteps) &&
                (distance<bestDistance-1.0 ||
                 (std::abs(distance-bestDistance)<=1.0&&cost<bestCost))))
            : (!found || (free&&!bestFree) || (free==bestFree &&
               (free?gatewayBetter()
                    :clearance>bestClearance+.5 ||
                (std::abs(clearance-bestClearance)<=.5&&gatewayBetter()))));
        traceCandidatePath(trace,keyOf(target.pose),candidate,"viable","corridor_all_lanes_C2");
        if(!better) continue;
        for(const auto& p:tail) append(candidate,p);
        best={};best.phase=LaneChangePhase::Prepare;best.routeMode=LaneChangeRouteMode::KeepTargetLane;
        best.obstacleId=blocker?int(blocker->raw.id):-1;
        best.sourceRoadId=world.egoRoadId;best.sourceSectionIndex=world.egoSectionIndex;best.sourceLaneId=world.egoLaneId;
        best.targetLaneId=target.pose.laneId;best.reconnectLane=keyOf(target.pose);best.reconnectLaneValid=true;
        best.shiftCompletedLane=keyOf(target.pose);best.shiftCompletedLaneValid=true;
        best.startRouteS=world.egoPathS;best.rejoinRouteS=completion;best.roadConnectionManeuver=gatewayOnly;
        best.missionRouteAuthority=referenceIsGatewayAuthority;
        const double lateral=-(target.pose.x-start.x)*std::sin(start.yaw)+(target.pose.y-start.y)*std::cos(start.yaw);
        best.turnSignal=std::abs(lateral)<.2?0U:(lateral>0?1U:2U);best.outwardTurnSignal=best.turnSignal;
        best.path=std::move(candidate);best.bestProgressFallback=!free;
        best.validatedObstacleClearance=candidateClearance;
        best.followupObstacleId=followupObject;
        best.followupObstacleClearance=std::isfinite(clearance)?clearance:-1.;
        bestCost=cost;bestClearance=clearance;bestDistance=distance;
        bestGatewayReachable=missionReachable;
        bestFree=free;
        bestGatewaySteps=r.gatewayLateralSteps[id];
        bestStartOffset=startOffset;bestLaneOffset=laneOffset;found=true;
    }
    cursor=0;
search_finished:
    // Candidates are always rebuilt against the current world. Only the scan
    // position survives; a stale trajectory is never restored from a cache.
    // Do not commit a merely finite-clearance candidate discovered in an
    // early budget slice while the blocker is still far away.  That used to
    // select a cheap in-lane offset after inspecting only 9 of 400+ choices,
    // even though a later lane provided the only route through a dense cone
    // field.  A completely clear candidate may be committed immediately.  If
    // every lane remains obstructed, keep advancing the common cursor and
    // allow the best-progress fallback once the obstacle is close enough that
    // delaying further would consume the manoeuvre window.
    const bool provisionalDistantFallback =
        found && deferred && !gatewayOnly && !bestFree &&
        blockerGap > std::max(config.latticeMediumDistance,
                              config.obstacleMandatoryAvoidanceDistance);
    if (provisionalDistantFallback) {
        found = false;
        traceDecision(trace, "corridor_lattice",
                      "partial_finite_candidate_deferred",
                      "reason=all_corridor_lanes_not_yet_examined");
    }
    // Preserve the ordinary 0.30 m comfort margin whenever it works. If a
    // complete mandatory scan finds no route, retry this same blocker with
    // the configured tight margin on following frames. This is a geometric
    // fallback shared by every scenario; collision checks are never removed.
    if(!found && !deferred && !gatewayOnly && mandatoryAvoidance &&
       !tightClearance &&
       config.latticeTightObstacleClearance<
           config.latticeObstacleClearance) {
        r.obstacleTightClearanceMode=true;
        cursor=0;
        deferred=true;
        traceDecision(trace,"corridor_lattice","tight_clearance_retry",
                      "reason=nominal_margin_exhausted");
    }
    std::ostringstream detail;detail<<"endpoints="<<tried<<" deferred="<<deferred
        <<" next_candidate="<<cursor<<" gateway_required="<<gatewayRequired
        <<" mandatory_avoidance="<<mandatoryAvoidance
        <<" mandatory_distance="<<config.obstacleMandatoryAvoidanceDistance
        <<" gateway_steps="<<currentGatewaySteps<<" blocker_gap="<<blockerGap
        <<" blocker_direct_projection="<<blockerFromDirectProjection
        <<" candidate_clearance="<<candidateClearance
        <<" tight_clearance="<<tightClearance;
    for(const auto& [reason,count]:rejected) detail<<" "<<reason<<"="<<count;
    if(found) {state=std::move(best);detail<<" cost="<<bestCost
        <<" selected_distance="<<bestDistance
        <<" selected_start_offset="<<bestStartOffset
        <<" selected_lane_offset="<<bestLaneOffset
        <<" selected_mission_reachable="<<bestGatewayReachable
        <<" selected_route_authority="<<state.missionRouteAuthority
        <<" target_gateway_steps="<<bestGatewaySteps
        <<" followup_clearance="<<bestClearance
        <<" blocker_dynamic="<<blockerIsDynamic;
        if(!gatewayOnly) ++r.routeRevision;
        traceCandidatePath(trace,state.shiftCompletedLane,state.path,"selected","corridor_all_lanes_C2");}
    traceDecision(trace,"corridor_lattice",found?"selected":(deferred?"search_deferred":"no_safe_candidate"),detail.str());
    if(found) {
        if(!gatewayOnly) r.obstacleTightClearanceMode=false;
        std::ostringstream routeDetail;
        routeDetail<<"revision="<<r.routeRevision
                   <<" reason="<<(gatewayOnly?"gateway_alignment"
                                              :(blockerIsDynamic?"dynamic_obstacle_invalidated_route"
                                                                   :"static_obstacle_route"))
                   <<" start="<<world.egoRoadId<<':'<<world.egoSectionIndex
                   <<':'<<world.egoLaneId
                   <<" target="<<state.shiftCompletedLane.roadId<<':'
                   <<state.shiftCompletedLane.sectionIndex<<':'
                   <<state.shiftCompletedLane.laneId
                   <<" route_authority="<<state.missionRouteAuthority
                   <<" remaining_gateway_stages="<<bestGatewaySteps;
        traceDecision(trace,"corridor_global_route","materialized",routeDetail.str());
    }
    return found;
}
