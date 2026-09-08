#include "planning/local/corridor_lattice.hpp"
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

// Own immutable map/graph snapshots. The control loop only polls the result;
// it never waits for global A* or shares mutable planning state with it.
struct CorridorGlobalRouteWorker {
    struct Result {
        std::size_t leg{};
        std::uint64_t revision{}, staticSignature{};
        LaneKey startLane;
        double startRoadS{}, receiveTime{};
        bool success{};
        bool exception{};
        std::vector<Waypoint> path;
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
bool visible(const PrecomputedLaneNetwork& n,std::int64_t id) {
    if(id<0 || id>=static_cast<std::int64_t>(n.points.size())) return false;
    const auto& p=n.points[id];
    const auto lane=n.lanes.find(keyOf(p.pose));
    return p.replacedByTransition<0 && lane!=n.lanes.end() && lane->second.topologyStatus!="excluded_by_review";
}
std::int64_t containing(const CorridorRuntime& r,double x,double y,double yaw,
                        bool corridorOnly=true);
void addEdge(CorridorRuntime& r,std::int64_t from,CorridorEdge e) {
    if(!visible(r.network,from) || !visible(r.network,e.to)) return;
    r.forward[from].push_back(e);
    r.reverse[e.to].push_back({from,e.distance,e.transition,e.lateral});
}
void activate(CorridorRuntime& r,const MissionCorridorData& mission) {
    if(r.activeLeg==mission.activeLegIndex) return;
    r.activeLeg=mission.activeLegIndex;
    r.committedTransition=-1;
    r.committedTransitionLeg=mission.activeLegIndex;
    r.committedTransitionProgress=0;
    r.committedTransitionEntered=false;
    r.committedGlobalPath.clear();
    r.committedGlobalPathLeg=mission.activeLegIndex;
    r.committedGlobalPathProgress=0;
    r.committedGlobalPathReachesGateway=false;
    r.recoveryTrackingPath.clear();
    r.recoveryTrackingPathLeg=mission.activeLegIndex;
    r.recoveryTrackingPathProgress=0;
    r.recoveryTrackingPathReachesReference=false;
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
            if(visible(r.network,p.id) &&
               (nextLeg.allowedLanes.count(key)||nextLeg.targetLanes.count(key))) {
                r.trackingAllowed[p.id]=true;
                // Join smoothing validates the physical swept surface.  The
                // next leg is already a prevalidated road corridor, so its
                // surface must be visible during this atomic handoff too.
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
        // Prefer the gateway-aware choice. If the strict search finds no
        // edge here (e.g. a handoff gap right at a leg boundary), fall back
        // to any allowed forward lane rather than truncating the tracking
        // path. A short/awkward continuation is always better than stopping
        // the vehicle because "no path" was found.
        const auto* e=forwardChoice(r,id);
        if(!e) e=forwardChoice(r,id,false);
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
                                                  const std::array<double,3>& distances) {
    using Item=std::pair<double,std::int64_t>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> q;
    std::map<std::int64_t,double> costs;costs[start]=0.;q.push({0.,start});
    std::map<std::pair<LaneKey,int>,Item> selected;
    while(!q.empty()) {
        const auto [d,id]=q.top();q.pop();if(d>costs[id]+1e-8 || d>distances[2]+1.) continue;
        const auto& p=r.network.points[id];
        for(int i=0;i<3;++i) if(std::abs(d-distances[i])<=.75 && p.width>=1.88) {
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
           const PrecomputedLanePoint& target,double* maximum) {
    const auto& end=target.pose;const double length=gap(start,end);
    if(length<3. || std::abs(normalizeAngle(end.yaw-start.yaw))>1.4) return false;
    const auto x=polynomial(start.x,length*std::cos(start.yaw),-length*length*startK*std::sin(start.yaw),
                            end.x,length*std::cos(end.yaw),-length*length*target.curvature*std::sin(end.yaw));
    const auto y=polynomial(start.y,length*std::sin(start.yaw),length*length*startK*std::cos(start.yaw),
                            end.y,length*std::sin(end.yaw),length*length*target.curvature*std::cos(end.yaw));
    const int count=static_cast<int>(std::ceil(length/.2));*maximum=0.;
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
                        bool corridorOnly) {
    const int gx=static_cast<int>(std::floor(x/10.)),gy=static_cast<int>(std::floor(y/10.));
    std::int64_t result=-1;double best=inf;
    const double c=std::cos(yaw),s=std::sin(yaw);
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
        const auto cell=r.spatial.find({gx+dx,gy+dy});if(cell==r.spatial.end()) continue;
        for(auto id:cell->second) {
            if((corridorOnly && !r.surfaceAllowed[id]) ||
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
                     bool corridorOnly=true) {
    const double width=std::max(1.88,std::hypot(world.egoObb.corners[0].x-world.egoObb.corners[1].x,
                                             world.egoObb.corners[0].y-world.egoObb.corners[1].y));
    std::int64_t previous=-1;
    for(auto& p:path) {
        const auto id=containing(r,p.x,p.y,p.yaw,corridorOnly);
        if(id<0) {reason=corridorOnly?"center_outside_corridor":"center_outside_driving_surface";return false;}
        const auto& center=r.network.points[id].pose;
        if(previous>=0 && !boundaryAllowed(map,r.network.points[previous].pose,center)) {
            reason="solid_or_forbidden_lane_change";return false;
        }
        previous=id;
        p.roadId=center.roadId;p.sectionIndex=center.sectionIndex;p.laneId=center.laneId;p.roadS=center.roadS;
        const double c=std::cos(p.yaw),s=std::sin(p.yaw);
        for(double along=-1.055;along<=3.8+1e-6;along+=.4855) for(double side:{-width/2.,0.,width/2.}) {
            const auto corner=containing(r,p.x+along*c-side*s,p.y+along*s+side*c,p.yaw,
                                         corridorOnly);
            if(corner<0) {reason=corridorOnly?"footprint_outside_corridor":"footprint_outside_driving_surface";return false;}
            if(!boundaryAllowed(map,center,r.network.points[corner].pose)) {
                reason="footprint_crosses_solid";return false;
            }
        }
    }
    return true;
}
double staticClearance(const std::vector<Waypoint>& path,const WorldState& world,
                       double margin=0.,int* objectId=nullptr) {
    if(objectId) *objectId=-1;
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
                vehicleSized&&crossingOrientation?1.50:0.;
            const auto box=makeObb2D(
                p.x+1.3725*std::cos(p.yaw),
                p.y+1.3725*std::sin(p.yaw),p.yaw,
                4.855+2*(margin+startUncertainty),
                1.88+2*(margin+startUncertainty));
            if(intersects(box,o.obb)) {
                if(objectId) *objectId=static_cast<int>(o.raw.id);
                return p.s-path.front().s;
            }
        }
    }
    return inf;
}
bool collisionFree(const std::vector<Waypoint>& path,const WorldState& world,
                   const PlanningBehaviorConfig& config,const LaneChangeConfig& laneConfig) {
    // Check every stationary object, including the one that triggered this
    // search, over the complete manoeuvre. The moving prediction below only
    // contains moving objects and cannot replace this test.
    if(std::isfinite(staticClearance(path,world,laneConfig.latticeObstacleClearance))) return false;
    WorldState moving=world;
    moving.egoSpeed=std::max({1.,std::abs(world.egoSpeed),laneConfig.maximumLaneChangeSpeed});
    moving.objects.erase(std::remove_if(moving.objects.begin(),moving.objects.end(),
        [](const auto& o){return std::abs(o.raw.speed)<.3;}),moving.objects.end());
    LocalPathData local;local.valid=true;local.points=path;
    PathSafetyResult safety;resetPathSafety(safety);
    checkCurrentCollisions(safety,world.egoObb,world.objects);
    checkPredictedCollisions(safety,moving,local,config.collisionPredictionHorizon,config.collisionPredictionStep,true);
    return !safety.collision&&!safety.predictedCollision;
}
bool smoothSourceJoins(std::vector<Waypoint>& path,const CorridorRuntime& r,
                       const PathGeneratorData& map,const WorldState& world,
                       std::string* failure=nullptr,bool corridorOnly=true) {
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
        // A sustained tight XODR connector is real road geometry and is
        // handled by the curvature speed cap.  Only an isolated yaw jump is
        // a broken join that needs C2 replacement.
        if(std::abs(yawDelta)<=.19*distance+1e-6 || sustainedMapCurve) continue;
        bool fixed=false;
        for(double radius:{3.,5.,8.}) {
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
            if(!curve(replacement,path[first],r.network.points[a].curvature,end,&maxK) ||
               !surfaceAndMarks(r,map,replacement,world,reason,corridorOnly)) continue;
            std::vector<Waypoint> joined(path.begin(),path.begin()+first);
            for(const auto& p:replacement) append(joined,p);
            for(auto j=last+1;j<path.size();++j) append(joined,path[j]);
            path=std::move(joined);i=first+replacement.size()-1;fixed=true;break;
        }
        if(!fixed) {
            if(failure) {
                std::ostringstream detail;
                detail<<"index="<<i<<" distance="<<distance
                      <<" yaw_delta="
                      <<std::abs(normalizeAngle(path[i].yaw-path[i-1].yaw))
                      <<" from="<<path[i-1].roadId<<':'
                      <<path[i-1].sectionIndex<<':'<<path[i-1].laneId
                      <<" to="<<path[i].roadId<<':'
                      <<path[i].sectionIndex<<':'<<path[i].laneId;
                *failure=detail.str();
            }
            return false;
        }
    }
    return true;
}

bool isStationaryRouteObstacle(const WorldObjectState& object) {
    if(std::abs(object.raw.speed)>=.3) return false;
    // Pedestrians and similarly compact vulnerable actors remain under the
    // crosswalk/EBS state machine.  Treating a pedestrian waiting beside a
    // crossing as permanent road furniture would make the global planner
    // change lanes around the crossing instead of yielding.  Vehicle-sized
    // and elongated static actors are corridor obstacles from the first
    // frame, including the narrow 2.0 x 0.85 m scenario objects.
    const bool vehicleOrElongated=
        object.raw.length>=1.5 || object.raw.width>=1.2;
    // Scaled cones/barrels in the competition scenes are roughly
    // 0.9 x 0.9 x 0.9 m. They are genuine static road obstacles, but the old
    // vehicle-size threshold removed them from the global route scan. Height
    // separates them from a standing pedestrian without object-name or
    // scenario-specific rules.
    const bool compactLowObstacle=
        object.raw.height>0.05 && object.raw.height<1.0 &&
        object.raw.length>=0.2 && object.raw.width>=0.2;
    return vehicleOrElongated || compactLowObstacle;
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
                         std::size_t begin=0,int* objectId=nullptr) {
    if(objectId) *objectId=-1;
    if(path.empty()) return false;
    begin=std::min(begin,path.size()-1);
    for(std::size_t i=begin;i<path.size();++i) {
        const auto& p=path[i];
        const auto box=makeObb2D(
            p.x+1.3725*std::cos(p.yaw),
            p.y+1.3725*std::sin(p.yaw),p.yaw,
            4.855+2.*margin,1.88+2.*margin);
        for(const auto& object:world.objects) {
            if(!isStationaryRouteObstacle(object)) continue;
            if(!intersects(box,object.obb)) continue;
            if(objectId) *objectId=static_cast<int>(object.raw.id);
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
            r.network.points[point].width/2.-.94-
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
                    physical,world,r.laneConfig.latticeObstacleClearance)) {
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
            output,world,r.laneConfig.latticeObstacleClearance)) {
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
bool materializePrecomputedGatewayRoute(
    std::vector<Waypoint>& output,CorridorRuntime& r,
    const PathGeneratorData& map,const WorldState& world,
    const MissionCorridorData& mission,std::int64_t start,
    DecisionTraceData* trace) {
    output.clear();
    const auto* leg=activeMissionLeg(mission);
    if(!leg || start<0 || !r.allowed[start] ||
       leg->toCheckpoint>=mission.checkpoints.size()) return false;

    std::vector<bool> goal(r.network.points.size(),false);
    std::size_t goalCount=0;
    for(const auto& key:leg->targetLanes) {
        const auto lane=r.network.lanes.find(key);
        if(lane==r.network.lanes.end()) continue;
        for(const auto id:lane->second.pointIds) {
            if(id<0 || !r.allowed[id] ||
               !missionPointIsTarget(*leg,r.network.points[id].pose)) continue;
            if(!goal[id]) {goal[id]=true;++goalCount;}
        }
    }
    if(goalCount==0) return false;

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
            r.network.points[point].width/2.-.94-
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
    const std::size_t stateCount=
        r.network.points.size()*offsetStateCount;
    std::vector<signed char> blocked(stateCount,-1);
    const auto stateBlocked=[&](std::int64_t state) {
        auto& cached=blocked[state];
        if(cached>=0) return cached!=0;
        const auto pose=offsetPose(statePoint(state),stateOffset(state));
        const auto box=makeObb2D(
            pose.x+1.3725*std::cos(pose.yaw),
            pose.y+1.3725*std::sin(pose.yaw),pose.yaw,
            4.855+2.*r.laneConfig.latticeObstacleClearance,
            1.88+2.*r.laneConfig.latticeObstacleClearance);
        cached=0;
        for(const auto& object:world.objects) {
            if(!isStationaryRouteObstacle(object)) continue;
            if(std::abs(object.raw.z-world.ego.z)>3.) continue;
            if(intersects(box,object.obb)) {cached=1;break;}
        }
        return cached!=0;
    };
    const auto heuristic=[&](std::int64_t state) {
        const double value=r.gatewayCost[statePoint(state)];
        return std::isfinite(value)?value:0.;
    };
    const auto edgeKey=[](const Parent& step) {
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
        const auto a=offsetPose(statePoint(fromState),stateOffset(fromState));
        const auto b=offsetPose(toPoint,toOffset);
        const double dx=b.x-a.x,dy=b.y-a.y;
        const double lengthSquared=dx*dx+dy*dy;
        bool nearStationary=false;
        for(const auto& object:world.objects) {
            if(!isStationaryRouteObstacle(object) ||
               std::abs(object.raw.z-world.ego.z)>3.) continue;
            const double u=lengthSquared>1e-6?std::clamp(
                ((object.raw.x-a.x)*dx+(object.raw.y-a.y)*dy)/
                    lengthSquared,0.,1.):0.;
            const double px=a.x+u*dx,py=a.y+u*dy;
            const double conservativeRadius=3.0+
                .5*std::hypot(object.raw.length,object.raw.width)+
                r.laneConfig.latticeObstacleClearance;
            if(std::hypot(object.raw.x-px,object.raw.y-py)<=
               conservativeRadius) {
                nearStationary=true;break;
            }
        }
        if(!nearStationary) {
            generatedCollisionCache[key]=true;
            return true;
        }
        std::vector<Waypoint> physical;
        const bool clear=offsetCurve(
                physical,statePoint(fromState),stateOffset(fromState),
                toPoint,toOffset) &&
            stationaryPathClear(
                physical,world,r.laneConfig.latticeObstacleClearance);
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
        const auto startState=stateId(start,1);
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
            if(goal[id] && currentOffset==1) {
                reachedState=currentState;break;
            }
            const auto relax=[&](const CorridorEdge& edge,int targetOffset,
                                 bool generatedCurve,double penalty) {
                if(edge.to<0 || !r.allowed[edge.to]) return;
                const auto targetState=stateId(edge.to,targetOffset);
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
                const double candidate=distance[currentState]+
                    std::max(.0001,edge.distance)+penalty;
                if(candidate+1e-8>=distance[targetState]) return;
                distance[targetState]=candidate;
                parent[targetState]=step;
                queue.push({candidate+heuristic(targetState),targetState});
            };
            for(const auto& edge:r.forward[id]) {
                if(edge.lateral) {
                    for(int targetOffset=0;targetOffset<offsetStateCount;
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
            // Shifts within one lane are necessary for staggered narrow
            // obstacles.  The graph assumes the shift, then the selected
            // quintic is validated exactly below.
            const std::array<double,4> shiftLengths{
                6.,8.,r.laneConfig.latticeShortDistance,
                r.laneConfig.latticeMediumDistance};
            for(const double desired:shiftLengths) {
                auto target=id;double travelled=0.;
                std::set<std::int64_t> seen;
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
                    relax({target,travelled,-1,true},targetOffset,true,.35);
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
        for(const auto& step:reversed) {
            const auto from=statePoint(step.fromState);
            const int fromOffset=stateOffset(step.fromState);
            std::vector<Waypoint> physical;
            if(step.generatedCurve) {
                if(!offsetCurve(physical,from,fromOffset,
                                step.edge.to,step.targetOffset) ||
                   !surfaceAndMarks(r,map,physical,world,failure)) {
                    selectedRouteValid=false;failedStep=step;break;
                }
            } else if(step.edge.transition>=0) {
                physical=corridorEdgePath(r,from,step.edge);
            } else {
                physical={offsetPose(from,fromOffset),
                          offsetPose(step.edge.to,step.targetOffset)};
            }
            if(physical.empty() || !stationaryPathClear(
                    physical,world,r.laneConfig.latticeObstacleClearance)) {
                failure="selected_edge_intersects_stationary_object";
                selectedRouteValid=false;failedStep=step;break;
            }
            for(const auto& point:physical) append(output,point);
        }
        if(!selectedRouteValid) {
            ++rejectedSelected;
            std::ostringstream detail;
            detail<<"attempt="<<attempt
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
                  <<" generated="<<failedStep.generatedCurve;
            traceDecision(trace,"corridor_global_route",
                          "fast_selected_edge_rejected",detail.str());
            rejectedEdges.insert(edgeKey(failedStep));
            output.clear();
            continue;
        }
        repairTrackingTangents(output);
        if(output.size()<2 ||
           !smoothSourceJoins(output,r,map,world,&failure) ||
           !stationaryPathClear(
               output,world,r.laneConfig.latticeObstacleClearance)) {
            traceDecision(trace,"corridor_global_route",
                          "fast_materialization_failed",failure);
            output.clear();return false;
        }
        break;
    }
    if(reachedState<0 || output.size()<2) {
        traceDecision(trace,"corridor_global_route",
                      "precomputed_gateway_unreachable",
                      "expanded="+std::to_string(expandedTotal)+
                      " goals="+std::to_string(goalCount)+
                      " blocked_samples="+std::to_string(rejectedStatic)+
                      " rejected_selected_edges="+
                          std::to_string(rejectedSelected));
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
          <<" points="<<output.size()
          <<" expanded="<<expandedTotal
          <<" blocked_samples="<<rejectedStatic
          <<" rejected_selected_edges="<<rejectedSelected
          <<" rule=stationary_clear_first_directed_sample_on_required_exit_road";
    traceDecision(trace,"corridor_global_route",
                  "precomputed_gateway_materialized",detail.str());
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
                        const WorldState& world,double horizon) {
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
       !stationaryPathClear(route,world,r.laneConfig.latticeObstacleClearance,
                            nearestIndex)) {
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
            r.committedGlobalPath,world,r.laneConfig.latticeObstacleClearance,
            r.committedGlobalPathProgress);
    }
    if(retained) {output=std::move(current);return true;}

    if (r.budgetedSearch && mission.topologyAuthority && r.globalRouteWorker) {
        auto& worker = *r.globalRouteWorker;
        if (worker.pending.valid() &&
            worker.pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto result = worker.pending.get();
            if (result.leg == r.activeLeg && result.revision == r.globalRouteRequestRevision) {
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
                if (result.success && best <= 1.25 && nearestIndex+1 < result.path.size() &&
                    stationaryPathClear(result.path,world,r.laneConfig.latticeObstacleClearance,nearestIndex)) {
                    r.committedGlobalPath = std::move(result.path);
                    r.committedGlobalPathLeg = r.activeLeg;
                    r.committedGlobalPathProgress = nearestIndex;
                    r.committedGlobalPathReachesGateway = true;
                    r.globalRouteSearchFailed = false;
                    r.globalRouteReplanRequested = false;
                    r.recoveryTrackingPath.clear();
                    ++r.routeRevision;
                    traceDecision(trace,"corridor_global_route","async_route_adopted",
                                  "current_pose_and_observed_static_suffix_revalidated=1");
                    if (globalRouteSlice(output,r,world,horizon)) return true;
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
            try {
              worker.pending = std::async(std::launch::async,
                [snapshot,mapSnapshot,world,mission,signature,revision]() {
                    CorridorGlobalRouteWorker::Result result;
                    result.leg = mission.activeLegIndex;
                    result.revision = revision;
                    result.staticSignature = signature;
                    result.startLane = {world.egoRoadId,world.egoSectionIndex,world.egoLaneId};
                    result.startRoadS = world.egoRoadS;
                    result.receiveTime = world.receiveTime;
                    try {
                        activate(*snapshot,mission);
                        const auto start = nearest(*snapshot,world);
                        if (start>=0) result.success = materializePrecomputedGatewayRoute(
                            result.path,*snapshot,*mapSnapshot,world,mission,start,nullptr);
                    } catch (...) {result.success=false;result.exception=true;}
                    return result;
                });
              r.globalRouteReplanRequested = true;
              traceDecision(trace,"corridor_global_route","async_search_started",
                          "scope=whole_active_corridor received_objects="+
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
        if (sliceAvailable) {output=std::move(current);return true;}
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
        if(!retrySuppressed && start>=0 && materializePrecomputedGatewayRoute(
                rebuilt,r,map,world,mission,start,trace)) {
            r.committedGlobalPath=std::move(rebuilt);
            r.committedGlobalPathLeg=r.activeLeg;
            r.committedGlobalPathProgress=0;
            r.committedGlobalPathReachesGateway=true;
            r.globalRouteSearchFailed=false;
            r.globalRouteReplanRequested=false;
            r.recoveryTrackingPath.clear();
            ++r.routeRevision;
            if(globalRouteSlice(output,r,world,horizon)) return true;
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
        r.committedGlobalPathLeg=r.activeLeg;
        r.committedGlobalPathProgress=0;
        r.committedGlobalPathReachesGateway=true;
        r.globalRouteSearchFailed=false;
        r.globalRouteReplanRequested=false;
        r.recoveryTrackingPath.clear();
        ++r.routeRevision;
        if(globalRouteSlice(output,r,world,horizon)) return true;
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
                         double horizon,DecisionTraceData* trace) {
    using Choice=std::pair<double,std::int64_t>;
    const int gx=int(std::floor(world.ego.x/10.));
    const int gy=int(std::floor(world.ego.y/10.));
    const double c=std::cos(world.ego.heading),s=std::sin(world.ego.heading);
    auto attempt=[&](bool physicalNetwork,std::size_t& tested) {
        std::map<std::pair<LaneKey,int>,Choice> choices;
        for(int dx=-4;dx<=4;++dx) for(int dy=-4;dy<=4;++dy) {
            const auto cell=r.spatial.find({gx+dx,gy+dy});
            if(cell==r.spatial.end()) continue;
            for(const auto id:cell->second) {
                if(physicalNetwork?!visible(r.network,id):!r.trackingAllowed[id]) continue;
                const auto& p=r.network.points[id];
                const double px=p.pose.x-world.ego.x,py=p.pose.y-world.ego.y;
                const double ahead=px*c+py*s;
                if(ahead<6. || ahead>35. || std::abs(-px*s+py*c)>10. ||
                   p.width<1.88 || std::abs(normalizeAngle(p.pose.yaw-world.ego.heading))>1.) continue;
                for(int bucket=0;bucket<3;++bucket) {
                    const double score=std::abs(ahead-(12.+9.*bucket));
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
            if(!curve(candidate,start,r.curvature,target,&maxK) ||
               !surfaceAndMarks(r,map,candidate,world,reason,!physicalNetwork)) continue;
            const double completion=candidate.back().s;
            auto tail=forwardPath(r,id,horizon,completion,physicalNetwork);
            if(!smoothSourceJoins(tail,r,map,world,nullptr,!physicalNetwork)) continue;
            auto safety=candidate;
            for(const auto& p:tail) {if(p.s>completion+2.) break;append(safety,p);}
            if(!collisionFree(safety,world,PlanningBehaviorConfig{},r.laneConfig)) continue;
            const double score=std::abs(-(target.pose.x-start.x)*s+(target.pose.y-start.y)*c)+
                               20.*maxK+(std::isfinite(r.gatewayCost[id])?0.:20.);
            if(score>=best) continue;
            for(const auto& p:tail) append(candidate,p);
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
    r->tangent.resize(r->network.points.size());
    for(const auto& p:r->network.points) {
        if(!map.roads.count(p.pose.roadId)) {if(error)*error="precomputed road absent from active XODR";return false;}
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
    for(const auto& p:r->network.points) if(visible(r->network,p.id)) {
        for(auto side:{std::make_pair(p.leftPoint,p.leftChangeAllowed),std::make_pair(p.rightPoint,p.rightChangeAllowed)}) {
            if(!side.second || !visible(r->network,side.first)) continue;
            auto id=side.first,source=p.id;double distance=0.,sourceDistance=0.;bool legal=true;
            while(distance<r->lateralLength) {
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
            if(legal) addEdge(*r,p.id,{id,distance,-1,true});
        }
    }
    // Snapshot once at startup, not on a control frame or at each replan.
    // The worker owns its graph/masks and cannot race activation/map matching.
    auto worker=std::make_shared<CorridorGlobalRouteWorker>();
    worker->runtime=std::make_shared<CorridorRuntime>(*r);
    auto frozenMap=std::make_shared<PathGeneratorData>(map);
    frozenMap->corridorRuntime.reset();
    worker->map=std::move(frozenMap);
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
                            const CorridorRuntime& r,
                            const PathGeneratorData& map,
                            const WorldState& world,double horizon,
                            DecisionTraceData* trace,
                            bool corridorOnly=true) {
    output.clear();
    if(reference.size()<2) return false;
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
                          r.laneConfig)) continue;
        const double score=std::abs(lateral)+20.*maxK+.01*ahead;
        if(score>=bestScore) continue;
        output=std::move(candidate);bestScore=score;
    }
    if(output.size()<2) return false;
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

bool buildCorridorTrackingPath(PathPlannerData& output,const PathPlannerConfig& config,
                              const WorldState& world,const PathGeneratorData& map,
                              const MissionCorridorData& mission,double horizon,DecisionTraceData* trace) {
    auto& r=*map.corridorRuntime;activate(r,mission);
    output.config=config;
    const auto recoverAndCache=[&]() {
        if(!recoverTrackingPath(output.waypoints,r,map,world,horizon,trace))
            return false;
        r.recoveryTrackingPath=output.waypoints;
        r.recoveryTrackingPathLeg=r.activeLeg;
        r.recoveryTrackingPathProgress=0;
        r.recoveryTrackingPathReachesReference=false;
        traceDecision(trace,"corridor_lattice","tracking_recovery_cached",
                      "points="+std::to_string(output.waypoints.size()));
        return true;
    };
    const auto cacheCurrentOutput=[&]() {
        r.recoveryTrackingPath=output.waypoints;
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
    const bool globalAttempted = r.globalRouteReplanRequested ||
        (r.globalRouteWorker && r.globalRouteWorker->pending.valid());
    if(globalAttempted && ensureObstacleAwareGlobalRoute(
            output.waypoints,r,map,world,mission,horizon,trace)) return true;
    if(recoveryRouteSlice(output.waypoints,r,world,horizon)) return true;
    if(!globalAttempted && ensureObstacleAwareGlobalRoute(
            output.waypoints,r,map,world,mission,horizon,trace)) return true;
    traceDecision(trace,"corridor_global_route","fallback_selected",
                  "reason=no_complete_static_obstacle_aware_route");
    if(r.committedGlobalPathReachesGateway &&
       connectToReferencePath(output.waypoints,r.committedGlobalPath,
                              r.committedGlobalPathProgress,r,map,world,
                              horizon,trace)) {
        cacheCurrentOutput();
        return true;
    }
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
       recoveryRouteSlice(output.waypoints,r,world,horizon)) {
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
    r.recoveryTrackingPath=output.waypoints;
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
                           bool activeReferenceIsMissionGuide) {
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
        1.25, 0.5 * std::max(world.egoLaneWidth, 1.88));
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
            const auto box=makeObb2D(p.x+1.3725*std::cos(p.yaw),p.y+1.3725*std::sin(p.yaw),p.yaw,4.855,1.88);
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
        const double pathEnvelope=.5*1.88+obstacleLateralHalf+
            config.latticeObstacleClearance+.15;
        if(std::abs(lateral)>pathEnvelope) continue;
        // Ego reference is the rear axle: 1.3725 m to OBB centre plus half
        // the 4.855 m body gives 3.8 m to the front bumper.
        const double contactGap=std::max(
            0.,along-(1.3725+.5*4.855)-obstacleLongitudinalHalf);
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
    auto endpoints=targets(r,startId,horizons);
    if(!gatewayOnly) {
        // Densely spaced obstacles and the last dashed window need shorter
        // manoeuvres than the nominal 12/20/30 m choices. Every added endpoint
        // still passes the same curvature, swept-footprint and collision tests.
        for(const auto& extra:std::array<std::array<double,3>,2>{{
                {{prefixLength+6.,prefixLength+8.,prefixLength+10.}},
                {{prefixLength+16.,prefixLength+24.,prefixLength+40.}}}}) {
            for(const auto& endpoint:targets(r,startId,extra)) {
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
        if(!gatewayOnly && !mandatoryAvoidance && !missionReachable) {
            ++rejected["distant_gateway_unreachable"];
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
        if(!collisionFree(safety,world,safetyConfig,collisionConfig)) {++rejected["collision"];traceCandidatePath(trace,keyOf(target.pose),candidate,"rejected","collision");continue;}
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
                                             candidateClearance)) ||
               beyond<1.055+0.5*std::hypot(blocker->raw.length,blocker->raw.width)+
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
            &followupObject);
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
            ? (!found || distance<bestDistance-1.0 ||
               (std::abs(distance-bestDistance)<=1.0&&cost<bestCost))
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
