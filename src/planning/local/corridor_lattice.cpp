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
void addEdge(CorridorRuntime& r,std::int64_t from,CorridorEdge e) {
    if(!visible(r.network,from) || !visible(r.network,e.to)) return;
    r.forward[from].push_back(e);
    r.reverse[e.to].push_back({from,e.distance,e.transition,e.lateral});
}
void activate(CorridorRuntime& r,const MissionCorridorData& mission) {
    if(r.activeLeg==mission.activeLegIndex) return;
    r.activeLeg=mission.activeLegIndex;
    const auto* leg=activeMissionLeg(mission);
    r.allowed.assign(r.network.points.size(),false);
    r.trackingAllowed.assign(r.network.points.size(),false);
    r.surfaceAllowed.assign(r.network.points.size(),false);
    r.gatewayCost.assign(r.network.points.size(),inf);
    r.handoffCost.assign(r.network.points.size(),inf);
    r.gatewayLateralSteps.assign(r.network.points.size(),
                                 std::numeric_limits<int>::max());
    if(!leg) return;
    using Item=std::pair<double,std::int64_t>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    for(const auto& p:r.network.points) {
        const auto key=keyOf(p.pose);
        r.allowed[p.id]=visible(r.network,p.id) && (leg->allowedLanes.count(key)||leg->targetLanes.count(key));
        r.trackingAllowed[p.id]=r.allowed[p.id];
        r.surfaceAllowed[p.id]=(leg->allowedLanes.count(key)||leg->targetLanes.count(key)) &&
            r.network.lanes.at(key).topologyStatus!="excluded_by_review";
        if(r.allowed[p.id] && leg->targetLanes.count(key)) {
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
                r.gatewayCost[e.to]=next;queue.push({next,e.to});
            }
        }
    }
    // A separate 0-1 shortest path records how many lane changes remain.
    // Distance alone cannot tell a 30 m straight continuation from one legal
    // lateral stage, which previously rejected safe intermediate lanes.
    std::deque<std::int64_t> stepQueue;
    for(const auto& p:r.network.points) if(r.allowed[p.id] &&
        leg->targetLanes.count(keyOf(p.pose))) stepQueue.push_back(p.id);
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
            if(r.trackingAllowed[p.id] && nextLeg.targetLanes.count(key)) {
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
std::int64_t nearest(const CorridorRuntime& r,const WorldState& world) {
    std::int64_t result=-1;double score=inf;
    const int gx=static_cast<int>(std::floor(world.ego.x/10.));
    const int gy=static_cast<int>(std::floor(world.ego.y/10.));
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
        auto cell=r.spatial.find({gx+dx,gy+dy});if(cell==r.spatial.end()) continue;
        for(auto id:cell->second) {
            if(!r.allowed[id]) continue;
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
const CorridorEdge* forwardChoice(const CorridorRuntime& r,std::int64_t id,bool requireGateway=true) {
    const CorridorEdge* best=nullptr;double score=inf;
    for(const auto& e:r.forward[id]) {
        if(e.lateral || !r.allowed[e.to] || (requireGateway&&!std::isfinite(r.gatewayCost[e.to]))) continue;
        // Keep the current physical lane when it still leads to the gateway.
        const double value=(e.transition>=0?10000.:0.)+r.gatewayCost[e.to];
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
            const double value=(e.transition>=0?1.:0.)+r.handoffCost[e.to];
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
std::vector<Waypoint> forwardPath(const CorridorRuntime& r,std::int64_t start,double horizon,double startS) {
    std::vector<Waypoint> out;
    auto first=r.network.points[start].pose;first.s=startS;out.push_back(first);
    std::set<std::int64_t> seen;
    for(auto id=start;seen.insert(id).second && out.back().s-startS<horizon;) {
        const auto* e=forwardChoice(r,id);if(!e) break;
        if(e->transition>=0) {
            const auto& t=r.network.transitions[e->transition];
            for(const auto& p:t.points) {
                Waypoint q=r.network.points[t.toPoint].pose;
                q.x=p.x;q.y=p.y;q.yaw=p.yaw;q.roadS=p.roadS;
                append(out,q);
            }
        } else append(out,r.network.points[e->to].pose);
        id=e->to;
    }
    repairTrackingTangents(out);
    return out;
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
std::int64_t containing(const CorridorRuntime& r,double x,double y,double yaw) {
    const int gx=static_cast<int>(std::floor(x/10.)),gy=static_cast<int>(std::floor(y/10.));
    std::int64_t result=-1;double best=inf;
    const double c=std::cos(yaw),s=std::sin(yaw);
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
        const auto cell=r.spatial.find({gx+dx,gy+dy});if(cell==r.spatial.end()) continue;
        for(auto id:cell->second) {
            if(!r.surfaceAllowed[id]) continue;
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
                     std::vector<Waypoint>& path,const WorldState& world,std::string& reason) {
    const double width=std::max(1.88,std::hypot(world.egoObb.corners[0].x-world.egoObb.corners[1].x,
                                             world.egoObb.corners[0].y-world.egoObb.corners[1].y));
    std::int64_t previous=-1;
    for(auto& p:path) {
        const auto id=containing(r,p.x,p.y,p.yaw);
        if(id<0) {reason="center_outside_corridor";return false;}
        const auto& center=r.network.points[id].pose;
        if(previous>=0 && !boundaryAllowed(map,r.network.points[previous].pose,center)) {
            reason="solid_or_forbidden_lane_change";return false;
        }
        previous=id;
        p.roadId=center.roadId;p.sectionIndex=center.sectionIndex;p.laneId=center.laneId;p.roadS=center.roadS;
        const double c=std::cos(p.yaw),s=std::sin(p.yaw);
        for(double along=-1.055;along<=3.8+1e-6;along+=.4855) for(double side:{-width/2.,0.,width/2.}) {
            const auto corner=containing(r,p.x+along*c-side*s,p.y+along*s+side*c,p.yaw);
            if(corner<0) {reason="footprint_outside_corridor";return false;}
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
        const auto box=makeObb2D(p.x+1.3725*std::cos(p.yaw),p.y+1.3725*std::sin(p.yaw),p.yaw,4.855+2*margin,1.88+2*margin);
        for(const auto& o:world.objects) if(std::abs(o.raw.speed)<.3 && intersects(box,o.obb)) {
            if(objectId) *objectId=static_cast<int>(o.raw.id);
            return p.s-path.front().s;
        }
    }
    return inf;
}
bool collisionFree(const std::vector<Waypoint>& path,const WorldState& world,
                   const PlanningBehaviorConfig& config,const LaneChangeConfig& laneConfig) {
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
                       std::string* failure=nullptr) {
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
            const auto a=containing(r,path[first].x,path[first].y,path[first].yaw);
            const auto b=containing(r,path[last].x,path[last].y,path[last].yaw);
            if(a<0 || b<0) continue;
            auto end=r.network.points[b];end.pose=path[last];
            std::vector<Waypoint> replacement;double maxK{};std::string reason;
            if(!curve(replacement,path[first],r.network.points[a].curvature,end,&maxK) ||
               !surfaceAndMarks(r,map,replacement,world,reason)) continue;
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

bool buildCorridorTrackingPath(PathPlannerData& output,const PathPlannerConfig& config,
                              const WorldState& world,const PathGeneratorData& map,
                              const MissionCorridorData& mission,double horizon,DecisionTraceData* trace) {
    auto& r=*map.corridorRuntime;activate(r,mission);
    const auto id=nearest(r,world);if(id<0) return false;
    if(r.poseValid && !world.respawnDetected) {
        const double d=std::hypot(world.ego.x-r.previousX,world.ego.y-r.previousY);
        if(d>.15 && d<3.) r.curvature=.7*r.curvature+.3*std::clamp(normalizeAngle(world.ego.heading-r.previousYaw)/d,-maximumCurvature,maximumCurvature);
        if(d>.15) {r.previousX=world.ego.x;r.previousY=world.ego.y;r.previousYaw=world.ego.heading;}
    } else {r.previousX=world.ego.x;r.previousY=world.ego.y;r.previousYaw=world.ego.heading;r.curvature=r.network.points[id].curvature;r.poseValid=true;}
    output.config=config;
    output.waypoints=forwardPath(r,id,horizon,world.egoPathS);
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
                      "source_join_not_smoothable_inside_corridor "+joinFailure);return false;
    }
    // Behind samples prevent nearest/PP from selecting an unrelated branch;
    // preserving physical arc distance across road changes avoids tail loss.
    if(output.waypoints.size()<2) {
        traceDecision(trace,"corridor_lattice","tracking_unavailable","no forward point chain to gateway");return false;
    }
    return true;
}

bool prepareCorridorLattice(LaneChangeData& state,const LaneChangeConfig& config,
                           const PlanningBehaviorConfig& safetyConfig,const WorldState& world,
                           const PathGeneratorData& map,const MissionCorridorData& mission,
                           const std::vector<int>* confirmed,double* blockerDistance,
                           bool gatewayOnly,DecisionTraceData* trace) {
    if(blockerDistance) *blockerDistance=-1.;
    if(state.phase!=LaneChangePhase::None || !world.egoLaneValid || !world.egoPathValid) return false;
    auto& r=*map.corridorRuntime;activate(r,mission);
    const auto startId=nearest(r,world);if(startId<0) return false;
    auto base=forwardPath(r,startId,safetyConfig.localPathHorizon,world.egoPathS);
    if(!smoothSourceJoins(base,r,map,world)) return false;
    if(base.size()<2) return false;
    const WorldObjectState* blocker=nullptr;double blockerGap=inf;
    for(const auto& o:world.objects) {
        if(std::abs(o.raw.speed)>=.3 || (confirmed && std::find(confirmed->begin(),confirmed->end(),int(o.raw.id))==confirmed->end())) continue;
        for(const auto& p:base) {
            const double along=p.s-world.egoPathS;
            if(along>config.obstacleTriggerMaximumDistance || along>=blockerGap) break;
            const auto box=makeObb2D(p.x+1.3725*std::cos(p.yaw),p.y+1.3725*std::sin(p.yaw),p.yaw,4.855,1.88);
            if(intersects(box,o.obb)) {blocker=&o;blockerGap=along;break;}
        }
    }
    if(blockerDistance&&blocker) *blockerDistance=blockerGap;
    const double available=base.back().s-base.front().s;
    const auto* leg=activeMissionLeg(mission);
    const bool atGateway=leg&&leg->targetLanes.count(keyOf(base.back()));
    const int currentGatewaySteps=r.gatewayLateralSteps[startId];
    const double speed=std::abs(world.egoSpeed);
    const double prefix=speed*config.prepareTime+std::max(0.,(speed*speed-config.maximumLaneChangeSpeed*config.maximumLaneChangeSpeed)/4.);
    // Reserve one medium-distance manoeuvre for every lateral stage still
    // required. A single fixed 30 m deadline started a two-stage alignment so
    // late that the second shift reached the lane taper with no usable road.
    const double gatewayPlanningDistance=
        config.latticeMediumDistance*currentGatewaySteps+
        config.gatewayReserveDistance+prefix;
    const bool gatewayRequired=!atGateway && currentGatewaySteps>0 &&
        currentGatewaySteps<std::numeric_limits<int>::max() &&
        available<gatewayPlanningDistance;
    if(gatewayOnly ? !gatewayRequired : !blocker) return false;
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
    auto endpoints=targets(r,startId,horizons);
    double bestCost=inf,bestClearance=-1.,bestDistance=inf;
    int bestGatewaySteps=std::numeric_limits<int>::max();
    LaneChangeData best;bool found=false;
    std::map<std::string,int> rejected;
    std::size_t tried=0;
    for(const auto& [distance,id]:endpoints) {
        ++tried;
        const auto& target=r.network.points[id];traceCandidateLane(trace,keyOf(target.pose));
        // Obstacle avoidance has absolute priority. An escape lane must be
        // inside the corridor and collision-free now; it is not discarded
        // merely because this map leg has no known route from that lane to the
        // gateway. A safe gateway stage may be chained during avoidance, but
        // gateway reachability never hard-filters obstacle-mode candidates.
        if(gatewayOnly && !std::isfinite(r.gatewayCost[id])) {
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
        const double completion=candidate.back().s;
        auto tail=forwardPath(r,id,safetyConfig.localPathHorizon,completion);
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
        if(!collisionFree(safety,world,safetyConfig,config)) {++rejected["collision"];traceCandidatePath(trace,keyOf(target.pose),candidate,"rejected","collision");continue;}
        int followupObject=-1;
        // Obstacle clearance is evaluated on the untrimmed physical lane
        // continuation even when the controller tail stops at a mission-leg
        // boundary, so a terminal gateway never hides an obstacle.
        const double clearance=staticClearance(
            collisionTail,world,config.latticeObstacleClearance,
            &followupObject);
        const double hops=std::abs(-(target.pose.x-start.x)*std::sin(start.yaw)+(target.pose.y-start.y)*std::cos(start.yaw))/3.5;
        const double cost=config.latticeLaneCost*hops+config.latticeCurvatureCost*maxK+
            config.latticeProgressCost/(1.+std::min(80.,clearance));
        const bool free=clearance>=config.latticeShortDistance+config.gatewayReserveDistance;
        // A gateway manoeuvre is never allowed to point back into an obstacle
        // waiting just beyond its polynomial endpoint. Obstacle-mode planning
        // can still keep such a best-progress fallback and chain another
        // avoidance, but gateway alignment may use only a clear settling tail.
        if(gatewayOnly && !free) {
            ++rejected["gateway_tail_blocked"];continue;
        }
        const bool bestFree=bestClearance>=config.latticeShortDistance+config.gatewayReserveDistance;
        // For gateway stages, select the shortest safe longitudinal horizon.
        // Curvature is already hard-limited above; preferring the smoothest
        // 30 m curve consumed the road needed by the following gateway stage.
        const bool better=gatewayOnly
            ? (!found || distance<bestDistance-1.0 ||
               (std::abs(distance-bestDistance)<=1.0&&cost<bestCost))
            : (!found || (free&&!bestFree) || (free==bestFree &&
               (free?cost<bestCost:clearance>bestClearance+.5 ||
                (std::abs(clearance-bestClearance)<=.5&&cost<bestCost))));
        traceCandidatePath(trace,keyOf(target.pose),candidate,"viable","corridor_all_lanes_C2");
        if(!better) continue;
        for(const auto& p:tail) append(candidate,p);
        best={};best.phase=LaneChangePhase::Prepare;best.routeMode=LaneChangeRouteMode::KeepTargetLane;
        best.obstacleId=blocker?int(blocker->raw.id):-1;
        best.sourceRoadId=world.egoRoadId;best.sourceSectionIndex=world.egoSectionIndex;best.sourceLaneId=world.egoLaneId;
        best.targetLaneId=target.pose.laneId;best.reconnectLane=keyOf(target.pose);best.reconnectLaneValid=true;
        best.shiftCompletedLane=keyOf(target.pose);best.shiftCompletedLaneValid=true;
        best.startRouteS=world.egoPathS;best.rejoinRouteS=completion;best.roadConnectionManeuver=gatewayOnly;
        const double lateral=-(target.pose.x-start.x)*std::sin(start.yaw)+(target.pose.y-start.y)*std::cos(start.yaw);
        best.turnSignal=std::abs(lateral)<.2?0U:(lateral>0?1U:2U);best.outwardTurnSignal=best.turnSignal;
        best.path=std::move(candidate);best.bestProgressFallback=!free;
        best.followupObstacleId=followupObject;
        best.followupObstacleClearance=std::isfinite(clearance)?clearance:-1.;
        bestCost=cost;bestClearance=clearance;bestDistance=distance;
        bestGatewaySteps=r.gatewayLateralSteps[id];found=true;
    }
    std::ostringstream detail;detail<<"endpoints="<<tried<<" gateway_required="<<gatewayRequired
        <<" gateway_steps="<<currentGatewaySteps<<" blocker_gap="<<blockerGap;
    for(const auto& [reason,count]:rejected) detail<<" "<<reason<<"="<<count;
    if(found) {state=std::move(best);detail<<" cost="<<bestCost
        <<" selected_distance="<<bestDistance
        <<" target_gateway_steps="<<bestGatewaySteps
        <<" followup_clearance="<<bestClearance;
        traceCandidatePath(trace,state.shiftCompletedLane,state.path,"selected","corridor_all_lanes_C2");}
    traceDecision(trace,"corridor_lattice",found?"selected":"no_safe_candidate",detail.str());
    return found;
}
