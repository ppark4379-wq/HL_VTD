#include "planning/local/corridor_lattice.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/collision/collision_checker.hpp"
#include "logger/decision_trace.hpp"
#include "utils/geometry.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace {
void require(bool okay,const char* message) {
    if(!okay) throw std::runtime_error(message);
}
PathGeneratorData fixture(double elevation=0.,bool known=true,bool middleSolid=false) {
    PathGeneratorData map;PrecomputedLaneNetwork network;
    for(int rid=1;rid<=2;++rid) {
        Road road;road.id=rid;road.length=100.;
        Geometry geometry;geometry.length=100.;geometry.x=(rid-1)*100.;
        road.geometries.push_back(geometry);
        if(known) road.elevations.push_back({0.,10.+elevation-5.*(rid-1),-.05,0.,0.});
        LaneSection section;
        for(int lid=-1;lid>=-3;--lid) {
            LaneDefinition lane;lane.id=lid;lane.type="driving";
            lane.widths.push_back({0.,3.5,0.,0.,0.});
            lane.roadMarks.push_back({0.,middleSolid&&lid==-2?"none":"both",
                                     "white",middleSolid&&lid==-2?"solid":"broken"});
            section.lanes[lid]=lane;
            PrecomputedLane raw;raw.key={rid,0,lid};raw.topologyStatus="connected";
            LaneSegment segment;segment.key=raw.key;
            for(int i=0;i<=200;++i) {
                PrecomputedLanePoint p;p.id=network.points.size();p.pointIndex=i;
                p.width=3.5;p.usable=true;p.pose.x=geometry.x+i*.5;
                p.pose.y=(lid+.5)*3.5;p.pose.s=p.pose.roadS=i*.5;
                p.pose.roadId=rid;p.pose.laneId=lid;
                if(i<200) p.nextPoint=p.id+1;
                raw.pointIds.push_back(p.id);segment.points.push_back(p.pose);
                network.points.push_back(p);
            }
            network.lanes[raw.key]=raw;map.laneSegments[raw.key]=segment;
        }
        road.sections.push_back(section);map.roads[rid]=road;
    }
    for(auto& p:network.points) for(int side:{1,-1}) {
        const auto lane=network.lanes.find({p.pose.roadId,0,p.pose.laneId+side});
        if(lane==network.lanes.end()) continue;
        const auto id=lane->second.pointIds[p.pointIndex];
        if(side>0) {p.leftPoint=id;p.leftChangeAllowed=true;}
        else {p.rightPoint=id;p.rightChangeAllowed=true;}
    }
    for(int lid=-1;lid>=-3;--lid) {
        PrecomputedLaneConnection c;c.from={1,0,lid};c.to={2,0,lid};
        c.status="continuous";c.fromPoint=network.lanes[c.from].pointIds.back();
        c.toPoint=network.lanes[c.to].pointIds.front();network.connections.push_back(c);
        map.laneGraph[c.from].push_back(c.to);
    }
    std::string error;
    require(initializeCorridorRuntime(map,std::move(network),LaneChangeConfig{},&error),error.c_str());
    return map;
}
MissionCorridorData mission() {
    MissionCorridorData m;m.valid=m.initialized=m.topologyAuthority=true;
    m.checkpoints.resize(2);m.checkpoints[0].checkpoint.x=10.;
    m.checkpoints[1].checkpoint.x=180.;
    MissionLegData leg;leg.valid=leg.topologyAuthority=leg.finalLeg=true;
    leg.gatewayCheckpoint=leg.toCheckpoint=1;
    for(int rid=1;rid<=2;++rid) for(int lid=-1;lid>=-3;--lid) {
        leg.allowedLanes.insert({rid,0,lid});
        if(rid==2) {leg.targetLanes.insert({rid,0,lid});leg.targetRoadS[{rid,0,lid}]=80.;}
    }
    m.legs.push_back(leg);return m;
}
WorldState world(double elevation=0.,double x=10.) {
    WorldState w;w.ego.x=x;w.ego.y=-1.75;w.ego.z=10.+elevation-.05*x;
    w.egoSpeed=3.;w.inputValid=w.egoLaneValid=w.egoPathValid=true;
    w.egoRoadId=x<100.?1:2;w.egoLaneId=-1;
    w.egoRoadS=x<100.?x:x-100.;w.egoPathS=x;
    w.egoObb=makeObb2D(x+1.3725,-1.75,0.,4.855,1.88);return w;
}
WorldObjectState object(double x,double z,double length=4.,double width=2.) {
    WorldObjectState o;o.raw.id=7;o.raw.x=x;o.raw.y=-1.75;o.raw.z=z;
    o.raw.length=length;o.raw.width=width;o.raw.height=1.8;
    o.obb=makeObb2D(x,-1.75,0.,length,width);
    o.isAhead=o.isOnEgoPath=o.pathValid=true;o.relativeS=x-10.;o.relativeD=0.;
    o.ttc=1.;return o;
}
std::vector<Waypoint> straight(double end=180.) {
    std::vector<Waypoint> path;
    for(double x=0.;x<=end;x+=.5) {
        Waypoint p;p.x=p.s=x;p.y=-1.75;p.roadId=x<100.?1:2;
        p.laneId=-1;p.roadS=x<100.?x:x-100.;path.push_back(p);
    }
    return path;
}
void asyncScene(double elevation,bool known,bool otherDeck,bool compact) {
    auto map=fixture(elevation,known);auto m=mission();auto w=world(elevation);
    w.objects.push_back(object(130.,3.5+elevation+(otherDeck?20.:0.),
                               compact?.6:4.,compact?.7:2.));
    const auto originalCount=w.objects.size();
    auto& runtime=*map.corridorRuntime;PathPlannerData path;
    DecisionTraceData trace;trace.enabled=true;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
    do {
        beginCorridorSearchFrame(runtime);
        buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace);
        if(runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while(std::chrono::steady_clock::now()<deadline);
    require(runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested,
            "whole-corridor stationary-aware async route did not materialize");
    require(runtime.committedGlobalPath.back().x>179.,"global route missed gateway station");
    double maximumShift=0.;
    const auto body=vehicleFootprintFromObb(w.egoObb,{w.ego.x,w.ego.y,w.ego.heading});
    for(const auto& p:runtime.committedGlobalPath) {
        maximumShift=std::max(maximumShift,std::abs(p.y+1.75));
        if(!otherDeck || !known)
            require(!intersects(vehicleFootprintAt(body,{p.x,p.y,p.yaw},.3),w.objects.front().obb),
                    "search occupancy and assembled-edge collision masks disagree");
    }
    if(otherDeck && known) require(maximumShift<.1,"known other deck caused false detour");
    else require(maximumShift>.5,"same-road stationary box was not avoided");
    require(w.objects.size()==originalCount,"route level checks deleted received observations");
}
void handoffAndFreePrefix() {
    auto map=fixture();auto m=mission();auto w=world();PathPlannerData output;
    beginCorridorSearchFrame(*map.corridorRuntime);
    buildCorridorTrackingPath(output,PathPlannerConfig{},w,map,m,80.,nullptr);
    w.objects.push_back(object(45.,7.75));
    auto blocked=straight(100.);int blocker=-1;
    require(!corridorTrackingPathIsSafe(blocked,w,.05,&blocker,nullptr,&map) && blocker==7,
            "blocked global successor approved for completed avoidance");
    const double free=corridorStationaryFreeDistance(blocked,w,.05,&map,&blocker);
    require(free>20.&&free<32.&&blocker==7,"free-prefix distance includes consumed samples or ignores body");
    auto detached=w;detached.ego.y+=2.;
    detached.egoObb=makeObb2D(detached.ego.x+1.3725,detached.ego.y,0.,4.855,1.88);
    require(!corridorTrackingPathIsSafe(blocked,detached,.05,nullptr,nullptr,&map),
            "detached trajectory approved as a handoff");
    require(std::isfinite(corridorStationaryFreeDistance(blocked,detached,.05,&map,&blocker)) && blocker==7,
            "detached selected geometry bypassed known static obstacle speed guard");
    std::vector<Waypoint> safe;
    for(double x=10.;x<=100.;x+=.25) {
        const double t=std::clamp((x-10.)/25.,0.,1.);
        const double blend=t*t*t*(10.+t*(-15.+6.*t));
        const double derivative=(t>0.&&t<1.)?30.*t*t*(1.-t)*(1.-t)/25.:0.;
        Waypoint p;p.x=x;p.y=-1.75-3.5*blend;p.yaw=std::atan2(-3.5*derivative,1.);
        p.roadId=1;p.roadS=x;p.laneId=x<25.?-1:-2;
        p.s=safe.empty()?10.:safe.back().s+std::hypot(p.x-safe.back().x,p.y-safe.back().y);
        safe.push_back(p);
    }
    require(retainCorridorAvoidanceTail(map,m,w,safe),"safe avoidance tail was not retained");
    auto& r=*map.corridorRuntime;r.committedGlobalPath=blocked;
    r.committedGlobalPathLeg=m.activeLegIndex;r.committedGlobalPathProgress=20;
    r.committedGlobalPathReachesGateway=true;
    requestCorridorGlobalReplan(r);beginCorridorSearchFrame(r);
    require(buildCorridorTrackingPath(output,PathPlannerConfig{},w,map,m,80.,nullptr),
            "pending global replan discarded existing safe geometry");
    require(r.trackingPathApproved && corridorTrackingPathIsSafe(output.waypoints,w,.05,nullptr,nullptr,&map),
            "blocked global reference displaced safe avoidance continuation");
    w=world(0.,60.);w.objects.push_back(object(35.,8.25));
    require(std::isinf(corridorStationaryFreeDistance(straight(),w,.05,&map)),
            "already-passed stationary object still blocks unconsumed suffix");
}
void collisionLevels() {
    auto map=fixture();auto w=world();auto points=straight();
    LocalPathData local;local.valid=true;local.points=points;
    auto o=object(11.,29.45);w.objects={o};
    PathSafetyResult safety;resetPathSafety(safety);
    checkCurrentCollisions(safety,w.egoObb,w.objects,-1,&w,&map);
    checkPredictedCollisions(safety,w,local,4.,.1,true,&map);
    updateMinimumTTC(safety,w.objects,false,-1,&local,&map);
    require(!safety.collision&&!safety.predictedCollision&&std::isinf(safety.minimumTTC),
            "known other deck leaks into current/predicted/TTC safety");
    w.objects.front().raw.z=9.45;resetPathSafety(safety);
    checkCurrentCollisions(safety,w.egoObb,w.objects,-1,&w,&map);
    require(safety.collision,"same-road box dropped by current collision level check");
    w.objects={object(25.,8.75)};w.objects.front().raw.speed=1.;
    resetPathSafety(safety);
    checkPredictedCollisions(safety,w,local,8.,.1,true,&map);
    require(safety.predictedCollision,"same-grade moving actor prediction lost");
    auto unknown=fixture(0.,false);w.objects.front().raw.z=100.;
    require(!corridorTrackingPathIsSafe(points,w,.05,nullptr,nullptr,&unknown),
            "unknown elevation incorrectly treated as empty");
}
void acceptedTightMarginMatchesSpeedGuard() {
    auto map=fixture();auto w=world();auto path=straight(80.);
    auto adjacent=object(45.,7.75);
    adjacent.raw.y=-3.80;
    adjacent.obb=makeObb2D(45.,-3.80,0.,4.,2.);
    w.objects={adjacent};
    // The accepted .05m bypass is clear; a larger comfort envelope is not.
    w.ego.y+=.083;
    w.ego.heading=.0186;
    w.egoObb=makeObb2D(w.ego.x+1.3725*std::cos(w.ego.heading),
        w.ego.y+1.3725*std::sin(w.ego.heading),w.ego.heading,4.855,1.88);
    int blocker=-1;
    require(std::isinf(corridorStationaryFreeDistance(path,w,.05,&map,&blocker)) && blocker<0,
            "accepted tight bypass permanently inflated by current tracking error");
    require(std::isfinite(corridorStationaryFreeDistance(path,w,.30,&map,&blocker)) && blocker==7,
            "selected candidate clearance was not honored");
    // Removing the false future inflation must not hide real current contact.
    w.objects={object(w.ego.x,double(w.ego.z))};
    require(corridorStationaryFreeDistance(path,w,.05,&map,&blocker)==0. && blocker==7,
            "actual measured body contact escaped stationary guard");
}

void stationarySpeedGuardUsesApprovedEnvelope() {
    WorldState w;
    w.ego.heading=std::atan2(1.,1.);
    const double c=std::cos(w.ego.heading),s=std::sin(w.ego.heading);
    w.egoObb=makeObb2D(1.3725*c,1.3725*s,w.ego.heading,4.855,1.88);
    std::vector<Waypoint> path;
    for(double x=0.;x<=20.;x+=.2) {
        Waypoint p;p.x=p.y=x;p.yaw=w.ego.heading;p.s=x*std::sqrt(2.);
        path.push_back(p);
    }
    // A diagonal avoidance path passes a longitudinally parked vehicle.
    // Body + .30m is clear; a path-heading-triggered extra 1.5m is not.
    auto parked=object(10.,0.);
    parked.raw.y=4.5;parked.obb=makeObb2D(10.,4.5,0.,4.,2.);
    w.objects={parked};int blocker=-1;
    require(corridorTrackingPathIsSafe(path,w,.30),
            "diagonal stationary-clear fixture is not an approved path");
    require(std::isinf(corridorStationaryFreeDistance(path,w,.30,nullptr,&blocker)) && blocker<0,
            "stationary speed guard invented a larger envelope than path approval");
    // Preserve rejection of a real obstruction; path existence alone is not
    // a waiver of collision checking.
    w.objects.front().raw.y=10.;
    w.objects.front().obb=makeObb2D(10.,10.,0.,4.,2.);
    require(std::isfinite(corridorStationaryFreeDistance(path,w,.30,nullptr,&blocker)) && blocker==7,
            "real stationary obstruction escaped the selected-path guard");
}

bool awaitGlobalRoute(PathGeneratorData& map,const MissionCorridorData& m,
                      const WorldState& w,const char* name) {
    auto& r=*map.corridorRuntime;PathPlannerData tracking;
    DecisionTraceData trace;trace.enabled=true;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
    do {
        trace.frameEvents.clear();beginCorridorSearchFrame(r);
        buildCorridorTrackingPath(tracking,PathPlannerConfig{},w,map,m,80.,&trace);
        if(r.committedGlobalPathReachesGateway && !r.globalRouteReplanRequested)
            return true;
        // A fixed fixture cannot improve while waiting for the same failed
        // snapshot. Report the first completed search failure immediately.
        if(r.globalRouteSearchFailed) {
            std::cerr<<name<<": global search exhausted\n";
            for(const auto& event:trace.frameEvents)
                if(event.find("worker_detail")!=std::string::npos)
                    std::cerr<<event<<'\n';
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while(std::chrono::steady_clock::now()<deadline);
    throw std::runtime_error(std::string(name)+": async worker did not finish within 10s");
}

void requireGatewayAndWholeBodyClear(const PathGeneratorData& map,
                                     const MissionCorridorData& m,const WorldState& w) {
    const auto& path=map.corridorRuntime->committedGlobalPath;
    require(path.size()>2,"global route contains no usable trajectory");
    const auto& end=path.back();const auto& leg=m.legs.front();
    require(end.x>179. && leg.targetLanes.count({end.roadId,end.sectionIndex,end.laneId})!=0,
            "global route failed to terminate at a permitted gateway lane/station");
    const auto body=vehicleFootprintFromObb(w.egoObb,{w.ego.x,w.ego.y,w.ego.heading});
    const auto check=[&](double x,double y,double yaw) {
        const auto inflated=vehicleFootprintAt(body,{x,y,yaw},.3);
        for(const auto& o:w.objects)
            require(!intersects(inflated,o.obb),
                    "dense global route grazes an obstacle with actual body + 0.30m clearance");
        const auto physical=vehicleFootprintAt(body,{x,y,yaw});
        for(const auto& corner:physical.corners)
            require(corner.y>=-10.5-1e-6 && corner.y<=1e-6,
                    "dense global route leaves the three-lane physical road surface");
    };
    for(std::size_t i=0;i<path.size();++i) {
        const auto& p=path[i];check(p.x,p.y,p.yaw);
        if(!i) continue;
        const auto& before=path[i-1];
        // Also test between exported samples, not just graph vertices.
        const double yawDelta=std::atan2(std::sin(p.yaw-before.yaw),
                                         std::cos(p.yaw-before.yaw));
        const int steps=std::max(1,int(std::ceil(std::hypot(p.x-before.x,p.y-before.y)/.2)));
        for(int k=1;k<steps;++k) {
            const double t=double(k)/steps;
            check(before.x+t*(p.x-before.x),before.y+t*(p.y-before.y),
                  before.yaw+t*yawDelta);
        }
    }
}

void globalAlternativesDoNotChangeTrackingPolicy() {
    auto map=fixture();const auto& r=*map.corridorRuntime;
    bool medium=false,longer=false,nonAdjacent=false;
    for(const auto& p:r.network.points) {
        for(const auto& e:r.forward[p.id]) {
            if(!e.lateral) continue;
            require(std::abs(p.pose.laneId-r.network.points[e.to].pose.laneId)==1,
                    "non-adjacent global alternative leaked into gateway tracking policy");
            require(std::abs(e.distance-r.laneConfig.latticeShortDistance)<.51,
                    "long global alternative changed established short gateway policy");
        }
        for(const auto& e:r.globalLateral[p.id]) {
            medium=medium||std::abs(e.distance-r.laneConfig.latticeMediumDistance)<.51;
            longer=longer||std::abs(e.distance-r.laneConfig.latticeLongDistance)<.51;
            nonAdjacent=nonAdjacent||std::abs(p.pose.laneId-r.network.points[e.to].pose.laneId)>1;
        }
    }
    require(medium&&longer&&nonAdjacent,
            "global search lacks medium/long/non-adjacent legal lane alternatives");
}

void denseStaggeredGlobalRoute() {
    auto map=fixture();auto m=mission();auto w=world();
    // Three staggered rows leave alternating full-width escape lanes. The
    // scene requires several choices before the gateway, across a road-ID
    // boundary, rather than a single obstacle / one lane-change demonstration.
    const std::array<std::pair<double,int>,6> parked{{
        {50.,-1},{54.,-2},{110.,-3},{114.,-2},{160.,-2},{164.,-1}}};
    for(std::size_t i=0;i<parked.size();++i) {
        const auto [x,lane]=parked[i];auto o=object(x,10.-.05*x);
        o.raw.id=static_cast<std::uint32_t>(100+i);
        o.raw.y=(lane+.5)*3.5;
        o.obb=makeObb2D(x,o.raw.y,0.,o.raw.length,o.raw.width);
        w.objects.push_back(o);
    }
    require(awaitGlobalRoute(map,m,w,"dense staggered"),
            "staggered stationary rows did not produce a complete gateway route");
    requireGatewayAndWholeBodyClear(map,m,w);
}

void emptyRoadRemainsStraight() {
    auto map=fixture();auto m=mission();auto w=world();
    require(awaitGlobalRoute(map,m,w,"empty road"),"empty-road global route failed");
    requireGatewayAndWholeBodyClear(map,m,w);
    double lateralVariation=0.;
    const auto& path=map.corridorRuntime->committedGlobalPath;
    for(std::size_t i=0;i<path.size();++i) {
        require(std::abs(path[i].y+1.75)<.01,"empty-road route needlessly left its entry lane");
        if(i) lateralVariation+=std::abs(path[i].y-path[i-1].y);
    }
    require(lateralVariation<.01,"empty-road global route oscillates laterally");
}

void solidIntermediateBoundaryCannotBeSkipped() {
    auto map=fixture(0.,true,true);const auto& r=*map.corridorRuntime;
    for(const auto& p:r.network.points) {
        const auto check=[&](const CorridorEdge& edge) {
            if(!edge.lateral) return;
            const int target=r.network.points[edge.to].pose.laneId;
            require((p.pose.laneId==-3)==(target==-3),
                    "global/direct multi-lane edge skipped the intermediate solid boundary");
        };
        for(const auto& edge:r.forward[p.id]) check(edge);
        for(const auto& edge:r.globalLateral[p.id]) check(edge);
    }
    auto m=mission();auto w=world();auto& leg=m.legs.front();
    leg.targetLanes={{2,0,-3}};leg.targetRoadS.clear();leg.targetRoadS[{2,0,-3}]=80.;
    require(!awaitGlobalRoute(map,m,w,"solid-separated gateway (expected unreachable)"),
            "global route crossed a solid boundary to manufacture gateway reachability");
}

void denseGlobalRegressions() {
    globalAlternativesDoNotChangeTrackingPolicy();
    denseStaggeredGlobalRoute();emptyRoadRemainsStraight();
    solidIntermediateBoundaryCannotBeSkipped();
}
}
int main(int argc,char** argv) {
    try {
        if(argc==2 && std::string(argv[1])=="--dense-global") {
            denseGlobalRegressions();
            std::cout<<"dense global / unchanged tracking / empty-road / solid-boundary regressions PASS\n";
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--handoff-only") {
            handoffAndFreePrefix();
            std::cout<<"async safe-tail handoff regression PASS\n";return 0;
        }
        asyncScene(0.,true,false,false);
        asyncScene(20.,true,false,true);
        asyncScene(-20.,true,false,false);
        asyncScene(0.,true,true,false);
        asyncScene(0.,false,true,false);
        handoffAndFreePrefix();collisionLevels();acceptedTightMarginMatchesSpeedGuard();
        stationarySpeedGuardUsesApprovedEnvelope();
        denseGlobalRegressions();
        std::cout<<"global obstacle level/compact-box/handoff regressions PASS\n";
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
}
