#include "planning/local/corridor_lattice.hpp"
#include "planning/local/recovery_selection.hpp"
#include "planning/local/path_planner.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "logger/decision_trace.hpp"
#include "utils/geometry.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <array>
#include <thread>

void require(bool okay,const char* message) {if(!okay) throw std::runtime_error(message);}
PathGeneratorData fixture(bool solid=false,bool sourceKnot=false,
                          double translationX=0.,double translationY=0.,
                          const LaneChangeConfig& config=LaneChangeConfig{}) {
    PathGeneratorData map;PrecomputedLaneNetwork network;
    for(int rid=1;rid<=2;++rid) {
        Road road;road.id=rid;road.length=100.;
        Geometry geometry;geometry.length=100.;geometry.x=(rid-1)*100.+translationX;
        geometry.y=translationY;road.geometries.push_back(geometry);
        LaneSection section;
        for(int lid=-1;lid>=-3;--lid) {
            LaneDefinition lane;lane.id=lid;lane.type="driving";lane.widths.push_back({0.,3.5,0.,0.,0.});
            lane.roadMarks.push_back({0.,solid?"none":"both","white",solid?"solid":"broken"});section.lanes[lid]=lane;
            PrecomputedLane raw;raw.key={rid,0,lid};raw.topologyStatus="connected";
            LaneSegment segment;segment.key=raw.key;
            for(int i=0;i<=200;++i) {
                PrecomputedLanePoint p;p.id=network.points.size();p.pointIndex=i;p.width=3.5;p.usable=true;
                p.pose.x=(rid-1)*100.+i*.5+translationX;p.pose.y=(lid+.5)*3.5+translationY;p.pose.s=p.pose.roadS=i*.5;p.pose.roadId=rid;p.pose.laneId=lid;
                if(i<200) p.nextPoint=p.id+1;
                if(sourceKnot && i==100) p.nextPoint=-1;
                if(sourceKnot && i==101) p.pose.x-=.5;
                raw.pointIds.push_back(p.id);segment.points.push_back(p.pose);network.points.push_back(p);
            }
            network.lanes[raw.key]=raw;map.laneSegments[raw.key]=segment;
        }
        road.sections.push_back(section);map.roads[rid]=road;
    }
    for(auto& p:network.points) {
        for(int side:{1,-1}) {
            const LaneKey key{p.pose.roadId,0,p.pose.laneId+side};
            const auto lane=network.lanes.find(key);if(lane==network.lanes.end()) continue;
            auto id=lane->second.pointIds[p.pointIndex];
            if(side>0) {p.leftPoint=id;p.leftChangeAllowed=!solid;} else {p.rightPoint=id;p.rightChangeAllowed=!solid;}
        }
    }
    for(int lid=-1;lid>=-3;--lid) {
        PrecomputedLaneConnection c;c.from={1,0,lid};c.to={2,0,lid};c.status="continuous";
        c.fromPoint=network.lanes[c.from].pointIds.back();c.toPoint=network.lanes[c.to].pointIds.front();network.connections.push_back(c);
        map.laneGraph[c.from].push_back(c.to);
    }
    std::string error;require(initializeCorridorRuntime(map,std::move(network),config,&error),error.c_str());return map;
}
MissionCorridorData mission(bool onlyThird=false) {
    MissionCorridorData m;m.valid=m.initialized=true;MissionLegData leg;leg.valid=true;
    for(int rid=1;rid<=2;++rid) for(int lid=-1;lid>=-3;--lid) {
        if(onlyThird&&rid==2&&lid!=-3) continue;
        leg.allowedLanes.insert({rid,0,lid});if(rid==2&&(!onlyThird||lid==-3))leg.targetLanes.insert({rid,0,lid});
    }
    m.legs.push_back(leg);return m;
}
WorldState world(double x=10.) {
    WorldState w;w.ego.x=x;w.ego.y=-1.75;w.egoSpeed=3.;w.inputValid=w.egoLaneValid=w.egoPathValid=true;
    w.egoRoadId=1;w.egoLaneId=-1;w.egoRoadS=w.egoPathS=x;
    w.egoObb=makeObb2D(x+1.3725,-1.75,0.,4.855,1.88);return w;
}
void obstacle(WorldState& w) {
    WorldObjectState o;o.raw.id=7;o.raw.x=35.;o.raw.y=-1.75;o.raw.length=4.;o.raw.width=2.;
    o.obb=makeObb2D(35.,-1.75,0.,4.,2.);w.objects.push_back(o);
}
void m08StaticObjects(WorldState& w) {
    for(const auto& v:std::vector<std::array<double,4>>{{10,311.9199,-31.36795,-1.11447683},{11,308.00555,-30.785,-1.1179985},{9,309.1715,-25.9545,-1.12527636},{3,296.3041,-20.790875,-1.15131493},{2,291.685975,-19.517975,-1.16087225},{4,293.738925,-15.5779,-1.16919168},{8,289.57965,-13.649175,-1.17845256},{7,291.78735,-10.6143,-1.18343507}}) {
        WorldObjectState o;o.raw.id=v[0];o.raw.x=v[1];o.raw.y=v[2];o.raw.heading=v[3];o.raw.length=4.394;o.raw.width=1.808;
        o.obb=makeObb2D(v[1],v[2],v[3],4.394,1.808);w.objects.push_back(o);
    }
}
int main(int argc,char** argv) {
    try {
        if(argc==2 && std::string(argv[1])=="--clear-center-return") {
            auto map=fixture();auto m=mission();auto w=world(30.);
            PathPlannerData seed;
            require(buildCorridorTrackingPath(seed,PathPlannerConfig{},w,map,m,120.,nullptr),"initialize centre fixture");
            auto& runtime=*map.corridorRuntime;
            std::vector<Waypoint> path;
            for(int i=0;i<=240;++i) {
                Waypoint p;p.x=30.+i*.5;p.y=-3.5;p.s=p.x;p.roadId=p.x<100.?1:2;
                p.roadS=p.x<100.?p.x:p.x-100.;p.laneId=-1;
                const double u=std::clamp((p.x-70.)/20.,0.,1.);
                p.y+=1.75*u*u*u*(10.-15.*u+6.*u*u);
                p.yaw=std::atan(1.75*30.*u*u*(1.-u)*(1.-u)/20.);
                path.push_back(p);
            }
            const auto original=path;w.ego.y=-3.5;w.egoObb=makeObb2D(31.3725,-3.5,0.,4.855,1.88);
            w.receiveTime=10.;
            require(advanceClearLaneCenterReturn(path,w,map,m,PlanningBehaviorConfig{}),"clear offset failed to return to centre");
            const auto at=std::min_element(path.begin(),path.end(),[](const auto& a,const auto& b){return std::abs(a.x-50.)<std::abs(b.x-50.);});
            require(std::abs(at->y+1.75)<.1,"centre return stayed on dashed boundary");
            require(std::abs(path.front().y-w.ego.y)<1e-5 && std::abs(path.front().yaw-w.ego.heading)<1e-5,"return starts with a steering jump");
            for(const auto& p:original) if(p.x>=90.) {
                const auto q=std::find_if(path.begin(),path.end(),[&](const auto& a){return a.x==p.x && a.y==p.y && a.s==p.s && a.roadId==p.roadId && a.laneId==p.laneId;});
                require(q!=path.end(),"centring changed the original gateway tail");
            }
            w.receiveTime+=2.;const auto accepted=path;
            require(!advanceClearLaneCenterReturn(path,w,map,m,PlanningBehaviorConfig{}),"committed centre return was regenerated");
            require(path.size()==accepted.size(),"failed centre probe mutated its path");
            path=original;w.receiveTime+=2.;obstacle(w);
            w.objects.back().raw.length=.5;w.objects.back().raw.width=.3;
            w.objects.back().obb=makeObb2D(35.,-1.75,0.,.5,.3);
            require(corridorTrackingPathIsSafe(path,w,.3,nullptr,nullptr,&map),"fine offset fixture must already be safe");
            require(!advanceClearLaneCenterReturn(path,w,map,m,PlanningBehaviorConfig{}),"nearby obstacle did not preserve fine avoidance");
            require(path.size()==original.size() && path[20].y==original[20].y,"rejected return changed active avoidance");
            w.objects.clear();runtime.soleGatewayApproachSelected=true;w.receiveTime+=2.;
            require(!advanceClearLaneCenterReturn(path,w,map,m,PlanningBehaviorConfig{}),"queue approach was changed cosmetically");
            runtime.soleGatewayApproachSelected=false;
            auto solidMap=fixture(true);PathPlannerData solidSeed;
            auto sw=world(30.);buildCorridorTrackingPath(solidSeed,PathPlannerConfig{},sw,solidMap,m,120.,nullptr);
            sw=w;sw.receiveTime+=2.;
            require(!advanceClearLaneCenterReturn(path,sw,solidMap,m,PlanningBehaviorConfig{}),"centre return crossed a solid boundary");
            std::cout<<"clear centre return / unchanged gateway tail / dense and queue guards PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--cached-global-priority") {
            auto map=fixture();auto m=mission();auto w=world();
            m.topologyAuthority=true;m.checkpoints.resize(1);
            m.checkpoints[0].checkpoint.x=120.;
            m.checkpoints[0].checkpoint.y=-1.75;
            auto& leg=m.legs[0];leg.topologyAuthority=true;leg.finalLeg=true;
            for(const auto& lane:leg.targetLanes)leg.targetRoadS[lane]=20.;
            PathPlannerData tracking;DecisionTraceData trace;trace.enabled=true;
            require(buildCorridorTrackingPath(tracking,PathPlannerConfig{},w,map,m,80.,&trace),
                    "fixture could not create initial gateway route");
            auto& runtime=*map.corridorRuntime;
            require(runtime.committedGlobalPathReachesGateway &&
                    !runtime.globalRouteReplanRequested,
                    "fixture did not establish an idle committed global route");
            runtime.recoveryTrackingPath.clear();
            for(const auto& point:runtime.committedGlobalPath) {
                if(point.x>65.)break;
                runtime.recoveryTrackingPath.push_back(point);
            }
            runtime.recoveryTrackingPathLeg=m.activeLegIndex;
            runtime.recoveryTrackingPathProgress=0;
            runtime.recoveryTrackingPathClearance=.3;
            runtime.recoveryTrackingPathReachesReference=true;
            require(runtime.recoveryTrackingPath.size()>20 &&
                    corridorTrackingPathIsSafe(runtime.recoveryTrackingPath,w,.3,nullptr,nullptr,&map),
                    "fixture recovery prefix must remain attached and clear");
            const auto originalProgress=runtime.committedGlobalPathProgress;
            w=world(25.);w.receiveTime=1.;
            beginCorridorSearchFrame(runtime);
            require(buildCorridorTrackingPath(tracking,PathPlannerConfig{},w,map,m,80.,&trace),
                    "cached route became unavailable after normal forward motion");
            require(runtime.committedGlobalPathProgress>originalProgress &&
                    runtime.committedGlobalPath[runtime.committedGlobalPathProgress].x>=24.5,
                    "attached recovery prefix froze committed global progress");
            require(tracking.waypoints.back().x>runtime.recoveryTrackingPath.back().x+10.,
                    "short recovery prefix kept priority over the full safe global route");

            // The bridge still ends at x=65. A newly observed object at x=95
            // is outside that bridge but ON the stored full gateway route.
            // Global validation must see it before the bridge is exhausted.
            obstacle(w);w.objects.back().raw.x=95.;
            w.objects.back().obb=makeObb2D(95.,-1.75,0.,4.,2.);
            require(corridorTrackingPathIsSafe(runtime.recoveryTrackingPath,w,.3,nullptr,nullptr,&map),
                    "future obstacle unexpectedly intersects short recovery prefix");
            trace.frameEvents.clear();w.receiveTime+=.05;
            beginCorridorSearchFrame(runtime);
            require(buildCorridorTrackingPath(tracking,PathPlannerConfig{},w,map,m,80.,&trace),
                    "global obstacle recheck erased the safe recovery prefix");
            require(runtime.globalRouteReplanRequested &&
                    std::any_of(trace.frameEvents.begin(),trace.frameEvents.end(),[](const auto& event) {
                        return event.find("async_search_started")!=std::string::npos;
                    }),"safe short prefix suppressed a new obstacle on the global route");
            require(corridorTrackingPathIsSafe(tracking.waypoints,w,.3,nullptr,nullptr,&map),
                    "pending global search selected its blocked route instead of the safe prefix");
            std::cout<<"cached global route priority / live obstacle recheck PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--no-cosmetic-replan") {
            auto map=fixture();auto m=mission();auto w=world();
            w.ego.y=-3.5;
            w.egoObb=makeObb2D(w.ego.x+1.3725,w.ego.y,0.,4.855,1.88);
            std::vector<Waypoint> current;
            for(int i=0;i<=160;++i) {
                Waypoint p;p.x=p.s=p.roadS=10.+i*.5;p.y=w.ego.y;
                p.roadId=1;p.laneId=-1;current.push_back(p);
            }
            LaneChangeData replacement;
            for(int i=0;i<20;++i)
                require(!prepareLocalCorridorFallback(replacement,LaneChangeConfig{},PlanningBehaviorConfig{},
                    w,map,m,current,nullptr),"safe off-centre travel requested a cosmetic replacement");
            require(!map.corridorRuntime->localFallbackPending && !map.corridorRuntime->localFallbackEngaged,
                    "rollback left a cosmetic search worker active");
            require(replacement.phase==LaneChangePhase::None && replacement.path.empty(),
                    "rollback generated an unnecessary manoeuvre");
            require(current.front().y==-3.5 && current.back().y==-3.5,"incumbent path was mutated");
            // The selected route already bypasses the original lane blocker.
            // Seeing that same blocker on the unused straight lane must not
            // start another local search and replace a safe committed route.
            auto detourMap=fixture();auto dw=world();obstacle(dw);
            std::vector<Waypoint> detour;
            for(int i=0;i<=160;++i) {
                Waypoint p;p.x=10.+.5*i;p.roadS=p.x;p.roadId=1;
                const double u=std::clamp((p.x-10.)/20.,0.,1.);
                p.y=-1.75-3.5*(3.*u*u-2.*u*u*u);
                p.yaw=std::atan(-3.5*6.*u*(1.-u)/20.);
                p.laneId=u<.5?-1:-2;
                p.s=detour.empty()?0.:detour.back().s+std::hypot(p.x-detour.back().x,p.y-detour.back().y);
                detour.push_back(p);
            }
            require(corridorTrackingPathIsSafe(detour,dw,.3,nullptr,nullptr,&detourMap),
                    "fixture detour not safe");
            for(int i=0;i<20;++i)
                require(!prepareLocalCorridorFallback(replacement,LaneChangeConfig{},PlanningBehaviorConfig{},
                    dw,detourMap,m,detour,nullptr),"clear detour was replaced because straight lane is blocked");
            require(!detourMap.corridorRuntime->localFallbackEngaged &&
                    !detourMap.corridorRuntime->localFallbackPending,
                    "clear detour launched redundant search");
            // This retention must not suppress a new blocker ON the route.
            auto next=dw.objects.front();next.raw.id=8;next.raw.x=45.;next.raw.y=-5.25;
            next.obb=makeObb2D(45.,-5.25,0.,4.,2.);dw.objects.push_back(next);
            prepareLocalCorridorFallback(replacement,LaneChangeConfig{},PlanningBehaviorConfig{},
                dw,detourMap,m,detour,nullptr);
            require(detourMap.corridorRuntime->localFallbackEngaged,
                    "new obstacle on selected route did not activate local planning");
            std::cout<<"no cosmetic replanning PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--lost-corridor-recovery") {
            auto map=fixture();auto m=mission();auto w=world();
            m.legs[0].allowedLanes={{1,0,-2},{2,0,-2}};
            m.legs[0].targetLanes={{2,0,-2}};
            w.egoSpeed=0.;w.egoPathValid=false; // valid measured pose, missing trajectory
            map.corridorRuntime->laneConfig.localFallbackSearchBudgetMs=80.;
            map.corridorRuntime->laneConfig.localFallbackRetryInterval=0.;
            PathPlannerData tracking;DecisionTraceData trace;trace.enabled=true;
            bool recovered=false;
            const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
            while(std::chrono::steady_clock::now()<deadline) {
                buildCorridorTrackingPath(tracking,PathPlannerConfig{},w,map,m,80.,&trace);
                if(map.corridorRuntime->trackingPathApproved && map.corridorRuntime->trackingPathRelocalized) {
                    recovered=true;break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            if(!recovered)for(const auto& event:trace.frameEvents)std::cerr<<event<<'\n';
            require(recovered,"lost trajectory without an obstacle never triggered checked corridor reentry");
            require(tracking.waypoints.back().laneId==-2,"recovery chose the nearby wrong lane instead of official corridor");
            require(std::hypot(tracking.waypoints.front().x-w.ego.x,tracking.waypoints.front().y-w.ego.y)<.01,
                    "recovery directly tracked a detached gateway instead of connecting the measured pose");
            require(tracking.config.straightSpeed<=map.corridorRuntime->laneConfig.localFallbackSpeed,
                    "lost corridor return must be low speed");
            const auto wrong=map.corridorRuntime->network.lanes.at({1,0,-1}).pointIds[20];
            require(!map.corridorRuntime->allowed[wrong],"recovery rewrote official mission permissions");
            require(m.activeLegIndex==0,"recovery skipped the mission gateway");
            std::cout<<"lost corridor recovery PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--bounded-reentry") {
            auto map=fixture();auto m=mission();auto w=world();
            m.legs[0].allowedLanes={{1,0,-2},{2,0,-2}};
            m.legs[0].targetLanes={{2,0,-2}};
            w.ego.y=-5.25;w.egoLaneId=-2;w.egoSpeed=0.;
            w.egoObb=makeObb2D(w.ego.x+1.3725,w.ego.y,0.,4.855,1.88);
            obstacle(w);auto& obj=w.objects[0];obj.raw.y=-5.25;
            obj.raw.length=14.;obj.raw.width=3.;obj.raw.height=3.;
            obj.obb=makeObb2D(35.,-5.25,0.,14.,3.);
            std::vector<Waypoint> reference;
            for(int i=0;i<=160;++i) {
                Waypoint p;p.x=p.s=p.roadS=10.+i*.5;p.y=-5.25;p.roadId=1;p.laneId=-2;
                reference.push_back(p);
            }
            LaneChangeConfig config;config.localFallbackSearchBudgetMs=80.;
            config.localFallbackRetryInterval=0.;config.localFallbackHorizon=90.;
            DecisionTraceData trace;trace.enabled=true;
            LaneChangeData result;bool success=false;int highestTier=0;
            const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(8);
            while(std::chrono::steady_clock::now()<deadline) {
                success=prepareLocalCorridorFallback(result,config,PlanningBehaviorConfig{},
                    w,map,m,reference,&trace);
                highestTier=std::max(highestTier,map.corridorRuntime->localFallbackSearchTier);
                if(success) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            if(!success) for(const auto& event:trace.frameEvents) std::cerr<<event<<'\n';
            require(success,"bounded reentry failed around a long vehicle on a single-lane corridor");
            require(highestTier==2,"outside reentry did not wait for both strict search tiers");
            require(result.path.back().laneId==-2,"reentry did not return to the official lane");
            require(std::any_of(result.path.begin(),result.path.end(),[](const auto& p){return std::abs(p.y+5.25)>1.75;}),
                    "fixture did not exercise outside-corridor geometry");
            require(corridorTrackingPathIsSafe(result.path,w,result.validatedObstacleClearance,nullptr,nullptr,&map),
                    "bounded reentry intersected the long vehicle");
            const auto id=map.corridorRuntime->network.lanes.at({1,0,-1}).pointIds[40];
            require(!map.corridorRuntime->allowed[id] && !map.corridorRuntime->surfaceAllowed[id],
                    "local reentry leaked into global corridor authority");
            auto gateMap=fixture();auto gateMission=mission();
            auto& leg=gateMission.legs[0];leg.targetLanes={{1,0,-2}};
            leg.targetRoadS[LaneKey{1,0,-2}]=50.;leg.topologyAuthority=true;
            auto gateWorld=w;gateWorld.objects.clear();
            PathPlannerData tracking;
            buildCorridorTrackingPath(tracking,PathPlannerConfig{},gateWorld,gateMap,gateMission,80.,nullptr);
            CorridorReentryScope permission;
            permission.preview.resize(gateMap.corridorRuntime->allowed.size(),false);
            permission.outsideCorridor=true;
            std::vector<Waypoint> passed,missed;
            for(int i=0;i<=120;++i) {
                Waypoint p;p.x=p.s=p.roadS=10.+i*.5;p.y=-5.25;p.roadId=1;p.laneId=-2;
                passed.push_back(p);
                if(p.x<55.) {p.y=-1.75;p.laneId=-1;}
                missed.push_back(p);
            }
            require(corridorReentryPreservesMission(passed,*gateMap.corridorRuntime,gateMission,gateWorld,permission),
                    "normal gateway passage followed by a forward tail was rejected");
            require(!corridorReentryPreservesMission(missed,*gateMap.corridorRuntime,gateMission,gateWorld,permission),
                    "late return bypassed the required gateway cross section");
            std::cout<<"bounded reentry around long vehicle PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--progress-recovery-selection") {
            using namespace recovery_selection;
            const std::array<std::vector<std::size_t>,3> groups{{{0,1,2},{3},{4,5}}};
            require(balancedOrder(groups)==std::vector<std::size_t>({0,3,4,1,5,2}),
                    "directional families were not interleaved");
            require(better({false,false,1000.},{true,false,1.},true),
                    "cheap collision beat a collision-free path");
            require(!better({true,false,1.},{false,false,1000.},false),
                    "collision displaced a clear incumbent");
            require(better({true,false,1000.},{true,true,1.},true),
                    "terminal overlap beat an escape");
            require(!better({true,false,98.},{true,false,100.},true),
                    "minor cost jitter replaced the incumbent");
            require(better({true,false,90.},{true,false,100.},true),
                    "material improvement was suppressed");
            ProgressRecoveryData held;held.active=true;held.blockedTime=3.4;
            held.noMotionTime=2.;held.retryRemaining=.3;
            suspendForTrafficStop(held,1.,2.,true);
            require(!held.active && held.blockedTime==3.4 && held.noMotionTime==0. && held.retryRemaining==0.,
                    "traffic hold erased static history or left recovery active");
            suspendForTrafficStop(held,1.,2.,false);
            require(held.blockedTime==0.,"a pure traffic wait armed path recovery");
            auto map=fixture();auto m=mission();auto w=world();
            w.ego.y=-5.25;w.egoLaneId=-2;w.egoSpeed=0.;
            w.egoObb=makeObb2D(w.ego.x+1.3725,w.ego.y,0.,4.855,1.88);
            obstacle(w);w.objects[0].raw.y=-5.25;w.objects[0].raw.height=3.;
            w.objects[0].obb=makeObb2D(35.,-5.25,0.,4.,2.);
            PlanningBehaviorConfig behavior;behavior.simulationProgressRecovery=1.;
            behavior.progressRecoverySearchBudgetMs=1000.;
            PathPlannerData path;std::size_t cursor=0;bool contact=true,retained=false;
            DecisionTraceData trace;trace.enabled=true;
            require(prepareCorridorProgressRecovery(path,contact,cursor,w,map,m,
                LaneChangeConfig{},behavior,{},std::vector<int>{7},&trace),
                "recovery produced no path in an open adjacent lane");
            require(!contact,"recovery selected contact despite a clear adjacent lane");
            const auto reference=path.waypoints;
            require(prepareCorridorProgressRecovery(path,contact,cursor,w,map,m,
                LaneChangeConfig{},behavior,reference,std::vector<int>{7},&trace,
                std::numeric_limits<double>::quiet_NaN(),&retained),
                "incumbent rescore lost all paths");
            require(!contact && retained,"unchanged clear incumbent was not retained");
            // All paths begin inside this body. Unconfirmed and pedestrian-
            // sized actors must remain hard exclusions in contact recovery.
            auto& o=w.objects[0];o.raw.x=11.;o.raw.y=-5.25;
            o.raw.length=8.;o.raw.width=2.;o.obb=makeObb2D(11.,-5.25,0.,8.,2.);
            require(!prepareCorridorProgressRecovery(path,contact,cursor,w,map,m,
                LaneChangeConfig{},behavior,{},std::vector<int>{},&trace),
                "unconfirmed overlapping actor was allowed");
            o.raw.speed=1.;
            require(!prepareCorridorProgressRecovery(path,contact,cursor,w,map,m,
                LaneChangeConfig{},behavior,{},std::vector<int>{7},&trace),
                "moving actor with an outdated confirmed ID was allowed");
            o.raw.speed=0.;
            o.raw.length=1.;o.raw.width=1.;o.raw.height=1.8;
            o.obb=makeObb2D(11.,-5.25,0.,1.,1.);
            require(!prepareCorridorProgressRecovery(path,contact,cursor,w,map,m,
                LaneChangeConfig{},behavior,{},std::vector<int>{7},&trace),
                "pedestrian-sized overlapping actor was allowed");
            std::cout<<"progress recovery selection PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--surface-permission") {
            auto map=fixture();auto m=mission();auto w=world();
            auto& leg=m.legs.front();leg.allowedLanes.erase({1,0,-3});
            leg.surfaceOnlySpans.push_back({{1,0,-3},20.,40.,{1,0,-2}});
            PathPlannerData path;
            buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,60.,nullptr);
            const auto& runtime=*map.corridorRuntime;
            for(const auto id:runtime.network.lanes.at({1,0,-3}).pointIds) {
                const double station=runtime.network.points[id].pose.roadS;
                require(runtime.surfaceAllowed[id]==(station>=20.&&station<=40.),
                        "surface-only permission escaped its bounded interval");
                require(!runtime.allowed[id] && !runtime.trackingAllowed[id] &&
                        !std::isfinite(runtime.gatewayCost[id]),
                        "surface-only permission incorrectly opened route graph");
            }
            std::cout<<"bounded surface-only permission PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--alignment-deadline") {
            for(const double distance:{30.,5.}) {
                auto map=fixture();auto m=mission(true);auto w=world();
                LaneChangeData candidate;LaneAlignmentContext context;
                context.maximumCompletionDistance=distance;context.stopLineId=1;
                const bool chosen=prepareCorridorLattice(candidate,LaneChangeConfig{},
                    PlanningBehaviorConfig{},w,map,m,nullptr,nullptr,true,nullptr,
                    nullptr,false,&context);
                require(chosen==(distance>5.),"alignment deadline candidate feasibility");
                if(chosen) {
                    require(candidate.shiftCompletedLane.laneId==-3,
                            "short intermediate stage displaced full gateway alignment");
                    require(candidate.rejoinRouteS-w.egoPathS<=distance,
                            "lane alignment ends after the stop-line deadline");
                }
            }
            std::cout<<"multi-stage stop-line alignment PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--vehicle-footprint-width") {
            auto m=mission();
            for(const double width:{1.88,3.8}) {
                auto map=fixture();auto w=world();obstacle(w);
                w.egoObb=makeObb2D(w.ego.x+1.3725,w.ego.y,0.,4.855,width);
                LaneChangeData candidate;
                const bool chosen=prepareCorridorLattice(
                    candidate,LaneChangeConfig{},PlanningBehaviorConfig{},w,
                    map,m,nullptr,nullptr,false,nullptr);
                require(chosen==(width<3.5),
                        "lattice target/surface eligibility ignored configured ego width");
            }
            std::cout<<"configured lattice vehicle width PASS\n";
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--extended-gateway-preview") {
            const auto run=[](bool observed,double dx,double dy) {
                LaneChangeConfig config;config.localFallbackHorizon=90.;
                auto map=fixture(false,false,dx,dy,config);auto m=mission();
                m.topologyAuthority=true;m.checkpoints.resize(4);
                m.checkpoints[0].checkpoint.x=90.+dx;
                m.checkpoints[1].checkpoint.x=99.+dx;
                m.checkpoints[2].checkpoint.x=102.+dx;
                m.checkpoints[3].checkpoint.x=190.+dx;
                auto& leg=m.legs.front();leg.topologyAuthority=true;leg.finalLeg=false;
                leg.gatewayCheckpoint=1;leg.toCheckpoint=2;
                leg.gatewayLanes={{1,0,-1}};leg.targetLanes={{2,0,-3}};
                leg.targetRoadS[{2,0,-3}]=2.;
                MissionLegData next;next.valid=next.topologyAuthority=next.finalLeg=true;
                next.fromCheckpoint=2;next.gatewayCheckpoint=next.toCheckpoint=3;
                next.allowedLanes={{2,0,-1},{2,0,-2},{2,0,-3}};
                next.targetLanes={{2,0,-3}};next.targetRoadS[{2,0,-3}]=90.;
                m.legs.push_back(next);
                auto w=world(110.);w.ego.x+=dx;w.ego.y+=dy;
                w.egoRoadId=2;w.egoRoadS=10.;
                w.egoObb=makeObb2D(w.ego.x+1.3725,w.ego.y,0.,4.855,1.88);
                // Two long parked vehicles close both left lanes beyond the
                // old 30 m preview. The free outer lane must be held until
                // the whole body passes them, then return to the required lane.
                for(int lane:{-2,-3}) {
                    WorldObjectState o;o.raw.id=20-lane;o.raw.x=143.+dx;
                    o.raw.y=(lane+.5)*3.5+dy;o.raw.length=40.;o.raw.width=2.;
                    o.obb=makeObb2D(o.raw.x,o.raw.y,0.,o.raw.length,o.raw.width);
                    w.objects.push_back(o);
                }
                PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
                auto& runtime=*map.corridorRuntime;
                if(observed) {
                    auto previous=w;previous.ego.x=90.+dx;previous.egoRoadId=1;
                    previous.egoRoadS=previous.egoPathS=90.;
                    previous.egoObb=makeObb2D(previous.ego.x+1.3725,previous.ego.y,0.,4.855,1.88);
                    // Use an actual measured gateway pose to seed passage;
                    // do not set the runtime permission bit directly.
                    beginCorridorSearchFrame(runtime);
                    buildCorridorTrackingPath(path,PathPlannerConfig{},previous,map,m,120.,&trace);
                }
                const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
                do {
                    w.receiveTime+=.05;beginCorridorSearchFrame(runtime);
                    buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,120.,&trace);
                    if(runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested)break;
                    if(runtime.globalRouteSearchFailed)break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } while(std::chrono::steady_clock::now()<deadline);
                const bool success=runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested;
                if(!observed) {
                    require(!success,"unobserved earlier gateway was silently skipped");
                    return;
                }
                if(!success)for(const auto& event:trace.frameEvents)std::cerr<<event<<'\n';
                require(success,"observed gateway could not extend past the long obstacle cluster");
                require(m.activeLegIndex==0,"preview advanced mission without measured target passage");
                const auto& route=runtime.committedGlobalPath;
                require(missionPointIsTarget(m.legs[0],route.back()),"extended route never returned to the required exit lane");
                require(route.back().x>185.+dx,"global preview still terminates inside the obstacle cluster");
                const auto body=vehicleFootprintFromObb(w.egoObb,{w.ego.x,w.ego.y,w.ego.heading});
                for(const auto& p:route)for(const auto& o:w.objects)
                    require(!intersects(vehicleFootprintAt(body,{p.x,p.y,p.yaw},.3),o.obb),
                            "extended gateway route touches a parked vehicle");
                const auto firstTarget=std::find_if(route.begin(),route.end(),[](const auto& p){return p.roadId==2 && p.laneId==-3;});
                require(firstTarget!=route.end() && firstTarget->roadS>60.,
                        "required exit-lane return happened before the long vehicles were passed");
            };
            run(false,0.,0.);run(true,0.,0.);run(true,230.,-120.);
            std::cout<<"observed gateway / extended obstacle preview / translated geometry PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--center-first-global") {
            const auto run=[](bool singleLane) {
                auto map=fixture();auto m=mission();auto w=world();
                m.topologyAuthority=true;m.checkpoints.resize(2);
                m.checkpoints[0].checkpoint.x=10.;m.checkpoints[1].checkpoint.x=180.;
                auto& leg=m.legs.front();leg.topologyAuthority=true;leg.finalLeg=true;
                leg.gatewayCheckpoint=leg.toCheckpoint=1;
                if(singleLane) {
                    leg.allowedLanes={{1,0,-1},{2,0,-1}};
                    leg.targetLanes={{2,0,-1}};
                }
                for(const auto& lane:leg.targetLanes) leg.targetRoadS[lane]=80.;
                obstacle(w);auto& o=w.objects.front();
                // A small obstruction touches the lane-centre envelope.
                // A full adjacent-lane detour and a cheaper in-lane shift
                // both fit; prefer the centre route when it is available.
                o.raw.x=55.;o.raw.y=-3.;o.raw.width=1.;
                o.obb=makeObb2D(o.raw.x,o.raw.y,0.,o.raw.length,o.raw.width);
                PathPlannerData tracking;DecisionTraceData trace;trace.enabled=true;
                auto& runtime=*map.corridorRuntime;
                const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
                do {
                    beginCorridorSearchFrame(runtime);
                    buildCorridorTrackingPath(tracking,PathPlannerConfig{},w,map,m,80.,&trace);
                    if(runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested) break;
                    if(runtime.globalRouteSearchFailed) break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } while(std::chrono::steady_clock::now()<deadline);
                if(!runtime.committedGlobalPathReachesGateway)
                    for(const auto& event:trace.frameEvents)std::cerr<<event<<'\n';
                require(runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested,
                        "centre-first/offset-fallback fixture has no complete gateway route");
                require(corridorTrackingPathIsSafe(runtime.committedGlobalPath,w,.3,nullptr,nullptr,&map),
                        "centre-first/offset-fallback route intersects the obstruction");
                require(missionPointIsTarget(leg,runtime.committedGlobalPath.back()),
                        "centre-first/offset-fallback route missed required gateway");
                if(singleLane) {
                    require(std::all_of(runtime.committedGlobalPath.begin(),runtime.committedGlobalPath.end(),
                        [](const auto& p){return p.laneId==-1;}),"offset fallback left its only permitted lane");
                    require(std::any_of(runtime.committedGlobalPath.begin(),runtime.committedGlobalPath.end(),
                        [](const auto& p){return p.y>-1.2;}),"failed centre search did not use needed fine offset");
                } else {
                    require(std::any_of(runtime.committedGlobalPath.begin(),runtime.committedGlobalPath.end(),
                        [](const auto& p){return p.y<-5.2;}),"cheap off-centre shift displaced feasible lane-centre detour");
                }
            };
            run(false);run(true);
            std::cout<<"centre-first global / fine offset fallback PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--observed-global-replan") {
            auto map=fixture();auto m=mission();auto w=world(10.);
            m.topologyAuthority=true;m.checkpoints.resize(2);
            m.checkpoints[0].checkpoint.x=10.;m.checkpoints[1].checkpoint.x=180.;
            auto& leg=m.legs.front();leg.topologyAuthority=true;leg.finalLeg=true;
            leg.gatewayCheckpoint=leg.toCheckpoint=1;
            for(const auto& lane:leg.targetLanes) leg.targetRoadS[lane]=80.;
            obstacle(w);w.objects.front().raw.x=130.;
            w.objects.front().obb=makeObb2D(130.,-1.75,0.,4.,2.);
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            auto& runtime=*map.corridorRuntime;
            const auto poll=[&]() {
                const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
                do {
                    beginCorridorSearchFrame(runtime);
                    buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace);
                    if(runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested)
                        return true;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } while(std::chrono::steady_clock::now()<deadline);
                return false;
            };
            require(poll(),"asynchronous observed-obstacle global route missing");
            require(runtime.committedGlobalPath.back().x>=179.5,
                    "route ended at exit lane start instead of finish station");
            for(const auto& p:runtime.committedGlobalPath)
                require(!intersects(makeObb2D(p.x+1.3725*std::cos(p.yaw),
                    p.y+1.3725*std::sin(p.yaw),p.yaw,4.855+.6,1.88+.6),w.objects.front().obb),
                    "global route ignored a received static obstacle beyond 70m");
            w=world(150.);w.ego.y=-8.75;w.egoRoadId=2;w.egoLaneId=-3;w.egoRoadS=50.;
            requestCorridorGlobalReplan(runtime);
            require(!runtime.committedGlobalPath.empty(),"replan erased the old trajectory prematurely");
            require(poll(),"post-avoidance gateway global route missing");
            require(std::hypot(runtime.committedGlobalPath.front().x-w.ego.x,
                               runtime.committedGlobalPath.front().y-w.ego.y)<1.,
                    "post-avoidance global route starts on the old lane");
            std::cout<<"observed whole-corridor / post-avoidance replan PASS\n";
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--multi-gateway-queue") {
            auto map=fixture();auto m=mission();auto w=world(70.);
            m.topologyAuthority=true;m.checkpoints.resize(2);
            m.checkpoints[1].checkpoint.x=140.;m.checkpoints[1].checkpoint.y=-3.5;
            auto& leg=m.legs[0];leg.topologyAuthority=true;leg.finalLeg=true;
            leg.gatewayCheckpoint=leg.toCheckpoint=1;
            leg.gatewayLanes=leg.targetLanes={{2,0,-1},{2,0,-2}};
            leg.allowedLanes.erase({2,0,-3});
            for(const auto& key:leg.targetLanes)leg.targetRoadS[key]=80.;
            for(int lane:{-1,-2}) {
                WorldObjectState o;o.raw.id=-lane;o.raw.x=140.;o.raw.y=(lane+.5)*3.5;
                o.raw.length=12.;o.raw.width=3.5;
                o.obb=makeObb2D(o.raw.x,o.raw.y,0.,12.,3.5);w.objects.push_back(o);
            }
            auto& r=*map.corridorRuntime;PathPlannerData path;
            const auto poll=[&](bool approach) {
                const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(5);
                do {
                    w.receiveTime+=.05;beginCorridorSearchFrame(r);
                    buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,160.,nullptr);
                    if(approach?r.soleGatewayApproachSelected:
                        (r.committedGlobalPathReachesGateway&&!r.globalRouteReplanRequested))return true;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }while(std::chrono::steady_clock::now()<until);
                return false;
            };
            require(poll(true),"all blocked gateway alternatives lost safe queue approach");
            require(corridorTrackingPathIsSafe(path.waypoints,w,.3,nullptr,nullptr,&map),
                    "multiple-gateway queue approach intersects a blocker");
            require(path.waypoints.back().laneId==-1,"unnecessary lane change to equivalent queue");
            w.objects.pop_back();requestCorridorGlobalReplan(r);
            require(poll(false),"opening one gateway did not restore a clear full route");
            require(r.soleGatewayApproachPath.empty(),"blocked queue outranked open gateway");
            require(corridorTrackingPathIsSafe(r.committedGlobalPath,w,.3,nullptr,nullptr,&map),
                    "open-gateway route intersects remaining queue");
            std::cout<<"multiple gateways: safe queue / open exit priority PASS\n";return 0;
        }
        if(argc==2 && std::string(argv[1])=="--sole-gateway-approach") {
            auto map=fixture();auto m=mission(true);auto w=world(45.);
            m.topologyAuthority=true;m.checkpoints.resize(2);
            m.checkpoints[0].checkpoint.x=45.;m.checkpoints[1].checkpoint.x=140.;
            m.checkpoints[1].checkpoint.y=-8.75;
            auto& leg=m.legs.front();leg.topologyAuthority=true;leg.finalLeg=true;
            leg.gatewayCheckpoint=leg.toCheckpoint=1;leg.gatewayLanes={{2,0,-3}};
            leg.targetRoadS[{2,0,-3}]=80.;
            obstacle(w);auto& o=w.objects.front();o.raw.x=135.;o.raw.y=-8.75;
            o.obb=makeObb2D(o.raw.x,o.raw.y,0.,4.,2.);
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            auto& runtime=*map.corridorRuntime;
            const auto poll=[&](bool approach) {
                const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
                do {
                    w.receiveTime+=.05;beginCorridorSearchFrame(runtime);
                    buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,160.,&trace);
                    if(approach?runtime.soleGatewayApproachSelected:
                        (runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested)) return true;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } while(std::chrono::steady_clock::now()<deadline);
                for(const auto& event:trace.frameEvents)std::cerr<<event<<'\n';
                return false;
            };
            require(poll(true),"blocked sole gateway produced no safe approach");
            require(!runtime.committedGlobalPathReachesGateway,"approach falsely completed the gateway");
            require(runtime.trackingPathApproved,"approach not approved for actual tracking");
            require(path.waypoints.back().x<132.,"approach penetrates the blocked gateway");
            require(corridorTrackingPathIsSafe(path.waypoints,w,.3,nullptr,nullptr,&map),"approach intersects actor");
            w.objects.clear();requestCorridorGlobalReplan(runtime);
            require(poll(false),"opening the gate did not restore the complete route");
            require(runtime.soleGatewayApproachPath.empty(),"stale approach masked the full route");
            // A detour around the same gateway actor exists in the adjacent
            // lane. The complete collision-free route MUST win over approach.
            auto alternateMap=fixture();auto alternate=mission();auto aw=world(45.);
            alternate.topologyAuthority=true;alternate.checkpoints=m.checkpoints;
            auto& al=alternate.legs.front();al.topologyAuthority=true;al.finalLeg=true;
            al.gatewayCheckpoint=al.toCheckpoint=1;al.gatewayLanes={{2,0,-3}};
            al.targetLanes={{2,0,-3}};al.targetRoadS[{2,0,-3}]=80.;
            obstacle(aw);aw.objects.front().raw.x=135.;aw.objects.front().raw.y=-8.75;
            aw.objects.front().obb=makeObb2D(135.,-8.75,0.,4.,2.);
            auto& ar=*alternateMap.corridorRuntime;
            const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(5);
            do {
                aw.receiveTime+=.05;beginCorridorSearchFrame(ar);
                buildCorridorTrackingPath(path,PathPlannerConfig{},aw,alternateMap,alternate,160.,&trace);
                require(!ar.soleGatewayApproachSelected,"partial approach beat a valid complete detour");
                if(ar.committedGlobalPathReachesGateway && !ar.globalRouteReplanRequested)break;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            } while(std::chrono::steady_clock::now()<until);
            require(ar.committedGlobalPathReachesGateway,"available alternate-lane detour was not found");
            require(corridorTrackingPathIsSafe(ar.committedGlobalPath,aw,.3,nullptr,nullptr,&alternateMap),
                    "preferred full detour collides with gateway blocker");
            // A blocked sole exit must be approached IN its lane, not by
            // truncating a late diagonal merge beside its stationary queue.
            auto queuedMap=fixture();auto qm=alternate;auto qw=world(70.);
            // Production laneGraph contains legal lateral links too. These
            // are searchable APPROACH edges, never proof of an entry lane.
            for(int road:{1,2}) for(int lane:{-1,-2}) {
                queuedMap.laneGraph[{road,0,lane}].push_back({road,0,lane-1});
                queuedMap.laneGraph[{road,0,lane-1}].push_back({road,0,lane});
            }
            qm.checkpoints[1].checkpoint.x=190.;
            WorldObjectState queue;queue.raw.id=101;queue.raw.x=190.;
            queue.raw.y=-8.75;queue.raw.length=24.;queue.raw.width=3.5;
            queue.obb=makeObb2D(190.,-8.75,0.,24.,3.5);qw.objects={queue};
            auto parallel=queue;parallel.raw.id=102;parallel.raw.x=184.;parallel.raw.y=-1.75;
            parallel.obb=makeObb2D(184.,-1.75,0.,24.,3.5);qw.objects.push_back(parallel);
            auto& qr=*queuedMap.corridorRuntime;
            const auto queueDeadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
            do {
                qw.receiveTime+=.05;beginCorridorSearchFrame(qr);
                buildCorridorTrackingPath(path,PathPlannerConfig{},qw,queuedMap,qm,160.,&trace);
                if(qr.soleGatewayApproachSelected)break;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }while(std::chrono::steady_clock::now()<queueDeadline);
            require(qr.soleGatewayApproachSelected,"entrance queue approach absent");
            require(!qr.soleGatewayApproachPath.empty() &&
                    qr.soleGatewayApproachPath.back().laneId==-3 &&
                    std::abs(qr.soleGatewayApproachPath.back().y+8.75)<.1,
                    "reachable entrance queue was replaced by waiting in a parallel lane");
            require(corridorTrackingPathIsSafe(qr.soleGatewayApproachPath,qw,.3,nullptr,nullptr,&queuedMap),
                    "gateway-lane approach ignored a real obstacle");
            std::cout<<"sole gateway: safe approach / no false completion / opening replan PASS\n";return 0;
        }
        if(argc==3 && std::string(argv[1])=="--precomputed-mission-audit") {
            PathGeneratorData map;
            require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            MissionCorridorData m;std::string error;
            const bool loaded=loadPrecomputedMission(m,map,argv[2],&error);
            require(loaded,error.c_str());
            require(m.topologyAuthority,"new artifact was not made authoritative");
            for(std::size_t i=0;i<m.legs.size();++i) {
                m.activeLegIndex=i;
                const auto& leg=m.legs[i];
                require(!missionLegRequiresGuideContinuity(m),"new mission uses legacy guide");
                for(const auto& target:leg.targetLanes) {
                    auto trial=m;trial.targetRoadStableFrames=0;
                    auto pose=map.laneSegments.at(target).points.front();
                    pose.roadS=leg.targetRoadS.at(target);
                    require(missionPointIsTarget(leg,pose),"target station not recognized");
                    auto before=pose;before.roadS+=(target.laneId<0?-1.:1.);
                    require(!missionPointIsTarget(leg,before),"gateway advances before exit station");
                    auto w=world();w.egoLaneValid=true;
                    w.egoRoadId=target.roadId;w.egoSectionIndex=target.sectionIndex;
                    w.egoLaneId=target.laneId;w.egoRoadS=pose.roadS;
                    // Deliberately far from CP: road/station, not CP radius,
                    // must govern progression; world normally comes from map matching.
                    w.ego.x=pose.x;w.ego.y=pose.y;
                    updateMissionCorridorProgress(trial,w);
                    require(trial.activeLegIndex==i,"single-frame match advanced gateway");
                    updateMissionCorridorProgress(trial,w);
                    require(trial.activeLegIndex==i+1,"directed gateway handoff failed");
                }
            }
            std::cout<<"precomputed mission audit PASS legs="<<m.legs.size()<<'\n';
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m15-exact-gateway-target") {
            PathGeneratorData map;
            require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),
                    "map load");
            std::string error;
            require(initializeCorridorRuntime(
                        map,"output/precomputed_lane_network",
                        LaneChangeConfig{},&error),error.c_str());
            // Minimal copy of the two M15 runtime legs around the failed fork.
            // Both connector lanes are physically usable, but only 2960/-3
            // lands on the artifact's handoff target 2809/-2.  The parallel
            // 2960/-2 lane lands on 2809/-1 -> 2969 and cannot make the next
            // required 2975 -> 2811 turn.
            MissionCorridorData m;
            m.valid=m.initialized=m.topologyAuthority=true;
            MissionLegData leg;leg.valid=leg.topologyAuthority=true;
            leg.allowedLanes={{2805,0,1},{2807,0,1},{2808,0,1},
                {2809,0,-2},{2809,0,-1},{2942,0,-1},{2948,0,-1},
                {2960,0,-3},{2960,0,-2}};
            leg.gatewayLanes={{2960,0,-3},{2960,0,-2}};
            leg.targetLanes={{2809,0,-2}};
            leg.targetRoadS[{2809,0,-2}]=2.;
            MissionLegData next;next.valid=next.topologyAuthority=true;
            next.allowedLanes={{2809,0,-2},{2809,0,-1},{2975,0,-1},
                {2811,0,-1},{2811,1,-1}};
            next.gatewayLanes={{2975,0,-1}};
            next.targetLanes={{2811,0,-1}};
            next.targetRoadS[{2811,0,-1}]=2.;
            m.legs={leg,next};

            Waypoint correctExit;correctExit.roadId=2809;
            correctExit.laneId=-2;correctExit.roadS=2.;
            auto wrongExit=correctExit;wrongExit.laneId=-1;
            require(missionPointIsTarget(m.legs.front(),correctExit),
                    "M15 exact handoff lane was not recognized");
            require(!missionPointIsTarget(m.legs.front(),wrongExit),
                    "M15 wrong parallel exit lane completed the gateway");

            auto w=world();
            w.ego.x=1315.67041;w.ego.y=542.813416;
            w.ego.heading=3.0952878;w.egoSpeed=.000185045864;
            w.egoPathS=435.06275;
            const auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;
            w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            w.egoLaneWidth=match.laneWidth;
            w.egoObb=makeObb2D(
                w.ego.x+1.3725*std::cos(w.ego.heading),
                w.ego.y+1.3725*std::sin(w.ego.heading),
                w.ego.heading,4.855,1.88);
            LaneChangeData candidate;DecisionTraceData trace;trace.enabled=true;
            const bool chosen=prepareCorridorLattice(
                candidate,LaneChangeConfig{},PlanningBehaviorConfig{},w,map,m,
                nullptr,nullptr,true,&trace);
            const auto& runtime=*map.corridorRuntime;
            const auto stepsAt=[&](const LaneKey& key,double roadS) {
                const auto& ids=runtime.network.lanes.at(key).pointIds;
                const auto found=std::min_element(
                    ids.begin(),ids.end(),[&](auto a,auto b) {
                        return std::abs(runtime.network.points[a].pose.roadS-roadS)<
                               std::abs(runtime.network.points[b].pose.roadS-roadS);
                    });
                return runtime.gatewayLateralSteps.at(*found);
            };
            require(!chosen,"correct M15 connector requested a needless lane change");
            require(stepsAt({2960,0,-3},5.)==0,
                    "correct M15 connector lost its direct handoff");
            require(stepsAt({2960,0,-2},5.)>0,
                    "wrong M15 connector was still a zero-step gateway");

            PathPlannerData route;
            require(buildCorridorTrackingPath(
                        route,PathPlannerConfig{},w,map,m,80.,&trace),
                    "M15 exact gateway route was not materialized");
            const auto exit=std::find_if(
                route.waypoints.begin(),route.waypoints.end(),
                [](const auto& point){return point.roadId==2809;});
            require(exit!=route.waypoints.end() && exit->laneId==-2,
                    "M15 route selected the 2809/-1 -> 2969 fork");
            std::cout<<"M15 exact gateway target / wrong parallel fork PASS\n";
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m14-guide-recovery") {
            PathGeneratorData map;
            require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),
                    "map load");
            std::string error;
            require(initializeCorridorRuntime(
                        map,"output/precomputed_lane_network",
                        LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;
            require(initializeMissionCorridor(
                        m,map,"scenario/mixed/M14/checkpoints.csv"),
                    "mission load");
            m.activeLegIndex=2;
            std::vector<Waypoint> reference;
            require(loadWaypointsCsv(
                        reference,"scenario/mixed/M14/route.csv"),
                    "reference load");
            auto w=world();
            // Exact first Settle pose after the M14 avoidance (rt25b frame
            // 5195).  The guide is still safely reachable here, 2.87 m away;
            // failing to bridge now lets the long manoeuvre tail take the
            // irreversible wrong fork roughly 150 m later.
            w.ego.x=1095.38623;w.ego.y=-704.912231;
            w.ego.heading=-4.07526588;w.egoSpeed=5.42657157;
            w.egoPathS=984.470194;
            const auto match=worldToLane(
                map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;
            w.egoSectionIndex=match.sectionIndex;
            w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            w.egoObb=makeObb2D(
                w.ego.x+1.3725*std::cos(w.ego.heading),
                w.ego.y+1.3725*std::sin(w.ego.heading),
                w.ego.heading,4.855,1.88);
            PathPlannerData recovered;DecisionTraceData trace;
            trace.enabled=true;
            const bool okay=buildReferenceRecoveryTrackingPath(
                recovered,PathPlannerConfig{},reference,2003,w,map,m,80.,
                &trace);
            for(const auto& event:trace.frameEvents) std::cout<<event<<'\n';
            std::cout<<"M14 guide recovery="<<okay
                     <<" points="<<recovered.waypoints.size()
                     <<" lane="<<w.egoRoadId<<':'<<w.egoSectionIndex<<':'
                     <<w.egoLaneId<<'\n';
            require(okay,"M14 detached guide recovery was not generated");
            require(recovered.waypoints.size()>20,
                    "M14 detached guide recovery has no forward horizon");
            return 0;
        }
        if(argc==2 && (std::string(argv[1])=="--m11-full-static-route" ||
                       std::string(argv[1])=="--m11-fast-static-route")) {
            const bool realtime=
                std::string(argv[1])=="--m11-fast-static-route";
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M11/checkpoints.csv"),"mission load");
            m.activeLegIndex=2;
            auto w=world();w.ego.x=1460.19153;w.ego.y=552.366028;
            w.ego.heading=4.7542;w.egoSpeed=2.78;w.egoPathS=246.21;
            const auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;
            w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            w.egoObb=makeObb2D(w.ego.x+1.3725*std::cos(w.ego.heading),w.ego.y+1.3725*std::sin(w.ego.heading),w.ego.heading,4.855,1.88);
            int objectId=2;
            for(const auto& pose:std::vector<std::array<double,3>>{
                    {1463.7791748046875,539.5462036132812,4.754200667397316},
                    {1462.48583984375,480.6123962402344,4.7541975565867585},
                    {1466.3792724609375,473.5582275390625,4.7541975565867585},
                    {1462.71484375,473.5582275390625,4.7541975565867585},
                    {1469.9522705078125,460.9156799316406,4.7541975565867585},
                    {1459.7513427734375,467.7730407714844,4.7541975565867585},
                    {1463.0521240234375,467.23468017578125,4.7541975565867585},
                    {1463.680419921875,453.8100280761719,4.7541975565867585}}) {
                WorldObjectState object;object.raw.id=objectId++;
                object.raw.x=pose[0];object.raw.y=pose[1];
                object.raw.heading=pose[2];object.raw.length=4.394;
                object.raw.width=1.808;
                object.obb=makeObb2D(pose[0],pose[1],pose[2],4.394,1.808);
                w.objects.push_back(object);
            }
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            if(realtime) beginCorridorSearchFrame(*map.corridorRuntime);
            const auto start=std::chrono::steady_clock::now();
            const bool okay=buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,250.,&trace);
            for(const auto& event:trace.frameEvents) std::cout<<event<<'\n';
            std::cout<<"M11 global route okay="<<okay
                     <<" points="<<path.waypoints.size()
                     <<" ms="<<std::chrono::duration<double,std::milli>(
                         std::chrono::steady_clock::now()-start).count()<<'\n';
            require(okay,"M11 complete static obstacle route missing");
            require(map.corridorRuntime->committedGlobalPathReachesGateway,
                    "M11 route did not reach gateway");
            for(const auto& point:path.waypoints) for(const auto& object:w.objects)
                require(!intersects(makeObb2D(
                    point.x+1.3725*std::cos(point.yaw),
                    point.y+1.3725*std::sin(point.yaw),point.yaw,
                    4.855+.6,1.88+.6),object.obb),
                    "M11 global route intersects a stationary obstacle");
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m11-transition-commitment") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M11/checkpoints.csv"),"mission load");
            auto& runtime=*map.corridorRuntime;
            int transitionId=-1;
            for(std::size_t i=0;i<runtime.network.transitions.size();++i) {
                const auto& t=runtime.network.transitions[i];
                if(t.kind=="split" && t.eventLane.roadId==2812 &&
                   runtime.network.points[t.toPoint].pose.laneId==-2) {
                    transitionId=static_cast<int>(i);break;
                }
            }
            require(transitionId>=0,"M11 split transition missing");
            const auto& transition=runtime.network.transitions[transitionId];
            auto w=world();
            const auto& entry=runtime.network.points[transition.fromPoint].pose;
            w.ego.x=entry.x;w.ego.y=entry.y;w.ego.heading=entry.yaw;
            w.egoRoadId=entry.roadId;w.egoSectionIndex=entry.sectionIndex;w.egoLaneId=entry.laneId;
            w.egoPathS=100.;w.egoRoadS=entry.roadS;
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace),"M11 entry tracking failed");
            require(runtime.committedTransition==transitionId,"M11 split was not committed");
            const std::size_t middle=transition.points.size()/3;
            const auto& p=transition.points[middle];
            w.ego.x=p.x;w.ego.y=p.y;w.ego.heading=p.yaw;w.egoPathS=108.;w.egoRoadS=p.roadS;
            const auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;w.egoLaneId=match.laneId;
            require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace),"M11 transition interior tracking failed");
            const auto nearest=std::min_element(path.waypoints.begin(),path.waypoints.end(),
                [&](const auto& a,const auto& b) {
                    return std::hypot(a.x-w.ego.x,a.y-w.ego.y)<
                           std::hypot(b.x-w.ego.x,b.y-w.ego.y);
                });
            const double remaining=path.waypoints.back().s-nearest->s;
            require(nearest->roadId==match.roadId &&
                    nearest->sectionIndex==match.sectionIndex &&
                    nearest->laneId==match.laneId,
                    "M11 transition sample carries the wrong physical lane metadata");
            std::cout<<"transition="<<transitionId<<" samples="<<transition.points.size()
                     <<" map_lane="<<match.roadId<<':'<<match.sectionIndex<<':'<<match.laneId
                     <<" path_points="<<path.waypoints.size()<<" remaining="<<remaining<<'\n';
            require(remaining>20.,"M11 committed transition lost its forward tail");
            require(runtime.committedTransition==transitionId,"M11 transition commitment was lost inside curve");
            const auto junctionSplit=std::find_if(
                runtime.network.transitions.begin(),runtime.network.transitions.end(),
                [](const auto& t) {
                    return t.kind=="split" && t.eventLane==LaneKey{3027,0,-2};
                });
            require(junctionSplit!=runtime.network.transitions.end(),
                    "M11 zero-width junction-road split was not precomputed");
            require(junctionSplit->vehicleFootprintValid &&
                    !junctionSplit->crossesSolid &&
                    !junctionSplit->violatesLaneChangePermission,
                    "M11 junction-road split failed physical/road-mark validation");
            require(runtime.network.lanes.at({3027,0,-2}).topologyStatus=="connected",
                    "M11 junction branch remained no_entry after transition generation");
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--all-scenario-handoff-audit") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            std::size_t audited=0;
            for(int scenario=5;scenario<=15;++scenario) {
                const std::string name="M"+(scenario<10?std::string("0"):std::string())+std::to_string(scenario);
                MissionCorridorData m;
                require(initializeMissionCorridor(
                            m,map,"scenario/mixed/"+name+"/checkpoints.csv"),
                        "mission load");
                for(std::size_t leg=0;leg+1<m.legs.size();++leg) {
                    m.activeLegIndex=leg;
                    const LaneKey seedKey=*m.legs[leg].allowedLanes.begin();
                    const auto seedLane=map.corridorRuntime->network.lanes.find(seedKey);
                    require(seedLane!=map.corridorRuntime->network.lanes.end(),
                            "corridor seed lane missing");
                    const auto seedId=seedLane->second.pointIds.front();
                    const auto& seed=map.corridorRuntime->network.points[seedId].pose;
                    auto seedWorld=world();seedWorld.ego.x=seed.x;seedWorld.ego.y=seed.y;
                    seedWorld.ego.heading=seed.yaw;seedWorld.egoPathS=0.;
                    seedWorld.egoRoadId=seed.roadId;
                    seedWorld.egoSectionIndex=seed.sectionIndex;
                    seedWorld.egoLaneId=seed.laneId;
                    PathPlannerData ignored;
                    buildCorridorTrackingPath(
                        ignored,PathPlannerConfig{},seedWorld,map,m,80.,nullptr);
                    auto& runtime=*map.corridorRuntime;
                    std::set<std::pair<LaneKey,LaneKey>> checked;
                    for(const auto& p:runtime.network.points) {
                        if(p.id<0 || p.id>=static_cast<std::int64_t>(runtime.allowed.size()) ||
                           !runtime.allowed[p.id] || runtime.gatewayCost[p.id]>1e-8)
                            continue;
                        for(const auto& edge:runtime.forward[p.id]) {
                            if(edge.lateral || runtime.allowed[edge.to] ||
                               !runtime.trackingAllowed[edge.to]) continue;
                            const auto pair=std::make_pair(
                                LaneKey{p.pose.roadId,p.pose.sectionIndex,p.pose.laneId},
                                LaneKey{runtime.network.points[edge.to].pose.roadId,
                                        runtime.network.points[edge.to].pose.sectionIndex,
                                        runtime.network.points[edge.to].pose.laneId});
                            if(!checked.insert(pair).second) continue;
                            WorldState w=world();w.ego.x=p.pose.x;w.ego.y=p.pose.y;
                            const auto& nextPose=runtime.network.points[edge.to].pose;
                            w.ego.heading=p.pose.yaw;
                            // Derive the drivable boundary tangent from the
                            // next spatially distinct precomputed point.  The
                            // coincident endpoint itself may be an XODR
                            // one-sided derivative probe.
                            std::int64_t tangentId=edge.to;
                            std::set<std::int64_t> tangentSeen;
                            while(tangentSeen.insert(tangentId).second) {
                                const auto nextEdge=std::find_if(
                                    runtime.forward[tangentId].begin(),
                                    runtime.forward[tangentId].end(),
                                    [](const CorridorEdge& candidate) {
                                        return !candidate.lateral;
                                    });
                                if(nextEdge==runtime.forward[tangentId].end()) break;
                                tangentId=nextEdge->to;
                                const auto& tangentPose=
                                    runtime.network.points[tangentId].pose;
                                const double dx=tangentPose.x-nextPose.x;
                                const double dy=tangentPose.y-nextPose.y;
                                if(std::hypot(dx,dy)>=0.02) {
                                    w.ego.heading=std::atan2(dy,dx);
                                    break;
                                }
                            }
                            w.egoPathS=100.;
                            w.egoRoadId=p.pose.roadId;
                            w.egoSectionIndex=p.pose.sectionIndex;
                            w.egoLaneId=p.pose.laneId;
                            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
                            if(!buildCorridorTrackingPath(
                                    path,PathPlannerConfig{},w,map,m,80.,&trace) ||
                               path.waypoints.size()<2) {
                                for(const auto& event:trace.frameEvents)
                                    std::cerr<<event<<'\n';
                                std::cerr<<name<<" leg "<<leg
                                         <<" empty handoff "
                                         <<pair.first.roadId<<':'
                                         <<pair.first.sectionIndex<<':'
                                         <<pair.first.laneId<<" -> "
                                         <<pair.second.roadId<<':'
                                         <<pair.second.sectionIndex<<':'
                                         <<pair.second.laneId
                                         <<" source_handoff_cost="
                                         <<runtime.handoffCost[p.id]
                                         <<" target_handoff_cost="
                                         <<runtime.handoffCost[edge.to]<<'\n';
                                std::abort();
                            }
                            require(corridorHasLongitudinalConnection(
                                        map,pair.first,pair.second),
                                    "handoff topology rejected a precomputed connection");
                            ++audited;
                        }
                    }
                }
            }
            require(audited>0,"no cross-leg corridor handoffs audited");
            std::cout<<"all-scenario handoffs audited="<<audited<<'\n';
            return 0;
        }
        if(argc>=2 && std::string(argv[1])=="--m09-dense-obstacle-replay") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M09/checkpoints.csv"),"mission load");
            m.activeLegIndex=3;
            auto w=world();w.ego.x=867.603271;w.ego.y=-843.49231;w.ego.heading=-2.47250891;
            w.egoSpeed=2.77887861;w.egoPathS=564.657011;
            if(argc==5) {w.ego.x=std::stod(argv[2]);w.ego.y=std::stod(argv[3]);w.ego.heading=std::stod(argv[4]);}
            const auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            w.egoObb=makeObb2D(w.ego.x+1.3725*std::cos(w.ego.heading),w.ego.y+1.3725*std::sin(w.ego.heading),w.ego.heading,4.855,1.88);
            // Preserved M09 OBB centres/dimensions, not production exceptions.
            for(const auto& v:std::vector<std::array<double,6>>{
                {11,820.61125,-880.5578,-2.4928,4.394,1.808},
                {3,815.65717,-884.31207,-2.4758,4.394,1.808},
                {2,852.3479,-851.4160,-2.4775,4.394,1.808},
                {16,821.0730,-875.6166,.66903,.9,.9},
                {17,803.3130,-890.12365,0.,2.16,2.43}}) {
                WorldObjectState o;o.raw.id=v[0];o.raw.x=v[1];o.raw.y=v[2];o.raw.heading=v[3];o.raw.length=v[4];o.raw.width=v[5];
                o.obb=makeObb2D(v[1],v[2],v[3],v[4],v[5]);w.objects.push_back(o);
            }
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace),"dense tracking missing");
            LaneChangeData maneuver;
            const bool chosen=prepareCorridorLattice(maneuver,LaneChangeConfig{},PlanningBehaviorConfig{},w,map,m,nullptr,nullptr,false,&trace);
            for(const auto& e:trace.frameEvents)std::cout<<e<<'\n';
            for(const auto& p:trace.candidatePaths)std::cout<<p.lane.roadId<<':'<<p.lane.laneId<<' '<<p.status<<' '<<p.reason<<'\n';
            require(chosen,"dense obstacle candidate missing");
            std::cout<<"chosen lane="<<maneuver.targetLaneId<<" followup="<<maneuver.followupObstacleId<<" clearance="<<maneuver.followupObstacleClearance<<'\n';
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m09-compact-obstacle-replay") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M09/checkpoints.csv"),"mission load");
            m.activeLegIndex=3;
            auto w=world();w.ego.x=831.778442;w.ego.y=-867.7677;
            w.ego.heading=-2.47277164;w.egoSpeed=2.77696145;
            w.egoPathS=610.60809;
            const auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;
            w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            w.egoLaneWidth=match.laneWidth;
            w.egoObb=makeObb2D(w.ego.x+1.3725*std::cos(w.ego.heading),
                               w.ego.y+1.3725*std::sin(w.ego.heading),
                               w.ego.heading,4.855,1.88);
            WorldObjectState cone;cone.raw.id=16;cone.raw.x=821.072998;
            cone.raw.y=-875.616638;cone.raw.heading=.669031215;
            cone.raw.length=.9;cone.raw.width=.9;cone.raw.height=.9;
            cone.obb=makeObb2D(cone.raw.x,cone.raw.y,cone.raw.heading,
                               cone.raw.length,cone.raw.width);
            w.objects.push_back(cone);
            // A parked full-size vehicle occupies the adjacent lane only
            // 9.6 m beyond the cone.  The valid passage is the narrow slot
            // between both objects, not an unconstrained lane change.
            WorldObjectState parked;parked.raw.id=3;
            parked.raw.x=815.6571655;parked.raw.y=-884.3120728;
            parked.raw.heading=-2.47580265;parked.raw.length=4.394;
            parked.raw.width=1.808;parked.raw.height=1.45;
            parked.obb=makeObb2D(
                parked.raw.x,parked.raw.y,parked.raw.heading,
                parked.raw.length,parked.raw.width);
            w.objects.push_back(parked);
            const std::vector<int> confirmed{16};
            LaneChangeData maneuver;double blockerDistance=-1.;
            DecisionTraceData trace;trace.enabled=true;
            const bool selected=prepareCorridorLattice(
                maneuver,LaneChangeConfig{},PlanningBehaviorConfig{},
                w,map,m,&confirmed,&blockerDistance,false,&trace);
            for(const auto& event:trace.frameEvents) std::cout<<event<<'\n';
            require(selected,
                    "M09 compact static obstacle avoidance missing");
            require(blockerDistance<=15.,
                    "compact obstacle was not triggered in mandatory range");
            for(const auto& point:maneuver.path) for(const auto& object:w.objects)
                require(!intersects(makeObb2D(
                            point.x+1.3725*std::cos(point.yaw),
                            point.y+1.3725*std::sin(point.yaw),point.yaw,
                            4.855,1.88),object.obb),
                        "M09 compact avoidance physically intersects an object");
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m09-exhausted-maneuver-replay") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M09/checkpoints.csv"),"mission load");
            m.activeLegIndex=1;
            auto w=world();w.ego.x=647.829346;w.ego.y=-741.896851;
            w.ego.heading=.67775166;w.egoSpeed=0.;w.egoPathS=177.450938;
            const auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;
            w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace),
                    "M09 exhausted manoeuvre returned an empty path");
            require(path.waypoints.back().s-path.waypoints.front().s>20.,
                    "M09 retained a spent or short path instead of continuing");
            require(std::any_of(path.waypoints.begin(),path.waypoints.end(),
                               [](const auto& p){return p.roadId==2007&&p.laneId==-2;}),
                    "M09 precomputed longitudinal continuation was ignored");
            std::cout<<"M09 recovery points="<<path.waypoints.size()<<'\n';
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m08-corridor-handoff-replay") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M08/checkpoints.csv"),"mission load");
            // Exact first invalid frame from m08_rviz.  Leg 3 ends on
            // 418:0:-4 while the validated merge continues into leg 4.
            m.activeLegIndex=3;
            auto w=world();w.ego.x=420.005768;w.ego.y=-175.280945;
            w.ego.heading=-4.03696775;w.egoSpeed=7.53142951;
            w.egoPathS=649.293299;
            const auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;
            w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace),
                    "M08 corridor handoff returned an empty path");
            require(path.waypoints.size()>20,"M08 handoff path has no usable horizon");
            require(corridorHasLongitudinalConnection(
                        map,{418,0,-4},{418,1,-4}),
                    "precomputed M08 merge rejected by tracking topology");
            bool enteredNextLeg=false;
            for(const auto& p:path.waypoints)
                enteredNextLeg|=p.roadId==418&&p.sectionIndex==1&&p.laneId==-4;
            require(enteredNextLeg,"M08 path did not search the next-leg lanes");
            std::cout<<"M08 handoff points="<<path.waypoints.size()
                     <<" end="<<path.waypoints.back().roadId<<':'
                     <<path.waypoints.back().sectionIndex<<':'
                     <<path.waypoints.back().laneId<<'\n';
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m08-final-gateway-replay") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M08/checkpoints.csv"),"mission load");
            auto w=world();w.ego.x=310.989655;w.ego.y=-38.799324;w.ego.heading=5.50819445;w.egoSpeed=3.58191634;w.egoPathS=200.425;
            auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            w.egoObb=makeObb2D(w.ego.x+1.3725*std::cos(w.ego.heading),w.ego.y+1.3725*std::sin(w.ego.heading),w.ego.heading,4.855,1.88);
            m08StaticObjects(w);
            LaneChangeData maneuver;DecisionTraceData trace;trace.enabled=true;
            const bool chosen=prepareCorridorLattice(maneuver,LaneChangeConfig{},PlanningBehaviorConfig{},w,map,m,nullptr,nullptr,true,&trace);
            for(const auto& e:trace.frameEvents)std::cout<<e<<'\n';
            require(chosen,"M08 final gateway candidate rejected at the mission-leg tail");
            require(maneuver.roadConnectionManeuver,"final gateway replay is not a gateway maneuver");
            require(maneuver.reconnectLaneValid,"final gateway target key missing");
            std::cout<<"final_gateway_target="<<maneuver.reconnectLane.roadId<<':'
                     <<maneuver.reconnectLane.sectionIndex<<':'
                     <<maneuver.reconnectLane.laneId<<'\n';
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m08-obstacle-replay") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M08/checkpoints.csv"),"mission load");
            auto w=world();w.ego.x=284.093353;w.ego.y=8.27479362;w.ego.heading=5.07411194;w.egoSpeed=4.1297519;w.egoPathS=144.668542;
            auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            w.egoObb=makeObb2D(w.ego.x+1.3725*std::cos(w.ego.heading),w.ego.y+1.3725*std::sin(w.ego.heading),w.ego.heading,4.855,1.88);
            m08StaticObjects(w);
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace),"M08 tracking missing");
            LaneChangeData maneuver;double blockerDistance=-1.;
            const bool chosen=prepareCorridorLattice(maneuver,LaneChangeConfig{},PlanningBehaviorConfig{},w,map,m,nullptr,&blockerDistance,false,&trace);
            for(const auto& e:trace.frameEvents)std::cout<<e<<'\n';
            std::cout<<"M08 obstacle selected="<<chosen<<" blocker_distance="<<blockerDistance<<'\n';
            require(chosen,"M08 obstacle avoidance candidate missing");
            require(!maneuver.roadConnectionManeuver,"obstacle maneuver incorrectly became gateway maneuver");
            for(const auto& event:trace.frameEvents)
                require(event.find("gateway_unreachable")==std::string::npos,"gateway rejected an obstacle escape candidate");
            require(maneuver.followupObstacleId>=0 || !maneuver.bestProgressFallback,
                    "fallback did not preserve its next blocker for an early chained avoidance");
            std::cout<<"target_lane="<<maneuver.targetLaneId
                     <<" target_key="<<maneuver.reconnectLane.roadId<<':'
                     <<maneuver.reconnectLane.sectionIndex<<':'
                     <<maneuver.reconnectLane.laneId
                     <<" followup_id="<<maneuver.followupObstacleId
                     <<" followup_clearance="<<maneuver.followupObstacleClearance
                     <<" completion_s="<<maneuver.rejoinRouteS
                     <<" path_points="<<maneuver.path.size()<<'\n';
            return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m08-gateway-replay") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M08/checkpoints.csv"),"mission load");
            auto w=world();w.ego.x=300.424622;w.ego.y=-22.6708355;w.ego.heading=5.14076233;w.egoSpeed=4.16663035;w.egoPathS=177.978018;
            auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            w.egoObb=makeObb2D(w.ego.x+1.3725*std::cos(w.ego.heading),w.ego.y+1.3725*std::sin(w.ego.heading),w.ego.heading,4.855,1.88);
            // Stationary object OBBs from the preserved M08 trace. Moving
            // object 12 is intentionally not reconstructed from a later frame.
            m08StaticObjects(w);
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace),"M08 tracking missing");
            LaneChangeData maneuver;
            const bool chosen=prepareCorridorLattice(maneuver,LaneChangeConfig{},PlanningBehaviorConfig{},w,map,m,nullptr,nullptr,true,&trace);
            for(const auto& e:trace.frameEvents)std::cout<<e<<'\n';
            for(const auto& p:trace.candidatePaths) {
                const double length=p.points.empty()?0.:p.points.back().s-p.points.front().s;
                std::cout<<"candidate "<<p.lane.roadId<<':'<<p.lane.sectionIndex<<':'<<p.lane.laneId
                         <<" status="<<p.status<<" reason="<<p.reason<<" length="<<length<<'\n';
            }
            std::cout<<"M08 selected="<<chosen<<'\n';
            require(chosen,"M08 safe intermediate gateway stage missing");
            require(maneuver.roadConnectionManeuver,"M08 stage not marked as gateway maneuver");
            require(maneuver.targetLaneId==3,"M08 skipped the safe intermediate lane");
            require(maneuver.rejoinRouteS-w.egoPathS<29.,
                    "gateway stage consumed the space reserved for its next stage");
            bool unsafe=false;
            for(double margin:{0.,.1,.2,.3}) {
                int count=0;
                for(const auto& p:maneuver.path) {
                    auto box=makeObb2D(p.x+1.3725*std::cos(p.yaw),p.y+1.3725*std::sin(p.yaw),p.yaw,4.855+2*margin,1.88+2*margin);
                    for(const auto& o:w.objects)count+=intersects(box,o.obb);
                }
                std::cout<<"margin="<<margin<<" overlapping sampled poses="<<count<<'\n';
                if(margin==.3)unsafe=count>0;
            }
            require(!unsafe,"gateway candidate has no tracking-error clearance");return 0;
        }
        if(argc==2 && std::string(argv[1])=="--m05-tail-regression") {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,"scenario/mixed/M05/checkpoints.csv"),"mission load");
            auto w=world();w.ego.x=1392.07422;w.ego.y=918.371521;w.ego.heading=3.68765497;w.egoSpeed=0.;w.egoPathS=22.8155;
            auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            w.egoObb=makeObb2D(w.ego.x+1.3725*std::cos(w.ego.heading),w.ego.y+1.3725*std::sin(w.ego.heading),w.ego.heading,4.855,1.88);
            WorldObjectState o;o.raw.id=3;o.raw.x=1365.32239;o.raw.y=901.834473;o.raw.heading=3.68666911;o.raw.length=2.076;o.raw.width=.85;
            o.obb=makeObb2D(o.raw.x,o.raw.y,o.raw.heading,o.raw.length,o.raw.width);w.objects.push_back(o);
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace),"M05 tracking missing");
            LaneChangeData maneuver;
            require(prepareCorridorLattice(maneuver,LaneChangeConfig{},PlanningBehaviorConfig{},w,map,m,nullptr,nullptr,false,&trace),"M05 avoidance missing");
            double maximum=0.;
            for(std::size_t i=1;i<maneuver.path.size();++i) {
                const auto& a=maneuver.path[i-1];const auto& b=maneuver.path[i];
                const double ds=b.s-a.s;if(ds>1e-6)maximum=std::max(maximum,std::abs(std::remainder(b.yaw-a.yaw,2.*3.141592653589793))/ds);
            }
            std::cout<<"M05 full maneuver + tracking tail maximum yaw/ds="<<maximum<<'\n';
            require(maximum<.19,"raw tail tangent discontinuity poisoned speed planning");return 0;
        }
        if(argc==5 || argc==6) {
            PathGeneratorData map;require(loadMap(map,"src/data/map/HL_FMA_VTD_LivingLab.xodr"),"map load");
            std::string error;require(initializeCorridorRuntime(map,"output/precomputed_lane_network",LaneChangeConfig{},&error),error.c_str());
            MissionCorridorData m;require(initializeMissionCorridor(m,map,std::string("scenario/mixed/")+argv[1]+"/checkpoints.csv"),"mission load");
            if(argc==6)m.activeLegIndex=std::stoul(argv[5]);
            auto w=world(std::stod(argv[2]));w.ego.y=std::stod(argv[3]);w.ego.heading=std::stod(argv[4]);
            const auto match=worldToLane(map,w.ego.x,w.ego.y,w.ego.heading);
            w.egoRoadId=match.roadId;w.egoSectionIndex=match.sectionIndex;w.egoLaneId=match.laneId;w.egoRoadS=match.roadS;
            PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
            const bool okay=buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace);
            double maxK=0.;
            for(std::size_t i=1;i<path.waypoints.size();++i) {
                const auto& a=path.waypoints[i-1];const auto& b=path.waypoints[i];
                if(b.s-a.s>1e-6)maxK=std::max(maxK,std::abs(std::remainder(b.yaw-a.yaw,2.*3.141592653589793))/(b.s-a.s));
            }
            std::cout<<"tracking="<<okay<<" points="<<path.waypoints.size()<<" max_curvature="<<maxK<<" ego="<<w.egoRoadId<<':'<<w.egoSectionIndex<<':'<<w.egoLaneId<<'\n';
            auto& r=*map.corridorRuntime;
            double closest=1e9;std::int64_t closestId=-1;
            for(const auto& p:r.network.points) {
                if(!r.allowed[p.id]) continue;
                const double distance=std::hypot(p.pose.x-w.ego.x,p.pose.y-w.ego.y);
                if(distance<closest) {closest=distance;closestId=p.id;}
            }
            if(closestId>=0) {
                std::cout<<"closest="<<closestId<<" cost="<<r.gatewayCost[closestId]<<'\n';
                for(const auto& edge:r.forward[closestId])
                    std::cout<<"edge="<<edge.to<<" lateral="<<edge.lateral<<" cost="<<r.gatewayCost[edge.to]<<'\n';
            }
            for(const auto& key:m.legs.at(m.activeLegIndex).allowedLanes) {
                const auto lane=r.network.lanes.find(key);if(lane==r.network.lanes.end()) continue;
                int allowed=0,reachable=0;for(auto id:lane->second.pointIds){allowed+=r.allowed[id];reachable+=std::isfinite(r.gatewayCost[id]);}
                std::cout<<key.roadId<<':'<<key.sectionIndex<<':'<<key.laneId<<" allowed="<<allowed<<" reachable="<<reachable<<'\n';
            }
            return okay&&maxK<.19?0:1;
        }
        auto map=fixture();auto m=mission();auto w=world();obstacle(w);
        PathPlannerData path;DecisionTraceData trace;trace.enabled=true;
        require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,map,m,80.,&trace),"tracking missing");
        LaneChangeData maneuver;double pending=-1.;
        auto before=std::chrono::steady_clock::now();
        require(prepareCorridorLattice(maneuver,LaneChangeConfig{},PlanningBehaviorConfig{},w,map,m,nullptr,&pending,false,&trace),"free adjacent lane avoidance missing");
        require(maneuver.routeMode==LaneChangeRouteMode::KeepTargetLane,"forced source return");
        for(const auto& p:maneuver.path) {
            const auto box=makeObb2D(p.x+1.3725*std::cos(p.yaw),p.y+1.3725*std::sin(p.yaw),p.yaw,4.855,1.88);
            for(const auto& object:w.objects)
                require(!intersects(box,object.obb),"selected avoidance intersects the very obstacle it must avoid");
        }
        bool third=false;for(const auto& lane:trace.candidateLanes) third|=lane.laneId==-3;
        require(third,"not all corridor lanes enumerated");
        require(maneuver.path.back().x>maneuver.rejoinRouteS+10.,"tracking tail lost after shift");
        std::cout<<"all-lane selection ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count()<<'\n';
        auto blockedMap=fixture(true);auto stopped=world();obstacle(stopped);LaneChangeData rejected;
        const bool progress=prepareCorridorLattice(rejected,LaneChangeConfig{},PlanningBehaviorConfig{},stopped,blockedMap,m,nullptr,&pending,false,&trace);
        if(progress) {
            require(rejected.targetLaneId==-1,"solid line crossed");
            for(const auto& p:rejected.path) require(std::abs(p.y+1.75)<.01,"solid line crossed during intermediate path");
        }
        // A nearby blocker must be passed, not deferred by another short
        // straight segment. The threshold is independent of the 70 m search.
        auto nearby=world(17.);obstacle(nearby);
        LaneChangeConfig mandatoryConfig;
        require(mandatoryConfig.obstacleMandatoryAvoidanceDistance==15.,"mandatory default is not 15 m");
        LaneChangeData nearbyAvoidance;
        require(prepareCorridorLattice(nearbyAvoidance,mandatoryConfig,PlanningBehaviorConfig{},nearby,map,m,nullptr,&pending,false,&trace),
                "15 m mandatory avoidance failed with an open adjacent lane");
        require(pending<=15.,"fixture does not exercise mandatory avoidance");
        require(nearbyAvoidance.path.back().x>40.,"mandatory avoidance did not pass obstacle");
        for(const auto& p:nearbyAvoidance.path) {
            const auto box=makeObb2D(p.x+1.3725*std::cos(p.yaw),p.y+1.3725*std::sin(p.yaw),p.yaw,4.855,1.88);
            require(!intersects(box,nearby.objects.front().obb),"mandatory avoidance still hits blocker");
        }
        LaneChangeData unavailable;
        require(!prepareCorridorLattice(unavailable,mandatoryConfig,PlanningBehaviorConfig{},nearby,blockedMap,m,nullptr,&pending,false,&trace),
                "mandatory avoidance accepted a short path ending before a blocked solid lane");
        // Reducing the urgency distance must not turn a path which merely
        // ends before a stationary blocker into a successful avoidance.
        // Urgency and physical validity are independent.
        mandatoryConfig.obstacleMandatoryAvoidanceDistance=1.;
        require(!prepareCorridorLattice(unavailable,mandatoryConfig,PlanningBehaviorConfig{},nearby,blockedMap,m,nullptr,&pending,false,&trace),
                "non-mandatory search accepted a path ending before a stationary blocker");
        // Real-time searches resume rather than repeatedly testing only the
        // first few candidates. Each resumed candidate uses the current world.
        auto incrementalMap=fixture();LaneChangeData incremental;
        bool incrementalReady=false,advanced=false;
        for(int frame=0;frame<200 && !incrementalReady;++frame) {
            auto& runtime=*incrementalMap.corridorRuntime;
            beginCorridorSearchFrame(runtime);
            incrementalReady=prepareCorridorLattice(incremental,LaneChangeConfig{},PlanningBehaviorConfig{},nearby,incrementalMap,m,nullptr,&pending,false,&trace);
            advanced|=runtime.searchCursor[0]>0;
            LaneChangeData duplicate;
            require(!prepareCorridorLattice(duplicate,LaneChangeConfig{},PlanningBehaviorConfig{},nearby,incrementalMap,m,nullptr,&pending,false,&trace),
                    "duplicate obstacle search ran in the same control frame");
        }
        require(incrementalReady,"incremental search starved an available avoidance candidate");
        (void)advanced; // Fast machines may complete the search in one frame.
        for(const auto& p:incremental.path)
            require(!intersects(makeObb2D(p.x+1.3725*std::cos(p.yaw),p.y+1.3725*std::sin(p.yaw),p.yaw,4.855,1.88),nearby.objects.front().obb),
                    "incremental search returned an unsafe path");
        // A far obstacle must not make the vehicle abandon the gateway when
        // the current connector is temporarily absent from active-leg
        // topology.  It can use an unreachable escape only after the blocker
        // enters the mandatory window.
        auto unreachableMap=fixture();
        require(buildCorridorTrackingPath(path,PathPlannerConfig{},w,
                                          unreachableMap,m,80.,&trace),
                "unreachable fixture tracking missing");
        std::fill(unreachableMap.corridorRuntime->gatewayCost.begin(),
                  unreachableMap.corridorRuntime->gatewayCost.end(),
                  std::numeric_limits<double>::infinity());
        std::fill(unreachableMap.corridorRuntime->gatewayLateralSteps.begin(),
                  unreachableMap.corridorRuntime->gatewayLateralSteps.end(),
                  std::numeric_limits<int>::max());
        LaneChangeData distantUnreachable;
        require(!prepareCorridorLattice(
                    distantUnreachable,LaneChangeConfig{},
                    PlanningBehaviorConfig{},w,unreachableMap,m,nullptr,
                    &pending,false,&trace),
                "far obstacle selected a gateway-unreachable branch");
        auto mandatoryUnreachable=world(22.);obstacle(mandatoryUnreachable);
        LaneChangeData emergencyEscape;
        require(prepareCorridorLattice(
                    emergencyEscape,LaneChangeConfig{},
                    PlanningBehaviorConfig{},mandatoryUnreachable,
                    unreachableMap,m,nullptr,&pending,false,&trace),
                "mandatory obstacle could not use the only physical escape");
        auto border=world(95.);border.egoLaneId=-2;border.ego.y=-5.25;
        require(buildCorridorTrackingPath(path,PathPlannerConfig{},border,map,m,60.,&trace),"road boundary tracking missing");
        require(path.waypoints.back().roadId==2 && path.waypoints.back().x>140.,"horizon truncated at road id boundary");
        auto knotMap=fixture(false,true);
        require(buildCorridorTrackingPath(path,PathPlannerConfig{},world(40.),knotMap,m,80.,&trace),"coincident source knot lost gateway reachability");
        require(path.waypoints.back().x>110.,"coincident source knot truncated tracking");
        // An isolated missing edge must trigger a geometric search of the
        // precomputed lane samples rather than strand the vehicle at a node.
        auto recoveryMap=fixture();
        recoveryMap.corridorRuntime->forward[20].clear();
        require(buildCorridorTrackingPath(path,PathPlannerConfig{},world(10.),recoveryMap,m,80.,&trace),
                "all-lane recovery failed at an isolated missing link");
        require(path.waypoints.back().s-path.waypoints.front().s>30.,
                "recovery reused an exhausted path");
        // A missed fork must not turn into a valid route merely because some
        // unrelated XODR lane happens to be underneath the vehicle. Recovery
        // may bridge a missing sampled edge inside the active mission
        // corridor, but it must never escape to the whole-map network.
        auto outsideMap=fixture();
        MissionCorridorData outsideMission;outsideMission.valid=outsideMission.initialized=true;
        MissionLegData outsideLeg;outsideLeg.valid=true;
        outsideLeg.allowedLanes.insert({2,0,-3});
        outsideLeg.targetLanes.insert({2,0,-3});
        outsideMission.legs.push_back(outsideLeg);
        auto outsideWorld=world(10.);
        require(!buildCorridorTrackingPath(path,PathPlannerConfig{},outsideWorld,
                                           outsideMap,outsideMission,80.,&trace),
                "whole-map recovery escaped the active mission corridor");
        // Restoring mobility must not bypass collision checks when all road
        // space in front is physically occupied.
        auto occupied=world(10.);
        WorldObjectState wall;wall.raw.id=99;wall.raw.x=30.;wall.raw.y=-5.25;
        wall.raw.length=30.;wall.raw.width=20.;
        wall.obb=makeObb2D(30.,-5.25,0.,30.,20.);occupied.objects.push_back(wall);
        require(!buildCorridorTrackingPath(path,PathPlannerConfig{},occupied,recoveryMap,m,80.,&trace),
                "recovery ignored a wall covering all lanes");
        auto gateMap=fixture();auto gate=mission(true);auto approach=world(68.);LaneChangeData align;
        PathPlannerData firstGlobal,secondGlobal;
        require(buildCorridorTrackingPath(firstGlobal,PathPlannerConfig{},approach,gateMap,gate,80.,&trace),
                "corridor-entry global route missing");
        const auto initialRevision=gateMap.corridorRuntime->routeRevision;
        require(buildCorridorTrackingPath(secondGlobal,PathPlannerConfig{},approach,gateMap,gate,80.,&trace),
                "latched corridor global route disappeared");
        require(gateMap.corridorRuntime->routeRevision==initialRevision,
                "unchanged corridor rebuilt its global route");
        require(gateMap.corridorRuntime->gatewaySpine[
                    gateMap.corridorRuntime->network.lanes.at({1,0,-3}).pointIds.front()],
                "gateway-connected lane was not identified as the stable spine");
        require(prepareCorridorLattice(align,LaneChangeConfig{},PlanningBehaviorConfig{},approach,gateMap,gate,nullptr,nullptr,true,&trace),"gateway alignment missing");
        require(align.targetLaneId==-2&&align.roadConnectionManeuver,"gateway did not use one safe lateral stage");
        // A previously static actor may start moving into the route.  It is
        // no longer in the confirmed-static set, but must still invalidate
        // the route and produce a checked avoidance candidate before EBS is
        // the only remaining response.
        auto dynamicMap=fixture();auto cutIn=world(10.);
        WorldObjectState moving;moving.raw.id=41;moving.raw.x=25.;moving.raw.y=-1.75;
        moving.raw.heading=3.14159265358979323846;moving.raw.speed=2.;
        moving.raw.length=4.;moving.raw.width=2.;
        moving.obb=makeObb2D(moving.raw.x,moving.raw.y,moving.raw.heading,
                             moving.raw.length,moving.raw.width);
        cutIn.objects.push_back(moving);
        std::vector<int> noConfirmedStatic;
        LaneChangeData dynamicAvoidance;double dynamicDistance=-1.;
        require(prepareCorridorLattice(dynamicAvoidance,LaneChangeConfig{},
                    PlanningBehaviorConfig{},cutIn,dynamicMap,m,
                    &noConfirmedStatic,&dynamicDistance,false,&trace),
                "moving cut-in did not invalidate and replan the corridor route");
        require(dynamicDistance>=0. && dynamicAvoidance.obstacleId==41,
                "dynamic route invalidation selected the wrong blocker");
        std::cout<<"corridor_lattice_tests: PASS (all lanes, keep target, solid rejection, cross-road horizon, gateway deadline)\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
