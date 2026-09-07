#include "planning/local/corridor_lattice.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "logger/decision_trace.hpp"
#include "utils/geometry.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <array>

void require(bool okay,const char* message) {if(!okay) throw std::runtime_error(message);}
PathGeneratorData fixture(bool solid=false,bool sourceKnot=false) {
    PathGeneratorData map;PrecomputedLaneNetwork network;
    for(int rid=1;rid<=2;++rid) {
        Road road;road.id=rid;road.length=100.;
        Geometry geometry;geometry.length=100.;geometry.x=(rid-1)*100.;road.geometries.push_back(geometry);
        LaneSection section;
        for(int lid=-1;lid>=-3;--lid) {
            LaneDefinition lane;lane.id=lid;lane.type="driving";lane.widths.push_back({0.,3.5,0.,0.,0.});
            lane.roadMarks.push_back({0.,solid?"none":"both","white",solid?"solid":"broken"});section.lanes[lid]=lane;
            PrecomputedLane raw;raw.key={rid,0,lid};raw.topologyStatus="connected";
            LaneSegment segment;segment.key=raw.key;
            for(int i=0;i<=200;++i) {
                PrecomputedLanePoint p;p.id=network.points.size();p.pointIndex=i;p.width=3.5;p.usable=true;
                p.pose.x=(rid-1)*100.+i*.5;p.pose.y=(lid+.5)*3.5;p.pose.s=p.pose.roadS=i*.5;p.pose.roadId=rid;p.pose.laneId=lid;
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
    std::string error;require(initializeCorridorRuntime(map,std::move(network),LaneChangeConfig{},&error),error.c_str());return map;
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
        auto border=world(95.);border.egoLaneId=-2;border.ego.y=-5.25;
        require(buildCorridorTrackingPath(path,PathPlannerConfig{},border,map,m,60.,&trace),"road boundary tracking missing");
        require(path.waypoints.back().roadId==2 && path.waypoints.back().x>140.,"horizon truncated at road id boundary");
        auto knotMap=fixture(false,true);
        require(buildCorridorTrackingPath(path,PathPlannerConfig{},world(40.),knotMap,m,80.,&trace),"coincident source knot lost gateway reachability");
        require(path.waypoints.back().x>110.,"coincident source knot truncated tracking");
        auto gateMap=fixture();auto gate=mission(true);auto approach=world(68.);LaneChangeData align;
        require(prepareCorridorLattice(align,LaneChangeConfig{},PlanningBehaviorConfig{},approach,gateMap,gate,nullptr,nullptr,true,&trace),"gateway alignment missing");
        require(align.targetLaneId==-2&&align.roadConnectionManeuver,"gateway did not use one safe lateral stage");
        std::cout<<"corridor_lattice_tests: PASS (all lanes, keep target, solid rejection, cross-road horizon, gateway deadline)\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
