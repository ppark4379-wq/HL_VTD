#include "planning/local/corridor_lattice.hpp"
#include "logger/decision_trace.hpp"
#include "utils/geometry.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
void require(bool condition,const char* message) {
    if(!condition) throw std::runtime_error(message);
}

PathGeneratorData mapFixture() {
    PathGeneratorData map;
    PrecomputedLaneNetwork network;
    for(int roadId=1;roadId<=2;++roadId) {
        Road road;road.id=roadId;road.length=100.;
        Geometry geometry;geometry.length=100.;geometry.x=(roadId-1)*100.;
        road.geometries.push_back(geometry);
        LaneSection section;
        for(int laneId=-1;laneId>=-3;--laneId) {
            LaneDefinition lane;lane.id=laneId;lane.type="driving";
            lane.widths.push_back({0.,3.5,0.,0.,0.});
            lane.roadMarks.push_back({0.,"both","white","broken"});
            section.lanes[laneId]=lane;
            PrecomputedLane sampled;sampled.key={roadId,0,laneId};
            sampled.topologyStatus="connected";
            LaneSegment segment;segment.key=sampled.key;
            for(int index=0;index<=200;++index) {
                PrecomputedLanePoint point;point.id=network.points.size();
                point.pointIndex=index;point.width=3.5;point.usable=true;
                point.pose.x=geometry.x+index*.5;
                point.pose.y=(laneId+.5)*3.5;
                point.pose.s=point.pose.roadS=index*.5;
                point.pose.roadId=roadId;point.pose.laneId=laneId;
                if(index<200) point.nextPoint=point.id+1;
                sampled.pointIds.push_back(point.id);
                segment.points.push_back(point.pose);network.points.push_back(point);
            }
            network.lanes[sampled.key]=sampled;
            map.laneSegments[sampled.key]=segment;
        }
        road.sections.push_back(section);map.roads[roadId]=road;
    }
    for(auto& point:network.points) for(int side:{-1,1}) {
        const auto lane=network.lanes.find(
            {point.pose.roadId,0,point.pose.laneId+side});
        if(lane==network.lanes.end()) continue;
        const auto adjacent=lane->second.pointIds[point.pointIndex];
        if(side<0) {point.rightPoint=adjacent;point.rightChangeAllowed=true;}
        else {point.leftPoint=adjacent;point.leftChangeAllowed=true;}
    }
    for(int lane=-1;lane>=-3;--lane) {
        PrecomputedLaneConnection edge;edge.from={1,0,lane};edge.to={2,0,lane};
        edge.status="continuous";
        edge.fromPoint=network.lanes[edge.from].pointIds.back();
        edge.toPoint=network.lanes[edge.to].pointIds.front();
        network.connections.push_back(edge);
        map.laneGraph[edge.from].push_back(edge.to);
    }
    LaneChangeConfig config;config.localFallbackHorizon=90.;
    std::string error;
    require(initializeCorridorRuntime(map,std::move(network),config,&error),error.c_str());
    return map;
}

MissionCorridorData missionFixture() {
    MissionCorridorData mission;
    mission.valid=mission.initialized=mission.topologyAuthority=true;
    mission.checkpoints.resize(3);
    mission.checkpoints[1].checkpoint.x=102.;
    mission.checkpoints[2].checkpoint.x=180.;
    MissionLegData first;
    first.valid=first.topologyAuthority=true;
    first.gatewayCheckpoint=first.toCheckpoint=1;
    first.gatewayLanes.insert({1,0,-1});
    first.targetLanes.insert({2,0,-1});first.targetRoadS[{2,0,-1}]=2.;
    for(int road=1;road<=2;++road) for(int lane=-1;lane>=-3;--lane)
        first.allowedLanes.insert({road,0,lane});
    MissionLegData next;
    next.valid=next.topologyAuthority=next.finalLeg=true;
    next.fromCheckpoint=1;next.gatewayCheckpoint=next.toCheckpoint=2;
    next.targetLanes.insert({2,0,-1});next.targetRoadS[{2,0,-1}]=80.;
    for(int lane=-1;lane>=-3;--lane) next.allowedLanes.insert({2,0,lane});
    mission.legs={first,next};
    return mission;
}

void verify(bool blockedFinalTarget) {
    auto map=mapFixture();auto mission=missionFixture();
    WorldState world;world.inputValid=world.egoLaneValid=world.egoPathValid=true;
    world.ego.x=110.;world.ego.y=-1.75;world.egoSpeed=2.;
    world.egoRoadId=2;world.egoLaneId=-1;world.egoRoadS=10.;world.egoPathS=110.;
    world.egoObb=makeObb2D(111.3725,-1.75,0.,4.855,1.88);
    WorldObjectState object;object.raw.id=42;object.raw.x=blockedFinalTarget?183.:145.;
    object.raw.y=-1.75;object.raw.length=blockedFinalTarget?34.:4.;
    object.raw.width=2.6;object.raw.height=2.;
    object.obb=makeObb2D(object.raw.x,object.raw.y,0.,object.raw.length,object.raw.width);
    world.objects.push_back(object);
    auto& runtime=*map.corridorRuntime;
    PathPlannerData tracking;DecisionTraceData trace;trace.enabled=true;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(std::chrono::steady_clock::now()<deadline) {
        beginCorridorSearchFrame(runtime);
        buildCorridorTrackingPath(tracking,PathPlannerConfig{},world,map,mission,80.,&trace);
        if(runtime.committedGlobalPathReachesGateway || runtime.globalRouteSearchFailed) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    require(runtime.globalExitPreviewActive,"fixture did not enable a next-corridor frontier");
    if(blockedFinalTarget) {
        if(runtime.committedGlobalPathReachesGateway) {
            const auto& end=runtime.committedGlobalPath.back();
            std::cerr<<"dead-end route end="<<end.x<<","<<end.y
                     <<" lane="<<end.laneId<<" road_s="<<end.roadS<<'\n';
        }
        require(!runtime.committedGlobalPathReachesGateway,
                "empty frontier beyond last legal merge was approved despite blocked mandatory successor");
    } else {
        require(runtime.committedGlobalPathReachesGateway,
                "successor-reachability guard removed a safe avoidance and return route");
        const auto& end=runtime.committedGlobalPath.back();
        require(end.laneId==-1 && end.x>=180.,
                "safe avoidance did not rejoin the only required next target lane");
    }
}
} // namespace

int main() {
    try {
        verify(true);verify(false);
        std::cout<<"gateway preview dead-end rejection / safe return PASS\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
}
