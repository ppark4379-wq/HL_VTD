#include "planning/global/mission_corridor.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool okay,const char* message) {
    if(!okay) throw std::runtime_error(message);
}
MissionLegData leg(bool positive=false) {
    MissionLegData result;result.valid=result.topologyAuthority=true;
    const int sign=positive?1:-1;
    const LaneKey anchor{20,0,sign*4};
    result.targetLanes.insert(anchor);result.targetRoadS[anchor]=10.;
    result.allowedLanes={anchor,{20,0,sign*3},{20,1,sign*2},
                         {20,0,-sign*3},{30,0,sign*3}};
    return result;
}
Waypoint point(int road,int section,int lane,double s) {
    Waypoint p;p.roadId=road;p.sectionIndex=section;p.laneId=lane;p.roadS=s;return p;
}
void targetGuards() {
    auto m=leg();
    require(missionPointIsTarget(m,point(20,0,-3,11.)),
            "parallel permitted exit lane was rejected");
    require(missionPointIsTarget(m,point(20,1,-2,30.)),
            "same directed exit after a section boundary was rejected");
    require(!missionPointIsTarget(m,point(20,0,-3,9.)),
            "gateway completed before its road-s station");
    require(!missionPointIsTarget(m,point(30,0,-3,11.)),
            "another allowed road was mistaken for the required exit");
    require(!missionPointIsTarget(m,point(20,0,3,11.)),
            "opposite-direction exit lane was accepted");
    require(!missionPointIsTarget(m,point(20,0,-1,11.)),
            "lane outside the permitted corridor was accepted");
    require(!missionPointIsTarget(m,point(20,0,0,11.)),"centre line became an exit lane");
    require(!missionPointIsTarget(m,point(20,0,-3,std::numeric_limits<double>::quiet_NaN())),
            "unknown road-s completed the gateway");
    m.finalLeg=true;
    require(!missionPointIsTarget(m,point(20,0,-3,11.)),"final exact lane target was broadened");
    require(missionPointIsTarget(m,point(20,0,-4,11.)),"final exact target stopped working");
    m=leg(true);
    require(missionPointIsTarget(m,point(20,0,3,9.)),"positive-lane reverse-s exit was rejected");
    require(!missionPointIsTarget(m,point(20,0,3,11.)),"positive-lane exit passed too early");
    m.topologyAuthority=false;m.targetRoadS.clear();
    require(missionPointIsTarget(m,point(20,0,4,0.)),"legacy target without station changed");
    require(!missionPointIsTarget(m,point(20,0,3,0.)),"legacy target lane membership broadened");
}
void progressRequiresTwoPhysicalExitFrames() {
    MissionCorridorData mission;mission.valid=mission.initialized=mission.topologyAuthority=true;
    mission.legs={leg(),leg()};mission.legs.back().finalLeg=true;
    WorldState world;world.egoLaneValid=true;world.egoRoadId=20;
    world.egoLaneId=-3;world.egoRoadS=11.;
    updateMissionCorridorProgress(mission,world);
    require(mission.activeLegIndex==0,"parallel exit bypassed stable-frame guard");
    updateMissionCorridorProgress(mission,world);
    require(mission.activeLegIndex==1,"parallel permitted exit did not advance the leg");
}
}
int main() {
    try {
        targetGuards();progressRequiresTwoPhysicalExitFrames();
        std::cout<<"directed parallel exit target guards PASS\n";
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
}
