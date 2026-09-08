#include "planning/local/corridor_lattice.hpp"
#include "logger/decision_trace.hpp"
#include "utils/geometry.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

// Exercise the internal reconnect entry directly: no global worker, VTD,
// full-map materialization or time-based sleeps are needed for this cache test.
bool connectToReferencePath(std::vector<Waypoint>&,
    const std::vector<Waypoint>&,std::size_t,CorridorRuntime&,
    const PathGeneratorData&,const WorldState&,double,DecisionTraceData*,bool);

namespace {
void require(bool okay,const char* message) {
    if(!okay) throw std::runtime_error(message);
}
struct Fixture {
    CorridorRuntime runtime;
    PathGeneratorData map;
    WorldState world;
    DecisionTraceData trace;
    std::vector<Waypoint> output;
    double horizon{80.};
    Fixture() {
        beginCorridorSearchFrame(runtime);
        runtime.activeLeg=0;runtime.routeRevision=2;
        runtime.globalRouteRequestRevision=3;
        // No connection target is within 50m. This is a deterministic failed
        // reconnect, without needing any map/curve/collision fixture.
        for(double x:{100.,101.}) {
            Waypoint p;p.x=p.s=p.roadS=x;p.roadId=1;p.laneId=-1;
            runtime.committedGlobalPath.push_back(p);
        }
        world.receiveTime=100.;world.egoRoadId=1;world.egoLaneId=-1;
        world.egoObb=makeObb2D(1.3725,0.,0.,4.855,1.88);
        WorldObjectState object;object.raw.id=7;
        object.raw.x=40.;object.raw.y=2.;object.raw.length=4.;
        object.raw.width=2.;object.raw.height=1.5;
        object.obb=makeObb2D(40.,2.,0.,4.,2.);
        world.objects.push_back(object);
        object.raw.id=8;object.raw.x=60.;
        object.obb=makeObb2D(60.,2.,0.,4.,2.);
        world.objects.push_back(object);
        trace.enabled=true;
        expectHit(false,"initial reconnect unexpectedly used cache");
        require(runtime.committedReconnectFailure.valid,"failure was not cached");
        world.receiveTime+=.1;
    }
    void expectHit(bool expected,const char* message) {
        trace.frameEvents.clear();
        output.push_back(Waypoint{});
        require(!connectToReferencePath(output,runtime.committedGlobalPath,
                    runtime.committedGlobalPathProgress,runtime,map,world,
                    horizon,&trace,true),"out-of-range reference unexpectedly connected");
        require(output.empty(),"failed/cache-hit reconnect leaked stale output");
        const bool hit=std::any_of(trace.frameEvents.begin(),trace.frameEvents.end(),
            [](const auto& event){return event.find("tracking_reconnect_retry_throttled")!=std::string::npos;});
        require(hit==expected,message);
    }
};
void unchangedFailureAndFrameReset() {
    Fixture f;
    beginCorridorSearchFrame(f.runtime);
    f.expectHit(true,"begin-frame erased useful negative cache");
    std::reverse(f.world.objects.begin(),f.world.objects.end());
    f.expectHit(true,"packet slot order incorrectly invalidated identical observations");
    f.world.ego.x=.05F;
    f.world.egoObb=makeObb2D(1.4225,0.,0.,4.855,1.88);
    f.expectHit(true,"small pose noise defeated negative cache");
    f.world.receiveTime=100.5;
    f.expectHit(false,"half-second retry deadline did not expire");
}
void invalidateChangedInputs() {
    const std::vector<std::function<void(Fixture&)>> changes{
        [](auto& f){++f.runtime.routeRevision;},
        [](auto& f){++f.runtime.globalRouteRequestRevision;},
        [](auto& f){++f.runtime.activeLeg;},
        [](auto& f){++f.runtime.committedGlobalPathProgress;},
        [](auto& f){f.runtime.committedGlobalPath.push_back(f.runtime.committedGlobalPath.back());},
        [](auto& f){f.world.ego.x=.3F;},
        [](auto& f){f.world.ego.heading=.03F;},
        [](auto& f){f.world.ego.z=.2F;},
        [](auto& f){f.world.egoSpeed=.2;},
        [](auto& f){f.world.egoLaneId=-2;},
        [](auto& f){f.runtime.curvature=.001;},
        [](auto& f){f.horizon=70.;},
        [](auto& f){f.world.objects[0].raw.x+=.01F;},
        [](auto& f){f.world.objects[0].raw.z+=.01F;},
        [](auto& f){f.world.objects[0].raw.speed=1.F;},
        [](auto& f){f.world.objects[0].velocityX=.5;f.world.objects[0].velocityValid=true;},
        [](auto& f){f.world.objects[0].obb.corners[0].x+=.01;},
        [](auto& f){f.world.objects[0].raw.width+=.01F;},
        [](auto& f){f.world.objects.pop_back();},
        [](auto& f){f.world.egoObb=makeObb2D(1.3725,0.,0.,4.855,2.1);},
        [](auto& f){f.runtime.laneConfig.latticeObstacleClearance+=.05;},
        [](auto& f){f.world.receiveTime=99.;},
        [](auto& f){f.world.respawnDetected=true;},
        [](auto& f){f.world.objects[0].raw.speed=std::numeric_limits<float>::quiet_NaN();}
    };
    for(const auto& change:changes) {
        Fixture f;change(f);
        f.expectHit(false,"changed reconnect input was incorrectly suppressed");
    }
}
void otherReferencesAndOfflineCallsRemainUncached() {
    Fixture f;
    const auto separateReference=f.runtime.committedGlobalPath;
    f.trace.frameEvents.clear();
    require(!connectToReferencePath(f.output,separateReference,0,f.runtime,f.map,
        f.world,80.,&f.trace,true),"out-of-range separate reference unexpectedly connected");
    require(f.trace.frameEvents.empty(),"committed-route cache suppressed another reference");
    f.runtime.budgetedSearch=false;
    f.expectHit(false,"offline exhaustive caller was throttled");
}
}
int main() {
    try {
        unchangedFailureAndFrameReset();invalidateChangedInputs();
        otherReferencesAndOfflineCallsRemainUncached();
        std::cout<<"committed-route reconnect negative-cache regressions PASS\n";
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
}
