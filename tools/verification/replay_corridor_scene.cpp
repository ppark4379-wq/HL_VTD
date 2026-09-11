// Diagnostic replay of the production corridor worker and optional planning
// pipeline, using fixed observations or recorded poses. No VTD socket or
// control command; recorded poses do not respond to the replay's steering.
#include "control/config/control_config.hpp"
#include "logger/decision_trace.hpp"
#include "planning/global/mission_corridor.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/local/corridor_lattice.hpp"
#include "planning/planning_process.hpp"
#include "utils/geometry.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <thread>

namespace {
using Tree = boost::property_tree::ptree;
using Clock = std::chrono::steady_clock;

struct Options {
    std::string snapshot, mission, output;
    std::string map{"src/data/map/HL_FMA_VTD_LivingLab.xodr"};
    std::string pointDirectory{"output/precomputed_lane_network"};
    std::string config{"config/control.yaml"};
    std::string gatewayHistorySnapshot;
    double timeout{5.};
    double snapshotTime{-1.};
    bool localFallback{};
    bool trackingRecovery{};
    bool queueApproach{};
    int planningFrames{};
    std::string trajectory;
};

Options parse(int argc, char** argv) {
    Options result;
    for (int i=1; i<argc; ++i) {
        const std::string flag=argv[i];
        if (i+1>=argc) throw std::runtime_error("Missing value for "+flag);
        const std::string value=argv[++i];
        if (flag=="--snapshot") result.snapshot=value;
        else if (flag=="--mission") result.mission=value;
        else if (flag=="--output-dir") result.output=value;
        else if (flag=="--map") result.map=value;
        else if (flag=="--point-dir") result.pointDirectory=value;
        else if (flag=="--control-config") result.config=value;
        else if (flag=="--gateway-history-snapshot") result.gatewayHistorySnapshot=value;
        else if (flag=="--timeoutseconds") result.timeout=std::stod(value);
        else if (flag=="--time") result.snapshotTime=std::stod(value);
        else if (flag=="--local-fallback") result.localFallback=value=="1";
        else if (flag=="--tracking-recovery") result.trackingRecovery=value=="1";
        else if (flag=="--queue-approach") result.queueApproach=value=="1";
        else if (flag=="--planning-frames") result.planningFrames=std::stoi(value);
        else if (flag=="--trajectory-jsonl") result.trajectory=value;
        else throw std::runtime_error("Unknown argument: "+flag);
    }
    if (result.snapshot.empty() || result.mission.empty() || result.output.empty() ||
        !std::isfinite(result.timeout) || result.timeout<=0.)
        throw std::runtime_error("Usage: replay_corridor_scene --snapshot JSON --mission complete_network.json --output-dir DIR [--timeoutseconds 5]");
    return result;
}

Obb2D readObb(const Tree& item, const Obb2D& fallback) {
    const auto points=item.get_child_optional("obb");
    if (!points || points->size()!=4) return fallback;
    Obb2D result;
    std::size_t index=0;
    for (const auto& row:*points) {
        if (row.second.size()<2) throw std::runtime_error("Invalid snapshot OBB");
        auto value=row.second.begin();
        result.corners[index].x=(value++)->second.get_value<double>();
        result.corners[index++].y=value->second.get_value<double>();
    }
    return result;
}

double projectedRoadS(const PathGeneratorData& map, const LaneKey& lane,
                      double x, double y) {
    const auto found=map.laneSegments.find(lane);
    if (found==map.laneSegments.end() || found->second.points.empty())
        throw std::runtime_error("Logged Ego lane is absent from XODR; refusing to replace its ID");
    const auto& points=found->second.points;
    double best=std::numeric_limits<double>::infinity(), station=points.front().roadS;
    for (std::size_t i=0;i<points.size();++i) {
        const auto& a=points[i]; const auto& b=points[std::min(i+1,points.size()-1)];
        const double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy;
        const double u=d2>1e-12?std::clamp(((x-a.x)*dx+(y-a.y)*dy)/d2,0.,1.):0.;
        const double error=std::hypot(x-a.x-u*dx,y-a.y-u*dy);
        if(error<best) {best=error;station=a.roadS+u*(b.roadS-a.roadS);}
    }
    return station;
}

WorldState readWorld(const Tree& document, const PathGeneratorData& map,
                     LaneMatch& nearest) {
    WorldState world;
    const auto& ego=document.get_child("ego");
    world.ego.x=ego.get<float>("x"); world.ego.y=ego.get<float>("y");
    world.ego.z=ego.get<float>("z"); world.ego.heading=ego.get<float>("heading");
    world.egoSpeed=ego.get<double>("speed");
    world.trafficLight.id=document.get<int>("traffic_light.id",-1);
    world.trafficLight.state=document.get<int>("traffic_light.state",0);
    world.receiveTime=document.get<double>("receive_time_s",0.);
    world.egoRoadId=ego.get<int>("road_id");
    world.egoSectionIndex=ego.get<int>("section_index");
    world.egoLaneId=ego.get<int>("lane_id");
    nearest=worldToLane(map,world.ego.x,world.ego.y,world.ego.heading);
    const bool same=nearest.valid && nearest.roadId==world.egoRoadId &&
        nearest.sectionIndex==world.egoSectionIndex && nearest.laneId==world.egoLaneId;
    world.egoRoadS=same?nearest.roadS:projectedRoadS(map,
        {world.egoRoadId,world.egoSectionIndex,world.egoLaneId},world.ego.x,world.ego.y);
    world.egoLaneWidth=nearest.valid?nearest.laneWidth:3.2;
    world.egoLaneCenterDistance=nearest.distance;
    world.inputValid=world.egoLaneValid=world.egoPathValid=true;
    world.egoPathS=0.;
    world.egoObb=readObb(ego,makeObb2D(
        world.ego.x+1.3725*std::cos(world.ego.heading),
        world.ego.y+1.3725*std::sin(world.ego.heading),world.ego.heading,4.855,1.88));
    const auto footprint=vehicleFootprintFromObb(world.egoObb,
        {world.ego.x,world.ego.y,world.ego.heading});
    world.frontBumperOffset=footprint.frontExtent();
    world.frontBumperPathS=world.frontBumperOffset;
    for (const auto& row:document.get_child("objects")) {
        const auto& item=row.second;
        WorldObjectState object;
        object.raw.id=item.get<std::uint32_t>("id");
        object.raw.x=item.get<float>("x"); object.raw.y=item.get<float>("y");
        object.raw.z=item.get<float>("z"); object.raw.heading=item.get<float>("heading");
        object.raw.speed=item.get<float>("speed");
        object.raw.length=item.get<float>("length");
        object.raw.width=item.get<float>("width"); object.raw.height=item.get<float>("height");
        object.obb=readObb(item,makeObb2D(object.raw.x,object.raw.y,
            object.raw.heading,object.raw.length,object.raw.width));
        object.roadId=item.get<int>("road_id",-1);
        object.sectionIndex=item.get<int>("section_index",-1);
        object.laneId=item.get<int>("lane_id",0);
        const auto match=worldToLane(map,object.raw.x,object.raw.y,object.raw.heading);
        object.laneValid=match.valid;
        object.roadS=match.roadS;
        object.pathS=item.get<double>("path_s",0.);
        object.pathD=item.get<double>("path_d",0.);
        const double dx=object.raw.x-world.ego.x,dy=object.raw.y-world.ego.y;
        object.relativeS=dx*std::cos(world.ego.heading)+dy*std::sin(world.ego.heading);
        object.relativeD=-dx*std::sin(world.ego.heading)+dy*std::cos(world.ego.heading);
        object.pathValid=true;
        object.isAhead=item.get<bool>("ahead",object.relativeS>0.);
        object.isOnEgoPath=item.get<bool>("on_path",std::abs(object.relativeD)<2.);
        object.isCollisionRisk=item.get<bool>("collision",false);
        world.objects.push_back(object);
    }
    return world;
}
} // namespace

int main(int argc, char** argv) {
    try {
        const auto options=parse(argc,argv);
        const std::filesystem::path directory=options.output;
        std::filesystem::create_directories(directory);
        Tree snapshot;
        if(options.snapshotTime<0.) {
            boost::property_tree::read_json(options.snapshot,snapshot);
        } else {
            std::ifstream frames(options.snapshot);
            if(!frames) throw std::runtime_error("Cannot open snapshot JSONL");
            std::string line; bool found=false;
            while(std::getline(frames,line)) {
                if(line.empty()) continue;
                std::istringstream input(line); Tree candidate;
                boost::property_tree::read_json(input,candidate);
                if(candidate.get<double>("time_s")>=options.snapshotTime) {
                    snapshot=std::move(candidate);found=true;break;
                }
            }
            if(!found) throw std::runtime_error("No frame at or after --time");
        }
        DrivingConfig config;
        if(!loadDrivingConfig(config,options.config)) throw std::runtime_error("Control config load failed");
        PathGeneratorData map;
        if(!loadMap(map,options.map)) throw std::runtime_error("XODR load failed");
        std::string error;
        if(!initializeCorridorRuntime(map,options.pointDirectory,config.laneChange,&error))
            throw std::runtime_error(error);
        MissionCorridorData mission;
        if(!loadPrecomputedMission(mission,map,options.mission,&error)) throw std::runtime_error(error);
        mission.activeLegIndex=snapshot.get<std::size_t>("decision.active_leg");
        if(mission.activeLegIndex>=mission.legs.size()) throw std::runtime_error("Invalid snapshot active leg");
        LaneMatch nearest;
        auto world=readWorld(snapshot,map,nearest);
        std::ifstream trajectory;
        if(!options.trajectory.empty()) {
            trajectory.open(options.trajectory);
            if(!trajectory || options.planningFrames<=0)
                throw std::runtime_error("Trajectory requires --planning-frames and a readable JSONL");
        }
        DecisionTraceData trace;
        initializeDecisionTrace(trace,(directory/"decision.log").string());
        auto& runtime=*map.corridorRuntime;
        PathPlannerData tracking;
        PlanningProcessData planning;
        planning.config=config.behavior;
        planning.laneChangeConfig=config.laneChange;
        planning.pathPlanner.config=config.planner;
        planning.missionCorridor=mission;
        bool gatewayHistoryVerified=false;
        if(!options.gatewayHistorySnapshot.empty()) {
            Tree previousSnapshot;
            boost::property_tree::read_json(options.gatewayHistorySnapshot,previousSnapshot);
            if(previousSnapshot.get<std::size_t>("decision.active_leg")!=mission.activeLegIndex ||
               previousSnapshot.get<double>("time_s")>=snapshot.get<double>("time_s"))
                throw std::runtime_error("Gateway history must precede the snapshot in the same active leg");
            LaneMatch previousMatch;
            const auto previousWorld=readWorld(previousSnapshot,map,previousMatch);
            const LaneKey previousLane{previousWorld.egoRoadId,previousWorld.egoSectionIndex,previousWorld.egoLaneId};
            if(!previousMatch.valid || previousMatch.roadId!=previousLane.roadId ||
               previousMatch.sectionIndex!=previousLane.sectionIndex || previousMatch.laneId!=previousLane.laneId ||
               !mission.legs[mission.activeLegIndex].gatewayLanes.count(previousLane))
                throw std::runtime_error("Gateway history pose is not on the measured required gateway lane");
            // Replay the real earlier observation through the production
            // latch. Discard its asynchronous search revision so the route
            // below still uses every object in the current snapshot.
            beginDecisionTraceFrame(trace,0,0.);
            beginCorridorSearchFrame(runtime);
            buildCorridorTrackingPath(tracking,config.planner,previousWorld,map,mission,80.,&trace);
            requestCorridorGlobalReplan(runtime);
            gatewayHistoryVerified=true;
        }
        bool success=false, timedOut=false;
        std::size_t polls=0;
        double maxPollMs=0.;
        const auto started=Clock::now();
        while(true) {
            const auto pollStart=Clock::now();
            const double elapsed=std::chrono::duration<double>(pollStart-started).count();
            beginDecisionTraceFrame(trace,polls,elapsed);
            beginCorridorSearchFrame(runtime);
            if(trajectory.is_open()) {
                std::string line;
                if(!std::getline(trajectory,line)) break;
                std::istringstream input(line);Tree current;
                boost::property_tree::read_json(input,current);
                world=readWorld(current,map,nearest);
            }
            if(options.planningFrames>0) {
                auto frameWorld=world;
                if(!trajectory.is_open()) frameWorld.receiveTime+=polls*.05;
                GtFrame gt;gt.ego=world.ego;
                runPlanningProcess(planning,gt,frameWorld,map,.05,&trace);
                tracking=planning.roadLanePlanner;
            } else buildCorridorTrackingPath(tracking,config.planner,world,map,mission,80.,&trace);
            ++polls;
            maxPollMs=std::max(maxPollMs,std::chrono::duration<double,std::milli>(Clock::now()-pollStart).count());
            success=options.queueApproach ? runtime.soleGatewayApproachSelected && runtime.trackingPathApproved :
                options.trackingRecovery ? runtime.trackingPathApproved &&
                    !std::isfinite(runtime.trackingGatewayStopDistance) :
                runtime.committedGlobalPathReachesGateway && !runtime.globalRouteReplanRequested;
            if(options.planningFrames>0 ? polls>=static_cast<std::size_t>(options.planningFrames) :
               (success || (!options.trackingRecovery && runtime.globalRouteSearchFailed))) break;
            if(std::chrono::duration<double>(Clock::now()-started).count()>=options.timeout) {
                timedOut=true; break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(trajectory.is_open()?50:10));
        }
        LaneChangeData local;
        if(options.localFallback) {
            // A detached successful global route can keep being rejected at
            // adoption until this diagnostic's global timeout. Local recovery
            // must still be exercised: that is precisely its live use case.
            timedOut=false;
            // The global planner above is unmodified. Exercise its local
            // fallback against the same fixed observation, without sending
            // commands or moving the vehicle synthetically.
            success=false;
            std::vector<Waypoint> observedReference;
            if(const auto samples=snapshot.get_child_optional("tracked_path")) {
                for(const auto& row:*samples) {
                    if(row.second.size()<2) continue;
                    auto value=row.second.begin();Waypoint p;
                    p.x=(value++)->second.get_value<double>();
                    p.y=value->second.get_value<double>();
                    observedReference.push_back(p);
                }
                for(std::size_t i=0;i<observedReference.size();++i) {
                    const auto a=i>0?i-1:i,b=std::min(i+1,observedReference.size()-1);
                    auto& p=observedReference[i];
                    p.yaw=std::atan2(observedReference[b].y-observedReference[a].y,
                                     observedReference[b].x-observedReference[a].x);
                    const auto lane=worldToLane(map,p.x,p.y,p.yaw);
                    if(!lane.valid) throw std::runtime_error("Recorded tracking sample cannot be map matched");
                    p.roadId=lane.roadId;p.sectionIndex=lane.sectionIndex;
                    p.laneId=lane.laneId;p.roadS=lane.roadS;
                    if(i) p.s=observedReference[i-1].s+std::hypot(p.x-observedReference[i-1].x,p.y-observedReference[i-1].y);
                }
            }
            const auto& localReference=observedReference.size()>=2?observedReference:tracking.waypoints;
            const auto localStart=Clock::now();
            maxPollMs=0.;polls=0;
            while(Clock::now()-localStart<std::chrono::duration<double>(options.timeout)) {
                const auto pollStart=Clock::now();
                beginDecisionTraceFrame(trace,polls,
                    std::chrono::duration<double>(pollStart-started).count());
                success=prepareLocalCorridorFallback(local,config.laneChange,
                    config.behavior,world,map,mission,localReference,&trace);
                ++polls;
                maxPollMs=std::max(maxPollMs,std::chrono::duration<double,std::milli>(Clock::now()-pollStart).count());
                if(success || !runtime.localFallbackEngaged ||
                   (!runtime.localFallbackPending && runtime.localFallbackIssued)) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            timedOut=!success && runtime.localFallbackPending;
        }
        const double wallMs=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
        const auto& path=options.localFallback?local.path:options.queueApproach?runtime.soleGatewayApproachPath:
            options.trackingRecovery?tracking.waypoints:runtime.committedGlobalPath;
        if(!path.empty() && !savePath(path,(directory/(options.localFallback?
                "local_path.csv":"global_path.csv")).string()))
            throw std::runtime_error("Cannot write diagnostic path CSV");
        if(!tracking.waypoints.empty()) savePath(tracking.waypoints,(directory/"tracking_path.csv").string());
        {
            std::ofstream goals(directory/"global_search_goals.csv");
            goals << "point_id,road_id,section_index,lane_id,road_s,x,y,handoff_cost\n";
            goals << std::setprecision(12);
            for(const auto& point:runtime.network.points) {
                if(point.id<0 || static_cast<std::size_t>(point.id)>=runtime.globalSearchGoal.size() ||
                   !runtime.globalSearchGoal[point.id]) continue;
                const auto& p=point.pose;
                goals << point.id << ',' << p.roadId << ',' << p.sectionIndex << ',' << p.laneId
                      << ',' << p.roadS << ',' << p.x << ',' << p.y << ','
                      << runtime.handoffCost[point.id] << '\n';
            }
        }
        {
            std::ofstream scope(directory/"global_search_scope.csv");
            scope << "point_id,road_id,section_index,lane_id,road_s,x,y,global_allowed,goal,ordinary,handoff_cost\n";
            scope << std::setprecision(12);
            for(const auto& point:runtime.network.points) {
                if(point.id<0 || static_cast<std::size_t>(point.id)>=runtime.trackingAllowed.size() ||
                   !runtime.trackingAllowed[point.id]) continue;
                const auto& p=point.pose;
                scope << point.id << ',' << p.roadId << ',' << p.sectionIndex << ',' << p.laneId
                      << ',' << p.roadS << ',' << p.x << ',' << p.y << ','
                      << runtime.globalSearchAllowed[point.id] << ',' << runtime.globalSearchGoal[point.id]
                      << ',' << runtime.ordinaryRoadPoint[point.id] << ',' << runtime.handoffCost[point.id] << '\n';
            }
        }
        int blocker=-1;
        const bool safe=!path.empty() && corridorTrackingPathIsSafe(path,world,
            options.localFallback?local.validatedObstacleClearance:
                runtime.globalPathClearance(),&blocker,&config.behavior,&map);
        const bool goal=!path.empty() && missionPointIsTarget(mission.legs[mission.activeLegIndex],path.back());
        std::ostringstream summary;
        summary << std::boolalpha << std::setprecision(10)
                << "{\"success\":" << success << ",\"goal\":" << goal
                << ",\"local_fallback\":" << options.localFallback
                << ",\"queue_approach\":" << runtime.soleGatewayApproachSelected
                << ",\"planning_frames\":" << options.planningFrames
                << ",\"planning_valid\":" << planning.result.valid
                << ",\"planning_speed\":" << planning.result.targetSpeed
                << ",\"planning_lane_change\":" << static_cast<int>(planning.laneChange.phase)
                << ",\"progress_recovery\":" << planning.result.progressRecoveryActive
                << ",\"contact_expected\":" << planning.result.progressRecoveryContactExpected
                << ",\"tracking_approved\":" << runtime.trackingPathApproved
                << ",\"reentry_selected\":" << runtime.trackingPathRelocalized
                << ",\"gateway_stop_required\":" << std::isfinite(runtime.trackingGatewayStopDistance)
                << ",\"validated_clearance\":" << (options.localFallback?local.validatedObstacleClearance:runtime.globalPathClearance())
                << ",\"safe\":" << safe << ",\"blocking_object\":" << blocker
                << ",\"timed_out\":" << timedOut << ",\"search_failed\":" << runtime.globalRouteSearchFailed
                << ",\"wall_ms\":" << wallMs << ",\"max_poll_ms\":" << maxPollMs
                << ",\"polls\":" << polls << ",\"point_count\":" << path.size()
                << ",\"revision\":" << runtime.routeRevision << ",\"request_revision\":" << runtime.globalRouteRequestRevision
                << ",\"leg\":" << mission.activeLegIndex
                << ",\"gateway_history_verified\":" << gatewayHistoryVerified
                << ",\"snapshot_time_s\":" << snapshot.get<double>("time_s",0.)
                << ",\"ego_lane\":[" << world.egoRoadId << ',' << world.egoSectionIndex << ',' << world.egoLaneId << ']'
                << ",\"nearest_lane\":[" << nearest.roadId << ',' << nearest.sectionIndex << ',' << nearest.laneId << ']'
                << ",\"ego_road_s\":" << world.egoRoadS << ",\"objects\":" << world.objects.size();
        if(!path.empty()) {
            const auto& last=path.back();
            summary << ",\"last_point\":{\"x\":" << last.x << ",\"y\":" << last.y
                    << ",\"road_id\":" << last.roadId << ",\"section_index\":" << last.sectionIndex
                    << ",\"lane_id\":" << last.laneId << ",\"road_s\":" << last.roadS << '}';
        }
        summary << "}\n";
        { std::ofstream out(directory/"summary.json"); out << summary.str(); }
        trace.output.flush(); std::cout << summary.str() << std::flush;
        // This isolated diagnostic process must not exceed its timeout while
        // std::future destruction joins an uncancellable production worker.
        if(timedOut) std::_Exit(124);
        return success?0:2;
    } catch(const std::exception& error) {
        std::cerr << "replay_corridor_scene: " << error.what() << '\n';
        return 1;
    }
}
