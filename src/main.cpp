#include "network/receive/gt_receiver.hpp"
#include "network/receive/gt_printer.hpp"
#include "network/receive/rdb_clock.hpp"
#include "network/network_config.hpp"
#include "network/send/control_sender.hpp"
#include "network/vtd_client.hpp"
#include "control/config/control_config.hpp"
#include "control/control_process.hpp"
#include "logger/logger.hpp"
#include "logger/decision_trace.hpp"
#include "visualization/visualization_logger.hpp"
#include "planning/planning_process.hpp"
#include "data/gt/world_process.hpp"
#include "data/map/static_zones.hpp"
#include "data/vehicle/vehicle_config.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/global/mission_corridor.hpp"
#include "planning/local/corridor_lattice.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

struct CliOptions {
    std::filesystem::path config_path = "config/network.yaml";
    std::filesystem::path control_config_path = "config/control.yaml";
    std::filesystem::path reference_path = "output/reference_path.csv";
    std::filesystem::path checkpoint_path;
    std::filesystem::path mission_output_path;
    std::optional<std::size_t> mission_start_leg;
    std::filesystem::path map_config_path = "config/maps/living_lab.yaml";
    std::filesystem::path vehicle_config_path = "config/vehicle.yaml";
    CsvLoggerConfig logger;
    bool logging = false;
    bool check_config_only = false;
    bool receive_only = false;
    bool debug_objects = false;
    bool ignore_traffic_lights = false;
    bool decision_logging = false;
    std::filesystem::path decision_log_path;
    bool visualization_logging = false;
    std::filesystem::path visualization_directory;
    std::filesystem::path ready_file;
};

const char* stopReasonName(StopReason reason) {
    switch (reason) {
        case StopReason::None: return "none";
        case StopReason::RedLight: return "red_light";
        case StopReason::FlashingRed: return "flashing_red";
        case StopReason::Pedestrian: return "pedestrian";
        case StopReason::CrosswalkBlocked: return "crosswalk_blocked";
    }
    return "multiple";
}

const char* zeroTargetSpeedCause(const PlanningResult& planning) {
    if (planning.goalReached) return "goal_reached";
    if (planning.emergencyBrake) return "emergency_brake";
    if (planning.stopReason != StopReason::None) {
        return stopReasonName(planning.stopReason);
    }
    if (planning.health == SystemHealth::Recovery) return "recovery";
    if (!planning.valid) return "invalid_plan_or_input";
    return "planner_speed_constraint";
}

CliOptions parseOptions(int argc, char* argv[]) {
    CliOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--config") {
            if (++i >= argc) {
                throw std::invalid_argument("--config requires a file path");
            }
            options.config_path = argv[i];
        } else if (argument == "--control-config") {
            if (++i >= argc) {
                throw std::invalid_argument("--control-config requires a file path");
            }
            options.control_config_path = argv[i];
        } else if (argument == "--path") {
            if (++i >= argc) {
                throw std::invalid_argument("--path requires a file path");
            }
            options.reference_path = argv[i];
        } else if (argument == "--checkpoints") {
            if (++i >= argc) {
                throw std::invalid_argument("--checkpoints requires a file path");
            }
            options.checkpoint_path = argv[i];
        } else if (argument == "--mission-start-leg") {
            if (++i >= argc) {
                throw std::invalid_argument("--mission-start-leg requires an index");
            }
            const std::string value = argv[i];
            if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
                throw std::invalid_argument("--mission-start-leg must be a non-negative integer");
            }
            options.mission_start_leg = std::stoull(value);
        } else if (argument == "--mission-output") {
            if (++i >= argc) {
                throw std::invalid_argument(
                    "--mission-output requires an SVG file path");
            }
            options.mission_output_path = argv[i];
        } else if (argument == "--map-config") {
            if (++i >= argc) throw std::invalid_argument("--map-config requires a file path");
            options.map_config_path = argv[i];
        } else if (argument == "--vehicle-config") {
            if (++i >= argc) throw std::invalid_argument("--vehicle-config requires a file path");
            options.vehicle_config_path = argv[i];
        } else if (argument == "--log") {
            if (++i >= argc) {
                throw std::invalid_argument("--log requires a file path");
            }
            options.logger.outputPath = argv[i];
            options.logging = true;
        } else if (argument == "--scenario-id") {
            if (++i >= argc) {
                throw std::invalid_argument("--scenario-id requires a value");
            }
            options.logger.scenarioId = argv[i];
        } else if (argument == "--run-id") {
            if (++i >= argc) {
                throw std::invalid_argument("--run-id requires a value");
            }
            options.logger.runId = argv[i];
        } else if (argument == "--check-config") {
            options.check_config_only = true;
        } else if (argument == "--receive-only") {
            options.receive_only = true;
        } else if (argument == "--debug-objects") {
            options.debug_objects = true;
        } else if (argument == "--ignore-traffic-lights") {
            options.ignore_traffic_lights = true;
        } else if (argument == "--decision-log") {
            if (++i >= argc) {
                throw std::invalid_argument(
                    "--decision-log requires a file path");
            }
            options.decision_log_path = argv[i];
            options.decision_logging = true;
        } else if (argument == "--visualization-dir") {
            if (++i >= argc) {
                throw std::invalid_argument(
                    "--visualization-dir requires a directory path");
            }
            options.visualization_directory = argv[i];
            options.visualization_logging = true;
        } else if (argument == "--ready-file") {
            if (++i >= argc) {
                throw std::invalid_argument("--ready-file requires a file path");
            }
            options.ready_file = argv[i];
        } else if (argument == "--help" || argument == "-h") {
            std::cout
                << "Usage: hl_vtd [--config PATH] [--control-config PATH] "
                   "[--path PATH] [--checkpoints PATH] "
                   "[--mission-output SVG] [--mission-start-leg INDEX] [--map-config PATH] "
                   "[--vehicle-config PATH] "
                   "[--log CSV] [--scenario-id ID] "
                   "[--run-id ID] [--receive-only] [--debug-objects] "
                   "[--ignore-traffic-lights] "
                   "[--decision-log PATH] "
                   "[--visualization-dir DIR] "
                   "[--ready-file PATH] [--check-config]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + argument);
        }
    }
    return options;
}

void printDrivingStatus(const GtFrame& gt,
                        const PlanningResult& planning,
                        const ControlProcessData& control) {
    const PathPlan& plan = planning.trackingPlan;
    std::cout << "[drive] ego=(" << gt.ego.x << ", " << gt.ego.y << ')'
              << " speed=" << control.egoSpeed
              << " nearest=" << plan.nearestIndex
              << " target=" << plan.targetIndex
              << " Ld=" << plan.lookahead
              << " targetSpeed=" << plan.targetSpeed
              << " speedLimitKph=" << planning.currentSpeedLimit * 3.6
              << " schoolZone=" << planning.schoolZoneActive
              << " crosswalkControl=" << planning.crosswalkActive
              << " steering=" << control.latestCommand.steering
              << " targetAccel=" << control.latestCommand.targetAccel
              << " trafficLightId=" << gt.trafficLight.id
              << " trafficLightState="
              << static_cast<unsigned int>(gt.trafficLight.state);

    if (plan.targetSpeed <= 0.0) {
        std::cout << " zeroSpeedCause=" << zeroTargetSpeedCause(planning)
                  << " stopReason=" << stopReasonName(planning.stopReason)
                  << " goalReached=" << planning.goalReached
                  << " emergencyBrake=" << planning.emergencyBrake
                  << " collision=" << planning.pathSafety.collision
                  << " predictedCollision="
                  << planning.pathSafety.predictedCollision
                  << " riskObjectId=" << planning.pathSafety.riskObjectId;
    }
    std::cout << '\n';
}

struct FrameClockData {
    double previousReceiveTime{};
    double previousSimulationTime{};
    double dt{0.05};
    bool wallClockInitialized{};
    bool simulationClockInitialized{};
    bool usingSimulationTime{};
};

void updateFrameClock(FrameClockData& clock,
                      std::optional<double> simulationTime,
                      double receiveTime,
                      double nominalDt) {
    if (simulationTime.has_value() && std::isfinite(*simulationTime)) {
        double measured = nominalDt;
        if (clock.simulationClockInitialized) {
            if (*simulationTime >= clock.previousSimulationTime) {
                measured = *simulationTime - clock.previousSimulationTime;
            }
        }
        // RDB 시각은 시뮬레이션이 느리거나 빠르게 실행되어도 0.05 s 단위로
        // 진행한다. 큰 점프는 드롭된 GT 프레임의 실제 시간차이므로 유지한다.
        // 같은 simTime은 새 물리 프레임이 아니라 중복 snapshot이다.
        clock.dt = measured;
        clock.previousSimulationTime = *simulationTime;
        clock.simulationClockInitialized = true;
        clock.usingSimulationTime = true;
        return;
    }

    // 한 프레임이라도 RDB 좌표 매칭이 끊기면 다음 일치값을 새 기준점으로
    // 삼는다. 누락 구간을 다음 한 프레임의 이동량에 중복 적용하지 않는다.
    clock.simulationClockInitialized = false;
    clock.usingSimulationTime = false;
    const double measured =
        clock.wallClockInitialized && receiveTime > clock.previousReceiveTime
            ? receiveTime - clock.previousReceiveTime
            : nominalDt;
    // TCP는 여러 GT 프레임을 한 recv에 합칠 수 있다. 버퍼에서 연속으로
    // 꺼낸 프레임의 wall-clock dt가 거의 0이어도 시뮬레이션 주기는 0이 아니다.
    clock.dt = measured >= nominalDt * 0.25 && measured <= nominalDt * 4.0
                   ? measured
                   : nominalDt;
    clock.previousReceiveTime = receiveTime;
    clock.wallClockInitialized = true;
}

void printNetworkConfig(
    const CliOptions& options,
    const NetworkConfig& config) {
    std::cout << "[network] config: " << options.config_path << '\n'
              << "[network] VTD TCP: " << config.vtd.host_ip << ':'
              << config.vtd.tcp_port
              << " (1109-byte GT receive + 9-byte control send)\n"
              << "[network] VTD RDB clock: " << config.vtd.host_ip << ':'
              << config.vtd.rdb_port << " (frameNo/simTime)\n"
              << "[network] VTD RTSP: " << config.vtd.host_ip << ':'
              << config.vtd.rtsp_port << '\n'
              << "[network] configured controller UDP (unused by control): "
              << config.controller.ip << ':' << config.controller.udp_port
              << '\n';
}

void printMapReadiness(const StaticZoneData& zones,
                       double schoolZoneTargetSpeed) {
    const auto activeStopLines = std::count_if(
        zones.stopLines.begin(), zones.stopLines.end(),
        [](const StopLine& line) { return line.routeRelevant; });
    const auto activeCrosswalks = std::count_if(
        zones.crosswalks.begin(), zones.crosswalks.end(),
        [](const Crosswalk& crosswalk) { return crosswalk.routeRelevant; });
    const auto activeSpeedZones = std::count_if(
        zones.speedZones.begin(), zones.speedZones.end(),
        [](const SpeedZone& zone) { return zone.routeRelevant; });
    std::cout << "[map] static features: stop_lines=" << zones.stopLines.size()
              << " (route_active=" << activeStopLines << ')'
              << " crosswalks=" << zones.crosswalks.size()
              << " (route_active=" << activeCrosswalks << ')'
              << " speed_zones=" << zones.speedZones.size()
              << " (route_active=" << activeSpeedZones << ')'
              << " school_zone_reference_points="
              << zones.schoolZoneReferencePoints.size()
              << " flashing_red_ids="
              << zones.flashingRedControllerIds.size() << '\n';
    if (activeSpeedZones == 0) {
        std::cout << "[config-warning] current route has no protection/speed zone";
        if (!zones.schoolZoneReferencePoints.empty()) {
            std::cout << " (" << zones.schoolZoneReferencePoints.size()
                      << " map reference points loaded)";
        }
        std::cout << '\n';
    }
    for (const auto& zone : zones.speedZones) {
        if (!zone.routeRelevant || !zone.schoolZone ||
            !zone.hasRouteRange) continue;
        std::cout << "[map] school zone: route_s=["
                  << zone.routeSBegin << ", " << zone.routeSEnd
                  << "] map_limit=" << zone.speedLimit * 3.6
                  << " km/h control_target="
                  << schoolZoneTargetSpeed * 3.6 << " km/h\n";
    }
    if (zones.flashingRedControllerIds.empty()) {
        std::cout << "[config-warning] no packet traffic-light ID is marked as flashing red\n";
    }
    if (!zones.finish.valid) {
        std::cout << "[config-warning] official finish coordinate is not configured\n";
    }
}

void writeReadyFile(const std::filesystem::path& path) {
    if (path.empty()) return;
    std::ofstream stream(path, std::ios::trunc);
    if (!stream) {
        throw std::runtime_error("cannot write ready file: " + path.string());
    }
    stream << "connected\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const auto options = parseOptions(argc, argv);
        const auto config = loadNetworkConfig(options.config_path);

        DrivingConfig drivingConfig;
        if (!loadDrivingConfig(drivingConfig,
                               options.control_config_path.string())) {
            throw std::runtime_error(
                "cannot load control config: " +
                options.control_config_path.string());
        }
        drivingConfig.behavior.ignoreTrafficLights =
            options.ignore_traffic_lights;

        printNetworkConfig(options, config);

        VehicleGeometry vehicle;
        if (!loadVehicleGeometry(vehicle,
                                 options.vehicle_config_path.string())) {
            throw std::runtime_error("cannot load vehicle config: " +
                                     options.vehicle_config_path.string());
        }

        MapRuntimeConfig mapConfig;
        if (!loadMapRuntimeConfig(mapConfig,
                                  options.map_config_path.string())) {
            throw std::runtime_error("cannot load map config: " +
                                     options.map_config_path.string());
        }

        PlanningProcessData planning;
        if (!initializePlanning(planning, drivingConfig.planner,
                                drivingConfig.behavior,
                                drivingConfig.laneChange,
                                options.reference_path.string())) {
            throw std::runtime_error(
                "cannot load reference path: " + options.reference_path.string());
        }

        PathGeneratorData mapData;
        if (!loadMap(mapData, mapConfig.xodrPath)) {
            throw std::runtime_error("cannot load XODR map: " +
                                     mapConfig.xodrPath);
        }
        if (options.mission_start_leg && options.checkpoint_path.empty()) {
            throw std::invalid_argument("--mission-start-leg requires --checkpoints");
        }
        if (!options.checkpoint_path.empty()) {
            std::string networkError;
            if(!initializeCorridorRuntime(mapData,"output/precomputed_lane_network",
                                          drivingConfig.laneChange,&networkError))
                throw std::runtime_error("cannot load precomputed corridor network: "+networkError);
            if (!preparePrecomputedMission(
                    planning.missionCorridor, mapData,
                    options.checkpoint_path.string(),
                    mapConfig.xodrPath, "output/precomputed_lane_network",
                    "output/runtime_mission", &networkError)) {
                throw std::runtime_error(
                    "cannot prepare checkpoint mission corridor: " + networkError);
            }
            const std::size_t startLeg = options.mission_start_leg.value_or(0);
            if (startLeg >= planning.missionCorridor.legs.size()) {
                throw std::invalid_argument("--mission-start-leg exceeds the available mission legs");
            }
            planning.missionCorridor.activeLegIndex = startLeg;
            const auto& start = planning.missionCorridor.legs[startLeg];
            std::cout << "[mission] start_leg=" << startLeg
                      << " checkpoint " << start.fromCheckpoint + 1
                      << " -> gateway " << start.gatewayCheckpoint + 1
                      << " -> exit " << start.toCheckpoint + 1 << '\n';
            std::cout << "[mission] loaded "
                      << planning.missionCorridor.checkpoints.size()
                      << " checkpoints and "
                      << planning.missionCorridor.legs.size()
                      << " free-driving corridor legs\n";
            if (!options.mission_output_path.empty()) {
                const std::filesystem::path parent =
                    options.mission_output_path.parent_path();
                if (!parent.empty()) {
                    std::filesystem::create_directories(parent);
                }
                std::filesystem::path csvPath = options.mission_output_path;
                csvPath.replace_extension(".csv");
                if (!saveMissionCorridorArtifacts(
                        planning.missionCorridor, mapData,
                        planning.pathPlanner.waypoints,
                        options.mission_output_path.string(),
                        csvPath.string())) {
                    throw std::runtime_error(
                        "cannot save mission corridor artifacts: " +
                        options.mission_output_path.string());
                }
                std::cout << "[mission] visualization: "
                          << options.mission_output_path << '\n';
            }
        }
        if (!alignStaticZonesToRoute(
                mapConfig.zones, planning.pathPlanner.waypoints)) {
            throw std::runtime_error(
                "cannot align static zones to reference path");
        }
        mapData.staticZones = mapConfig.zones;
        std::cout << "[map] XODR loaded\n";
        printMapReadiness(mapData.staticZones,
                          drivingConfig.planner.schoolZoneSpeed);

        VisualizationLoggerData visualization;
        if (options.visualization_logging) {
            initializeVisualizationLogger(
                visualization, options.visualization_directory,
                mapData, planning.missionCorridor);
            std::cout << "[rviz-data] static map: "
                      << visualization.staticMapPath << '\n'
                      << "[rviz-data] live frame: "
                      << visualization.latestFramePath << '\n';
        }

        if (options.check_config_only) {
            std::cout << "[config] all configured files and values are valid\n";
            return 0;
        }

        ControlProcessData control;
        control.controller.config = drivingConfig.controller;
        WorldProcessData world;
        world.config.speedFilterAlpha =
            drivingConfig.runtime.speedFilterAlpha;
        world.config.maxValidSpeed =
            drivingConfig.runtime.maxValidPositionSpeed;
        FrameClockData clock;

        CsvLoggerData logger;
        if (options.logging) {
            initializeCsvLogger(logger, options.logger);
            std::cout << "[logger] output: " << logger.outputPath << '\n';
        }

        DecisionTraceData decisionTrace;
        const bool decisionLogging = options.decision_logging ||
                                     options.visualization_logging;
        if (options.decision_logging) {
            initializeDecisionTrace(decisionTrace,
                                    options.decision_log_path);
            std::cout << "[decision-log] output: "
                      << decisionTrace.outputPath << '\n';
        } else if (options.visualization_logging) {
            // RViz용 프레임 후보는 메모리에만 수집한다. 20 Hz 제어
            // 루프에서 판단 로그를 매 이벤트마다 flush하지 않는다.
            decisionTrace.enabled = true;
        }

        auto receiver = makeGtReceiver(config.vtd);
        connectGtReceiver(receiver);
        writeReadyFile(options.ready_file);
        setVtdReceiveTimeout(receiver.client,
                             drivingConfig.runtime.inputTimeout);

        auto rdbClock = makeRdbClock(config.vtd);
        std::size_t nextRdbConnectionAttemptFrame = 0;
        bool simulationClockAnnounced = false;

        using LogClock = std::chrono::steady_clock;
        const auto logStartTime = LogClock::now();
        std::size_t frameCount = 0;
        std::size_t reportedDroppedGtFrames = 0;

        while (receiveGtFrame(receiver)) {
            const GtFrame& gt = getLatestGtFrame(receiver);
            if (!isRdbClockConnected(rdbClock) &&
                frameCount >= nextRdbConnectionAttemptFrame) {
                // VTD 초기화 순서에 따라 첫 GT 시점에는 RDB 포트가 아직
                // 준비되지 않을 수 있으므로 5초 간격으로 재시도한다.
                nextRdbConnectionAttemptFrame = frameCount + 100;
                try {
                    connectRdbClock(rdbClock);
                    setVtdReceiveTimeout(rdbClock.client, 0.002);
                    std::cout << "[network] RDB simulation clock connected\n";
                } catch (const std::exception& error) {
                    disconnectRdbClock(rdbClock);
                    std::cerr
                        << "[network-warning] RDB simulation clock unavailable; "
                        << "using guarded receive-time fallback: "
                        << error.what() << '\n';
                }
            }
            std::optional<double> simulationTime;
            if (isRdbClockConnected(rdbClock)) {
                try {
                    pollRdbClock(rdbClock);
                    simulationTime = matchRdbSimulationTime(rdbClock, gt.ego);
                } catch (const std::exception& error) {
                    disconnectRdbClock(rdbClock);
                    nextRdbConnectionAttemptFrame = frameCount + 100;
                    std::cerr
                        << "[network-warning] RDB simulation clock lost; "
                        << "using guarded receive-time fallback: "
                        << error.what() << '\n';
                }
            }
            const double elapsedWallTime = std::chrono::duration<double>(
                LogClock::now() - logStartTime).count();
            DecisionTraceData* trace = decisionLogging
                                           ? &decisionTrace
                                           : nullptr;
            if (trace != nullptr) {
                beginDecisionTraceFrame(*trace, frameCount,
                                        elapsedWallTime);
                if (receiver.droppedFrames != reportedDroppedGtFrames) {
                    traceDecision(trace, "network", "stale_gt_frames_discarded",
                        "count=" + std::to_string(receiver.droppedFrames-reportedDroppedGtFrames) +
                        " total=" + std::to_string(receiver.droppedFrames));
                    reportedDroppedGtFrames = receiver.droppedFrames;
                }
            }
            updateFrameClock(clock, simulationTime,
                             getLatestGtReceiveTime(receiver),
                             drivingConfig.runtime.nominalDt);
            if (clock.usingSimulationTime && !simulationClockAnnounced) {
                std::cout << "[network] ego speed now uses VTD simulation time\n";
                simulationClockAnnounced = true;
            }
            runWorldProcess(world, gt, getLatestGtReceiveTime(receiver), clock.dt,
                            mapData, planning.pathPlanner.waypoints, vehicle,
                            &planning.missionCorridor);
            runPlanningProcess(planning, gt, world.state, mapData, clock.dt,
                               trace);
            runControlProcess(control, world.state, planning.result, clock.dt);

            if (options.visualization_logging) {
                writeVisualizationFrame(
                    visualization, frameCount, elapsedWallTime,
                    world.state, planning, control, trace);
            }

            if (!options.receive_only) {
                sendControlCommand(receiver.client, control.safeCommand);
            }

            if (options.logging) {
                const LogFrameInput logInput{
                    frameCount, elapsedWallTime, gt, world.state,
                    planning.result, control};
                runLoggingProcess(logger, logInput);
            }

            if (frameCount % 10 == 0) {
                printDrivingStatus(gt, planning.result, control);
                if (options.debug_objects) {
                    printWorldObjectStates(world.state);
                }
            }
            ++frameCount;

            if (planning.result.goalReached) {
                std::cout << "[drive] goal reached\n";
                break;
            }
        }

        if (didVtdReceiveTimeOut(receiver.client)) {
            ControlCommand timeoutCommand{};
            timeoutCommand.targetAccel = static_cast<float>(
                drivingConfig.controller.minAcceleration);
            sendControlCommand(receiver.client, timeoutCommand);
            std::cerr << "[safety] GT receive timeout; emergency brake sent\n";
        }

        disconnectGtReceiver(receiver);
        disconnectRdbClock(rdbClock);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[error] " << error.what() << '\n';
        return 1;
    }
}
