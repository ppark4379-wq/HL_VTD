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
#include "planning/planning_process.hpp"
#include "data/gt/world_process.hpp"
#include "data/map/static_zones.hpp"
#include "data/vehicle/vehicle_config.hpp"
#include "planning/global/reference_path_generator.hpp"

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
        } else if (argument == "--ready-file") {
            if (++i >= argc) {
                throw std::invalid_argument("--ready-file requires a file path");
            }
            options.ready_file = argv[i];
        } else if (argument == "--help" || argument == "-h") {
            std::cout
                << "Usage: hl_vtd [--config PATH] [--control-config PATH] "
                   "[--path PATH] [--map-config PATH] [--vehicle-config PATH] "
                   "[--log CSV] [--scenario-id ID] "
                   "[--run-id ID] [--receive-only] [--debug-objects] "
                   "[--ignore-traffic-lights] "
                   "[--decision-log PATH] "
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
    std::cout << "[map] static features: stop_lines=" << zones.stopLines.size()
              << " crosswalks=" << zones.crosswalks.size()
              << " speed_zones=" << zones.speedZones.size()
              << " school_zone_reference_points="
              << zones.schoolZoneReferencePoints.size()
              << " flashing_red_ids="
              << zones.flashingRedControllerIds.size() << '\n';
    if (zones.speedZones.empty()) {
        std::cout << "[config-warning] current route has no protection/speed zone";
        if (!zones.schoolZoneReferencePoints.empty()) {
            std::cout << " (" << zones.schoolZoneReferencePoints.size()
                      << " map reference points loaded)";
        }
        std::cout << '\n';
    }
    for (const auto& zone : zones.speedZones) {
        if (!zone.schoolZone || !zone.hasRouteRange) continue;
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
        if (!alignStaticZonesToRoute(
                mapConfig.zones, planning.pathPlanner.waypoints)) {
            throw std::runtime_error(
                "cannot align static zones to reference path");
        }
        mapData.staticZones = mapConfig.zones;
        std::cout << "[map] XODR loaded\n";
        printMapReadiness(mapData.staticZones,
                          drivingConfig.planner.schoolZoneSpeed);

        if (options.check_config_only) {
            std::cout << "[config] all configured files and values are valid\n";
            return 0;
        }

        ControlProcessData control;
        control.controller.config = drivingConfig.controller;
        WorldProcessData world;
        FrameClockData clock;

        CsvLoggerData logger;
        if (options.logging) {
            initializeCsvLogger(logger, options.logger);
            std::cout << "[logger] output: " << logger.outputPath << '\n';
        }

        DecisionTraceData decisionTrace;
        if (options.decision_logging) {
            initializeDecisionTrace(decisionTrace,
                                    options.decision_log_path);
            std::cout << "[decision-log] output: "
                      << decisionTrace.outputPath << '\n';
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
            DecisionTraceData* trace = options.decision_logging
                                           ? &decisionTrace
                                           : nullptr;
            if (trace != nullptr) {
                beginDecisionTraceFrame(*trace, frameCount,
                                        elapsedWallTime);
            }
            updateFrameClock(clock, simulationTime,
                             getLatestGtReceiveTime(receiver),
                             drivingConfig.runtime.nominalDt);
            if (clock.usingSimulationTime && !simulationClockAnnounced) {
                std::cout << "[network] ego speed now uses VTD simulation time\n";
                simulationClockAnnounced = true;
            }
            runWorldProcess(world, gt, getLatestGtReceiveTime(receiver), clock.dt,
                            mapData, planning.pathPlanner.waypoints, vehicle);
            runPlanningProcess(planning, gt, world.state, mapData, clock.dt,
                               trace);
            runControlProcess(control, world.state, planning.result, clock.dt);

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
