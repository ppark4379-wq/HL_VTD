#include "network/receive/gt_receiver.hpp"
#include "network/receive/gt_printer.hpp"
#include "network/network_config.hpp"
#include "network/send/control_sender.hpp"
#include "control/control_config.hpp"
#include "control/control_process.hpp"
#include "logger/logger.hpp"
#include "planning/planning_process.hpp"
#include "data/gt/gt_processor.hpp"
#include "planning/reference_path_generator.hpp"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct CliOptions {
    std::filesystem::path config_path = "config/network.yaml";
    std::filesystem::path control_config_path = "config/control.yaml";
    std::filesystem::path reference_path = "output/reference_path.csv";
    CsvLoggerConfig logger;
    bool logging = false;
    bool check_config_only = false;
    bool receive_only = false;
    bool debug_objects = false;
};

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
        } else if (argument == "--help" || argument == "-h") {
            std::cout
                << "Usage: hl_vtd [--config PATH] [--control-config PATH] "
                   "[--path PATH] [--log CSV] [--scenario-id ID] "
                   "[--run-id ID] [--receive-only] [--debug-objects] "
                   "[--check-config]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + argument);
        }
    }
    return options;
}

void printDrivingStatus(const GtFrame& gt,
                        const PathPlan& plan,
                        const ControlProcessData& control) {
    std::cout << "[drive] ego=(" << gt.ego.x << ", " << gt.ego.y << ')'
              << " speed=" << control.egoSpeed
              << " nearest=" << plan.nearestIndex
              << " target=" << plan.targetIndex
              << " Ld=" << plan.lookahead
              << " targetSpeed=" << plan.targetSpeed
              << " steering=" << control.latestCommand.steering
              << " targetAccel=" << control.latestCommand.targetAccel
              << " trafficLightId=" << gt.trafficLight.id
              << " trafficLightState="
              << static_cast<unsigned int>(gt.trafficLight.state)
              << '\n';
}

void printNetworkConfig(
    const CliOptions& options,
    const NetworkConfig& config) {
    std::cout << "[network] config: " << options.config_path << '\n'
              << "[network] VTD TCP: " << config.vtd.host_ip << ':'
              << config.vtd.tcp_port << '\n'
              << "[network] VTD RTSP: " << config.vtd.host_ip << ':'
              << config.vtd.rtsp_port << '\n'
              << "[network] controller UDP: " << config.controller.ip << ':'
              << config.controller.udp_port << '\n';
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

        printNetworkConfig(options, config);

        if (options.check_config_only) {
            std::cout << "[network] config is valid\n";
            return 0;
        }

        PathPlannerData planner;
        planner.config = drivingConfig.planner;
        if (!initializePlanning(planner, options.reference_path.string())) {
            throw std::runtime_error(
                "cannot load reference path: " + options.reference_path.string());
        }

        PathGeneratorData mapData;
        const std::string xodrPath =
            "src/data/map/HL_FMA_VTD_LivingLab.xodr";
        if (!loadMap(mapData, xodrPath)) {
            throw std::runtime_error("cannot load XODR map: " + xodrPath);
        }
        std::cout << "[map] XODR loaded\n";

        ControlProcessData control;
        control.controller.config = drivingConfig.controller;

        CsvLoggerData logger;
        if (options.logging) {
            initializeCsvLogger(logger, options.logger);
            std::cout << "[logger] output: " << logger.outputPath << '\n';
        }

        auto receiver = makeGtReceiver(config.vtd);
        connectGtReceiver(receiver);

        using LogClock = std::chrono::steady_clock;
        const auto logStartTime = LogClock::now();
        std::size_t frameCount = 0;

        while (receiveGtFrame(receiver)) {
            const GtFrame& gt = getLatestGtFrame(receiver);
            const WorldState world =
                makeWorldState(gt, mapData, planner.waypoints);

            const PathPlan plan = runPlanningProcess(planner, gt, world);
            const ControlCommand command =
                runControlProcess(control, gt, plan, drivingConfig.runtime);

            if (!options.receive_only) {
                sendControlCommand(receiver.client, command);
            }

            if (options.logging) {
                const double elapsedWallTime = std::chrono::duration<double>(
                    LogClock::now() - logStartTime).count();
                const LogFrameInput logInput{
                    frameCount, elapsedWallTime, gt, plan, control};
                runLoggingProcess(logger, logInput);
            }

            if (frameCount % 10 == 0) {
                printDrivingStatus(gt, plan, control);
                if (options.debug_objects) {
                    printWorldObjectStates(world);
                }
            }
            ++frameCount;

            if (plan.goalReached) {
                std::cout << "[drive] goal reached\n";
                break;
            }
        }

        disconnectGtReceiver(receiver);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[error] " << error.what() << '\n';
        return 1;
    }
}
