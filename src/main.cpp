#include "network/receive/gt_receiver.hpp"
#include "network/receive/gt_printer.hpp"
#include "network/network_config.hpp"
#include "network/send/control_sender.hpp"
#include "control/control_config.hpp"
#include "control/control_process.hpp"
#include "planning/planning_process.hpp"

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
    bool check_config_only = false;
    bool receive_only = false;
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
        } else if (argument == "--check-config") {
            options.check_config_only = true;
        } else if (argument == "--receive-only") {
            options.receive_only = true;
        } else if (argument == "--help" || argument == "-h") {
            std::cout
                << "Usage: hl_vtd [--config PATH] [--control-config PATH] "
                   "[--path PATH] [--receive-only] [--check-config]\n";
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

        ControlProcessData control;
        control.controller.config = drivingConfig.controller;

        auto receiver = makeGtReceiver(config.vtd);
        connectGtReceiver(receiver);

        using Clock = std::chrono::steady_clock;
        auto previousTime = Clock::now();
        std::size_t frameCount = 0;

        while (receiveGtFrame(receiver)) {
            const GtFrame& gt = getLatestGtFrame(receiver);
            const auto currentTime = Clock::now();
            double dt = std::chrono::duration<double>(
                currentTime - previousTime).count();
            previousTime = currentTime;
            if (frameCount == 0 || dt <= 0.0 || dt > 1.0) {
                dt = drivingConfig.nominalDt;
            }

            const PathPlan plan = runPlanningProcess(planner, gt);
            const ControlCommand command =
                runControlProcess(control, gt, plan, dt);

            if (!options.receive_only) {
                sendControlCommand(receiver.client, command);
            }

            if (frameCount % 10 == 0) {
                printDrivingStatus(gt, plan, control);
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
