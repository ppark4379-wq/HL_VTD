#include "network/gt_receiver.hpp"
#include "network/gt_printer.hpp"
#include "network/network_config.hpp"

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct CliOptions {
    std::filesystem::path config_path = "config/network.yaml";
    bool check_config_only = false;
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
        } else if (argument == "--check-config") {
            options.check_config_only = true;
        } else if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: hl_vtd [--config PATH] [--check-config]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + argument);
        }
    }
    return options;
}

void printNetworkConfig(
    const CliOptions& options,
    const hlvtd::network::NetworkConfig& config) {
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
        const auto config = hlvtd::network::loadNetworkConfig(options.config_path);

        printNetworkConfig(options, config);

        if (options.check_config_only) {
            std::cout << "[network] config is valid\n";
            return 0;
        }

        auto receiver = hlvtd::network::makeGtReceiver(config.vtd);
        hlvtd::network::connectGtReceiver(receiver);

        while (hlvtd::network::receiveGtFrame(receiver)) {
            hlvtd::network::printGtFrame(
                hlvtd::network::getLatestGtFrame(receiver));
        }

        hlvtd::network::disconnectGtReceiver(receiver);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[error] " << error.what() << '\n';
        return 1;
    }
}
