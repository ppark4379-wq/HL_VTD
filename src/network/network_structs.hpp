#pragma once

#include "data/gt_structs.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hlvtd::network {

inline constexpr std::size_t GT_FRAME_SIZE = 1109;

struct VtdNetworkConfig {
    std::string host_ip;
    std::uint16_t tcp_port{};
    std::uint16_t rtsp_port{};
};

struct ControllerNetworkConfig {
    std::string ip;
    std::uint16_t udp_port{};
};

struct NetworkConfig {
    VtdNetworkConfig vtd;
    ControllerNetworkConfig controller;
};

struct VtdClientData {
    std::string host;
    std::uint16_t port{};
    std::intptr_t socket{-1};
#ifdef _WIN32
    bool winsockInitialized{};
#endif
};

struct GtReceiverData {
    VtdClientData client;
    std::array<std::uint8_t, 64 * 1024> receiveBuffer{};
    std::vector<std::uint8_t> streamBuffer;
    GtFrame latestGt;
};

}  // namespace hlvtd::network
