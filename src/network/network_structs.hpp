#pragma once

#include "data/gt/gt_structs.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

inline constexpr std::size_t GT_FRAME_SIZE = 1109;

struct VtdNetworkConfig {
    std::string host_ip;
    std::uint16_t tcp_port{};
    std::uint16_t rtsp_port{};
    // VTD 표준 RDB 스트림. GT 패킷에서 빠진 frameNo/simTime을 읽는다.
    std::uint16_t rdb_port{48190};
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
    bool receiveTimedOut{};
#ifdef _WIN32
    bool winsockInitialized{};
#endif
};

struct GtReceiverData {
    VtdClientData client;
    std::array<std::uint8_t, 64 * 1024> receiveBuffer{};
    std::vector<std::uint8_t> streamBuffer;
    GtFrame latestGt;
    double latestReceiveTime{};
    std::size_t droppedFrames{};
};

struct RdbClockSample {
    std::uint32_t frameNo{};
    double simulationTime{};
    double egoX{};
    double egoY{};
};

struct RdbClockData {
    VtdClientData client;
    std::array<std::uint8_t, 64 * 1024> receiveBuffer{};
    std::vector<std::uint8_t> streamBuffer;
    std::deque<RdbClockSample> samples;
    RdbClockSample lastMatchedSample;
    bool hasLastMatchedSample{};
};
