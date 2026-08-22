#pragma once

#include "network/network_config.hpp"
#include "network/network_structs.hpp"

#include <cstddef>

VtdClientData makeVtdClient(const VtdNetworkConfig& config);

void connectVtdClient(VtdClientData& client);
void disconnectVtdClient(VtdClientData& client) noexcept;

bool isVtdClientConnected(const VtdClientData& client) noexcept;

void sendVtdData(
    VtdClientData& client,
    const void* data,
    std::size_t size);

std::size_t receiveVtdData(
    VtdClientData& client,
    void* buffer,
    std::size_t capacity);
