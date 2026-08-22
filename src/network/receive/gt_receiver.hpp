#pragma once

#include "data/gt/gt_structs.hpp"
#include "network/network_config.hpp"
#include "network/network_structs.hpp"

GtReceiverData makeGtReceiver(const VtdNetworkConfig& config);

void connectGtReceiver(GtReceiverData& receiver);
void disconnectGtReceiver(GtReceiverData& receiver) noexcept;

bool receiveGtStreamData(GtReceiverData& receiver);
bool extractGtFrame(GtReceiverData& receiver);
bool receiveGtFrame(GtReceiverData& receiver);

const GtFrame& getLatestGtFrame(const GtReceiverData& receiver);
