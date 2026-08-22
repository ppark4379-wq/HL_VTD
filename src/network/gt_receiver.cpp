#include "network/gt_receiver.hpp"

#include "network/packet_parser.hpp"
#include "network/vtd_client.hpp"

namespace hlvtd::network {

GtReceiverData makeGtReceiver(const VtdNetworkConfig& config) {
    GtReceiverData receiver;
    receiver.client = makeVtdClient(config);
    receiver.streamBuffer.reserve(GT_FRAME_SIZE * 2);
    return receiver;
}

void connectGtReceiver(GtReceiverData& receiver) {
    connectVtdClient(receiver.client);
}

void disconnectGtReceiver(GtReceiverData& receiver) noexcept {
    disconnectVtdClient(receiver.client);
}

bool receiveGtStreamData(GtReceiverData& receiver) {
    const std::size_t received = receiveVtdData(
        receiver.client,
        receiver.receiveBuffer.data(),
        receiver.receiveBuffer.size());
    if (received == 0) {
        return false;
    }

    receiver.streamBuffer.insert(
        receiver.streamBuffer.end(),
        receiver.receiveBuffer.begin(),
        receiver.receiveBuffer.begin() + received);
    return true;
}

bool extractGtFrame(GtReceiverData& receiver) {
    if (receiver.streamBuffer.size() < GT_FRAME_SIZE) {
        return false;
    }
    if (!parseGtFrame(
            receiver.streamBuffer.data(),
            GT_FRAME_SIZE,
            receiver.latestGt)) {
        return false;
    }

    receiver.streamBuffer.erase(
        receiver.streamBuffer.begin(),
        receiver.streamBuffer.begin() + GT_FRAME_SIZE);
    return true;
}

bool receiveGtFrame(GtReceiverData& receiver) {
    while (receiver.streamBuffer.size() < GT_FRAME_SIZE) {
        if (!receiveGtStreamData(receiver)) {
            return false;
        }
    }
    return extractGtFrame(receiver);
}

const GtFrame& getLatestGtFrame(const GtReceiverData& receiver) {
    return receiver.latestGt;
}

}  // namespace hlvtd::network
