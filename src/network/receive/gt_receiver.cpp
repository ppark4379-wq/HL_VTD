#include "network/receive/gt_receiver.hpp"

#include "network/receive/packet_parser.hpp"
#include "network/vtd_client.hpp"

#include <chrono>
#include <algorithm>

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
    const auto complete=receiver.streamBuffer.size()/GT_FRAME_SIZE;
    const auto skipped=(complete-1)*GT_FRAME_SIZE;
    if (!parseGtFrame(
            receiver.streamBuffer.data()+skipped,
            GT_FRAME_SIZE,
            receiver.latestGt)) {
        return false;
    }

    receiver.streamBuffer.erase(
        receiver.streamBuffer.begin(),
        receiver.streamBuffer.begin() + skipped + GT_FRAME_SIZE);
    receiver.droppedFrames+=complete-1;
    receiver.latestReceiveTime = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return true;
}

bool receiveGtFrame(GtReceiverData& receiver) {
    while (receiver.streamBuffer.size() < GT_FRAME_SIZE) {
        if (!receiveGtStreamData(receiver)) {
            return false;
        }
    }
    // Read only the bytes already queued at this instant. Never wait for the
    // next packet while draining, and retain an incomplete trailing packet.
    std::size_t pending=std::min(pendingVtdBytes(receiver.client),
                               receiver.receiveBuffer.size());
    while(pending>0) {
        const auto received=receiveVtdData(receiver.client,
            receiver.receiveBuffer.data(),pending);
        if(received==0) break;
        receiver.streamBuffer.insert(receiver.streamBuffer.end(),
            receiver.receiveBuffer.begin(),receiver.receiveBuffer.begin()+received);
        pending-=received;
    }
    return extractGtFrame(receiver);
}

const GtFrame& getLatestGtFrame(const GtReceiverData& receiver) {
    return receiver.latestGt;
}

double getLatestGtReceiveTime(const GtReceiverData& receiver) {
    return receiver.latestReceiveTime;
}
