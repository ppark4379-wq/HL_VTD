#include "network/receive/gt_receiver.hpp"
#include <cassert>
#include <cstring>
#include <iostream>

int main() {
    GtReceiverData receiver;
    std::vector<std::uint8_t> packet(GT_FRAME_SIZE,0);
    auto add=[&](float x) {
        std::uint32_t bits;std::memcpy(&bits,&x,4);
        for(int i=0;i<4;++i) packet[i]=(bits>>(8*i))&255;
        receiver.streamBuffer.insert(receiver.streamBuffer.end(),packet.begin(),packet.end());
    };
    add(1);add(2);add(3);
    const auto trailing=std::vector<std::uint8_t>(packet.begin(),packet.begin()+17);
    receiver.streamBuffer.insert(receiver.streamBuffer.end(),trailing.begin(),trailing.end());
    assert(extractGtFrame(receiver));
    assert(receiver.latestGt.ego.x==3 && receiver.droppedFrames==2);
    assert(receiver.streamBuffer==trailing);
    assert(!extractGtFrame(receiver));
    receiver.streamBuffer.insert(receiver.streamBuffer.end(),packet.begin()+17,packet.end());
    assert(extractGtFrame(receiver));
    assert(receiver.latestGt.ego.x==3 && receiver.streamBuffer.empty());
    assert(receiver.droppedFrames==2);
    std::cout<<"GT latest complete frame and partial packet preservation: PASS\n";
}
