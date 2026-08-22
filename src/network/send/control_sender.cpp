#include "network/send/control_sender.hpp"

#include "network/vtd_client.hpp"

#include <cstring>

// float32 하나를 GT 패킷과 동일한 little-endian 4 Byte로 저장한다.
void writeFloat32LE(float value, std::uint8_t* output) {
    static_assert(sizeof(float) == 4, "control protocol requires float32");
    std::uint32_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    output[0] = static_cast<std::uint8_t>(bits);
    output[1] = static_cast<std::uint8_t>(bits >> 8);
    output[2] = static_cast<std::uint8_t>(bits >> 16);
    output[3] = static_cast<std::uint8_t>(bits >> 24);
}

// 패킷 생성 함수: steering 4B + targetAccel 4B + turnSignal 1B를 순서대로 넣는다.
ControlPacket makeControlPacket(const ControlCommand& command) {
    ControlPacket packet{};
    writeFloat32LE(command.steering, packet.data());
    writeFloat32LE(command.targetAccel, packet.data() + 4);
    packet[8] = command.turnSignal;
    return packet;
}

// 패킷 전송 함수: 제어 명령을 패킷으로 만들어 기존 TCP 소켓으로 보낸다.
void sendControlCommand(VtdClientData& client,
                        const ControlCommand& command) {
    const ControlPacket packet = makeControlPacket(command);
    sendVtdData(client, packet.data(), packet.size());
}
