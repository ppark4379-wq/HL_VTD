#pragma once

#include "control/control_structs.hpp"
#include "network/network_structs.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

inline constexpr std::size_t CONTROL_PACKET_SIZE = 9;
using ControlPacket = std::array<std::uint8_t, CONTROL_PACKET_SIZE>;

// 제어 구조체를 padding 없는 9 Byte 전송 패킷으로 만든다.
ControlPacket makeControlPacket(const ControlCommand& command);

// 이미 연결된 VTD TCP 소켓으로 제어 명령 한 개를 전송한다.
void sendControlCommand(VtdClientData& client,
                        const ControlCommand& command);
