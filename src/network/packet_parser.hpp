#pragma once

#include <cstddef>
#include <cstdint>

#include "../data/gt_state.hpp"

class PacketParser
{
public:
    // 현재 실제 수신으로 확인된 GT Frame 크기
    static constexpr std::size_t GT_FRAME_SIZE = 1109;

    // 1109 Byte Raw GT -> GtFrame
    static bool parseGtFrame(
        const std::uint8_t* data,
        std::size_t size,
        GtFrame& output
    );
};