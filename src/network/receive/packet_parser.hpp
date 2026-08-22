#pragma once

#include "data/gt/gt_structs.hpp"

#include <cstddef>
#include <cstdint>

bool parseGtFrame(
    const std::uint8_t* data,
    std::size_t size,
    GtFrame& output);
