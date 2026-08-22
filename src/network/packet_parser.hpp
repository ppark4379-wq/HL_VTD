#pragma once

#include "data/gt_structs.hpp"

#include <cstddef>
#include <cstdint>

namespace hlvtd::network {

bool parseGtFrame(
    const std::uint8_t* data,
    std::size_t size,
    GtFrame& output);

}  // namespace hlvtd::network
