#pragma once

#include "network/network_structs.hpp"

#include <filesystem>

namespace hlvtd::network {

NetworkConfig loadNetworkConfig(const std::filesystem::path& path);

}  // namespace hlvtd::network
