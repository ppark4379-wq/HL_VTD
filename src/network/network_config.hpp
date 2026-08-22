#pragma once

#include "network/network_structs.hpp"

#include <filesystem>

NetworkConfig loadNetworkConfig(const std::filesystem::path& path);
