#pragma once

#include "data/vehicle/vehicle_structs.hpp"

#include <string>

bool loadVehicleGeometry(VehicleGeometry& geometry,
                         const std::string& yamlPath);
