#pragma once

#include "control/control_structs.hpp"

#include <string>

// control.yaml을 읽어 Pure Pursuit, 속도, PD, 명령 제한값을 로드한다.
bool loadDrivingConfig(DrivingConfig& config,
                       const std::string& yamlPath);
