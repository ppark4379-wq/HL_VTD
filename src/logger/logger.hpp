#pragma once

#include "logger/log_types.hpp"

// Logger 상태를 초기화하고 CSV header를 기록한다.
void initializeCsvLogger(CsvLoggerData& logger,
                         const CsvLoggerConfig& config);

// GT, Planning, Control 결과를 한 프레임 로그로 변환해 CSV에 기록한다.
void runLoggingProcess(CsvLoggerData& logger,
                       const LogFrameInput& input);
