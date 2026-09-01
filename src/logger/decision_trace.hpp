#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>

// 상세 판단 로그는 일반 주행 CSV와 분리한다. 차선변경 후보가 선택되거나
// 탈락한 이유를 사람이 시간순으로 읽을 수 있도록 한 줄짜리 이벤트로 남긴다.
struct DecisionTraceData {
    bool enabled{};
    std::filesystem::path outputPath;
    std::ofstream output;
    std::size_t frameIndex{};
    double elapsedWallTimeS{};
};

void initializeDecisionTrace(DecisionTraceData& data,
                             const std::filesystem::path& outputPath);
void beginDecisionTraceFrame(DecisionTraceData& data,
                             std::size_t frameIndex,
                             double elapsedWallTimeS);
void traceDecision(DecisionTraceData* data,
                   const std::string& category,
                   const std::string& event,
                   const std::string& detail);

