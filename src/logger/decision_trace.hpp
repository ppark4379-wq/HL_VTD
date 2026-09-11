#pragma once

#include "data/map/map_structs.hpp"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

struct VisualizedCandidatePath {
    LaneKey lane;
    std::string status;
    std::string reason;
    std::vector<Waypoint> points;
};

// 상세 판단 로그는 일반 주행 CSV와 분리한다. 차선변경 후보가 선택되거나
// 탈락한 이유를 사람이 시간순으로 읽을 수 있도록 한 줄짜리 이벤트로 남긴다.
struct RepairTraceWriter;
struct DecisionTraceData {
    bool enabled{};
    std::filesystem::path outputPath;
    std::ofstream output;
    // Separate repair evidence; never alters the legacy decision log or console.
    std::shared_ptr<RepairTraceWriter> repairWriter;
    bool repairOnly{};
    std::size_t repairDetailCount{}, repairEventCount{};
    std::size_t frameIndex{};
    double elapsedWallTimeS{};
    std::vector<std::string> frameEvents;
    std::vector<LaneKey> candidateLanes;
    std::vector<VisualizedCandidatePath> candidatePaths;
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

void traceCandidateLane(DecisionTraceData* data, const LaneKey& lane);
void traceCandidatePath(DecisionTraceData* data,
                        const LaneKey& lane,
                        const std::vector<Waypoint>& points,
                        const std::string& status,
                        const std::string& reason = {});

void initializeRepairTrace(DecisionTraceData& data,
                           const std::filesystem::path& outputPath);
void traceRepairDecision(DecisionTraceData* data,
                         const std::string& category,
                         const std::string& event,
                         const std::string& detail);
void finishRepairTrace(DecisionTraceData& data);
