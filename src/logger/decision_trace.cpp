#include "logger/decision_trace.hpp"

#include <algorithm>
#include <iomanip>
#include <stdexcept>

void initializeDecisionTrace(DecisionTraceData& data,
                             const std::filesystem::path& outputPath) {
    data = {};
    data.outputPath = outputPath;
    if (outputPath.has_parent_path()) {
        std::filesystem::create_directories(outputPath.parent_path());
    }
    data.output.open(outputPath, std::ios::trunc);
    if (!data.output) {
        throw std::runtime_error(
            "cannot open decision log: " + outputPath.string());
    }
    data.enabled = true;
    data.output << "# HL_VTD detailed planning decision trace\n";
    data.output << "# time_s frame category event detail\n";
}

void beginDecisionTraceFrame(DecisionTraceData& data,
                             std::size_t frameIndex,
                             double elapsedWallTimeS) {
    // One busy obstacle frame can emit dozens of trace events. Flushing every
    // event stalls the control loop long enough for VTD's RDB socket backlog
    // to overflow. Preserve crash-readable logs while paying at most one
    // flush per completed frame.
    if (data.output && data.frameIndex != frameIndex) {
        data.output.flush();
    }
    data.frameIndex = frameIndex;
    data.elapsedWallTimeS = elapsedWallTimeS;
    data.frameEvents.clear();
    data.candidateLanes.clear();
    data.candidatePaths.clear();
}

void traceDecision(DecisionTraceData* data,
                   const std::string& category,
                   const std::string& event,
                   const std::string& detail) {
    if (data == nullptr || !data->enabled) return;
    if (data->output) {
        data->output << std::fixed << std::setprecision(3)
                     << data->elapsedWallTimeS << " frame=" << data->frameIndex
                     << " [" << category << "] " << event;
        if (!detail.empty()) data->output << " | " << detail;
        data->output << '\n';
    }
    data->frameEvents.push_back(
        category + ": " + event +
        (detail.empty() ? std::string{} : " | " + detail));
    if (data->frameEvents.size() > 12) {
        data->frameEvents.erase(data->frameEvents.begin());
    }
}

void traceCandidateLane(DecisionTraceData* data, const LaneKey& lane) {
    if (data == nullptr || !data->enabled) return;
    if (std::find(data->candidateLanes.begin(), data->candidateLanes.end(),
                  lane) == data->candidateLanes.end()) {
        data->candidateLanes.push_back(lane);
    }
}

void traceCandidatePath(DecisionTraceData* data,
                        const LaneKey& lane,
                        const std::vector<Waypoint>& points,
                        const std::string& status,
                        const std::string& reason) {
    if (data == nullptr || !data->enabled || points.size() < 2) {
        return;
    }
    if (data->candidatePaths.size() >= 24) {
        // 선택된 경로는 많은 탈락 후보 뒤에 생기더라도 RViz에 반드시
        // 남겨야 한다. 일반 후보는 프레임당 24개로 제한한다.
        if (status != "selected") return;
        data->candidatePaths.pop_back();
    }
    data->candidatePaths.push_back({lane, status, reason, points});
}
