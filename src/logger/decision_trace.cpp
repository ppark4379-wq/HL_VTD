#include "logger/decision_trace.hpp"

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
    data.frameIndex = frameIndex;
    data.elapsedWallTimeS = elapsedWallTimeS;
}

void traceDecision(DecisionTraceData* data,
                   const std::string& category,
                   const std::string& event,
                   const std::string& detail) {
    if (data == nullptr || !data->enabled || !data->output) return;
    data->output << std::fixed << std::setprecision(3)
                 << data->elapsedWallTimeS << " frame=" << data->frameIndex
                 << " [" << category << "] " << event;
    if (!detail.empty()) data->output << " | " << detail;
    data->output << '\n';
    data->output.flush();
}

