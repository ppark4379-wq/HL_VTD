#include "logger/decision_trace.hpp"

#include "logger/diagnostic_queue.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>
#include <iomanip>
#include <iostream>
#include <stdexcept>

struct RepairTraceRecord {
    std::size_t frame{};
    double wallTime{};
    std::array<char, 64> category{};
    std::array<char, 96> event{};
    std::array<char, 1536> detail{};
    bool truncated{};
};

struct RepairTraceWriter {
    DiagnosticQueue<RepairTraceRecord, 512> queue;
    std::ofstream output, health;
    std::thread thread;
    std::atomic<bool> stopping{false}, failed{false};
    std::atomic<std::size_t> limited{0}, duplicates{0}, truncated{0};
    ~RepairTraceWriter();
};

namespace {

void copyTraceText(char* destination, std::size_t capacity, const std::string& source) {
    const auto count = std::min(source.size(), capacity - 1);
    std::memcpy(destination, source.data(), count);
    destination[count] = '\0';
    for (std::size_t i = 0; i < count; ++i) {
        if (destination[i] == '\n' || destination[i] == '\r') destination[i] = ' ';
    }
}

void writeRepairHealth(RepairTraceWriter& writer, bool final) {
    writer.health.seekp(0);
    writer.health << "{\"schema\":1,\"mode\":\"async_bounded\""
        << ",\"dropped_events_total\":" << writer.queue.dropped.load()
        << ",\"limited_events_total\":" << writer.limited.load()
        << ",\"duplicate_events_removed\":" << writer.duplicates.load()
        << ",\"truncated_events_total\":" << writer.truncated.load()
        << ",\"queued_total\":" << writer.queue.written.load()
        << ",\"written_total\":" << writer.queue.read.load()
        << ",\"final\":" << (final ? 1 : 0) << "}\n";
    writer.health.flush();
    writer.output.flush();
}

void runRepairWriter(RepairTraceWriter* writer) {
    try {
        auto flushed = std::chrono::steady_clock::now();
        while (true) {
            const auto* record = peekDiagnostic(writer->queue);
            if (record) {
                writer->output << std::fixed << std::setprecision(3)
                    << record->wallTime << " frame=" << record->frame << " ["
                    << record->category.data() << "] " << record->event.data();
                if (record->detail[0]) writer->output << " | " << record->detail.data();
                if (record->truncated) writer->output << " [detail_truncated]";
                writer->output << '\n';
                releaseDiagnostic(writer->queue);
            } else {
                if (writer->stopping.load(std::memory_order_acquire)) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - flushed >= std::chrono::milliseconds(100)) {
                writeRepairHealth(*writer, false);
                flushed = now;
            }
            if (!writer->output || !writer->health) throw std::runtime_error("repair trace write failed");
        }
        writeRepairHealth(*writer, true);
        if (!writer->output || !writer->health) throw std::runtime_error("repair trace final flush failed");
    } catch (...) {
        writer->failed.store(true, std::memory_order_release);
        std::cerr << "[diagnostics-error] asynchronous decision writer failed; evidence is incomplete\n";
    }
}

void stopRepairWriter(RepairTraceWriter& writer) {
    writer.stopping.store(true, std::memory_order_release);
    if (writer.thread.joinable()) writer.thread.join();
    writer.output.close();
    writer.health.close();
}

bool detailedRepairEvent(const std::string& category, const std::string& event) {
    if (event == "blocker_selected" || event == "selected_distance" ||
        event == "next_global_goal" || event == "required_forward_goal" ||
        event == "keep_route_connection_result" || event == "keep_route_build_failed") return false;
    return category == "obstacle_scan" || category == "target_lane" ||
           category == "candidate_obb" || category == "lane_topology";
}

}  // namespace

RepairTraceWriter::~RepairTraceWriter() { stopRepairWriter(*this); }

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
    data.repairDetailCount = 0;
    data.repairEventCount = 0;
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
    traceRepairDecision(data, category, event, detail);
    if (data->repairOnly) return;
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

void initializeRepairTrace(DecisionTraceData& data,
                           const std::filesystem::path& outputPath) {
    if (data.repairWriter) throw std::runtime_error("repair trace already initialized");
    if (outputPath.has_parent_path()) std::filesystem::create_directories(outputPath.parent_path());
    const auto healthPath = outputPath.parent_path() / "repair_health.json";
    if (std::filesystem::exists(outputPath) || std::filesystem::exists(healthPath)) {
        throw std::runtime_error("repair trace already exists: " + outputPath.string());
    }
    auto writer = std::make_shared<RepairTraceWriter>();
    writer->output.open(outputPath);
    writer->health.open(healthPath);
    if (!writer->output || !writer->health) throw std::runtime_error("cannot open repair trace: " + outputPath.string());
    writer->output << "# HL_VTD repair-only decision evidence v2, async/bounded\n";
    writer->output << "# per frame: up to 32 detail + 64 key events; see repair_health.json\n";
    writer->thread = std::thread(runRepairWriter, writer.get());
    data.repairOnly = !data.enabled;
    data.repairWriter = std::move(writer);
    data.enabled = true;
}

void traceRepairDecision(DecisionTraceData* data,
                         const std::string& category,
                         const std::string& event,
                         const std::string& detail) {
    if (data == nullptr || !data->repairWriter) return;
    auto& writer = *data->repairWriter;
    if (writer.failed.load(std::memory_order_acquire)) return;
    // The final, post-safety frame is already in diagnostics.jsonl.
    if (category == "planning" && event == "frame_result") {
        writer.duplicates.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const bool detailed = detailedRepairEvent(category, event);
    auto& count = detailed ? data->repairDetailCount : data->repairEventCount;
    const std::size_t limit = detailed ? 32 : 64;
    if (count >= limit) {
        writer.limited.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    ++count;
    auto* record = reserveDiagnostic(writer.queue);
    if (!record) return;
    record->frame = data->frameIndex;
    record->wallTime = data->elapsedWallTimeS;
    copyTraceText(record->category.data(), record->category.size(), category);
    copyTraceText(record->event.data(), record->event.size(), event);
    copyTraceText(record->detail.data(), record->detail.size(), detail);
    record->truncated = category.size() >= record->category.size() ||
        event.size() >= record->event.size() || detail.size() >= record->detail.size();
    if (record->truncated) writer.truncated.fetch_add(1, std::memory_order_relaxed);
    commitDiagnostic(writer.queue);
}

void finishRepairTrace(DecisionTraceData& data) {
    if (data.repairWriter) stopRepairWriter(*data.repairWriter);
    data.repairWriter.reset();
}

void traceCandidateLane(DecisionTraceData* data, const LaneKey& lane) {
    if (data == nullptr || !data->enabled || data->repairOnly) return;
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
    if (data == nullptr || !data->enabled || data->repairOnly || points.size() < 2) {
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
