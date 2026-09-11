#include "logger/field_diagnostics.hpp"
#include "logger/decision_trace.hpp"
#include "logger/diagnostic_queue.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <algorithm>
#include <numeric>

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

std::string readText(const std::filesystem::path& path) {
    std::ifstream stream(path);
    return std::string(std::istreambuf_iterator<char>(stream), {});
}

void testLegacyTraceSeparation(const std::filesystem::path& directory) {
    DecisionTraceData legacy;
    DecisionTraceData combined;
    initializeDecisionTrace(legacy, directory / "legacy.log");
    initializeDecisionTrace(combined, directory / "combined.log");
    initializeRepairTrace(combined, directory / "repair_decision.log");
    for (std::size_t frame = 0; frame < 3; ++frame) {
        beginDecisionTraceFrame(legacy, frame, frame * 0.05);
        beginDecisionTraceFrame(combined, frame, frame * 0.05);
        traceDecision(&legacy, "lane_change", "phase_transition", "prepare->execute");
        traceDecision(&combined, "lane_change", "phase_transition", "prepare->execute");
        traceRepairDecision(&combined, "lane_topology", "side_scanned", "side=left");
        require(legacy.frameEvents == combined.frameEvents, "repair trace changed legacy events");
    }
    legacy.output.close();
    combined.output.close();
    finishRepairTrace(combined);
    require(readText(directory / "legacy.log") == readText(directory / "combined.log"),
            "repair trace changed legacy file");
    const auto repair = readText(directory / "repair_decision.log");
    require(repair.find("phase_transition") != std::string::npos, "legacy event missing from repair");
    require(repair.find("side_scanned") != std::string::npos, "new event missing from repair");
    for (const auto* name : {"legacy.log", "combined.log", "repair_decision.log", "repair_health.json"}) {
        std::filesystem::remove(directory / name);
    }
}

void testFullQueueNeverWaits() {
    DiagnosticQueue<std::size_t, 4> queue;
    for (std::size_t i = 0; i < 4; ++i) {
        auto* slot = reserveDiagnostic(queue);
        require(slot != nullptr, "queue unexpectedly full");
        *slot = i;
        commitDiagnostic(queue);
    }
    // Consumer is deliberately not running: a blocking enqueue would hang.
    for (std::size_t i = 0; i < 10000; ++i) {
        require(reserveDiagnostic(queue) == nullptr, "full queue overwrote unread evidence");
    }
    require(queue.dropped.load() == 10000, "queue losses not counted");
    for (std::size_t i = 0; i < 4; ++i) {
        const auto* slot = peekDiagnostic(queue);
        require(slot && *slot == i, "queue order changed");
        releaseDiagnostic(queue);
    }
    require(peekDiagnostic(queue) == nullptr, "queue should be empty");
    *reserveDiagnostic(queue) = 9;
    commitDiagnostic(queue);
    require(*peekDiagnostic(queue) == 9, "queue did not recover after saturation");
}

void testRepairLimits(const std::filesystem::path& directory) {
    DecisionTraceData trace;
    initializeRepairTrace(trace, directory / "repair_decision.log");
    beginDecisionTraceFrame(trace, 17, 0.85);
    for (int i = 0; i < 100; ++i) traceDecision(&trace, "obstacle_scan", "candidate", "id=12");
    traceDecision(&trace, "lane_change", "phase_transition", "prepare->execute");
    traceDecision(&trace, "planning", "frame_result", "duplicate final state");
    traceDecision(&trace, "network", "error", std::string(2000, 'x'));
    require(trace.frameEvents.empty(), "repair-only trace collected legacy events");
    traceCandidatePath(&trace, {}, std::vector<Waypoint>(100), "selected");
    require(trace.candidatePaths.empty(), "repair-only trace copied visualization paths");
    finishRepairTrace(trace);
    const auto health = readText(directory / "repair_health.json");
    require(health.find("\"limited_events_total\":68") != std::string::npos, "detail limit missing");
    require(health.find("\"truncated_events_total\":1") != std::string::npos, "truncation missing");
    require(health.find("\"final\":1") != std::string::npos, "writer not drained");
    const auto events = readText(directory / "repair_decision.log");
    require(events.find("phase_transition") != std::string::npos, "detail flood hid key transition");
    require(events.find("frame_result") == std::string::npos, "duplicate final state persisted");
    std::filesystem::remove(directory / "repair_health.json");
    std::filesystem::remove(directory / "repair_decision.log");
}

int main(int argc, char** argv) {
    const auto directory = argc > 1 ? std::filesystem::path(argv[1]) :
        std::filesystem::temp_directory_path() / ("hl_field_test_" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    FieldDiagnosticData logger;
    initializeFieldDiagnostics(logger, directory);
    testFullQueueNeverWaits();
    testLegacyTraceSeparation(directory);
    testRepairLimits(directory);
    GtReceiverData receiver;
    WorldState world;
    world.ego.x = 10.0F;
    world.egoSpeed = 0.0;
    world.inputValid = true;
    world.egoLaneValid = true;
    world.egoRoadId = 72;
    world.egoLaneId = -1;
    world.objects.resize(30);
    for (std::size_t i = 0; i < 30; ++i) {
        world.objects[i].raw.id = static_cast<std::uint32_t>(i + 1);
        world.objects[i].ttc = std::numeric_limits<double>::infinity();
        receiver.latestGt.objects[i].id = static_cast<std::uint32_t>(i + 1);
    }
    PlanningProcessData planning;
    planning.result.valid = true;
    planning.result.targetSpeed = 3.0;
    planning.result.pathSafety.minimumTTC = std::numeric_limits<double>::infinity();
    planning.result.localPath.points.resize(200);
    ControlProcessData control;
    control.nominalCommand.targetAccel = 2.0F;
    control.safeCommand.targetAccel = 1.5F;
    PathGeneratorData map;
    const auto started = std::chrono::steady_clock::now();
    for (std::size_t frame = 0; frame < 20; ++frame) {
        writeFieldReceived(logger, receiver, frame);
        const FieldDiagnosticFrame input{frame, frame * 0.05, frame * 0.05,
            0.05, 45.0, 0.2, 1.0, 2.0, 0.1, false,
            receiver, world, planning, control, map, true, 0.1, 50.0, 3.0};
        writeFieldFrame(logger, input);
        world.ego.x += 1.0F; // Writer must see the copied value, not this mutation.
    }
    writeFieldStage(logger, "error", 20, "quoted \"value\"\nnext\\line");
    require(control.safeCommand.targetAccel == 1.5F, "logger changed command");
    require(planning.result.targetSpeed == 3.0, "logger changed planning");
    finishFieldDiagnostics(logger);
    std::ifstream stream(directory / "diagnostics.jsonl");
    std::size_t lines = 0;
    std::size_t frames = 0, sends = 0;
    std::string line;
    while (std::getline(stream, line)) {
        ++lines;
        require(line.find(":inf") == std::string::npos, "non-JSON infinity");
        if (line.find("\"kind\":\"frame\"") != std::string::npos) ++frames;
        if (line.find("\"kind\":\"send_result\"") != std::string::npos) ++sends;
    }
    require(frames == 20 && sends == 20 && lines >= 63, "record lost");
    const auto content = readText(directory / "diagnostics.jsonl");
    require(content.find("\"ego\":{\"x\":10,") != std::string::npos, "snapshot not immutable");
    require(content.find("\"point_count\":200,\"sample_stride\":13") != std::string::npos, "path not bounded");
    require(content.find("\"dropped_records_total\":0") != std::string::npos, "unexpected queue loss");
    require(content.find("\"final\":1") != std::string::npos, "writer not finalized");
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    std::cout << "Async evidence tests passed / 20 frames / 30 objects: " << ms << " ms; " << directory << '\n';
    if (argc == 1) {
        std::filesystem::remove(directory / "diagnostics.jsonl");
        std::filesystem::remove(directory);
    }
}
