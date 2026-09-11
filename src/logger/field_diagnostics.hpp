#pragma once

#include "control/control_structs.hpp"
#include "network/network_structs.hpp"

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>

// Data-only contracts. Instrumentation never chooses or modifies commands.
struct FieldDiagnosticWriter;
struct FieldDiagnosticData {
    std::shared_ptr<FieldDiagnosticWriter> writer;
    bool enabled{};
};

struct FieldDiagnosticFrame {
    std::size_t frame;
    double wallTime;
    std::optional<double> simulationTime;
    double dt;
    double receiveWaitMs;
    double clockMs;
    double worldMs;
    double planningMs;
    double controlMs;
    bool receiveOnly;
    const GtReceiverData& receiver;
    const WorldState& world;
    const PlanningProcessData& planning;
    const ControlProcessData& control;
    const PathGeneratorData& map;
    bool sendCompleted{};
    double sendMs{};
    double sendIntervalMs{}; // 0 for the first send; not a measured Hz.
    double receiveToSendMs{};
};

void initializeFieldDiagnostics(FieldDiagnosticData& data,
                                const std::filesystem::path& directory);
void writeFieldStage(FieldDiagnosticData& data, const char* stage,
                     std::size_t frame = 0, const std::string& detail = {});
void writeFieldFrame(FieldDiagnosticData& data, const FieldDiagnosticFrame& input);
void writeFieldReceived(FieldDiagnosticData& data, const GtReceiverData& receiver,
                         std::size_t frame);
void finishFieldDiagnostics(FieldDiagnosticData& data);
