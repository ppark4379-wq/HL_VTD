#pragma once

#include "control/control_structs.hpp"
#include "data/gt/gt_structs.hpp"
#include "planning/planning_structs.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

// Logger 시작 시 한 번 설정하는 출력 경로와 실행 식별자.
struct CsvLoggerConfig {
    std::filesystem::path outputPath;
    std::string scenarioId{"unspecified"};
    std::string runId{"unspecified"};
};

// CSV 출력 스트림과 실행 중 유지할 Logger 상태.
struct CsvLoggerData {
    std::filesystem::path outputPath;
    std::string scenarioId;
    std::string runId;
    std::ofstream output;
};

// Logger가 한 프레임을 처리할 때 필요한 입력 묶음.
struct LogFrameInput {
    std::size_t frameIndex;
    double elapsedWallTimeS;
    const GtFrame& gt;
    const WorldState& world;
    const PlanningResult& planning;
    const ControlProcessData& control;
};

// 현재 수신 데이터에서 계산할 수 있는 최근접 객체 정보.
// 객체 종류, 상대속도, TTC와 충돌 여부는 인터페이스 확정 후 별도로 추가한다.
struct NearestObjectLog {
    bool valid{};
    std::uint32_t id{};
    float x{};
    float y{};
    float heading{};
    float speed{};
    float length{};
    float width{};
    double distance{};
    double longitudinalDistance{};
    double lateralDistance{};
    double relativeSpeed{};
    bool ttcValid{};
    double ttc{};
};

// 현재 Logger 구현에서 사용하는 한 프레임의 내부 레코드.
// 대회 최종 로그 형식이 아니라, 확정된 현재 입력만 묶은 MVP 자료형이다.
struct LogRecord {
    std::size_t frameIndex{};
    double elapsedWallTimeS{};
    EgoState ego;
    double egoSpeed{};
    bool egoLaneValid{};
    int egoRoadId{};
    int egoLaneId{};
    double egoLaneWidth{};
    double egoLaneCenterDistance{};
    bool egoPathValid{};
    double egoPathS{};
    double egoPathD{};
    std::size_t objectCount{};
    NearestObjectLog nearestObject;
    TrafficLightState trafficLight;
    PathPlan plan;
    ControlCommand command;
    double currentSpeedLimit{};
    bool schoolZoneActive{};
    bool stopDistanceValid{};
    double stopDistance{};
    bool crosswalkActive{};
    bool laneChangeActive{};
    bool collision{};
};
