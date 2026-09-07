#pragma once

#include "data/map/static_zones.hpp"

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct XmlElement {
    std::string name;
    std::unordered_map<std::string, std::string> attributes;
    std::vector<std::pair<std::string, XmlElement>> children;
};

struct Checkpoint { int sequence{}; double x{}; double y{}; };

struct Waypoint {
    double x{}; double y{}; double yaw{}; double s{}; double roadS{};
    int roadId{}; int laneId{}; int sectionIndex{};
};

struct LaneMatch {
    bool valid{}; int roadId{}; int laneId{}; int sectionIndex{};
    double roadS{}; double distance{}; double laneWidth{};
    std::size_t pointIndex{};
};

struct Polynomial { double s{}; double a{}; double b{}; double c{}; double d{}; };
enum class GeometryType { Line, Arc, Spiral, Poly3 };

struct Geometry {
    GeometryType type{GeometryType::Line};
    double s{}; double x{}; double y{}; double heading{}; double length{};
    double curvature{}; double curvatureStart{}; double curvatureEnd{};
    double a{}; double b{}; double c{}; double d{};
};

struct RoadPose { double x{}; double y{}; double yaw{}; };

struct RoadMarkEntry {
    double s{};
    std::string laneChange;
    std::string color;
    // 기존 테스트의 {s, laneChange, color} aggregate 초기화를
    // 깨지 않도록 마지막에 추가한다.
    std::string type;
};

struct LaneDefinition {
    int id{}; std::string type;
    std::optional<int> predecessor; std::optional<int> successor;
    std::vector<Polynomial> widths;
    std::vector<RoadMarkEntry> roadMarks;
};

struct LaneSection { double s{}; std::unordered_map<int, LaneDefinition> lanes; };
struct RoadLink { bool valid{}; std::string elementType; int elementId{}; std::string contactPoint; };

struct Road {
    int id{}; int junction{-1}; double length{};
    std::vector<Geometry> geometries;
    std::vector<Polynomial> laneOffsets;
    std::vector<LaneSection> sections;
    RoadLink predecessor; RoadLink successor;
};

// XODR <road><objects><object> 하나.
// 위치가 (roadId, s, t)인 도로 기준 좌표이므로 월드 x/y를 쓰려면
// roadToWorld()로 정변환해야 한다. 정지선/횡단보도/노면 제한속도 표시가 여기 있다.
struct RoadObject {
    int roadId{};
    int id{};
    std::string name;
    std::string type;
    std::string subtype;
    double s{};
    double t{};
    double hdg{};
    double length{};
    double width{};
};

// XODR <road><signals><signal> 하나. 이 id는 개별 램프 id이며,
// VTD가 보내는 trafficLight.id는 이 램프를 묶는 controller id다.
struct RoadSignal {
    int roadId{};
    int id{};
    std::string name;
    std::string type;
    std::string subtype;
    double s{};
    double t{};
    bool dynamic{};
    std::string orientation;
};

// OpenDRIVE controller 하나는 적/황/녹/화살표 등 여러 개별 signal을 묶는다.
// VTD GT의 trafficLight.id는 개별 signal id가 아니라 이 controller id다.
struct TrafficController {
    int id{-1};
    std::string name;
    std::vector<int> signalIds;
};

struct JunctionLaneLink {
    int incomingRoad{}; int connectingRoad{}; std::string contactPoint;
    int fromLane{}; int toLane{};
};

struct LaneKey { int roadId{}; int sectionIndex{}; int laneId{}; };
bool operator==(const LaneKey& left, const LaneKey& right);
bool operator<(const LaneKey& left, const LaneKey& right);

struct LaneSegment {
    LaneKey key; double startS{}; double endS{}; double length{};
    std::vector<Waypoint> points;
};

struct PathGeneratorData {
    double sampleInterval{0.5};
    std::vector<Checkpoint> checkpoints;
    std::unordered_map<int, Road> roads;
    std::vector<JunctionLaneLink> junctionLinks;
    std::map<LaneKey, LaneSegment> laneSegments;
    std::map<LaneKey, std::vector<LaneKey>> laneGraph;
    std::vector<RoadObject> roadObjects;
    std::vector<RoadSignal> roadSignals;
    std::unordered_map<int, TrafficController> trafficControllers;
    std::unordered_map<int, int> signalToController;
    StaticZoneData staticZones;
};
