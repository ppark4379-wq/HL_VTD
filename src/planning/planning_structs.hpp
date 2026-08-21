#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using std::pair;
using std::string;
using std::unordered_map;
using std::vector;

// XODR의 XML 태그 하나와 속성/하위 태그.
struct XmlElement {
    string name;
    unordered_map<string, string> attributes;
    vector<pair<string, XmlElement>> children;
};

using std::size_t;
using std::map;
using std::optional;
using std::string;
using std::unordered_map;
using std::vector;

// 대회 CSV에서 읽은 필수 통과 지점.
struct Checkpoint {
    int sequence{};
    double x{};
    double y{};
};

// 전역 reference path를 구성하는 한 점.
struct Waypoint {
    double x{};
    double y{};
    double yaw{};
    double s{};       // 전역경로 시작부터의 누적거리
    double roadS{};   // 현재 OpenDRIVE road 시작부터의 거리
    int roadId{};
    int laneId{};
    int sectionIndex{};
};

// 월드 좌표를 XODR 차선에 매칭한 결과.
struct LaneMatch {
    bool valid{};
    int roadId{};
    int laneId{};
    int sectionIndex{};
    double roadS{};
    double distance{};
    double laneWidth{};
    size_t pointIndex{};
};

// 팀원 GT 구조체가 확정되기 전 테스트용 Ego 입력.
struct EgoPose {
    double x{};
    double y{};
    double heading{};
};

// 아래 자료형은 XODR 파싱과 lane graph 생성에만 사용하는 내부 데이터다.
struct Polynomial {
    double s{};
    double a{};
    double b{};
    double c{};
    double d{};
};

enum class GeometryType { Line, Arc, Spiral, Poly3 };

struct Geometry {
    GeometryType type{GeometryType::Line};
    double s{};
    double x{};
    double y{};
    double heading{};
    double length{};
    double curvature{};
    double curvatureStart{};
    double curvatureEnd{};
    double a{};
    double b{};
    double c{};
    double d{};
};

// XODR road 기준선의 특정 지점에 대한 VTD 월드 좌표와 진행 방향.
// geometry 시작점 기준의 로컬 좌표가 아니라 이미 계산이 끝난 월드 좌표다.
// 차선 중심점을 만들 때 이 위치에서 횡방향 lane offset을 적용하기 위해 사용한다.
struct RoadPose {
    double x{};
    double y{};
    double yaw{};
};

// roadMark은 lane의 "중심에서 먼 쪽" 경계선을 기술한다 (ASAM OpenDRIVE 표준 규약).
// s는 laneSection.s를 더한 절대 road s 기준.
struct RoadMarkEntry {
    double s{};
    string laneChange;  // "none" / "increase" / "decrease" / "both"
};

struct LaneDefinition {
    int id{};
    string type;
    optional<int> predecessor;
    optional<int> successor;
    vector<Polynomial> widths;
    vector<RoadMarkEntry> roadMarks;
};

struct LaneSection {
    double s{};
    unordered_map<int, LaneDefinition> lanes;
};

struct RoadLink {
    bool valid{};
    string elementType;
    int elementId{};
    string contactPoint;
};

struct Road {
    int id{};
    int junction{-1};
    double length{};
    vector<Geometry> geometries;
    vector<Polynomial> laneOffsets;
    vector<LaneSection> sections;
    RoadLink predecessor;
    RoadLink successor;
};

struct JunctionLaneLink {
    int incomingRoad{};
    int connectingRoad{};
    string contactPoint;
    int fromLane{};
    int toLane{};
};

struct LaneKey {
    int roadId{};
    int sectionIndex{};
    int laneId{};

};

// LaneKey를 비교하는 일반 함수. 구조체 자체는 데이터만 저장한다.
bool operator==(const LaneKey& left, const LaneKey& right);
bool operator<(const LaneKey& left, const LaneKey& right);

struct LaneSegment {
    LaneKey key;
    double startS{};
    double endS{};
    double length{};
    vector<Waypoint> points;
};

// 경로 생성에 필요한 데이터만 저장한다. 함수는 이 구조체 밖에 정의한다.
struct PathGeneratorData {
    double sampleInterval{0.5};
    vector<Checkpoint> checkpoints;
    unordered_map<int, Road> roads;
    vector<JunctionLaneLink> junctionLinks;
    map<LaneKey, LaneSegment> laneSegments;
    map<LaneKey, vector<LaneKey>> laneGraph;
};
