#pragma once

#include "data/gt/gt_structs.hpp"
#include "planning/global/reference_path_generator.hpp"

#include <optional>

// BEV 기준 2D OBB의 네 꼭짓점을 계산
Obb2D makeObjectObb(const ObjectState& object);

// Raw Object GT에 OBB + Road/Lane 정보 붙이기
WorldObjectState makeWorldObjectState(
    const ObjectState& object,
    const PathGeneratorData& mapData,
    const std::vector<Waypoint>& globalPath);

// GT 한 프레임을 지도 매칭된 WorldState로 변환
WorldState makeWorldState(
    const GtFrame& gt,
    const PathGeneratorData& mapData,
    const std::vector<Waypoint>& globalPath,
    std::optional<double> expectedEgoRouteS = std::nullopt,
    bool preferGlobalRouteLane = true);

// 차량과 객체의 종/횡방향 관계를 이번 프레임에 실제로 추종할 경로로
// 다시 계산한다. checkpoint mission에서는 guide route가 아니라 이 결과가
// 장애물 탐색, TTC, 충돌예측과 lane-change 완료 판정의 기준이다.
bool projectWorldToActivePath(WorldState& world,
                              const std::vector<Waypoint>& activePath,
                              double frontBumperOffset,
                              double lateralMargin = 0.5);
