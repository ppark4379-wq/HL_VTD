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
    std::optional<double> expectedEgoRouteS = std::nullopt);
