#pragma once

#include "planning/planning_structs.hpp"

struct WorldState;

// 활성 정지 요청 중 가장 앞선(가장 제약이 강한) 정지점을 선택한다.
// Ego가 이미 지나친 정지점도 버리지 않는다. 정지 요청의 해제는 위치가 아니라
// 원인(신호 녹색 전환, 보행자 이탈 등)이 사라질 때만 이루어져야 한다.
// Read : manager.requests
// Write: output
void selectStopDecision(StopDecision& output,
                        const StopManagerData& manager,
                        double egoRouteS);

// 제약과 정지 결정을 종방향 행동 및 목표속도로 변환한다.
// Read : state, constraints, stop, world
// Write: state.longitudinal, state.health, state.recoveryHoldTime, output
void updateBehavior(BehaviorState& state,
                    BehaviorDecision& output,
                    const ConstraintSet& constraints,
                    const StopDecision& stop,
                    const WorldState& world,
                    double cruiseSpeed,
                    const PlanningBehaviorConfig& config,
                    double dt);
