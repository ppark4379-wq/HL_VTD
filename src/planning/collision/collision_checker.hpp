#pragma once

#include "data/gt/gt_structs.hpp"
#include "planning/planning_structs.hpp"

#include <vector>

// 종방향 간격과 접근속도로 TTC를 계산한다. 멀어지는 중이면 무한대.
double calculateTTC(double longitudinalGap, double closingSpeed);

// 전역경로 누적거리 targetS에 해당하는 경로 위 자세를 보간한다.
// 경로 범위를 벗어나면 false.
bool routeToWorld(Pose2D& output,
                  const std::vector<Waypoint>& points,
                  double targetS);

// 매 프레임 검사 시작 시 결과를 초기화한다.
// Write: output 전체
void resetPathSafety(PathSafetyResult& output);

// 현재 Ego OBB와 모든 객체 OBB의 교차 여부를 검사한다.
// Read : egoObb, objects
// Write: output.collision, output.riskObjectId, output.minimumTTC
void checkCurrentCollisions(PathSafetyResult& output,
                            const Obb2D& egoObb,
                            const std::vector<WorldObjectState>& objects);

// 현재 차선 전방 객체들의 TTC 중 최솟값을 기록한다.
// Read : objects
// Write: output.minimumTTC, output.riskObjectId
void updateMinimumTTC(PathSafetyResult& output,
                      const std::vector<WorldObjectState>& objects,
                      bool ignoreStaticObjects = false,
                      int ignoredObjectId = -1);

// Ego가 지역경로를 따라 주행한다고 가정하고 미래 OBB 충돌을 검사한다.
// 직선 등속 예측을 쓰면 곡선에서 반대 차선 차량과 가짜 충돌이 발생하므로
// Ego 미래 자세는 반드시 경로 위에서 계산한다.
// Read : world, localPath
// Write: output.collision, output.minimumTTC, output.riskObjectId
void checkPredictedCollisions(PathSafetyResult& output,
                              const WorldState& world,
                              const LocalPathData& localPath,
                              double horizon,
                              double step);

// 현재 충돌이거나 충돌/TTC가 비상제동 임계 이내일 때만 true.
// 차선 복귀 후보가 멀리서 막힌 사실은 복귀 속도를 낮추거나
// 대기 지점으로 감속할 이유이지, 즉시 비상정지 이유는 아니다.
bool requiresEmergencyBrake(const PathSafetyResult& safety,
                            double emergencyTTC);
