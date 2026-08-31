#pragma once

#include "data/gt/gt_structs.hpp"
#include "planning/planning_structs.hpp"

#include <cstddef>
#include <string>
#include <vector>

// reference_path.csv의 x/y/yaw/s/road/lane 값을 Waypoint 구조체로 읽는다.
bool loadWaypointsCsv(std::vector<Waypoint>& waypoints,
                      const std::string& csvPath);

// Ego와 가장 가까운 전역경로 점의 index를 찾는다.
std::size_t findNearestWaypoint(const std::vector<Waypoint>& waypoints,
                                const GtFrame& gt,
                                std::size_t previousIndex);

// 최근접점에서 경로 누적거리로 lookahead 이상 앞선 점을 찾는다.
std::size_t findTargetWaypoint(const std::vector<Waypoint>& waypoints,
                               std::size_t nearestIndex,
                               double lookahead);

// 앞쪽 경로 구간의 최대 곡률을 계산한다.
double findMaxCurvature(const std::vector<Waypoint>& waypoints,
                        std::size_t nearestIndex,
                        double checkDistance);

// 직선/곡선 여부에 따라 lookahead distance를 선택한다.
double chooseLookahead(double maxCurvature,
                       const PathPlannerConfig& config,
                       double egoSpeed);

// Ego heading과 lookahead target 방향 사이의 각도 오차를 계산한다.
double computeHeadingError(const GtFrame& gt, const Waypoint& target);

// Pure Pursuit 공식으로 조향각을 계산한다.
double computePurePursuitSteering(double headingError,
                                  double targetDistance,
                                  double wheelbase);

// 경로 곡률에 따라 직선/곡선 목표속도를 선택한다.
double chooseTargetSpeed(double maxCurvature,
                         const PathPlannerConfig& config);

// 위 단계를 순서대로 실행해 한 프레임의 경로추종 결과를 만든다.
PathPlan runPurePursuit(PathPlannerData& planner,
                        const GtFrame& gt,
                        double egoSpeed);
