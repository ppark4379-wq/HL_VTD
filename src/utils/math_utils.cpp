#include "math_utils.hpp"

constexpr double kPi = 3.14159265358979323846;

// 각도를 -pi~pi 범위로 정규화한다.
double normalizeAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

// 두 평면 좌표 사이 거리의 제곱을 반환한다.
double distanceSquared(double x1, double y1, double x2, double y2) {
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return dx * dx + dy * dy;
}
