#include "utils/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {

Point2D subtract(const Point2D& left, const Point2D& right) {
    return {left.x - right.x, left.y - right.y};
}

double dot(const Point2D& left, const Point2D& right) {
    return left.x * right.x + left.y * right.y;
}

bool overlapsOnAxis(const Obb2D& first, const Obb2D& second,
                    const Point2D& axis) {
    double firstMin = std::numeric_limits<double>::infinity();
    double firstMax = -std::numeric_limits<double>::infinity();
    double secondMin = std::numeric_limits<double>::infinity();
    double secondMax = -std::numeric_limits<double>::infinity();
    for (const auto& corner : first.corners) {
        const double value = dot(corner, axis);
        firstMin = std::min(firstMin, value);
        firstMax = std::max(firstMax, value);
    }
    for (const auto& corner : second.corners) {
        const double value = dot(corner, axis);
        secondMin = std::min(secondMin, value);
        secondMax = std::max(secondMax, value);
    }
    return firstMax >= secondMin && secondMax >= firstMin;
}

}  // namespace

Obb2D makeObb2D(double centerX, double centerY, double heading,
                double length, double width) {
    const double halfLength = 0.5 * length;
    const double halfWidth = 0.5 * width;
    const std::array<Point2D, 4> local{{
        {halfLength, halfWidth},
        {halfLength, -halfWidth},
        {-halfLength, -halfWidth},
        {-halfLength, halfWidth},
    }};
    const double cosHeading = std::cos(heading);
    const double sinHeading = std::sin(heading);
    Obb2D result;
    for (std::size_t index = 0; index < local.size(); ++index) {
        result.corners[index].x = centerX +
            local[index].x * cosHeading - local[index].y * sinHeading;
        result.corners[index].y = centerY +
            local[index].x * sinHeading + local[index].y * cosHeading;
    }
    return result;
}

bool intersects(const Obb2D& first, const Obb2D& second) {
    const Point2D firstEdge0 = subtract(first.corners[1], first.corners[0]);
    const Point2D firstEdge1 = subtract(first.corners[3], first.corners[0]);
    const Point2D secondEdge0 = subtract(second.corners[1], second.corners[0]);
    const Point2D secondEdge1 = subtract(second.corners[3], second.corners[0]);
    const std::array<Point2D, 4> axes{{
        {-firstEdge0.y, firstEdge0.x},
        {-firstEdge1.y, firstEdge1.x},
        {-secondEdge0.y, secondEdge0.x},
        {-secondEdge1.y, secondEdge1.x},
    }};
    for (const auto& axis : axes) {
        if (!overlapsOnAxis(first, second, axis)) return false;
    }
    return true;
}

bool containsPoint(const std::vector<Point2D>& polygon,
                   const Point2D& point) {
    if (polygon.size() < 3) return false;
    bool inside = false;
    for (std::size_t current = 0, previous = polygon.size() - 1;
         current < polygon.size(); previous = current++) {
        const Point2D& a = polygon[current];
        const Point2D& b = polygon[previous];
        if ((a.y > point.y) == (b.y > point.y)) continue;
        const double crossX = (b.x - a.x) * (point.y - a.y) /
                                  (b.y - a.y) +
                              a.x;
        if (point.x <= crossX) inside = !inside;
    }
    return inside;
}
