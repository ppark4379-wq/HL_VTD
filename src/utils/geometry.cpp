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

double cross(const Point2D& first, const Point2D& second,
             const Point2D& third) {
    return (second.x - first.x) * (third.y - first.y) -
           (second.y - first.y) * (third.x - first.x);
}

bool pointOnSegment(const Point2D& point, const Point2D& begin,
                    const Point2D& end) {
    constexpr double kEpsilon = 1e-9;
    return std::abs(cross(begin, end, point)) <= kEpsilon &&
           point.x >= std::min(begin.x, end.x) - kEpsilon &&
           point.x <= std::max(begin.x, end.x) + kEpsilon &&
           point.y >= std::min(begin.y, end.y) - kEpsilon &&
           point.y <= std::max(begin.y, end.y) + kEpsilon;
}

bool segmentsIntersect(const Point2D& firstBegin, const Point2D& firstEnd,
                       const Point2D& secondBegin,
                       const Point2D& secondEnd) {
    constexpr double kEpsilon = 1e-9;
    const double firstSideBegin = cross(firstBegin, firstEnd, secondBegin);
    const double firstSideEnd = cross(firstBegin, firstEnd, secondEnd);
    const double secondSideBegin = cross(secondBegin, secondEnd, firstBegin);
    const double secondSideEnd = cross(secondBegin, secondEnd, firstEnd);
    if (((firstSideBegin > kEpsilon && firstSideEnd < -kEpsilon) ||
         (firstSideBegin < -kEpsilon && firstSideEnd > kEpsilon)) &&
        ((secondSideBegin > kEpsilon && secondSideEnd < -kEpsilon) ||
         (secondSideBegin < -kEpsilon && secondSideEnd > kEpsilon))) {
        return true;
    }
    return (std::abs(firstSideBegin) <= kEpsilon &&
            pointOnSegment(secondBegin, firstBegin, firstEnd)) ||
           (std::abs(firstSideEnd) <= kEpsilon &&
            pointOnSegment(secondEnd, firstBegin, firstEnd)) ||
           (std::abs(secondSideBegin) <= kEpsilon &&
            pointOnSegment(firstBegin, secondBegin, secondEnd)) ||
           (std::abs(secondSideEnd) <= kEpsilon &&
            pointOnSegment(firstEnd, secondBegin, secondEnd));
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

VehicleFootprint2D vehicleFootprintFromObb(const Obb2D& body,
                                         const Pose2D& reference) {
    VehicleFootprint2D footprint;
    footprint.width = std::hypot(body.corners[0].x - body.corners[1].x,
                                 body.corners[0].y - body.corners[1].y);
    footprint.length = std::hypot(body.corners[1].x - body.corners[2].x,
                                  body.corners[1].y - body.corners[2].y);
    Point2D center;
    for (const auto& corner : body.corners) {
        center.x += 0.25 * corner.x;
        center.y += 0.25 * corner.y;
    }
    footprint.centerOffset =
        (center.x - reference.x) * std::cos(reference.heading) +
        (center.y - reference.y) * std::sin(reference.heading);
    return footprint;
}

Obb2D vehicleFootprintAt(const VehicleFootprint2D& footprint,
                        const Pose2D& reference, double margin) {
    return makeObb2D(
        reference.x + footprint.centerOffset * std::cos(reference.heading),
        reference.y + footprint.centerOffset * std::sin(reference.heading),
        reference.heading, footprint.length + 2.0 * margin,
        footprint.width + 2.0 * margin);
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

bool intersects(const std::vector<Point2D>& polygon, const Obb2D& obb) {
    if (polygon.size() < 3) return false;
    if (std::any_of(obb.corners.begin(), obb.corners.end(),
                    [&](const Point2D& corner) {
                        return containsPoint(polygon, corner);
                    })) {
        return true;
    }
    const std::vector<Point2D> obbPolygon(obb.corners.begin(),
                                          obb.corners.end());
    if (std::any_of(polygon.begin(), polygon.end(),
                    [&](const Point2D& point) {
                        return containsPoint(obbPolygon, point);
                    })) {
        return true;
    }
    for (std::size_t polygonIndex = 0;
         polygonIndex < polygon.size(); ++polygonIndex) {
        const Point2D& polygonBegin = polygon[polygonIndex];
        const Point2D& polygonEnd =
            polygon[(polygonIndex + 1) % polygon.size()];
        for (std::size_t obbIndex = 0; obbIndex < obb.corners.size();
             ++obbIndex) {
            const Point2D& obbBegin = obb.corners[obbIndex];
            const Point2D& obbEnd =
                obb.corners[(obbIndex + 1) % obb.corners.size()];
            if (segmentsIntersect(polygonBegin, polygonEnd,
                                  obbBegin, obbEnd)) {
                return true;
            }
        }
    }
    return false;
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
