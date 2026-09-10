"""Small finite-polygon operations for OpenDRIVE assessment measurements.

No route, road-network, controller, or scenario generation takes place here.
"""
from __future__ import annotations

import math


def polygon_area(points):
    return sum(a[0] * b[1] - a[1] * b[0]
               for a, b in zip(points, points[1:] + points[:1])) / 2


def clip_convex(subject, boundary):
    """Intersect a convex footprint and finite convex boundary polygon."""
    if len(subject) < 3 or len(boundary) < 3:
        return []
    area = polygon_area(boundary)
    if abs(area) < 1e-12:
        return []
    sign = 1 if area > 0 else -1
    result = list(subject)
    for a, b in zip(boundary, boundary[1:] + boundary[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        def side(point):
            return sign * (dx * (point[1] - a[1]) - dy * (point[0] - a[0]))
        old, result = result, []
        if not old:
            break
        previous, previous_side = old[-1], side(old[-1])
        for current in old:
            current_side = side(current)
            previous_inside, current_inside = previous_side >= -1e-10, current_side >= -1e-10
            if previous_inside != current_inside:
                fraction = previous_side / (previous_side - current_side)
                result.append((previous[0] + fraction * (current[0] - previous[0]),
                               previous[1] + fraction * (current[1] - previous[1])))
            if current_inside:
                result.append(current)
            previous, previous_side = current, current_side
    return result


def finite_strip_intrusion(footprint, inner_start, inner_end, outer_start, outer_end):
    """Maximum penetration into a finite sidewalk strip, never past its far edge.

    Two triangles also handle slightly non-convex sampled offset quadrilaterals.
    The return value is zero when the footprint does not overlap the strip.
    """
    dx, dy = inner_end[0] - inner_start[0], inner_end[1] - inner_start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-12:
        return 0.0
    nx, ny = -dy / length, dx / length
    outer_x = (outer_start[0] + outer_end[0]) / 2
    outer_y = (outer_start[1] + outer_end[1]) / 2
    if (outer_x - inner_start[0]) * nx + (outer_y - inner_start[1]) * ny < 0:
        nx, ny = -nx, -ny
    depth = 0.0
    for triangle in ((inner_start, outer_start, outer_end),
                     (inner_start, outer_end, inner_end)):
        for point in clip_convex(footprint, list(triangle)):
            depth = max(depth, (point[0] - inner_start[0]) * nx
                        + (point[1] - inner_start[1]) * ny)
    return depth


def subtract_convex(subject, boundary):
    """Disjoint convex pieces of ``subject`` outside ``boundary``.

    Used to verify complete finite-patch coverage; sampling just footprint
    edges would miss small gaps between two allegedly connected map roads.
    """
    if len(subject) < 3 or abs(polygon_area(subject)) < 1e-12:
        return []
    if len(boundary) < 3 or abs(polygon_area(boundary)) < 1e-12:
        return [list(subject)]
    sign = 1 if polygon_area(boundary) > 0 else -1
    remaining, outside = list(subject), []
    for a, b in zip(boundary, boundary[1:] + boundary[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        def split(points, keep_inside):
            result = []
            if not points:
                return result
            previous = points[-1]
            previous_side = sign * (dx * (previous[1] - a[1]) - dy * (previous[0] - a[0]))
            for current in points:
                side = sign * (dx * (current[1] - a[1]) - dy * (current[0] - a[0]))
                previous_ok = previous_side >= 0 if keep_inside else previous_side <= 0
                current_ok = side >= 0 if keep_inside else side <= 0
                if previous_ok != current_ok:
                    fraction = previous_side / (previous_side - side)
                    result.append(tuple(p + fraction * (q - p) for p, q in zip(previous, current)))
                if current_ok:
                    result.append(current)
                previous, previous_side = current, side
            return result
        piece = split(remaining, False)
        if len(piece) >= 3 and abs(polygon_area(piece)) > 1e-12:
            outside.append(piece)
        remaining = split(remaining, True)
        if len(remaining) < 3:
            break
    return outside


def finite_patches_cover(footprint, patches, tolerance_m2=1e-6):
    """Whether the union of finite convex patches covers a footprint."""
    uncovered = [list(footprint)]
    for patch in patches:
        uncovered = [piece for polygon in uncovered for piece in subtract_convex(polygon, patch)]
        if not uncovered:
            return True
    return sum(abs(polygon_area(polygon)) for polygon in uncovered) <= tolerance_m2
