"""Project VTD world positions onto the official Ego route."""
from __future__ import annotations

import csv
import math
from pathlib import Path


def _angle_error(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


class RouteProgress:
    def __init__(self, path: Path, max_distance_m: float = 20.0):
        points = []
        with Path(path).open(newline="", encoding="utf-8-sig") as stream:
            for row in csv.DictReader(stream):
                point = {
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "s": float(row["s"]),
                    "road": int(row["road_id"]),
                    "lane": int(row["lane_id"]),
                }
                if not all(math.isfinite(point[key]) for key in ("x", "y", "s")):
                    raise ValueError("route points must be finite")
                if points and point["s"] <= points[-1]["s"]:
                    raise ValueError("route s must be strictly increasing")
                points.append(point)
        if len(points) < 2:
            raise ValueError("route requires at least two points")
        if not math.isfinite(max_distance_m) or max_distance_m <= 0:
            raise ValueError("route max_distance_m must be positive")
        self.points = points
        self.max_distance_m = max_distance_m
        self.min_s = points[0]["s"]
        self.max_s = points[-1]["s"]

    def project(self, ego: dict, road: dict | None = None, hint_s: float | None = None) -> dict | None:
        x, y, heading = ego["x"], ego["y"], ego.get("h", 0.0)
        best = None
        for index, (first, second) in enumerate(zip(self.points, self.points[1:])):
            dx, dy = second["x"] - first["x"], second["y"] - first["y"]
            length2 = dx * dx + dy * dy
            if length2 <= 1e-12:
                continue
            alpha = max(0.0, min(1.0, ((x - first["x"]) * dx + (y - first["y"]) * dy) / length2))
            px, py = first["x"] + alpha * dx, first["y"] + alpha * dy
            distance2 = (x - px) ** 2 + (y - py) ** 2
            route_s = first["s"] + alpha * (second["s"] - first["s"])
            segment_heading = math.atan2(dy, dx)
            heading_penalty = 4.0 * _angle_error(heading, segment_heading) ** 2
            road_penalty = 0.0
            if road and road.get("road") not in {first["road"], second["road"]}:
                road_penalty = 25.0
            hint_penalty = 0.0 if hint_s is None else min(abs(route_s - hint_s), 100.0) * 1e-4
            score = distance2 + heading_penalty + road_penalty + hint_penalty
            if best is None or score < best[0]:
                best = (score, distance2, route_s, index, px, py)
        if best is None or best[1] > self.max_distance_m ** 2:
            return None
        return {
            "s": best[2],
            "distance_m": math.sqrt(best[1]),
            "segment": best[3],
            "x": best[4],
            "y": best[5],
        }

    def point_at(self, route_s: float) -> tuple[float, float]:
        route_s = max(self.min_s, min(self.max_s, route_s))
        for first, second in zip(self.points, self.points[1:]):
            if route_s <= second["s"]:
                span = second["s"] - first["s"]
                alpha = (route_s - first["s"]) / span
                return (
                    first["x"] + alpha * (second["x"] - first["x"]),
                    first["y"] + alpha * (second["y"] - first["y"]),
                )
        return self.points[-1]["x"], self.points[-1]["y"]

    def checkpoint_at(self, route_s: float) -> dict:
        x, y = self.point_at(route_s)
        for first, second in zip(self.points, self.points[1:]):
            if route_s <= second['s']:
                endpoint = second if route_s == second['s'] else first
                return dict(x=x, y=y, s=route_s, road=endpoint['road'], lane=endpoint['lane'],
                            heading=math.atan2(second['y']-first['y'], second['x']-first['x']))
        raise ValueError('checkpoint outside route')

    def reaches_checkpoint(self, before: dict, ego: dict, checkpoint: dict,
                           tolerance: float, road: dict | None = None) -> bool:
        """Continuous forward entry into the endpoint circle, including swept entry.

        The caller must exclude resets, gaps and respawns before calling this.
        Starting/teleporting inside the circle is not proof of completion.
        """
        cx, cy = checkpoint['x'], checkpoint['y']
        if math.hypot(before['x']-cx, before['y']-cy) <= tolerance:
            return False
        if road and (road.get('road') != checkpoint['road'] or
                     road.get('lane', 0)*checkpoint['lane'] <= 0):
            return False
        dx, dy = ego['x']-before['x'], ego['y']-before['y']
        length2 = dx*dx + dy*dy
        if length2 <= 1e-12 or dx*math.cos(checkpoint['heading'])+dy*math.sin(checkpoint['heading']) <= 0:
            return False
        projection = self.project(ego, road, checkpoint['s'])
        if projection is None or abs(projection['s']-checkpoint['s']) > tolerance+math.sqrt(length2):
            return False
        alpha = max(0.0, min(1.0, ((cx-before['x'])*dx+(cy-before['y'])*dy)/length2))
        return math.hypot(before['x']+alpha*dx-cx, before['y']+alpha*dy-cy) <= tolerance
