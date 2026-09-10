#!/usr/bin/env python3
"""Read-only summary of an opt-in planner replay, including actual send timing.

Stdout is JSON. Collision episodes below are the controller's model OBBs,
not an independent simulator mesh/contact sensor. Handles a running log's
incomplete final row. No scenario IDs or tuning affect planning.
"""
import argparse
import csv
import gzip
import json
import math
from pathlib import Path


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (position - low)


def stats(values):
    return {"count": len(values), "median": percentile(values, .5),
            "p95": percentile(values, .95), "p99": percentile(values, .99),
            "max": max(values) if values else None}


def intersects(left, right):
    for poly in (left, right):
        for a, b in zip(poly, poly[1:] + poly[:1]):
            nx, ny = -(b[1] - a[1]), b[0] - a[0]
            lp = [p[0] * nx + p[1] * ny for p in left]
            rp = [p[0] * nx + p[1] * ny for p in right]
            if max(lp) < min(rp) or max(rp) < min(lp):
                return False
    return True


def read_rows(path):
    if not path.exists():
        return []
    with path.open() as stream:
        return [row for row in csv.DictReader(stream)
                if None not in row and all(value is not None for value in row.values())]


def analyze(directory):
    timing = read_rows(directory / "timing.csv")
    sends = [float(row["send_wall_s"]) for row in timing if row["send_wall_s"]]
    send_gaps = [b - a for a, b in zip(sends, sends[1:])]
    result = {"directory": str(directory), "timing": {
        "send_hz": (len(sends) - 1) / (sends[-1] - sends[0]) if len(sends) > 1 else None,
        "send_gap_ms": stats([value * 1000 for value in send_gaps]),
        "send_gaps_over_100ms": sum(value > .1 for value in send_gaps),
        "send_gaps_over_250ms": sum(value > .25 for value in send_gaps),
        "dropped_gt_frames": int(timing[-1]["dropped_gt_frames_total"]) if timing else None,
    }}
    for key in ("world_ms", "planning_ms", "control_ms", "visualization_ms", "loop_total_ms"):
        result["timing"][key] = stats([float(row[key]) for row in timing])
    rows = read_rows(directory / "run.csv")
    result["frames"] = len(rows)
    if rows:
        result["duration_s"] = float(rows[-1]["elapsed_wall_time_s"])
        result["goal_reached"] = any(row["goal_reached"] == "1" for row in rows)
        result["max_received_objects"] = max(int(row["object_count"]) for row in rows)
        result["logged_collision_frames"] = sum(row["collision"] == "1" for row in rows)
    episodes, active, first_seen = [], {}, {}
    history = directory / "rviz" / "frames.jsonl"
    compressed_history = directory / "rviz" / "frames.jsonl.gz"
    last_frame = None
    if history.exists() or compressed_history.exists():
        stream_context = (
            history.open()
            if history.exists()
            else gzip.open(compressed_history, "rt", encoding="utf-8")
        )
        with stream_context as stream:
            for line in stream:
                try:
                    frame = json.loads(line)
                except json.JSONDecodeError:
                    continue
                last_frame = frame
                hits = set()
                for obj in frame["objects"]:
                    identity = obj["id"]
                    first_seen.setdefault(identity, {
                        "frame": frame["frame"], "time_s": frame["time_s"],
                        "ego_road": frame["ego"]["road_id"],
                        "distance_m": math.hypot(obj["x"]-frame["ego"]["x"], obj["y"]-frame["ego"]["y"]),
                        "size": [obj["length"], obj["width"], obj["height"]],
                    })
                    if not intersects(frame["ego"]["obb"], obj["obb"]):
                        continue
                    hits.add(identity)
                    if identity not in active:
                        active[identity] = {
                            "object_id": identity, "start_frame": frame["frame"],
                            "start_time_s": frame["time_s"], "ego_road": frame["ego"]["road_id"],
                            "ego_lane": frame["ego"]["lane_id"], "ego_speed_kph": frame["ego"]["speed"]*3.6,
                            "object_speed_mps": obj["speed"], "first_seen": first_seen[identity],
                            "global_route": {k: v for k, v in frame.get("global_route", {}).items() if k != "points"},
                            "decision": frame["decision"],
                        }
                    active[identity]["end_frame"] = frame["frame"]
                    active[identity]["end_time_s"] = frame["time_s"]
                for identity in set(active) - hits:
                    episodes.append(active.pop(identity))
        episodes.extend(active.values())
    result["obb_contact_episodes"] = sorted(episodes, key=lambda event: event["start_frame"])
    result["first_seen_objects"] = first_seen
    if last_frame:
        result["history_last_frame"] = last_frame["frame"]
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.directory), ensure_ascii=False, indent=2))
