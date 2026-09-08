#!/usr/bin/env python3
"""Publish HL_VTD's ROS-independent visualization files as RViz markers."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import Point, TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.utilities import remove_ros_args
    from rclpy.executors import ExternalShutdownException
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError as error:  # Give a useful error on a non-ROS development PC.
    raise SystemExit(
        "ROS2 Python packages are unavailable. Source /opt/ros/<distro>/setup.bash "
        "and run this script again.\n"
        f"Import error: {error}"
    )


def parse_args(arguments: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-dir",
        type=Path,
        required=True,
        help="Run-specific directory containing static_map.csv and latest.json",
    )
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--poll-hz", type=float, default=20.0)
    return parser.parse_args(arguments)


def ros_point(values: list[float] | tuple[float, ...]) -> Point:
    point = Point()
    point.x = float(values[0])
    point.y = float(values[1])
    point.z = float(values[2]) if len(values) > 2 else 0.0
    return point


def rgba(marker: Marker, red: float, green: float, blue: float, alpha: float) -> None:
    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = alpha


class HLVtdRvizBridge(Node):
    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__("hl_vtd_rviz_bridge")
        self.data_dir = options.data_dir.resolve()
        self.frame_id = options.frame_id
        self.static_path = self.data_dir / "static_map.csv"
        self.latest_path = self.data_dir / "latest.json"
        self.static_mtime_ns = -1
        self.latest_mtime_ns = -1
        self.marker_id = 0
        self.lane_points: dict[tuple[int, int, int], list[list[float]]] = {}
        self.signal_positions: dict[int, list[list[float]]] = {}

        static_qos = QoSProfile(depth=1)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        dynamic_qos = QoSProfile(depth=2)
        dynamic_qos.reliability = ReliabilityPolicy.RELIABLE
        self.static_publisher = self.create_publisher(
            MarkerArray, "/hl_vtd/static_map", static_qos
        )
        self.dynamic_publisher = self.create_publisher(
            MarkerArray, "/hl_vtd/planning", dynamic_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / max(1.0, options.poll_hz), self.poll)
        self.get_logger().info(f"Watching RViz data: {self.data_dir}")

    def base_marker(self, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def line_marker(
        self,
        namespace: str,
        points: list[list[float]],
        width: float,
        color: tuple[float, float, float, float],
        marker_type: int = Marker.LINE_STRIP,
    ) -> Marker:
        marker = self.base_marker(namespace, marker_type)
        marker.scale.x = width
        rgba(marker, *color)
        marker.points = [ros_point(point) for point in points]
        return marker

    def text_marker(
        self,
        namespace: str,
        position: list[float],
        text: str,
        size: float = 0.8,
        color: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0),
    ) -> Marker:
        marker = self.base_marker(namespace, Marker.TEXT_VIEW_FACING)
        marker.pose.position = ros_point(position)
        marker.scale.z = size
        marker.text = text
        rgba(marker, *color)
        return marker

    @staticmethod
    def dashed_points(points: list[list[float]]) -> list[list[float]]:
        # Map samples are normally 0.5 m apart: three metres on, three off.
        result: list[list[float]] = []
        for index in range(len(points) - 1):
            if (index // 6) % 2 == 0:
                result.extend((points[index], points[index + 1]))
        return result

    def load_static_map(self) -> None:
        try:
            stat = self.static_path.stat()
        except OSError:
            return
        if stat.st_mtime_ns == self.static_mtime_ns:
            return

        groups: dict[tuple[str, str, str], list[dict[str, str]]] = {}
        with self.static_path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                key = (row["layer"], row["group"], row["style"])
                groups.setdefault(key, []).append(row)

        self.marker_id = 0
        self.lane_points.clear()
        self.signal_positions.clear()
        markers: list[Marker] = []
        corridor_endpoints: list[list[float]] = []
        for (layer, group, style), rows in groups.items():
            rows.sort(key=lambda row: int(row["point_index"]))
            points = [
                [float(row["x"]), float(row["y"]), float(row["z"])]
                for row in rows
            ]
            if layer == "lane_center":
                lane = (
                    int(rows[0]["road_id"]),
                    int(rows[0]["section_index"]),
                    int(rows[0]["lane_id"]),
                )
                self.lane_points[lane] = points
                markers.append(
                    self.line_marker(layer, points, 0.045, (0.55, 0.58, 0.62, 0.35))
                )
            elif layer == "road_center":
                parts = style.split("|")
                color = (
                    (1.0, 0.72, 0.05, 0.80)
                    if len(parts) > 1 and parts[1] == "yellow"
                    else (0.72, 0.72, 0.72, 0.45)
                )
                if "broken" in style:
                    dashed = self.dashed_points(points)
                    if len(dashed) >= 2:
                        markers.append(
                            self.line_marker(layer, dashed, 0.10, color, Marker.LINE_LIST)
                        )
                else:
                    markers.append(self.line_marker(layer, points, 0.10, color))
            elif layer == "lane_boundary":
                mark_type, color_name, _ = (style.split("|") + ["", "", ""])[:3]
                color = (
                    (1.0, 0.72, 0.05, 0.95)
                    if color_name == "yellow"
                    else (0.92, 0.92, 0.92, 0.85)
                )
                if "broken" in mark_type:
                    dashed = self.dashed_points(points)
                    if len(dashed) >= 2:
                        markers.append(
                            self.line_marker(layer, dashed, 0.10, color, Marker.LINE_LIST)
                        )
                elif mark_type not in {"none", ""}:
                    markers.append(self.line_marker(layer, points, 0.11, color))
            elif layer == "stop_line":
                markers.append(
                    self.line_marker(layer, points, 0.28, (1.0, 1.0, 1.0, 1.0))
                )
            elif layer == "crosswalk":
                markers.append(
                    self.line_marker(layer, points, 0.20, (1.0, 1.0, 1.0, 0.85))
                )
            elif layer == "corridor":
                markers.append(
                    self.line_marker(layer, points, 2.8, (1.0, 0.80, 0.0, 0.13))
                )
                # Corridor lane sections are exported as separate polylines.
                # RViz line strips have uncapped ends, leaving dark seams at
                # otherwise continuous section boundaries.  Flat endpoint
                # discs close those purely visual gaps.
                if points:
                    corridor_endpoints.extend((points[0], points[-1]))
            elif layer == "gateway":
                markers.append(
                    self.line_marker(layer, points, 0.55, (1.0, 0.93, 0.05, 0.88))
                )
            elif layer == "precomputed_network":
                markers.append(self.line_marker(layer, points, 0.12, (1.0, 1.0, 0.0, 0.9)))
            elif layer == "checkpoint" and points:
                x, y = points[0][:2]
                ring = [[x + 1.2 * math.cos(i * 2 * math.pi / 48),
                         y + 1.2 * math.sin(i * 2 * math.pi / 48), 0.35]
                        for i in range(49)]
                markers.append(self.line_marker(layer, ring, 0.18, (1.0, 0.15, 0.65, 1.0)))
            elif layer == "traffic_signal" and points:
                prefix = "controller="
                controller_text = style[len(prefix):] if style.startswith(prefix) else style
                try:
                    controller = int(controller_text)
                except ValueError:
                    controller = -1
                self.signal_positions.setdefault(controller, []).append(points[0])
                sphere = self.base_marker(layer, Marker.SPHERE)
                sphere.pose.position = ros_point(points[0])
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.45
                rgba(sphere, 0.18, 0.18, 0.18, 0.85)
                markers.append(sphere)

        if corridor_endpoints:
            caps = self.base_marker("corridor_caps", Marker.SPHERE_LIST)
            caps.scale.x = caps.scale.y = 2.8
            caps.scale.z = 0.025
            rgba(caps, 1.0, 0.80, 0.0, 0.13)
            caps.points = [ros_point(point) for point in corridor_endpoints]
            markers.append(caps)

        self.static_publisher.publish(MarkerArray(markers=markers))
        self.static_mtime_ns = stat.st_mtime_ns
        self.get_logger().info(
            f"Published static map: {len(markers)} markers, "
            f"{len(self.lane_points)} driving lanes"
        )

    @staticmethod
    def traffic_color(state: int) -> tuple[float, float, float, float]:
        if state in {1, 6}:
            return (1.0, 0.02, 0.02, 1.0)
        if state == 2:
            return (1.0, 0.72, 0.0, 1.0)
        if state in {3, 4, 5}:
            return (0.0, 1.0, 0.12, 1.0)
        return (0.35, 0.35, 0.35, 0.85)

    def add_lane_marker(
        self,
        markers: list[Marker],
        lane_values: list[int],
        namespace: str,
        width: float,
        color: tuple[float, float, float, float],
    ) -> None:
        if len(lane_values) != 3:
            return
        points = self.lane_points.get(tuple(int(value) for value in lane_values))
        if points:
            markers.append(self.line_marker(namespace, points, width, color))

    def publish_dynamic(self, data: dict[str, Any]) -> None:
        self.marker_id = 0
        markers: list[Marker] = []
        delete = self.base_marker("dynamic", Marker.LINE_STRIP)
        delete.action = Marker.DELETEALL
        markers.append(delete)

        ego = data.get("ego", {})
        ego_obb = list(ego.get("obb", []))
        if ego_obb:
            ego_obb.append(ego_obb[0])
            markers.append(
                self.line_marker("ego", ego_obb, 0.20, (0.0, 0.85, 1.0, 1.0))
            )
        heading = float(ego.get("heading", 0.0))
        ego_x = float(ego.get("x", 0.0))
        ego_y = float(ego.get("y", 0.0))
        # 회전은 지도 방향으로 고정하고 카메라만 Ego를 따라가게 한다.
        view_transform = TransformStamped()
        view_transform.header.frame_id = self.frame_id
        view_transform.header.stamp = self.get_clock().now().to_msg()
        view_transform.child_frame_id = "ego_view"
        view_transform.transform.translation.x = ego_x
        view_transform.transform.translation.y = ego_y
        view_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(view_transform)
        arrow = self.base_marker("ego_heading", Marker.ARROW)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.22, 0.42, 0.48
        rgba(arrow, 0.0, 0.85, 1.0, 1.0)
        arrow.points = [
            ros_point([ego_x, ego_y, 0.28]),
            ros_point([ego_x + 3.2 * math.cos(heading),
                       ego_y + 3.2 * math.sin(heading), 0.28]),
        ]
        markers.append(arrow)

        for obj in data.get("objects", []):
            box = list(obj.get("obb", []))
            if not box:
                continue
            box.append(box[0])
            if obj.get("collision"):
                color = (1.0, 0.0, 0.0, 1.0)
            elif obj.get("on_path"):
                color = (1.0, 0.25, 0.0, 1.0)
            else:
                color = (0.75, 0.25, 1.0, 0.85)
            markers.append(self.line_marker("objects", box, 0.16, color))

        selected_lane = data.get("selected_lane", [])
        selected_tuple = tuple(selected_lane) if len(selected_lane) == 3 else None
        for lane in data.get("candidate_lanes", []):
            if tuple(lane) == selected_tuple:
                continue
            self.add_lane_marker(
                markers, lane, "candidate_lanes", 0.22, (0.82, 0.2, 1.0, 0.85)
            )
        self.add_lane_marker(
            markers, selected_lane, "selected_lane", 0.34,
            (0.05, 1.0, 0.12, 1.0),
        )

        for candidate in data.get("candidate_paths", []):
            status = str(candidate.get("status", "candidate"))
            if status == "selected":
                color, width = (0.0, 1.0, 0.12, 1.0), 0.30
            elif status == "rejected":
                color, width = (1.0, 0.08, 0.08, 0.48), 0.10
            else:
                color, width = (1.0, 0.20, 0.85, 0.75), 0.14
            points = candidate.get("points", [])
            if len(points) >= 2:
                markers.append(
                    self.line_marker("candidate_paths_" + status, points, width, color)
                )

        maneuver_path = data.get("maneuver_path", [])
        if len(maneuver_path) >= 2:
            markers.append(
                self.line_marker(
                    "latched_maneuver", maneuver_path, 0.24,
                    (0.0, 0.95, 0.35, 0.92),
                )
            )
        tracked_path = data.get("tracked_path", [])
        if len(tracked_path) >= 2:
            markers.append(
                self.line_marker(
                    "actual_tracked_path", tracked_path, 0.36,
                    (0.0, 0.55, 1.0, 1.0),
                )
            )

        traffic = data.get("traffic_light", {})
        controller = int(traffic.get("id", -1))
        state = int(traffic.get("state", 0))
        signal_color = self.traffic_color(state)
        for position in self.signal_positions.get(controller, []):
            light = self.base_marker("active_traffic_light", Marker.SPHERE)
            light.pose.position = ros_point(position)
            light.scale.x = light.scale.y = light.scale.z = 0.80
            rgba(light, *signal_color)
            markers.append(light)

        decision = data.get("decision", {})
        speed_kph = float(ego.get("speed", 0.0)) * 3.6
        target_kph = float(decision.get("target_speed", 0.0)) * 3.6
        stopped = target_kph <= 0.05
        if bool(decision.get("emergency_brake")):
            status = "STOP: emergency_brake"
        elif stopped:
            reason = str(decision.get("stop_reason_text", "none"))
            if reason in {"", "none", "0"}:
                if str(decision.get("health", "")) == "recovery":
                    reason = "recovery"
                elif not bool(decision.get("plan_valid", True)):
                    reason = "local_path_invalid"
                else:
                    reason = "zero_target_speed"
            status = f"STOP: {reason}"
        else:
            status = "DRIVE"
        summary = f"{status}\nv={speed_kph:.1f}  target={target_kph:.1f} km/h"
        panel_color = (1.0, 0.25, 0.12, 1.0) if stopped else (1.0, 1.0, 1.0, 1.0)
        markers.append(
            self.text_marker(
                "decision_panel", [ego_x, ego_y, 4.0], summary, 0.58,
                panel_color,
            )
        )
        self.dynamic_publisher.publish(MarkerArray(markers=markers))

    def load_latest_frame(self) -> None:
        try:
            stat = self.latest_path.stat()
        except OSError:
            return
        if stat.st_mtime_ns == self.latest_mtime_ns:
            return
        try:
            data = json.loads(self.latest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        self.publish_dynamic(data)
        self.latest_mtime_ns = stat.st_mtime_ns

    def poll(self) -> None:
        self.load_static_map()
        self.load_latest_frame()


def main() -> None:
    rclpy.init()
    options = parse_args(remove_ros_args()[1:])
    node = HLVtdRvizBridge(options)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
