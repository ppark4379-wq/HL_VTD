"""Single-purpose evaluators used by the live assessment orchestrator."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

from assessment_geometry import contains, crosses, intrusion, local_point, obb, overlaps, relevant, signed_distance
from assessment_types import EgoState, FrameData, Point2D, RoadPosition, ViolationResult


ROAD_USERS = {1, 2, 3, 4, 5, 6, 13, 14, 18, 30, 31, 32, 33}
STATIC_TYPES = {7, 8, 9, 15, 16, 19, 21, 22, 23, 25}


@dataclass
class AssessmentContext:
    """All per-frame values shared by independent item evaluators."""

    frame: FrameData
    ego: EgoState
    road: Optional[RoadPosition]
    before: Optional[EgoState]
    dt: float
    row: Dict[str, Any]
    corners: List[Point2D]
    rear_offset: Optional[float]
    rear: Optional[Point2D]


def evaluate_respawn(
    continuous: bool,
    step_m: float,
    dt_s: float,
    speed_mps: float,
    previous_speed_mps: float,
    jump_limit_m: Optional[float],
) -> Tuple[bool, bool]:
    """Return position-relocation and broader discontinuity flags."""
    jump = bool(
        continuous
        and jump_limit_m is not None
        and step_m > max(jump_limit_m, (speed_mps + previous_speed_mps) * dt_s + 5)
    )
    discontinuity = bool(
        continuous
        and step_m > max(5, dt_s * max(100, speed_mps, previous_speed_mps))
    )
    return jump, discontinuity


def resolve_rear_axle(
    ego: EgoState,
    metadata: Dict[str, Any],
    rules: Dict[str, Any],
) -> Tuple[Optional[float], Optional[Point2D]]:
    """Resolve the configured or reference-vehicle rear axle point."""
    rear_offset = metadata.get('rear_axle_offset_m')
    vehicle = rules.get('vehicle_reference', {})
    if (
        rear_offset is None
        and vehicle
        and abs(ego['length'] - vehicle['length_m']) < .05
        and abs(ego['width'] - vehicle['width_m']) < .05
    ):
        rear_offset = ego['off_x'] + ego['length'] / 2 - vehicle['rear_axle_to_front_m']
    rear = local_point(ego, rear_offset) if rear_offset is not None else None
    return rear_offset, rear


def evaluate_school_zone(
    context: AssessmentContext,
    metadata: Dict[str, Any],
    school_zones: Any,
) -> None:
    """Write the rear-axle school-zone membership fields."""
    if context.rear is not None and 'school_zones' in metadata:
        context.row['school_zone_active'] = any(
            contains(zone['polygon'], context.rear) for zone in metadata['school_zones']
        )
        context.row['school_zone_coverage_complete'] = True
    elif school_zones:
        active, zone_ids = school_zones.measure(context.road, context.rear)
        context.row['school_zone_active'] = active
        context.row['school_zone_ids'] = ','.join(zone_ids)
        context.row['school_zone_coverage_complete'] = active is not None
        context.row['school_zone_source'] = 'user_boundary_file+OpenDRIVE_lane_links+rear_axle'


def evaluate_crosswalk(context: AssessmentContext, metadata: Dict[str, Any]) -> None:
    """Write rear-axle occupancy of configured crosswalk polygons."""
    if context.rear is not None and 'crosswalks' in metadata:
        occupied = [
            crosswalk for crosswalk in metadata['crosswalks']
            if contains(crosswalk['polygon'], context.rear)
        ]
        # OpenDRIVE crosswalk polygons can overlap another road in world XY
        # (for example at grade-separated roads).  If the rear axle is inside
        # such a road/lane-constrained polygon but ROAD_POS is absent, neither
        # active nor inactive is provable.  Leave the field unavailable instead
        # of manufacturing a trusted negative sample.
        unresolved = [
            crosswalk for crosswalk in occupied
            if (
                'road_id' in crosswalk
                and (not context.road or 'road' not in context.road)
            ) or (
                crosswalk.get('lane_ids')
                and (not context.road or 'lane' not in context.road)
            )
        ]
        matches = [
            crosswalk for crosswalk in occupied
            if relevant(crosswalk, context.road)
        ]
        if not matches and unresolved:
            return
        context.row['crosswalk_active'] = bool(matches)
        context.row['crosswalk_ids'] = ','.join(str(crosswalk['id']) for crosswalk in matches)
        context.row['crosswalk_source'] = 'map_polygon_rear_axle'


def _evaluate_intrusion_field(
    context: AssessmentContext,
    metadata: Dict[str, Any],
    feature_name: str,
    field_name: str,
) -> None:
    if feature_name in metadata:
        context.row[field_name] = max(
            [
                intrusion(line, context.corners)
                for line in metadata[feature_name]
                if relevant(line, context.road)
            ]
            or [0.0]
        )


def evaluate_centerline(context: AssessmentContext, metadata: Dict[str, Any]) -> None:
    """Measure configured centerline intrusion using the Ego OBB."""
    _evaluate_intrusion_field(context, metadata, 'centerlines', 'centerline_intrusion_m')


def evaluate_sidewalk(context: AssessmentContext, metadata: Dict[str, Any]) -> None:
    """Measure configured sidewalk-boundary intrusion using the Ego OBB."""
    _evaluate_intrusion_field(context, metadata, 'sidewalk_boundaries', 'sidewalk_intrusion_m')


def evaluate_opendrive_solid_lane_change(
    context: AssessmentContext,
    map_data: Any,
    previous_road: Optional[RoadPosition],
    lane_change: bool,
) -> None:
    """Evaluate an actual Lane ID change against OpenDRIVE road marks."""
    if not map_data or not context.road or not context.before:
        return
    context.row['solid_line_crossing'] = False
    if lane_change:
        mark = map_data.boundary_type(
            context.road['road'],
            context.road['s'],
            previous_road['lane'],
            context.road['lane'],
        )
        contact = map_data.boundary_contact(
            context.before,
            context.ego,
            context.road['road'],
            context.road['s'],
            previous_road['lane'],
            context.road['lane'],
        )
        context.row['solid_line_crossing'] = (
            contact
            if mark in {'solid', 'solid solid'}
            else False
            if mark in {'broken', 'none', 'broken broken'}
            else None
        )


def evaluate_solid_lane_change(
    context: AssessmentContext,
    metadata: Dict[str, Any],
    lane_change: bool,
) -> None:
    """Override the solid-line result when explicit boundary metadata exists."""
    if 'lane_boundaries' in metadata and context.road:
        context.row['solid_line_crossing'] = False if context.before else None
    if 'lane_boundaries' not in metadata or not context.before:
        return
    crossed = [
        line for line in metadata['lane_boundaries']
        if relevant(line, context.road)
        and (
            crosses(line, (context.before['x'], context.before['y']), (context.ego['x'], context.ego['y']))
            or crosses(
                {**line, 'direction': -line.get('direction', 1)},
                (context.before['x'], context.before['y']),
                (context.ego['x'], context.ego['y']),
            )
        )
    ]
    before_and_current = obb(context.before) + context.corners
    context.row['solid_line_crossing'] = lane_change and any(
        line.get('type') == 'solid'
        and min(signed_distance(line, point) for point in before_and_current) <= 0
        <= max(signed_distance(line, point) for point in before_and_current)
        for line in crossed
    )


def evaluate_speed_limit(speed_mps: float, limit_kmh: float) -> List[ViolationResult]:
    """Return normal and major overspeed events without mutating session state."""
    overspeed = speed_mps * 3.6 - limit_kmh
    evidence = {'overspeed_kmh': overspeed}
    return [
        {'event': 'speed_limit', 'active': overspeed > 1, 'source': 'RDB speed', 'evidence': evidence},
        {'event': 'speed_limit_major', 'active': overspeed > 20, 'source': 'RDB speed', 'evidence': evidence},
    ]


def collect_assessment_objects(context: AssessmentContext) -> List[EgoState]:
    """Return non-Ego objects with usable 2D dimensions and categories."""
    return [
        item for item in context.frame['objects'].values()
        if item['id'] != context.ego['id']
        and item['length'] > 0
        and item['width'] > 0
        and item['category'] in {1, 5, 6}
    ]


def measure_nearest_object(context: AssessmentContext, objects: List[EgoState]) -> None:
    """Write the nearest object's identity and geometry for audit telemetry."""
    nearest = min(
        objects,
        key=lambda item: math.hypot(item['x'] - context.ego['x'], item['y'] - context.ego['y']),
        default=None,
    )
    if nearest is None:
        context.row['nearest_object_distance'] = None
        return
    context.row.update(
        nearest_object_id=nearest['id'],
        nearest_object_name=nearest.get('name', ''),
        nearest_object_category=nearest['category'],
        nearest_object_type=nearest['type'],
        nearest_object_x=nearest['x'],
        nearest_object_y=nearest['y'],
        nearest_object_speed=nearest.get('speed'),
        nearest_object_length=nearest['length'],
        nearest_object_width=nearest['width'],
        nearest_object_distance=math.hypot(
            nearest['x'] - context.ego['x'], nearest['y'] - context.ego['y']
        ),
    )


def evaluate_collisions(
    context: AssessmentContext,
    objects: List[EgoState],
    metadata: Dict[str, Any],
) -> None:
    """Classify provisional OBB contacts into official collision fields."""
    context.row['road_user_collision'] = context.row['static_collision'] = False
    unknown_contact = False
    road_user_contacts = []
    static_contacts = []
    unknown_contacts = []
    for item in objects:
        if abs((item['z'] + item['off_z']) - (context.ego['z'] + context.ego['off_z'])) > (
            item['height'] + context.ego['height']
        ) / 2:
            continue
        if not overlaps(context.corners, obb(item)):
            continue
        if item['type'] in ROAD_USERS and item['category'] == 1:
            context.row['road_user_collision'] = True
            road_user_contacts.append(item)
        elif item['type'] in STATIC_TYPES or metadata.get('object_roles', {}).get(item['name']) == 'static_obstacle':
            context.row['static_collision'] = True
            static_contacts.append(item)
        else:
            unknown_contact = True
            unknown_contacts.append(item)

    for prefix, contacts in (
        ('road_user_collision', road_user_contacts),
        ('static_collision', static_contacts),
        ('unknown_collision', unknown_contacts),
    ):
        context.row[f'{prefix}_ids'] = ','.join(str(item['id']) for item in contacts)
        context.row[f'{prefix}_names'] = ','.join(item.get('name', '') for item in contacts)
        context.row[f'{prefix}_types'] = ','.join(str(item['type']) for item in contacts)

    if unknown_contact:
        for field in ('road_user_collision', 'static_collision'):
            if not context.row[field]:
                context.row[field] = None
    context.row['collision_source'] = 'OBB_overlap_provisional'
    if not metadata.get('object_feed_complete', False):
        for field in ('road_user_collision', 'static_collision'):
            if not context.row[field]:
                context.row[field] = None


def find_stop_line(
    context: AssessmentContext,
    metadata: Dict[str, Any],
) -> Optional[Tuple[float, Dict[str, Any]]]:
    """Return the closest relevant stop line and its signed bumper distance."""
    bumper = local_point(
        context.ego,
        context.ego['off_x'] + context.ego['length'] / 2,
        context.ego['off_y'],
    )
    candidates = []
    for line in metadata.get('stop_lines', []):
        if relevant(line, context.road):
            distance = -signed_distance(line, bumper) * line.get('direction', 1)
            if -10 <= distance <= 60:
                candidates.append((abs(distance), distance, line))
    if not candidates:
        return None
    _, distance, line = min(candidates, key=lambda item: item[0])
    return distance, line


def evaluate_traffic_signal(
    context: AssessmentContext,
    stop_line: Optional[Tuple[float, Dict[str, Any]]],
) -> None:
    """Map the selected stop line's VTD signal mask to the configured state."""
    if stop_line is None:
        return
    distance, line = stop_line
    signal = context.frame['signals'].get(line.get('signal_id'))
    if not signal:
        return
    state_masks = line.get('state_masks', {})
    state = state_masks.get(signal['mask']) or state_masks.get(str(signal['mask']))
    if state:
        context.row.update(
            stop_line_id=line['id'],
            stop_line_distance_m=distance,
            traffic_light_state=state,
        )


def evaluate_green_start(
    context: AssessmentContext,
    objects: List[EgoState],
    stop_line: Optional[Tuple[float, Dict[str, Any]]],
    object_feed_complete: bool,
    forward_distance_m: float,
    corridor_factory: Callable[[EgoState, Dict[str, Any]], List[List[Point2D]]],
) -> None:
    """Record objects that justify waiting at a mapped signal stop line."""
    if stop_line is None or not object_feed_complete:
        return
    corridors = corridor_factory(context.ego, context.row)
    blockers = [
        item for item in objects
        if any(overlaps(corridor, obb(item)) for corridor in corridors)
    ]
    context.row['justified_stop'] = bool(blockers)
    context.row['green_blocker_forward_m'] = forward_distance_m
    context.row['green_blocker_ids'] = ','.join(str(item['id']) for item in blockers)
    context.row['green_blocker_source'] = 'planned_path_vehicle_width_overlap_object_obb'


def evaluate_pedestrian_response(
    context: AssessmentContext,
    objects: List[EgoState],
    metadata: Dict[str, Any],
    pedestrian_stops: Dict[str, bool],
    stop_speed_kmh: float,
) -> None:
    """Evaluate the finalized crosswalk stopping and release rule."""
    if 'crosswalks' not in metadata or not metadata.get('object_feed_complete', False):
        return
    context.row['pedestrian_violation'] = False
    for crosswalk in metadata['crosswalks']:
        if not relevant(crosswalk, context.road):
            continue
        pedestrians = [
            item for item in objects
            if item['type'] in {5, 6}
            and (
                overlaps(crosswalk['polygon'], obb(item))
                or (
                    item['name'] in crosswalk.get('crossing_roles', [])
                    and any(
                        contains(polygon, (item['x'], item['y']))
                        for polygon in crosswalk.get('entry_polygons', [])
                    )
                )
            )
        ]
        stopped = abs(context.ego['speed']) <= stop_speed_kmh / 3.6
        identifier = str(crosswalk['id'])
        center_x = sum(point[0] for point in crosswalk['polygon']) / len(crosswalk['polygon'])
        center_y = sum(point[1] for point in crosswalk['polygon']) / len(crosswalk['polygon'])
        ahead = (
            (center_x - context.ego['x']) * math.cos(context.ego['h'])
            + (center_y - context.ego['y']) * math.sin(context.ego['h'])
        )
        near = -context.ego['length'] <= ahead <= crosswalk.get('approach_distance_m', 30)
        if pedestrians and stopped and near:
            pedestrian_stops[identifier] = True
        if pedestrians and near and not stopped and (
            overlaps(context.corners, crosswalk['polygon'])
            or pedestrian_stops.get(identifier, False)
        ):
            context.row['pedestrian_violation'] = True
        if not pedestrians or not near:
            pedestrian_stops.pop(identifier, None)
        if pedestrians and near:
            context.row['justified_stop'] = True
