"""Passive live assessment: RDB frames -> evidence -> official deductions -> UI."""
from __future__ import annotations
import json
import math
import threading
import time
import uuid
from collections import defaultdict
from pathlib import Path
import sys
from typing import Any, Dict, Optional
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'verification'))
import evaluate
from assessment_geometry import obb, local_point, contains, crosses, relevant, validate_metadata
from assessment_evaluators import (
    AssessmentContext,
    collect_assessment_objects,
    evaluate_centerline,
    evaluate_collisions,
    evaluate_crosswalk,
    evaluate_green_start,
    evaluate_opendrive_solid_lane_change,
    evaluate_pedestrian_response,
    evaluate_respawn,
    evaluate_school_zone,
    evaluate_sidewalk,
    evaluate_solid_lane_change,
    evaluate_speed_limit,
    evaluate_traffic_signal,
    find_stop_line,
    measure_nearest_object,
    resolve_rear_axle,
)
from assessment_types import AssessmentSnapshot, EgoState, FrameData, Point2D, RoadPosition
from opendrive_assessment import OpenDriveMap
from route_assessment import RouteProgress
from school_zones import SchoolZones
from map_features import extract_features, finish_candidates, finish_from_object

class LiveAssessment:
    def __init__(self, rules_path, output_dir):
        self.rules_path = Path(rules_path)
        self._load_rules()
        self._configure_completion()
        self._configure_map()
        self._configure_school_zones()
        self._configure_route()
        self.output_dir = Path(output_dir)
        self.lock = threading.RLock()
        self.session = self.rules['session']
        self._initialize_runtime_state()

    def _load_rules(self):
        self.rules = yaml.safe_load(self.rules_path.read_text(encoding='utf-8'))
        if not isinstance(self.rules, dict):
            raise ValueError('assessment rules must be a mapping')
        required_groups = (
            (self.rules, ('base_score', 'normal_speed_kmh', 'school_speed_kmh')),
            (self.rules.get('session', {}), ('max_sample_gap_s', 'time_limit_s', 'stop_speed_kmh')),
        )
        for group, keys in required_groups:
            for key in keys:
                value = group.get(key)
                if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value < 0:
                    raise ValueError(f'{key}: finite nonnegative number required')
        session = self.rules['session']
        if session['time_limit_s'] <= 0 or session['max_sample_gap_s'] <= 0:
            raise ValueError('time_limit_s and max_sample_gap_s must be positive')
        if session.get('start_trigger', 'controller_packet_arrival') != 'controller_packet_arrival':
            raise ValueError('start_trigger must be controller_packet_arrival under finalized rules')
        green_exception = self.rules.get('green_start_exception') or {}
        self.green_blocker_forward_m = green_exception.get('forward_path_m', 30.0)
        value = self.green_blocker_forward_m
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError('green_start_exception.forward_path_m must be finite and positive')
        for key, value in {'respawn_jump_m': session.get('respawn_jump_m')}.items():
            if value is not None and (not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0):
                raise ValueError(f'{key}: positive number or null required')

    def _configure_completion(self):
        completion = self.rules.get('completion') or {}
        self.completion_mode = completion.get('mode', 'finish_line')
        if self.completion_mode not in {'final_checkpoint', 'finish_line'}:
            raise ValueError('completion mode must be final_checkpoint or finish_line')
        self.checkpoint_tolerance = completion.get('tolerance_m', 1.5)
        value = self.checkpoint_tolerance
        if self.completion_mode == 'final_checkpoint' and (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError('checkpoint tolerance_m must be finite and positive')

    def _configure_map(self):
        self.meta = self.rules.get('metadata') or {}
        validate_metadata(self.meta)
        map_path = self.rules.get('map', {}).get('xodr')
        if map_path:
            map_path = Path(map_path)
            if not map_path.is_absolute():
                map_path = Path(__file__).resolve().parents[2] / map_path
        self.map = OpenDriveMap(map_path) if map_path else None
        self.map_stop_lines = []
        if self.map and self.rules.get('map', {}).get('extract_crosswalks', False):
            self.map_stop_lines, crosswalks = extract_features(self.map)
            self.meta.setdefault('crosswalks', crosswalks)
        selection = self.rules.get('finish_stop_line')
        if self.completion_mode == 'finish_line' and selection and selection.get('confirmed', False):
            if not self.map:
                raise ValueError('finish_stop_line requires an OpenDRIVE map')
            if not self.map_stop_lines:
                self.map_stop_lines, _ = extract_features(self.map)
            self.meta['finish_line'] = finish_from_object(self.map_stop_lines, selection)
        validate_metadata(self.meta)

    def _configure_school_zones(self):
        school_path = self.rules.get('school_zones_file')
        if school_path:
            school_path = Path(school_path)
            if not school_path.is_absolute():
                school_path = Path(__file__).resolve().parents[2] / school_path
            if self.map is None:
                raise ValueError('school_zones_file requires OpenDRIVE map')
        corrections_path = self.rules.get('school_zone_corrections_file')
        if corrections_path:
            corrections_path = Path(corrections_path)
            if not corrections_path.is_absolute():
                corrections_path = Path(__file__).resolve().parents[2] / corrections_path
        self.school_zones = SchoolZones(school_path, self.map, corrections_path) if school_path else None
        if self.school_zones and self.school_zones.limit != self.rules['school_speed_kmh']:
            raise ValueError('school-zone speed limit differs from assessment rules')

    def _configure_route(self):
        route_config = self.rules.get('route') or {}
        route_path = route_config.get('csv')
        if route_path:
            route_path = Path(route_path)
            if not route_path.is_absolute():
                route_path = Path(__file__).resolve().parents[2] / route_path
        self.route = RouteProgress(route_path, route_config.get('max_projection_distance_m', 20.0)) if route_path else None
        self.route_start_s = route_config.get('start_s_m', self.route.min_s if self.route else None)
        self.route_goal_s = route_config.get('target_s_m', self.route.max_s if self.route else None)
        self.route_section_count = route_config.get('section_count', 5)
        if self.route:
            if not isinstance(self.route_section_count, int) or isinstance(self.route_section_count, bool) or self.route_section_count <= 0:
                raise ValueError('route section_count must be a positive integer')
            for key, value in (('start_s_m', self.route_start_s), ('target_s_m', self.route_goal_s)):
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f'route {key} must be finite')
            self.route_start_s = max(self.route.min_s, float(self.route_start_s))
            self.route_goal_s = min(self.route.max_s, float(self.route_goal_s))
            if self.route_goal_s <= self.route_start_s:
                raise ValueError('route target must follow start')
        self.checkpoint = None
        if self.completion_mode == 'final_checkpoint':
            if not self.route:
                raise ValueError('final_checkpoint completion requires a route CSV')
            self.checkpoint = self.route.checkpoint_at(self.route_goal_s)
        self.finish_candidates = finish_candidates(self.map_stop_lines, self.route) if self.completion_mode == 'finish_line' else []
        self.section_ids = [str(i) for i in range(1, self.route_section_count + 1)] if self.route else []

    def _initialize_runtime_state(self):
        self.rows = []
        self.status = 'WAITING'
        self.session_id = None
        self.previous = None
        self._previous_road = None
        self._previous_lane_info = None
        self.expected_side = None
        self.last_received = None
        self.gap = False
        self.started_at = None
        self.pending_control = None
        self.last_control = None
        self.start_evidence = None
        self.fresh_start = False
        self.distance = 0.0
        self.section = None
        self.route_hint_s = None
        self.respawns = defaultdict(int)
        self.turn_since = None
        self.indicator_observed_since = None
        self.turn = None
        self.lane_duration = 0.0
        self.previous_lane_active = False
        self.pedestrian_stops = {}
        self.active_events = {}
        self.events = []
        self.deductions = []
        self.deducted = {}
        self.file = None
        self.cache = None
        self.cache_at = 0

    def on_gap(self):
        with self.lock:
            self.gap = True

    def on_control_received(self, evidence):
        """Start on controller arrival, including a command whose values are zero.

        Simulation timestamps are sampled by RDB, so the first received command
        is aligned to the latest fresh frame (or first subsequent frame).
        """
        with self.lock:
            self.last_control=dict(evidence)
            if self.status != 'WAITING': return
            self.pending_control=dict(evidence)
            self.pending_control['received_monotonic']=time.monotonic()
            if self.previous and self.last_received is not None and time.monotonic()-self.last_received <= 2:
                self.start_evidence=self.pending_control
                self._begin(self.previous['time'])

    def _write(self, kind, data):
        if self.file:
            self.file.write(json.dumps({'kind':kind, **data}, ensure_ascii=False, allow_nan=False)+'\n')
            self.file.flush()

    def _begin(self, now):
        self.session_id = time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.file = (self.output_dir / (self.session_id+'.jsonl')).open('x',encoding='utf-8')
        self.started_at = now
        self.status = 'RUNNING'
        self.fresh_start = True
        self.pending_control = None
        self.cache = None
        self._write('session', {'session_id':self.session_id,'rules':self.rules,'start_sim_time':now,
                               'start_source':'controller_packet_arrival','start_evidence':self.start_evidence})

    def _finish(self, status):
        self.status = status
        self.cache = None
        self.snapshot()
        self._write('result', self.cache)
        if self.file:
            self.file.close()
            self.file = None

    def close(self):
        with self.lock:
            if self.file:
                self._finish('ABORTED')

    def _reset(self):
        if self.session_id:
            self._finish('RESET' if self.status == 'RUNNING' else self.status)
        self.rows = []
        self.session_id = None
        self.started_at = self.pending_control = self.start_evidence = None
        self.last_control = None
        self.fresh_start = False
        self.previous = None
        self._previous_road = None
        self._previous_lane_info = None
        self.expected_side = None
        self.distance = 0.0
        self.section = None
        self.route_hint_s = None
        self.respawns.clear()
        self.turn = self.turn_since = None
        self.indicator_observed_since = None
        self.lane_duration = 0.0
        self.previous_lane_active = False
        self.pedestrian_stops.clear()
        self.active_events.clear()
        self.events = []
        self.deductions = []
        self.deducted = {}
        self.cache = None
        self.status = 'WAITING'

    def on_frame(self, frame: FrameData):
        with self.lock:
            now = frame['time']
            ego = next((o for o in frame['objects'].values() if o['name'] == self.meta.get('ego_name','Ego') and o['category'] == 1), None)
            if ego is None:
                self.gap = True
                return
            if self.previous and (now < self.previous['time'] or frame['frame'] < self.previous['frame']):
                self._reset()
            self.last_received = time.monotonic()
            previous = self.previous
            if previous and now == previous['time']:
                return  # Paused VTD frame: no time, distance, or violation accrues.
            dt = now-previous['time'] if previous else 0.0
            continuous = previous is not None and 0 < dt <= self.session['max_sample_gap_s'] and not self.gap
            self.gap = False
            before = previous.get('ego') if previous else None
            frame['ego'] = ego
            self.previous = frame
            speed = ego.get('speed')
            step = math.hypot(ego['x']-before['x'],ego['y']-before['y']) if before else 0.0
            jump_limit = self.session.get('respawn_jump_m')
            jump, discontinuity = evaluate_respawn(
                continuous,
                step,
                dt,
                speed or 0,
                before.get('speed') or 0 if before else 0,
                jump_limit,
            )
            if speed is None and continuous and not jump and not discontinuity:
                speed = step/dt
                ego['speed'] = speed
                ego['speed_source'] = 'position_difference'
            if speed is None or not math.isfinite(speed):
                self.gap = True
                return
            if self.status == 'WAITING':
                if self.pending_control:
                    if time.monotonic()-self.pending_control['received_monotonic'] <= 2:
                        self.start_evidence=self.pending_control
                        self._begin(now)
                    else:
                        self.pending_control=None
            if self.status != 'RUNNING':
                return
            if self.fresh_start:
                continuous=False
                jump=discontinuity=False
                self.fresh_start=False
            if not continuous:
                self.turn_since = self.turn = None
                self.lane_duration = 0
                self.previous_lane_active = False
            if continuous and not jump and not discontinuity and self.rows:
                self.distance += step
            row = self._measure(frame, ego, before if continuous and not jump and not discontinuity else None, dt if continuous and not jump and not discontinuity else 0)
            if jump:
                source_section = self.rows[-1]['evaluation_section_id'] if self.rows else row['evaluation_section_id']
                self.respawns[source_section] += 1
                row['evaluation_section_id'] = source_section
                row['respawn_count'] = self.respawns[source_section]
                row['goal_reached'] = False
                self.active_events['respawn'] = False
                self._event('respawn', row, True, 'vtd_position_relocation', {'jump_m':step})
            elif jump_limit is not None:
                row['respawn_count'] = self.respawns[row['evaluation_section_id']]
                self.active_events['respawn'] = False
            row['data_valid'] = 1
            if not continuous or jump or discontinuity:
                row['max_sample_gap_s'] = 1e-12
            self.rows.append({k: '' if v is None else str(int(v)) if isinstance(v,bool) else str(v) for k,v in row.items()})
            self._write('frame',row)
            self.cache = None
            if row['goal_reached']:
                self._finish('COMPLETED')
            elif row['elapsed_wall_time_s'] >= self.session['time_limit_s']:
                self._finish('TIMEOUT')

    def _event(self, name, row, active, source, evidence=None):
        if active and not self.active_events.get(name,False):
            event = dict(event=name,sim_time_s=row['sim_time_s'],section=row['evaluation_section_id'],
                         x=row['ego_x'],y=row['ego_y'],source=source,evidence=evidence or {})
            self.events.append(event)
            self._write('event',event)
        self.active_events[name] = active

    def _route_progress(
        self,
        ego: EgoState,
        road: Optional[RoadPosition],
        rear: Optional[Point2D],
    ):
        projection = self.route.project(ego, road, self.route_hint_s) if self.route else None
        route_s = projection['s'] if projection else None
        if projection:
            self.route_hint_s = route_s
        section_point = rear or (ego['x'], ego['y'])
        if self.route and route_s is not None:
            progress = max(0.0, min(1.0, (route_s-self.route_start_s)/(self.route_goal_s-self.route_start_s)))
            section = str(min(self.route_section_count, int(progress*self.route_section_count)+1))
        elif self.route:
            section = self.section
        else:
            section = next(
                (str(item['id']) for item in self.meta.get('sections', []) if contains(item['polygon'], section_point)),
                None,
            )
        self.section = section
        return projection, route_s, section

    def _new_assessment_row(
        self,
        frame: FrameData,
        ego: EgoState,
        section: Optional[str],
        projection: Optional[Dict[str, Any]],
        route_s: Optional[float],
    ) -> Dict[str, Any]:
        key = section or 'unsegmented'
        row = dict(
            scenario_id=self.meta.get('scenario_id', 'VTD_LIVE'),
            run_id=self.session_id,
            frame=frame['frame'],
            sim_time_s=frame['time'],
            elapsed_wall_time_s=frame['time']-self.started_at,
            ego_x=ego['x'],
            ego_y=ego['y'],
            ego_heading=ego['h'],
            ego_speed=ego['speed'],
            evaluation_section_id=key if section else '',
            distance_m=self.distance,
            current_speed_limit_kmh=self.rules['normal_speed_kmh'],
            goal_reached=False,
            stop_speed_kmh=self.session['stop_speed_kmh'],
            max_sample_gap_s=self.session['max_sample_gap_s'],
            nearest_object_distance=None,
            command_steering=None,
            target_accel=None,
            speed_source=ego.get('speed_source', 'RDB_OBJECT_STATE_EXT'),
            measurement_source='RDB+scenario_metadata',
        )
        if self.last_control:
            row['command_steering'] = self.last_control.get('steering')
            row['target_accel'] = self.last_control.get('accel')
        if self.route:
            row['route_s_m'] = route_s
            row['route_distance_m'] = projection['distance_m'] if projection else None
        return row

    def _evaluate_completion(self, context: AssessmentContext) -> None:
        row = context.row
        ego = context.ego
        if self.checkpoint:
            row['checkpoint_distance_m'] = math.hypot(
                ego['x']-self.checkpoint['x'],
                ego['y']-self.checkpoint['y'],
            )
            row['goal_source'] = 'final_checkpoint'
            if context.before:
                row['goal_reached'] = self.route.reaches_checkpoint(
                    context.before,
                    ego,
                    self.checkpoint,
                    self.checkpoint_tolerance,
                    context.road,
                )
        finish = self.meta.get('finish_line') if self.completion_mode == 'finish_line' else None
        if finish and context.rear is not None and context.before and relevant(finish, context.road):
            row['goal_reached'] = crosses(
                finish,
                local_point(context.before, context.rear_offset),
                context.rear,
            )
            row['finish_line_id'] = finish.get('id', 'configured_finish')

    def _evaluate_lane_rules(self, context: AssessmentContext) -> None:
        frame = context.frame
        ego = context.ego
        road = context.road
        row = context.row
        now = frame['time']
        lane = frame['lanes'].get((ego['id'], road['road'], road['lane'])) if road else None
        previous_road = context.before and self._previous_road
        lane_change = bool(
            previous_road
            and road
            and road['road'] == previous_road['road']
            and road['lane'] != previous_road['lane']
        )
        if road and road['lane'] and (not previous_road or road['road'] != previous_road['road']):
            self.expected_side = -1 if math.cos(road['heading']) >= 0 else 1
            if self.map and road['road'] in self.map.roads and self.map.roads[road['road']].get('rule') == 'LHT':
                self.expected_side *= -1
        map_values = self.map.measure(ego, road, self.expected_side) if self.map and road and self.expected_side else {}
        for field in ('centerline_intrusion_m', 'sidewalk_intrusion_m'):
            if field not in row and field in map_values:
                row[field] = map_values[field]
        evaluate_opendrive_solid_lane_change(context, self.map, previous_road, lane_change)
        self._previous_road = road
        self._evaluate_turn_signal(context, lane, previous_road, lane_change)
        self._previous_lane_info = lane
        evaluate_solid_lane_change(context, self.meta, lane_change)

    def _evaluate_turn_signal(
        self,
        context: AssessmentContext,
        lane: Optional[Dict[str, Any]],
        previous_road: Optional[RoadPosition],
        lane_change: bool,
    ) -> None:
        frame = context.frame
        ego = context.ego
        road = context.road
        row = context.row
        now = frame['time']
        indicator = frame['lights'].get(ego['id'])
        if indicator is None or context.dt == 0:
            self.indicator_observed_since = now if indicator is not None else None
        elif self.indicator_observed_since is None:
            self.indicator_observed_since = now
        if indicator != self.turn or context.dt == 0:
            self.turn, self.turn_since = indicator, now if indicator else None
        if indicator is not None and road and lane:
            row['turn_signal_violation'] = False
            if lane_change:
                old_lane = frame['lanes'].get(
                    (ego['id'], previous_road['road'], previous_road['lane'])
                ) or self._previous_lane_info
                direction = (
                    'LEFT' if old_lane and old_lane['left'] == road['lane']
                    else 'RIGHT' if old_lane and old_lane['right'] == road['lane']
                    else None
                )
                known_history = self.indicator_observed_since is not None and now-self.indicator_observed_since >= 3-1e-9
                row['turn_signal_violation'] = (
                    None
                    if direction is None or not known_history
                    else indicator != direction or self.turn_since is None or now-self.turn_since < 3-1e-9
                )

    def _record_measurement_events(self, row: Dict[str, Any], ego: EgoState) -> None:
        for field in ('solid_line_crossing', 'turn_signal_violation', 'pedestrian_violation', 'static_collision', 'road_user_collision'):
            if field in row:
                self._event(field, row, row[field] is True, 'RDB+OBB+metadata')
        for result in evaluate_speed_limit(ego['speed'], row['current_speed_limit_kmh']):
            self._event(
                result['event'],
                row,
                result['active'],
                result['source'],
                result['evidence'],
            )

    def _measure(
        self,
        frame: FrameData,
        ego: EgoState,
        before: Optional[EgoState],
        dt: float,
    ) -> Dict[str, Any]:
        road = frame['road'].get(ego['id'])
        corners = obb(ego)
        rear_offset, rear = resolve_rear_axle(ego, self.meta, self.rules)
        projection, route_s, section = self._route_progress(ego, road, rear)
        row = self._new_assessment_row(frame, ego, section, projection, route_s)
        row['rear_axle_available']=rear is not None
        context = AssessmentContext(
            frame=frame,
            ego=ego,
            road=road,
            before=before,
            dt=dt,
            row=row,
            corners=corners,
            rear_offset=rear_offset,
            rear=rear,
        )
        evaluate_school_zone(context, self.meta, self.school_zones)
        evaluate_crosswalk(context, self.meta)
        self._evaluate_completion(context)
        evaluate_centerline(context, self.meta)
        evaluate_sidewalk(context, self.meta)

        self._evaluate_lane_rules(context)
        self._objects_and_signals(context)
        self._record_measurement_events(row, ego)
        return row

    def _forward_path_corridors(self, ego, row):
        """Vehicle-width polygons from the front bumper to 30 m along the planned route."""
        half_width = ego['width'] / 2
        front_offset = ego.get('off_x', 0) + ego['length'] / 2
        front = local_point(ego, front_offset, ego.get('off_y', 0))
        points = [front]
        route_s = row.get('route_s_m')
        if self.route and route_s is not None:
            start_s = min(self.route_goal_s, route_s + max(0.0, front_offset))
            end_s = min(self.route_goal_s, start_s + self.green_blocker_forward_m)
            sample_s = start_s
            while sample_s < end_s:
                point = self.route.point_at(sample_s)
                if math.hypot(point[0]-points[-1][0], point[1]-points[-1][1]) > 1e-6:
                    points.append(point)
                sample_s = min(end_s, sample_s + 1.0)
            endpoint = self.route.point_at(end_s)
            if math.hypot(endpoint[0]-points[-1][0], endpoint[1]-points[-1][1]) > 1e-6:
                points.append(endpoint)
        if len(points) == 1:
            points.append(local_point(ego, front_offset + self.green_blocker_forward_m,
                                      ego.get('off_y', 0)))
        corridors = []
        for first, second in zip(points, points[1:]):
            dx, dy = second[0]-first[0], second[1]-first[1]
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue
            nx, ny = -dy/length*half_width, dx/length*half_width
            corridors.append([(first[0]-nx, first[1]-ny),
                              (second[0]-nx, second[1]-ny),
                              (second[0]+nx, second[1]+ny),
                              (first[0]+nx, first[1]+ny)])
        return corridors

    def _objects_and_signals(self, context, ego=None, road=None, row=None) -> None:
        # Preserve the pre-refactor helper signature used by diagnostics and
        # tests while the normal frame path passes one typed context object.
        if not isinstance(context, AssessmentContext):
            context = AssessmentContext(
                frame=context,
                ego=ego,
                road=road,
                before=None,
                dt=0.0,
                row=row,
                corners=obb(ego),
                rear_offset=None,
                rear=None,
            )
        objects = collect_assessment_objects(context)
        measure_nearest_object(context, objects)
        evaluate_collisions(context, objects, self.meta)
        stop_line = find_stop_line(context, self.meta)
        evaluate_traffic_signal(context, stop_line)
        evaluate_green_start(
            context,
            objects,
            stop_line,
            self.meta.get('object_feed_complete', False),
            self.green_blocker_forward_m,
            self._forward_path_corridors,
        )
        evaluate_pedestrian_response(
            context,
            objects,
            self.meta,
            self.pedestrian_stops,
            self.session['stop_speed_kmh'],
        )

    def snapshot(self) -> AssessmentSnapshot:
        with self.lock:
            if self.cache is None or time.monotonic()-self.cache_at>1:
                results = evaluate.evaluate_official_for(self.rows,self.rules['normal_speed_kmh'],self.rules['school_speed_kmh']) if self.rows else []
                grid = [dict(number=r.number,status=r.status,penalty=r.penalty,detail=r.detail,
                             by_section=r.severity_by_section,penalty_by_section=r.penalty_by_section) for r in results]
                fields={1:'ego_speed',2:'school_zone_active',4:'centerline_intrusion_m',5:'sidewalk_intrusion_m',
                        6:'solid_line_crossing',7:'traffic_light_state',8:'justified_stop',9:'traffic_light_state',
                        10:'pedestrian_violation',11:'static_collision',12:'crosswalk_active',
                        13:'turn_signal_violation',14:'road_user_collision',15:'respawn_count'}
                section_rows=defaultdict(list)
                for record in self.rows: section_rows[record['evaluation_section_id'] or 'unsegmented'].append(record)
                for item in grid:
                    field=fields[item['number']]
                    item['observed_sections']=[key for key,records in section_rows.items() if any(r.get(field,'')!='' for r in records)]
                    item['partial_sections']=[key for key,records in section_rows.items() if any(r.get(field,'')=='' for r in records)]
                if self.school_zones and self.school_zones.pending:
                    for item in grid:
                        if item['number']==2:
                            item['detail'] += f"; 보호구역 {len(self.school_zones.accepted)}개 반영 / {len(self.school_zones.pending)}개 경계 확인 대기"
                reached = list(dict.fromkeys(r['evaluation_section_id'] for r in self.rows if r['evaluation_section_id']))
                total_penalty = sum(r.penalty for r in results)
                unsegmented = any(not r['evaluation_section_id'] for r in self.rows)
                score = self.rules['base_score']+total_penalty
                for item in results:
                    for section,points in item.penalty_by_section.items():
                        key = (section,item.number)
                        delta = points-self.deducted.get(key,0)
                        if delta>0:
                            event = dict(item=item.number,section=section,delta=delta,total=points,
                                         sim_time_s=self.rows[-1]['sim_time_s'],source='DOCX+organizer_answers',detail=item.detail)
                            self.deductions.append(event)
                            self._write('deduction',event)
                            self.deducted[key]=points
                major = sum(points//6 if item.number==15 else int(item.severity_by_section.get(section)==2)
                            for item in results for section,points in item.penalty_by_section.items())
                unavailable = [r.number for r in results if r.status=='UNAVAILABLE']
                school_partial=bool(self.school_zones and self.school_zones.pending)
                missing=[key for key in ('school_zones','crosswalks','stop_lines','centerlines','sidewalk_boundaries') if key not in self.meta and not (key=='school_zones' and self.school_zones) and not (key in ('centerlines','sidewalk_boundaries') and self.map)]
                if not self.route and 'sections' not in self.meta: missing.append('sections')
                if self.completion_mode == 'finish_line' and 'finish_line' not in self.meta: missing.append('finish_line')
                if not self.meta.get('object_feed_complete',False): missing.append('object_feed_complete')
                warnings=[]
                if missing: warnings.append('필수 데이터 확인 필요: '+', '.join(missing))
                if school_partial: warnings.append(f'보호구역 {len(self.school_zones.accepted)}개 반영 / {len(self.school_zones.pending)}개 경계 확인 필요')
                if self.rows and self.rows[-1].get('rear_axle_available')=='0':
                    affected = '종료선·보호구역·횡단보도' if self.completion_mode == 'finish_line' else '보호구역·횡단보도'
                    warnings.append('차량 후륜축 위치 미확정: '+affected+' 판정 불가')
                self.cache = dict(session_id=self.session_id,status=self.status,
                                  unsegmented_frames=unsegmented,
                                  sections=self.section_ids or [str(s['id']) for s in self.meta.get('sections',[])],reached_sections=reached,
                                  official_grid=grid,summary=dict(official_score=score,official_penalty=total_penalty,
                                  official_evaluation='WAITING' if not self.rows else 'PARTIAL' if unavailable or unsegmented or school_partial or missing or any(i['partial_sections'] for i in grid) else 'COMPLETE',
                                  official_unavailable_items=unavailable,major_count=major),
                                  latest=self.rows[-1] if self.rows else {},time_limit_s=self.session['time_limit_s'],
                                  events=self.events[-100:],deductions=self.deductions[-100:],
                                  school_zone_validation=self.school_zones.report() if self.school_zones else None,
                                  missing_metadata=missing,warnings=warnings,finish_candidates=self.finish_candidates,
                                  start_source='controller_packet_arrival',start_evidence=self.start_evidence,
                                  completion=dict(mode=self.completion_mode, checkpoint=self.checkpoint,
                                                  tolerance_m=self.checkpoint_tolerance if self.checkpoint else None),
                                  finish_line=self.meta.get('finish_line') if self.completion_mode == 'finish_line' else None,
                                  feature_counts=dict(crosswalks=len(self.meta.get('crosswalks',[])),stop_line_candidates=len(self.map_stop_lines)),
                                  pending_criteria=[],
                                  provisional=['OBB collision','host packet arrival aligned to RDB time','position-relocation respawn detection'])
                self.cache_at=time.monotonic()
            payload = dict(self.cache)
            if self.status=='RUNNING' and self.last_received is not None and time.monotonic()-self.last_received>2:
                payload['status']='NO_DATA'
            return payload
