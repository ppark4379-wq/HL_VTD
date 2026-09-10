"""Passive live assessment: RDB frames -> evidence -> official deductions -> UI."""
from __future__ import annotations
import json
import math
import threading
import time
import uuid
import copy
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

TURN_SIGNAL_LANE_CONFIRM_FRAMES = 3


class LiveAssessment:
    def __init__(self, rules_path, output_dir, active_profile_path=None):
        self.rules_path = Path(rules_path)
        self.active_profile_path = Path(active_profile_path) if active_profile_path else None
        self._load_rules()
        self._configure_completion()
        self._configure_map()
        self._configure_school_zones()
        self._configure_route()
        self.active_profile = None
        self.active_profile_error = None
        self.active_scenario_id = self.meta.get('scenario_id', 'VTD_LIVE')
        self.output_dir = Path(output_dir)
        self.lock = threading.RLock()
        self.session = self.rules['session']
        self.respawn_detection_confirmed = bool(
            self.session.get('respawn_detection_confirmed', False)
        )
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
        if not isinstance(session.get('respawn_detection_confirmed', False), bool):
            raise ValueError('respawn_detection_confirmed must be boolean')

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

    def _configure_route(self, route_config=None):
        if route_config is None:
            route_config = self.rules.get('route') or {}
        route_path = route_config.get('csv')
        if route_path:
            route_path = Path(route_path)
            if not route_path.is_absolute():
                route_path = Path(__file__).resolve().parents[2] / route_path
        self.route_path = route_path
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

    def _activate_current_profile(self):
        """Select the route published by ``run_scenario`` before scoring starts."""
        self.active_profile = None
        self.active_profile_error = None
        self.active_scenario_id = self.meta.get('scenario_id', 'VTD_LIVE')
        if self.active_profile_path is None:
            return
        self._configure_route()
        try:
            profile = json.loads(self.active_profile_path.read_text(encoding='utf-8'))
            if not isinstance(profile, dict) or profile.get('schema_version') != 1:
                raise ValueError('invalid active assessment profile schema')
            if profile.get('status') != 'RUNNING':
                raise ValueError('active assessment profile is not RUNNING')
            process_id = profile.get('publisher_pid')
            if not isinstance(process_id, int) or process_id <= 0 or not Path(f'/proc/{process_id}').exists():
                raise ValueError('active assessment profile publisher is not running')
            created_ns = profile.get('created_wall_time_ns')
            if not isinstance(created_ns, int) or abs(time.time_ns() - created_ns) > 3600 * 1_000_000_000:
                raise ValueError('active assessment profile is stale')
            route_path = Path(profile.get('route_csv', '')).expanduser().resolve()
            if not route_path.is_file():
                raise ValueError(f'active route does not exist: {route_path}')
            route_config = {
                'csv': str(route_path),
                'section_count': profile.get('section_count', 5),
                'max_projection_distance_m': profile.get(
                    'max_projection_distance_m',
                    (self.rules.get('route') or {}).get('max_projection_distance_m', 20.0),
                ),
            }
            if profile.get('target_s_m') is not None:
                route_config['target_s_m'] = profile['target_s_m']
            self._configure_route(route_config)
            start_projection = None
            if self.previous and self.previous.get('ego'):
                ego = self.previous['ego']
                road = self.previous.get('road', {}).get(ego['id'])
                start_projection = self.route.project(ego, road, None)
            if start_projection:
                self.route_start_s = start_projection['s']
            elif profile.get('start_s_m') is not None:
                start_s = float(profile['start_s_m'])
                self.route_start_s = max(self.route.min_s, min(start_s, self.route_goal_s))
            if self.route_goal_s <= self.route_start_s:
                raise ValueError('active route start must precede target')
            self.active_profile = dict(profile)
            self.active_profile['resolved_route_csv'] = str(route_path)
            self.active_profile['resolved_start_s_m'] = self.route_start_s
            self.active_profile['resolved_target_s_m'] = self.route_goal_s
            self.active_scenario_id = str(profile.get('scenario_id') or 'VTD_LIVE')
        except FileNotFoundError:
            # Running the dashboard without run_scenario remains supported via
            # the explicitly configured default assessment route.
            return
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            self.active_profile_error = str(error)
            self._configure_route()

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
        self.respawn_candidates = defaultdict(int)
        self.turn_since = None
        self.indicator_observed_since = None
        self.turn = None
        self.pending_lane_change = None
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
            for key in ('wall_time_ns','monotonic_ns'):
                if self.last_control.get(key) is not None:
                    # JSON numbers above 2**53 lose precision in the browser.
                    # Store evidence nanoseconds as decimal strings in API/log
                    # payloads while the local monotonic freshness check keeps
                    # using the small floating-point ``at`` value.
                    self.last_control[key]=str(self.last_control[key])
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
        self._activate_current_profile()
        self.session_id = time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.file = (self.output_dir / (self.session_id+'.jsonl')).open('x',encoding='utf-8')
        self.started_at = now
        self.status = 'RUNNING'
        self.fresh_start = True
        self.pending_control = None
        self.cache = None
        effective_rules = copy.deepcopy(self.rules)
        effective_rules['route'] = dict(
            csv=str(self.route_path) if self.route_path else None,
            start_s_m=self.route_start_s,
            target_s_m=self.route_goal_s,
            section_count=self.route_section_count,
            max_projection_distance_m=self.route.max_distance_m if self.route else None,
        )
        self._write('session', {'session_id':self.session_id,'rules':effective_rules,'start_sim_time':now,
                               'start_source':'controller_packet_arrival','start_evidence':self.start_evidence,
                               'active_profile':self.active_profile,
                               'active_profile_error':self.active_profile_error})

    def _finish(self, status, reason=None):
        self.status = status
        self.cache = None
        self.snapshot()
        if reason:
            self.cache['finish_reason'] = reason
        self._write('result', self.cache)
        if self.file:
            self.file.close()
            self.file = None

    def finalize_incomplete(self, reason='manual_stop'):
        """Close a live, unfinished session without ever claiming completion."""
        with self.lock:
            if self.status != 'RUNNING':
                return False
            self._finish('ABORTED', reason)
            return True

    def close(self):
        with self.lock:
            if self.file:
                self._finish('ABORTED', 'dashboard_shutdown')

    def _reset(self):
        if self.session_id:
            self._finish(
                'RESET' if self.status == 'RUNNING' else self.status,
                'vtd_frame_or_time_reset' if self.status == 'RUNNING' else None,
            )
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
        self.respawn_candidates.clear()
        self.turn = self.turn_since = None
        self.indicator_observed_since = None
        self.pending_lane_change = None
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
                self.indicator_observed_since = None
                self.pending_lane_change = None
                self.lane_duration = 0
                self.previous_lane_active = False
            if continuous and not jump and not discontinuity and self.rows:
                self.distance += step
            row = self._measure(frame, ego, before if continuous and not jump and not discontinuity else None, dt if continuous and not jump and not discontinuity else 0)
            if jump:
                source_section = self.rows[-1]['evaluation_section_id'] if self.rows else row['evaluation_section_id']
                row['evaluation_section_id'] = source_section
                self.respawn_candidates[source_section] += 1
                row['respawn_candidate_count'] = self.respawn_candidates[source_section]
                if self.respawn_detection_confirmed:
                    self.respawns[source_section] += 1
                    row['respawn_count'] = self.respawns[source_section]
                row['goal_reached'] = False
                event_name = 'respawn' if self.respawn_detection_confirmed else 'respawn_candidate'
                self.active_events[event_name] = False
                self._event(event_name, row, True, 'vtd_position_relocation', {
                    'jump_m':step,
                    'officially_confirmed':self.respawn_detection_confirmed,
                })
            elif jump_limit is not None and self.respawn_detection_confirmed:
                row['respawn_count'] = self.respawns[row['evaluation_section_id']]
                self.active_events['respawn'] = False
            row['data_valid'] = 1
            if not continuous or jump or discontinuity:
                row['max_sample_gap_s'] = 1e-12
            self.rows.append({k: '' if v is None else str(int(v)) if isinstance(v,bool) else str(v) for k,v in row.items()})
            self._write('frame',row)
            self.cache = None
            if row['goal_reached']:
                self._finish('COMPLETED', 'final_checkpoint_reached')
            elif row['elapsed_wall_time_s'] >= self.session['time_limit_s']:
                self._finish('TIMEOUT', 'time_limit_reached')

    def _event(self, name, row, active, source, evidence=None):
        if active and not self.active_events.get(name,False):
            event = dict(event=name,sim_time_s=row['sim_time_s'],section=row['evaluation_section_id'],
                         x=row['ego_x'],y=row['ego_y'],source=source,evidence=evidence or {},
                         received_wall_time_ns=(
                             str(row['received_wall_time_ns'])
                             if row.get('received_wall_time_ns') is not None else None
                         ),
                         received_monotonic_ns=(
                             str(row['received_monotonic_ns'])
                             if row.get('received_monotonic_ns') is not None else None
                         ))
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
        road: Optional[RoadPosition],
        section: Optional[str],
        projection: Optional[Dict[str, Any]],
        route_s: Optional[float],
    ) -> Dict[str, Any]:
        key = section or 'unsegmented'
        row = dict(
            scenario_id=self.active_scenario_id,
            run_id=self.session_id,
            frame=frame['frame'],
            sim_time_s=frame['time'],
            received_wall_time_ns=frame.get('received_wall_time_ns'),
            received_monotonic_ns=frame.get('received_monotonic_ns'),
            elapsed_wall_time_s=frame['time']-self.started_at,
            ego_x=ego['x'],
            ego_y=ego['y'],
            ego_heading=ego['h'],
            ego_speed=ego['speed'],
            evaluation_section_id=key if section else '',
            distance_m=self.distance,
            current_speed_limit_kmh=self.rules['normal_speed_kmh'],
            posted_speed_limit_kmh=self.rules['normal_speed_kmh'],
            speed_limit_source='assessment_rule',
            goal_reached=False,
            stop_speed_kmh=self.session['stop_speed_kmh'],
            max_sample_gap_s=self.session['max_sample_gap_s'],
            nearest_object_distance=None,
            command_steering=None,
            target_accel=None,
            speed_source=ego.get('speed_source', 'RDB_OBJECT_STATE_EXT'),
            measurement_source='RDB+scenario_metadata',
        )
        if road:
            row.update(
                current_road_id=road['road'],
                current_lane_id=road['lane'],
                current_road_s_m=road.get('s'),
                current_lane_offset_m=road.get('offset'),
                current_road_heading=road.get('heading'),
                current_road_flags=road.get('flags'),
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
            # RDB ROAD_POS.heading is the lane-tangent-relative angle, not an
            # indication of forward/rearward OpenDRIVE road direction.  M15
            # legally enters positive lanes with heading ~= 0, so the old
            # cosine test marked a lane-centred Ego as a multi-metre intrusion.
            # viRDBIcd.h defines flag 0x01=DIR_FWD and 0x02=DIR_REAR.
            flags = int(road.get('flags', 0))
            forward = bool(flags & 0x01)
            rearward = bool(flags & 0x02)
            if forward != rearward:
                self.expected_side = -1 if forward else 1
                if self.map and road['road'] in self.map.roads and self.map.roads[road['road']].get('rule') == 'LHT':
                    self.expected_side *= -1
            else:
                # Missing/ambiguous direction metadata must not create a
                # whole-carriageway false positive. Anchor to the entry lane
                # and retain that side for the rest of this road.
                self.expected_side = 1 if road['lane'] > 0 else -1
        map_values = self.map.measure(ego, road, self.expected_side) if self.map and road and self.expected_side else {}
        for field in ('centerline_intrusion_m', 'sidewalk_intrusion_m'):
            if field not in row and field in map_values:
                row[field] = map_values[field]
        evaluate_opendrive_solid_lane_change(context, self.map, previous_road, lane_change)
        evaluate_solid_lane_change(context, self.meta, lane_change)
        solid_line_crossing = row.get('solid_line_crossing') if lane_change else None
        if lane_change:
            # A raw ROAD_POS lane-id edge is only a candidate. Publish the
            # solid-line result after the target lane remains stable for the
            # same confirmation window used by the turn-signal assessment.
            row['solid_line_crossing'] = None
        self._previous_road = road
        self._evaluate_turn_signal(
            context,
            lane,
            previous_road,
            lane_change,
            solid_line_crossing,
        )
        self._previous_lane_info = lane

    def _evaluate_turn_signal(
        self,
        context: AssessmentContext,
        lane: Optional[Dict[str, Any]],
        previous_road: Optional[RoadPosition],
        lane_change: bool,
        solid_line_crossing: Optional[bool],
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
        if not road:
            self.pending_lane_change = None
            return

        if indicator is not None:
            row['turn_signal_violation'] = False
        if lane_change:
            # A one-frame ROAD_POS lane-ID oscillation at a boundary is not an
            # auditable lane change. If it immediately returns to the source
            # lane, cancel the candidate; otherwise start a new candidate and
            # require a few consecutive target-lane observations.
            pending = self.pending_lane_change
            reverted = bool(
                pending
                and road['road'] == pending['road_id']
                and previous_road['lane'] == pending['to_lane_id']
                and road['lane'] == pending['from_lane_id']
            )
            if reverted:
                self.pending_lane_change = None
                return
            old_lane = frame['lanes'].get(
                (ego['id'], previous_road['road'], previous_road['lane'])
            ) or self._previous_lane_info
            direction = None
            direction_evidence = None
            if self.map:
                direction, direction_evidence = self.map.lane_change_direction(
                    road['road'],
                    road.get('s', 0.0),
                    previous_road['lane'],
                    road['lane'],
                    ego['h'],
                )
            if direction_evidence is None:
                direction_evidence = {'source': 'unavailable'}
            direction_evidence['rdb_left_lane_id'] = old_lane.get('left') if old_lane else None
            direction_evidence['rdb_right_lane_id'] = old_lane.get('right') if old_lane else None
            observation_history = (
                now-self.indicator_observed_since
                if self.indicator_observed_since is not None else None
            )
            self.pending_lane_change = dict(
                road_id=road['road'],
                from_lane_id=previous_road['lane'],
                to_lane_id=road['lane'],
                transition_sim_time_s=now,
                confirmations=1,
                expected=direction,
                direction_evidence=direction_evidence,
                observed=indicator,
                observation_history_s=observation_history,
                known_history=(
                    observation_history is not None
                    and observation_history >= 3-1e-9
                ),
                active_duration_s=(
                    now-self.turn_since if self.turn_since is not None else None
                ),
                solid_line_crossing=solid_line_crossing,
            )
        elif self.pending_lane_change:
            pending = self.pending_lane_change
            if (
                road['road'] != pending['road_id']
                or road['lane'] != pending['to_lane_id']
            ):
                self.pending_lane_change = None
                return
            pending['confirmations'] += 1

        pending = self.pending_lane_change
        if not pending or pending['confirmations'] < TURN_SIGNAL_LANE_CONFIRM_FRAMES:
            if pending and (
                pending['expected'] is None or not pending['known_history']
            ):
                row['turn_signal_violation'] = None
            return
        row.update(
            turn_signal_expected=pending['expected'],
            turn_signal_observed=pending['observed'],
            turn_signal_observation_history_s=pending['observation_history_s'],
            turn_signal_active_duration_s=pending['active_duration_s'],
            turn_signal_from_road_id=pending['road_id'],
            turn_signal_from_lane_id=pending['from_lane_id'],
            turn_signal_to_road_id=pending['road_id'],
            turn_signal_to_lane_id=pending['to_lane_id'],
            turn_signal_transition_sim_time_s=pending['transition_sim_time_s'],
            turn_signal_confirmation_frames=pending['confirmations'],
            turn_signal_direction_source=pending['direction_evidence'].get('source'),
            turn_signal_direction_signed_lateral_m=pending['direction_evidence'].get('signed_lateral_m'),
            turn_signal_rdb_left_lane_id=pending['direction_evidence'].get('rdb_left_lane_id'),
            turn_signal_rdb_right_lane_id=pending['direction_evidence'].get('rdb_right_lane_id'),
        )
        row['solid_line_crossing'] = pending['solid_line_crossing']
        row['turn_signal_violation'] = (
            None
            if pending['expected'] is None or not pending['known_history']
            else pending['observed'] != pending['expected']
            or pending['active_duration_s'] is None
            or pending['active_duration_s'] < 3-1e-9
        )
        self.pending_lane_change = None

    def _record_measurement_events(self, row: Dict[str, Any], ego: EgoState) -> None:
        for field in ('solid_line_crossing', 'turn_signal_violation', 'pedestrian_violation', 'static_collision', 'road_user_collision'):
            if field in row:
                evidence = None
                if field in ('static_collision', 'road_user_collision'):
                    evidence = {
                        'object_ids': row.get(f'{field}_ids', ''),
                        'object_names': row.get(f'{field}_names', ''),
                        'object_types': row.get(f'{field}_types', ''),
                    }
                elif field == 'turn_signal_violation':
                    evidence = {
                        'expected': row.get('turn_signal_expected'),
                        'observed': row.get('turn_signal_observed'),
                        'observation_history_s': row.get('turn_signal_observation_history_s'),
                        'active_duration_s': row.get('turn_signal_active_duration_s'),
                        'from_road_id': row.get('turn_signal_from_road_id'),
                        'from_lane_id': row.get('turn_signal_from_lane_id'),
                        'to_road_id': row.get('turn_signal_to_road_id'),
                        'to_lane_id': row.get('turn_signal_to_lane_id'),
                        'transition_sim_time_s': row.get('turn_signal_transition_sim_time_s'),
                        'confirmation_frames': row.get('turn_signal_confirmation_frames'),
                        'direction_source': row.get('turn_signal_direction_source'),
                        'direction_signed_lateral_m': row.get('turn_signal_direction_signed_lateral_m'),
                        'rdb_left_lane_id': row.get('turn_signal_rdb_left_lane_id'),
                        'rdb_right_lane_id': row.get('turn_signal_rdb_right_lane_id'),
                    }
                self._event(field, row, row[field] is True, 'RDB+OBB+metadata', evidence)
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
        row = self._new_assessment_row(frame, ego, road, section, projection, route_s)
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
                        6:'respawn_count',7:'turn_signal_violation',8:'solid_line_crossing',
                        9:'traffic_light_state',10:'justified_stop',11:'traffic_light_state',
                        12:'crosswalk_active',13:'static_collision',14:'pedestrian_violation',
                        15:'road_user_collision'}
                section_rows=defaultdict(list)
                for record in self.rows: section_rows[record['evaluation_section_id'] or 'unsegmented'].append(record)
                for item in grid:
                    field=fields[item['number']]
                    item['observed_sections']=[key for key,records in section_rows.items() if any(r.get(field,'')!='' for r in records)]
                    item['partial_sections']=[key for key,records in section_rows.items() if any(r.get(field,'')=='' for r in records)]
                school_uncertain = bool(
                    self.school_zones
                    and any(
                        record.get('school_zone_coverage_complete') != '1'
                        for record in self.rows
                    )
                )
                if self.school_zones and self.school_zones.pending:
                    for item in grid:
                        if item['number']==2:
                            scope = '주행 위치에 영향 가능' if school_uncertain else '주행 위치 밖'
                            item['detail'] += f"; 보호구역 {len(self.school_zones.accepted)}개 반영 / {len(self.school_zones.pending)}개 경계 확인 대기({scope})"
                reached = list(dict.fromkeys(r['evaluation_section_id'] for r in self.rows if r['evaluation_section_id']))
                # A violation with no route section is useful review evidence,
                # but cannot be an official deduction under the per-section
                # rules. Only sectioned penalties enter the score calculation.
                total_penalty = -sum(
                    points
                    for result in results
                    for section, points in result.penalty_by_section.items()
                    if section != 'unsegmented'
                )
                evidence_penalty = sum(r.penalty for r in results)
                unsegmented = any(not r['evaluation_section_id'] for r in self.rows)
                route_projection_count = sum(bool(r.get('route_s_m', '').strip()) for r in self.rows)
                route_projection_miss_count = (
                    len(self.rows) - route_projection_count if self.route else 0
                )
                elapsed = float(self.rows[-1]['elapsed_wall_time_s']) if self.rows else 0.0
                route_mismatch = bool(
                    self.route
                    and elapsed >= max(2.0, self.session['max_sample_gap_s'] * 5)
                    and route_projection_count == 0
                )
                # A score split into the wrong route's sections is actively
                # misleading.  Keep individual evidence available for review,
                # but withhold the aggregate until the configured route has at
                # least one valid projection.
                for item in results:
                    for section,points in item.penalty_by_section.items():
                        if section == 'unsegmented':
                            continue
                        key = (section,item.number)
                        delta = points-self.deducted.get(key,0)
                        if delta>0:
                            event = dict(item=item.number,section=section,delta=delta,total=points,
                                         sim_time_s=self.rows[-1]['sim_time_s'],source='DOCX+organizer_answers',detail=item.detail,
                                         received_wall_time_ns=self.rows[-1].get('received_wall_time_ns'),
                                         received_monotonic_ns=self.rows[-1].get('received_monotonic_ns'))
                            self.deductions.append(event)
                            self._write('deduction',event)
                            self.deducted[key]=points
                major = sum(points//6 if item.number==6 else int(item.severity_by_section.get(section)==2)
                            for item in results for section,points in item.penalty_by_section.items()
                            if section != 'unsegmented')
                unavailable = [r.number for r in results if r.status=='UNAVAILABLE']
                school_partial=school_uncertain
                missing=[key for key in ('school_zones','crosswalks','stop_lines','centerlines','sidewalk_boundaries') if key not in self.meta and not (key=='school_zones' and self.school_zones) and not (key in ('centerlines','sidewalk_boundaries') and self.map)]
                if not self.route and 'sections' not in self.meta: missing.append('sections')
                if self.completion_mode == 'finish_line' and 'finish_line' not in self.meta: missing.append('finish_line')
                if not self.meta.get('object_feed_complete',False): missing.append('object_feed_complete')
                if not self.respawn_detection_confirmed: missing.append('confirmed_respawn_event_source')
                warnings=[]
                if missing: warnings.append('필수 데이터 확인 필요: '+', '.join(missing))
                if self.active_profile_error:
                    warnings.append('실행 경로 프로필 적용 실패: '+self.active_profile_error)
                if route_mismatch:
                    warnings.append(
                        '평가 경로 불일치: 시작 후 경로 투영이 한 번도 성공하지 않았습니다. '
                        '현재 시나리오용 --assessment-config를 확인하세요'
                    )
                if school_partial: warnings.append(f'보호구역 {len(self.school_zones.accepted)}개 반영 / 현재 주행 위치가 {len(self.school_zones.pending)}개 보류 구역의 불확실 범위와 겹침')
                if self.rows and self.rows[-1].get('rear_axle_available')=='0':
                    affected = '종료선·보호구역·횡단보도' if self.completion_mode == 'finish_line' else '보호구역·횡단보도'
                    warnings.append('차량 후륜축 위치 미확정: '+affected+' 판정 불가')
                partial_fields = any(item['partial_sections'] for item in grid)
                score_withheld_reasons=[]
                if route_mismatch: score_withheld_reasons.append('route_mismatch')
                elif self.route and route_projection_count == 0: score_withheld_reasons.append('route_not_yet_projected')
                if route_projection_miss_count: score_withheld_reasons.append('route_projection_gaps')
                if unavailable: score_withheld_reasons.append('unavailable_items')
                if unsegmented: score_withheld_reasons.append('unsegmented_frames')
                if school_partial: score_withheld_reasons.append('school_zone_coverage_pending')
                if missing: score_withheld_reasons.append('missing_metadata')
                if partial_fields: score_withheld_reasons.append('partial_measurements')
                complete = bool(self.rows) and not score_withheld_reasons
                score = self.rules['base_score']+total_penalty if complete else None
                evaluation = (
                    'WAITING' if not self.rows
                    else 'CONFIG_MISMATCH' if route_mismatch
                    else 'COMPLETE' if complete
                    else 'PARTIAL'
                )
                section_boundaries=[]
                if self.route:
                    span=(self.route_goal_s-self.route_start_s)/self.route_section_count
                    for index in range(self.route_section_count):
                        section_boundaries.append(dict(
                            id=str(index+1),
                            start_s_m=self.route_start_s+span*index,
                            end_s_m=self.route_start_s+span*(index+1),
                        ))
                self.cache = dict(session_id=self.session_id,status=self.status,
                                  unsegmented_frames=unsegmented,
                                  sections=self.section_ids or [str(s['id']) for s in self.meta.get('sections',[])],reached_sections=reached,
                                  official_grid=grid,summary=dict(official_score=score,official_penalty=total_penalty,
                                  evidence_only_score=self.rules['base_score']+evidence_penalty,
                                  official_evaluation=evaluation,
                                  official_unavailable_items=unavailable,major_count=major,
                                  score_withheld_reasons=score_withheld_reasons),
                                  latest=self.rows[-1] if self.rows else {},time_limit_s=self.session['time_limit_s'],
                                  latest_received_wall_time_ns=self.rows[-1].get('received_wall_time_ns') if self.rows else None,
                                  latest_received_monotonic_ns=self.rows[-1].get('received_monotonic_ns') if self.rows else None,
                                  events=self.events[-100:],deductions=self.deductions[-100:],
                                  school_zone_validation=self.school_zones.report() if self.school_zones else None,
                                  missing_metadata=missing,warnings=warnings,finish_candidates=self.finish_candidates,
                                  start_source='controller_packet_arrival',start_evidence=self.start_evidence,
                                  completion=dict(mode=self.completion_mode, checkpoint=self.checkpoint,
                                                  tolerance_m=self.checkpoint_tolerance if self.checkpoint else None),
                                  finish_line=self.meta.get('finish_line') if self.completion_mode == 'finish_line' else None,
                                  feature_counts=dict(crosswalks=len(self.meta.get('crosswalks',[])),stop_line_candidates=len(self.map_stop_lines)),
                                  assessment_profile=dict(
                                      rules_path=str(self.rules_path),
                                      route_csv=str(self.route_path) if self.route_path else None,
                                      route_projection_count=route_projection_count,
                                      route_projection_miss_count=route_projection_miss_count,
                                      route_mismatch=route_mismatch,
                                      source='run_scenario' if self.active_profile else 'assessment_config',
                                      scenario_id=self.active_scenario_id,
                                      active_profile_error=self.active_profile_error,
                                      section_basis='configured_route_distance_equal_fifths',
                                      section_boundaries=section_boundaries,
                                  ),
                                  pending_criteria=[],
                                  provisional=['OBB collision','host packet arrival aligned to RDB time','position-relocation respawn detection'])
                self.cache_at=time.monotonic()
            payload = dict(self.cache)
            if self.status=='RUNNING' and self.last_received is not None and time.monotonic()-self.last_received>2:
                payload['status']='NO_DATA'
            return payload
