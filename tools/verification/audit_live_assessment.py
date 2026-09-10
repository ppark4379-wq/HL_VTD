#!/usr/bin/env python3
"""Audit recorded observations against assessment output; never drive VTD.

Raw means the recorder's decoded RDB frames, NOT independent ground truth.
Numeric comparisons independently check transport/display correctness. Lane
links are read directly from XODR rather than calling the scoring evaluator.
Other per-item results are explicitly reported as claims, not re-certified.
"""
from __future__ import annotations

import argparse
import bisect
from collections import Counter, defaultdict
import csv
import gzip
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

FIELDS = {
    1: ('제한속도', 'ego_speed'), 2: ('보호구역 속도', 'school_zone_active'),
    4: ('중앙선 침범', 'centerline_intrusion_m'), 5: ('보도 침범', 'sidewalk_intrusion_m'),
    6: ('리스폰', 'respawn_count'), 7: ('차로변경 방향지시등', 'turn_signal_violation'),
    8: ('실선 차로변경', 'solid_line_crossing'), 9: ('적색신호 정지', 'traffic_light_state'),
    10: ('녹색신호 출발', 'justified_stop'), 11: ('적색점멸 정지', 'traffic_light_state'),
    12: ('횡단보도 정차', 'crosswalk_active'), 13: ('정적 장애물 충돌', 'static_collision'),
    14: ('보행자 대응', 'pedestrian_violation'), 15: ('도로이용자 충돌', 'road_user_collision'),
}
SCORING_INPUTS = {
    1: ('ego_speed','posted_speed_limit_kmh','speed_limit_source'),
    2: ('ego_speed','school_zone_active','school_zone_coverage_complete'),
    4: ('centerline_intrusion_m','sim_time_s','max_sample_gap_s','data_valid'),
    5: ('sidewalk_intrusion_m','sim_time_s','max_sample_gap_s','data_valid'),
    6: ('respawn_count',), 7: ('turn_signal_violation',), 8: ('solid_line_crossing',),
    9: ('traffic_light_state','stop_line_id','stop_line_distance_m','ego_speed','stop_speed_kmh','sim_time_s','max_sample_gap_s','data_valid'),
    10: ('traffic_light_state','stop_line_id','stop_line_distance_m','ego_speed','stop_speed_kmh','justified_stop','sim_time_s','max_sample_gap_s','data_valid'),
    11: ('traffic_light_state','stop_line_id','stop_line_distance_m','ego_speed','stop_speed_kmh','sim_time_s','max_sample_gap_s','data_valid'),
    12: ('crosswalk_active','crosswalk_source','ego_speed','rear_axle_available','sim_time_s','max_sample_gap_s','data_valid'),
    13: ('static_collision','collision_source'), 14: ('pedestrian_violation',),
    15: ('road_user_collision','collision_source'),
}


def read_jsonl(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        for line_no, line in enumerate(stream, 1):
            if line.strip():
                try:
                    yield json.loads(line)
                except json.JSONDecodeError as error:
                    raise ValueError(f'{path}:{line_no}: malformed JSON') from error


def num(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def key(row):
    return int(row['frame']), round(float(row.get('sim_time_s', row.get('time'))), 7)


def value_present(value):
    return value is not None and value != ''


def stats(values):
    values = [v for v in values if v is not None]
    return dict(count=len(values), min=min(values, default=None), max=max(values, default=None))


def scenario_entities(path):
    """Preserve exact XML names for all explicitly declared entity kinds."""
    root = ET.parse(path).getroot()
    entities = []
    for kind, query in (('Player','.//Player/Description'),('Character','.//Character'),('Object','.//Object')):
        for node in root.findall(query):
            name = node.get('Name')
            if name is not None:
                entities.append(dict(kind=kind,name=name,attributes=dict(node.attrib)))
    return entities


class LaneTopology:
    """An independent laneSection-link reader; absent links remain unknown."""
    def __init__(self, path):
        self.roads = {int(r.get('id')): r for r in ET.parse(path).getroot().findall('road')}

    def section(self, road_id, s):
        road = self.roads.get(int(road_id))
        if road is None:
            return None, []
        sections = sorted(road.findall('lanes/laneSection'), key=lambda n: float(n.get('s')))
        index = bisect.bisect_right([float(n.get('s')) for n in sections], s + 1e-7) - 1
        return (index if index >= 0 else None), sections

    @staticmethod
    def lane(section, lane_id):
        return next((n for n in section.findall('./*/lane') if int(n.get('id')) == lane_id), None)

    def classify(self, before, after):
        if before['road'] != after['road']:
            return dict(classification='ROAD_CHANGE_NOT_AUDITED')
        old_index, sections = self.section(before['road'], before['s'])
        new_index, _ = self.section(after['road'], after['s'])
        if old_index is None or new_index is None:
            return dict(classification='UNKNOWN_SECTION')
        result = dict(from_section=old_index, to_section=new_index,
                      from_lane=before['lane'], to_lane=after['lane'], links=[])
        if old_index == new_index:
            result['classification'] = 'SAME_LANE' if before['lane'] == after['lane'] else 'LANE_CHANGE_CANDIDATE'
            return result
        step = 1 if new_index > old_index else -1
        direction = 'successor' if step == 1 else 'predecessor'
        lane_id = before['lane']
        for index in range(old_index, new_index, step):
            lane = self.lane(sections[index], lane_id)
            link = lane.find(f'link/{direction}') if lane is not None else None
            if link is None:
                result.update(classification='UNKNOWN_MISSING_LANE_LINK', mapped_lane=lane_id)
                return result
            linked = int(link.get('id'))
            result['links'].append(dict(section=index, lane=lane_id, relation=direction, linked_lane=linked))
            lane_id = linked
        result['mapped_lane'] = lane_id
        result['classification'] = 'LANE_CONTINUATION_RENUMBER' if lane_id == after['lane'] else 'LANE_CHANGE_CANDIDATE'
        return result

    @staticmethod
    def polynomial(node, p, suffix=''):
        return sum(float(node.get(coefficient+suffix,0))*p**power for power,coefficient in enumerate('abcd'))

    def lateral_center(self, road_id, s, lane_id):
        index, sections = self.section(road_id,s)
        if index is None or not lane_id:
            return None
        section = sections[index]
        ds = s-float(section.get('s'))
        road = self.roads[road_id]
        offsets = [n for n in road.findall('lanes/laneOffset') if float(n.get('s')) <= s+1e-7]
        offset = max(offsets,key=lambda n:float(n.get('s'))) if offsets else None
        edge = self.polynomial(offset,s-float(offset.get('s'))) if offset is not None else 0.0
        side, sign = ('left',1) if lane_id>0 else ('right',-1)
        for lane in sorted(section.findall(side+'/lane'),key=lambda n:abs(int(n.get('id')))):
            widths = [n for n in lane.findall('width') if float(n.get('sOffset')) <= ds+1e-7]
            if not widths:
                return None
            node = max(widths,key=lambda n:float(n.get('sOffset')))
            width = self.polynomial(node,ds-float(node.get('sOffset')))
            if width < 0:
                return None
            if int(lane.get('id')) == lane_id:
                return edge+sign*width/2
            edge += sign*width
        return None

    def reference_heading(self, road_id, s):
        geometries = [n for n in self.roads[road_id].findall('planView/geometry') if float(n.get('s')) <= s+1e-7]
        if not geometries:
            return None
        geometry = max(geometries,key=lambda n:float(n.get('s')))
        ds, base, length = s-float(geometry.get('s')), float(geometry.get('hdg')), float(geometry.get('length'))
        curve = next(iter(geometry),None)
        if curve is None or ds > length+.05:
            return None
        if curve.tag == 'line':
            return base
        if curve.tag == 'arc':
            return base+float(curve.get('curvature'))*ds
        if curve.tag == 'spiral':
            start,end = float(curve.get('curvStart')),float(curve.get('curvEnd'))
            return base+start*ds+(end-start)*ds*ds/(2*length)
        if curve.tag not in ('poly3','paramPoly3'):
            return None

        def derivative(p,suffix):
            return sum(power*float(curve.get(c+suffix,0))*p**(power-1) for power,c in enumerate('abcd') if power)

        def tangent(p):
            return (1.0,derivative(p,'')) if curve.tag=='poly3' else (derivative(p,'U'),derivative(p,'V'))

        def arc_length(p):
            # Independent Simpson quadrature and inverse arc length, unlike
            # the evaluator's tessellated reference-line interpolation.
            count=64
            values=[math.hypot(*tangent(p*i/count)) for i in range(count+1)]
            return p/(3*count)*(values[0]+values[-1]+4*sum(values[1:-1:2])+2*sum(values[2:-1:2]))

        high = ds if curve.tag=='poly3' else (1.0 if curve.get('pRange')=='normalized' else length)
        if arc_length(high)+.05 < ds:
            return None
        low=0.0
        for _ in range(30):
            middle=(low+high)/2
            if arc_length(middle)<ds:
                low=middle
            else:
                high=middle
        du,dv=tangent((low+high)/2)
        return base+math.atan2(dv,du) if math.hypot(du,dv)>1e-10 else None

    def direction(self,road_id,s,old_lane,new_lane,ego_heading):
        old = self.lateral_center(road_id,s,old_lane)
        new = self.lateral_center(road_id,s,new_lane)
        heading = self.reference_heading(road_id,s)
        if old is None or new is None or heading is None:
            return dict(expected=None,reason='unsupported_or_missing_geometry')
        signed = (new-old)*math.cos(heading-ego_heading)
        expected = 'LEFT' if signed>.25 else 'RIGHT' if signed<-.25 else None
        return dict(expected=expected,signed_lateral_m=signed,old_lane_center_t=old,new_lane_center_t=new,
            reference_heading=heading,ego_heading=ego_heading,
            source='independent_XODR_width_polynomial_and_reference_tangent')


def indicator_history(raw, at, expected, max_gap):
    """Require uninterrupted known samples for the preceding three sim seconds."""
    now = raw[at]['time']
    current = raw[at].get('indicator')
    result = dict(observed=current, expected=expected, continuously_correct_s=0.0,
                  preceding_history_s=0.0, sufficient_history=False, verdict='UNKNOWN')
    previous = now
    continuous_start = now
    correct = current == expected and expected in ('LEFT', 'RIGHT')
    for index in range(at - 1, -1, -1):
        sample = raw[index]
        gap = previous - sample['time']
        if gap <= 0 or gap > max_gap or sample.get('indicator') is None:
            break
        result['preceding_history_s'] = now - sample['time']
        if correct and sample['indicator'] == expected:
            continuous_start = sample['time']
        else:
            correct = False
        previous = sample['time']
        if now - sample['time'] >= 3.0 - 1e-7:
            result['sufficient_history'] = True
            break
    result['continuously_correct_s'] = now - continuous_start
    if expected in ('LEFT', 'RIGHT') and current is not None:
        if result['continuously_correct_s'] >= 3.0 - 1e-7:
            result['verdict'] = 'CORRECT_INDICATOR_OBSERVED_FOR_3S'
        elif result['sufficient_history']:
            result['verdict'] = 'INDICATOR_VIOLATION_CANDIDATE'
    return result


def lane_audit(raw, xodr, max_gap):
    topology = LaneTopology(xodr)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'dashboard'))
    from opendrive_assessment import OpenDriveMap
    shared_map = OpenDriveMap(xodr)
    findings = []
    for i in range(1, len(raw)):
        before, after = raw[i - 1], raw[i]
        a, b = before.get('road'), after.get('road')
        if not a or not b or (a['road'], a['lane']) == (b['road'], b['lane']):
            continue
        finding = dict(frame=after['frame'], sim_time_s=after['time'],
                       x=after['ego']['x'], y=after['ego']['y'],
                       from_road=a['road'], to_road=b['road'], **topology.classify(a, b))
        finding.setdefault('from_lane',a['lane'])
        finding.setdefault('to_lane',b['lane'])
        finding['continuous_samples'] = 0 < after['time'] - before['time'] <= max_gap
        tail = raw[i:i + 3]
        finding['stable_next_3_frames'] = len(tail) == 3 and all(
            sample.get('road') and (sample['road']['road'], sample['road']['lane']) == (b['road'], b['lane'])
            and (j == 0 or 0 < sample['time'] - tail[j - 1]['time'] <= max_gap)
            for j, sample in enumerate(tail))
        if a['road'] == b['road'] and finding['classification'] == 'LANE_CHANGE_CANDIDATE':
            mapped_old = finding.get('mapped_lane',a['lane'])
            independent = topology.direction(b['road'],b['s'],mapped_old,b['lane'],after['ego']['h'])
            finding['independent_direction'] = independent
            expected, geometry = shared_map.lane_change_direction(b['road'], b['s'], mapped_old, b['lane'], after['ego']['h'])
            finding['secondary_direction'] = dict(expected=expected, evidence=geometry,
                limitation='Shared production geometry helper: corroboration only, not independent proof')
            finding['raw_indicator_audit'] = indicator_history(raw, i, independent['expected'], max_gap)
        findings.append(finding)
    return findings


def run_audit(args):
    records = list(read_jsonl(args.assessment))
    sessions = [r for r in records if r.get('kind') == 'session']
    rows = [r for r in records if r.get('kind') == 'frame']
    if len(sessions) != 1 or not rows:
        raise ValueError('Expected exactly one assessment session with frame rows')
    session = sessions[0]
    session['start_sim_time'] = float(session['start_sim_time'])
    # Frame JSONL currently writes native numbers, while API snapshots expose
    # the same fields as strings. Accept both representations explicitly.
    for row in rows:
        for field in ('frame','sim_time_s','ego_x','ego_y','ego_heading','ego_speed',
                      'elapsed_wall_time_s','distance_m','turn_signal_transition_sim_time_s'):
            if value_present(row.get(field)):
                row[field] = num(row[field])
        if any(row.get(field) is None for field in ('frame','sim_time_s','ego_x','ego_y','ego_speed','elapsed_wall_time_s','distance_m')):
            raise ValueError('Assessment frame lacks finite position/speed/time/distance fields')
    final = next((r for r in reversed(records) if r.get('kind') == 'result'), {})
    rules = session.get('rules', {})
    max_gap = float(rules.get('session', {}).get('max_sample_gap_s', .5))
    declared_entities = scenario_entities(args.scenario)
    expected = {entity['name'] for entity in declared_entities}
    expected_without_ego = {n for n in expected if n.lower() != args.ego_name.lower()}
    wanted = {key(row): row for row in rows}
    if len(wanted) != len(rows):
        raise ValueError('Assessment repeats a frame/time key: split or disambiguate sessions before auditing')
    first_wall, last_wall = (num(rows[n].get('received_monotonic_ns')) for n in (0, -1))
    matched, raw = {}, []
    names, name_counts, packages, signals = set(), Counter(), Counter(), Counter()
    missing_counts, raw_count, raw_no_ego, frame_gaps, last_frame = Counter(), 0, 0, [], None
    match_candidates, ambiguous_keys, raw_fields = Counter(), set(), Counter()
    for frame in read_jsonl(args.frames):
        clock = num(frame.get('received_monotonic_ns'))
        if first_wall is not None and last_wall is not None and clock is not None:
            if clock < first_wall - 4e9 or clock > last_wall + 1e8:
                continue
        elif not session['start_sim_time'] - 4 <= frame['time'] <= rows[-1]['sim_time_s'] + .1:
            continue
        raw_count += 1
        frame_names = {o['name'] for o in frame.get('objects', {}).values() if o.get('name')}
        names.update(frame_names)
        name_counts.update(frame_names)
        missing_counts.update(expected - frame_names)
        packages.update(frame.get('packages', []))
        signals.update((str(ident), value.get('mask')) for ident, value in frame.get('signals', {}).items())
        ego = next((o for o in frame.get('objects', {}).values() if o.get('name', '').lower() == args.ego_name.lower()), None)
        if not ego:
            raw_no_ego += 1
            continue
        ident = str(ego['id'])
        compact = dict(frame=frame['frame'], time=frame['time'], ego=ego,
            road=frame.get('road', {}).get(ident), indicator=frame.get('lights', {}).get(ident),
            received_monotonic_ns=clock, received_wall_time_ns=frame.get('received_wall_time_ns'))
        raw_fields.update('ego.'+field for field,value in ego.items() if value_present(value))
        raw_fields.update('road.'+field for field,value in (compact['road'] or {}).items() if value_present(value))
        if compact['indicator'] is not None:
            raw_fields['indicator'] += 1
        if frame.get('signals'):
            raw_fields['signal_package_nonempty'] += 1
        raw.append(compact)
        if last_frame and frame['frame'] > last_frame['frame'] + 1:
            frame_gaps.append(dict(from_frame=last_frame['frame'], to_frame=frame['frame'], sim_gap_s=frame['time']-last_frame['time']))
        last_frame = compact
        k = key(frame)
        if k in wanted:
            target_clock = num(wanted[k].get('received_monotonic_ns'))
            if target_clock is not None and clock is not None and target_clock != clock:
                continue
            match_candidates[k] += 1
            if match_candidates[k] > 1:
                ambiguous_keys.add(k)
                matched.pop(k,None)
            elif k not in ambiguous_keys:
                matched[k]=compact
    pairs = [(row, matched[key(row)]) for row in rows if key(row) in matched]
    speed_errors = [abs(row['ego_speed']-r['ego']['speed']) for row,r in pairs if num(r['ego'].get('speed')) is not None]
    position_errors = [math.hypot(row['ego_x']-r['ego']['x'], row['ego_y']-r['ego']['y']) for row,r in pairs]
    elapsed_errors = [abs(row['elapsed_wall_time_s']-(r['time']-session['start_sim_time'])) for row,r in pairs]
    distance = 0.0
    distance_errors = []
    discontinuities = []
    previous = None
    for row, r in pairs:
        if previous:
            old_row, old = previous
            dt = r['time'] - old['time']
            step = math.hypot(r['ego']['x']-old['ego']['x'], r['ego']['y']-old['ego']['y'])
            if 0 < dt <= max_gap and step < (num(rules.get('session',{}).get('respawn_jump_m')) or 5):
                distance += step
            else:
                discontinuities.append(dict(frame=r['frame'], dt=dt, step_m=step))
        distance_errors.append(abs(row['distance_m']-distance))
        previous = (row, r)
    transitions = lane_audit(raw, args.xodr, max_gap)
    grid = {item['number']: item for item in final.get('official_grid', [])}
    items = []
    for number, (name, field) in FIELDS.items():
        measured = [r for r in rows if value_present(r.get(field))]
        values = [num(r[field]) for r in measured]
        claim = grid.get(number, {})
        input_fields = ('evaluation_section_id',)+SCORING_INPUTS[number]
        input_coverage = {name:dict(present=sum(value_present(r.get(name)) for r in rows),
            missing=sum(not value_present(r.get(name)) for r in rows)) for name in input_fields}
        items.append(dict(number=number, name=name, field=field, measured_frames=len(measured),
            missing_frames=len(rows)-len(measured), numeric_range=stats(values),
            positive_frames=sum(v is not None and v > 0 for v in values),
            claimed_status=claim.get('status','NO_FINAL_RESULT'), claimed_penalty=claim.get('penalty'),
            claimed_detail=claim.get('detail'), claimed_sections=claim.get('penalty_by_section', {}),
            scoring_input_coverage=input_coverage,
            all_scoring_inputs_present_frames=sum(all(value_present(r.get(name)) for name in input_fields) for r in rows),
            audit_status='MEASUREMENT_COVERAGE_ONLY' if measured else 'UNAVAILABLE',
            note='Evaluator claim; absence of a recorded violation does not certify detection accuracy'))
    speed_limit = float(rules.get('normal_speed_kmh',50))
    independent_speed_severity = {}
    for row, r in pairs:
        speed = num(r['ego'].get('speed'))
        if speed is None:
            continue
        excess = speed*3.6 - speed_limit
        level = 2 if excess > 20 else 1 if excess > 1 else 0
        section = str(row.get('evaluation_section_id') or 'unsegmented')
        if level:
            independent_speed_severity[section] = max(level,independent_speed_severity.get(section,0))
    independent_speed_penalty = -sum(6 if level == 2 else 3 for level in independent_speed_severity.values())
    items[0].update(audit_status='RAW_SPEED_THRESHOLD_CHECKED' if len(speed_errors)==len(rows) else 'PARTIAL_RAW_SPEED',
        independent_penalty=independent_speed_penalty, independent_severity_by_section=independent_speed_severity,
        note='Recalculated from raw speed; section placement and configured regulatory limit remain shared inputs')
    reported_changes = [r for r in rows if value_present(r.get('turn_signal_expected'))]
    for transition in transitions:
        related = [r for r in reported_changes if abs(float(r.get('turn_signal_transition_sim_time_s', -999))-transition['sim_time_s']) < .001]
        transition['assessment_matches'] = [{k:v for k,v in r.items() if k.startswith('turn_signal_') or k in ('frame','sim_time_s','solid_line_crossing')} for r in related]
        if transition['classification'] == 'LANE_CONTINUATION_RENUMBER' and any(r.get('turn_signal_violation') or r.get('solid_line_crossing') for r in related):
            transition['finding'] = 'FALSE_POSITIVE_LANE_RENUMBERING'
    controller = dict(available=False)
    if args.controller:
        with args.controller.open(newline='',encoding='utf-8-sig') as stream:
            controller_rows = list(csv.DictReader(stream))
        controller = dict(available=True, frames=len(controller_rows), fields=list(controller_rows[0]) if controller_rows else [],
            speed_mps=stats([num(r.get('ego_speed')) for r in controller_rows]),
            final=controller_rows[-1] if controller_rows else None,
            note='Controller frame is not assumed identical to RDB frame. No unsafe index-only matching.')
    manifest_path = args.frames.parent/'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else None
    report = dict(schema='hl_vtd_live_assessment_audit/v1',
        audit_status='NO_MATCHING_RAW_EVIDENCE' if not pairs else 'PARTIAL_EVIDENCE' if len(pairs)!=len(rows) or not final else 'EVIDENCE_COMPARED_NOT_FULL_SCORING_CERTIFICATION',
        sources={k:str(getattr(args,k)) if getattr(args,k) else None for k in ('assessment','frames','scenario','xodr','controller')},
        assessment_session=session, final_summary=final.get('summary'), final_status=final.get('status'),
        final_missing_metadata=final.get('missing_metadata'), assessment_frames=len(rows),
        raw_recording_manifest=manifest,
        raw_observation=dict(frames=raw_count, frames_without_ego=raw_no_ego, gaps=frame_gaps,
            package_counts=dict(packages), available_field_counts=dict(raw_fields),
            signal_mask_counts=[dict(id=i,mask=m,count=n) for (i,m),n in signals.items()]),
        transport=dict(matched_frames=len(pairs), missing_matches=len(rows)-len(pairs),
            ambiguous_frame_time_keys=[list(k) for k in sorted(ambiguous_keys)],
            match_method='frame_number + simulation_time + exact_monotonic_timestamp_when_present; ambiguous duplicates rejected',
            speed_error_mps=stats(speed_errors), position_error_m=stats(position_errors), elapsed_error_s=stats(elapsed_errors),
            independently_summed_distance_m=distance, final_dashboard_distance_m=rows[-1].get('distance_m'),
            distance_error_m=stats(distance_errors), discontinuities=discontinuities,
            limitation='Same Decoder feed validates propagation, not wire-decoder correctness or simulator ground truth'),
        route=dict(config=rules.get('route'), projected_frames=sum(value_present(r.get('route_s_m')) for r in rows),
            unsegmented_frames=sum(not r.get('evaluation_section_id') for r in rows),
            sections=dict(Counter(str(r.get('evaluation_section_id') or 'unsegmented') for r in rows)),
            projection_distance_m=stats([num(r.get('route_distance_m')) for r in rows]),
            route_s_m=stats([num(r.get('route_s_m')) for r in rows]),
            elapsed_s=stats([num(r.get('elapsed_wall_time_s')) for r in rows]),
            speed_mps=stats([num(r.get('ego_speed')) for r in rows])),
        entities=dict(expected=sorted(expected), observed=sorted(names), missing_entire_capture=sorted(expected-names),
            declared=declared_entities,
            declared_kind_counts=dict(Counter(e['kind'] for e in declared_entities)),
            missing_by_kind={kind:sorted(e['name'] for e in declared_entities if e['kind']==kind and e['name'] not in names)
                for kind in ('Player','Character','Object')},
            duplicate_declared_names={name:count for name,count in Counter(e['name'] for e in declared_entities).items() if count>1},
            unexpected=sorted(names-expected), missing_frame_counts=dict(missing_counts), observed_frame_counts=dict(name_counts),
            all_scenario_entities_observed=(expected <= names),
            all_non_ego_entities_observed=(expected_without_ego <= names),
            matching='Exact strings only: no whitespace stripping, case folding, or truncated-name aliases',
            limitation='Names establish presence of declared Players/Characters/Objects only, not identity uniqueness, map assets, spawn lifecycle, or collision feed completeness'),
        items=items, lane_transitions=transitions, controller=controller,
        deductions=[r for r in records if r.get('kind')=='deduction'],
        events=[r for r in records if r.get('kind')=='event'],
        turn_signal_event_field_coverage={name:dict(present=sum(value_present(r.get(name)) for r in reported_changes),
            missing=sum(not value_present(r.get(name)) for r in reported_changes)) for name in (
                'turn_signal_expected','turn_signal_observed','turn_signal_observation_history_s',
                'turn_signal_active_duration_s','turn_signal_from_road_id','turn_signal_from_lane_id',
                'turn_signal_to_road_id','turn_signal_to_lane_id','turn_signal_transition_sim_time_s')},
        limitations=['Single M16 run cannot cover unencountered rule cases',
            'No visual ground truth is asserted by this report; synchronized mainRS/birdRS still require review',
            'Road changes and missing lane links require review',
            'Lane-direction secondary geometry shares production code and is not independent proof'])
    return report


def markdown(report):
    transport, route, entities = (report[k] for k in ('transport','route','entities'))
    lines = ['# M16 자동채점 실주행 증거 감사', '',
        f"- 감사 상태: {report['audit_status']}",
        f"- 평가 프레임: {report['assessment_frames']}; 원본 매칭: {transport['matched_frames']}",
        f"- 최종 상태: {report['final_status']}; 요약: `{json.dumps(report['final_summary'],ensure_ascii=False)}`",
        f"- 경로 투영: {route['projected_frames']}/{report['assessment_frames']}; 미구간: {route['unsegmented_frames']}",
        f"- 속도 오차 최대: {transport['speed_error_mps']['max']} m/s; 위치 오차 최대: {transport['position_error_m']['max']} m",
        f"- 거리 독립 합산: {transport['independently_summed_distance_m']:.6f} m; 대시보드: {transport['final_dashboard_distance_m']} m",
        f"- 전체 캡처에서 빠진 시나리오 객체: {', '.join(entities['missing_entire_capture']) or '없음'}", '',
        '| 번호 | 항목 | 측정/누락 프레임 | 평가기 주장 | 감점 | 감사 범위 |',
        '|---|---|---|---|---|---|']
    for item in report['items']:
        lines.append(f"| {item['number']} | {item['name']} | {item['measured_frames']}/{item['missing_frames']} | {item['claimed_status']} | {item['claimed_penalty']} | {item['audit_status']} |")
    lines += ['', '## 차선 전환 증거', '', '| 시간 | 도로 | 차선 | XODR 연결 검사 | 3프레임 안정 | 방향지시등 |', '|---|---|---|---|---|---|']
    for change in report['lane_transitions']:
        indication = change.get('raw_indicator_audit',{})
        lines.append(f"| {change['sim_time_s']:.3f} | {change['from_road']}→{change['to_road']} | {change.get('from_lane','?')}→{change.get('to_lane','?')} | {change['classification']} | {change['stable_next_3_frames']} | {indication.get('verdict','미확인')} ({indication.get('continuously_correct_s',0):.3f}s) |")
    lines += ['', '## 해석 제한', ''] + ['- '+note for note in report['limitations']]
    lines += ['', '상세 프레임·오차·감점·차선 연결 증거는 같은 폴더의 audit.json에 저장됩니다.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('assessment','frames','scenario','xodr','report-dir'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--controller',type=Path)
    parser.add_argument('--ego-name',default='Ego')
    args = parser.parse_args()
    report = run_audit(args)
    args.report_dir.mkdir(parents=True,exist_ok=True)
    json_path, md_path = args.report_dir/'audit.json', args.report_dir/'audit.md'
    # Explicit report paths; input logs and the simulator are never modified.
    json_path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    md_path.write_text(markdown(report),encoding='utf-8')
    print(json.dumps(dict(report_json=str(json_path),report_markdown=str(md_path),
        matched=report['transport']['matched_frames'],assessment_frames=report['assessment_frames'],
        missing_entities=report['entities']['missing_entire_capture']),ensure_ascii=False))


if __name__ == '__main__':
    main()
