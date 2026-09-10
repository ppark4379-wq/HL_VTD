"""Evidence-based RDB signal decoding and conservative map/stop-line binding.

Only dashboard observation is implemented here. No simulation commands are sent.
Wire layout: supplied viRDBIcd.h, pack(4), TRAFFIC_LIGHT_BASE(12), EXT(16),
PHASE(8); phase enum is RDB_TRLIGHT_PHASE, not an arbitrary traffic-light enum.
"""
from __future__ import annotations

import hashlib
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

from assessment_geometry import local_point, signed_distance

PHASE_NAMES = {0: 'OFF', 1: 'STOP', 2: 'STOP_ATTN', 3: 'GO',
               4: 'GO_EXCL', 5: 'ATTN', 6: 'BLINK', 7: 'UNKNOWN'}
PHASE_STATES = {'STOP': 'RED', 'GO': 'GREEN', 'ATTN': 'YELLOW'}
SCENARIO_PHASES = {'off': 0, 'stop': 1, 'stop_attention': 2, 'go': 3,
                   'go_exclusive': 4, 'attention': 5, 'blink': 6, 'unknown': 7}

# Numerical duplicate tolerance, not an extension of a stop line or road rule.
STOP_LINE_DUPLICATE_TOLERANCE_M = .001


def _duplicate_line_evidence(first, second):
    """Prove equivalent finite segments and approach direction, or do not merge."""
    def known(line):
        lanes = line.get('lane_ids')
        numbers = [line.get('s'), line.get('t')]
        for key in ('begin', 'end'):
            point = line.get(key)
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                return False
            numbers.extend(point)
        return (isinstance(line.get('id'), str) and bool(line['id'])
                and type(line.get('road_id')) is int
                and isinstance(lanes, (list, tuple)) and bool(lanes)
                and all(type(lane) is int and lane != 0 for lane in lanes)
                and type(line.get('direction')) is int and line['direction'] in (-1, 1)
                and all(isinstance(value, (int, float)) and not isinstance(value, bool)
                        and math.isfinite(value) for value in numbers))
    if not known(first) or not known(second):
        return None
    if first['road_id'] != second['road_id'] or set(first['lane_ids']) != set(second['lane_ids']):
        return None
    for key in ('source', 'signal_id', 'signal_ids', 'controller_ids', 'state_masks'):
        if (key in first) != (key in second) or first.get(key) != second.get(key):
            return None
    tolerance = STOP_LINE_DUPLICATE_TOLERANCE_M
    if abs(first['s']-second['s']) > tolerance or abs(first['t']-second['t']) > tolerance:
        return None
    a, b, c, d = first['begin'], first['end'], second['begin'], second['end']
    distance = lambda p, q: math.hypot(p[0]-q[0], p[1]-q[1])
    lengths = distance(a, b), distance(c, d)
    if min(lengths) <= tolerance:
        return None
    direct = max(distance(a, c), distance(b, d))
    reverse = max(distance(a, d), distance(b, c))
    error = min(direct, reverse)
    if error > tolerance:
        return None
    dot = ((b[0]-a[0])*(d[0]-c[0])+(b[1]-a[1])*(d[1]-c[1])) / (lengths[0]*lengths[1])
    if dot * first['direction'] * second['direction'] < 1-1e-8:
        return None
    return dict(first_id=first['id'], second_id=second['id'], endpoint_error_m=error,
                road_s_error_m=abs(first['s']-second['s']),
                road_t_error_m=abs(first['t']-second['t']), reversed_endpoints=reverse < direct)


def normalize_stop_line_candidates(candidates):
    """Collapse only pairwise proven aliases; input segments are never changed.

    Adjacent collinear pieces, different lanes, unknown metadata, different
    road coordinates and transitive chains are not merged. The canonical
    segment is retained exactly, without extension, averaging or movement.
    """
    groups = []
    for candidate in candidates:
        for group in groups:
            proofs = [_duplicate_line_evidence(member[2], candidate[2]) for member in group]
            if all(proof is not None for proof in proofs):
                group.append(candidate)
                break
        else:
            groups.append([candidate])
    normalized, merges = [], []
    for group in groups:
        # Representative identity must not follow the current distance sort:
        # a swap while crossing almost coincident aliases would reset the
        # signal evaluator's per-stop-line observation history.
        if len(group) > 1:
            group = sorted(group, key=lambda candidate: candidate[2]['id'])
        absolute, distance, original = group[0]
        line = dict(original)
        if len(group) > 1:
            alias_ids = sorted({item[2]['id'] for item in group})
            proof = dict(canonical_id=line['id'], alias_ids=alias_ids,
                reason='same_road_lane_set_road_coordinates_finite_segment_and_approach_direction',
                tolerance_m=STOP_LINE_DUPLICATE_TOLERANCE_M,
                comparisons=[_duplicate_line_evidence(group[i][2], group[j][2])
                             for i in range(len(group)) for j in range(i+1, len(group))])
            line.update(alias_ids=alias_ids, duplicate_normalization=proof)
            merges.append(proof)
        normalized.append((absolute, distance, line))
    return normalized, merges


def decode_traffic_light(data, flags):
    """Preserve malformed evidence but never assign a phase to invalid input."""
    result = dict(phase_valid=False, phase_error=None, logic_phase=None,
                  controller_id=None, cycle_time_s=None, phases=[],
                  evidence=dict(source='RDB_PACKAGE_27', flags=flags,
                                element_bytes=len(data), raw_hex=data.hex()))
    if len(data) < 12:
        if len(data) >= 4:
            result['id'] = struct.unpack_from('<i', data)[0]
        result['phase_error'] = 'truncated_base'
        return result
    ident, state, mask = struct.unpack_from('<ifI', data)
    result.update(id=ident, state=state if math.isfinite(state) else None, mask=mask)
    def invalid(reason):
        result['phase_error'] = reason
        return result
    if not math.isfinite(state) or not 0 <= state <= 1:
        return invalid('invalid_normalized_state')
    if not flags & 1:
        return invalid('extended_phase_data_not_supplied')
    if len(data) < 28:
        return invalid('truncated_extension')
    ctrl, cycle, count, size = struct.unpack_from('<ifH2xI', data, 12)
    result.update(controller_id=ctrl, cycle_time_s=cycle if math.isfinite(cycle) else None)
    result['evidence'].update(no_phases=count, phase_data_size=size)
    if ctrl < 0 or not math.isfinite(cycle) or cycle <= 0:
        return invalid('invalid_controller_or_cycle')
    if not count or count > 4096 or size != count * 8 or len(data) != 28 + size:
        return invalid('invalid_phase_count_or_length')
    total = 0.0
    for index in range(count):
        duration, phase_type = struct.unpack_from('<fB3x', data, 28 + index * 8)
        if not math.isfinite(duration) or not 0 <= duration <= 1 or phase_type not in PHASE_NAMES:
            return invalid('invalid_phase_entry')
        result['phases'].append(dict(index=index, type=phase_type, name=PHASE_NAMES[phase_type],
            duration=duration, duration_s=duration * cycle, begin=total, end=total + duration))
        total += duration
    if abs(total - 1.0) > 1e-4:
        return invalid('phase_durations_do_not_sum_to_one')
    position = 0.0 if state == 1.0 else state
    nonzero = [p for p in result['phases'] if p['duration'] > 0]
    current = next((p for p in nonzero if p['begin'] <= position < p['end']), None)
    if current is None and nonzero and 0 <= position - total <= 1e-4:
        current = nonzero[-1]
    if current is None:
        return invalid('phase_not_resolved')
    if any(abs(position - p['begin']) <= 1e-6 for p in nonzero[1:]):
        return invalid('phase_boundary_rounding_ambiguous')
    result.update(phase_valid=True, logic_phase=current['name'], phase_index=current['index'],
                  cycle_position_s=position * cycle)
    return result


def _int(value, fallback=None):
    try:
        return int(value)
    except (TypeError, ValueError):
        return fallback


def _hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_signal_models(directory):
    """Read actual DynObjects StateBindings; never infer color from XODR IDs."""
    models = {}
    for path in sorted(Path(directory).glob('*.xml')):
        try:
            root = ET.parse(path).getroot()
        except (OSError, ET.ParseError):
            continue
        for node in root.findall('.//DynamicObjectType'):
            if node.get('category') != 'trafficLight':
                continue
            bindings = node.findall('.//StateBindings/State')
            colors = {b.get('name', '').upper() for b in bindings}
            if not colors or not colors <= {'RED', 'YELLOW', 'GREEN'}:
                continue
            key = (_int(node.get('type')), _int(node.get('subtype'), -1))
            entry = dict(colors=sorted(colors), name=node.get('name'),
                bindings=[dict(b.attrib) for b in bindings], source=str(path.resolve()),
                sha256=_hash(path))
            name = node.get('name', '')
            entry['movement'] = ('UTURN' if 'UTurn' in name else
                'LEFT' if name.endswith('Left') else 'RIGHT' if name.endswith('Right') else
                'STRAIGHT' if name.endswith('Straight') or name.endswith('Down') else
                'AMBIGUOUS_ARROW' if 'Arrow' in name else 'ALL')
            if key in models and models[key]['colors'] != entry['colors']:
                models[key] = dict(colors=[], source='conflicting_model_definitions')
            else:
                models[key] = entry
    return models


class SignalResolver:
    """Resolve only a proven approach/phase; uncertainty remains explicit."""

    def __init__(self, xodr_path, map_data, stop_lines, *, scenario_file=None, signal_models=None,
                 authority='strict'):
        if authority not in {'strict', 'rdb_logic'}:
            raise ValueError('signal authority must be strict or rdb_logic')
        self.authority = authority
        self.map = map_data
        self.stop_lines = stop_lines
        self.xodr_path = Path(xodr_path)
        self.source = dict(xodr=str(self.xodr_path.resolve()), xodr_sha256=_hash(self.xodr_path),
                           signal_authority=authority)
        root = ET.parse(self.xodr_path).getroot()
        links = {}
        for controller in root.findall('controller'):
            for control in controller.findall('control'):
                links.setdefault(_int(control.get('signalId')), set()).add(_int(controller.get('id')))
        self.signals = {}
        for road in root.findall('road'):
            for signal in road.findall('./signals/signal'):
                if signal.get('dynamic') != 'yes':
                    continue
                sid = _int(signal.get('id'))
                self.signals[sid] = dict(id=sid, road_id=_int(road.get('id')),
                    s=float(signal.get('s', 0)), t=float(signal.get('t', 0)),
                    type=_int(signal.get('type')), subtype=_int(signal.get('subtype'), -1),
                    orientation=signal.get('orientation'), h_offset=float(signal.get('hOffset', 0)),
                    controller_ids=sorted(c for c in links.get(sid, set()) if c is not None),
                    validity=[(_int(v.get('fromLane')), _int(v.get('toLane')))
                              for v in signal.findall('validity')],
                    position_road=[dict(p.attrib) for p in signal.findall('positionRoad')])
        if isinstance(signal_models, dict):
            self.models = signal_models
        else:
            paths = [Path(signal_models)] if signal_models else sorted(
                (Path.home() / 'VIRES').glob('VTD.*/Data/Distros/Current/Config/DynObjects/Logic'))
            self.models = load_signal_models(paths[-1]) if paths else {}
        self.scenario_phases = {}
        self.scenario_lights = {}
        self.scenario_error = None
        if scenario_file:
            try:
                path = Path(scenario_file)
                scenario = ET.parse(path).getroot()
                self.source.update(scenario_file=str(path.resolve()), scenario_sha256=_hash(path))
                for controller in scenario.findall('.//LightSigns/SignalController'):
                    self.scenario_phases[_int(controller.get('Id'))] = [
                        dict(type=SCENARIO_PHASES.get(p.get('Type')), duration_s=float(p.get('Duration', 0)))
                        for p in controller.findall('Phase')]
                for signal in scenario.findall('.//LightSigns/Signal'):
                    self.scenario_lights[_int(signal.get('Id'))] = [
                        [light.get('state') == 'true' for light in state.findall('Light')]
                        for state in signal.findall('LightState')]
            except (OSError, ET.ParseError, ValueError) as error:
                self.scenario_error = str(error)

    def _controller_state(self, records, frame, movement=None):
        diagnostic = self._strict_controller_state(records, frame, movement)
        if self.authority == 'strict':
            return diagnostic
        warnings = diagnostic['reasons'] if diagnostic['status'] != 'READY' else []
        def finish(status, state, reasons):
            return dict(status=status, state=state, reasons=reasons,
                        warnings=warnings, evidence=diagnostic['evidence'],
                        authority='rdb_logic', strict_status=diagnostic['status'])
        phases = set()
        for record in records:
            raw = frame.get('signals', {}).get(record['id']) or frame.get('signals', {}).get(str(record['id']))
            if not raw or not raw.get('phase_valid'):
                return finish('UNAVAILABLE', None, ['extended_phase_unavailable_or_invalid'])
            if raw.get('controller_id') not in record['controller_ids']:
                return finish('CONFLICT', None, ['rdb_xodr_controller_mismatch'])
            phases.add(raw.get('logic_phase'))
        if len(phases) != 1:
            return finish('CONFLICT', None, ['controller_lamps_phase_disagree'])
        phase = phases.pop()
        if phase == 'BLINK':
            # The phase enum itself does not encode flashing RED vs YELLOW.
            if diagnostic['status'] == 'READY' and diagnostic['state'] in {'FLASHING_RED', 'FLASHING_YELLOW'}:
                return finish('READY', diagnostic['state'], [])
            return finish('UNAVAILABLE', None, ['blink_color_not_verified'])
        state = PHASE_STATES.get(phase)
        if state is None:
            return finish('UNAVAILABLE', None, ['logic_phase_not_an_unambiguous_competition_state'])
        if state == 'GREEN':
            # Phase GO does not grant an unverified directional arrow movement.
            for record in records:
                model = self.models.get((record['type'], record['subtype']))
                if not model or not model.get('colors'):
                    return finish('UNAVAILABLE', None, ['signal_movement_model_unavailable'])
                if 'GREEN' in model['colors']:
                    arrow = model.get('movement', 'ALL')
                    if arrow != 'ALL' and (arrow == 'AMBIGUOUS_ARROW' or arrow != movement):
                        return finish('UNAVAILABLE', None, ['green_arrow_route_movement_not_verified'])
        return finish('READY', state, [])

    def _strict_controller_state(self, records, frame, movement=None):
        """Check raw phase, scenario phase and per-lamp render assignment."""
        proof = []
        phase = None
        rendered_colors = set()
        conflicts = []
        unavailable = []
        for record in records:
            signal = frame.get('signals', {}).get(record['id'])
            if signal is None:
                signal = frame.get('signals', {}).get(str(record['id']))
            item = dict(map_signal=record, raw_signal=signal)
            proof.append(item)
            if not signal or not signal.get('phase_valid'):
                unavailable.append('extended_phase_unavailable_or_invalid')
                continue
            if signal.get('controller_id') not in record['controller_ids']:
                conflicts.append('rdb_xodr_controller_mismatch')
            current = signal['logic_phase']
            if phase is not None and phase != current:
                conflicts.append('controller_lamps_phase_disagree')
            phase = current
            plan = self.scenario_phases.get(signal['controller_id'])
            light_states = self.scenario_lights.get(record['id'])
            if plan is not None:
                raw_phases = signal['phases']
                if len(plan) != len(raw_phases) or any(
                    p['type'] != raw['type'] or abs(p['duration_s'] - raw['duration_s']) > .01
                    for p, raw in zip(plan, raw_phases)):
                    conflicts.append('rdb_scenario_phase_plan_mismatch')
            model = self.models.get((record['type'], record['subtype']))
            item['model'] = model
            if light_states is None:
                unavailable.append('scenario_lamp_states_unavailable')
                continue
            if len(light_states) != len(signal['phases']):
                conflicts.append('scenario_lamp_phase_count_mismatch')
                item['scenario_light_states'] = light_states
                continue
            if not model or len(model.get('colors', [])) != 1:
                unavailable.append('single_color_model_binding_not_verified')
                continue
            lights = light_states[signal['phase_index']]
            item['scenario_active_lights'] = lights
            if len(lights) != 1:
                unavailable.append('multi_element_lamp_binding_not_supported')
                continue
            active = lights[0]
            color = model['colors'][0]
            # For a verified single-color, one-element lamp, mask==0 proves
            # no active gfx switch; a nonzero mask alone never identifies color.
            mask = signal['mask']
            if active != bool(mask):
                if not (phase == 'BLINK' and active and mask == 0):
                    conflicts.append('scenario_render_mask_disagree')
            if mask and mask & (mask - 1):
                unavailable.append('multi_bit_render_mask_not_verified')
            if active:
                arrow = model.get('movement', 'ALL')
                if color == 'GREEN' and arrow != 'ALL' and arrow != movement:
                    unavailable.append('green_arrow_route_movement_not_verified')
                rendered_colors.add(color)
        state = PHASE_STATES.get(phase)
        if phase == 'BLINK':
            if rendered_colors == {'RED'}:
                state = 'FLASHING_RED'
            elif rendered_colors == {'YELLOW'}:
                state = 'FLASHING_YELLOW'
            else:
                unavailable.append('blink_color_unresolved')
        elif state is None:
            unavailable.append('logic_phase_not_an_unambiguous_competition_state')
        elif rendered_colors and rendered_colors != {state}:
            conflicts.append('logic_render_color_conflict')
        elif not rendered_colors:
            unavailable.append('render_color_unresolved')
        if conflicts:
            return dict(status='CONFLICT', state=None, reasons=sorted(set(conflicts)), evidence=proof)
        if unavailable:
            return dict(status='UNAVAILABLE', state=None, reasons=sorted(set(unavailable)), evidence=proof)
        return dict(status='READY', state=state, reasons=[], evidence=proof)

    def resolve(self, frame, ego, road, route=None, route_s=None):
        evidence = dict(self.source, frame=frame.get('frame'), sim_time=frame.get('time'),
                        road_position=road, candidate_stop_lines=[])
        result = dict(status='UNAVAILABLE', reason=None, traffic_light_state=None,
                      signal_phase=None, signal_authority=self.authority,
                      signal_source=('RDB_EXT_PHASE_AUTHORITY+XODR' if self.authority == 'rdb_logic'
                                     else 'RDB_EXT_PHASE+SCENARIO_LIGHTS+XODR+MODEL'),
                      stop_line=None, stop_line_distance_m=None, evidence=evidence)
        def finish(status, reason):
            result.update(status=status, reason=reason)
            return result
        if self.scenario_error:
            evidence['scenario_error'] = self.scenario_error
            if self.authority == 'strict':
                return finish('CONFLICT', 'scenario_config_unreadable')
        if not road or not self.map:
            return finish('UNAVAILABLE', 'road_position_or_map_unavailable')
        pose = self.map.reference_pose(road['road'], road['s'])
        if pose is None:
            return finish('UNAVAILABLE', 'road_reference_pose_unavailable')
        # reference_pose returns x,y,heading. Derive actual travel sense from
        # Ego yaw, never lane-ID sign or signal artwork orientation alone.
        alignment = math.cos(ego['h'] - pose[2])
        if abs(alignment) < .8:
            return finish('UNAVAILABLE', 'ego_not_aligned_with_road')
        travel = 1 if alignment > 0 else -1
        evidence['travel_s_direction'] = travel
        movement = None
        if route is not None:
            projection = route.project(ego, road, route_s)
            evidence['route_projection'] = projection
            if projection is None:
                return finish('UNAVAILABLE', 'route_projection_unavailable')
            index = projection['segment']
            first, second = route.points[index:index + 2]
            direction = math.atan2(second['y'] - first['y'], second['x'] - first['x'])
            if math.cos(direction - ego['h']) < .8:
                return finish('UNAVAILABLE', 'route_and_ego_direction_disagree')
            if road['road'] not in {first['road'], second['road']}:
                return finish('UNAVAILABLE', 'route_and_ego_road_disagree')
            near = route.point_at(projection['s'] + 5)
            far = route.point_at(projection['s'] + 30)
            if math.hypot(far[0]-near[0], far[1]-near[1]) > 5:
                delta = math.atan2(math.sin(math.atan2(far[1]-near[1], far[0]-near[0]) - ego['h']),
                                   math.cos(math.atan2(far[1]-near[1], far[0]-near[0]) - ego['h']))
                # This is only a route movement candidate, not an explicit lane
                # permission. Composite or near-boundary arrows stay unavailable.
                movement = 'LEFT' if delta > math.radians(35) else 'RIGHT' if delta < -math.radians(35) else 'STRAIGHT' if abs(delta) < math.radians(15) else None
                evidence['route_movement_candidate'] = movement
        bumper = local_point(ego, ego.get('off_x', 0) + ego['length']/2, ego.get('off_y', 0))
        candidates = []
        for original in self.stop_lines:
            if original.get('road_id') != road['road'] or road['lane'] not in original.get('lane_ids', []):
                continue
            line = dict(original)
            a, b = line['begin'], line['end']
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            if length <= 0:
                continue
            normal = (-(b[1]-a[1])/length, (b[0]-a[0])/length)
            dot = normal[0]*math.cos(ego['h']) + normal[1]*math.sin(ego['h'])
            if abs(dot) < .8:
                continue
            line['direction'] = 1 if dot > 0 else -1
            distance = -signed_distance(line, bumper) * line['direction']
            if not -10 <= distance <= 60:
                continue
            evidence['candidate_stop_lines'].append(dict(id=line['id'], distance_m=distance))
            candidates.append((abs(distance), distance, line))
        if not candidates:
            road_lines = [l for l in self.stop_lines if l.get('road_id') == road['road']
                          and road['lane'] in l.get('lane_ids', [])]
            road_signals = [s for s in self.signals.values() if s['road_id'] == road['road']]
            # Establish complete spatial coverage of this lane on its road,
            # including every authored signal approach. No-feature roads are
            # deliberately not certified by a negative lookup alone.
            if route is not None and road_lines and road_signals and all(
                any(abs(s['s']-l['s']) <= 3 for l in road_lines) for s in road_signals):
                remaining = [(float(l['s']) - road['s']) * travel for l in road_lines]
                if all(d > 70 or d < -15 for d in remaining):
                    evidence['lane_stop_line_remaining_s_m'] = remaining
                    return finish('NOT_APPLICABLE', 'mapped_lane_signal_approaches_outside_evaluation_range')
            return finish('UNAVAILABLE', 'no_stop_line_for_current_lane')
        candidates, merges = normalize_stop_line_candidates(candidates)
        evidence['stop_line_normalization'] = dict(
            tolerance_m=STOP_LINE_DUPLICATE_TOLERANCE_M,
            raw_candidate_count=len(evidence['candidate_stop_lines']),
            normalized_candidate_count=len(candidates), merges=merges)
        candidates.sort(key=lambda c: c[0])
        if len(candidates) > 1 and abs(candidates[0][0] - candidates[1][0]) < .3:
            return finish('UNAVAILABLE', 'ambiguous_duplicate_stop_lines')
        _, distance, line = candidates[0]
        result.update(stop_line=line, stop_line_distance_m=distance)
        records = []
        for signal in self.signals.values():
            if signal['road_id'] != road['road'] or abs(signal['s'] - line['s']) > 3:
                continue
            # Model headings discriminate opposite approaches. Require close
            # alignment and record the raw orientation to expose map conflicts.
            signal_pose = self.map.reference_pose(signal['road_id'], signal['s'])
            if signal_pose is None or math.cos(ego['h'] - signal_pose[2] - signal['h_offset']) < .8:
                continue
            valid = signal['validity']
            if valid and not any(lo is not None and hi is not None and min(lo, hi) <= road['lane'] <= max(lo, hi)
                                 for lo, hi in valid):
                continue
            records.append(signal)
        if not records:
            return finish('UNAVAILABLE', 'no_signal_for_stop_line_approach')
        evidence['signal_mapping_basis'] = 'XODR_same_road_stop_s_heading_lane_validity_or_controller_consensus'
        groups = {}
        for record in records:
            if len(record['controller_ids']) != 1:
                return finish('UNAVAILABLE', 'signal_controller_binding_ambiguous')
            groups.setdefault(record['controller_ids'][0], []).append(record)
        states = {cid: self._controller_state(items, frame, movement) for cid, items in groups.items()}
        evidence['controllers'] = states
        result['warnings'] = sorted({reason for state in states.values()
                                     for reason in state.get('warnings', [])})
        if self.scenario_error:
            result['warnings'].append('scenario_config_unreadable')
        # Preserve the two independently observed GT channels even on conflict.
        # A normalized cycle position is not a lamp color; a mask is not a phase.
        # These fields are diagnostic only and never enable grading on their own.
        lit_colors = set()
        unknown_lamps = []
        for record in records:
            raw = frame.get('signals', {}).get(record['id']) or frame.get('signals', {}).get(str(record['id']))
            model = self.models.get((record['type'], record['subtype']))
            if raw is None or not model or len(model.get('colors', [])) != 1:
                unknown_lamps.append(record['id'])
            elif raw.get('mask'):
                lit_colors.update(model['colors'])
        evidence['runtime_lamp_colors'] = sorted(lit_colors)
        evidence['runtime_lamp_unknown_ids'] = unknown_lamps
        result['gt_lamp_colors'] = sorted(lit_colors) if not unknown_lamps else None
        result['conflict_reasons'] = sorted({reason for item in states.values() for reason in item['reasons']})
        observed_phases = {s['logic_phase'] for s in frame.get('signals', {}).values()
                           if s.get('id') in {r['id'] for r in records} and s.get('logic_phase')}
        result['signal_phase'] = observed_phases.pop() if len(observed_phases) == 1 else None
        result['gt_logical_state'] = PHASE_STATES.get(result['signal_phase'])
        line.update(signal_ids=sorted(r['id'] for r in records), controller_ids=sorted(groups))
        if any(s['status'] == 'CONFLICT' for s in states.values()):
            return finish('CONFLICT', 'signal_configuration_or_render_conflict')
        if any(s['status'] != 'READY' for s in states.values()):
            return finish('UNAVAILABLE', 'signal_state_not_fully_verified')
        values = {s['state'] for s in states.values()}
        if len(values) != 1:
            return finish('UNAVAILABLE', 'possible_lane_controllers_disagree')
        result['traffic_light_state'] = values.pop()
        line['signal_id'] = records[0]['id'] if len(groups) == 1 else None
        return finish('READY', 'verified_rdb_logic_and_approach_consensus' if self.authority == 'rdb_logic'
                      else 'verified_phase_render_and_approach_consensus')
