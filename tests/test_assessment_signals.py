import copy
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools' / 'dashboard'))
from assessment_signals import decode_traffic_light, SignalResolver, load_signal_models
from opendrive_assessment import OpenDriveMap
from assessment_evaluators import AssessmentContext, evaluate_traffic_signal, evaluate_pedestrian_response
from assessment_geometry import obb


def wire(ident=1, state=.2, mask=8, controller=10, phases=(1, 3, 5)):
    return (struct.pack('<ifIifH2xI', ident, state, mask, controller, 30., len(phases), len(phases)*8)
            + b''.join(struct.pack('<fB3x', 1/len(phases), p) for p in phases))


class SignalDecoderTests(unittest.TestCase):
    def test_m16_actual_extended_bytes(self):
        data = bytes.fromhex('9d010000230f463f000000008200000000009f4304000000200000006682f13e060000006682f13e010000001f35413d03000000e5901a3c05000000')
        result = decode_traffic_light(data, 1)
        self.assertTrue(result['phase_valid'])
        self.assertEqual((result['id'], result['controller_id'], result['logic_phase']), (413, 130, 'STOP'))
        self.assertEqual([p['name'] for p in result['phases']], ['BLINK', 'STOP', 'GO', 'ATTN'])
        self.assertAlmostEqual(result['cycle_position_s'], 246.02657, places=4)
        self.assertEqual(result['evidence']['raw_hex'], data.hex())

    def test_variable_phase_lengths_and_boundary(self):
        for types in [(1,), (1, 3), (6, 1, 3, 5)]:
            result = decode_traffic_light(wire(phases=types), 1)
            self.assertTrue(result['phase_valid'])
            self.assertEqual(len(result['phases']), len(types))
        self.assertFalse(decode_traffic_light(wire(state=.5, phases=(1, 3)), 1)['phase_valid'])
        self.assertEqual(decode_traffic_light(wire(state=1), 1)['logic_phase'], 'STOP')

    def test_invalid_lengths_and_nan_are_fail_closed(self):
        data = wire()
        for invalid in [data[:4], data[:12], data[:27], data[:-1], data+b'extra', wire(state=float('nan'))]:
            result = decode_traffic_light(invalid, 1)
            self.assertFalse(result['phase_valid'])
            self.assertIsNone(result['logic_phase'])
            self.assertTrue(result['phase_error'])
        self.assertFalse(decode_traffic_light(data[:12], 0)['phase_valid'])

    def test_bad_phase_size_duration_and_unknown_type(self):
        for offset, fmt, value in [(24, '<I', 999), (28, '<f', .9), (32, '<B', 99), (16, '<f', 0.)]:
            data = bytearray(wire())
            struct.pack_into(fmt, data, offset, value)
            self.assertFalse(decode_traffic_light(data, 1)['phase_valid'])


class SignalResolverTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        directory = Path(self.temp.name)
        self.map_path = directory/'map.xodr'
        self.map_path.write_text('''<OpenDRIVE><road id="1" length="200" junction="-1">
          <planView><geometry s="0" x="0" y="0" hdg="0" length="200"><line/></geometry></planView>
          <signals>
            <signal id="1" dynamic="yes" type="1" subtype="-1" s="100" t="-1" hOffset="0"/>
            <signal id="2" dynamic="yes" type="2" subtype="-1" s="100" t="-1" hOffset="0"/>
            <signal id="3" dynamic="yes" type="3" subtype="-1" s="100" t="-1" hOffset="0"/>
          </signals></road><controller id="10"><control signalId="1"/><control signalId="2"/><control signalId="3"/></controller></OpenDRIVE>''')
        self.scenario_path = directory/'scenario.xml'
        self.scenario_path.write_text('''<Scenario><LightSigns><SignalController Id="10">
          <Phase Duration="10" Type="stop"/><Phase Duration="10" Type="go"/><Phase Duration="10" Type="attention"/>
          </SignalController>
          <Signal Id="1"><LightState><Light state="true"/></LightState><LightState><Light state="false"/></LightState><LightState><Light state="false"/></LightState></Signal>
          <Signal Id="2"><LightState><Light state="false"/></LightState><LightState><Light state="true"/></LightState><LightState><Light state="false"/></LightState></Signal>
          <Signal Id="3"><LightState><Light state="false"/></LightState><LightState><Light state="false"/></LightState><LightState><Light state="true"/></LightState></Signal>
          </LightSigns></Scenario>''')
        self.map = OpenDriveMap(self.map_path)
        self.lines = [dict(id='1:stop', road_id=1, lane_ids=[-1], s=100., t=-1.5,
                           begin=(100., -3.), end=(100., 0.))]
        self.models = {(i, -1): dict(colors=[color], movement='ALL', source='fixture_StateBindings')
                       for i, color in [(1, 'RED'), (2, 'GREEN'), (3, 'YELLOW')]}
        self.resolver = SignalResolver(self.map_path, self.map, self.lines,
            scenario_file=self.scenario_path, signal_models=self.models)
        self.ego = dict(x=90., y=-1.5, h=0., length=4., off_x=0., off_y=0.)
        self.road = dict(road=1, lane=-1, s=90.)
        self.frame = dict(frame=1, time=6., signals={i: decode_traffic_light(
            wire(ident=i, mask=8 if i==1 else 0), 1) for i in [1, 2, 3]})

    def resolve(self):
        return self.resolver.resolve(self.frame, self.ego, self.road)

    def test_valid_red_requires_phase_render_and_approach(self):
        result = self.resolve()
        self.assertEqual((result['status'], result['traffic_light_state']), ('READY', 'RED'))
        self.assertEqual(result['stop_line_distance_m'], 8.)
        self.assertEqual(result['signal_phase'], 'STOP')
        self.assertIn('controllers', result['evidence'])

    def test_rdb_logic_red_keeps_display_conflict_as_warning(self):
        self.resolver.authority = 'rdb_logic'
        self.resolver.scenario_lights[1][0] = [False]
        self.resolver.scenario_lights[2][0] = [True]
        self.frame['signals'][1]['mask'] = 0
        self.frame['signals'][2]['mask'] = 8
        result = self.resolve()
        self.assertEqual((result['status'], result['traffic_light_state']), ('READY', 'RED'))
        self.assertEqual(result['gt_lamp_colors'], ['GREEN'])
        self.assertIn('logic_render_color_conflict', result['warnings'])
        self.assertEqual(result['signal_authority'], 'rdb_logic')
        self.resolver.authority = 'strict'
        self.assertEqual(self.resolve()['status'], 'CONFLICT')

    def test_rdb_logic_phase_count_and_scenario_read_error_are_warnings(self):
        self.resolver.authority = 'rdb_logic'
        self.resolver.scenario_error = 'fixture file unavailable'
        for states in self.resolver.scenario_lights.values():
            states.pop()
        result = self.resolve()
        self.assertEqual(result['traffic_light_state'], 'RED')
        self.assertIn('scenario_lamp_phase_count_mismatch', result['warnings'])
        self.assertIn('scenario_config_unreadable', result['warnings'])

    def test_rdb_logic_does_not_bypass_missing_gt_or_controller_binding(self):
        self.resolver.authority = 'rdb_logic'
        self.frame['signals'][1]['controller_id'] = 999
        self.assertEqual(self.resolve()['status'], 'CONFLICT')
        self.frame['signals'][1]['controller_id'] = 10
        self.frame['signals'][1]['phase_valid'] = False
        self.assertEqual(self.resolve()['status'], 'UNAVAILABLE')

    def test_rdb_logic_green_requires_verified_arrow_movement(self):
        self.resolver.authority = 'rdb_logic'
        for i in [1, 2, 3]:
            self.frame['signals'][i] = decode_traffic_light(wire(ident=i, state=.5, mask=8 if i == 2 else 0), 1)
        self.assertEqual(self.resolve()['traffic_light_state'], 'GREEN')
        self.resolver.models[(2, -1)]['movement'] = 'LEFT'
        self.assertEqual(self.resolve()['status'], 'UNAVAILABLE')
        self.resolver.models = {}
        self.assertEqual(self.resolve()['status'], 'UNAVAILABLE')

    def test_rdb_logic_blink_color_and_unknown_phase_remain_unavailable(self):
        self.resolver.authority = 'rdb_logic'
        for phase in (6, 7, 4):
            for i in [1, 2, 3]:
                self.frame['signals'][i] = decode_traffic_light(wire(ident=i, mask=0, phases=(phase, 3, 5)), 1)
            self.assertEqual(self.resolve()['status'], 'UNAVAILABLE')

    def test_missing_model_or_signal_is_not_a_pass(self):
        self.resolver.models = {}
        self.assertEqual(self.resolve()['status'], 'UNAVAILABLE')
        self.resolver.models = self.models
        del self.frame['signals'][2]
        self.assertEqual(self.resolve()['status'], 'UNAVAILABLE')

    def test_controller_and_mask_mismatch_are_conflicts(self):
        self.frame['signals'][1]['controller_id'] = 99
        self.assertEqual(self.resolve()['status'], 'CONFLICT')
        self.frame['signals'][1]['controller_id'] = 10
        self.frame['signals'][1]['mask'] = 0
        self.assertEqual(self.resolve()['status'], 'CONFLICT')

    def test_m16_phase_count_mismatch_is_conflict(self):
        for signal in self.frame['signals'].values():
            signal['phases'].append(dict(type=5, duration_s=3))
        self.assertEqual(self.resolve()['status'], 'CONFLICT')

    def test_logic_render_color_conflict_is_not_numeric_state(self):
        self.resolver.scenario_lights[1][0] = [False]
        self.resolver.scenario_lights[2][0] = [True]
        self.frame['signals'][1]['mask'] = 0
        self.frame['signals'][2]['mask'] = 8
        result = self.resolve()
        self.assertEqual(result['status'], 'CONFLICT')
        self.assertIsNone(result['traffic_light_state'])
        self.assertEqual(result['gt_logical_state'], 'RED')
        self.assertEqual(result['gt_lamp_colors'], ['GREEN'])
        self.assertIn('logic_render_color_conflict', result['conflict_reasons'])

    def test_blink_requires_verified_red_color_and_allows_dark_half_cycle(self):
        self.resolver.scenario_phases[10][0]['type'] = 6
        for i in [1, 2, 3]:
            self.frame['signals'][i] = decode_traffic_light(wire(ident=i, mask=0, phases=(6,3,5)), 1)
        self.assertEqual(self.resolve()['traffic_light_state'], 'FLASHING_RED')
        self.resolver.models = {}
        self.assertEqual(self.resolve()['status'], 'UNAVAILABLE')

    def test_duplicate_stop_line_and_wrong_lane_are_unavailable(self):
        # A genuinely different stop line 2 cm away must remain ambiguous.
        self.lines.append(dict(self.lines[0], id='duplicate', s=100.02,
                               begin=(100.02, -3.), end=(100.02, 0.)))
        self.assertEqual(self.resolve()['reason'], 'ambiguous_duplicate_stop_lines')
        self.lines.pop()
        self.road['lane'] = -2
        self.assertEqual(self.resolve()['status'], 'UNAVAILABLE')

    def test_identical_stop_line_aliases_work_in_strict_and_rdb_logic(self):
        self.lines.append(dict(self.lines[0], id='alias'))
        original = copy.deepcopy(self.lines)
        for authority in ('strict', 'rdb_logic'):
            self.resolver.authority = authority
            result = self.resolve()
            self.assertEqual((result['status'], result['traffic_light_state']), ('READY', 'RED'))
            self.assertEqual(result['stop_line']['alias_ids'], ['1:stop', 'alias'])
            self.assertEqual(result['stop_line_distance_m'], 8.)
        self.assertEqual(self.lines, original)

    def test_opposite_approach_is_not_scored(self):
        self.ego['h'] = math.pi
        self.assertEqual(self.resolve()['status'], 'UNAVAILABLE')

    def test_verified_distant_approach_is_not_applicable(self):
        self.ego['x'] = 5.
        self.road['s'] = 5.
        class Route:
            points=[dict(x=0,y=-1.5,road=1),dict(x=200,y=-1.5,road=1)]
            def project(self, *args): return dict(s=5., segment=0)
            def point_at(self, value): return (value, -1.5)
        result = self.resolver.resolve(self.frame, self.ego, self.road, Route())
        self.assertEqual(result['status'], 'NOT_APPLICABLE')
        self.resolver.stop_lines = []
        self.assertEqual(self.resolver.resolve(self.frame,self.ego,self.road,Route())['status'],'UNAVAILABLE')

    def test_model_color_comes_from_xml_binding(self):
        directory = Path(self.temp.name)/'models'
        directory.mkdir()
        (directory/'lamp.xml').write_text('''<DynamicObjectTypes><DynamicObjectType type="123" subtype="-1" name="MisleadingGreenName" category="trafficLight"><Messages><StateBindings><State name="Red" bit="0"/></StateBindings></Messages></DynamicObjectType></DynamicObjectTypes>''')
        model = load_signal_models(directory)[(123, -1)]
        self.assertEqual(model['colors'], ['RED'])
        self.assertEqual(len(model['sha256']), 64)

    def signal_context(self, metadata):
        ego = dict(self.ego, id=1, name='Ego', category=1, type=1,
                   z=0., p=0., r=0., width=2., height=1.5, speed=0.)
        frame = dict(self.frame, objects={1: ego}, road={1: self.road})
        return AssessmentContext(frame=frame, ego=ego, road=self.road, before=None,
            dt=.04, row={}, corners=obb(ego), rear_offset=0, rear=(ego['x'],ego['y']),
            metadata=metadata)

    def test_explicit_metadata_cannot_bypass_resolver_conflict(self):
        from live_assessment import LiveAssessment
        # The old manual mask says GREEN, while actual GT/scenario validation
        # conflicts. Normal integration must use the resolver in this case.
        manual = dict(self.lines[0], direction=-1, signal_id=1, state_masks={8:'GREEN'})
        context = self.signal_context({'stop_lines':[manual]})
        self.resolver.scenario_lights[1].pop()
        assessment = LiveAssessment.__new__(LiveAssessment)
        assessment.signal_resolver = self.resolver
        assessment.route = None
        assessment.green_blocker_forward_m = 30
        assessment._forward_path_corridors = lambda *_: []
        assessment.pedestrian_stops = {}
        assessment.session = {'stop_speed_kmh':0}
        assessment._objects_and_signals(context)
        self.assertEqual(context.row['signal_resolution_status'], 'CONFLICT')
        self.assertIsNone(context.row.get('traffic_light_state'))

    def test_legacy_explicit_mask_checks_extended_gt_when_present(self):
        manual = dict(self.lines[0], direction=-1, signal_id=1, state_masks={8:'GREEN'})
        context = self.signal_context({'stop_lines':[manual]})
        evaluate_traffic_signal(context, (8.,manual))
        self.assertIsNone(context.row.get('traffic_light_state'))
        self.assertEqual(context.row['signal_resolution_status'], 'CONFLICT')
        # A base-only, explicitly configured legacy feed remains supported;
        # no extended truth is available to contradict its confirmed mapping.
        context.frame['signals']={1:{'state':.2,'mask':8}}
        context.row={}
        evaluate_traffic_signal(context,(8.,manual))
        self.assertEqual(context.row['traffic_light_state'],'GREEN')
        # An extension that is present but malformed must not fall back to
        # the manual mask and turn bad evidence into a valid numeric score.
        context.frame['signals']={1:decode_traffic_light(wire()[:-1],1)}
        context.row={}
        evaluate_traffic_signal(context,(8.,manual))
        self.assertIsNone(context.row.get('traffic_light_state'))

    def test_pedestrian_crosswalk_100m_lateral_does_not_create_stop_or_violation(self):
        metadata = dict(object_feed_complete=True, object_roles={'Ped':'pedestrian'},
            crosswalks=[dict(id='far_lateral', road_id=1,
                            polygon=[(0.,98.),(2.,98.),(2.,102.),(0.,102.)])])
        context = self.signal_context(metadata)
        context.ego.update(x=0., y=0., h=0.)
        context.corners = obb(context.ego)
        pedestrian = dict(context.ego, id=2, name='Ped', type=5,
                          x=1., y=100., length=.5, width=.5, height=1.8)
        stops = {}
        # Stop then resume on the same road while the crossing lies 100 m
        # sideways. Longitudinal dot distance alone incorrectly linked it.
        for speed in (0., 1.):
            with self.subTest(speed=speed):
                context.ego['speed'] = speed
                context.row = {}
                evaluate_pedestrian_response(context, [pedestrian], metadata, stops, 0.)
                self.assertFalse(context.row['pedestrian_violation'])
                self.assertIsNot(context.row.get('justified_stop'), True)
                self.assertEqual(stops, {})


if __name__ == '__main__':
    unittest.main()
