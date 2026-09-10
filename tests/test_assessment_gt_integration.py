"""Scenario-inventory/live-evaluator regressions, NOT a live VTD validation.

The M16 declaration is real; every RDB-like frame here is deliberately
synthetic. These tests verify integration contracts, not rendered mesh contact.
"""
from __future__ import annotations

from collections import Counter
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/dashboard'))

from assessment_ground_truth import ScenarioGroundTruth, obb_overlap_3d
from live_assessment import LiveAssessment


class GroundTruthIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario = ROOT / 'scenario/mixed/M16/M16.xml'
        cls.inventory = ScenarioGroundTruth(cls.scenario)

    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='hl-vtd-gt-integration-')
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        rules = {
            'base_score': 500, 'normal_speed_kmh': 50, 'school_speed_kmh': 30,
            'completion': {'mode': 'finish_line'},
            'session': {
                'start_trigger': 'controller_packet_arrival',
                'max_sample_gap_s': .5, 'time_limit_s': 900,
                'stop_speed_kmh': .5,
            },
            'metadata': {
                'scenario_file': str(self.scenario), 'rear_axle_offset_m': 0,
                'school_zones': [], 'stop_lines': [], 'centerlines': [],
                'sidewalk_boundaries': [],
                'crosswalks': [{
                    'id': 'test_crosswalk',
                    'polygon': [[-1, -4], [3, -4], [3, 4], [-1, 4]],
                }],
                'sections': [{
                    'id': '1', 'polygon': [
                        [-10000, -10000], [10000, -10000],
                        [10000, 10000], [-10000, 10000]],
                }],
            },
        }
        rules_path = self.directory / 'rules.yaml'
        rules_path.write_text(yaml.safe_dump(rules), encoding='utf-8')
        self.engine = LiveAssessment(rules_path, self.directory / 'assessment')
        self.addCleanup(self.engine.close)
        self.engine.on_control_received({'steering': 0, 'accel': 0, 'indicator': 0})

    def frame(self, index=0, *, absent=None, changes=None, time_s=None):
        objects = {}
        for identifier, (name, role) in enumerate(self.inventory.roles.items(), 1):
            if name == absent:
                continue
            person = role in {'pedestrian', 'cyclist'}
            obj = {
                'id': identifier, 'name': name,
                'category': 5 if role == 'static_obstacle' else 1,
                'type': 29 if role == 'static_obstacle' else 6 if person else 1,
                'x': 0 if name == 'Ego' else 100 + identifier * 10,
                'y': 0, 'z': 0, 'h': 0, 'p': 0, 'r': 0,
                'length': .5 if person else 4,
                'width': .5 if person else 2,
                'height': 1.7 if person else 1.5,
                'off_x': 0, 'off_y': 0, 'off_z': 0,
                'speed': 1 if name == 'Ego' else 0,
            }
            obj.update((changes or {}).get(name, {}))
            objects[identifier] = obj
        return {
            'frame': index, 'time': index * .1 if time_s is None else time_s,
            'version': 0x126, 'objects': objects, 'road': {}, 'lanes': {},
            'lights': {1: 'OFF'}, 'signals': {}, 'packages': [5, 9, 10, 2],
        }

    def send(self, index=0, **kwargs):
        self.engine.on_frame(self.frame(index, **kwargs))
        return self.engine.rows[-1]

    def item(self, number):
        return next(item for item in self.engine.snapshot()['official_grid']
                    if item['number'] == number)

    def test_all_38_m16_entities_are_complete_and_negatives_measured(self):
        self.assertEqual(len(self.inventory.roles), 38)
        self.assertEqual(Counter(self.inventory.roles.values()), {
            'road_user': 18, 'static_obstacle': 16, 'pedestrian': 2, 'cyclist': 2})
        row = self.send()
        self.assertEqual(row['object_feed_complete'], '1')
        self.assertEqual(row['object_expected_count'], '38')
        self.assertEqual(row['object_observed_count'], '38')
        self.assertEqual(row['object_feed_source'], 'scenario_inventory_per_rdb_frame')
        for field in ('static_collision', 'road_user_collision', 'pedestrian_violation'):
            self.assertEqual(row[field], '0', field)

    def test_one_missing_object_stops_pedestrian_judgment_then_restores(self):
        human = 'New Character'
        position = {human: {'x': 1, 'y': 2}}
        self.send(0)
        missing = self.send(1, absent='New Object05', changes=position)
        self.assertEqual(missing['object_feed_complete'], '0')
        self.assertEqual(json.loads(missing['object_missing_names']), ['New Object05'])
        self.assertEqual(missing.get('pedestrian_violation', ''), '')
        self.assertEqual(missing.get('static_collision', ''), '')
        self.assertEqual(missing.get('road_user_collision', ''), '')
        restored = self.send(2, changes=position)
        self.assertEqual(restored['object_feed_complete'], '1')
        self.assertEqual(restored['pedestrian_violation'], '1')
        self.assertIn(human, restored['pedestrian_names'])

    def test_missing_then_restored_negatives_never_claim_whole_run_pass(self):
        self.send(0)
        self.send(1, absent='New Object05')
        restored = self.send(2)
        self.assertEqual(restored['object_feed_complete'], '1')
        for number in (13, 14, 15):
            item = self.item(number)
            self.assertEqual(item['rule_status'], 'PASS')
            self.assertEqual(item['status'], 'PARTIAL')
            self.assertEqual(item['coverage_status'], 'PARTIAL')
            self.assertEqual(item['measured_frames'], 2)
            self.assertEqual(item['missing_frames'], 1)
        summary = self.engine.snapshot()['summary']
        self.assertIsNone(summary['official_score'])
        self.assertIn('object_inventory_gaps', summary['score_withheld_reasons'])
        self.assertIn('partial_measurements', summary['score_withheld_reasons'])

    def test_human_and_cyclist_same_rdb_type_are_disambiguated_by_scenario(self):
        # Both type=6 deliberately: the scenario role, not broad RDB type,
        # decides whether the actor triggers item 14.
        cyclist = self.send(0, changes={'New Character01': {'x': 1, 'y': 2}})
        self.assertEqual(cyclist['pedestrian_violation'], '0')
        human = self.send(1, changes={'New Character': {'x': 1, 'y': 2}})
        self.assertEqual(human['pedestrian_violation'], '1')
        self.assertEqual(human['pedestrian_names'], 'New Character')

    def test_scenario_object_type_29_contact_is_item_13_not_15(self):
        row = self.send(changes={'New Object': {'x': 0, 'y': 0}})
        self.assertEqual(row['static_collision'], '1')
        self.assertEqual(row['road_user_collision'], '0')
        self.assertEqual(row['static_collision_names'], 'New Object')
        self.assertEqual(row['static_collision_types'], '29')
        self.assertEqual(self.item(13)['penalty'], -3)
        self.assertEqual(self.item(15)['penalty'], 0)

    def test_stationary_car_contact_is_item_15_not_13(self):
        row = self.send(changes={'New Player': {'x': 0, 'y': 0, 'speed': 0}})
        self.assertEqual(row['road_user_collision'], '1')
        self.assertEqual(row['static_collision'], '0')
        self.assertEqual(row['road_user_collision_names'], 'New Player')
        self.assertEqual(self.item(15)['penalty'], -6)
        self.assertEqual(self.item(13)['penalty'], 0)

    def test_grade_separated_objects_do_not_collide_despite_xy_overlap(self):
        row = self.send(changes={
            'New Player': {'x': 0, 'y': 0, 'z': 8},
            'New Object': {'x': 0, 'y': 0, 'z': -8},
        })
        self.assertEqual(row['collision_source'], 'RDB_3D_OBB_SAT')
        self.assertEqual(row['road_user_collision'], '0')
        self.assertEqual(row['static_collision'], '0')

    def test_obb_local_vertical_offset_is_applied(self):
        first = self.frame()['objects'][1]
        second = copy.deepcopy(first)
        second.update(z=8, off_z=-8)
        self.assertTrue(obb_overlap_3d(first, second))
        second['off_z'] = 0
        self.assertFalse(obb_overlap_3d(first, second))

    def test_observation_time_gap_withholds_aggregate_even_with_complete_objects(self):
        self.send(0)
        row = self.send(1, time_s=2)
        self.assertEqual(row['object_feed_complete'], '1')
        snapshot = self.engine.snapshot()
        self.assertEqual(len(snapshot['observation_gaps']), 1)
        self.assertEqual(snapshot['observation_gaps'][0]['reason'], 'rdb_observation_gap')
        self.assertIsNone(snapshot['summary']['official_score'])
        self.assertIn('observation_gaps', snapshot['summary']['score_withheld_reasons'])

    def assert_gap_withheld(self, expected_reason):
        snapshot = self.engine.snapshot()
        self.assertTrue(any(gap['reason'] == expected_reason
                            for gap in snapshot['observation_gaps']))
        self.assertIsNone(snapshot['summary']['official_score'])
        self.assertIn('observation_gaps', snapshot['summary']['score_withheld_reasons'])

    def test_nan_ego_is_skipped_without_exception_and_restoration_still_withholds(self):
        self.send(0)
        self.engine.snapshot()  # Prime the cache before the rejected sample.
        self.engine.on_frame(self.frame(1, changes={'Ego': {'p': float('nan')}}))
        self.assertEqual(len(self.engine.rows), 1)
        self.assert_gap_withheld('missing_duplicate_or_invalid_ego_GT')
        restored = self.send(2)
        self.assertEqual(len(self.engine.rows), 2)
        self.assertEqual(restored['object_feed_complete'], '1')
        self.assertEqual(restored['road_user_collision'], '0')
        self.assert_gap_withheld('missing_duplicate_or_invalid_ego_GT')

    def test_final_frame_missing_ego_immediately_withholds_without_next_frame(self):
        self.send(0)
        self.engine.snapshot()  # No subsequent good frame may be necessary.
        self.engine.on_frame(self.frame(1, absent='Ego'))
        self.assertEqual(len(self.engine.rows), 1)
        self.assert_gap_withheld('missing_duplicate_or_invalid_ego_GT')

    def test_duplicate_ego_is_skipped_and_recorded_as_observation_gap(self):
        self.send(0)
        frame = self.frame(1)
        frame['objects'][999] = dict(frame['objects'][1], id=999)
        self.engine.on_frame(frame)
        self.assertEqual(len(self.engine.rows), 1)
        self.assert_gap_withheld('missing_duplicate_or_invalid_ego_GT')

    def test_nonfinite_simulation_time_is_skipped_and_withheld(self):
        self.send(0)
        self.engine.on_frame(self.frame(1, time_s=float('nan')))
        self.assertEqual(len(self.engine.rows), 1)
        self.assert_gap_withheld('missing_duplicate_or_invalid_ego_GT')

    def test_nonfinite_ego_speed_is_skipped_and_restoration_still_withholds(self):
        self.send(0)
        self.engine.on_frame(self.frame(1, changes={'Ego': {'speed': float('nan')}}))
        self.assertEqual(len(self.engine.rows), 1)
        self.assert_gap_withheld('invalid_ego_speed_GT')
        self.send(2)
        self.assertEqual(len(self.engine.rows), 2)
        self.assert_gap_withheld('invalid_ego_speed_GT')

    def test_duplicate_name_and_invalid_dimensions_reject_complete_inventory(self):
        frame = self.frame()
        frame['objects'][999] = dict(frame['objects'][2], id=999)
        evidence = self.inventory.inspect(frame)
        self.assertFalse(evidence['complete'])
        self.assertEqual(evidence['duplicate'], ['New Player'])
        evidence = self.inventory.inspect(self.frame(changes={'New Player': {'height': 0}}))
        self.assertFalse(evidence['complete'])
        self.assertEqual(evidence['invalid_geometry'], ['New Player'])

    def test_expected_actor_with_filtered_category_is_not_complete(self):
        """An actor excluded by the collector must not count as measured."""
        evidence = self.inventory.inspect(self.frame(changes={'New Player': {'category': 99}}))
        self.assertFalse(evidence['complete'])
        row = self.send(changes={'New Player': {'category': 99}})
        self.assertEqual(row['object_feed_complete'], '0')
        self.assertEqual(row.get('road_user_collision', ''), '')

    def test_nonfinite_orientation_and_offsets_are_not_complete(self):
        for field in ('p', 'r', 'off_x', 'off_y', 'off_z'):
            with self.subTest(field=field):
                evidence = self.inventory.inspect(self.frame(changes={
                    'New Player': {field: float('nan')},
                }))
                self.assertFalse(evidence['complete'])
                self.assertEqual(evidence['invalid_geometry'], ['New Player'])

    def test_invalid_actor_is_not_scored_as_contact_and_next_frame_recovers(self):
        invalid = self.send(changes={
            'New Player': {'p': float('nan'), 'off_x': float('nan')},
        })
        self.assertEqual(invalid['object_feed_complete'], '0')
        for field in ('static_collision', 'road_user_collision', 'pedestrian_violation'):
            self.assertEqual(invalid.get(field, ''), '', field)
        restored = self.send(1)
        self.assertEqual(restored['object_feed_complete'], '1')
        self.assertEqual(restored['road_user_collision'], '0')

    def test_z_separated_pedestrian_xy_candidate_withholds_judgment(self):
        # XY crosswalk polygons alone cannot distinguish an overpass from a
        # steep same-road approach. Do not turn that ambiguity into PASS.
        row = self.send(changes={'New Character': {'x': 1, 'y': 2, 'z': 20}})
        self.assertEqual(row['object_feed_complete'], '1')
        self.assertEqual(row.get('pedestrian_violation', ''), '')
        self.assertEqual(row.get('pedestrian_names', ''), '')

    def test_z_separated_green_blocker_xy_candidate_withholds_judgment(self):
        self.engine.meta['stop_lines'] = [{
            'id': 'test_stop', 'begin': [8, 2], 'end': [8, -2],
            'direction': 1, 'signal_id': 1, 'state_masks': {1: 'GREEN'},
        }]
        frame = self.frame(changes={'New Player': {'x': 6, 'y': 0, 'z': 20}})
        frame['signals'] = {1: {'mask': 1}}
        self.engine.on_frame(frame)
        row = self.engine.rows[-1]
        self.assertEqual(row['object_feed_complete'], '1')
        self.assertEqual(row['traffic_light_state'], 'GREEN')
        self.assertEqual(row.get('justified_stop', ''), '')
        self.assertEqual(row['road_user_collision'], '0')


if __name__ == '__main__':
    unittest.main()
