"""Synthetic-frame regression audit using actual M16 OpenDRIVE geometry.

These tests do not connect to, run, or validate a live VTD simulation. They
isolate lane identity and turn-signal scoring. The former false deductions at
genuine laneSection predecessor links are permanent passing regressions.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/dashboard'))

from live_assessment import LiveAssessment
from opendrive_assessment import OpenDriveMap


class M16LaneContinuityAuditTests(unittest.TestCase):
    """Real map lane topology with controlled, synthetic RDB-equivalent frames."""

    @classmethod
    def setUpClass(cls):
        cls.map_path = ROOT / 'src/data/map/HL_FMA_VTD_LivingLab.xodr'
        cls.map_data = OpenDriveMap(cls.map_path)

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='hl-vtd-lane-audit-')
        self.addCleanup(self.directory.cleanup)
        output = Path(self.directory.name)
        rules = {
            'base_score': 500,
            'normal_speed_kmh': 50,
            'school_speed_kmh': 30,
            'map': {'xodr': str(self.map_path)},
            'completion': {'mode': 'finish_line'},
            'session': {
                'start_trigger': 'controller_packet_arrival',
                'max_sample_gap_s': .5,
                'time_limit_s': 900,
                'stop_speed_kmh': 0,
            },
            'metadata': {
                'rear_axle_offset_m': 0,
                'object_feed_complete': True,
                'school_zones': [],
                'crosswalks': [],
                'stop_lines': [],
                'sections': [{
                    'id': '1',
                    'polygon': [
                        [-10000, -10000], [10000, -10000],
                        [10000, 10000], [-10000, 10000],
                    ],
                }],
            },
        }
        rules_path = output / 'rules.yaml'
        rules_path.write_text(yaml.safe_dump(rules), encoding='utf-8')
        self.engine = LiveAssessment(rules_path, output / 'assessment')
        self.addCleanup(self.engine.close)
        self.engine.on_control_received({'steering': 0, 'accel': 0, 'indicator': 0})

    def send_frame(self, index, road_id, road_s, lane_id, indicator, *, reverse=False,
                   position=None):
        center = self.map_data.lane_center_world(road_id, road_s, lane_id)
        self.assertIsNotNone(center)
        x, y = center if position is None else position
        reference_heading = self.map_data.reference_pose(road_id, road_s)[2]
        # Positive lanes normally travel against the OpenDRIVE reference.
        heading = reference_heading + (0 if reverse else math.pi)
        ego = {
            'id': 1, 'name': 'Ego', 'category': 1, 'type': 1,
            'x': x, 'y': y, 'z': 0, 'h': heading, 'p': 0, 'r': 0,
            'length': 4.848, 'width': 1.886, 'height': 1.5,
            'off_x': 1, 'off_y': 0, 'off_z': .75, 'speed': 1,
        }
        self.engine.on_frame({
            'frame': index,
            'time': index * .1,
            'version': 0x126,
            'objects': {1: ego},
            'road': {1: {
                'road': road_id, 'lane': lane_id, 's': road_s,
                't': 0, 'offset': 0, 'heading': 0,
                'flags': 1 if reverse else 2,
            }},
            'lanes': {}, 'lights': {1: indicator}, 'signals': {},
            'packages': [5, 9, 10, 2],
        })

    def item(self, number):
        return next(item for item in self.engine.snapshot()['official_grid']
                    if item['number'] == number)

    def assert_continuation_geometry(self, road_id, boundary):
        high_section = next(
            section for section in self.map_data.roads[road_id].findall('lanes/laneSection')
            if abs(float(section.get('s')) - boundary) < .00001
        )
        predecessor = high_section.find('./left/lane[@id="1"]/link/predecessor')
        self.assertIsNotNone(predecessor)
        self.assertEqual(predecessor.get('id'), '2')
        before = self.map_data.lane_center_world(road_id, boundary + .01, 1)
        after = self.map_data.lane_center_world(road_id, boundary - .01, 2)
        self.assertLess(math.dist(before, after), .03)

    def replay_continuation(self, road_id, boundary):
        self.assert_continuation_geometry(road_id, boundary)
        for index in range(34):
            lane_id = 1 if index <= 30 else 2
            road_s = boundary + (.31 - index * .01) - (.01 if index >= 31 else 0)
            self.send_frame(index, road_id, road_s, lane_id, 'OFF')

    def assert_no_lane_change_deduction(self):
        self.assertEqual(self.item(7)['penalty'], 0)
        self.assertEqual(self.item(8)['penalty'], 0)
        self.assertFalse(any(
            event['event'] in {'turn_signal_violation', 'solid_line_crossing'}
            for event in self.engine.events
        ))

    def replay_actual_lane_change(self, indicator, *, reverse=False):
        road_id, road_s = 2626, 7.883
        old_center = self.map_data.lane_center_world(road_id, road_s, 2)
        new_center = self.map_data.lane_center_world(road_id, road_s, 1)
        self.assertGreater(math.dist(old_center, new_center), 2.9)
        # Observe the indicator for >3 seconds, then move gradually across a
        # physical boundary within one laneSection. ROAD_POS changes near the
        # shared boundary and persists for the three-frame confirmation window.
        for index in range(44):
            fraction = min(1.0, max(0.0, (index - 30) / 10))
            position = tuple(
                old + fraction * (new - old)
                for old, new in zip(old_center, new_center)
            )
            lane_id = 2 if fraction <= .5 else 1
            self.send_frame(index, road_id, road_s, lane_id, indicator,
                            reverse=reverse, position=position)
        transition_rows = [
            row for row in self.engine.rows if row.get('turn_signal_expected')
        ]
        self.assertEqual(len(transition_rows), 1)
        return transition_rows[0]

    def test_m16_boundaries_really_are_same_lane_predecessor_links(self):
        for road_id, boundary in ((2469, 41.201), (2626, 63.901)):
            with self.subTest(road_id=road_id):
                self.assert_continuation_geometry(road_id, boundary)

    def test_m16_road_2469_lane_renumber_must_not_require_indicator(self):
        self.replay_continuation(2469, 41.201)
        self.assert_no_lane_change_deduction()

    def test_m16_road_2626_lane_renumber_must_not_require_indicator(self):
        self.replay_continuation(2626, 63.901)
        self.assert_no_lane_change_deduction()

    def test_actual_positive_lane_change_with_left_indicator_passes(self):
        transition = self.replay_actual_lane_change('LEFT')
        self.assertEqual(transition['turn_signal_expected'], 'LEFT')
        self.assertEqual(transition['turn_signal_violation'], '0')
        self.assertEqual(self.item(7)['penalty'], 0)

    def test_actual_positive_lane_change_without_indicator_loses_three_points(self):
        transition = self.replay_actual_lane_change('OFF')
        self.assertEqual(transition['turn_signal_expected'], 'LEFT')
        self.assertEqual(transition['turn_signal_violation'], '1')
        self.assertEqual(self.item(7)['penalty'], -3)

    def test_opposite_vehicle_heading_requires_right_indicator(self):
        transition = self.replay_actual_lane_change('RIGHT', reverse=True)
        self.assertEqual(transition['turn_signal_expected'], 'RIGHT')
        self.assertEqual(transition['turn_signal_violation'], '0')
        self.assertEqual(self.item(7)['penalty'], 0)

    def test_opposite_vehicle_heading_with_left_indicator_loses_three_points(self):
        transition = self.replay_actual_lane_change('LEFT', reverse=True)
        self.assertEqual(transition['turn_signal_expected'], 'RIGHT')
        self.assertEqual(transition['turn_signal_violation'], '1')
        self.assertEqual(self.item(7)['penalty'], -3)


if __name__ == '__main__':
    unittest.main()
