"""Latest user rule: the configured final checkpoint ends the assessment."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/dashboard'))
from live_assessment import LiveAssessment
from test_dashboard_assessment import frame


class CheckpointTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)
        route = self.path/'route.csv'
        route.write_text('x,y,s,road_id,lane_id\n0,0,0,1,-1\n50,0,50,1,-1\n100,0,100,1,-1\n')
        self.rules = yaml.safe_load((ROOT/'tools/dashboard/assessment_rules.yaml').read_text())
        self.rules.pop('school_zones_file', None)
        self.rules['map'] = {}
        self.rules['route'] = dict(csv=str(route), section_count=5)
        self.rules['metadata'] = dict(object_feed_complete=True, school_zones=[], crosswalks=[],
                                     stop_lines=[], centerlines=[], sidewalk_boundaries=[])
        self.engine = self.make_engine(self.rules)
        self.engine.on_control_received(dict(steering=0, accel=0, indicator=0))

    def make_engine(self, rules):
        path = self.path/'rules.yaml'
        path.write_text(yaml.safe_dump(rules))
        return LiveAssessment(path, self.path/'output')

    def tearDown(self):
        self.engine.close()
        self.tmp.cleanup()

    def test_default_endpoint_does_not_require_stop_line(self):
        payload = self.engine.snapshot()
        self.assertEqual(payload['completion']['checkpoint']['x'], 100)
        self.assertEqual(payload['completion']['tolerance_m'], 1.5)
        self.assertNotIn('finish_line', payload['missing_metadata'])
        self.assertEqual(payload['finish_candidates'], [])
        self.assertIsNone(payload['finish_line'])

    def test_arrival_finishes_without_rear_axle_and_freezes(self):
        e = self.engine
        e.on_frame(frame(0, 97, speed=10))
        e.on_frame(frame(.2, 98.5, speed=10))
        self.assertEqual(e.status, 'COMPLETED')
        self.assertEqual(e.rows[-1]['rear_axle_available'], '0')
        self.assertEqual(e.rows[-1]['evaluation_section_id'], '5')
        self.assertEqual(e.rows[-1]['goal_source'], 'final_checkpoint')
        last = dict(e.rows[-1])
        e.on_frame(frame(.4, 110, speed=10))
        self.assertEqual(e.rows[-1], last)

    def test_swept_entry_does_not_miss_checkpoint_between_samples(self):
        e = self.engine
        e.on_frame(frame(0, 96, speed=40))
        e.on_frame(frame(.1, 104, speed=40))
        self.assertEqual(e.status, 'COMPLETED')

    def test_gap_and_teleport_do_not_complete_or_complete_one_frame_later(self):
        for mode in ('gap', 'teleport'):
            with self.subTest(mode=mode):
                self.engine.close()
                self.engine = self.make_engine(self.rules)
                e = self.engine
                e.on_control_received(dict(steering=0))
                e.on_frame(frame(0, 97 if mode == 'gap' else 0, speed=0))
                if mode == 'gap': e.on_gap()
                e.on_frame(frame(.1, 100, speed=0))
                e.on_frame(frame(.2, 100.1, speed=0))
                self.assertEqual(e.status, 'RUNNING')

    def test_reverse_entry_and_parallel_lane_do_not_complete(self):
        e = self.engine
        e.on_frame(frame(0, 103, speed=10))
        e.on_frame(frame(.2, 100, speed=10))
        self.assertEqual(e.status, 'RUNNING')
        e.on_gap()
        first, second = frame(.3, 97, speed=10), frame(.5, 100, speed=10)
        first['objects'][1]['y'] = second['objects'][1]['y'] = 2
        e.on_frame(first)
        e.on_frame(second)
        self.assertEqual(e.status, 'RUNNING')

    def test_wrong_road_does_not_complete(self):
        e = self.engine
        e.on_frame(frame(0, 97, speed=10))
        current = frame(.2, 100, speed=10)
        current['road'][1] = dict(road=2, lane=-1, s=100, t=-1, heading=0)
        e.on_frame(current)
        self.assertEqual(e.status, 'RUNNING')

    def test_old_finish_line_and_intermediate_point_do_not_end_session(self):
        e = self.engine
        e.meta['finish_line'] = dict(begin=[50,5], end=[50,-5])
        e.meta['rear_axle_offset_m'] = 0
        e.on_frame(frame(0, 48, speed=10))
        e.on_frame(frame(.2, 51, speed=10))
        self.assertEqual(e.status, 'RUNNING')
        self.assertIsNone(e.snapshot()['finish_line'])

    def test_no_control_no_start_or_finish(self):
        self.engine.pending_control = None
        self.engine.on_frame(frame(0, 97, speed=10))
        self.engine.on_frame(frame(.2, 100, speed=10))
        self.assertEqual(self.engine.status, 'WAITING')

    def test_invalid_checkpoint_configuration_is_rejected(self):
        for tolerance in (0, -1, float('nan'), float('inf'), True):
            with self.subTest(tolerance=tolerance):
                rules = copy.deepcopy(self.rules)
                rules['completion']['tolerance_m'] = tolerance
                with self.assertRaises(ValueError): self.make_engine(rules)
        rules = copy.deepcopy(self.rules)
        rules['route'] = {}
        with self.assertRaises(ValueError): self.make_engine(rules)


if __name__ == '__main__':
    unittest.main()
