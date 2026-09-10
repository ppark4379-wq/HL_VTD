import copy
import math
import tempfile
from pathlib import Path
import unittest
from check_start_readiness import inspect_start, inspect_window, runtime_lanes_from_csv
from corridor_surface_bounds import surface_segments
from generate_start_stress_suite import probe_observation
from test_whole_mission_corridors import lane, mission
from render_mission_corridors import RoadBackground, band
from whole_mission_corridors import Network


class StartReadinessTests(unittest.TestCase):
    def fixture(self, x=5.):
        geo = dict([lane(1, 0, 10), lane(2, 10, 30)])
        net = Network(geo, [((1, 0, -1), (2, 0, -1))])
        m = net.build(mission([(x, 0), (25, 0)]))
        bg = RoadBackground()
        for k, g in geo.items():
            bg.driving[k] = bg.path(band(g['points'])*[1, -1])
        obs = probe_observation(dict(x=x, y=0., heading=0.), (1, 0, -1))
        return geo, net, m, bg, obs

    def test_exact_section_seam_retains_identity_without_upstream_fill(self):
        for offset in (0., .25):
            geo, net, m, bg, obs = self.fixture(10.+offset)
            self.assertTrue(m['initialized_ok'])
            k = tuple(m['checkpoint_matches'][0]['nearest_lane'])
            self.assertIn(k, m['corridor_groups'][0]['lanes'])
            pieces = surface_segments(m, geo, set(m['legs'][0]['allowed']))
            self.assertTrue(all(p[:, 0].min() >= 10.+offset-1e-7 for chunks in pieces.values() for p in chunks))

    def test_exact_seam_with_observed_previous_lane_can_support_body(self):
        geo, net, m, bg, obs = self.fixture(10.)
        r = inspect_start(m, obs, net, bg, set(m['legs'][0]['allowed']))
        self.assertEqual(r['status'], 'STATIC_START_OK')

    def test_dropped_real_start_lane_is_blocked_despite_valid_mission_flag(self):
        geo, net, m, bg, obs = self.fixture()
        m['corridor_groups'][0]['lanes'].remove((1, 0, -1))
        r = inspect_start(m, obs, net, bg)
        self.assertEqual(r['status'], 'BLOCKED')
        self.assertIn('EGO_LANE_MISSING_FROM_FIRST_CORRIDOR', r['findings'])

    def test_actual_runtime_lane_set_is_checked_separately(self):
        geo, net, m, bg, obs = self.fixture()
        r = inspect_start(m, obs, net, bg, {(2, 0, -1)})
        self.assertIn('EGO_LANE_MISSING_FROM_RUNTIME_LEG_0', r['findings'])

    def test_centre_inside_but_body_outside_is_blocked(self):
        geo, net, m, bg, obs = self.fixture()
        obs = probe_observation(dict(x=5., y=1.3, heading=0.), (1, 0, -1))
        r = inspect_start(m, obs, net, bg)
        self.assertIn('VEHICLE_BODY_OUTSIDE_START_SURFACE', r['findings'])
        self.assertNotIn('CENTER_OUTSIDE_START_SURFACE', r['findings'])

    def test_missing_or_nonfinite_evidence_never_passes(self):
        geo, net, m, bg, obs = self.fixture()
        del obs['ego']['obb']
        self.assertEqual(inspect_start(m, obs, net, bg)['status'], 'INCOMPLETE')
        obs['ego']['x'] = math.nan
        self.assertEqual(inspect_start(m, obs, net, bg)['status'], 'INCOMPLETE')

    def frames(self):
        obs = self.fixture()[-1]
        frames = []
        for i in range(5):
            f = copy.deepcopy(obs); f.update(frame=i, time_s=float(i))
            f['ego'].update(x=5.+i*.4, speed=.4 if i else 0.)
            f['decision'].update(plan_valid=True, target_speed=1.)
            frames.append(f)
        return frames

    def test_valid_motion_with_runtime_evidence_passes_observation_only(self):
        r = inspect_window(self.frames(), dict(status='STATIC_START_OK'), True)
        self.assertEqual(r['status'], 'OBSERVED_START_OK')
        self.assertEqual(inspect_window(self.frames(), dict(status='STATIC_START_OK'), False)['status'], 'INCOMPLETE')

    def test_separate_plan_and_motion_frames_do_not_make_a_pass(self):
        frames = self.frames()
        for i, f in enumerate(frames):
            f['decision']['plan_valid'] = i % 2 == 0
            if i % 2 == 0:
                f['ego']['speed'] = 0.
        self.assertEqual(inspect_window(frames, dict(status='STATIC_START_OK'), True)['status'], 'BLOCKED')

    def test_invalid_plans_are_not_mislabelled_as_traffic_wait(self):
        frames = self.frames()
        for f in frames:
            f['decision'].update(plan_valid=False, stop_reason=1, target_speed=0.)
        self.assertEqual(inspect_window(frames, dict(status='STATIC_START_OK'), True)['reason'], 'no_valid_local_plan_observed')

    def test_traffic_wait_and_stale_frames_are_not_passes(self):
        frames = self.frames()
        for f in frames:
            f['decision'].update(stop_reason=2, target_speed=0.)
        self.assertEqual(inspect_window(frames, dict(status='STATIC_START_OK'), True)['status'], 'WAIT_CONDITION')
        frames[-1]['frame'] = frames[-2]['frame']
        self.assertEqual(inspect_window(frames, dict(status='STATIC_START_OK'), True)['status'], 'INCOMPLETE')

    def test_late_capture_does_not_certify_start(self):
        frames = self.frames(); frames[0]['ego']['speed'] = 2.
        self.assertEqual(inspect_window(frames, dict(status='STATIC_START_OK'), True)['reason'], 'capture_started_after_motion')

    def test_raw_rdb_lane_is_checked_when_logged(self):
        geo, net, m, bg, obs = self.fixture()
        obs['ego']['rdb_lane'] = [999, 0, -1]
        self.assertIn('RDB_LANE_MISSING_FROM_FIRST_CORRIDOR', inspect_start(m, obs, net, bg)['findings'])

    def test_runtime_csv_does_not_union_all_legs(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'static_map.csv'
            p.write_text('layer,group,road_id,section_index,lane_id\ncorridor,0:1:0:-1,1,0,-1\ncorridor,1:9:0:-1,9,0,-1\ngateway,0:8:0:-1,8,0,-1\n')
            self.assertEqual(runtime_lanes_from_csv(p), {(1, 0, -1)})


    def test_runtime_csv_rejects_other_mission_checkpoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'static_map.csv'
            p.write_text('layer,group,road_id,section_index,lane_id,x,y\ncheckpoint,0,0,0,0,100,100\ncorridor,0:1:0:-1,1,0,-1,0,0\n')
            with self.assertRaisesRegex(ValueError, 'does not match'):
                runtime_lanes_from_csv(p, dict(x=0., y=0.))


if __name__ == '__main__':
    unittest.main()
