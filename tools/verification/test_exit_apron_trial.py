"""Surface-only M12 G1 experiment: never widen graph permissions."""
import copy
import unittest

from prepare_runtime_mission import apply_surface_trial, same_exit_surface_spans
from test_whole_mission_corridors import lane
from whole_mission_corridors import Network


class ExitApronTrialTests(unittest.TestCase):
    def fixture(self):
        geometry = dict([
            lane(2625, -20, -10, lane_id=-4),
            lane(2653, -10, 20, junction=72),
            lane(2445, 20, 40, lane_id=4),
            lane(2445, 20, 40, y=3, lane_id=3),
            lane(2445, 20, 40, y=6, lane_id=2),
            lane(2659, 0, 20, y=6, lane_id=-1, junction=72),
            lane(2659, 0, 20, y=3, lane_id=-2, junction=72),
            # Correct endpoint but opposite approach direction is not apron.
            lane(900, 40, 20, y=6, junction=72),
            # Same direction, other intersection: also excluded.
            lane(901, 0, 20, y=6, junction=73),
        ])
        # The real trial's exit lane is section 3.
        for k in list(geometry):
            if k[0] in (2625, 2445):
                geometry[k[0], 3, k[2]] = geometry.pop(k)
        edges = [((2625, 3, -4), (2653, 0, -1)),
                 ((2653, 0, -1), (2445, 3, 4)),
                 ((2659, 0, -1), (2445, 3, 2)),
                 ((2659, 0, -2), (2445, 3, 3)),
                 ((900, 0, -1), (2445, 3, 2)),
                 ((901, 0, -1), (2445, 3, 2))]
        network = Network(geometry, edges)
        leg = dict(allowed=[(2625, 3, -4), (2653, 0, -1),
                            (2445, 3, 2), (2445, 3, 3), (2445, 3, 4)],
                   gateway_lanes=[(2653, 0, -1)],
                   targets=[dict(lane=(2445, 3, 4), road_s=2.)],
                   witness_route=[(2625, 3, -4), (2653, 0, -1), (2445, 3, 4)])
        result = dict(legs=[dict(declared_gateway_index=1, entry_edges=[edges[0]])])
        return network, result, [leg]

    def test_opt_in_only_and_no_route_graph_change(self):
        network, result, legs = self.fixture()
        original = copy.deepcopy(legs)
        apply_surface_trial(network, result, legs)
        self.assertEqual(legs, original)
        graph = copy.deepcopy(dict(network.search_graph))
        apply_surface_trial(network, result, legs, 'm12-g1-exit-apron')
        for k in ('allowed', 'gateway_lanes', 'targets', 'witness_route'):
            self.assertEqual(legs[0][k], original[0][k])
        self.assertEqual(dict(network.search_graph), graph)
        self.assertEqual({s['lane'] for s in legs[0]['surface_only_spans']},
                         {(2659, 0, -1), (2659, 0, -2)})

    def test_clip_terminal_arc_and_reject_wrong_direction_or_junction(self):
        network, _, legs = self.fixture()
        spans = same_exit_surface_spans(network, legs[0], {2659, 900, 901})
        self.assertEqual(len(spans), 2)
        for s in spans:
            self.assertAlmostEqual(s['road_s_min'], 5.)
            self.assertAlmostEqual(s['road_s_max'], 20.)
            self.assertAlmostEqual(s['arc_length_m'], 15.)
            self.assertNotIn(s['lane'], legs[0]['allowed'])

    def test_wrong_mission_cannot_activate_trial(self):
        network, result, legs = self.fixture()
        legs[0]['targets'][0]['lane'] = (2445, 3, 3)
        with self.assertRaises(ValueError):
            apply_surface_trial(network, result, legs, 'm12-g1-exit-apron')

    def test_broken_endpoint_cannot_be_painted_over(self):
        network, _, legs = self.fixture()
        network.geometry[2659, 0, -1]['points'][-1, 0] += .2
        spans = same_exit_surface_spans(network, legs[0], {2659})
        self.assertEqual([s['lane'] for s in spans], [(2659, 0, -2)])


if __name__ == '__main__':
    unittest.main()
