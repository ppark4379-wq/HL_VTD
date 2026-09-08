import unittest
from prepare_runtime_mission import runtime_legs
import test_whole_mission_corridors as fixtures
from whole_mission_corridors import Network


class RuntimeMissionTests(unittest.TestCase):
    def test_parallel_exits_not_arbitrary_cp_lane(self):
        geometry, edges = fixtures.WholeMissionTests().parallel()
        network = Network(geometry, edges)
        result = network.build(fixtures.mission([(1, 3), (9, 3), (21, 3), (29, 3)]))
        legs = runtime_legs(network, result)
        self.assertEqual(len(legs), 2)
        self.assertEqual({tuple(t['lane']) for t in legs[0]['targets']},
                         {(2, 0, -1), (2, 0, -2)})
        self.assertEqual(legs[0]['gateway_lanes'], [(10, 0, -1), (11, 0, -1)])
        self.assertTrue(legs[-1]['final'])
        self.assertTrue(all(t['road_s'] == 9 for t in legs[-1]['targets']))

    def test_preview_with_odd_checkpoints_cannot_drive(self):
        geometry, edges = fixtures.WholeMissionTests().parallel()
        network = Network(geometry, edges)
        result = network.build(fixtures.mission([(1, 0), (9, 0), (29, 0)]))
        with self.assertRaises(ValueError):
            runtime_legs(network, result)

    def test_validation_failure_cannot_be_silently_ignored(self):
        geometry, edges = fixtures.WholeMissionTests().parallel()
        network = Network(geometry, edges)
        result = network.build(fixtures.mission([(1, 0), (9, 0), (21, 0), (29, 0)]))
        result['validation']['surface_interfaces_connected'] = False
        with self.assertRaises(ValueError):
            runtime_legs(network, result)


if __name__ == '__main__':
    unittest.main()
