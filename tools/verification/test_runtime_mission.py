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

    def test_final_junction_connector_is_the_runtime_target(self):
        source=fixtures.lane(1,0,10)
        connector=fixtures.lane(100,10,20,junction=1)
        exit_=fixtures.lane(2,20,30)
        geometry=dict([source,connector,exit_])
        network=Network(geometry,[(source[0],connector[0]),
                                  (connector[0],exit_[0])])
        result=network.build(fixtures.mission([(1,0),(15,0)]))
        legs=runtime_legs(network,result)
        self.assertEqual({tuple(t['lane']) for t in legs[-1]['targets']},
                         {connector[0]})
        self.assertAlmostEqual(legs[-1]['targets'][0]['road_s'],5.)
        self.assertEqual(legs[-1]['witness_route'],
                         [source[0],connector[0]])

    def test_overlapping_spawn_connectors_are_runtime_allowed(self):
        connector_a=fixtures.lane(100,0,10,junction=1)
        connector_b=fixtures.lane(101,0,10,junction=1)
        exit_=fixtures.lane(2,10,30)
        geometry=dict([connector_a,connector_b,exit_])
        network=Network(geometry,[(connector_a[0],exit_[0]),
                                  (connector_b[0],exit_[0])])
        result=network.build(fixtures.mission([(5,0),(29,0)]))
        legs=runtime_legs(network,result)
        self.assertIn(connector_a[0],legs[0]['allowed'])
        self.assertIn(connector_b[0],legs[0]['allowed'])
        self.assertEqual(legs[0]['witness_route'][0],connector_a[0])

    def test_validation_failure_cannot_be_silently_ignored(self):
        geometry, edges = fixtures.WholeMissionTests().parallel()
        network = Network(geometry, edges)
        result = network.build(fixtures.mission([(1, 0), (9, 0), (21, 0), (29, 0)]))
        result['validation']['surface_interfaces_connected'] = False
        with self.assertRaises(ValueError):
            runtime_legs(network, result)


if __name__ == '__main__':
    unittest.main()
