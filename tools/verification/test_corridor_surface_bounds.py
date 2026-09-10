"""Regressions for excess surface, independent of mission and road numbers."""
import unittest
import numpy as np
from corridor_surface_bounds import slice_points, surface_segments
from test_whole_mission_corridors import lane, mission
from whole_mission_corridors import Network


class CorridorBoundsTests(unittest.TestCase):
    def pieces(self, result, geo):
        return surface_segments(result, geo, {k for g in result['legs'] for k in g['allowed']})

    def test_ingress_excludes_upstream_merge_but_keeps_parallel_road_width(self):
        for offset in (0, 6000):
            geo = dict([lane(offset+1, 0, 10), lane(offset+10, 10, 20, junction=1),
                        lane(offset+2, 20, 40), lane(offset+2, 20, 40, 3, -2),
                        lane(offset+99, -30, 20, 8, y1=0)])
            k = lambda road, n=-1: (offset+road, 0, n)
            edges = [(k(1), k(10)), (k(10), k(2)), (k(99), k(2))]
            d = Network(geo, edges).build(mission([(1, 0), (9, 0), (21, 0), (35, 0)]))
            self.assertTrue(d['initialized_ok'])
            p = self.pieces(d, geo)
            self.assertNotIn(k(99), p)
            self.assertIn(k(2, -2), p)
            self.assertAlmostEqual(p[k(2)][0][-1, 0], 35)

    def test_egress_excludes_unused_downstream_fork(self):
        geo = dict([lane(1, 0, 10), lane(99, 10, 35), lane(10, 10, 20, junction=1, y1=10), lane(2, 20, 40, 10)])
        edges = [((1, 0, -1), (99, 0, -1)), ((1, 0, -1), (10, 0, -1)), ((10, 0, -1), (2, 0, -1))]
        d = Network(geo, edges).build(mission([(1, 0), (9, 0), (21, 10), (35, 10)]))
        self.assertTrue(d['initialized_ok'])
        self.assertNotIn((99, 0, -1), self.pieces(d, geo))

    def test_junction_endpoints_cut_before_spawn_and_after_finish(self):
        geo = dict([lane(10, 0, 10, junction=1), lane(1, 10, 30), lane(20, 30, 40, junction=2)])
        edges = [((10, 0, -1), (1, 0, -1)), ((1, 0, -1), (20, 0, -1))]
        d = Network(geo, edges).build(mission([(8.13, 0), (31.87, 0)]))
        self.assertTrue(d['initialized_ok'])
        p = self.pieces(d, geo)
        self.assertAlmostEqual(p[(10, 0, -1)][0][0, 0], 8.13)
        self.assertAlmostEqual(p[(20, 0, -1)][0][-1, 0], 31.87)
        self.assertAlmostEqual(p[(1, 0, -1)][0][0, 0], 10)
        self.assertAlmostEqual(p[(1, 0, -1)][0][-1, 0], 30)

    def test_alias_uses_own_station_and_preserves_connector_chain(self):
        geo = dict([lane(10, 0, 10, junction=1), lane(11, 0, 10, .04, junction=1),
                    lane(12, 10, 20, junction=1), lane(1, 20, 40)])
        geo[(11, 0, -1)]['points'][:, 3] += 100
        edges = [((10, 0, -1), (12, 0, -1)), ((11, 0, -1), (12, 0, -1)), ((12, 0, -1), (1, 0, -1))]
        d = Network(geo, edges).build(mission([(8.13, 0), (35, 0)]))
        self.assertTrue(d['initialized_ok'])
        p = self.pieces(d, geo)
        for road in (10, 11):
            self.assertAlmostEqual(p[(road, 0, -1)][0][0, 0], 8.13)
        self.assertAlmostEqual(p[(12, 0, -1)][0][0, 0], 10)
        self.assertAlmostEqual(p[(12, 0, -1)][0][-1, 0], 20)

    def test_positive_lane_endpoints_follow_decreasing_road_s(self):
        geo = dict([lane(10, 40, 30, lane_id=1, junction=1), lane(1, 30, 10, lane_id=1),
                    lane(20, 10, 0, lane_id=1, junction=2)])
        for g in geo.values():
            g['points'][:, 3] = g['points'][::-1, 3]
        edges = [((10, 0, 1), (1, 0, 1)), ((1, 0, 1), (20, 0, 1))]
        d = Network(geo, edges).build(mission([(31.87, 0), (8.13, 0)]))
        self.assertTrue(d['initialized_ok'])
        p = self.pieces(d, geo)
        self.assertAlmostEqual(p[(10, 0, 1)][0][0, 0], 31.87)
        self.assertAlmostEqual(p[(20, 0, 1)][0][-1, 0], 8.13)

    def test_slice_interpolates_both_boundaries_between_samples(self):
        _, g = lane(1, 0, 10)
        p = slice_points(g['points'], 1.13, 1.27)
        self.assertEqual(len(p), 2)
        np.testing.assert_allclose(p[:, 0], [1.13, 1.27])
        np.testing.assert_allclose(p[:, 5], [1.13, 1.27])
        np.testing.assert_allclose(p[:, 7], [1.13, 1.27])

    def test_gateway_from_excluded_upstream_branch_is_removed_too(self):
        geo = dict([lane(1, 0, 10), lane(99, -20, 0, 3, y1=0),
                    lane(10, 10, 20, junction=1), lane(98, 0, 20, junction=1), lane(2, 20, 40)])
        edges = [((99, 0, -1), (1, 0, -1)), ((1, 0, -1), (10, 0, -1)),
                 ((99, 0, -1), (98, 0, -1)), ((98, 0, -1), (2, 0, -1)), ((10, 0, -1), (2, 0, -1))]
        d = Network(geo, edges).build(mission([(1, 0), (9, 0), (21, 0), (35, 0)]))
        self.assertTrue(d['initialized_ok'])
        p = self.pieces(d, geo)
        self.assertNotIn((99, 0, -1), p)
        self.assertNotIn((98, 0, -1), p)
        self.assertEqual(d['legs'][0]['connection_lanes'], [(10, 0, -1)])

    def test_repeated_component_keeps_two_required_visits_without_filling_gap(self):
        geo = dict([lane(1, 0, 10), lane(10, 10, 20, junction=1),
                    lane(2, 20, 30), lane(20, 30, 0, junction=2)])
        edges = [((1, 0, -1), (10, 0, -1)), ((10, 0, -1), (2, 0, -1)),
                 ((2, 0, -1), (20, 0, -1)), ((20, 0, -1), (1, 0, -1))]
        d = Network(geo, edges).build(mission([(8, 0), (9, 0), (21, 0), (29, 0), (1, 0), (2, 0)]))
        self.assertTrue(d['initialized_ok'])
        p = self.pieces(d, geo)[(1, 0, -1)]
        self.assertEqual(len(p), 2)
        self.assertEqual([(v[:, 0].min(), v[:, 0].max()) for v in p], [(0., 2.), (8., 10.)])

    def test_junction_endpoint_keeps_connected_parallel_width_with_same_cut(self):
        geo = dict([lane(10, 0, 10, junction=1), lane(10, 0, 10, 3, -2, junction=1),
                    lane(1, 10, 30), lane(1, 10, 30, 3, -2),
                    lane(20, 30, 40, junction=2), lane(20, 30, 40, 3, -2, junction=2)])
        edges = [((10, 0, n), (1, 0, n)) for n in (-1, -2)]
        edges += [((1, 0, n), (20, 0, n)) for n in (-1, -2)]
        edges += [((1, 0, -1), (1, 0, -2)), ((1, 0, -2), (1, 0, -1))]
        net = Network(geo, edges)
        d = net.build(mission([(8.13, 0), (31.87, 0)]))
        self.assertTrue(d['initialized_ok'])
        p = self.pieces(d, geo)
        for n in (-1, -2):
            self.assertAlmostEqual(p[(10, 0, n)][0][0, 0], 8.13)
            self.assertAlmostEqual(p[(20, 0, n)][0][-1, 0], 31.87)
        self.assertEqual(d['endpoint_connections']['start']['aliases'], [(10, 0, -1)])
        self.assertEqual(d['endpoint_connections']['start']['surface_extensions'], [(10, 0, -2)])

    def test_parallel_endpoint_lane_to_other_road_is_excluded(self):
        geo = dict([lane(10, 0, 10, junction=1), lane(10, 0, 10, 3, -2, junction=1),
                    lane(1, 10, 30), lane(2, 10, 30, 3)])
        edges = [((10, 0, -1), (1, 0, -1)), ((10, 0, -2), (2, 0, -1))]
        d = Network(geo, edges).build(mission([(8.13, 0), (28, 0)]))
        self.assertTrue(d['initialized_ok'])
        self.assertNotIn((10, 0, -2), self.pieces(d, geo))

    def test_parallel_endpoint_lane_across_physical_gap_is_excluded(self):
        geo = dict([lane(10, 0, 10, junction=1), lane(10, 0, 10, 8, -2, junction=1),
                    lane(1, 10, 30), lane(1, 10, 30, 8, -2)])
        edges = [((10, 0, n), (1, 0, n)) for n in (-1, -2)]
        edges += [((1, 0, -2), (1, 0, -1))]
        d = Network(geo, edges).build(mission([(8.13, 0), (28, 0)]))
        self.assertTrue(d['initialized_ok'])
        self.assertNotIn((10, 0, -2), self.pieces(d, geo))

    def test_disjoint_ranges_do_not_paint_gap_on_revisited_lane(self):
        k, g = lane(1, 0, 10)
        d = dict(surface_ranges=[dict(key=k, ranges=[dict(road_s_min=0, road_s_max=2),
                                                    dict(road_s_min=8, road_s_max=10)])])
        p = surface_segments(d, {k: g}, {k})[k]
        self.assertEqual(len(p), 2)
        self.assertLessEqual(p[0][:, 0].max(), 2)
        self.assertGreaterEqual(p[1][:, 0].min(), 8)


if __name__ == '__main__':
    unittest.main()
