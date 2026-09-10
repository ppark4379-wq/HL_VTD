"""Read-only topology/geometry regression; no generated controller inputs."""
import gzip
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/dashboard'))
from opendrive_assessment import OpenDriveMap


def section(sidewalk=True, origin_lane=-1):
    return ('<laneSection s="0"><center><lane id="0"><roadMark sOffset="0" '
            'type="solid" color="yellow"/></lane></center><right>'
            f'<lane id="{origin_lane}" type="driving"><link><predecessor id="-1"/>'
            '<successor id="-1"/></link><width sOffset="0" a="3"/></lane>'
            + ('<lane id="-2" type="sidewalk"><width sOffset="0" a="4"/></lane>' if sidewalk else '')
            + '</right></laneSection>')


def road(rid, x, y=0, z=0, links='', junction=-1, sidewalk=True):
    return (f'<road id="{rid}" length="20" junction="{junction}"><link>{links}</link>'
            f'<planView><geometry s="0" x="{x}" y="{y}" hdg="0" length="20"><line/></geometry></planView>'
            f'<elevationProfile><elevation s="0" a="{z}"/></elevationProfile>'
            f'<lanes>{section(sidewalk)}</lanes></road>')


def ego(x=19, y=-2.5):
    return dict(x=x, y=y, z=0, h=0, length=4, width=2, off_x=0, off_y=0)


class ConnectedPatchTests(unittest.TestCase):
    def map_from(self, text):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'map.xodr'
        path.write_text('<OpenDRIVE>' + text + '</OpenDRIVE>')
        return OpenDriveMap(path)

    def linked_map(self, *, sidewalk=True, x=20, z=0):
        return self.map_from(
            road(1, 0, links='<successor elementType="road" elementId="2" contactPoint="start"/>') +
            road(2, x, z=z, sidewalk=sidewalk,
                 links='<predecessor elementType="road" elementId="1" contactPoint="end"/>'))

    def test_linked_road_end_keeps_finite_sidewalk_depth(self):
        m = self.linked_map()
        result = m.measure(ego(), dict(road=1, s=19, lane=-1), -1)
        self.assertEqual(result['sidewalk_measurement_status'], 'AVAILABLE', result)
        self.assertAlmostEqual(result['sidewalk_intrusion_m'], .5)
        self.assertEqual(result['centerline_intrusion_m'], 0)
        self.assertEqual(result['boundary_patch_road_ids'], [1, 2])

    def test_linked_road_end_preserves_true_centerline_intrusion(self):
        m = self.linked_map()
        result = m.measure(ego(y=-.4), dict(road=1, s=19, lane=-1), -1)
        # Only the road-side portion of the next patch can certify coverage;
        # corners beyond the outer road domain must remain unavailable.
        self.assertEqual(result['centerline_measurement_status'], 'UNAVAILABLE')

    def test_missing_neighbor_sidewalk_does_not_become_zero(self):
        m = self.linked_map(sidewalk=False)
        result = m.measure(ego(y=-1.5), dict(road=1, s=19, lane=-1), -1)
        self.assertNotIn('sidewalk_intrusion_m', result)
        self.assertEqual(result['sidewalk_measurement_status'], 'UNAVAILABLE')

    def test_gap_between_linked_surfaces_remains_unknown(self):
        m = self.linked_map(x=22)
        result = m.measure(ego(), dict(road=1, s=19, lane=-1), -1)
        self.assertNotIn('sidewalk_intrusion_m', result)

    def test_thin_gap_between_edge_samples_also_remains_unknown(self):
        m = self.linked_map(x=20.1)
        result = m.measure(ego(), dict(road=1, s=19, lane=-1), -1)
        self.assertNotIn('sidewalk_intrusion_m', result)
        self.assertEqual(result['sidewalk_measurement_reason'], 'gap_in_connected_finite_road_surfaces')

    def test_link_to_vertically_separated_road_does_not_cover_footprint(self):
        m = self.linked_map(z=8)
        result = m.measure(ego(), dict(road=1, s=19, lane=-1), -1)
        self.assertNotIn('sidewalk_intrusion_m', result)

    def test_unlinked_same_height_road_does_not_cover_footprint(self):
        m = self.map_from(road(1, 0) + road(2, 20))
        result = m.measure(ego(), dict(road=1, s=19, lane=-1), -1)
        self.assertNotIn('sidewalk_intrusion_m', result)

    def connector_map(self, second_y=0):
        junction = ('<junction id="7"><connection id="1" incomingRoad="1" connectingRoad="2" contactPoint="start">'
                    '<laneLink from="-1" to="-1"/></connection><connection id="2" incomingRoad="1" '
                    'connectingRoad="3" contactPoint="start"><laneLink from="-1" to="-1"/></connection></junction>')
        return self.map_from(road(1, -20) + road(2, 0, junction=7) + road(3, 0, y=second_y, junction=7) + junction)

    def test_shared_origin_overlapping_identity_and_no_boundary_crossing(self):
        m = self.connector_map()
        result = m.classify_lane_transition(dict(road=2, s=5, lane=-1, t=-1.5),
                                            dict(road=3, s=5.05, lane=-1, t=-1.5))
        self.assertEqual(result['classification'], 'continuity', result)
        self.assertEqual(result['reason'], 'overlapping_connector_gt_match_replacement')

    def test_shared_origin_disjoint_branch_is_not_automatic_continuity(self):
        m = self.connector_map(second_y=4)
        result = m.classify_lane_transition(dict(road=2, s=5, lane=-1, t=-1.5),
                                            dict(road=3, s=5.05, lane=-1, t=-1.5))
        self.assertEqual(result['classification'], 'unknown', result)

    def test_shared_origin_boundary_crossing_is_not_automatic_continuity(self):
        m = self.connector_map()
        result = m.classify_lane_transition(dict(road=2, s=5, lane=-1, t=.2),
                                            dict(road=3, s=5.05, lane=-1, t=-.2))
        self.assertEqual(result['classification'], 'unknown', result)

    def test_actual_m16_match_replacement_has_positive_interior_margin(self):
        m = OpenDriveMap(ROOT / 'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        result = m.classify_lane_transition(
            dict(road=2612, lane=-1, s=10.21677017211914, t=-0.0030674911104142666),
            dict(road=2609, lane=-3, s=10.079960823059082, t=-7.461355686187744))
        self.assertEqual(result['classification'], 'continuity', result)
        self.assertGreater(result['lane_link_chain'][0]['minimum_new_lane_interior_margin_m'], 1.4)


if __name__ == '__main__':
    unittest.main()
