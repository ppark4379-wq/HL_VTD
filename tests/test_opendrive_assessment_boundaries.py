"""Map-only regression tests; these do not run or certify live VTD driving."""
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/dashboard'))
from opendrive_assessment import OpenDriveMap


def position(road, s, lane):
    return dict(road=road, s=s, lane=lane)


def ego(x=20, y=-2.5, heading=0, length=4, width=2):
    return dict(x=x, y=y, h=heading, length=length, width=width, off_x=0, off_y=0)


def lane(lane_id, kind='driving', links='', width=3):
    return f'<lane id="{lane_id}" type="{kind}"><link>{links}</link><width sOffset="0" a="{width}"/></lane>'


def road(road_id, sections, links='', junction=-1):
    return (f'<road id="{road_id}" length="100" junction="{junction}"><link>{links}</link>'
            '<planView><geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry></planView>'
            f'<lanes>{sections}</lanes></road>')


def section(s=0, left='', right='', mark=''):
    return (f'<laneSection s="{s}"><center><lane id="0">{mark}</lane></center>'
            f'<left>{left}</left><right>{right}</right></laneSection>')


class OpenDriveBoundaryAuditTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def map_from(self, text):
        path = Path(self.directory.name) / 'map.xodr'
        path.write_text('<OpenDRIVE>' + text + '</OpenDRIVE>')
        return OpenDriveMap(path)

    def test_m16_predecessor_renumber_is_continuity_in_both_directions(self):
        m = OpenDriveMap(ROOT / 'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        for rid, boundary, before_lane, after_lane in ((2469, 41.201, 1, 2),
                                                     (2626, 63.901, 1, 2),
                                                     (2626, 63.901, 2, 3)):
            before, after = position(rid, boundary + .01, before_lane), position(rid, boundary - .01, after_lane)
            with self.subTest(road=rid, before=before_lane, after=after_lane):
                forward = m.classify_lane_transition(before, after)
                self.assertEqual(forward['classification'], 'continuity')
                self.assertEqual(forward['mapped_old_lane'], after_lane)
                self.assertEqual(m.classify_lane_transition(after, before)['classification'], 'continuity')

    def test_actual_change_uses_old_lane_mapped_into_new_section(self):
        m = OpenDriveMap(ROOT / 'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        result = m.classify_lane_transition(position(2469, 41.211, 1), position(2469, 41.191, 1))
        self.assertEqual(result['classification'], 'actual')
        self.assertEqual(result['mapped_old_lane'], 2)

    def test_missing_section_link_is_unknown_even_if_ids_match(self):
        m = self.map_from(road(1, section(right=lane(-1)) + section(50, right=lane(-1))))
        result = m.classify_lane_transition(position(1, 49, -1), position(1, 51, -1))
        self.assertEqual(result['classification'], 'unknown')
        self.assertIn('lane_section_link', result['reason'])

    def test_same_section_actual_change_and_missing_road(self):
        m = self.map_from(road(1, section(right=lane(-1) + lane(-2))))
        result = m.classify_lane_transition(position(1, 20, -1), position(1, 21, -2))
        self.assertEqual(result['classification'], 'actual')
        self.assertEqual(result['mapped_old_lane'], -1)
        self.assertEqual(m.classify_lane_transition(position(1, 20, -1), position(9, 0, -1))['classification'], 'unknown')

    def test_direct_road_end_connection_with_reversed_reference(self):
        first = road(1, section(right=lane(-1, links='<successor id="2"/>')),
                     '<successor elementType="road" elementId="2" contactPoint="end"/>')
        second = road(2, section(left=lane(1) + lane(2, links='<successor id="-1"/>')))
        m = self.map_from(first + second)
        result = m.classify_lane_transition(position(1, 99, -1), position(2, 99, 2))
        self.assertEqual(result['classification'], 'continuity')
        self.assertEqual(result['mapped_old_lane'], 2)
        mismatch = m.classify_lane_transition(position(1, 99, -1), position(2, 99, 1))
        self.assertEqual(mismatch['classification'], 'unknown')

    def test_junction_unique_link_and_ambiguous_link(self):
        first = road(1, section(right=lane(-1)), '<successor elementType="junction" elementId="7"/>')
        second = road(2, section(right=lane(-1) + lane(-2)), junction=7)
        link = '<connection id="1" incomingRoad="1" connectingRoad="2" contactPoint="start"><laneLink from="-1" to="-2"/></connection>'
        m = self.map_from(first + second + f'<junction id="7">{link}</junction>')
        self.assertEqual(m.classify_lane_transition(position(1, 99, -1), position(2, 1, -2))['classification'], 'continuity')
        duplicate = '<connection id="2" incomingRoad="1" connectingRoad="2" contactPoint="start"><laneLink from="-1" to="-1"/></connection>'
        m = self.map_from(first + second + f'<junction id="7">{link}{duplicate}</junction>')
        self.assertEqual(m.classify_lane_transition(position(1, 99, -1), position(2, 1, -2))['classification'], 'unknown')

    def test_junction_explicit_yellow_divider_with_only_negative_lanes(self):
        mark = '<roadMark sOffset="0" type="solid" color="yellow"/>'
        m = self.map_from(road(1, section(right=lane(-1), mark=mark), junction=7))
        result = m.measure(ego(y=-.4), position(1, 20, -1), -1)
        self.assertAlmostEqual(result['centerline_intrusion_m'], .6)
        self.assertEqual(result['centerline_measurement_status'], 'AVAILABLE')
        self.assertNotIn('sidewalk_intrusion_m', result)
        self.assertEqual(result['sidewalk_measurement_reason'], 'sidewalk_not_defined')

    def test_connector_guidance_line_is_not_invented_as_central_divider(self):
        mark = '<roadMark sOffset="0" type="broken" color="standard"/>'
        m = self.map_from(road(1, section(right=lane(-1), mark=mark), junction=7))
        result = m.measure(ego(y=-.4), position(1, 20, -1), -1)
        self.assertNotIn('centerline_intrusion_m', result)
        self.assertEqual(result['centerline_measurement_reason'], 'center_divider_not_defined')

    def test_sidewalk_is_finite_width_polygon_not_infinite_half_plane(self):
        m = self.map_from(road(1, section(right=lane(-1) + lane(-2, 'sidewalk', width=4)), junction=7))
        result = m.measure(ego(y=-2.5), position(1, 20, -1), -1)
        self.assertAlmostEqual(result['sidewalk_intrusion_m'], .5)
        result = m.measure(ego(y=-9), position(1, 20, -1), -1)
        self.assertEqual(result['sidewalk_intrusion_m'], 0)
        spanning = m.measure(ego(y=-5, width=6), position(1, 20, -1), -1)
        self.assertAlmostEqual(spanning['sidewalk_intrusion_m'], 4)

    def test_road_end_is_not_extrapolated_or_snapped_to_interior(self):
        m = self.map_from(road(1, section(right=lane(-1) + lane(-2, 'sidewalk', width=4))))
        self.assertIsNone(m.project(1, (-1, -3), 1))
        self.assertIsNone(m.project(1, (101, -3), 99))
        result = m.measure(ego(x=1), position(1, 1, -1), -1)
        self.assertNotIn('sidewalk_intrusion_m', result)
        self.assertEqual(result['sidewalk_measurement_reason'], 'footprint_outside_defined_road_geometry')

    def test_curved_sidewalk_depth_includes_footprint_edge_between_corners(self):
        lanes = section(left=lane(1) + lane(2, 'sidewalk', width=4))
        curved = ('<road id="1" length="60" junction="7"><planView>'
                  '<geometry s="0" x="0" y="0" hdg="0" length="60">'
                  '<arc curvature="0.02"/></geometry></planView>'
                  f'<lanes>{lanes}</lanes></road>')
        m = self.map_from(curved)
        s = 20
        heading = s * .02
        # Radius 50 m, sidewalk inner radius 47 m. The footprint's middle
        # left edge reaches radius 45.5 m; its corners are less intrusive.
        x = (50 - 3.5) * math.sin(heading)
        y = 50 - (50 - 3.5) * math.cos(heading)
        result = m.measure(ego(x=x, y=y, heading=heading), position(1, s, 1), 1)
        self.assertEqual(result['sidewalk_measurement_status'], 'AVAILABLE')
        self.assertAlmostEqual(result['sidewalk_intrusion_m'], 1.5, delta=.003)

    def test_mark_record_end_leaves_partial_boundary_unknown(self):
        mark = '<roadMark sOffset="0" type="solid" color="yellow"/><roadMark sOffset="21" type="none"/>'
        m = self.map_from(road(1, section(right=lane(-1), mark=mark), junction=7))
        result = m.measure(ego(x=20, y=-.4), position(1, 20, -1), -1)
        self.assertNotIn('centerline_intrusion_m', result)
        self.assertEqual(result['centerline_measurement_reason'], 'center_divider_definition_partial')

    def test_m16_explicit_junction_sidewalks_and_yellow_divider_are_measurable(self):
        m = OpenDriveMap(ROOT / 'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        for rid in (2482, 2612, 2641, 2793):
            with self.subTest(road=rid):
                s = float(m.roads[rid].get('length')) / 2
                x, y = m.lane_center_world(rid, s, -1)
                heading = m.reference_pose(rid, s)[2]
                result = m.measure(ego(x=x, y=y, heading=heading), position(rid, s, -1), -1)
                self.assertEqual(result['sidewalk_measurement_status'], 'AVAILABLE', result)
                self.assertEqual(result['sidewalk_intrusion_m'], 0)
        x, y = m.lane_center_world(2609, 18, -1)
        result = m.measure(ego(x=x, y=y, heading=m.reference_pose(2609, 18)[2]), position(2609, 18, -1), -1)
        self.assertEqual(result['centerline_measurement_status'], 'AVAILABLE', result)
        self.assertEqual(result['centerline_intrusion_m'], 0)


if __name__ == '__main__':
    unittest.main()
