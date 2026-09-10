"""Reject convenient but wrong-lane endpoint pairings from VTD screenshots."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/dashboard'))
from opendrive_assessment import OpenDriveMap
from school_zones import SchoolZones, load_school_zone_data

SOURCE = ROOT / 'tools/dashboard/living_lab_school_zones.yaml'
OVERLAY = ROOT / 'tools/dashboard/living_lab_school_zone_corrections.yaml'


class ScreenshotEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map = OpenDriveMap(ROOT / 'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        cls.zones = SchoolZones(SOURCE, cls.map, OVERLAY)
        data, _ = load_school_zone_data(SOURCE, OVERLAY)
        cls.by_id = {zone['id']: zone for zone in data['school_zones']}

    def test_screenshot_points_agree_with_map_but_do_not_enable_unknown_zones(self):
        report = self.zones.report()
        self.assertEqual(len(report['accepted']), 43)
        self.assertEqual({zone['id'] for zone in report['pending']},
                         {'school_zone_15', 'school_zone_28'})
        observations = [observation for zone in report['pending']
                        for observation in zone['review_observations']]
        self.assertEqual(len(observations), 5)
        for observation in observations:
            self.zones._validate_boundary(observation['point'])
            self.assertFalse(observation['assessment_applied'])
            self.assertTrue(observation['needed'])

    def test_second_image_connects_to_adjacent_positive_lane_not_requested_lane(self):
        zone = self.by_id['school_zone_28']
        first, second = [observation['point'] for observation in zone['review_observations']]
        self.assertEqual(self.zones._paths(first, second), [])
        self.assertEqual(self.zones._paths(zone['point_1'], second), [])
        paths = self.zones._paths(second, self.by_id['school_zone_29']['point_1'])
        self.assertEqual(len(paths), 1)
        self.assertEqual([(road, lane) for road, lane, _, _ in paths[0]],
                         [(2352, -1), (2313, 1), (2392, -3), (2386, 2)])

    def test_third_image_is_opposite_direction_to_original_negative_lane_pair(self):
        zone = self.by_id['school_zone_15']
        point = zone['review_observations'][0]['point']
        self.assertIn((1769, 0, 2, 1), self.zones._next(1775, 0, -2, -1))
        for endpoint in ('point_1', 'point_2'):
            self.assertEqual(self.zones._paths(point, zone[endpoint]), [])

    def test_followup_target_lane_point_does_not_validate_opposite_direction_start(self):
        zone = self.by_id['school_zone_15']
        observations = {item['observation_id']: item
                        for item in zone['review_observations']}
        start = observations['user_20260910_image_4']['point']
        end = observations['user_20260910_image_5']['point']
        self.assertEqual((end['track_id'], end['lane_id']), (1769, -1))
        self.assertIn((1769, 0, 1, 1), self.zones._next(1775, 0, -1, -1))
        self.assertEqual(self.zones._paths(start, end), [])
        self.assertEqual(self.zones._paths(end, start), [])


if __name__ == '__main__':
    unittest.main()
