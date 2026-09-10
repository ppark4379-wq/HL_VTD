"""School-zone connection/repair evidence must never become guessed coverage."""
import copy
import csv
import math
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/dashboard'))
from opendrive_assessment import OpenDriveMap
from school_zones import SchoolZones

SOURCE=ROOT/'tools/dashboard/living_lab_school_zones.yaml'
FIXES=ROOT/'tools/dashboard/living_lab_school_zone_corrections.yaml'


class SchoolZoneReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map=OpenDriveMap(ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        cls.raw=yaml.safe_load(SOURCE.read_text())
        cls.zones=SchoolZones(SOURCE,cls.map,FIXES)

    def rear_at(self,rid,lid,s):
        samples=self.map.reference(rid)
        for a,b in zip(samples,samples[1:]):
            if a[0]<=s<=b[0] and b[0]>a[0]:
                alpha=(s-a[0])/(b[0]-a[0]); h=math.atan2(b[2]-a[2],b[1]-a[1])
                lane=self.map.lanes_at(rid,s)['lanes'][lid]
                t=(lane['inner']+lane['outer'])/2
                return a[1]+alpha*(b[1]-a[1])-math.sin(h)*t, a[2]+alpha*(b[2]-a[2])+math.cos(h)*t
        self.fail('test sample outside road')

    def test_only_eight_previously_pending_zones_are_enabled(self):
        report=self.zones.report()
        self.assertEqual(len(report['accepted']),43)
        self.assertEqual(len(report['resolved_ranges']),77)
        self.assertEqual({p['id'] for p in report['pending']},{'school_zone_15','school_zone_28'})
        self.assertFalse(report['complete'])
        self.assertLess(report['max_boundary_s_error_m'],.01)
        for pending in report['pending']:
            self.assertEqual(pending['uncertainty_scope'],'bounded_path_budget')
            self.assertEqual(len(pending['boundary_points']),2)
            self.assertGreater(pending['path_budget_m'],20)

    def test_reverse_junction_link_is_real_and_capture_order_independent(self):
        z=self.zones
        self.assertEqual(z._direct_next(1603,0,-1,-1),[])
        self.assertIn((1612,0,-1,-1),z._next(1603,0,-1,-1))
        for num in (12,27,29):
            raw=self.raw['school_zones'][num-1]
            paths=z._paths(raw['point_1'],raw['point_2'])
            reverse=z._paths(raw['point_2'],raw['point_1'])
            self.assertEqual(len(paths),1)
            self.assertEqual(len(reverse),1)
            self.assertEqual(paths[0],list(reversed(reverse[0])))

    def test_all_recovered_strips_detect_their_midpoint(self):
        restored={f'school_zone_{n:02}' for n in (12,27,29,39,40,42,44,45)}
        for r in self.zones.ranges:
            if r['id'] not in restored: continue
            with self.subTest(zone=r['id'],road=r['road'],lane=r['lane']):
                s=(r['s_min']+r['s_max'])/2
                rear=self.rear_at(r['road'],r['lane'],s)
                active,ids=self.zones.measure(dict(road=r['road'],s=s),rear)
                self.assertTrue(active)
                self.assertIn(r['id'],ids)

    def test_corrected_endpoints_stay_in_their_own_lanes(self):
        for num,lane,start,end in ((39,-1,136.130,170.710),(40,-2,136.090,170.444),
                                   (42,-3,657.966,738.979),(44,-1,655.863,739.451),
                                   (45,-4,696.957,738.520)):
            strips=[r for r in self.zones.ranges if r['id']==f'school_zone_{num:02}']
            self.assertEqual({r['lane'] for r in strips},{lane})
            self.assertAlmostEqual(min(r['s_min'] for r in strips),start)
            self.assertAlmostEqual(max(r['s_max'] for r in strips),end)

    def test_unresolved_pairs_are_not_filled_by_lane_change_or_uturn(self):
        for num in (15,28):
            raw=self.raw['school_zones'][num-1]
            self.assertEqual(self.zones._paths(raw['point_1'],raw['point_2']),[])
            self.assertFalse(any(r['id']==raw['id'] for r in self.zones.ranges))

    def test_pending_zone_is_unknown_only_inside_its_conservative_envelope(self):
        raw=self.raw['school_zones'][14]
        rear=self.rear_at(
            raw['point_1']['track_id'],raw['point_1']['lane_id'],
            raw['point_1']['s'],
        )
        self.assertEqual(
            self.zones.measure(
                dict(road=raw['point_1']['track_id'],s=raw['point_1']['s']),
                rear,
            ),
            (None,[]),
        )

    def test_m15_route_is_provably_outside_all_pending_envelopes(self):
        route=ROOT/'scenario/mixed/M15/route.csv'
        with route.open(encoding='utf-8-sig',newline='') as stream:
            rows=list(csv.DictReader(stream))
        self.assertGreater(len(rows),100)
        for row in rows:
            active,ids=self.zones.measure(
                {'road':int(row['road_id']),'s':float(row['road_s'])},
                (float(row['x']),float(row['y'])),
            )
            self.assertFalse(active)
            self.assertEqual(ids,[])

    def test_unverified_pending_boundary_remains_global_unknown(self):
        changed=copy.deepcopy(self.raw)
        changed['school_zones'][14]['point_1']['x']+=1000
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'invalid_boundary.yaml'
            source.write_text(yaml.safe_dump(changed),encoding='utf-8')
            zones=SchoolZones(source,self.map,FIXES)
            pending=next(p for p in zones.pending if p['id']=='school_zone_15')
            self.assertEqual(pending['uncertainty_scope'],'global')
            with (ROOT/'scenario/mixed/M15/route.csv').open(
                encoding='utf-8-sig',newline=''
            ) as stream:
                route_row=next(csv.DictReader(stream))
            self.assertEqual(
                zones.measure(
                    {'road':int(route_row['road_id']),'s':float(route_row['road_s'])},
                    (float(route_row['x']),float(route_row['y'])),
                ),
                (None,[]),
            )

    def test_source_unchanged_and_wrong_source_rejected(self):
        before=SOURCE.read_bytes()
        SchoolZones(SOURCE,self.map,FIXES)
        self.assertEqual(SOURCE.read_bytes(),before)
        changed=copy.deepcopy(self.raw)
        changed['school_zones'][38]['point_1']['s']+=1
        with tempfile.TemporaryDirectory() as tmp:
            file=Path(tmp)/'changed.yaml'; file.write_text(yaml.safe_dump(changed))
            with self.assertRaisesRegex(ValueError,'source has changed'):
                SchoolZones(file,self.map,FIXES)

    def test_recovered_boundary_does_not_cover_outside_or_opposite_lane(self):
        for s,lane in ((650,-3),(739.8,-3),(700,2)):
            active,ids=self.zones.measure(dict(road=2076,s=s),self.rear_at(2076,lane,s))
            self.assertNotIn('school_zone_42',ids)
            if not ids: self.assertFalse(active)  # Bounded pending zones are far away.


if __name__=='__main__': unittest.main()
