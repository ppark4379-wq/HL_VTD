"""Assessment-only duplicate normalization, including real M17 raw GT evidence."""
import copy
import math
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/dashboard'))
sys.path.insert(0, str(ROOT/'tools/verification'))
from assessment_signals import normalize_stop_line_candidates, SignalResolver, decode_traffic_light
from map_features import extract_features
from opendrive_assessment import OpenDriveMap
from evaluate import evaluate_signals


class StopLineNormalizationTests(unittest.TestCase):
    def setUp(self):
        self.line = dict(id='1:a', road_id=1, lane_ids=[-1], s=10., t=-1.5,
                         begin=[10., -3.], end=[10., 0.], direction=1)

    def normalize(self, *lines):
        return normalize_stop_line_candidates([(8., 8., line) for line in lines])

    def test_micro_duplicates_merge_without_mutating_originals(self):
        second = copy.deepcopy(self.line)
        second.update(id='1:b', t=-1.5+2e-6)
        second['begin'][1] += 2e-6
        second['end'][1] += 2e-6
        original = copy.deepcopy([self.line, second])
        normalized, merges = self.normalize(self.line, second)
        self.assertEqual(len(normalized), 1)
        self.assertEqual(normalized[0][2]['alias_ids'], ['1:a', '1:b'])
        self.assertEqual(merges[0]['tolerance_m'], .001)
        self.assertEqual(normalized[0][2]['begin'], original[0]['begin'])
        self.assertEqual([self.line, second], original)

    def test_reverse_endpoint_order_requires_same_physical_approach(self):
        reverse = dict(self.line, id='1:b', begin=self.line['end'], end=self.line['begin'], direction=-1)
        self.assertEqual(len(self.normalize(self.line, reverse)[0]), 1)
        reverse['direction'] = 1
        self.assertEqual(len(self.normalize(self.line, reverse)[0]), 2)

    def test_alias_canonical_is_stable_when_distance_order_reverses(self):
        alias = dict(self.line, id='1:b', begin=self.line['end'], end=self.line['begin'], direction=-1)
        candidates=[(1.,1.,self.line),(.999999,.999999,alias)]
        original=copy.deepcopy(candidates)
        normal,merges=normalize_stop_line_candidates(candidates)
        reverse,reversed_merges=normalize_stop_line_candidates(list(reversed(candidates)))
        self.assertEqual(normal,reverse)
        self.assertEqual(merges,reversed_merges)
        self.assertEqual(normal[0][:2],(1.,1.))
        self.assertEqual(normal[0][2]['id'],'1:a')
        self.assertEqual(normal[0][2]['direction'],self.line['direction'])
        self.assertEqual(normal[0][2]['begin'],self.line['begin'])
        self.assertEqual(normal[0][2]['end'],self.line['end'])
        self.assertEqual(candidates,original)

    def test_road_lane_coordinate_direction_and_binding_mismatch_do_not_merge(self):
        variants = [dict(road_id=2), dict(lane_ids=[-2]), dict(lane_ids=[-1,-2]),
                    dict(s=10.02), dict(t=-1.48), dict(direction=-1),
                    dict(signal_id=99), dict(begin=[10.02,-3.],end=[10.02,0.])]
        for change in variants:
            with self.subTest(change=change):
                self.assertEqual(len(self.normalize(self.line, dict(self.line,id='1:b',**change))[0]), 2)

    def test_unknown_metadata_and_nonfinite_geometry_do_not_merge(self):
        for field in ('road_id','lane_ids','s','t','direction','begin','id'):
            other = dict(self.line)
            del other[field]
            self.assertEqual(len(self.normalize(self.line, other)[0]), 2)
        self.assertEqual(len(self.normalize(self.line, dict(self.line, t=math.nan))[0]), 2)

    def test_close_but_adjacent_collinear_segments_are_not_extended(self):
        next_piece = dict(self.line,id='1:b',t=1.5,begin=[10.,0.],end=[10.,3.])
        self.assertEqual(len(self.normalize(self.line,next_piece)[0]),2)

    def test_transitive_drift_chain_does_not_merge_all(self):
        lines=[]
        for i in range(3):
            delta=i*.0007
            lines.append(dict(self.line,id=f'1:{i}',s=10.+delta,
                              begin=[10.+delta,-3.],end=[10.+delta,0.]))
        self.assertEqual(len(self.normalize(*lines)[0]),2)


class M17RecordedStopRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map_path=ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr'
        cls.map=OpenDriveMap(cls.map_path)
        cls.lines,_=extract_features(cls.map)

    def test_m17_203_56_and_203_64_red_stop_is_now_measurable(self):
        # Actual raw payload: session 20260910_202525_53adf817 frame5090.
        raw=bytes.fromhex('ac01000035ad543f00000010870000000000d041030000001800000042ecc43e03000000f94eec3d050000000000003f01000000')
        models={(1000020,-1):dict(colors=['RED'],movement='ALL'),
                (1000008,-1):dict(colors=['YELLOW'],movement='ALL'),
                (1000012,-1):dict(colors=['GREEN'],movement='ALL')}
        resolver=SignalResolver(self.map_path,self.map,self.lines,
                                signal_models=models,authority='rdb_logic')
        ego=dict(x=1014.0536606755466,y=87.99193831503086,h=2.2364656925201416,
                 length=4.8480000495910645,off_x=1.3839999437332153,off_y=0.,speed=0.)
        road=dict(road=2625,lane=2,s=5.384566307067871)
        original=copy.deepcopy(self.lines)
        scored_rows=[]
        for frame_no,stamp,state in ((5090,203.56,.8307679295539856),(5092,203.64,.8338448405265808)):
            signals={}
            for ident in (428,429,430):
                data=bytearray(raw)
                struct.pack_into('<ifI',data,0,ident,state,0x10000000 if ident==428 else 0)
                signals[ident]=decode_traffic_light(bytes(data),1)
            result=resolver.resolve(dict(frame=frame_no,time=stamp,signals=signals),ego,road)
            self.assertEqual((result['status'],result['traffic_light_state']),('READY','RED'))
            self.assertAlmostEqual(result['stop_line_distance_m'],1.4248987793681003,places=6)
            self.assertEqual(result['stop_line']['alias_ids'],['2625:4507','2625:4508'])
            self.assertEqual(result['evidence']['stop_line_normalization']['normalized_candidate_count'],1)
            scored_rows.append(dict(sim_time_s=str(stamp),ego_speed='0',evaluation_section_id='2',
                traffic_light_state=result['traffic_light_state'],stop_line_id=result['stop_line']['id'],
                stop_line_distance_m=str(result['stop_line_distance_m'])))
        self.assertEqual(self.lines,original)
        red,green,flashing=evaluate_signals(scored_rows)
        self.assertEqual((red.status,red.penalty),('PASS',0))

    def test_missing_lane_2445_minus1_is_not_synthesized(self):
        self.assertFalse(any(line['road_id']==2445 and -1 in line['lane_ids'] for line in self.lines))

    def test_all_three_reported_m17_duplicate_pairs_are_proven_aliases(self):
        for pair in [('2445:3955','2445:3956'),('2625:4507','2625:4508'),('2674:4676','2674:4677')]:
            with self.subTest(pair=pair):
                selected=[(1.,1.,line) for line in self.lines if line['id'] in pair]
                self.assertEqual(len(selected),2)
                normalized,merges=normalize_stop_line_candidates(selected)
                self.assertEqual(len(normalized),1)
                self.assertEqual(normalized[0][2]['alias_ids'],list(pair))


if __name__=='__main__':
    unittest.main()
