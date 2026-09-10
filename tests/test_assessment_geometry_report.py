import gzip
import json
from pathlib import Path
import tempfile
import unittest

from tools.verification.assessment_geometry_report import generate_report,pick_indices,PolylineIndex


class GeometryReportTests(unittest.TestCase):
    def test_events_survive_sampling(self):
        index=[dict(receive_time_s=i/10) for i in range(101)]
        selected=pick_indices(index,[dict(received_monotonic_ns=4300000000)],10)
        self.assertIn(43,selected)
        self.assertIn(38,selected)
        self.assertIn(48,selected)
        self.assertLessEqual(len(selected),10)

    def test_segment_distance_is_not_nearest_vertex(self):
        self.assertAlmostEqual(PolylineIndex([[0,0],[10,0]]).distance([5,2]),2)

    def test_report_only_reads_sources_and_preserves_missing_input(self):
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory);run=base/'run';(run/'rviz').mkdir(parents=True)
            route=base/'route.csv';route.write_text('x,y\n0,0\n10,0\n')
            cp=base/'checkpoints.csv';cp.write_text('seq,x,y\n1,10,0\n')
            mission=run/'mission_corridor.csv'
            mission.write_text('leg_index,role,road_id,section_index,lane_id,point_index,x,y\n0,gateway,1,0,1,0,0,0\n0,gateway,1,0,1,1,10,0\n')
            frames=[dict(frame=i,time_s=i,receive_time_s=100+i,ego=dict(x=i*5,y=1,road_id=1,section_index=0,lane_id=1,speed=2,obb=[]),objects=[],decision=dict(active_leg=0),tracked_path=[[i*5,1],[10,1]]) for i in range(2)]
            history=run/'rviz/frames.jsonl.gz'
            with gzip.open(history,'wt') as f:
                for frame in frames:f.write(json.dumps(frame)+'\n')
            before={path:path.read_bytes() for path in (route,cp,mission,history)}
            out=generate_report(run_dir=run,report_dir=base/'report',route_csv=route,checkpoints_csv=cp,max_frames=5)
            for path,content in before.items():self.assertEqual(path.read_bytes(),content)
            report=json.loads(Path(out['json']).read_text())
            self.assertEqual(report['summary']['route_deviation_max_m'],1)
            self.assertEqual(report['summary']['outside_recorded_corridor_lane_frames'],0)
            self.assertIn('assessment_session',report['missing'])
            self.assertTrue(Path(out['html']).is_file())
            self.assertEqual(report['sampling']['recorded_frames'],2)

    def test_reject_input_root_as_report_target(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):generate_report(run_dir=directory,report_dir=directory)

    def test_report_never_overwrites_metadata_input(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'run').mkdir();(root/'report').mkdir()
            metadata=root/'report/report.json';metadata.write_text('{"original":true}')
            with self.assertRaisesRegex(ValueError,'overwrite an input'):
                generate_report(run_dir=root/'run',report_dir=root/'report',metadata_paths=[metadata])
            self.assertEqual(metadata.read_text(),'{"original":true}')


if __name__=='__main__':unittest.main()
