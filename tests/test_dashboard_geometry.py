"""Automatic report binding; never runs VTD or any controller/generator."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import urllib.request
import urllib.error
from http.server import ThreadingHTTPServer
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/dashboard'))
from geometry_service import GeometryReports, committed_frames


class GeometryBindingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.catalog = {}
        self.profile = self.root/'active.json'
        self.service = GeometryReports(root=self.root, verification_dir=self.root/'output/verification',
            profile_path=self.profile, refresh_seconds=0, catalog=lambda: self.catalog)
        self.addCleanup(self.service.close)

    def fixture(self, scenario='M16', run='r1', offset=0, history=True):
        created_wall_time_ns = time.time_ns()
        package = self.root/'scenario/mixed'/scenario
        package.mkdir(parents=True, exist_ok=True)
        self.catalog[scenario] = dict(package_path='mixed/'+scenario)
        route = package/'route.csv'; route.write_text(f'x,y\n{offset},0\n{offset+10},0\n')
        checkpoints = package/'checkpoints.csv'; checkpoints.write_text(f'seq,x,y\n1,{offset+10},0\n')
        directory = self.root/'output/verification'/scenario/run
        (directory/'rviz').mkdir(parents=True)
        mission = directory/'mission_corridor.csv'
        mission.write_text(f'leg_index,role,road_id,section_index,lane_id,x,y\n0,gateway,1,0,1,{offset},0\n0,gateway,1,0,1,{offset+10},0\n')
        frames = [dict(frame=i,time_s=i,receive_time_s=100+i,
            ego=dict(x=offset+i*5,y=1,road_id=1,section_index=0,lane_id=1,speed=2,obb=[]),
            objects=[],decision=dict(active_leg=0),tracked_path=[[offset+i*5,1],[offset+10,1]]) for i in range(2)]
        if history:
            with gzip.open(directory/'rviz/frames.jsonl.gz', 'wt') as stream:
                stream.write(''.join(json.dumps(row)+'\n' for row in frames))
        (directory/'rviz/latest.json').write_text(json.dumps(frames[-1]))
        (directory/'run.csv').write_text('scenario_id,run_id,frame,elapsed_wall_time_s,ego_x,ego_y,ego_heading,ego_speed\n'+
            ''.join(f'{scenario},{run},{i},{i},{offset+i*5},1,0,2\n' for i in range(2)))
        info = lambda p: dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest())
        manifest = dict(scenario_id=scenario,run_id=run,created_at='epoch1',status='COMPLETED',
                        inputs=dict(route=info(route),checkpoints=info(checkpoints)))
        (directory/'evidence_manifest.json').write_text(json.dumps(manifest))
        profile = dict(schema_version=1,status='RUNNING',publisher_pid=os.getpid(),ownership_token='epoch1',
                       created_wall_time_ns=created_wall_time_ns,
                       scenario_id=scenario,run_id=run,route_csv=str(route),checkpoints_csv=str(checkpoints))
        return directory, profile

    def finish(self, query):
        self.service.status(query)
        self.assertIsNotNone(self.service.task)
        self.service.task.result(timeout=10)
        result = self.service.status(query)
        self.assertEqual(result['status'], 'READY', result)
        path, _ = self.service.asset(result['url'].replace('index.html', 'geometry.json'))
        return result, json.loads(path.read_text())

    def test_history_switch_uses_exact_run_and_preserves_originals(self):
        first, _ = self.fixture()
        second, _ = self.fixture('M17', 'r2', 100)
        before = {p:p.read_bytes() for directory in (first,second) for p in directory.rglob('*') if p.is_file()}
        a, data_a = self.finish(dict(source=['history'],scenario_id=['M16'],run_id=['r1']))
        b, data_b = self.finish(dict(source=['history'],scenario_id=['M17'],run_id=['r2']))
        self.assertNotEqual(a['key'], b['key'])
        self.assertEqual(data_a['route'][0], [0,0])
        self.assertEqual(data_b['route'][0], [100,0])
        for path, value in before.items(): self.assertEqual(path.read_bytes(), value)

    def test_live_follows_profile_not_history_query(self):
        _, p = self.fixture()
        _, p2 = self.fixture('M17', 'r2', 100)
        self.profile.write_text(json.dumps(p))
        a, _ = self.finish(dict(source=['live'],scenario_id=['wrong'],run_id=['wrong']))
        self.profile.write_text(json.dumps(p2))
        pending = self.service.status(dict(source=['live']))
        self.assertEqual(pending['scenario_id'], 'M17')
        self.assertIsNone(pending['url'])
        self.service.task.result(timeout=10)
        b = self.service.status(dict(source=['live']))
        self.assertNotEqual(a['key'], b['key'])

    def test_no_profile_never_uses_latest_folder(self):
        self.fixture()
        result = self.service.status({})
        self.assertEqual(result['status'], 'WAITING_SOURCE')
        self.assertIsNone(result['url'])

    def test_missing_new_run_does_not_reuse_old_report(self):
        _, p = self.fixture()
        self.profile.write_text(json.dumps(p))
        first, _ = self.finish({})
        p.update(run_id='new', ownership_token='new')
        self.profile.write_text(json.dumps(p))
        next_run = self.service.status({})
        self.assertEqual(next_run['status'], 'WAITING_DATA')
        self.assertIsNone(next_run['url'])
        self.assertNotEqual(first['key'], next_run['key'])

    def test_reused_run_id_new_owner_invalidates_cached_report(self):
        _, p = self.fixture()
        self.profile.write_text(json.dumps(p))
        first, _ = self.finish({})
        p['ownership_token'] = 'another-execution'
        p['created_wall_time_ns'] = time.time_ns()
        self.profile.write_text(json.dumps(p))
        pending = self.service.status({})
        self.assertNotEqual(first['key'], pending['key'])
        self.assertIsNone(pending['url'])
        self.assertEqual(pending['status'], 'WAITING_DATA')
        self.assertTrue(any('이전 실행 동적자료 제외' in warning for warning in pending['warnings']))

    def test_reused_run_id_fresh_latest_never_imports_old_history(self):
        directory, p = self.fixture()
        self.profile.write_text(json.dumps(p))
        first, _ = self.finish({})
        p.update(ownership_token='new-epoch', created_wall_time_ns=time.time_ns())
        self.profile.write_text(json.dumps(p))
        latest = json.loads((directory/'rviz/latest.json').read_text())
        latest['frame'] = 100
        latest['ego']['x'] = 900
        (directory/'rviz/latest.json').write_text(json.dumps(latest))
        second, data = self.finish({})
        self.assertNotEqual(first['key'], second['key'])
        self.assertEqual([frame['frame'] for frame in data['frames']], [100])
        self.assertEqual(data['frames'][0]['ego']['x'], 900)
        self.assertTrue(any('frames.jsonl.gz' in warning for warning in second['warnings']))

    def test_reused_run_id_fresh_csv_never_imports_old_latest_or_history(self):
        directory, p = self.fixture()
        p.update(ownership_token='new-epoch', created_wall_time_ns=time.time_ns())
        self.profile.write_text(json.dumps(p))
        (directory/'run.csv').write_text('scenario_id,run_id,frame,elapsed_wall_time_s,ego_x,ego_y,ego_heading,ego_speed\n'
                                         'M16,r1,9,0,500,1,0,0\n')
        _, data = self.finish({})
        self.assertEqual([frame['frame'] for frame in data['frames']], [9])
        self.assertEqual(data['frames'][0]['ego']['x'], 500)

    def test_changed_archival_route_is_withheld_not_substituted(self):
        _, p = self.fixture()
        Path(p['route_csv']).write_text('x,y\n999,999\n1000,999\n')
        status, data = self.finish(dict(source=['history'],scenario_id=['M16'],run_id=['r1']))
        self.assertEqual(data['route'], [])
        self.assertTrue(any('실행 이후 파일 변경' in text for text in status['warnings']))

    def test_csv_fallback_preserves_track_without_inventing_old_plan(self):
        _, p = self.fixture(history=False)
        self.profile.write_text(json.dumps(p))
        status, data = self.finish({})
        self.assertEqual(len(data['frames']),2)
        self.assertEqual(data['frames'][0]['tracked_path'], [])
        self.assertTrue(data['frames'][1]['tracked_path'])
        self.assertTrue(any('full_rviz_history' in text for text in status['missing']))

    def test_new_frame_triggers_new_revision(self):
        directory, p = self.fixture(history=False)
        self.profile.write_text(json.dumps(p))
        first, _ = self.finish({})
        latest = json.loads((directory/'rviz/latest.json').read_text())
        latest['frame'] = 2
        (directory/'rviz/latest.json').write_text(json.dumps(latest))
        second, data = self.finish({})
        self.assertNotEqual(first['revision'], second['revision'])
        self.assertEqual(data['frames'][-1]['frame'], 2)

    def test_manifest_identity_mismatch_fails(self):
        directory, _ = self.fixture()
        (directory/'evidence_manifest.json').write_text(json.dumps(dict(scenario_id='M17',run_id='r1')))
        with self.assertRaisesRegex(ValueError, 'different run'):
            self.service.status(dict(source=['history'],scenario_id=['M16'],run_id=['r1']))

    def test_reject_traversal_and_escaped_input(self):
        with self.assertRaises(ValueError):
            self.service.status(dict(source=['history'],scenario_id=['../M16'],run_id=['r1']))
        with self.assertRaises((ValueError,FileNotFoundError)):
            self.service.asset('/geometry/../../etc/passwd')
        with self.assertRaises(ValueError): self.service._source('/etc/passwd')

    def test_gzip_prefix_and_closed_stream_same_committed_frames(self):
        path = self.root/'frames.jsonl.gz'
        with gzip.open(path,'wt') as stream:
            stream.write('{"frame":1}\n'); stream.flush()
            self.assertEqual(committed_frames(path), [dict(frame=1)])
            stream.write('{"frame":2'); stream.flush()
            self.assertEqual(committed_frames(path), [dict(frame=1)])
            stream.write('}\n')
        self.assertEqual(committed_frames(path), [dict(frame=1),dict(frame=2)])

    def test_gzip_complete_corrupt_row_not_silently_skipped(self):
        path = self.root/'frames.jsonl.gz'
        with gzip.open(path,'wt') as stream: stream.write('{bad}\n')
        with self.assertRaises(ValueError): committed_frames(path)

    def test_http_report_assets_and_binding_endpoint(self):
        import server
        directory, _ = self.fixture()
        status, _ = self.finish(dict(source=['history'],scenario_id=['M16'],run_id=['r1']))
        old_reports, old_assessment = server.GEOMETRY_REPORTS, server.LIVE_ASSESSMENT
        server.GEOMETRY_REPORTS, server.LIVE_ASSESSMENT = self.service, None
        http = ThreadingHTTPServer(('127.0.0.1',0),server.Handler)
        worker = threading.Thread(target=http.serve_forever,daemon=True);worker.start()
        try:
            base = f'http://127.0.0.1:{http.server_port}'
            with urllib.request.urlopen(base+'/api/geometry?source=history&scenario_id=M16&run_id=r1') as response:
                self.assertEqual(json.load(response)['key'],status['key'])
            with urllib.request.urlopen(base+status['url']) as response:
                self.assertIn('M16 / r1',response.read().decode())
                self.assertEqual(response.headers['Cache-Control'],'no-store')
            with urllib.request.urlopen(base+status['url'].replace('index.html','geometry.js')) as response:
                self.assertIn('window.GEOMETRY_DATA=',response.read().decode())
            with self.assertRaises(urllib.error.HTTPError):
                urllib.request.urlopen(base+'/api/geometry?source=history&scenario_id=..&run_id=r1')
        finally:
            http.shutdown();http.server_close();worker.join(timeout=2)
            server.GEOMETRY_REPORTS,server.LIVE_ASSESSMENT=old_reports,old_assessment


if __name__ == '__main__': unittest.main()
