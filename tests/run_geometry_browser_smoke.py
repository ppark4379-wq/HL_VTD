"""Render the real dashboard against two synthetic runs; no VTD connections."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from http.server import ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/dashboard'))
sys.path.insert(0,str(ROOT/'tests'))
from test_dashboard_geometry import GeometryBindingTests
import server


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    fixture=GeometryBindingTests();fixture.setUp()
    fixture.fixture();fixture.fixture('M17','r2',100)
    for scenario,run in (('M16','r1'),('M17','r2')):
        fixture.finish(dict(source=['history'],scenario_id=[scenario],run_id=[run]))
    server.GEOMETRY_REPORTS=fixture.service;server.LIVE_ASSESSMENT=None
    class Handler(server.Handler):
        def do_GET(self):
            parsed=urlparse(self.path);query=parse_qs(parsed.query)
            if parsed.path=='/api/scenarios':
                self._send_json([dict(scenario_id=s,category='test',run_count=1) for s in ('M16','M17')])
            elif parsed.path=='/api/runs':
                self._send_json([dict(run_id='r1' if query['scenario_id'][0]=='M16' else 'r2',mtime=1,live=False)])
            elif parsed.path=='/api/map':
                self._send_json(dict(checkpoints=[],route=[],meta=None))
            elif parsed.path=='/api/run':
                self._send_json(dict(frame_count=0,track=[],sections=[],official_grid=[],summary={},latest={}))
            else:super().do_GET()
    http=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
    try:
        for scenario,run in (('M16','r1'),('M17','r2')):
            with tempfile.TemporaryDirectory(prefix='hl-geometry-chrome-') as profile:
                target=args.output/(scenario+'.png')
                url=f'http://127.0.0.1:{http.server_port}/?source=history&scenario_id={scenario}&run_id={run}&view=geometry'
                command=['google-chrome','--headless','--no-sandbox','--disable-gpu','--disable-background-networking',
                    '--disable-dev-shm-usage',f'--user-data-dir={profile}','--window-size=1600,1100',
                    '--virtual-time-budget=8000',f'--screenshot={target}','--dump-dom',url]
                result=subprocess.run(command,capture_output=True,text=True,timeout=40)
                if result.returncode:raise RuntimeError(result.stderr[-2000:])
                (args.output/(scenario+'.html')).write_text(result.stdout)
                assert f'{scenario} {run} · 기록 시각화' in result.stdout, result.stdout[-2000:]
                assert 'geometry-active' in result.stdout
                assert target.is_file()
                print(json.dumps(dict(scenario=scenario,run=run,result='PASS',screenshot=str(target)),ensure_ascii=False),flush=True)
    finally:
        http.shutdown();http.server_close();thread.join(timeout=2);fixture.doCleanups()


if __name__=='__main__':main()
