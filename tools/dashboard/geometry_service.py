"""Automatic, read-only binding of geometry reports to a scenario/run.

No simulator commands or controller/generator imports. Background work writes
only versioned dashboard reports, never into the original verification run.
"""
from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import json
import os
import io
from pathlib import Path
import threading
import time
import uuid
import zlib

from run_repository import REPO_ROOT, VERIFICATION_DIR, catalog_by_id, safe_path_component
from assessment_geometry_report import generate_report

ASSETS = {'index.html': 'text/html; charset=utf-8',
          'geometry.js': 'application/javascript; charset=utf-8',
          'geometry.json': 'application/json; charset=utf-8',
          'report.json': 'application/json; charset=utf-8',
          'overview.svg': 'image/svg+xml', 'mission_corridor.svg': 'image/svg+xml'}


def committed_frames(path):
    """Decode a fixed byte prefix of a growing gzip; never accept a torn JSON row.

Missing gzip footer is expected while a recorder is open. Invalid complete
JSON or a bad gzip CRC still fails. The resulting list is reused by both report
passes so appended frames cannot change the index while a snapshot is built.
"""
    if not path.is_file():
        return []
    decoder = zlib.decompressobj(31)
    pending = b''
    result = []
    remaining = path.stat().st_size
    with path.open('rb') as stream:
        while remaining:
            block = stream.read(min(65536, remaining))
            if not block:
                break
            remaining -= len(block)
            while block:
                if decoder.eof:
                    decoder = zlib.decompressobj(31)
                pending += decoder.decompress(block)
                block = decoder.unused_data
                lines = pending.split(b'\n')
                pending = lines.pop()
                result.extend(json.loads(line) for line in lines if line.strip())
    return result


def csv_trajectory(path, scenario_id, run_id):
    """Use committed run.csv positions when full RViz history was not enabled.

Never invent old planned paths, OBBs, lane sections or receipt timestamps.
"""
    if not path.is_file():
        return []
    raw = path.read_bytes()
    raw = raw[:raw.rfind(b'\n')+1].decode('utf-8-sig')
    result = []
    for row in csv.DictReader(io.StringIO(raw)):
        if row.get('scenario_id') != scenario_id or row.get('run_id') != run_id:
            raise ValueError('run.csv identity does not match requested geometry')
        ego = {key: float(row[field]) for key, field in
               (('x', 'ego_x'), ('y', 'ego_y'), ('heading', 'ego_heading'), ('speed', 'ego_speed'))}
        for key, field in (('road_id', 'current_road_id'), ('lane_id', 'current_lane_id')):
            if row.get(field):
                ego[key] = int(row[field])
        result.append(dict(frame=int(row['frame']), time_s=float(row['elapsed_wall_time_s']),
                           ego=ego, objects=[], decision={}, tracked_path=[], maneuver_path=[]))
    return result


class GeometryReports:
    def __init__(self, *, root=REPO_ROOT, verification_dir=VERIFICATION_DIR,
                 output_dir=None, profile_path=Path('/tmp/hl_vtd_active_assessment.json'),
                 refresh_seconds=10, builder=generate_report, catalog=catalog_by_id):
        self.root = Path(root).resolve()
        self.verification = Path(verification_dir).resolve()
        self.output = Path(output_dir or self.root/'output/dashboard_geometry').resolve()
        if self.output.is_relative_to(self.verification) or not self.output.is_relative_to(self.root/'output'):
            raise ValueError('geometry output must be separate from verification inputs')
        self.profile_path = Path(profile_path)
        self.refresh_seconds = refresh_seconds
        self.builder, self.catalog = builder, catalog
        self.lock = threading.RLock()
        self.entries = {}
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='geometry-review')
        self.task = None

    def close(self):
        self.worker.shutdown(wait=True, cancel_futures=True)

    def _source(self, path):
        path = Path(path)
        path = (self.root/path).resolve() if not path.is_absolute() else path.resolve()
        if not path.is_relative_to(self.root):
            raise ValueError('geometry input outside repository')
        return path

    @staticmethod
    def _json(path):
        return json.loads(path.read_text(encoding='utf-8'))

    def _context(self, query, assessment):
        mode = query.get('source', ['live'])[0]
        if mode not in ('live', 'history'):
            raise ValueError('invalid geometry source')
        bound = None
        if assessment is not None:
            with assessment.lock:
                bound = dict(assessment.active_profile) if assessment.active_profile else None
        profile = None
        if mode == 'live':
            try:
                profile = self._json(self.profile_path)
                if profile.get('schema_version') != 1 or profile.get('status') != 'RUNNING':
                    raise ValueError('current execution profile is invalid')
                pid = profile.get('publisher_pid')
                if not isinstance(pid, int) or pid <= 0:
                    raise ValueError('current execution publisher is invalid')
                if not isinstance(profile.get('created_wall_time_ns'), int) or profile['created_wall_time_ns'] <= 0:
                    raise ValueError('current execution start timestamp is unavailable')
                try:
                    os.kill(pid, 0)
                except OSError:
                    raise ValueError('current execution publisher is no longer running')
            except FileNotFoundError:
                profile = bound
            if not profile:
                return None
            scenario_id, run_id = profile.get('scenario_id', ''), profile.get('run_id', '')
        else:
            scenario_id = query.get('scenario_id', [''])[0]
            run_id = query.get('run_id', [''])[0]
        scenario_id = safe_path_component(scenario_id, 'scenario_id')
        run_id = safe_path_component(run_id, 'run_id')
        run = (self.verification/scenario_id/run_id).resolve()
        if not run.is_relative_to(self.verification):
            raise ValueError('geometry run escapes verification directory')
        manifest_path = run/'evidence_manifest.json'
        manifest = self._json(manifest_path) if manifest_path.is_file() else {}
        if manifest and (manifest.get('scenario_id'), manifest.get('run_id')) != (scenario_id, run_id):
            raise ValueError('geometry manifest belongs to a different run')
        warnings = []
        inputs = manifest.get('inputs', {})
        def input_file(name, profile_key, default_name):
            original = inputs.get(name, {})
            value = original.get('path') or (profile or {}).get(profile_key)
            if not value:
                entry = self.catalog().get(scenario_id)
                if not entry:
                    return None
                value = self.root/'scenario'/entry['package_path']/default_name
                warnings.append(f'{name}: 실행 당시 원본 확인 불가 · 현재 시나리오 파일 참고 표시')
            path = self._source(value)
            if not path.is_file():
                warnings.append(f'{name}: 파일 없음')
                return None
            if original.get('sha256') and hashlib.sha256(path.read_bytes()).hexdigest() != original['sha256']:
                warnings.append(f'{name}: 실행 이후 파일 변경 · 이전 실행과 섞지 않도록 표시 보류')
                return None
            return path
        route = input_file('route', 'route_csv', 'route.csv')
        checkpoints = input_file('checkpoints', 'checkpoints_csv', 'checkpoints.csv')
        for name in ('mission_corridor.csv', 'mission_corridor.svg', 'rviz/static_map.csv',
                     'rviz/frames.jsonl.gz', 'rviz/latest.json', 'run.csv'):
            self._source(run/name)
        assessment_path = None
        if bound and (bound.get('scenario_id'), bound.get('run_id')) == (scenario_id, run_id):
            if not profile or profile.get('ownership_token') == bound.get('ownership_token'):
                with assessment.lock:
                    if assessment.session_id:
                        path = assessment.output_dir/(assessment.session_id+'.jsonl')
                        # Only a closed assessment log is an archival report input.
                        if assessment.file is None and path.is_file():
                            assessment_path = self._source(path)
        epoch = (profile or {}).get('ownership_token') or manifest.get('created_at', '')
        key = hashlib.sha256(f'{scenario_id}\0{run_id}\0{epoch}'.encode()).hexdigest()[:24]
        files = [run/n for n in ('mission_corridor.csv', 'mission_corridor.svg', 'rviz/static_map.csv',
                                'rviz/frames.jsonl.gz', 'rviz/latest.json', 'run.csv', 'evidence_manifest.json')]
        files += [p for p in (route, checkpoints, assessment_path) if p]
        fingerprint = [(str(p), p.stat().st_ino, p.stat().st_size, p.stat().st_mtime_ns)
                       for p in files if p.is_file()]
        signature = hashlib.sha256(json.dumps(fingerprint).encode()).hexdigest()
        updating = bool(mode == 'live' and profile and self.profile_path.is_file())
        if mode == 'history' and manifest.get('status') in ('PREPARING', 'RUNNING'):
            updating = True
        epoch_ns = (profile or {}).get('created_wall_time_ns') if mode == 'live' else None
        dynamic_paths = [run/name for name in ('rviz/frames.jsonl.gz', 'rviz/latest.json', 'run.csv')]
        fresh_dynamic = [path for path in dynamic_paths if path.is_file()
                         and (epoch_ns is None or path.stat().st_mtime_ns >= epoch_ns)]
        stale_dynamic = [path.relative_to(run).as_posix() for path in dynamic_paths
                         if path.is_file() and path not in fresh_dynamic]
        if stale_dynamic:
            warnings.append('이전 실행 동적자료 제외: '+', '.join(stale_dynamic))
        return dict(key=key, source=mode, scenario_id=scenario_id, run_id=run_id,
                    run=run, route=route, checkpoints=checkpoints, assessment=assessment_path,
                    signature=signature, warnings=warnings, updating=updating,
                    fresh_dynamic=fresh_dynamic, epoch_ns=epoch_ns,
                    has_data=run.is_dir() and (bool(fresh_dynamic) if epoch_ns is not None
                                              else any(p.is_file() for p in files[:5])))

    def status(self, query, assessment=None):
        context = self._context(query, assessment)
        if context is None:
            return dict(status='WAITING_SOURCE', key=None, url=None,
                        message='현재 실행 프로필 대기 · 이전 시나리오를 자동 대입하지 않습니다')
        with self.lock:
            key = context['key']
            entry = self.entries.setdefault(key, dict(status='WAITING_DATA', url=None, attempted=None, attempted_at=0))
            changed = entry.get('signature') != context['signature']
            available = self.task is None or self.task.done()
            due = time.monotonic()-entry['attempted_at'] >= self.refresh_seconds
            if context['has_data'] and changed and available and (due or not entry.get('attempted')):
                entry.update(status='BUILDING', attempted=context['signature'], attempted_at=time.monotonic(), error=None)
                self.task = self.worker.submit(self._build, context)
            payload = {k: context[k] for k in ('key', 'source', 'scenario_id', 'run_id', 'warnings', 'updating')}
            payload.update({k: entry.get(k) for k in ('status', 'url', 'revision', 'updated_at', 'frames', 'error', 'missing')})
            if not context['has_data']:
                payload.update(status='WAITING_DATA', url=None, message='현재 실행의 궤적·경계 출력 대기 · --rviz-data 기록을 확인하세요')
            return payload

    def _build(self, context):
        key = context['key']
        revision = uuid.uuid4().hex
        directory = self.output/key/revision
        try:
            history = context['run']/'rviz/frames.jsonl.gz'
            fresh = set(context['fresh_dynamic'])
            # New run IDs may reuse a directory without truncating an old
            # frames.gz (e.g. a new invocation without --evidence). A fresh
            # ownership token invalidates the cache but not those old bytes.
            # Only files written after this invocation's publication may feed
            # its live frame history; static mission files are intentionally
            # exempt because the runner generates them before publication.
            records = (committed_frames(history) if history in fresh else []) if (
                context['updating'] or context['epoch_ns'] is not None or not history.is_file()) else None
            csv_fallback = records is not None and not records
            used_csv = False
            if csv_fallback and context['run']/'run.csv' in fresh:
                records = csv_trajectory(context['run']/'run.csv', context['scenario_id'], context['run_id'])
                used_csv = bool(records)
            if records is not None:
                latest_path = context['run']/'rviz/latest.json'
                try:
                    latest = self._json(latest_path) if latest_path in fresh else None
                    if latest is None:
                        raise FileNotFoundError('latest frame belongs to an earlier invocation')
                    if records and latest.get('frame') == records[-1].get('frame'):
                        records[-1] = latest
                    elif not records or (latest.get('frame', -1) > records[-1].get('frame', -1)):
                        records.append(latest)
                except (FileNotFoundError, json.JSONDecodeError):
                    pass
            result = self.builder(run_dir=context['run'], report_dir=directory,
                route_csv=context['route'] or self.output/'missing-route.csv',
                checkpoints_csv=context['checkpoints'], assessment_jsonl=context['assessment'],
                history_records=records)
            report_path = Path(result['json'])
            report = self._json(report_path)
            report['dashboard_binding'] = {k: context[k] for k in ('scenario_id', 'run_id', 'source', 'warnings', 'updating')}
            report['dashboard_binding']['input_signature'] = context['signature']
            if csv_fallback:
                report['missing'].append('full_rviz_history: '+(
                    '과거 위치는 현재 실행 run.csv, 계획 경로·객체는 최신 RViz 프레임만 표시' if used_csv else
                    '현재 실행의 최신 RViz 프레임만 표시 · 이전 실행 history는 제외'))
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            original = context['run']/'mission_corridor.svg'
            if original.is_file():
                (directory/'mission_corridor.svg').write_bytes(original.read_bytes())
            from assessment_geometry_report import report_html
            report['title'] = f"{context['scenario_id']} / {context['run_id']}"
            report['original_mission_svg'] = 'mission_corridor.svg' if original.is_file() else None
            Path(result['html']).write_text(report_html(report), encoding='utf-8')
            with self.lock:
                self.entries[key].update(status='READY', signature=context['signature'], revision=revision,
                    url=f'/geometry/{key}/{revision}/index.html', updated_at=time.time(),
                    frames=report['sampling']['recorded_frames'], missing=report['missing'], error=None)
        except Exception as error:
            with self.lock:
                self.entries[key].update(status='ERROR', error=f'{type(error).__name__}: {error}')

    def asset(self, url):
        parts = url.strip('/').split('/')
        if len(parts) != 4 or parts[0] != 'geometry' or parts[3] not in ASSETS:
            raise FileNotFoundError('unknown geometry asset')
        key, revision, filename = parts[1:]
        if len(key) != 24 or len(revision) != 32 or any(c not in '0123456789abcdef' for c in key+revision):
            raise ValueError('invalid geometry report address')
        path = (self.output/key/revision/filename).resolve()
        if not path.is_relative_to(self.output):
            raise ValueError('geometry asset escapes report directory')
        return path, ASSETS[filename]
