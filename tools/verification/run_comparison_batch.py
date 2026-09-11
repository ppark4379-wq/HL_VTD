#!/usr/bin/env python3
"""Sequential, fixed-timeout comparison runner; never changes driving settings.

Runs a frozen repository root, keeps full controller evidence and an independent
RDB observation, and losslessly compresses this runner's finished static maps.
No score is inferred from a controller's own collision/stop decisions.
"""
import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

HOST_ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--scenarios', type=int, nargs='+', default=list(range(5, 19)))
    parser.add_argument('--runtime', type=int, default=900)
    parser.add_argument('--source-revision', required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    if args.runtime <= 0 or any(i not in range(5, 19) for i in args.scenarios):
        parser.error('positive runtime and scenarios M05..M18 required')
    out = HOST_ROOT / 'output/verification' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    binary_hash = digest(root / 'build/hl_vtd')
    config_hash = digest(root / 'config/control.yaml')
    if (root/'.git').exists():
        with (out/'source.patch').open('wb') as patch:
            subprocess.run(['git', '-C', str(root), 'diff', '--binary', 'HEAD', '--',
                            'src', 'config', 'CMakeLists.txt', 'tests'], stdout=patch, check=True)
    shutil.copy2(root/'build/hl_vtd', out/'hl_vtd.frozen')
    (out / 'identity.json').write_text(json.dumps(dict(
        root=str(root), source_revision=args.source_revision,
        binary_sha256=binary_hash, control_yaml_sha256=config_hash,
        runtime_seconds=args.runtime, scenarios=args.scenarios,
        policy='same timeout, no forced motion, no controller/scene parameter overrides',
        assessment='15 PDF items: observed, review, or unavailable; not official scoring',
        start_time='controller log time, not independently captured control arrival'), indent=2))
    sys.path.insert(0, str(root / 'tools/verification'))
    import run_scenario
    from vtd_scp import VtdScpClient
    configure_original = VtdScpClient.configure_scenario

    def configure(self, *positional, **kwargs):
        # Avoid VTD's unnecessary Configure teardown race. Scenario Apply,
        # Init, Play, triggers and all controller settings remain unchanged.
        kwargs['force_reconfigure'] = False
        kwargs['settle_delay'] = 5.0
        return configure_original(self, *positional, **kwargs)

    VtdScpClient.configure_scenario = configure
    sys.path.insert(0, str(HOST_ROOT / 'tools/dashboard'))
    from rdb_stream import RdbObserver
    from rdb_recording import RdbRecordingSession
    os.chdir(root)
    results = []
    for number in args.scenarios:
        scenario = f'M{number:02}'
        free = shutil.disk_usage(out).free
        if free < 2 * 1024**3:
            print(f'BLOCKED disk free {free}: preserving all existing evidence', flush=True)
            return 3
        if digest(root / 'build/hl_vtd') != binary_hash or digest(root / 'config/control.yaml') != config_hash:
            raise RuntimeError('frozen binary/config changed during batch')
        item = dict(scenario=scenario, status='RUNNING', started_wall=time.time())
        results.append(item)
        (out / 'batch.json').write_text(json.dumps(results, indent=2))
        print(f'BEGIN {scenario} {args.run_id} limit={args.runtime}s', flush=True)
        recording = RdbRecordingSession(out / scenario / 'rdb', '127.0.0.1', 48190)
        observer = RdbObserver('127.0.0.1', 48190, lambda frame: None,
                               lambda *a, **k: None, recorder=recording)
        observer.start()
        run_dir = root / 'output/verification' / scenario / args.run_id
        bridge = subprocess.Popen(['bash', '--noprofile', '--norc', '-c',
            'source /opt/ros/jazzy/setup.bash && exec python3 tools/visualization/rviz_bridge.py '
            '--data-dir "$1"', 'rviz-bridge', str(run_dir / 'rviz')],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            sys.argv = ['run_scenario.py', str(root / 'scenario/mixed' / scenario),
                        '--auto-vtd', '--skip-source-sync', '--run-id', args.run_id,
                        '--run-timeout', str(args.runtime), '--evidence']
            with (out / f'{scenario}.console.log').open('x') as log:
                with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
                    item['exit_code'] = run_scenario.main()
        finally:
            bridge.send_signal(signal.SIGINT)
            try:
                bridge.wait(timeout=3)
            except subprocess.TimeoutExpired:
                bridge.terminate()
                bridge.wait(timeout=3)
            item['rdb_closed_cleanly'] = observer.close()
            item['rdb_status'] = observer.status
        item.update(status='FINISHED', wall_seconds=round(time.time()-item['started_wall'], 3))
        item['rdb_manifest'] = str(recording.directory / 'manifest.json')
        # Only new artifacts owned by this invocation, after the bridge exits.
        # Decompression restores the original evidence-manifest hash exactly.
        archived = []
        for path in [run_dir / 'rviz/static_map.csv', run_dir / 'decision.log']:
            if path.is_file():
                original_hash = digest(path)
                subprocess.run(['gzip', '-6', '--', str(path)], check=True)
                archived.append(dict(original=str(path), compressed=str(path)+'.gz',
                                     uncompressed_sha256=original_hash))
        item['lossless_archives'] = archived
        (out / 'batch.json').write_text(json.dumps(results, indent=2))
        print(f'END {scenario} exit={item["exit_code"]} wall={item["wall_seconds"]}', flush=True)
        # Compact before starting the next scenario: bounded live disk usage,
        # no overlap with the recorder, no loss of raw bytes/timestamps.
        subprocess.run([sys.executable, str(HOST_ROOT/'tools/verification/zstd_comparison_evidence.py'),
                        '--run-id', args.run_id], check=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
