#!/usr/bin/env python3
"""Repack only closed comparison evidence, verifying every uncompressed byte."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def repack(source):
    target = source.with_suffix('.zst') if source.suffix == '.gz' else Path(str(source)+'.zst')
    if target.exists():
        raise RuntimeError(f'refusing to overwrite {target}')
    before = hashlib.sha256()
    process = subprocess.Popen(['zstd', '-q', '-T1', '-3', '--long=23', '-o', str(target)],
                               stdin=subprocess.PIPE)
    with (gzip.open(source, 'rb') if source.suffix == '.gz' else source.open('rb')) as reader:
        for block in iter(lambda: reader.read(1024*1024), b''):
            before.update(block)
            process.stdin.write(block)
    process.stdin.close()
    if process.wait():
        raise RuntimeError('compression failed; original retained')
    after = hashlib.sha256()
    check = subprocess.Popen(['zstd', '-q', '-d', '-c', str(target)], stdout=subprocess.PIPE)
    for block in iter(lambda: check.stdout.read(1024*1024), b''):
        after.update(block)
    check.stdout.close()
    if check.wait() or before.digest() != after.digest():
        raise RuntimeError('verification failed; original retained')
    result = dict(original=str(source), compressed=str(target),
                  old_bytes=source.stat().st_size, new_bytes=target.stat().st_size,
                  uncompressed_sha256=before.hexdigest())
    # Only the verified, redundant gzip copy of this batch's closed recording.
    source.unlink()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    batch = ROOT/'output/verification'/args.run_id
    for item in json.loads((batch/'batch.json').read_text()):
        if item['status'] != 'FINISHED':
            continue
        folder = batch/item['scenario']/'rdb'
        if json.loads((folder/'manifest.json').read_text())['active']:
            continue
        manifest = folder/'zstd_archives.json'
        entries = json.loads(manifest.read_text()) if manifest.exists() else []
        for source in (folder/'frames.jsonl.gz', folder/'stream.rdb.gz',
                       folder/'transport/receive.spool.gz', folder/'transport/receive.spool'):
            if source.exists():
                entry = repack(source)
                entries.append(entry)
                manifest.write_text(json.dumps(entries, indent=2))
                print(item['scenario'], source.name, entry['old_bytes'], '->', entry['new_bytes'], flush=True)


if __name__ == '__main__':
    main()
