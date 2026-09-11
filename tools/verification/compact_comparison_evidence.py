#!/usr/bin/env python3
"""Losslessly compact this batch's closed evidence; no old/user logs touched."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def compact(batch):
    entries = json.loads((batch / 'batch.json').read_text())
    for entry in entries:
        if entry['status'] != 'FINISHED':
            continue
        folder = batch / entry['scenario'] / 'rdb'
        record = folder / 'lossless_compaction.json'
        if record.exists():
            continue
        manifest = json.loads((folder / 'manifest.json').read_text())
        if manifest['active']:
            continue
        report = []
        spool = folder / 'transport/receive.spool'
        if spool.exists():
            subprocess.run(['gzip', '-9', '--', str(spool)], check=True)
            report.append(dict(original=str(spool), compressed=str(spool)+'.gz'))
        source = folder / 'frames.jsonl.gz'
        target = folder / 'frames.compacting.gz'
        digest = hashlib.sha256()
        old_size = source.stat().st_size
        with gzip.open(source, 'rb') as reader, gzip.open(target, 'xb', compresslevel=9) as writer:
            for block in iter(lambda: reader.read(1024*1024), b''):
                digest.update(block)
                writer.write(block)
        verified = hashlib.sha256()
        with gzip.open(target, 'rb') as reader:
            for block in iter(lambda: reader.read(1024*1024), b''):
                verified.update(block)
        if digest.digest() != verified.digest():
            raise RuntimeError('compression verification failed: original is preserved')
        target.replace(source)
        report.append(dict(file=str(source), original_compressed_bytes=old_size,
                           compressed_bytes=source.stat().st_size, uncompressed_sha256=digest.hexdigest()))
        record.write_text(json.dumps(report, indent=2))
        print('COMPACTED', entry['scenario'], old_size, source.stat().st_size, flush=True)
    return entries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--watch', action='store_true')
    args = parser.parse_args()
    batch = ROOT / 'output/verification' / args.run_id
    while True:
        entries = compact(batch)
        identity = json.loads((batch/'identity.json').read_text())
        if not args.watch or (len(entries) == len(identity['scenarios']) and
                              all(e['status'] == 'FINISHED' for e in entries)):
            return
        time.sleep(5)


if __name__ == '__main__':
    main()
