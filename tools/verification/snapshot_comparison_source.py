#!/usr/bin/env python3
"""Attach reproducible source/binary evidence to a run before another build."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run-id', required=True)
args = parser.parse_args()
out = root/'output/verification'/args.run_id
identity = json.loads((out/'identity.json').read_text())
binary = root/'build/hl_vtd'
assert hashlib.sha256(binary.read_bytes()).hexdigest() == identity['binary_sha256']
patch = subprocess.check_output(['git', '-C', str(root), 'diff', '--binary', 'HEAD', '--',
                                 'src', 'config', 'CMakeLists.txt', 'tests'])
with gzip.open(out/'source.patch.gz', 'xb') as stream:
    stream.write(patch)
shutil.copy2(binary, out/'hl_vtd.frozen')
