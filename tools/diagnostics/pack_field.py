#!/usr/bin/env python3
"""Re-create the brief/ZIP of an ended attempt, including incomplete logs."""

import argparse
from pathlib import Path

from contracts import PackRequest
from pack import pack_run


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    try:
        pack_run(PackRequest(args.directory.resolve()))
    except (OSError, ValueError) as error:
        print(f"[field] pack failed: {error}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
