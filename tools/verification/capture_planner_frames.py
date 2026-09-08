#!/usr/bin/env python3
"""Capture deduplicated atomic RViz snapshots outside the control process."""
import argparse
import json
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("latest", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float, default=240.)
    args = parser.parse_args()
    deadline = time.monotonic() + args.seconds
    previous = None
    count = 0
    # Exclusive output prevents accidentally appending runs or overwriting evidence.
    with args.output.open("x", encoding="utf-8", buffering=65536) as stream:
        while time.monotonic() < deadline:
            try:
                raw = args.latest.read_text(encoding="utf-8")
                frame = json.loads(raw)
                identity = (frame.get("receive_time_s"), frame.get("frame"))
                if identity != previous:
                    stream.write(raw.strip() + "\n")
                    previous = identity
                    count += 1
                    if count % 20 == 0:
                        stream.flush()
            except (OSError, ValueError):
                pass  # Startup/atomic replacement can precede the first frame.
            time.sleep(.04)
    print(f"Captured {count} frames in {args.output}")


if __name__ == "__main__":
    main()
