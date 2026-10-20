#!/usr/bin/env python3
"""Read-only subsecond disk samples; save changes and clock discontinuities."""
import argparse
import json
import shutil
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=7200.)
    args = parser.parse_args()
    start = time.monotonic()
    previous = None
    with args.output.open('x', buffering=1) as out:
        while time.monotonic()-start < args.duration:
            row = dict(wall=time.time(), monotonic=time.monotonic(),
                       free_bytes=shutil.disk_usage('/mnt/c').free)
            changed = previous is None or abs(row['free_bytes']-previous['free_bytes']) >= 100_000_000
            elapsed = 0. if previous is None else row['monotonic']-previous['monotonic']
            clock_gap = 0. if previous is None else row['wall']-previous['wall']-elapsed
            if changed or elapsed >= 15 or abs(clock_gap) >= 1 or row['free_bytes'] < 8_500_000_000:
                row['clock_discontinuity_s'] = clock_gap
                out.write(json.dumps(row)+'\n')
                previous = row
            time.sleep(.2)


if __name__ == '__main__':
    main()
