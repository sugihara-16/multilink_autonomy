#!/usr/bin/env python3
"""Run the selected paired experiment after checking its frozen implementation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seed', type=int, default=20261026)
    parser.add_argument('--labels', nargs='+', default=['run1_tail_a1', 'run1_tail_b1', 'run1_prefix'],
                        choices=['run1_tail_a1', 'run1_tail_b1', 'run1_prefix'])
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output directory; existing evidence is preserved.')
    here = Path(__file__).resolve().parent
    selected = json.loads((here/'selected_gimbal.json').read_text())
    for filename, expected in selected['files'].items():
        if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Selected implementation changed: '+filename)
    command = selected['command'].copy()
    command[command.index('--output')+1] = str(args.output.resolve())
    command[command.index('--seed')+1] = str(args.seed)
    begin = command.index('--labels')+1
    end = next(i for i in range(begin, len(command)) if command[i].startswith('--'))
    command[begin:end] = args.labels
    if args.dry_run:
        print(shlex.join(command))
        return
    os.chdir(here.parent)
    os.execv(command[0], command)


if __name__ == '__main__':
    main()
