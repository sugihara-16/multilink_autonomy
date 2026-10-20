#!/usr/bin/env python3
"""Follow the serial validation without publishing estimator/control messages."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    p.add_argument('--candidate', required=True)
    p.add_argument('--phase', choices=['validation','development'], default='validation')
    a = p.parse_args()
    declaration = 'frozen_candidate_' if a.phase == 'validation' else 'development_'
    frozen = json.loads((a.root/f'{declaration}{a.candidate}.json').read_text())
    seeds = frozen['validation_seeds'] if a.phase == 'validation' else frozen['seeds']
    def folder(seed, prefix=False):
        if a.phase == 'validation':
            return a.root/f'validation-{a.candidate}-seed{seed-20261000}{"-prefix" if prefix else ""}'
        return a.root/f'{a.candidate}-{"prefix" if prefix else "short"}-seed{seed-20261000}'
    flights = [folder(s)/(label+'_active') for s in seeds
               for label in ['run1_tail_a1', 'run1_tail_b1']]
    flights += [folder(s,True)/'run1_prefix_active' for s in seeds]
    proc = None
    try:
        for flight in flights:
            runtime_path = flight/'runtime.json'
            if runtime_path.exists():
                runtime = json.loads(runtime_path.read_text())
                if runtime.get('stopped') and runtime.get('status') == 'route_complete':
                    continue
                raise RuntimeError('Existing incomplete flight must be archived before retry: '+str(flight))
            deadline = time.monotonic()+1200
            while not (flight/'protocol.json').exists():
                if a.phase == 'development' and (a.root/f'{a.candidate}_early_rejection.json').exists():
                    return
                if time.monotonic() > deadline:
                    raise RuntimeError('Validation flight did not start: '+str(flight))
                time.sleep(2)
            output = a.root/(flight.parent.name+'_'+flight.name+'_host_resources.jsonl')
            proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name('observe_host_resources.py')),
                '--flight', str(flight), '--output', str(output)])
            if proc.wait() != 0:
                raise RuntimeError('Resource observer failed: '+str(flight))
            runtime = json.loads((flight/'runtime.json').read_text())
            if not runtime['stopped'] or runtime['status'] != 'route_complete':
                raise RuntimeError('Flight ended incomplete; observer stops with preserved records')
    finally:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=20)


if __name__ == '__main__':
    main()
