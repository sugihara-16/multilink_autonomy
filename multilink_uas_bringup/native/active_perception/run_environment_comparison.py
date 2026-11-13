#!/usr/bin/env python3
"""Concurrent isolated condition campaigns; retain every scheduled outcome."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--conditions', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--jobs', type=int, default=3)
    a = p.parse_args()
    if not 1 <= a.jobs <= 5:
        p.error('jobs must be 1 through 5')
    root = a.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    design = json.loads(a.conditions.read_text())
    frozen = {str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in HERE.glob('*.py')}
    protocol = dict(design=design, jobs=a.jobs, source_sha256=frozen,
                    task_count=len(design['conditions'])*9, run_seeds=[20262801,20262802,20262803],
                    execution='Independent ROS domains, ROS1 ports, Gazebo partitions, output directories. Physics timestep and sensor rates unchanged. Per-condition seeds/arms scheduled before outcomes.',
                    failure_policy='Retain every attempted trial, including failed acquisition/motion audits. No accuracy-based retry/exclusion. Do not report a complete average without all 3 complete intervals.')
    (root/'protocol.json').write_text(json.dumps(protocol, indent=2, ensure_ascii=False))
    active, completed = {}, {}
    todo = list(enumerate(design['conditions']))
    started = time.monotonic()
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while todo or active:
            while todo and len(active) < a.jobs:
                changed = [f for f,h in frozen.items() if hashlib.sha256(Path(f).read_bytes()).hexdigest() != h]
                if changed:
                    raise RuntimeError('Campaign source changed: '+str(changed))
                i, condition = todo.pop(0)
                name = condition['name']
                cmd = [sys.executable, str(HERE/'run_practical_comparison.py'), '--output', str(root/name),
                       '--world', condition['world'], '--domain-base', str(20+10*i), '--port-base', str(11700+10*i), '--continue-failures', '--distance', str(design.get('distance_m',90)),
                       '--origin', *map(str,design.get('origin',[5.,0.,2.]))]
                log = (root/(name+'.log')).open('w')
                child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                active[name] = (child, log)
                (root/(name+'.command.json')).write_text(json.dumps(cmd, indent=2))
                print('START', name, flush=True)
            for name,(child,log) in list(active.items()):
                if child.poll() is not None:
                    log.close()
                    completed[name] = child.returncode
                    del active[name]
                    print('FINISH', name, child.returncode, flush=True)
            (root/'status.json').write_text(json.dumps(dict(active=list(active), completed=completed,
                pending=[c['name'] for _,c in todo], wall_s=time.monotonic()-started), indent=2))
            if active:
                time.sleep(3)
        (root/'status.json').write_text(json.dumps(dict(status='complete', completed=completed, wall_s=time.monotonic()-started), indent=2))
    finally:
        for child,log in active.values():
            # Signal owner only: it drains/stops its children itself.
            child.send_signal(signal.SIGINT)
        for child,log in active.values():
            child.wait(timeout=90)
            log.close()


if __name__ == '__main__':
    main()
