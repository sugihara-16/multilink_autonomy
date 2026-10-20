#!/usr/bin/env python3
"""Short paired experiment; all estimator and controller source hashes frozen."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml

HERE = Path(__file__).resolve().parent
NATIVE = HERE.parent


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--end', type=float, default=120.)
    p.add_argument('--seed', type=int, default=20261026)
    p.add_argument('--mode', choices=['geometry', 'covariance'], default='geometry')
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out/'campaign').mkdir()
    (out/'diagnostic_implementation').mkdir()
    for filename in ['run_directional_diagnosis.py','observe_directional_complementarity.py']:
        (out/'diagnostic_implementation'/filename).write_bytes((HERE/filename).read_bytes())
    flight = out/'campaign/run2a_prefix_active'
    selected = json.loads((HERE/'selected_gimbal.json').read_text())
    checks = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() == sha
              for p, sha in selected['files'].items()}
    assert all(checks.values())
    (out/'selected_source_checks.json').write_text(json.dumps(checks, indent=2)+'\n')
    reference = NATIVE/'logs/lio-failure-comparison-20261019/run2a_prefix/commands.json'
    command = json.loads(reference.read_text())['flight']
    def replace(flag, value):
        command[command.index(flag)+1] = str(value)
    replace('--route-end', a.end)
    replace('--output', flight)
    replace('--seed', a.seed)
    config = yaml.safe_load((HERE/'tracking_aware_v57.yaml').read_text())
    config['log_candidates'] = True
    planner = out/'planner_diagnostic.yaml'
    planner.write_text(yaml.safe_dump(config))
    replace('--planner-config', planner)
    while '--pacing-extra-lidar' in command:
        i = command.index('--pacing-extra-lidar')
        del command[i:i+2]
    names = ['duplicate_active', 'pure_lio', 'fixed_velocity_active_tilt',
             'active_velocity_fixed_tilt']
    if a.mode == 'covariance':
        names = ['duplicate_active', 'active_velocity_fixed_cov',
                 'fixed_velocity_active_cov', 'fixed_velocity_active_tilt']
    for name in names:
        command += ['--pacing-extra-lidar', '/direction_'+name+'/lidar/manager/debug']
    commands = dict(observer=[sys.executable, str(HERE/'observe_directional_complementarity.py'),
        '--flight', str(flight), '--output', str(out/'observer'), '--port', '11485',
        '--mode', a.mode],
        gpu=[sys.executable, str(HERE/'gpu_keepalive.py'), '--duration', '1300',
             '--blocks', '32', '--interval', '.001', '--cycles', '1000000'],
        flight=command)
    (out/'commands.json').write_text(json.dumps(commands, indent=2)+'\n')
    processes = {}
    started = time.monotonic()
    def interrupt(*unused):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        for name, cmd in commands.items():
            with (out/(name+'.log')).open('w') as log:
                processes[name] = subprocess.Popen(cmd, cwd=NATIVE, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
        (out/'owners.json').write_text(json.dumps({k: v.pid for k, v in processes.items()},
                                                  indent=2)+'\n')
        while processes['flight'].poll() is None:
            if processes['observer'].poll() is not None and not (flight/'runtime.json').exists():
                raise RuntimeError('Observer exited before reference')
            time.sleep(.25)
        if processes['flight'].returncode:
            raise RuntimeError('Reference failed')
        processes['observer'].wait(timeout=40)
        if processes['observer'].returncode:
            raise RuntimeError('Observer failed')
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for process in processes.values():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes.values():
            try:
                process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        (out/'completion.json').write_text(json.dumps(dict(stopped=True,
            wall_s=time.monotonic()-started,
            processes={k: p.returncode for k, p in processes.items()}), indent=2)+'\n')


if __name__ == '__main__':
    main()
