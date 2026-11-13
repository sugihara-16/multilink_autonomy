#!/usr/bin/env python3
"""Three isolated seed pairs, using the completed UniPilot environment unchanged."""
import argparse
import hashlib
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parents[1] / 'active_perception'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    args = parser.parse_args()
    reference = args.reference.resolve()
    original = json.loads((reference/'comparison_protocol.json').read_text())
    assert original['seeds'] == [20262601, 20262602, 20262603]
    assert original['distance_m'] == 90 and original['origin'] == [5, 0, 2]
    changed = [p for p,h in original['sha256'].items()
               if not Path(p).exists() or hashlib.sha256(Path(p).read_bytes()).hexdigest() != h]
    if changed:
        raise RuntimeError('Inputs changed since the UniPilot reference: '+str(changed))
    assert json.loads((reference/'final_audit.json').read_text())['passed']
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    design = dict(reference=str(reference), reference_summary_sha256=hashlib.sha256((reference/'report/summary.json').read_bytes()).hexdigest(),
                  world=original['world'], seeds=original['seeds'], arms=['gimbal','joint'], distance_m=90, speed_mps=.5, origin=[5,0,2],
                  original_frozen_inputs_match=True, original_frozen_count=len(original['sha256']),
                  jobs=3, task_count=6, retention='All attempts, no accuracy-based retries or exclusions; no 3-run mean unless every interval is complete',
                  interpretation='Same environment and nominal route; independent flights and sensor realizations. Adopted configurations, not mechanism-only isolation.',
                  configurations=original['configurations'])
    (root/'protocol.json').write_text(json.dumps(design,indent=2)+'\n')
    active, completed = {}, {}
    started = time.monotonic()
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        for i, seed in enumerate(design['seeds']):
            name = f'seed{seed}'
            arms = ['gimbal','joint'] if i % 2 == 0 else ['joint','gimbal']
            cmd = [sys.executable,str(HERE/'run_practical_comparison.py'), '--output',str(root/name),
                   '--world',design['world'],'--seeds',str(seed),'--arms',*arms,
                   '--distance','90','--origin','5','0','2','--domain-base',str(181+10*i),
                   '--port-base',str(11811+10*i),'--continue-failures']
            (root/(name+'.command.json')).write_text(json.dumps(cmd,indent=2)+'\n')
            log = (root/(name+'.log')).open('w')
            child = subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            active[name] = (child,log)
            print('START',name,flush=True)
        while active:
            for name,(child,log) in list(active.items()):
                if child.poll() is not None:
                    log.close()
                    completed[name] = child.returncode
                    del active[name]
                    print('FINISH',name,child.returncode,flush=True)
            (root/'status.json').write_text(json.dumps(dict(active=list(active),completed=completed,wall_s=time.monotonic()-started),indent=2)+'\n')
            if active:
                time.sleep(3)
        (root/'status.json').write_text(json.dumps(dict(status='complete',completed=completed,wall_s=time.monotonic()-started),indent=2)+'\n')
    finally:
        for child,_ in active.values():
            child.send_signal(signal.SIGINT)
        for child,log in active.values():
            child.wait(timeout=90)
            log.close()


if __name__ == '__main__':
    main()
