#!/usr/bin/env python3
"""Run selected prefix with same-input pure LIO and fused duplicate observers."""
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
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed',type=int,default=20261026)
    p.add_argument('--end',type=float,default=1070.)
    args=p.parse_args()
    selected=json.loads((HERE/'selected_gimbal.json').read_text())
    checked={name:hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest
             for name,digest in selected['files'].items()}
    if not all(checked.values()):raise RuntimeError('Selected implementation changed')
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'selected_source_checks.json').write_text(json.dumps(checked,indent=2)+'\n')
    command=selected['command'].copy()
    command[command.index('--output')+1]=str(args.output/'campaign')
    command[command.index('--seed')+1]=str(args.seed)
    begin=command.index('--labels')+1
    end=next(i for i in range(begin,len(command)) if command[i].startswith('--'))
    command[begin:end]=['run1_prefix']
    command.extend(['--end',str(args.end)])
    for name in ['duplicate_active','pure_lio']:
        command.extend(['--pacing-extra-lidar','/lio_ablation_'+name+'/lidar/manager/debug'])
    flight=args.output/'campaign/run1_prefix_active'
    sidecar=[sys.executable,str(HERE/'observe_pure_lio.py'),'--flight',str(flight),
             '--output',str(args.output/'observers'),'--port','11485']
    (args.output/'commands.json').write_text(json.dumps(dict(campaign=command,observers=sidecar),indent=2)+'\n')
    processes={}
    def interrupt(*unused):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:
        for name,cmd in [('observers',sidecar),('campaign',command)]:
            with (args.output/(name+'.log')).open('w') as log:
                processes[name]=subprocess.Popen(cmd,cwd=HERE.parent,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        print(json.dumps(dict(output=str(args.output),pids={k:v.pid for k,v in processes.items()})),flush=True)
        while processes['campaign'].poll() is None:
            if processes['observers'].poll() is not None and not (flight/'runtime.json').exists():
                raise RuntimeError('Observers exited before campaign completion')
            time.sleep(.5)
        if processes['campaign'].returncode:raise RuntimeError('Campaign failed')
        processes['observers'].wait(timeout=30)
        if processes['observers'].returncode:raise RuntimeError('Observers failed')
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        for process in processes.values():
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        for process in processes.values():
            try:process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
        result=dict(processes={k:v.returncode for k,v in processes.items()},stopped=True)
        (args.output/'completion.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)


if __name__=='__main__':main()
