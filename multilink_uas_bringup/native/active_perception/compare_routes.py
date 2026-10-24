#!/usr/bin/env python3
"""Six sequential online flights; retained manifest and resumable per-run outputs."""
import argparse,json,subprocess,sys,time,hashlib,signal
from pathlib import Path
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent
LABELS=['run1_tail_a1','run1_tail_b1','run1_prefix']
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--labels',nargs='+',default=LABELS);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    if (a.output/'campaign.json').exists():previous=json.loads((a.output/'campaign.json').read_text())
    else:previous={}
    original=json.loads((NATIVE/'simulation_inputs/zab_catalog.json').read_text())
    manifest={'scope':'new live flights, not original bag replay','seed':20261013,'drpm_fusion':False,
        'view_only_drpm_diagnostics':True,'fixed_warmup':True,'speed_mps':.5,'runs':{},'status':'running'}
    manifest['runs'].update(previous.get('runs',{}))
    def save():
        (a.output/'campaign.json').write_text(json.dumps(manifest,indent=2)+'\n')
    save()
    for label in a.labels:
        ref=Path(original[label]['route']);world=json.loads((ref/'world.json').read_text())
        ss=json.loads((ref/'segments.json').read_text())['segments']
        end=1070. if label=='run1_prefix' else ss[-1]['t0']+ss[-1]['duration']
        for mode in ['fixed','active']:
            out=a.output/(label+'_'+mode);key=label+'_'+mode
            if (out/'failure_classification.json').exists():
                manifest['runs'][key]={'status':'estimator_failed','path':str(out),'failure':json.loads((out/'failure_classification.json').read_text())};save();continue
            if (out/'runtime.json').exists() and json.loads((out/'runtime.json').read_text())['status']=='route_complete':
                manifest['runs'][key]={'status':'complete','path':str(out)};save();continue
            if out.exists():raise RuntimeError('Incomplete previous attempt; keep for review: '+str(out))
            cmd=[sys.executable,str(HERE/'run.py'),'--mode',mode,'--world',world['world_file'],
                 '--spawn',*map(str,world['spawn_xyz']),'--route',str(ref),'--route-end',str(end),
                 '--output',str(out),'--no-drpm','--validate','--seed','20261013','--duration',str(1200+5*end)]
            manifest['runs'][key]={'status':'running','path':str(out),'route_end':end,'command':cmd};save()
            print('Starting '+key,flush=True);t=time.monotonic()
            with (a.output/(key+'.log')).open('w') as log:
                proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
                try:code=proc.wait()
                except KeyboardInterrupt:
                    proc.send_signal(signal.SIGINT);proc.wait();manifest['status']='interrupted';save();raise
            result=json.loads((out/'runtime.json').read_text())
            manifest['runs'][key].update(status=result['status'],wall_s=time.monotonic()-t,exit_code=code);save()
            if code or result['status']!='route_complete':raise RuntimeError('Flight failed: '+key)
            print('Completed '+key,flush=True)
    manifest['status']='complete';save()
if __name__=='__main__':main()
