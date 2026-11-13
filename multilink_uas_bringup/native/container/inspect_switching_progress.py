#!/usr/bin/env python3
"""Read-only progress and full-interval scores; do not select or retry runs."""
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'active_perception'))
from practical_comparison_metrics import evaluate
from compact_record_io import record_path


def load(path):
    return json.loads(path.read_text())


def main():
    root=Path(sys.argv[1]);protocol=load(root/'protocol.json')
    result={}
    for c in protocol['design']['conditions']:
        directory=root/c['name'];state=load(directory/'status.json')
        info=result[c['name']]=dict(completed=len(state['completed']),trials=[])
        for record in state['completed']:
            trial=directory/record['name'];row=dict(record)
            try:
                if record['name'].startswith('unipilot'):
                    route=np.genfromtxt(record_path(trial/'route_tracking.csv'),delimiter=',',names=True)
                    moving=route[route['route_t']>0];start=float(moving['sim_t'][0]-moving['route_t'][0])
                else:
                    start=next(e['time'] for e in load(trial/'flight_events.json') if e['stage']=='active')
                truth=np.loadtxt(record_path(trial/'truth.csv'),delimiter=',',skiprows=1,ndmin=2)[:,:8];truth[:,0]*=1e-9
                for estimator in ['mimosa','rovio']:
                    estimate=np.loadtxt(record_path(trial/f'{estimator}.csv'),delimiter=',',skiprows=1,ndmin=2)[:,:8];estimate[:,0]*=1e-9
                    row[estimator]=evaluate(truth,estimate,start,262.)[-1]['position_rmse_m']
                validation=trial/'validation.json'
                if validation.exists():row['failed_checks']=[k for k,v in load(validation)['checks'].items() if not v]
            except Exception as exc:row['evaluation_error']=str(exc)
            info['trials'].append(row)
        if 'active' in state:
            info['active']=state['active'];trial=directory/state['active']
            for name in ['route_status.json','flight_status.json']:
                p=trial/name
                if p.exists():
                    data=load(p);info[name]={k:data[k] for k in ('stage','sim_time','route_time_s','failure','failed') if k in data}
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
