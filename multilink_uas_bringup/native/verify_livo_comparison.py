#!/usr/bin/env python3
"""Validate complete outputs, shared inputs, source stamps and tracking coverage."""
import csv,json,hashlib
from pathlib import Path
import numpy as np
from plot_livo_comparison import BASE,LABELS,METHODS,read

def write_csv(path,a,window):
    t,p,q=a;keep=(t>=window['start_ns'])&(t<=window['end_ns'])
    with path.open('w') as f:
        w=csv.writer(f);w.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw'])
        w.writerows([[int(stamp),*pos,*quat] for stamp,pos,quat in zip(t[keep],p[keep],q.as_quat()[keep])])

def main():
    result={}
    for label in LABELS:
        inp=BASE/'inputs'/label;manifest=json.loads((inp/'manifest.json').read_text());window=manifest['window']
        stamps=[];cloud_stamps=[]
        for chunk in manifest['chunks']:
            with np.load(inp/chunk) as a:stamps.extend(a['cam1_t'].tolist());cloud_stamps.extend(a['cloud_t'].tolist())
        stamps=np.array(stamps,np.int64);assert np.all(np.diff(stamps)>0)
        expected=stamps[(stamps>=window['start_ns'])&(stamps<=window['end_ns'])]
        dest=BASE/'tracking'/label;dest.mkdir(parents=True,exist_ok=True)
        truth=read(inp/'truth.csv');write_csv(dest/'truth.csv',truth,window)
        vio=read(BASE/'rovio'/label/'result_vio.csv');write_csv(dest/'rovio.csv',vio,window)
        assert set(vio[0]).issubset(set(stamps))
        cases={}
        for name in METHODS:
            path=BASE/name/label
            summary=json.loads((path/'summary.json').read_text());assert summary['returncode']==0,(label,name,summary)
            a=read(path/'odometry.csv');t=a[0];tracking=t[(t>=window['start_ns'])&(t<=window['end_ns'])]
            assert np.array_equal(tracking,expected),(name,label,'missing',len(np.setdiff1d(expected,tracking)),'extra',len(np.setdiff1d(tracking,expected)))
            assert np.allclose(np.linalg.norm(a[2].as_quat(),axis=1),1.0)
            assert set(t).issubset(set(stamps)|set(cloud_stamps))
            log=(path/'run.log').read_text()
            assert 'Skipping LiDAR scan' not in log and 'higher than threshold' not in log
            write_csv(dest/(name+'.csv'),a,window)
            cases[name]=dict(tracking_count=len(tracking),missing_timestamps=0,extra_timestamps=0,max_gap_s=float(np.diff(tracking).max()/1e9),wall_s=summary['wall_s'],on_activations=log.count('DEGEN Handling'),drpm_logged_updates=log.count('DRPM probabilities'))
        assert np.array_equal(read(BASE/'mimosa_off'/label/'odometry.csv')[1],read(BASE/'mimosa_on'/label/'odometry.csv')[1])
        result[label]=dict(window=window,conditions=cases,rovio_input_sha256=hashlib.sha256((BASE/'rovio'/label/'result_vio.csv').read_bytes()).hexdigest())
        (dest/'window.json').write_text(json.dumps(window,indent=2))
    (BASE/'verification.json').write_text(json.dumps(result,indent=2))
    p=BASE/'experiment.json';m=json.loads(p.read_text());m['status']='complete';m['verification']='verification.json';p.write_text(json.dumps(m,indent=2))
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
