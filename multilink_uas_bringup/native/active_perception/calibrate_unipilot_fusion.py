#!/usr/bin/env python3
"""Freeze relative-odometry noise from previous fixed native-sensor flights.

No fusion outputs, active-policy outputs, or validation-flight truth are used.
"""
import argparse,json,hashlib,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from calibrate_rovio_relative import relative_error,calibration,stats,check_log
from report_routes import load,poses,interp

def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--integration-window-s',type=float,default=.05,
        help='Approximate correlated errors by matching accumulated error over this window; factor period stays 50 ms')
    a=p.parse_args();lag=int(round(a.integration_window_s/.05))
    if lag<1 or abs(lag*.05-a.integration_window_s)>1e-9:raise ValueError('Window must be a positive multiple of 50 ms')
    check_log();errors=[];sources=[]
    for label in ['run1_tail_a1','run1_tail_b1']:
        d=a.source/(label+'_active')/'fixed';v=poses(d/'rovio.csv');g=poses(d/'truth.csv')
        r=load(d/'route_tracking.csv');m=r['route_t']>0;start=r['sim_t'][m][0]*1e9;end=r['sim_t'][m][-1]*1e9
        v=v[(v[:,0]>=start)&(v[:,0]<=end)];t=v[:,0]
        assert np.all(np.diff(t)==50000000)
        pos,q=interp(v,t);gp,gq=interp(g,t);_,e=relative_error(t,pos,q,gp,gq,lag);errors.append(e)
        sources.append(dict(path=str(d.resolve()),rovio_sha256=hashlib.sha256((d/'rovio.csv').read_bytes()).hexdigest(),truth_sha256=hashlib.sha256((d/'truth.csv').read_bytes()).hexdigest(),statistics=stats(e)))
    # Equal flight weight, independent of recording length.
    # An independent-step approximation predicts window covariance N*C.
    # Use measured window error second moment / N. This preserves the
    # accumulated uncertainty at the chosen horizon, not exact time correlation.
    second=sum(e.T@e/len(e) for e in errors)/len(errors)/lag
    scale=np.sqrt(second.diagonal());cor=second/np.outer(scale,scale);val,vec=np.linalg.eigh(cor)
    C=(vec@np.diag(np.maximum(val,1e-6))@vec.T)*np.outer(scale,scale)
    result=dict(protocol='Frozen 50 ms relative-pose error second moment, equally weighted previous fixed native UniPilot a1/b1 flights. No performance-based search. Truth used only for offline calibration and evaluation.',sources=sources,covariance=C.tolist(),sigma_rotation_deg=np.rad2deg(np.sqrt(C.diagonal()[:3])).tolist(),sigma_translation_m=np.sqrt(C.diagonal()[3:]).tolist(),limitations=['Temporal error correlation remains','Shared IMU correlation is not explicitly modelled','Constant calibration does not imply exact per-frame covariance'])
    if lag>1:
        result.update(protocol='Approximate independent 50 ms factors matched to the measured '+str(a.integration_window_s)+' second relative-pose error second moment, equally weighted previous fixed native calibration flights. Horizon selected from the existing MIMOSA smoothing window, not flight accuracy.',
            calibration_window_s=a.integration_window_s,factor_interval_s=.05,equivalent_steps=lag,
            measured_window_second_moment=(second*lag).tolist())
        result['limitations'].append('Horizon-matched diffusion approximation: does not reproduce full temporal covariance or exact SE(3) composition')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k] for k in ['sigma_rotation_deg','sigma_translation_m']}))
if __name__=='__main__':main()
