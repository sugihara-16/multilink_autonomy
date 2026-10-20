#!/usr/bin/env python3
"""Calibrate body-frame displacement without a visual rotation residual."""
import argparse,gzip,hashlib,json
from pathlib import Path
import numpy as np
from report_routes import poses,interp,load,csv_path

def main():
    p=argparse.ArgumentParser();p.add_argument('calibration',type=Path);p.add_argument('output',type=Path);p.add_argument('--integration-window-s',type=float,default=.05);p.add_argument('--gravity-direction',action='store_true');a=p.parse_args()
    lag=int(round(a.integration_window_s/.05))
    if lag<1 or abs(lag*.05-a.integration_window_s)>1e-9:raise ValueError('Window must be a positive multiple of 50 ms')
    if a.output.exists():raise ValueError('Refusing to overwrite calibration')
    original=json.loads(a.calibration.read_text());moments=[];summaries=[];gravity_moments=[];gravity_summaries=[]
    for source in original['sources']:
        path=Path(source['path'])
        for name in ['rovio','truth']:
            file=csv_path(path/(name+'.csv'))
            with (gzip.open(file,'rb') if file.suffix=='.gz' else file.open('rb')) as stream:digest=hashlib.sha256(stream.read()).hexdigest()
            if digest!=source[name+'_sha256']:raise ValueError('Original calibration input changed')
        v=poses(path/'rovio.csv');r=load(path/'route_tracking.csv');t=r['sim_t'][r['route_t']>0]
        v=v[(v[:,0]>=t[0]*1e9)&(v[:,0]<=t[-1]*1e9)];t=v[:,0]
        if not np.all(abs(np.diff(t)-50000000)<=2):raise ValueError('Calibration timestamp gaps')
        vp,vq=interp(v,t);gp,gq=interp(poses(path/'truth.csv'),t)
        if a.gravity_direction:
            u=vq.inv().apply([0.,0.,1.]);ug=gq.inv().apply([0.,0.,1.])
            angle=np.arctan2(np.linalg.norm(np.cross(u,ug),axis=1),np.sum(u*ug,axis=1))
            gravity_moments.append(float(np.mean(angle*angle)/2))
            gravity_summaries.append(dict(source=str(path),samples=len(angle),rmse_deg=float(np.rad2deg(np.sqrt(np.mean(angle*angle))))))
        e=gq[:-lag].inv().apply(gp[lag:]-gp[:-lag])-vq[:-lag].inv().apply(vp[lag:]-vp[:-lag])
        moments.append(e.T@e/len(e)/lag);summaries.append(dict(source=str(path),samples=len(e),bias=e.mean(axis=0).tolist(),rmse=np.sqrt(np.mean(e*e,axis=0)).tolist()))
    C=np.mean(moments,axis=0)
    if np.linalg.eigvalsh(C).min()<=0:raise ValueError('Nonpositive covariance')
    result=dict(original)
    result.update(protocol='50ms body-frame displacement error second moment; equal weighting of original independent fixed-camera a1/b1 calibration. No visual relative-rotation residual. No fusion-outcome fitting.',
        translation_covariance=C.tolist(),translation_calibration=summaries,
        limitations=['Visual orientation still defines the coordinates of the measured displacement',
                    'Temporal and shared-IMU correlations are not explicitly modelled',
                    'Fixed-view calibration is an approximate nominal model for moving views'])
    if lag>1:
        result.update(protocol='Body-frame displacement error over the existing 1 s smoother horizon, divided by the number of 50 ms factors. Equal weight of original fixed calibration flights. No fusion-outcome fitting.',translation_window_s=a.integration_window_s,equivalent_steps=lag)
        result['limitations'].append('Horizon-matched diffusion approximation, not exact temporal covariance')
    if a.gravity_direction:
        result.update(gravity_sigma_rad=float(np.sqrt(np.mean(gravity_moments))),gravity_calibration=gravity_summaries,
            gravity_protocol='Isotropic tangent variance is half the absolute gravity-direction squared error, equally weighted original calibration flights; no pose alignment, no bias removal; applied sparsely at 1 Hz.')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print('translation sigma m',np.sqrt(C.diagonal()))
if __name__=='__main__':main()
