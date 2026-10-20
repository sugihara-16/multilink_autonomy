#!/usr/bin/env python3
"""Offline check only: does rotating nominal VIO uncertainty with gaze fit observed errors?"""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from report_routes import load,poses,interp
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from calibrate_rovio_relative import relative_error


def skew(v):
    x,y,z=v;return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--candidate',required=True);a=p.parse_args()
    calibration=json.loads((a.root/'native_fusion_window1s_calibration.json').read_text())
    C=np.array(calibration['measured_window_second_moment']);result=[]
    for campaign in sorted(a.root.glob(a.candidate+'-*')):
        for b in sorted(campaign.glob('*_active')):
            f=b/'runtime.json'
            if not f.exists() or json.loads(f.read_text()).get('status')!='route_complete':continue
            route=load(b/'route_tracking.csv');rt=route['sim_t'][route['route_t']>0];enc=load(b/'encoders.csv')
            for side,d in [('fixed',b/'fixed'),('active',b)]:
                v=poses(d/'rovio.csv');gt=poses(d/'truth.csv');t=v[(v[:,0]>=rt[0]*1e9)&(v[:,0]<=rt[-1]*1e9),0]
                assert np.all(np.abs(np.diff(t)-50000000)<2)
                pos,q=interp(v,t);gp,gq=interp(gt,t);te,e=relative_error(t,pos,q,gp,gq,20)
                mid=(t[20:]+t[:-20])*.5e-9
                angles=np.c_[np.interp(mid,enc['t'],enc['yaw']),np.interp(mid,enc['t'],enc['pitch'])]
                if side=='fixed':angles*=0.
                R=Rotation.from_euler('ZY',angles).as_matrix();lever=np.array([.18,0.,-.025])
                delta=lever-np.einsum('nij,j->ni',R,lever);A=np.zeros((len(e),6,6));A[:,:3,:3]=R;A[:,3:,3:]=R
                A[:,3:,:3]=np.array([skew(x)@rr for x,rr in zip(delta,R)])
                rotated=A@C@A.transpose(0,2,1)
                assert np.linalg.eigvalsh(rotated).min()>0
                nominal=np.einsum('ni,ij,nj->n',e,np.linalg.inv(C),e)
                predicted=np.einsum('ni,nij,nj->n',e,np.linalg.inv(rotated),e)
                # A Gaussian model scores both whitened residual and volume.
                score0=nominal+np.linalg.slogdet(C)[1];score1=predicted+np.linalg.slogdet(rotated)[1]
                result.append(dict(campaign=campaign.name,route=b.name,side=side,samples=len(e),
                    mean_normalized_squared_error=dict(body_fixed=float(nominal.mean()),gaze_rotated=float(predicted.mean())),
                    mean_gaussian_score_change=float(np.mean(score1-score0)),
                    fraction_angle_above_10deg=float(np.mean(np.linalg.norm(angles,axis=1)>np.deg2rad(10))),
                    relative_error_rmse=np.sqrt(np.mean(e*e,axis=0)).tolist()))
    output=dict(scope='Posthoc diagnosis using truth only for evaluation. Rotated covariance is a hypothesis, not a correction applied to any estimator. Midpoint gaze and fixed camera pivot; nominal covariance from previous fixed calibration flights. Shared-IMU and full temporal dependence remain unmodelled.',candidate=a.candidate,results=result)
    (a.root/f'view_covariance_diagnosis_{a.candidate}.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
