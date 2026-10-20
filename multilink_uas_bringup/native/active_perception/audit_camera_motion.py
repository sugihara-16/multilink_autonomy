#!/usr/bin/env python3
"""Post-hoc image/encoder timing check; truth never enters a running estimator."""
import argparse,json,struct
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation
from audit_raw_tracks import records,images,flow
from report_routes import poses,interp,load


def epipolar_pixels(p0,p1,R,t,K):
    """Symmetric Sampson approximation in pixels, with a known camera motion."""
    tx=np.array([[0.,-t[2],t[1]],[t[2],0.,-t[0]],[-t[1],t[0],0.]])
    inv=np.linalg.inv(K);F=inv.T@tx@R@inv
    x0=np.c_[p0,np.ones(len(p0))];x1=np.c_[p1,np.ones(len(p1))]
    a=x0@F.T;b=x1@F
    return np.abs(np.sum(x1*a,axis=1))/np.sqrt(np.maximum(np.sum(a[:,:2]**2+b[:,:2]**2,axis=1),1e-30))


def main():
    p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();cv2.setNumThreads(1)
    gt=poses(a.flight/'truth.csv');K=np.array(yaml.safe_load((a.flight/'cam0.yaml').read_text())['camera_matrix']['data']).reshape(3,3)
    ex=np.array([[ts,*struct.unpack_from('<7d',data,off)] for ts,data,off in records(a.flight/'vio_input.bin',2)])
    offsets=np.array([-.1,-.05,-.02,-.01,-.005,0.,.005,.01,.02,.05,.1]);rows=[];previous=None
    route=load(a.flight/'route_tracking.csv');mask=route['route_t']>0;lo=route['sim_t'][mask][0]*1e9;hi=route['sim_t'][mask][-1]*1e9
    for stamp,im in images(a.flight/'vio_input.bin'):
        if stamp<lo or stamp>hi:continue
        if previous is None:previous=(stamp,im);continue
        t0,im0=previous
        if stamp-t0<99999999:continue
        previous=(stamp,im)
        corners=cv2.goodFeaturesToTrack(im0,150,.01,12,blockSize=7)
        if corners is None:continue
        p0=corners.reshape(-1,2);p1,good,_=flow(im0,im,p0);p0=p0[good];p1=p1[good]
        if len(p0)<20:continue
        times=np.array([t0,stamp],dtype=float)
        if min(times)+offsets.min()*1e9<max(gt[0,0],ex[0,0]) or max(times)+offsets.max()*1e9>min(gt[-1,0],ex[-1,0]):continue
        row={'stamp_ns':stamp,'tracks':len(p0),'median_px':[],'p90_px':[]}
        for offset in offsets:
            ts=times+offset*1e9;bp,bq=interp(gt,ts);ep,eq=interp(ex,ts)
            cp=bp+bq.apply(ep);cq=bq*eq;R=(cq[1].inv()*cq[0]).as_matrix();t=cq[1].inv().apply(cp[0]-cp[1])
            error=epipolar_pixels(p0,p1,R,t,K)
            row['median_px'].append(float(np.median(error)));row['p90_px'].append(float(np.quantile(error,.9)))
            if offset==0.:
                row['baseline_m']=float(np.linalg.norm(t));row['camera_rotation_deg']=float(np.rad2deg((cq[1].inv()*cq[0]).magnitude()))
                row['gimbal_rotation_deg']=float(np.rad2deg((eq[1].inv()*eq[0]).magnitude()))
        if row['baseline_m']>=.01:rows.append(row)
    groups={}
    for name,rs in [('all',rows),('turning_gimbal',[r for r in rows if r['gimbal_rotation_deg']>.3]),('quiet_gimbal',[r for r in rows if r['gimbal_rotation_deg']<.05])]:
        if not rs:continue
        med=np.median([r['median_px'] for r in rs],axis=0)
        groups[name]={'pairs':len(rs),'median_pair_error_px':med.tolist(),'best_offset_s':float(offsets[np.argmin(med)]),'zero_offset_pair_median_p10_p50_p90_px':np.quantile([r['median_px'][5] for r in rs],[.1,.5,.9]).tolist()}
    result={'scope':__doc__,'flight':str(a.flight),'offsets_s':offsets.tolist(),'groups':groups,'pairs':rows}
    a.output.parent.mkdir(exist_ok=True,parents=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(groups,indent=2))


if __name__=='__main__':main()
