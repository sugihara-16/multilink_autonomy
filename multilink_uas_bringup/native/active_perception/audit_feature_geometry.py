#!/usr/bin/env python3
"""Post-hoc triangulation using GT camera poses and ROVIO's accepted feature bearings."""
import argparse,json
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from report_routes import poses,interp,load
from core import camera_rotation
p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('features',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True)
d=np.genfromtxt(a.features,delimiter=',',names=True);route=load(a.flight/'route_tracking.csv');track=route['route_t']>0;lo=route['sim_t'][track][0]*1e9;hi=route['sim_t'][track][-1]*1e9
d=d[(d['stamp_ns']>=lo)&(d['stamp_ns']<=hi)&(d['status']==4)]
mount=np.array(json.loads((a.flight/'camera_manifest.json').read_text())['cameras'][0]['T_FC_camera']);rot=Rotation.from_matrix(camera_rotation(mount[:3,:3],0,0));pos,q=interp(poses(a.flight/'truth.csv'),d['stamp_ns']);origins=pos+q.apply(mount[:3,3]);bear=np.c_[d['nx'],d['ny'],d['nz']];rays=(q*rot).apply(bear);rays/=np.linalg.norm(rays,axis=1)[:,None]
K=yaml.safe_load((a.flight/'cam0.yaml').read_text())['camera_matrix']['data'];fx=K[0];rows=[]
for key in np.unique(d['id']):
 mask=d['id']==key;f=d[mask];o=origins[mask];v=rays[mask]
 if len(f)<10:continue
 baseline=np.linalg.norm(o-o[0],axis=1).max();parallax=np.rad2deg(np.arccos(np.clip(v@v[0],-1,1))).max()
 if baseline<.3 or parallax<1:continue
 project=np.eye(3)[None,:,:]-v[:,:,None]*v[:,None,:];H=project.sum(axis=0);b=np.einsum('nij,nj->i',project,o)
 if np.linalg.cond(H)>1e7:continue
 point=np.linalg.solve(H,b);dist=np.linalg.norm(point-o,axis=1);pred=(point-o)/dist[:,None];angle=np.arccos(np.clip(np.sum(pred*v,axis=1),-1,1));err=angle*fx;positive=np.sum((point-o)*v,axis=1)>0
 ratio=np.median(f['depth_m']/dist)
 rows.append(dict(id=int(key),observations=len(f),start_sim_s=float(f['stamp_ns'][0]*1e-9),duration_s=float((f['stamp_ns'][-1]-f['stamp_ns'][0])*1e-9),baseline_m=float(baseline),parallax_deg=float(parallax),reprojection_rms_px=float(np.sqrt(np.mean(err**2))),positive_fraction=float(positive.mean()),true_triangulated_median_range_m=float(np.median(dist)),estimated_median_range_m=float(np.median(f['depth_m'])),median_depth_ratio=float(ratio),median_u=float(np.median(f['nx']/f['nz']*fx+K[2])),median_v=float(np.median(f['ny']/f['nz']*fx+K[5]))))
good=[r for r in rows if r['reprojection_rms_px']<2 and r['positive_fraction']>.9]
r=dict(scope='GT only for diagnosis. Uses accepted posterior feature bearings, not independent raw pixel measurements. Tracks with >.3m baseline and >1deg parallax; consistent tracks require reprojection RMS<2 pixels and positive depth>90%.',accepted_observations=len(d),unique_features=len(np.unique(d['id'])),triangulatable_tracks=len(rows),consistent_tracks=len(good),depth_ratio_percentiles=np.percentile([r['median_depth_ratio'] for r in good],[0,10,50,90,100]).tolist() if good else [],reprojection_rms_percentiles_px=np.percentile([r['reprojection_rms_px'] for r in rows],[0,10,50,90,100]).tolist() if rows else [],tracks=rows)
(a.output/'feature_geometry.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='tracks'},indent=2))
