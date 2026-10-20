#!/usr/bin/env python3
import json,numpy as np
from scipy.spatial import cKDTree
from analyze_zab_runs import OUT,pose
rows=[]
for f in sorted(OUT.glob('*/metrics.json')):
 r=json.loads(f.read_text());points=np.load(f.parent/'points.npy');d,idx=cKDTree(points).query(points,k=16);near=points[idx];dev=near-near.mean(1)[:,None,:];ev,vec=np.linalg.eigh(np.einsum('nki,nkj->nij',dev,dev)/16);mask=(ev[:,0]/np.maximum(ev.sum(1),1e-12)<.03)&(d[:,-1]<1.5);n=vec[mask,:,0];jrot=np.cross(points[mask],n);rot_ev,rot_v=np.linalg.eigh(jrot.T@jrot/max(1,len(jrot)));r['lidar']['rotation_proxy_eigenvalues']=rot_ev.tolist();r['lidar']['rotation_weak_ratio']=float(rot_ev[0]/rot_ev[-1]);r['lidar']['rotation_weak_to_translation_weak']=float(abs(rot_v[:,0]@np.linalg.eigh(n.T@n)[1][:,0]));f.write_text(json.dumps(r,indent=2)+'\n')
 row=dict(label=r['request']['label'],stage=r['request']['stage'],t=r['request']['time'],ratio=r['lidar']['weak_to_strong_ratio'],alignment=r['lidar']['weak_alignment_to_motion'],rot_ratio=r['lidar']['rotation_weak_ratio'],valid_points=r['lidar']['valid_0p8_30m'],corners=[c['corners'] for c in r['cameras'].values()],dark=[round(c['dark_fraction'],3) for c in r['cameras'].values()],survival=[round(c.get('lk_survival_fraction',0),3) for c in r['cameras'].values()],coverage=[round(c['corner_grid_coverage'],3) for c in r['cameras'].values()]);rows.append(row)
(OUT/'probe_summary.json').write_text(json.dumps(rows,indent=2)+'\n')
for r in rows:print(r)
