#!/usr/bin/env python3
"""Compare tracked feature distance with world-mesh ray distances (diagnostic only)."""
import argparse,json,re
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/vio_audit'
def main(label):
 inp=OUT/'inputs'/label;meta=json.loads((inp/'manifest.json').read_text());source=ROOT/'logs'/meta['source']['run'];truth=np.genfromtxt(source/'truth_raw.csv',delimiter=',',names=True);tt=truth['t'];pos=np.column_stack([truth[k] for k in 'xyz']);qr=Rotation.from_quat(np.column_stack([truth['q'+k] for k in 'xyzw']));slerp=Slerp(tt,qr)
 mesh=trimesh.load('/tmp/zab_collision.ply',process=False);tri=mesh.triangles;lo=pos.min(0)-65;hi=pos.max(0)+65;mask=np.all(tri.max(1)>=lo,axis=1)&np.all(tri.min(1)<=hi,axis=1);mesh=mesh.submesh([np.flatnonzero(mask)],append=True);del tri;print('MESH',label,len(mesh.faces),flush=True)
 text=(source/'rovio.info').read_text();cams=[]
 for i in range(3):
  b=re.search(r'Camera'+str(i)+r'\s*\{([^}]+)',text)[1];v={k:float(x) for k,x in re.findall(r'(qCM_\w|MrMC_\w)\s+([^;\s]+)',b)};cams.append((Rotation.from_quat([v['qCM_'+k] for k in 'xyzw']).inv(),np.array([v['MrMC_'+k] for k in 'xyz'])))
 for variant in ['baseline','depth10','features75','gyro_model_matched']:
  p=OUT/'experiments'/label/variant
  if not (p/'run.json').exists():continue
  if (p/'mesh_depth.json').exists():continue
  f=np.genfromtxt(p/'result_features.csv',delimiter=',',names=True);take=(f['stamp_ns']%5000000000==0)&(f['stamp_ns']>=meta['window']['start_ns'])&(f['stamp_ns']<=meta['window']['end_ns']);f=f[take];t=f['stamp_ns']*1e-9;q=slerp(t);centers=np.column_stack([np.interp(t,tt,pos[:,j]) for j in range(3)]);bearings=np.column_stack([f['b'+k] for k in 'xyz']);origin=np.empty_like(centers);direction=np.empty_like(centers)
  for c,(r,p0) in enumerate(cams):
   m=f['cam']==c;origin[m]=centers[m]+q[m].apply(p0);direction[m]=q[m].apply(r.apply(bearings[m]))
  distances=np.full(len(t),np.nan)
  for start in range(0,len(t),128):
   end=min(len(t),start+128);hits,rays,faces=mesh.ray.intersects_location(origin[start:end],direction[start:end],multiple_hits=False);distances[start+rays]=np.linalg.norm(hits-origin[start+rays],axis=1)
  own=np.array([f[f'status{int(c)}'][i] for i,c in enumerate(f['cam'])]);valid=np.isfinite(distances)&(distances>.8)&(distances<60)&(own==4);ratio=f['depth'][valid]/distances[valid];summary=dict(count=int(valid.sum()),ratio_quantiles=np.quantile(ratio,[.1,.5,.9]).tolist(),scene_distance_quantiles=np.quantile(distances[valid],[.1,.5,.9]).tolist(),estimated_distance_quantiles=np.quantile(f['depth'][valid],[.1,.5,.9]).tolist(),method='World collision mesh uses same visual mesh. True FC pose + calibrated camera, sampled 5s, own-camera TRACKED only. Robot geometry omitted; not a per-feature correspondence proof.')
  rows=np.column_stack([f['stamp_ns'],f['cam'],f['id'],f['depth'],distances,own]);np.savetxt(p/'mesh_depth.csv',rows,delimiter=',',header='stamp_ns,cam,id,estimated_depth_m,mesh_depth_m,status',comments='',fmt=['%.0f','%.0f','%.0f','%.8g','%.8g','%.0f']);(p/'mesh_depth.json').write_text(json.dumps(summary,indent=2));print(label,variant,summary,flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('labels',nargs='+');a=p.parse_args()
 for label in a.labels:main(label)
