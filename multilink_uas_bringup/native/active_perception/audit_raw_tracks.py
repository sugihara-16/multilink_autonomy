#!/usr/bin/env python3
"""Independent image-only patch tracking; GT is used only for post-hoc geometry.
Tracks start at a ROVIO landmark pixel, then evolve using raw image pairs only.
No ROVIO predictions, depths, poses or posterior bearings steer the tracker.
"""
import argparse,json,struct
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation
from report_routes import poses,interp,load

def records(path,kind_needed):
 with path.open('rb') as f:
  while header:=f.read(8):
   kind,size=struct.unpack('<II',header)
   if kind!=kind_needed:f.seek(size,1);continue
   data=f.read(size);assert len(data)==size
   _,s,ns,n=struct.unpack_from('<IIII',data);yield s*10**9+ns,data,16+n

def images(path):
 for stamp,data,off in records(path,1):
  h,w,n=struct.unpack_from('<III',data,off);off+=12;encoding=data[off:off+n];off+=n
  endian,step,size=struct.unpack_from('<BII',data,off);off+=9
  assert encoding==b'mono8' and size==h*step
  yield stamp,np.ndarray((h,w),dtype=np.uint8,buffer=data,offset=off,strides=(step,1)).copy()

def flow(prev,cur,points):
 p=np.asarray(points,np.float32).reshape(-1,1,2)
 q,status,err=cv2.calcOpticalFlowPyrLK(prev,cur,p,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
 back,s2,_=cv2.calcOpticalFlowPyrLK(cur,prev,q,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
 fb=np.linalg.norm(back[:,0]-p[:,0],axis=1);q=q[:,0]
 ok=(status[:,0]>0)&(s2[:,0]>0)&(fb<.5)&(q[:,0]>=10)&(q[:,1]>=10)&(q[:,0]<cur.shape[1]-10)&(q[:,1]<cur.shape[0]-10)
 return q,ok,fb

def triangulate(origins,rays):
 P=np.eye(3)[None,:,:]-rays[:,:,None]*rays[:,None,:];H=P.sum(axis=0)
 if np.linalg.cond(H)>1e7:return None
 return np.linalg.solve(H,np.einsum('nij,nj->i',P,origins))

def main():
 p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('features',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True);cv2.setNumThreads(1)
 f=np.genfromtxt(a.features,delimiter=',',names=True);gt=poses(a.flight/'truth.csv');route=load(a.flight/'route_tracking.csv');m=route['route_t']>0;lo=round(route['sim_t'][m][0]*1e9);hi=round(route['sim_t'][m][-1]*1e9)
 has_measurements='mx' in f.dtype.names
 f=f[(f['stamp_ns']>=lo)&(f['stamp_ns']<=hi)&(f['status']==4)]
 if has_measurements:f=f[(f['age_s']>1e-8)&(f['tracked_count']>0)]  # newly detected landmarks have no alignment measurement yet
 groups={}
 boundaries=np.r_[0,np.flatnonzero(np.diff(f['stamp_ns']))+1,len(f)]
 for start,end in zip(boundaries[:-1],boundaries[1:]):groups[int(f['stamp_ns'][start])]=f[start:end]
 has_measurements='mx' in f.dtype.names
 K=np.array(yaml.safe_load((a.flight/'cam0.yaml').read_text())['camera_matrix']['data']).reshape(3,3)
 extr={stamp:np.array(struct.unpack_from('<7d',data,off)) for stamp,data,off in records(a.flight/'vio_input.bin',2)}
 tracks={};done=[];prev=None;frame_count=0;observations=[];failed=0;attempted=0
 for stamp,im in images(a.flight/'vio_input.bin'):
  if stamp<lo:continue
  if stamp>hi:break
  frame_count+=1
  if prev is not None and tracks:
   keys=list(tracks);points=np.array([tracks[k]['pixel'] for k in keys]);nxt,ok,fb=flow(prev,im,points);attempted+=len(keys);failed+=int((~ok).sum())
   for i,key in enumerate(keys):
    if not ok[i]:done.append(tracks.pop(key));continue
    tracks[key]['pixel']=nxt[i]
  present={int(row['id']):row for row in groups.get(stamp,[])}
  for key in list(tracks):
   if key not in present:done.append(tracks.pop(key))
  for key,row in present.items():
   bearing=np.array([row['nx'],row['ny'],row['nz']]);pixel=(K@bearing)[:2]/bearing[2]
   measurement=np.array([row['mx'],row['my'],row['mz']]) if has_measurements else bearing
   measurement_pixel=(K@measurement)[:2]/measurement[2]
   if key not in tracks:
    if min(pixel)<10 or pixel[0]>=im.shape[1]-10 or pixel[1]>=im.shape[0]-10:continue
    tracks[key]=dict(id=key,pixel=measurement_pixel,rows=[])
   tr=tracks[key];v=[stamp,*tr['pixel'],float(row['depth_m']),float(np.linalg.norm(tr['pixel']-pixel)),*pixel,*measurement_pixel,float(np.linalg.norm(tr['pixel']-measurement_pixel))];tr['rows'].append(v);observations.append(v)
  prev=im
 done+=list(tracks.values());rows=[]
 for track in done:
  d=np.array(track['rows'])
  if len(d)<10:continue
  ts=d[:,0];p,q=interp(gt,ts);ex=np.array([extr[int(t)] for t in ts]);origins=p+q.apply(ex[:,:3]);R=q*Rotation.from_quat(ex[:,3:]);rays=R.apply(np.c_[d[:,1:3],np.ones(len(d))]@np.linalg.inv(K).T);rays/=np.linalg.norm(rays,axis=1)[:,None]
  baseline=np.linalg.norm(origins-origins[0],axis=1).max();parallax=np.rad2deg(np.arccos(np.clip(rays@rays[0],-1,1))).max()
  if baseline<.3 or parallax<1:continue
  point=triangulate(origins,rays)
  if point is None:continue
  cam=R.inv().apply(point-origins);uv=cam@K.T;uv=uv[:,:2]/uv[:,2:];errors=np.linalg.norm(uv-d[:,1:3],axis=1);distance=np.linalg.norm(point-origins,axis=1)
  rows.append(dict(id=track['id'],observations=len(d),start_ns=int(ts[0]),duration_s=float((ts[-1]-ts[0])*1e-9),baseline_m=float(baseline),parallax_deg=float(parallax),raw_geometry_rms_px=float(np.sqrt(np.mean(errors**2))),positive_fraction=float((cam[:,2]>0).mean()),posterior_vs_raw_median_px=float(np.median(d[:,4])),posterior_vs_raw_max_px=float(d[:,4].max()),measurement_vs_raw_median_px=float(np.median(d[:,9])),measurement_geometry_rms_px=float(np.sqrt(np.mean(np.sum((uv-d[:,7:9])**2,axis=1)))),posterior_geometry_rms_px=float(np.sqrt(np.mean(np.sum((uv-d[:,5:7])**2,axis=1)))),median_depth_ratio=float(np.median(d[:,3]/distance))))
 good=[r for r in rows if r['raw_geometry_rms_px']<2 and r['positive_fraction']>.9]
 result=dict(scope=__doc__,flight=str(a.flight),frames=frame_count,flow_attempts=attempted,flow_rejected=failed,track_segments=len(done),eligible_tracks=len(rows),consistent_tracks=len(good),posterior_vs_raw_px_percentiles=np.percentile(np.array(observations)[:,4],[0,50,90,95,99,100]).tolist(),consistent_depth_ratio_percentiles=np.percentile([r['median_depth_ratio'] for r in good],[0,10,50,90,100]).tolist() if good else [],tracks=rows)
 result['actual_rovio_measurements_logged']=has_measurements
 result['measurement_vs_raw_px_percentiles']=np.percentile(np.array(observations)[:,9],[0,50,90,95,99,100]).tolist() if has_measurements else None
 result['raw_consistent_measurement_inconsistent_tracks']=sum(r['measurement_geometry_rms_px']>=2 for r in good) if has_measurements else None
 result['raw_consistent_posterior_inconsistent_tracks']=sum(r['posterior_geometry_rms_px']>=2 for r in good)
 (a.output/'raw_tracks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='tracks'},indent=2))
if __name__=='__main__':main()
