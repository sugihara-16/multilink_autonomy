#!/usr/bin/env python3
"""Read-only probes of original archived images/pointclouds. Bounded cache."""
import json,hashlib,subprocess,concurrent.futures,csv
from pathlib import Path
import numpy as np,cv2,yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from analyze_zab_runs import ROOT,OUT,pose
cv2.setNumThreads(1)
TOPICS=['/dragon/lidar/points',*[f'/dragon/cam_link{i}/image' for i in [1,3,4]],'/dragon/imu/raw','/dragon/joint_states']
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def gray(m):
 a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)
 if m.encoding in ['mono8','8UC1']:return a[:,:m.width].copy()
 a=a[:,:m.width*3].reshape(m.height,m.width,3)
 return cv2.cvtColor(a,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
def image_metrics(im):
 corners=cv2.goodFeaturesToTrack(im,1000,.01,8)
 xy=corners[:,0,:] if corners is not None else np.empty((0,2));grid={(min(7,int(x/im.shape[1]*8)),min(5,int(y/im.shape[0]*6))) for x,y in xy}
 hist=np.bincount(im.ravel(),minlength=256)/im.size;h=hist[hist>0]
 return dict(corners=len(xy),corner_grid_coverage=len(grid)/48,entropy_bits=float(-np.sum(h*np.log2(h))),dark_fraction=float(np.mean(im<10)),mean_gray=float(im.mean()),laplacian_variance=float(cv2.Laplacian(im,cv2.CV_64F).var()))
def cloud_metrics(m,label,t):
 types={7:'<f4',8:'<f8',2:'u1',4:'<u2',6:'<u4',5:'<i4'}
 dt=np.dtype({'names':[f.name for f in m.fields],'formats':[types[f.datatype] for f in m.fields],'offsets':[f.offset for f in m.fields],'itemsize':m.point_step})
 a=np.frombuffer(m.data,dtype=dt);xyz=np.column_stack([a[n] for n in ['x','y','z']]);finite=np.isfinite(xyz).all(1);rr=np.linalg.norm(xyz,axis=1);valid=finite&(rr>=.8)&(rr<30);p=xyz[valid].astype(float)
 # Voxel reduce so repeated sampling of a nearby wall does not dominate proxy.
 _,idx=np.unique(np.floor(p/.3).astype(int),axis=0,return_index=True);p=p[idx];tree=cKDTree(p);d,idx=tree.query(p,k=16);neighbors=p[idx];center=neighbors.mean(1);delta=neighbors-center[:,None,:];cov=np.einsum('nki,nkj->nij',delta,delta)/16;ev,vec=np.linalg.eigh(cov);mask=(ev[:,0]/np.maximum(ev.sum(1),1e-12)<.03)&(d[:,-1]<1.5);normals=vec[mask,:,0];G=normals.T@normals/max(1,len(normals));eig,basis=np.linalg.eigh(G)
 tt,tp,tr=pose(label);i=int(np.argmin(abs(tt-t)));j0=max(0,np.searchsorted(tt,t-1));j1=min(len(tt)-1,np.searchsorted(tt,t+1));v=tp[j1]-tp[j0];v/=max(np.linalg.norm(v),1e-12);weak=tr[i].apply(basis[:,0]);J=np.column_stack([np.cross(p[mask],normals)/max(np.median(np.linalg.norm(p[mask],axis=1)),1),normals]);eig6=np.linalg.eigvalsh(J.T@J/max(1,len(J)))
 return dict(stamp_s=stamp(m),total=len(xyz),valid_0p8_30m=int(valid.sum()),finite_fraction=float(finite.mean()),range_median=float(np.median(rr[valid])),voxel_points=len(p),planar_normals=len(normals),normal_eigenvalues=eig.tolist(),weak_to_strong_ratio=float(eig[0]/eig[-1]),weak_world_direction=weak.tolist(),weak_alignment_to_motion=float(abs(weak@v)),scaled_sixd_eigenvalues=eig6.tolist()),p

def process(job,file):
 requests=job['requests'];mins=min(r['time'] for r in requests)-.2;maxs=max(r['time'] for r in requests)+1.2
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(file),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS));types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in TOPICS};keep={i:{} for i in range(len(requests))};imu=[];joints=[]
 while reader.has_next():
  topic,raw,stored=reader.read_next()
  if stored*1e-9<mins-.2:continue
  if stored*1e-9>maxs+.5:break
  m=deserialize_message(raw,types[topic]);t=stamp(m)
  for i,r in enumerate(requests):
   if r['time']-.15<=t<=r['time']+1.1:keep[i].setdefault(topic,[]).append(m)
 del reader
 source=ROOT/'logs'/job['source_run']
 for i,r in enumerate(requests):
  label,stage,t=r['label'],r['stage'],r['time'];dest=OUT/(label+'_'+stage);dest.mkdir(exist_ok=True);result=dict(request=r,source_chunk=job['receipt']['cloud_file'],cameras={})
  msgs=keep[i].get('/dragon/lidar/points',[]);assert msgs;msg=min(msgs,key=lambda m:abs(stamp(m)-t));result['lidar'],points=cloud_metrics(msg,label,t);np.save(dest/'points.npy',points)
  for cam,link in enumerate([1,3,4]):
   msgs=keep[i].get(f'/dragon/cam_link{link}/image',[]);assert msgs;msgs=sorted(msgs,key=lambda m:stamp(m));first=min(range(len(msgs)),key=lambda j:abs(stamp(msgs[j])-t));msgs=msgs[first:]
   raw=yaml.safe_load((source/f'raw_cam{cam}.yaml').read_text());rect=yaml.safe_load((source/f'cam{cam}.yaml').read_text());K=np.array(raw['camera_matrix']['data']).reshape(3,3);D=np.array(raw['distortion_coefficients']['data']);P=np.array(rect['camera_matrix']['data']).reshape(3,3);fn=cv2.fisheye.initUndistortRectifyMap if raw['distortion_model']=='equidistant' else cv2.initUndistortRectifyMap;maps=fn(K,D,np.eye(3),P,(rect['image_width'],rect['image_height']),cv2.CV_32FC1)
   images=[cv2.remap(gray(m),*maps,interpolation=cv2.INTER_LINEAR) for m in msgs];cv2.imwrite(str(dest/f'cam_link{link}.png'),images[0]);met=image_metrics(images[0]);met['stamp_s']=stamp(msgs[0]);met['sample_count']=len(images)
   corners=cv2.goodFeaturesToTrack(images[0],1000,.01,8)
   if corners is not None and len(images)>1:
    initial=corners.copy();cur=corners.copy();alive=np.ones(len(cur),bool)
    for prev,nextim in zip(images[:-1],images[1:]):
     nxt,status,_=cv2.calcOpticalFlowPyrLK(prev,nextim,cur,None);back,status2,_=cv2.calcOpticalFlowPyrLK(nextim,prev,nxt,None);alive&=(status[:,0]>0)&(status2[:,0]>0)&(np.linalg.norm(back[:,0]-cur[:,0],axis=1)<1);cur=nxt
    met['tracked_duration_s']=stamp(msgs[-1])-stamp(msgs[0]);met['lk_survival_fraction']=float(alive.mean());met['lk_displacement_px']=float(np.median(np.linalg.norm(cur[alive,0]-initial[alive,0],axis=1))) if np.any(alive) else None
   result['cameras'][str(link)]=met
  msgs=keep[i].get('/dragon/imu/raw',[])
  if msgs:
   acc=np.array([[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z] for m in msgs]);gyro=np.array([[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z] for m in msgs]);result['imu']=dict(count=len(msgs),acc_norm_mean=float(np.linalg.norm(acc,axis=1).mean()),acc_norm_std=float(np.linalg.norm(acc,axis=1).std()),gyro_norm_max=float(np.linalg.norm(gyro,axis=1).max()))
  msgs=keep[i].get('/dragon/joint_states',[])
  if msgs:result['joints']={n:dict(mean=float(np.mean([m.position[j] for m in msgs])),range=float(np.ptp([m.position[j] for m in msgs]))) for j,n in enumerate(msgs[0].name) if 'joint' in n}
  (dest/'metrics.json').write_text(json.dumps(result,indent=2)+'\n');print('PROBED',label,stage,result['lidar']['weak_to_strong_ratio'],flush=True)

def main():
 jobs=json.loads((OUT/'probe_jobs.json').read_text());cache=OUT/'cache';cache.mkdir(exist_ok=True)
 def fetch(job):
  if all((OUT/(r['label']+'_'+r['stage'])/'metrics.json').exists() for r in job['requests']):return None
  r=job['receipt'];f=cache/(job['source_run']+'_'+r['cloud_file']);subprocess.run(['rclone','copyto',r['remote'],str(f),'--retries','4'],check=True);h=hashlib.md5()
  with f.open('rb') as stream:
   for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
  assert h.hexdigest()==r['md5'];return f
 with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
  pending={i:pool.submit(fetch,j) for i,j in enumerate(jobs[:3])}
  for i,job in enumerate(jobs):
   f=pending.pop(i).result()
   if i+3<len(jobs):pending[i+3]=pool.submit(fetch,jobs[i+3])
   if f:process(job,f);f.unlink()
 print('ALL SENSOR PROBES COMPLETE',flush=True)
if __name__=='__main__':main()
