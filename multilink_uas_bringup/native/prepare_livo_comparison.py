#!/usr/bin/env python3
"""Make bounded, lossless rectified-image/IMU experiment inputs from adopted MCAPs."""
import argparse, concurrent.futures, hashlib, json, subprocess, shutil
from pathlib import Path
import cv2, numpy as np, yaml, rosbag2_py
from local_zab_bags import link_local_chunk
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'logs/zab-offline/livo_comparison'
CAM=[f'/dragon/cam_link{i}/image' for i in (1,3,4)]
TOPICS=[CAM[1],'/dragon/lidar/points','/dragon/imu/raw','/dragon/ground_truth','/dragon/joint_states']
cv2.setNumThreads(1)
def ns(m):return m.header.stamp.sec*1000000000+m.header.stamp.nanosec
def main(label):
 entry=next(x for x in json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags'] if x['label']==label)
 src=ROOT/'logs'/entry['run'];dest=OUT/'inputs'/label;dest.mkdir(parents=True,exist_ok=True)
 window=json.loads((ROOT/'logs/zab-offline/results'/label/'window.json').read_text());end=window['end_ns']+200000000
 maps=[]
 for i in range(3):
  raw=yaml.safe_load((src/f'raw_cam{i}.yaml').read_text());rect=yaml.safe_load((src/f'cam{i}.yaml').read_text())
  maps.append(cv2.initUndistortRectifyMap(np.array(raw['camera_matrix']['data']).reshape(3,3),np.array(raw['distortion_coefficients']['data']),np.eye(3),np.array(rect['camera_matrix']['data']).reshape(3,3),(480,360),cv2.CV_32FC1))
 names=yaml.safe_load((src/'cloud_metadata.yaml').read_text())['rosbag2_bagfile_information']['relative_file_paths'];names=[n for n in names if json.loads((src/'cloud_chunks'/(Path(n).stem+'.json')).read_text())['file_metadata']['starting_time']['nanoseconds_since_epoch']<=end]
 cache=OUT/'cache';cache.mkdir(exist_ok=True);audit=[]
 def fetch(n):
  if (dest/(Path(n).stem+'.npz')).exists():return None
  f=cache/(label+'_'+n);expected=json.loads((src/'cloud_chunks'/(Path(n).stem+'.json')).read_text())['md5']
  local=link_local_chunk(label,n,f,expected)
  if not local and f.is_symlink():f.unlink()
  reserve_gib=3 if cache.resolve().is_relative_to(Path('/dev/shm')) else 6
  if not local:assert shutil.disk_usage('/mnt/c').free > reserve_gib*2**30, 'Insufficient physical disk space'
  if not local and cache.resolve().is_relative_to(Path('/dev/shm')):
   assert shutil.disk_usage(cache).free > 2*2**30, 'Insufficient RAM-backed cache space'
  if not local:subprocess.run(['rclone','copyto',entry['remote']+'/'+n,str(f),'--retries','4'],check=True)
  h=hashlib.md5()
  with f.open('rb') as stream:
   for b in iter(lambda:stream.read(2**20),b''):h.update(b)
  assert h.hexdigest()==json.loads((src/'cloud_chunks'/(Path(n).stem+'.json')).read_text())['md5'];return f
 with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
  pending={i:pool.submit(fetch,n) for i,n in enumerate(names[:3])}
  for i,n in enumerate(names):
   f=pending.pop(i).result()
   if i+3<len(names):pending[i+3]=pool.submit(fetch,names[i+3])
   if f is None:continue
   r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(f),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS));types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in TOPICS}
   data={k:[] for k in ['imu_t','imu','imu_q','truth_t','truth','joint_t','joints','order','cloud_t','cloud']+[f'cam{j}_{s}' for j in range(3) for s in ['t','im']]};j_names=None
   while r.has_next():
    topic,raw,stored=r.read_next();m=deserialize_message(raw,types[topic]);t=ns(m)
    if t>end:continue
    if topic in CAM:
     j=CAM.index(topic);a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3);gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
     data['order'].append([t,j+1,len(data[f'cam{j}_t']),stored]);data[f'cam{j}_t'].append(t);data[f'cam{j}_im'].append(cv2.remap(gray,*maps[j],interpolation=cv2.INTER_LINEAR))
    elif topic.endswith('lidar/points'):
     dt=np.dtype({'names':['x','y','z','intensity','ring'],'formats':['<f4','<f4','<f4','<f4','<u2'],'offsets':[next(x.offset for x in m.fields if x.name==k) for k in ['x','y','z','intensity','ring']],'itemsize':m.point_step})
     a=np.frombuffer(m.data,dt); data['order'].append([t,4,len(data['cloud_t']),stored]);data['cloud_t'].append(t);data['cloud'].append(np.stack([a[k] for k in ['x','y','z','intensity','ring']],axis=-1))
    elif topic.endswith('imu/raw'):
     data['order'].append([t,0,len(data['imu_t']),stored]);data['imu_t'].append(t);data['imu'].append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]);q=m.orientation;data['imu_q'].append([q.x,q.y,q.z,q.w])
    elif topic.endswith('ground_truth'):
     p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear;w=m.twist.twist.angular;data['truth_t'].append(t);data['truth'].append([p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z])
    else:data['joint_t'].append(t);data['joints'].append(m.position);j_names=m.name
   del r
   np.savez_compressed(dest/(Path(n).stem+'.npz'),**{k:np.array(v,dtype=np.int64 if k.endswith('_t') or k=='order' else np.uint8 if k.endswith('_im') else np.float32 if k=='cloud' else float) for k,v in data.items()})
   f.unlink();print(label,i+1,'/',len(names),flush=True)
   if j_names:(dest/'joint_names.json').write_text(json.dumps(j_names))
 (dest/'manifest.json').write_text(json.dumps(dict(source=entry,window=window,chunks=[Path(n).stem+'.npz' for n in names],image_conversion='Same cv2 mono8 + remap as vio_images.py; no lossy compression; original integer header and storage timestamps preserved'),indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('labels',nargs='+');a=p.parse_args()
 for label in a.labels:main(label)
