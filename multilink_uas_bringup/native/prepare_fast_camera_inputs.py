#!/usr/bin/env python3
"""Fetch adopted MCAPs in bounded batches; cache link1/link4 and verify link3 preprocessing."""
import concurrent.futures,hashlib,json,subprocess,shutil
from pathlib import Path
import cv2,numpy as np,yaml,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=Path(__file__).resolve().parent;BASE=ROOT/'logs/zab-offline/livo_comparison';OUT=ROOT/'logs/zab-offline/fast_camera_comparison'
CAM=[f'/dragon/cam_link{i}/image' for i in (1,3,4)]
cv2.setNumThreads(1)
def md5(path):
 h=hashlib.md5()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(2**20),b''):h.update(b)
 return h.hexdigest()
def main(label):
 manifest=json.loads((BASE/'inputs'/label/'manifest.json').read_text());src=ROOT/'logs'/manifest['source']['run'];end=manifest['window']['end_ns']+200000000
 dest=OUT/'images'/label;dest.mkdir(parents=True,exist_ok=True);cache=OUT/'download_cache';cache.mkdir(exist_ok=True)
 maps=[]
 for i in range(3):
  raw=yaml.safe_load((src/f'raw_cam{i}.yaml').read_text());rect=yaml.safe_load((src/f'cam{i}.yaml').read_text())
  maps.append(cv2.initUndistortRectifyMap(np.array(raw['camera_matrix']['data']).reshape(3,3),np.array(raw['distortion_coefficients']['data']),np.eye(3),np.array(rect['camera_matrix']['data']).reshape(3,3),(480,360),cv2.CV_32FC1))
 chunks=manifest['chunks']
 def fetch(chunk):
  if (dest/chunk).exists() and (dest/(Path(chunk).stem+'.json')).exists():return None
  assert shutil.disk_usage('/mnt/c').free>6*2**30,'Physical host disk low'
  name=Path(chunk).with_suffix('.mcap').name;p=cache/(label+'_'+name)
  subprocess.run(['rclone','copyto',manifest['source']['remote']+'/'+name,str(p),'--retries','4'],check=True)
  expected=json.loads((src/'cloud_chunks'/(Path(chunk).stem+'.json')).read_text())['md5'];actual=md5(p);assert actual==expected
  return p,actual
 with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
  pending={i:pool.submit(fetch,c) for i,c in enumerate(chunks[:3])}
  for i,chunk in enumerate(chunks):
   result=pending.pop(i).result()
   if result is None:
    if i+3<len(chunks):pending[i+3]=pool.submit(fetch,chunks[i+3])
    continue
   p,checksum=result
   reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(p),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=CAM));typ=get_message('sensor_msgs/msg/Image')
   ts=[[],[],[]];ims=[[],[],[]]
   while reader.has_next():
    topic,raw,stored=reader.read_next();m=deserialize_message(raw,typ);t=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    if t>end:continue
    c=CAM.index(topic);a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
    assert m.encoding in ('rgb8','bgr8');gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
    ts[c].append(t);ims[c].append(cv2.remap(gray,*maps[c],interpolation=cv2.INTER_LINEAR))
   del reader
   ts=[np.asarray(t,np.int64) for t in ts];ims=[np.asarray(a,np.uint8) for a in ims]
   with np.load(BASE/'inputs'/label/chunk) as old:
    assert np.array_equal(ts[1],old['cam1_t']) and np.array_equal(ims[1],old['cam1_im']),'Reference preprocessing differs'
   # MCAP file boundaries can split a synchronized camera set. Exact complete
   # stream timestamps are checked when replacing the reference bag images.
   same_chunk_stamps=np.array_equal(ts[0],ts[1]) and np.array_equal(ts[2],ts[1])
   np.savez_compressed(dest/chunk,cam0_t=ts[0],cam0_im=ims[0],cam2_t=ts[2],cam2_im=ims[2])
   (dest/(Path(chunk).stem+'.json')).write_text(json.dumps(dict(raw_md5=checksum,link3_preprocessing_byte_identical=True,per_chunk_camera_stamps_identical=bool(same_chunk_stamps),camera_frame_counts=[len(t) for t in ts],frames=len(ts[1]),first_ns=int(ts[1][0]) if len(ts[1]) else None,last_ns=int(ts[1][-1]) if len(ts[1]) else None),indent=2))
   p.unlink()
   if i+3<len(chunks):pending[i+3]=pool.submit(fetch,chunks[i+3])
   print(label,i+1,'/',len(chunks),'frames',len(ts[1]),flush=True)
 (dest/'manifest.json').write_text(json.dumps(manifest,indent=2))
if __name__=='__main__':
 import sys
 for label in sys.argv[1:] or ['run2a_end','run1_tail_a2_prefix']:main(label)
