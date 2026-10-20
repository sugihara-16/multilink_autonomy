#!/usr/bin/env python3
"""Monocular ablation on five diverged flights; preregistered 100 m / 5 s early-stop.
Run in clean ROS2 Humble environment. No sensor results are retained.
"""
import argparse,concurrent.futures,copy,csv,hashlib,heapq,json,os,shutil,signal,struct,subprocess,time,fcntl
from pathlib import Path
import cv2,numpy as np,yaml,rosbag2_py
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import fast_stream_transport as wire
ROOT=Path(__file__).resolve().parent;LOG=ROOT/'logs/zab-offline';OUT=LOG/'fast_monocular_failures';WS=Path('/root/multilink_autonomy_ws/comparison_ws')
TOPICS=['/dragon/imu/raw','/dragon/lidar/points']+[f'/dragon/cam_{c}/image' for c in wire.CAMERAS]+['/dragon/ground_truth']
ENV=dict(HOME='/root',USER='root',PATH='/usr/local/bin:/usr/bin:/bin',LANG='C.UTF-8')
cv2.setNumThreads(1)
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(2**20),b''):h.update(b)
 return h.hexdigest()
def write(p,d):
 q=Path(str(p)+'.tmp');q.write_text(json.dumps(d,indent=2));q.replace(p)
def config(src):
 cfg=yaml.safe_load((LOG/'fast_multicamera/link1_link3_link4/run2a_end/trial1/config.yaml').read_text())
 cameras=json.loads((src/'camera_manifest.json').read_text())['cameras'];maps=[]
 for j,c in enumerate(cameras):
  assert c['link']==wire.CAMERAS[j] and c['id']==j
  rect=yaml.safe_load((src/f'cam{j}.yaml').read_text());raw=yaml.safe_load((src/f'raw_cam{j}.yaml').read_text());K=np.array(rect['camera_matrix']['data']).reshape(3,3)
  maps.append(cv2.initUndistortRectifyMap(np.array(raw['camera_matrix']['data']).reshape(3,3),np.array(raw['distortion_coefficients']['data']),np.eye(3),K,(480,360),cv2.CV_32FC1))
  ext=cfg['parameters']['extrin_calib'];R=Rotation.from_quat(c['qCM']).as_matrix();rig=cfg['parameters']['multicamera'][c['link']]
  rig['Rcl']=(R@np.array(ext['extrinsic_R']).reshape(3,3)).reshape(-1).tolist();rig['Pcl']=(R@(np.array(ext['extrinsic_T'])-np.array(c['MrMC']))).tolist()
  rig['camera'].update(cam_fx=float(K[0,0]),cam_fy=float(K[1,1]),cam_cx=float(K[0,2]),cam_cy=float(K[1,2]))
 cfg['parameters']['extrin_calib'].update({k:cfg['parameters']['multicamera']['link3'][k] for k in ['Rcl','Pcl']})
 cfg['parameters'].pop('multicamera')
 assert not cfg['parameters']['vio']['exposure_estimate_en']
 return cfg,maps

def process(entry):
 d=OUT/entry['label'];d.mkdir(exist_ok=True)
 with (d/'processing.lock').open('w') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX);return process_locked(entry)

def process_locked(entry):
 label=entry['label'];d=OUT/label;d.mkdir(exist_ok=True);src=ROOT/'logs'/entry['run'];window=json.loads((LOG/'results'/label/'window.json').read_text());end=window['end_ns']+200000000
 if (d/'summary.json').exists():
  saved=json.loads((d/'summary.json').read_text());assert saved['returncode']==0 or saved.get('recovered_complete_inference',False)
  if not (d/'metrics.json').exists():finalize(d)
  return
 start=time.monotonic();cfg,maps=config(src);(d/'config.yaml').write_text(yaml.safe_dump(cfg))
 names=yaml.safe_load((src/'cloud_metadata.yaml').read_text())['rosbag2_bagfile_information']['relative_file_paths'];receipts={n:json.loads((src/'cloud_chunks'/(Path(n).stem+'.json')).read_text()) for n in names}
 names=[n for n in names if receipts[n]['file_metadata']['starting_time']['nanoseconds_since_epoch']<=end]
 cache=OUT/'cache';cache.mkdir(exist_ok=True)
 def fetch(n):
  with (cache/(label+'_'+n+'.lock')).open('w') as lock:
   fcntl.flock(lock,fcntl.LOCK_EX);return fetch_locked(n)
 def fetch_locked(n):
  f=cache/(label+'_'+n);expected=receipts[n]['md5']
  def valid():
   if not f.exists():return False
   h=hashlib.md5()
   with f.open('rb') as s:
    for b in iter(lambda:s.read(2**20),b''):h.update(b)
   return h.hexdigest()==expected
  if not valid():
   assert shutil.disk_usage('/mnt/c').free>5*2**30,'Physical Windows disk low'
   with (d/'download.log').open('a') as log:
    subprocess.run(['rclone','copyto',entry['remote']+'/'+n,str(f),'--retries','5','--low-level-retries','10'],stdout=log,stderr=log,check=True)
   assert valid(),f'MD5 mismatch: {n}'
  return f
 counts=[0]*5;hashes=[hashlib.sha256() for _ in range(5)];first=[None]*5;last=[None]*5;image_stamps=[[] for _ in range(3)];heap=[];serial=0;maxstamp=-1;emitted=-1;truth_count=0;chunks=[]
 child=None;stop_decision=None
 from check_fast_monocular_failures import detect_divergence
 with (d/'run.log').open('w') as log,(d/'truth_full.csv').open('w') as tf,concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
  pending={i:pool.submit(fetch,n) for i,n in enumerate(names[:8])}
  writer=csv.writer(tf);writer.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz'])
  child=subprocess.Popen(['bash',str(ROOT/'comparison_ros1.sh'),'python3',str(ROOT/'run_fast_stream_child.py'),str(d/'config.yaml'),str(d/'odometry_full.csv')],env=ENV,stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,bufsize=2**20,start_new_session=True)
  def drain(limit):
   nonlocal emitted
   while heap and heap[0][0]<=limit:
    t,_,idx,payload=heapq.heappop(heap);assert t>=emitted,'Source timestamp disorder exceeds reorder buffer';emitted=t
    assert last[idx] is None or t>last[idx],f'Duplicate/reversed source stamp {idx} {t}'
    child.stdin.write(wire.packet(2 if idx==3 else idx,payload));counts[idx]+=1;hashes[idx].update(payload);last[idx]=t
    if first[idx] is None:first[idx]=t
    if idx>=2:image_stamps[idx-2].append(t)
  try:
   for i,n in enumerate(names):
    f=pending.pop(i).result()
    if i+8<len(names):pending[i+8]=pool.submit(fetch,names[i+8])
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(f),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=[TOPICS[j] for j in (0,1,3,5)]));types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in TOPICS}
    while r.has_next():
     topic,raw,stored=r.read_next();m=deserialize_message(raw,types[topic]);t=m.header.stamp.sec*10**9+m.header.stamp.nanosec
     if t>end:continue
     idx=TOPICS.index(topic)
     if idx==5:
      p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear;w=m.twist.twist.angular;writer.writerow([t,p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z]);truth_count+=1;continue
     if t<2003000000:continue
     if idx==0:payload=wire.imu(t,[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
     elif idx==1:
      dt=np.dtype({'names':['x','y','z','intensity','ring'],'formats':['<f4']*4+['<u2'],'offsets':[next(x.offset for x in m.fields if x.name==k) for k in ['x','y','z','intensity','ring']],'itemsize':m.point_step});a=np.frombuffer(m.data,dt);payload=wire.cloud(t,np.stack([a[k] for k in ['x','y','z','intensity','ring']],axis=-1))
     else:
      assert m.encoding in ['rgb8','bgr8'];a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3);gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY);gray=cv2.remap(gray,*maps[idx-2],interpolation=cv2.INTER_LINEAR);payload=wire.image(t,idx-2,gray)
     heapq.heappush(heap,(t,serial,idx,payload));serial+=1;maxstamp=max(maxstamp,t);drain(maxstamp-1000000000)
    del r
    size=f.stat().st_size;f.unlink();chunks.append(dict(name=n,md5=receipts[n]['md5'],bytes=size));child.stdin.flush();tf.flush()
    progress=dict(label=label,chunk=i+1,chunks=len(names),input_time_s=emitted/1e9,elapsed_s=time.monotonic()-start,counts=counts,physical_free_gib=shutil.disk_usage('/mnt/c').free/2**30)
    write(OUT/f'progress_{os.getpid()}.json',progress);write(d/'progress.json',progress);print(json.dumps(progress),flush=True)
    stop_decision=detect_divergence(d,window) if i+1<len(names) else None
    if stop_decision:
     write(d/'stop_decision.json',stop_decision)
     print('EARLY_STOP',label,json.dumps(stop_decision),flush=True)
     for future in pending.values():future.cancel()
     break
   drain(2**63-1);child.stdin.close()
   write(d/'input_audit.json',dict(counts=counts,first_stamp_ns=first,last_stamp_ns=last,serialized_sha256=[h.hexdigest() for h in hashes],chunks=chunks,truth_count=truth_count))
   rc=child.wait(timeout=600)
   if rc:raise RuntimeError(f'FAST exit {rc}')
  finally:
   if child.poll() is None:os.killpg(child.pid,signal.SIGINT);child.wait(timeout=20)
 summary=dict(returncode=rc,wall_s=time.monotonic()-start,cameras=['link3'],window=window,source=entry,adapter='ROS1 serialized stream, globally reordered with 1 s buffer; no estimator restart at chunk boundaries',counts=counts,first_stamp_ns=first,last_stamp_ns=last,serialized_sha256=[h.hexdigest() for h in hashes],chunks=chunks,truth_count=truth_count,binary_sha256=sha(WS/'devel/lib/fast_livo/fastlivo_stream'),exposure_estimate=False,reused=False,early_stop=stop_decision,full_source_replayed=len(chunks)==len(names),condition='upstream monocular branch, link3, fixed exposure; multicamera config absent')
 assert counts[2]==counts[4]==0 and counts[3]>0
 write(d/'summary.json',summary);finalize(d)
 print('COMPLETE',label,flush=True)

def finalize(d):
 from check_fast_monocular_failures import finalize
 finalize(d)

def main():
 p=argparse.ArgumentParser();p.add_argument('--labels',nargs='*');p.add_argument('--master-port',type=int,default=11376);a=p.parse_args();ENV['FAST_STREAM_MASTER_URI']=f'http://127.0.0.1:{a.master_port}';OUT.mkdir(exist_ok=True)
 entries=[e for e in json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags'] if e['label'] in ['run1_prefix','run1_middle','run1_tail_a1','run1_tail_b1','run2b']]
 for e in entries:
  if e['label'] not in ['run1_prefix','run1_middle','run1_tail_a1','run1_tail_b1','run2b']:continue
  if a.labels and e['label'] not in a.labels:continue
  process(e)
 write(OUT/'manifest.json',{e['label']:dict(complete=(OUT/e['label']/'metrics.json').exists(),directory=str(OUT/e['label'])) for e in entries})
if __name__=='__main__':main()
