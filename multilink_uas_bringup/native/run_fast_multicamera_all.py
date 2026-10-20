#!/usr/bin/env python3
"""All-flight campaign, bounded Drive cache and one persistent estimator per flight.
Run in clean ROS2 Humble environment. No sensor results are retained.
"""
import argparse,concurrent.futures,copy,csv,hashlib,heapq,json,os,shutil,signal,struct,subprocess,time,fcntl
from pathlib import Path
import cv2,numpy as np,yaml,rosbag2_py
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import fast_stream_transport as wire
ROOT=Path(__file__).resolve().parent;LOG=ROOT/'logs/zab-offline';OUT=LOG/'fast_multicamera_all';WS=Path('/root/multilink_autonomy_ws/comparison_ws')
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
 child=None
 with (d/'run.log').open('w') as log,(d/'truth_full.csv').open('w') as tf,concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
  pending={i:pool.submit(fetch,n) for i,n in enumerate(names[:8])}
  writer=csv.writer(tf);writer.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz'])
  child=subprocess.Popen(['bash',str(ROOT/'comparison_ros1.sh'),'python3',str(ROOT/'run_fast_stream_child.py'),str(d/'config.yaml'),str(d/'odometry_full.csv')],env=ENV,stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,bufsize=2**20,start_new_session=True)
  def drain(limit):
   nonlocal emitted
   while heap and heap[0][0]<=limit:
    t,_,idx,payload=heapq.heappop(heap);assert t>=emitted,'Source timestamp disorder exceeds reorder buffer';emitted=t
    assert last[idx] is None or t>last[idx],f'Duplicate/reversed source stamp {idx} {t}'
    child.stdin.write(wire.packet(idx,payload));counts[idx]+=1;hashes[idx].update(payload);last[idx]=t
    if first[idx] is None:first[idx]=t
    if idx>=2:image_stamps[idx-2].append(t)
  try:
   for i,n in enumerate(names):
    f=pending.pop(i).result()
    if i+8<len(names):pending[i+8]=pool.submit(fetch,names[i+8])
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(f),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS));types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in TOPICS}
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
   drain(2**63-1);child.stdin.close()
   write(d/'input_audit.json',dict(counts=counts,first_stamp_ns=first,last_stamp_ns=last,serialized_sha256=[h.hexdigest() for h in hashes],chunks=chunks,truth_count=truth_count))
   rc=child.wait(timeout=600)
   if rc:raise RuntimeError(f'FAST exit {rc}')
  finally:
   if child.poll() is None:os.killpg(child.pid,signal.SIGINT);child.wait(timeout=20)
 summary=dict(returncode=rc,wall_s=time.monotonic()-start,cameras=list(wire.CAMERAS),window=window,source=entry,adapter='ROS1 serialized stream, globally reordered with 1 s buffer; no estimator restart at chunk boundaries',counts=counts,first_stamp_ns=first,last_stamp_ns=last,serialized_sha256=[h.hexdigest() for h in hashes],chunks=chunks,truth_count=truth_count,binary_sha256=sha(WS/'devel/lib/fast_livo/fastlivo_stream'),exposure_estimate=False,reused=False)
 common=set(image_stamps[0])&set(image_stamps[1])&set(image_stamps[2]);summary['incomplete_image_stamps_ns']=sorted(set().union(*map(set,image_stamps))-common)
 assert not [t for t in summary['incomplete_image_stamps_ns'] if t<=window['end_ns']], 'Incomplete images inside tracking window'
 write(d/'summary.json',summary);finalize(d)
 print('COMPLETE',label,flush=True)

def finalize(d):
 from plot_fast_camera_comparison import read,errors
 s=json.loads((d/'summary.json').read_text());w=s['window'];start=w['start_ns'];end=w['end_ns']
 # Retain exact source stamps in tracking-only deliverables; full CSVs are audit records.
 for name in ['odometry','truth']:
  source=d/f'{name}_full.csv'
  with source.open() as f,(d/f'{name}.csv').open('w') as out:
   rows=csv.DictReader(f);wr=csv.DictWriter(out,fieldnames=rows.fieldnames);wr.writeheader();wr.writerows(r for r in rows if start<=int(r['stamp_ns'])<=end)
 a=read(d/'odometry_full.csv');truth=read(d/'truth_full.csv');native=a[0][(a[0]>=start)&(a[0]<=end)];expected=np.arange((start+49999999)//50000000*50000000,end+1,50000000,dtype=np.int64)
 missing=sorted(set(map(int,expected))-set(map(int,native)))
 assert np.array_equal(native,expected[:len(native)]),f'{d.name}: interior tracking gap or extra estimate'
 assert not missing or (len(missing)==1 and d.name=='run1_middle'),f'{d.name}: unexpected missing tracking estimates'
 evaluation_end=min(end,int(native[-1]))
 assert np.isfinite(a[1]).all() and np.isfinite(a[2].as_quat()).all()
 raw=np.genfromtxt(d/'odometry_full.csv',delimiter=',',names=True)
 quats=np.stack([raw[k] for k in ['qx','qy','qz','qw']],axis=-1)
 assert np.isfinite(quats).all() and np.allclose(np.linalg.norm(quats,axis=1),1.,rtol=0,atol=1e-6), 'Invalid output quaternion'
 grid=np.arange((start+99999999)//100000000*100000000,evaluation_end+1,100000000,dtype=np.int64);e,m=errors(a,truth,start,grid)
 with (d/'errors.csv').open('w') as f:
  wr=csv.writer(f);wr.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);wr.writerows(zip(grid,*e.T))
 import re
 text=(d/'run.log').read_text();cams={n:[int(a),int(b)] for n,a,b in re.findall(r'CAMERA_COUNTS (\w+) (\d+) (\d+)',text)};assert set(cams)==set(wire.CAMERAS) and all(x[1]>0 for x in cams.values())
 sync={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'CAMERA_SYNC (.+)',text)[1])};assert sync['duplicates']==sync['late']==sync['ready_dropped']==0
 inp=list(map(int,re.search(r'INPUT_COUNTS ([0-9 ]+)',text)[1].split()))
 if 'counts' in s:assert inp==s['counts'], 'Stream delivery counts differ'
 rows=re.findall(r'CAMERA_UPDATE ([0-9.e+-]+) (\w+) (\d+) ([0-9.e+-]+) ([0-9.e+-]+) ([0-9.e+-]+)',text)
 assert len(rows)%3==0
 for k in range(0,len(rows),3):
  g=rows[k:k+3];assert [r[1] for r in g]==list(wire.CAMERAS) and len({r[0] for r in g})==1
  assert float(g[1][4])==float(g[0][5]) and float(g[2][4])==float(g[1][5])
 m.update(native_samples=len(native),tracking_stamps_complete=not missing,missing_tracking_stamps_ns=missing,evaluation_end_ns=evaluation_end,tail_note="Recorded input ends before the next LiDAR frame needed to release the final visual update; no synthetic measurements or extrapolation" if missing else None,input_counts=inp,covariance_chain_exact=True,camera_counts=cams,synchronizer=sync,odometry_sha256=sha(d/'odometry.csv'),truth_sha256=sha(d/'truth.csv'));write(d/'metrics.json',m)

def reuse(label):
 d=OUT/label
 if (d/'metrics.json').exists():return
 old=LOG/'fast_multicamera/link1_link3_link4'/label/'trial1';d.mkdir(exist_ok=True)
 for n in ['config.yaml','run.log']:shutil.copyfile(old/n,d/n)
 shutil.copyfile(old/'odometry.csv',d/'odometry_full.csv');shutil.copyfile(LOG/'livo_comparison/inputs'/label/'truth.csv',d/'truth_full.csv')
 s=json.loads((old/'summary.json').read_text());s.update(reused=True,source_result=str(old),selection='trial1 selected before new campaign; no best-trial selection');write(d/'summary.json',s);finalize(d)

def main():
 p=argparse.ArgumentParser();p.add_argument('--labels',nargs='*');p.add_argument('--master-port',type=int,default=11376);a=p.parse_args();ENV['FAST_STREAM_MASTER_URI']=f'http://127.0.0.1:{a.master_port}';OUT.mkdir(exist_ok=True)
 entries=json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags']
 for l in ['run2a_end','run1_tail_a2_prefix']:reuse(l)
 for e in entries:
  if a.labels and e['label'] not in a.labels:continue
  process(e)
 write(OUT/'manifest.json',{e['label']:dict(complete=(OUT/e['label']/'metrics.json').exists(),directory=str(OUT/e['label'])) for e in entries})
if __name__=='__main__':main()
