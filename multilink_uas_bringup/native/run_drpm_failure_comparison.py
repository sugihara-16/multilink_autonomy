#!/usr/bin/env python3
"""Adopted DRPM+single link3 ROVIO on the three official FAST-LIVO2 failures."""
import argparse,csv,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
BASE=ROOT/'logs/zab-offline'
OUT=BASE/'drpm_fast_failures'
WS=Path('/root/multilink_autonomy_ws/comparison_ws')
LABELS=['run1_prefix','run1_tail_a1','run1_tail_b1']
ENV=dict(HOME='/root',USER='root',PATH='/usr/local/bin:/usr/bin:/bin',LANG='C.UTF-8',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='2')
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(2**20),b''):h.update(b)
 return h.hexdigest()
def save(p,obj):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(obj,indent=2))
def fixed_calibration():
 import numpy as np
 from calibrate_rovio_relative import calibration
 labels=['run2a_end','run1_tail_a2_prefix'];arrays=[];sources=[]
 for label in labels:
  f=BASE/'livo_calibrated/calibration'/label/'relative_errors.csv'
  a=np.loadtxt(f,delimiter=',',skiprows=1);arrays.append(a[:,1:]);sources.append(dict(label=label,samples=len(a),sha256=sha(f)))
 C,details=calibration(np.concatenate(arrays))
 result=dict(protocol='Pool previously measured 50 ms relative-pose errors from the two earlier flights; fixed uncentered second moment; no target-flight GT or tuning',training=sources,covariance=C.tolist(),**details)
 p=OUT/'calibration.json'
 if p.exists():assert json.loads(p.read_text())==result
 else:save(p,result)
 return C

def audit_inputs(label):
 import numpy as np
 inp=OUT/'inputs'/label;m=json.loads((inp/'manifest.json').read_text());stamps={k:[] for k in ['imu_t','cloud_t','cam1_t','truth_t']};files={}
 for name in m['chunks']:
  f=inp/name;files[name]=sha(f)
  with np.load(f) as a:
   for k in stamps:stamps[k].extend(a[k].astype('int64').tolist())
   assert a['cam1_im'].shape[1:]==(360,480)
 audit={}
 for k,period in [('imu_t',1000000),('cloud_t',100000000),('cam1_t',50000000)]:
  t=np.array(stamps[k],dtype=np.int64);t=t[t>=2003000000]
  assert len(t)>0 and (np.diff(t)>0).all(),k
  assert (np.diff(t)==period).all(),(k,np.unique(np.diff(t)))
  audit[k]=dict(count=len(t),first_ns=int(t[0]),last_ns=int(t[-1]),period_ns=period)
 save(inp/'input_audit.json',dict(streams=audit,prepared_chunk_sha256=files,source_chunks_md5_verified_during_prepare=True,preprocessing='Previously adopted ROVIO mono8 + calibrated rectification to 480x360; link3 only. Same physical source camera as official FAST, different estimator image preprocessing.'))

def stage_impl(phase,label):
 if phase=='prepare':
  import prepare_livo_comparison as prep,fcntl
  OUT.mkdir(parents=True,exist_ok=True)
  with (OUT/(label+'.prepare.lock')).open('w') as lock:
   fcntl.flock(lock,fcntl.LOCK_EX)
   prep.OUT=OUT;prep.main(label);audit_inputs(label)
 elif phase=='rovio':
  import replay_monocular_comparison as replay
  replay.OUT=OUT
  binary=OUT/'audit25'
  if not binary.exists():binary.symlink_to(BASE/'livo_comparison/audit25')
  replay.run(label)
  import numpy as np
  f=OUT/'rovio'/label/'result_vio.csv';r=list(csv.DictReader(f.open()));t=np.array([int(x['stamp_ns']) for x in r],np.int64)
  assert len(t)>10 and (np.diff(t)==50000000).all()
  s=json.loads((OUT/'rovio'/label/'summary.json').read_text());s.update(binary_sha256=sha(binary),result_sha256=sha(f),max_output_gap_s=.05);save(OUT/'rovio'/label/'summary.json',s)
 elif phase=='bag':
  import make_comparison_bag as b
  b.OUT=OUT;b.main(label,include_images=False)
 elif phase=='fusion':
  import rospy,yaml,numpy as np
  rospy.init_node('drpm_failure_driver',anonymous=True,disable_signals=True)
  dest=OUT/'fusion'/label;dest.mkdir(parents=True,exist_ok=True)
  if (dest/'summary.json').exists():
   assert json.loads((dest/'summary.json').read_text())['returncode']==0;return
  cfg=yaml.safe_load((BASE/'drpm_official/official_update/run2a_end/config.yaml').read_text())
  cfg['logs_directory']=str(dest)+'/'
  cfg['odometry']['manager']['relative_pose_covariance']=fixed_calibration().ravel().tolist()
  adopted=json.loads((ROOT/'comparison/adopted_drpm.json').read_text())['scan_to_map']
  assert all(cfg['lidar']['geometric']['scan_to_map'][k]==v for k,v in adopted.items())
  assert cfg['odometry']['manager']['enabled']
  (dest/'config.yaml').write_text(yaml.safe_dump(cfg));rospy.set_param('/mimosa',cfg)
  rospy.set_param('/mimosa/lidar/sensor',json.loads((WS/'src/mimosa/config/os_dummy.json').read_text()))
  bag=OUT/'inputs'/f'{label}.bag';rospy.set_param('/mimosa/bag_name',str(bag))
  cmd=[str(WS/'devel/lib/mimosa/mimosa_rosbag'),'__name:=mimosa','~imu/manager/imu_in:=/comparison/imu','~lidar/manager/lidar_in:=/comparison/points','~odometry/manager/odometry_in:=/comparison/vio']
  artifacts={str(p):sha(p) for p in [Path(cmd[0]),WS/'devel/lib/libmimosa_lib.so',WS/'src/mimosa/include/mimosa/lidar/official_drpm/degeneracy.h']}
  start=time.monotonic()
  with (dest/'run.log').open('w') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=3600)
  summary=dict(label=label,returncode=p.returncode,wall_s=time.monotonic()-start,command=cmd,artifacts=artifacts,input_sha256=sha(bag),window=json.loads((OUT/'inputs'/label/'manifest.json').read_text())['window'],camera='link3',calibration_sha256=sha(OUT/'calibration.json'))
  save(dest/'summary.json',summary);assert p.returncode==0,summary
  assert artifacts=={p:sha(p) for p in artifacts}
 elif phase=='evaluate':
  from evaluate_drpm_failure_comparison import evaluate
  evaluate(label)

def stage(phase,label):
 import fcntl
 OUT.mkdir(parents=True,exist_ok=True)
 with (OUT/(label+'.stage_'+phase+'.lock')).open('w') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX)
  marker=OUT/'stage_checks'/(label+'_'+phase+'.json')
  if marker.exists():return
  if phase=='fusion':
   with (OUT/'fusion_parameters.lock').open('w') as fusion_lock:
    fcntl.flock(fusion_lock,fcntl.LOCK_EX);stage_impl(phase,label)
  else:stage_impl(phase,label)
  save(marker,dict(complete=True,label=label,phase=phase))

def flight(label):
 for phase in ['prepare','rovio','bag','fusion','evaluate']:
  print('START',label,phase,flush=True)
  if phase=='prepare':cmd=['bash','--noprofile','--norc','-c','source /opt/ros/humble/setup.bash\nexec python3 "$@"','prepare',str(Path(__file__).resolve()),phase,label]
  else:cmd=['bash',str(ROOT/'comparison_ros1.sh'),'python3',str(Path(__file__).resolve()),phase,label]
  subprocess.run(cmd,env=ENV,check=True)
  print('DONE',label,phase,flush=True)

def campaign():
 OUT.mkdir(parents=True,exist_ok=True);fixed_calibration()
 save(OUT/'protocol.json',dict(labels=LABELS,camera='link3',method='Adopted MIMOSA + official DRPM update + radial noise + monocular ROVIO',scope='Full tracking windows; compare against saved official FAST on shared prefix only',target_truth_used_for_tuning=False,preprocessing='Retain adopted ROVIO rectified mono8 preprocessing; official FAST uses original RGB and upstream processing',reference_fast=str(BASE/'fast_official_raw')))
 for label in LABELS:
  if (OUT/'fusion'/label/'metrics.json').exists():continue
  for phase in ['prepare','rovio','bag','fusion','evaluate']:
   save(OUT/'runtime.json',dict(status='running',label=label,phase=phase))
   print('START',label,phase,flush=True)
   if phase=='prepare':cmd=['bash','--noprofile','--norc','-c','source /opt/ros/humble/setup.bash\nexec python3 "$@"','prepare',str(Path(__file__).resolve()),phase,label]
   else:cmd=['bash',str(ROOT/'comparison_ros1.sh'),'python3',str(Path(__file__).resolve()),phase,label]
   subprocess.run(cmd,env=ENV,check=True)
   print('DONE',label,phase,flush=True)
  inp=OUT/'inputs'/label
  for p in inp.glob('*.npz'):p.unlink()
  (OUT/'inputs'/f'{label}.bag').unlink()
 save(OUT/'runtime.json',dict(status='measurements_complete',labels=LABELS))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('phase',choices=['campaign','flight','prepare','rovio','bag','fusion','evaluate']);p.add_argument('label',nargs='?');a=p.parse_args()
 if a.phase=='campaign':campaign()
 elif a.phase=='flight':flight(a.label)
 else:stage(a.phase,a.label)
