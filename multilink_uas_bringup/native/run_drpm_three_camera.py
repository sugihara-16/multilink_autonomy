#!/usr/bin/env python3
"""Matched three-camera ROVIO + adopted DRPM on the three previous failures."""
import argparse,csv,json,re,shutil,struct,subprocess,time
from pathlib import Path
import numpy as np
import run_drpm_failure_comparison as base
ROOT=base.ROOT;BASE=base.BASE;OLD=BASE/'drpm_fast_failures';OUT=BASE/'drpm_three_camera';LABELS=base.LABELS
base.OUT=OUT

def build():
 OUT.mkdir(parents=True,exist_ok=True)
 source=(ROOT/'rovio_audit.cpp').read_text().splitlines();mono=(ROOT/'rovio_monocular.cpp').read_text().splitlines()
 for prefix in [' od<<"stamp_ns','  od<<t<<']:
  line=next(x for x in mono if x.startswith(prefix));source=[line if x.startswith(prefix) else x for x in source]
 cpp=ROOT/'rovio_three_camera.cpp';cpp.write_text('\n'.join(source)+'\n')
 commands=json.loads((BASE/'vio_audit/build25.json').read_text());assert '-DROVIO_NCAM=3' in commands[0] and '-DROVIO_NMAXFEATURE=25' in commands[0]
 commands=[[x.replace('/root/uas_native/logs/zab-offline/vio_audit/audit25',str(OUT/'audit25')).replace(str(BASE/'vio_audit/audit25'),str(OUT/'audit25')).replace('/root/uas_native/rovio_audit.cpp',str(cpp)).replace(str(ROOT/'rovio_audit.cpp'),str(cpp)) for x in cmd] for cmd in commands]
 assert commands[0][commands[0].index('-c')+1]==str(cpp)
 assert commands[1][commands[1].index('-o')+1]==str(OUT/'audit25')
 base.save(OUT/'build.json',commands)
 with (OUT/'build.log').open('w') as log:
  for cmd in commands:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
 base.save(OUT/'build_validation.json',dict(binary_sha256=base.sha(OUT/'audit25'),source_sha256=base.sha(cpp),camera_count=3,max_features_total=25,change_to_prior_three_camera_filter='Only add pose covariance to CSV export; filter code unchanged'))

def prepare(label):
 import prepare_livo_comparison as p
 p.OUT=OUT;p.TOPICS=[*p.CAM,'/dragon/lidar/points','/dragon/imu/raw','/dragon/ground_truth','/dragon/joint_states']
 # Fast temporary storage; numerical arrays and original source stamps unchanged.
 p.np.savez_compressed=p.np.savez
 p.main(label)
 inp=OUT/'inputs'/label;m=json.loads((inp/'manifest.json').read_text());streams={k:[] for k in ['imu_t','cloud_t','cam0_t','cam1_t','cam2_t']}
 for name in m['chunks']:
  with np.load(inp/name) as a:
   for k in streams:streams[k].extend(a[k].astype('int64').tolist())
   for c in range(3):assert a[f'cam{c}_im'].shape[1:]==(360,480)
 checks={}
 for k,ts in streams.items():
  t=np.array(ts,np.int64);t=t[t>=2003000000];period=1000000 if k=='imu_t' else 100000000 if k=='cloud_t' else 50000000
  assert (np.diff(t)==period).all(),k
  checks[k]=dict(count=len(t),first_ns=int(t[0]),last_ns=int(t[-1]),period_ns=period)
 assert streams['cam0_t']==streams['cam1_t']==streams['cam2_t']
 old=json.loads((OLD/'inputs'/label/'input_audit.json').read_text())['streams']
 for k in old:assert checks[k]==old[k]
 base.save(inp/'input_audit.json',dict(streams=checks,three_camera_stamps_identical=True,previous_single_camera_streams_match=True,source='MD5-verified persistent local originals; same preprocessing function',temporary_compression='none'))

def rovio(label):
 inp=OUT/'inputs'/label;m=json.loads((inp/'manifest.json').read_text());src=ROOT/'logs'/m['source']['run'];dest=OUT/'rovio'/label;dest.mkdir(parents=True,exist_ok=True)
 text=(src/'rovio.info').read_text()
 for pattern,value,n_expected in [(r'(gyb_[012]\s+)3e-4;','1e-10',3),(r'(gyb_[012]\s+)3.8e-7;','1e-16',3),(r'(att_[012]\s+)7.6e-7;','4e-11',3),(r'(initDepth\s+)0.5;','0.1',1)]:
  text,n=re.subn(pattern,rf'\g<1>{value};',text);assert n==n_expected
 # Verify scalar filter settings and selected link3 calibration against the previous monocular run.
 previous=(OLD/'rovio'/label/'rovio.info').read_text();strip=lambda t:re.sub(r'\s+','',re.sub(r'Camera[012]\s*\{[^}]+\}','',t))
 assert strip(text)==strip(previous)
 cam=re.search(r'Camera1\s*\{[^}]+\}',text).group().replace('Camera1','Camera0');assert re.sub(r'\s+','',cam)==re.sub(r'\s+','',re.search(r'Camera0\s*\{[^}]+\}',previous).group())
 (dest/'rovio.info').write_text(text)
 for i in range(3):shutil.copyfile(src/f'cam{i}.yaml',dest/f'cam{i}.yaml')
 assert (dest/'cam1.yaml').read_bytes()==(OLD/'rovio'/label/'cam0.yaml').read_bytes()
 command=['bash',str(ROOT/'comparison_ros1.sh'),str(OUT/'audit25'),str(dest/'rovio.info'),str(dest),str(dest/'result')]
 start=time.monotonic();counts=[0]*4
 with (dest/'replay.log').open('w') as log:
  proc=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,env=base.ENV)
  try:
   for name in m['chunks']:
    with np.load(inp/name) as a:
     arrays={k:a[k] for k in ['order','imu','cam0_im','cam1_im','cam2_im']}
    for stamp,kind,index,stored in arrays['order']:
     if stamp<2003000000 or kind>3:continue
     payload=struct.pack('<6d',*arrays['imu'][index]) if kind==0 else arrays[f'cam{kind-1}_im'][index].tobytes()
     proc.stdin.write(struct.pack('<IQ',int(kind),int(stamp))+payload);counts[kind]+=1
    print(label,'ROVIO',name,counts,flush=True)
   proc.stdin.close();rc=proc.wait(timeout=180);assert rc==0
  finally:
   if proc.poll() is None:proc.terminate();proc.wait(timeout=10)
 rows=list(csv.DictReader((dest/'result_vio.csv').open()));t=np.array([int(r['stamp_ns']) for r in rows],np.int64);assert (np.diff(t)==50000000).all()
 audit=json.loads((inp/'input_audit.json').read_text())['streams'];assert counts==[audit[k]['count'] for k in ['imu_t','cam0_t','cam1_t','cam2_t']]
 log=(dest/'replay.log').read_text();match=re.search(r'AUDIT_INPUT (\d+) (\d+) (\d+) (\d+) sets (\d+) incomplete (\d+)',log);assert match and [int(x) for x in match.groups()[:4]]==counts and int(match[5])==counts[1] and int(match[6])==0
 state=list(csv.DictReader((dest/'result_state.csv').open()));features={f'link{link}':sum(int(r[f'n{i}']) for r in state) for i,link in enumerate([1,3,4])}
 base.save(dest/'summary.json',dict(cameras=['link1','link3','link4'],counts=counts,wall_s=time.monotonic()-start,window=m['window'],returncode=0,max_features_total=25,feature_observations=features,all_camera_groups_complete=True,binary_sha256=base.sha(OUT/'audit25'),pose_sha256=base.sha(dest/'result_vio.csv'),only_camera_blocks_differ_from_mono=True))

def stage_impl(phase,label):
 OUT.mkdir(parents=True,exist_ok=True)
 if phase=='prepare':prepare(label)
 elif phase=='rovio':rovio(label)
 elif phase=='bag':base.stage_impl('bag',label)
 elif phase=='fusion':
  base.stage_impl('fusion',label);p=OUT/'fusion'/label/'summary.json';j=json.loads(p.read_text());j.pop('camera',None);j['cameras']=['link1','link3','link4'];base.save(p,j)
 elif phase=='evaluate':
  import evaluate_drpm_failure_comparison as e
  e.OUT=OUT;e.evaluate(label)
 base.save(OUT/'stage_checks'/f'{label}_{phase}.json',dict(complete=True))

def stage(phase,label):
 import fcntl
 OUT.mkdir(parents=True,exist_ok=True)
 with (OUT/f'{label}_{phase}.lock').open('w') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX)
  if not (OUT/'stage_checks'/f'{label}_{phase}.json').exists():stage_impl(phase,label)

def command(phase,label):
 if phase=='prepare':return ['bash','--noprofile','--norc','-c','source /opt/ros/humble/setup.bash\nexec python3 "$@"','prepare',str(Path(__file__).resolve()),phase,label]
 return ['bash',str(ROOT/'comparison_ros1.sh'),'python3',str(Path(__file__).resolve()),phase,label]

def campaign():
 OUT.mkdir(parents=True,exist_ok=True);base.fixed_calibration();assert (OUT/'calibration.json').read_bytes()==(OLD/'calibration.json').read_bytes()
 base.save(OUT/'protocol.json',dict(labels=LABELS,cameras=['link1','link3','link4'],max_features_total=25,fusion_covariance='Frozen previous monocular calibration',scope='Same complete tracking intervals and FAST common intervals as previous run',source='Persistent local bags',no_target_truth_tuning=True))
 for label in ['run1_tail_a1','run1_prefix','run1_tail_b1']:
  for phase in ['prepare','rovio','bag','fusion','evaluate']:
   marker=OUT/'stage_checks'/f'{label}_{phase}.json'
   if marker.exists():continue
   base.save(OUT/'runtime.json',dict(status='running',label=label,phase=phase));print('START',label,phase,flush=True)
   subprocess.run(command(phase,label),env=base.ENV,check=True);print('DONE',label,phase,flush=True)
  for p in (OUT/'inputs'/label).glob('*.npz'):p.unlink()
  (OUT/'inputs'/f'{label}.bag').unlink(missing_ok=True)
 base.save(OUT/'runtime.json',dict(status='measurements_complete'))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('phase');p.add_argument('label',nargs='?');a=p.parse_args()
 if a.phase=='build':build()
 elif a.phase=='campaign':campaign()
 else:stage(a.phase,a.label)
