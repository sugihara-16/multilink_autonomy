#!/usr/bin/env python3
"""Matched conventional MIMOSA vs adopted DRPM, reusing identical three-camera VIO."""
import argparse,csv,json,shutil,subprocess,time
from pathlib import Path
import numpy as np
import run_drpm_failure_comparison as base
ROOT=base.ROOT;REF=base.BASE/'drpm_three_camera';OUT=base.BASE/'three_camera_no_drpm';LABELS=base.LABELS

def prepare(label):
 import prepare_livo_comparison as p
 p.OUT=OUT;p.TOPICS=['/dragon/lidar/points','/dragon/imu/raw','/dragon/ground_truth','/dragon/joint_states'];p.np.savez_compressed=p.np.savez
 p.main(label)
 d=OUT/'inputs'/label;m=json.loads((d/'manifest.json').read_text());m['image_conversion']='Images not processed: reuse exact saved three-camera ROVIO output';base.save(d/'manifest.json',m)
 audit=json.loads((REF/'inputs'/label/'input_audit.json').read_text());checks={}
 for key in ['imu_t','cloud_t']:
  arrays=[]
  for name in m['chunks']:
   with np.load(d/name) as a:arrays.append(a[key])
  ts=np.concatenate(arrays);ts=ts[ts>=2003000000];period=1000000 if key=='imu_t' else 100000000
  assert (np.diff(ts)==period).all()
  checks[key]=dict(count=len(ts),first_ns=int(ts[0]),last_ns=int(ts[-1]),period_ns=period)
  assert checks[key]==audit['streams'][key]
 base.save(d/'input_audit.json',dict(streams=checks,source_md5_verified=True))
 dest=OUT/'rovio'/label;dest.mkdir(parents=True,exist_ok=True)
 shutil.copyfile(REF/'rovio'/label/'result_vio.csv',dest/'result_vio.csv')
 assert base.sha(dest/'result_vio.csv')==base.sha(REF/'rovio'/label/'result_vio.csv')

def bag(label):
 import make_comparison_bag as b
 b.OUT=OUT;b.main(label,include_images=False)
 d=OUT/'inputs'/label;old=json.loads((REF/'fusion'/label/'summary.json').read_text());h=base.sha(OUT/'inputs'/f'{label}.bag')
 base.save(d/'bag_match.json',dict(sha256=h,reference_sha256=old['input_sha256'],byte_identical=h==old['input_sha256']))
 assert h==old['input_sha256'],'Reconstructed bag differs; investigate before estimation'
 assert (d/'truth.csv').read_bytes()==(REF/'inputs'/label/'truth.csv').read_bytes()

def fusion(label):
 import rospy,yaml
 rospy.init_node('three_camera_no_drpm_driver',anonymous=True,disable_signals=True)
 d=OUT/'fusion'/label;d.mkdir(parents=True,exist_ok=True)
 old=json.loads((REF/'fusion'/label/'summary.json').read_text())
 for p,h in old['artifacts'].items():assert base.sha(p)==h
 cfg=yaml.safe_load((REF/'fusion'/label/'config.yaml').read_text());assert cfg['lidar']['geometric']['scan_to_map']['drpm_enabled']
 cfg['lidar']['geometric']['scan_to_map']['drpm_enabled']=False;cfg['logs_directory']=str(d)+'/'
 assert cfg['odometry']['manager']['enabled'] and not cfg['lidar']['geometric']['scan_to_map']['project_on_degneneracy']
 (d/'config.yaml').write_text(yaml.safe_dump(cfg));rospy.set_param('/mimosa',cfg)
 rospy.set_param('/mimosa/lidar/sensor',json.loads((base.WS/'src/mimosa/config/os_dummy.json').read_text()))
 rospy.set_param('/mimosa/bag_name',str(OUT/'inputs'/f'{label}.bag'))
 start=time.monotonic()
 with (d/'run.log').open('w') as log:p=subprocess.run(old['command'],stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=3600)
 base.save(d/'summary.json',dict(returncode=p.returncode,wall_s=time.monotonic()-start,window=old['window'],artifacts=old['artifacts'],input_sha256=old['input_sha256'],cameras=['link1','link3','link4'],drpm_enabled=False))
 assert p.returncode==0

def evaluate(label):
 import yaml
 from plot_fast_camera_comparison import read,errors
 from evaluate_drpm_failure_comparison import export,onset
 d=OUT/'fusion'/label;ref=REF/'fusion'/label;m=json.loads((ref/'metrics.json').read_text());window=m['window'];start=m['alignment_stamp_ns'];end=m['full_grid_end_ns'];grid=np.arange(start,end+1,100000000,dtype=np.int64)
 v=read(d/'odometry.csv');truth=read(OUT/'inputs'/label/'truth.csv');assert np.isfinite(v[1]).all() and np.isfinite(v[2].as_quat()).all()
 expected=np.arange((window['start_ns']+49999999)//50000000*50000000,window['end_ns']+1,50000000,dtype=np.int64)
 native=v[0][(v[0]>=window['start_ns'])&(v[0]<=window['end_ns'])];assert np.array_equal(expected,native)
 assert np.isin(grid,native).all();e,metrics=errors(v,truth,start,grid);export(d/'off_full_errors.csv',grid,e)
 metrics['first_sustained_100m_s']=onset(e,grid,start)
 log=(d/'run.log').read_text();assert 'Using calibrated full relative-pose covariance' in log
 assert not (d/'drpm_registration.csv').exists() and 'DRPM probabilities' not in log
 off=yaml.safe_load((d/'config.yaml').read_text());on=yaml.safe_load((ref/'config.yaml').read_text());off['logs_directory']=on['logs_directory'];off['lidar']['geometric']['scan_to_map']['drpm_enabled']=True;assert off==on
 with (d/'odometry.csv').open() as f, (d/'odometry_tracking.csv').open('w') as out:
  reader=csv.DictReader(f);writer=csv.DictWriter(out,fieldnames=reader.fieldnames);writer.writeheader();rows={int(r['stamp_ns']):r for r in reader if window['start_ns']<=int(r['stamp_ns'])<=window['end_ns']};writer.writerows(rows[t] for t in sorted(rows))
 shutil.copyfile(ref/'truth_tracking.csv',d/'truth_tracking.csv');shutil.copyfile(ref/'rovio_tracking.csv',d/'rovio_tracking.csv')
 result=dict(label=label,window=window,alignment_stamp_ns=start,full_grid_end_ns=end,off=metrics,on=m['full']['drpm'],rovio=m['full']['rovio'],checks=dict(same_bag_bytes=True,same_rovio=True,same_binary=True,only_drpm_switch_and_log_path_changed=True,all_20hz_stamps_present=True,drpm_branch_not_executed=True,vio_uncertainty_rejections=log.count('is higher than threshold')))
 base.save(d/'metrics.json',result);print('RESULT',label,json.dumps(result),flush=True)

def stage(phase,label):
 import fcntl
 OUT.mkdir(parents=True,exist_ok=True)
 with (OUT/f'{label}_{phase}.lock').open('w') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX);marker=OUT/'stage_checks'/f'{label}_{phase}.json'
  if marker.exists():return
  if phase=='fusion':
   with (OUT/'fusion_parameters.lock').open('w') as shared:
    fcntl.flock(shared,fcntl.LOCK_EX);globals()[phase](label)
  else:globals()[phase](label)
  base.save(marker,dict(complete=True))

def command(phase,label):
 script=str(Path(__file__).resolve())
 if phase=='prepare':return ['bash','--noprofile','--norc','-c','source /opt/ros/humble/setup.bash\nexec python3 "$@"','prepare',script,phase,label]
 return ['bash',str(ROOT/'comparison_ros1.sh'),'python3',script,phase,label]

def campaign():
 OUT.mkdir(parents=True,exist_ok=True);base.save(OUT/'protocol.json',dict(reference=str(REF),labels=LABELS,only_setting_change='lidar.geometric.scan_to_map.drpm_enabled: true -> false',native_mimosa_projection=False,rovio='Reuse previous three-camera CSV exactly',note='Compare adopted DRPM ICP pose-factor integration against ordinary MIMOSA point-to-plane factors; not only probability suppression within an identical solver',target_truth_tuning=False))
 for label in ['run1_tail_a1','run1_prefix','run1_tail_b1']:
  for phase in ['prepare','bag','fusion','evaluate']:
   base.save(OUT/'runtime.json',dict(status='running',label=label,phase=phase));print('START',label,phase,flush=True);subprocess.run(command(phase,label),env=base.ENV,check=True);print('DONE',label,phase,flush=True)
  for p in (OUT/'inputs'/label).glob('*.npz'):p.unlink()
  (OUT/'inputs'/f'{label}.bag').unlink(missing_ok=True)
 base.save(OUT/'runtime.json',dict(status='measurements_complete'))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('phase');p.add_argument('label',nargs='?');a=p.parse_args()
 if a.phase=='campaign':campaign()
 else:stage(a.phase,a.label)
