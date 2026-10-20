#!/usr/bin/env python3
"""Geometry, exact input stamps, static bias and gyro/time consistency audits."""
import hashlib,json,re,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation,Slerp
from scipy.integrate import cumulative_trapezoid
from dragon_cameras import link_transforms,transform,OPTICAL
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/vio_audit'
def calibration():
 results={}
 for run in json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags']:
  p=ROOT/'logs'/run['run'];urdf=ET.parse(p/'dragon_slam.urdf').getroot();angles={}
  for j in urdf.findall('./ros2_control/joint'):
   v=j.find("state_interface[@name='position']/param[@name='initial_value']")
   if v is not None:angles[j.get('name')]=float(v.text)
  frames=link_transforms(urdf,angles);cfg=(p/'rovio.info').read_text();checks=[]
  for i,link in enumerate(['link1','link3','link4']):
   sensor=urdf.find(f"gazebo[@reference='{link}']/sensor[@type='camera']");pose=list(map(float,sensor.findtext('pose').split()));T=np.linalg.inv(frames['fc'])@frames[link]@transform(pose[:3],pose[3:]);expected=T[:3,:3]@OPTICAL
   block=re.search(r'Camera'+str(i)+r'\s*\{([^}]+)',cfg)[1];v={k:float(x) for k,x in re.findall(r'(qCM_\w|MrMC_\w)\s+([^;\s]+)',block)};rotation=Rotation.from_quat([v['qCM_'+k] for k in 'xyzw']).inv();translation=np.array([v['MrMC_'+k] for k in 'xyz'])
   raw=yaml.safe_load((p/f'raw_cam{i}.yaml').read_text());rect=yaml.safe_load((p/f'cam{i}.yaml').read_text());hfov=float(sensor.findtext('camera/horizontal_fov'));fx=raw['image_width']/(2*np.tan(hfov/2))
   checks.append(dict(cam=i,translation_error_m=float(np.linalg.norm(T[:3,3]-translation)),rotation_error_deg=float((rotation.inv()*Rotation.from_matrix(expected)).magnitude()*180/np.pi),raw_focal_error_px=abs(raw['camera_matrix']['data'][0]-fx),rect_focal_error_px=abs(rect['camera_matrix']['data'][0]-fx*480/720),fc_position=translation.tolist(),optical_forward_fc=rotation.apply([0,0,1]).tolist()))
  results[run['label']]=dict(cameras=checks,config_sha256=hashlib.sha256(cfg.encode()).hexdigest(),joint_angles=angles)
 return results
def sensors(folder):
 meta=json.loads((folder/'manifest.json').read_text());keys=['imu_t','imu','truth_t','truth','joint_t','joints','order','cam0_t','cam1_t','cam2_t'];data={k:[] for k in keys}
 for f in meta['chunks']:
  with np.load(folder/f) as a:
   for k in keys:data[k].append(a[k])
 a={k:np.concatenate(v) for k,v in data.items()};it=a['imu_t']*1e-9;tt=a['truth_t']*1e-9;gt=a['truth'];q=Rotation.from_quat(gt[:,3:7]);sel=(it>=2)&(it<5);qi=Slerp(tt,q)(it[sel]);bias=a['imu'][sel,:3]-qi.inv().apply([0,0,9.81]);joints={name:float(np.ptp(a['joints'][:,j])) for j,name in enumerate(json.loads((folder/'joint_names.json').read_text())) if name.startswith('joint')}
 cadence={}
 for k in ['imu_t','cam0_t','cam1_t','cam2_t']:
  t=a[k];d=np.diff(t);cadence[k]=dict(count=len(t),min_ns=int(d.min()),max_ns=int(d.max()),nonincreasing=int(sum(d<=0)))
 # Compare gyro integrals with changes of truth orientation. Fixed sensor-frame axes.
 integral=cumulative_trapezoid(a['imu'][:,3:6],it,axis=0,initial=0);dq=(q[:-1].inv()*q[1:]).as_rotvec();dt=np.diff(tt);motion=(tt[:-1]>5)&(tt[1:]<it[-1]-.02)&(tt[:-1]>it[0]+.02);time_scores=[]
 for shift in np.arange(-.010,.0101,.001):
  integ=np.column_stack([np.interp(tt+shift,it,integral[:,j]) for j in range(3)]);res=(np.diff(integ,axis=0)-dq)/dt[:,None];res=res[motion];center=res-res.mean(0);time_scores.append(dict(shift_s=float(shift),rmse_rad_s=float(np.sqrt(np.mean(center**2))),mean_bias_rad_s=res.mean(0).tolist()))
 best=min(time_scores,key=lambda x:x['rmse_rad_s'])
 # This is a timestamp check, not proof of every image's render time.
 source_order=np.diff(a['order'][:,0]);cam_equal=bool(np.array_equal(a['cam0_t'],a['cam1_t']) and np.array_equal(a['cam0_t'],a['cam2_t']))
 joint_windows={}
 for name,start in [('after_initialization',2003000000),('tracking',meta['window']['start_ns'])]:
  jm=a['joint_t']>=start;joint_windows[name]={n:float(np.ptp(a['joints'][jm,j])) for j,n in enumerate(json.loads((folder/'joint_names.json').read_text())) if n.startswith('joint')}
 im=(it>tt[0])&(it<tt[-1]);imu_time=it[im];imu_q=Slerp(tt,q)(imu_time);world_acc=imu_q.apply(a['imu'][im,:3]-bias.mean(0))-[0,0,9.81];dv=cumulative_trapezoid(world_acc,imu_time,axis=0,initial=0)
 sample_t=np.arange(meta['window']['start_ns']*1e-9+.1,imu_time[-1]-.1,.1);sample_dv=np.column_stack([np.interp(sample_t,imu_time,dv[:,j]) for j in range(3)]);sample_v=np.column_stack([np.interp(sample_t,tt,gt[:,7+j]) for j in range(3)]);dv_error=np.diff(sample_dv-sample_v,axis=0)
 result=dict(cadence=cadence,all_camera_header_stamps_equal=cam_equal,storage_order_header_reversals=int(sum(source_order<0)),max_header_reversal_ns=int(max(0,-source_order.min())),stationary_2to5_bias_vs_9p81=bias.mean(0).tolist(),stationary_bias_std=bias.std(0).tolist(),joint_ranges=joints,joint_ranges_by_window=joint_windows,gyro_time_scores=time_scores,best_gyro_shift=best,imu_integrated_0p1s_velocity_error_rmse_mps=np.sqrt(np.mean(dv_error**2,axis=0)).tolist(),imu_integration_check_note='Diagnostic uses truth rotation and static bias, then compares integrated acceleration with truth velocity differences over 0.1s. Not estimator input.')
 (folder/'sensor_audit.json').write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
 result=dict(calibration=calibration(),sensors={p.name:sensors(p) for p in (OUT/'inputs').iterdir() if (p/'manifest.json').exists()});(OUT/'input_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:dict(bias=v['stationary_2to5_bias_vs_9p81'],gyro=v['best_gyro_shift'],sync=v['all_camera_header_stamps_equal']) for k,v in result['sensors'].items()},indent=2))
