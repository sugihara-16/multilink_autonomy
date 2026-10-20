#!/usr/bin/env python3
"""Audit delivered IMU, images and acquisition-time camera geometry, without truth."""
import argparse,json,struct,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from report_routes import load
p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();route=load(a.flight/'route_tracking.csv');mask=route['route_t']>0;lo=round(route['sim_t'][mask][0]*1e9);hi=round(route['sim_t'][mask][-1]*1e9)
parent=a.flight.parent if a.flight.name=='fixed' else a.flight
r=ET.parse(parent/'unipilot_gimbal.sdf');imu=r.find(".//sensor[@name='imu']");period=round(1e9/float(imu.findtext('update_rate')));cfg=yaml.safe_load((a.flight/'cam0.yaml').read_text());camera=r.find(".//sensor[@name='rgb_cam']/camera");w=int(camera.findtext('image/width'));h=int(camera.findtext('image/height'));fx=w/(2*np.tan(float(camera.findtext('horizontal_fov'))/2));assert cfg['image_width']==w and cfg['image_height']==h;np.testing.assert_allclose(cfg['camera_matrix']['data'],[fx,0,w/2,0,fx,h/2,0,0,1],atol=1e-10)
mount=np.fromstring(r.findtext(".//link[@name='active_camera_yaw']/pose"),sep=' ');home=Rotation.from_euler('xyz',mount[3:]);optical=Rotation.from_matrix([[0,0,1],[-1,0,0],[0,-1,0]]);encoder=load(a.flight/'encoders.csv');times=[[],[],[]];extrinsics=[];image_shapes=set();all_imu=[]
with (a.flight/'vio_input.bin').open('rb') as f:
 while header:=f.read(8):
  kind,size=struct.unpack('<II',header);data=f.read(size);assert len(data)==size;_,s,ns,n=struct.unpack_from('<IIII',data);stamp=s*10**9+ns;off=16+n
  if kind==0:all_imu.append(stamp)
  if not lo<=stamp<=hi:continue
  times[kind].append(stamp)
  if kind==1:image_shapes.add(struct.unpack_from('<II',data,off))
  if kind==2:extrinsics.append(struct.unpack_from('<7d',data,off))
for k in range(3):assert np.all(np.diff(times[k])>0)
assert times[1]==times[2] or set(times[1])==set(times[2]);assert image_shapes=={(h,w)}
x=np.array(extrinsics);t=np.array(times[2])*1e-9;yaw=np.interp(t,encoder['t'],encoder['yaw']);pitch=np.interp(t,encoder['t'],encoder['pitch']);zeros=np.zeros_like(yaw)
if a.flight.name=='fixed':yaw=zeros;pitch=zeros
expected=home*Rotation.from_rotvec(np.c_[zeros,zeros,yaw])*Rotation.from_rotvec(np.c_[zeros,pitch,zeros])*optical;rotation_error=(expected.inv()*Rotation.from_quat(x[:,3:])).magnitude();position_error=np.linalg.norm(x[:,:3]-mount[:3],axis=1)
result=dict(camera_calibration_matches_sdf=True,image_shapes=list(image_shapes),image_extrinsic_timestamps_match=True,extrinsic_position_max_error_m=float(position_error.max()),extrinsic_rotation_max_error_rad=float(rotation_error.max()),actual_encoder_range_deg=dict(yaw=np.rad2deg([yaw.min(),yaw.max()]).tolist(),pitch=np.rad2deg([pitch.min(),pitch.max()]).tolist()),imu=dict(count=len(times[0]),expected_period_ns=period,max_gap_ns=int(np.diff(times[0]).max()),missing_samples=int(np.sum(np.maximum(0,np.diff(times[0])//period-1)))),image=dict(count=len(times[1]),max_gap_ns=int(np.diff(times[1]).max())))
result['passed']=bool(rotation_error.max()<1e-5 and position_error.max()<1e-10 and result['imu']['missing_samples']==0 and result['image']['max_gap_ns']==50000000)
gaps=np.flatnonzero(np.diff(all_imu)>period)
result['acceptance_scope']='Trajectory tracking interval; full-stream gaps are separately reported, including initialization/approach.'
result['full_stream_imu_gaps']=[dict(after_stamp_ns=int(all_imu[i]),gap_ns=int(all_imu[i+1]-all_imu[i]),outside_tracking=bool(all_imu[i+1]<lo or all_imu[i]>hi)) for i in gaps]
a.output.parent.mkdir(exist_ok=True,parents=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
assert result['passed'],'Delivered input audit failed; details retained in '+str(a.output)
