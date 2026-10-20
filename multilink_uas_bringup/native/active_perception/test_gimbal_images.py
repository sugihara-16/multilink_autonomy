#!/usr/bin/env python3
"""Image-level gimbal regression: a stationary body observes textured cube walls.
Synthetic truth only generates sensor messages; estimator receives image/IMU/encoder.
"""
import argparse,json,struct,shutil,re
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation
from audit_raw_tracks import records

def make(source,out,body_motion=False,fixed_gimbal=False):
 out.mkdir(parents=True,exist_ok=True);cv2.setNumThreads(1)
 for n in ['unipilot_gimbal.sdf','world_serial.sdf','cam0.yaml','camera_manifest.json']:shutil.copy2(source/n,out/n)
 config=(source/'rovio.info').read_text()
 for section,end in [('    Covariance\n','ImgUpdate\n'),('Prediction\n','PoseUpdate\n')]:
  a=config.index(section);b=config.index(end,a);block=config[a:b]
  for k in ['acb','gyb']:
   for i in range(3):block=re.sub(rf'({k}_{i}\s+)[^;\s]+;',lambda m:m[1]+'1e-16;',block,count=1)
  config=config[:a]+block+config[b:]
 (out/'rovio.info').write_text(config);(out/'protocol.json').write_text(json.dumps(dict(stationary_initialization_duration=0.,stationary_mode='wait',route_end=20.,synthetic_test=True)))
 cfg=yaml.safe_load((source/'cam0.yaml').read_text());w,h=cfg['image_width'],cfg['image_height'];K=np.array(cfg['camera_matrix']['data']).reshape(3,3)
 pix=np.stack(np.meshgrid(np.arange(w),np.arange(h)),axis=-1).reshape(-1,2);rays=np.c_[pix,np.ones(len(pix))]@np.linalg.inv(K).T
 rng=np.random.default_rng(224);textures=[cv2.GaussianBlur(rng.integers(0,256,(1024,1024),dtype=np.uint8),(5,5),0) for _ in range(6)]
 optical=Rotation.from_matrix([[0,0,1],[-1,0,0],[0,-1,0]]);position=np.array([.18,0,-.025]);template={}
 for kind in [0,1,2]:
  stamp,data,off=next(records(source/'vio_input.bin',kind));template[kind]=(data,off)
 def header(data,t):
  b=bytearray(data);struct.pack_into('<II',b,4,t//10**9,t%10**9);return b
 def save(f,k,data):f.write(struct.pack('<II',k,len(data)));f.write(data)
 def body(sec):
  u=max(0.,sec-3.)
  if not body_motion:return np.zeros(3),Rotation.identity(),np.zeros(3)
  f=np.array([.5,.35,.4]);a=np.array([.5,.3,.2])
  pos=a*(1-np.cos(f*u));acc=a*f*f*np.cos(f*u) if u>0 else np.zeros(3)
  orientation=Rotation.from_euler('ZYX',np.deg2rad([30,10,7])*(1-np.cos(np.array([.25,.45,.3])*u)))
  return pos,orientation,acc
 enc=[]
 with (out/'vio_input.bin').open('wb') as f:
  for step in range(1000,26001):
   t=step*1000000;sec=step/1000;pos,Rwb,acc=body(sec)
   eps=1e-5;omega=(body(sec-eps)[1].inv()*body(sec+eps)[1]).as_rotvec()/(2*eps)
   specific=Rwb.inv().apply(acc+[0.,0.,9.8])
   b=header(template[0][0],t);off=template[0][1];struct.pack_into('<3d',b,off+104,*omega);struct.pack_into('<3d',b,off+200,*specific);save(f,0,b)
   if step%50:continue
   u=max(0.,sec-3);yaw=0. if fixed_gimbal else np.deg2rad(40)*np.sin(u*.35);pitch=0. if fixed_gimbal else np.deg2rad(65)*np.sin(u*.22);R=Rotation.from_euler('z',yaw)*Rotation.from_euler('y',pitch)*optical
   camera_position=pos+Rwb.apply(position)
   direction=(Rwb*R).apply(rays);distances=(np.where(direction>0,5.,-5.)-camera_position)/np.where(np.abs(direction)>1e-15,direction,1e-15);distances[np.abs(direction)<1e-15]=np.inf;axis=np.argmin(distances,axis=1);positive=direction[np.arange(len(axis)),axis]>0;face=2*axis+positive;gray=np.zeros(h*w,np.uint8)
   # Exact ray/cube intersection, including the camera/IMU lever arm.
   lengths=(np.where(positive,5.,-5.)-camera_position[axis])/direction[np.arange(len(axis)),axis];world=camera_position+lengths[:,None]*direction
   for face_id in range(6):
    mask=face==face_id;j=face_id//2;uv=world[:,[i for i in range(3) if i!=j]];uv=((uv/10+.5)*1023).astype(np.float32);sample=cv2.remap(textures[face_id],uv[:,0].reshape(h,w),uv[:,1].reshape(h,w),cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT);gray[mask]=sample.ravel()[mask]
   b=header(template[2][0],t);struct.pack_into('<7d',b,template[2][1],*position,*R.as_quat());save(f,2,b)
   b=header(template[1][0][:template[1][1]],t)+struct.pack('<III',h,w,5)+b'mono8'+struct.pack('<BII',0,w,w*h)+gray.tobytes();save(f,1,b);enc.append([sec,yaw,pitch])
 ts=np.arange(1.,26.001,.01);truth=np.array([np.r_[body(t)[0],body(t)[1].as_quat()] for t in ts]);np.savetxt(out/'truth.csv',np.c_[np.rint(ts*1e9),truth],delimiter=',',header='stamp_ns,x,y,z,qx,qy,qz,qw',comments='',fmt='%.15g');np.savetxt(out/'route_tracking.csv',np.c_[ts,np.clip(ts-5,0,20)],delimiter=',',header='sim_t,route_t',comments='',fmt='%.10g');np.savetxt(out/'encoders.csv',enc,delimiter=',',header='t,yaw,pitch',comments='')
 (out/'test_scope.json').write_text(json.dumps(dict(scope=__doc__,body_motion='simultaneous smooth 3-axis rotation and translation' if body_motion else 'zero',gimbal_motion='fixed' if fixed_gimbal else 'yaw ±40deg / pitch ±65deg; optical center fixed in body',images='raycast textured cube at ±5m; independent of gimbal state transformation code',imu='noiseless 1000Hz, analytic translation acceleration and central-difference angular velocity; lever arm included',frames=len(enc)),indent=2));print(out)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--body-motion',action='store_true');p.add_argument('--fixed-gimbal',action='store_true');a=p.parse_args();make(a.source,a.output,a.body_motion,a.fixed_gimbal)
