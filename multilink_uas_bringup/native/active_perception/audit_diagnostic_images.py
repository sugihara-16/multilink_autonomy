#!/usr/bin/env python3
import argparse,json,struct
from pathlib import Path
import numpy as np,cv2
p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True)
def index(path):
 f=path.open('rb');idx={}
 while data:=f.read(16):
  t,w,h=struct.unpack('<QII',data);idx[t]=(f.tell(),w,h);f.seek(w*h,1)
 return f,idx
def read(f,x):
 off,w,h=x;f.seek(off);return np.frombuffer(f.read(w*h),np.uint8).reshape(h,w)
cf,ci=index(a.flight/'diagnostic_sensors/clean.bin');wf,wi=index(a.flight/'diagnostic_sensors/wide.bin');metrics=[];samples=[]
with (a.flight/'vio_input.bin').open('rb') as f:
 while h:=f.read(8):
  kind,size=struct.unpack('<II',h)
  if kind!=1:f.seek(size,1);continue
  data=f.read(size);_,sec,ns,l=struct.unpack_from('<IIII',data);t=sec*10**9+ns
  if ns!=0 or t not in ci or t not in wi:continue
  off=16+l;hh,ww,enc=struct.unpack_from('<III',data,off);off+=12+enc;_,step,length=struct.unpack_from('<BII',data,off);off+=9;im=np.frombuffer(data[off:off+length],np.uint8).reshape(hh,step)[:,:ww]
  clean=read(cf,ci[t]);wide=read(wf,wi[t]);crop=wide[:,200:200+ww];diff=im.astype(float)-clean;cd=crop.astype(float)-clean
  metrics.append(dict(stamp_ns=t,image_noise_rmse=float(np.sqrt(np.mean(diff**2))),wide_crop_rmse=float(np.sqrt(np.mean(cd**2))),dark_fraction=float(np.mean(clean<20)),clean_std=float(clean.std())))
  if sec in [40,60,100,150,200]:
   for name,x in [('native',im),('clean',clean),('wide',wide)]:cv2.imwrite(str(a.output/f'{sec}_{name}.png'),x)
cf.close();wf.close();r=dict(samples=len(metrics),scope='Same timestamp and viewpoint. Wide camera preserves vertical FOV and focal length; center crop is independently rendered.',summary={k:np.percentile([m[k] for m in metrics],[0,50,100]).tolist() for k in ['image_noise_rmse','wide_crop_rmse','dark_fraction','clean_std']},samples_metrics=metrics)
(a.output/'image_audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r['summary'],indent=2))
