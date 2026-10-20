#!/usr/bin/env python3
"""Stream diagnostic interventions; preserve order/timestamps and all non-selected bytes."""
import argparse,hashlib,json,struct
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('fifo',type=Path);p.add_argument('--imu-overrides',type=Path);p.add_argument('--imu-source',type=Path);p.add_argument('--image-source',type=Path);p.add_argument('--minimum-image-end-ns',type=int,default=0);p.add_argument('--audit',type=Path,required=True);a=p.parse_args()
values={};images={};image_file=None;replacement=None;replacement_index=0;emitted_imu=0
if a.imu_source:
 replacement=np.loadtxt(a.imu_source,delimiter=',',skiprows=1)
 assert np.isfinite(replacement).all() and np.all(np.diff(replacement[:,0])>0)
if a.imu_overrides:
 x=np.loadtxt(a.imu_overrides,delimiter=',',skiprows=1,ndmin=2);values={int(row[0]):row[1:] for row in x}
if a.image_source:
 image_file=a.image_source.open('rb')
 while h:=image_file.read(16):
  stamp,w,h=struct.unpack('<QII',h);images[stamp]=(image_file.tell(),w,h);image_file.seek(w*h,1)
if image_file:assert max(images)>=a.minimum_image_end_ns>0,'Reference images do not cover evaluation window'
counts=[0,0,0];skipped_tail_images=0;changed=0;changed_images=0;digest=hashlib.sha256()
with a.source.open('rb') as src,a.fifo.open('wb') as dst:
 while header:=src.read(8):
  kind,size=struct.unpack('<II',header);payload=src.read(size);assert len(payload)==size;counts[kind]+=1
  _,sec,nsec,length=struct.unpack_from('<IIII',payload);key=sec*10**9+nsec;off=16+length
  if kind==0 and replacement is not None:
   assert replacement[0,0]<=key<=replacement[-1,0],('IMU reference coverage',key)
   if counts[0]==1:replacement_index=int(np.searchsorted(replacement[:,0],key))
   while replacement_index<len(replacement) and int(replacement[replacement_index,0])<=key:
    row=replacement[replacement_index];replacement_index+=1;ns=int(row[0]);v=row[1:];packet=bytearray(payload)
    struct.pack_into('<II',packet,4,ns//10**9,ns%10**9);struct.pack_into('<3d',packet,off+104,*v[:3]);struct.pack_into('<3d',packet,off+200,*v[3:])
    dst.write(header);dst.write(packet);emitted_imu+=1
   continue
  if kind==0 and key in values:
   payload=bytearray(payload);v=values[key];struct.pack_into('<3d',payload,off+104,*v[:3]);struct.pack_into('<3d',payload,off+200,*v[3:]);changed+=1
  if kind==1 and image_file:
   if key not in images and key>max(images) and key>a.minimum_image_end_ns:skipped_tail_images+=1;continue
   assert key in images,('Missing reference image inside evaluated interval',key)
   pos,w,h=images[key];image_file.seek(pos);raw=image_file.read(w*h);assert len(raw)==w*h
   payload=payload[:off]+struct.pack('<III',h,w,5)+b'mono8'+struct.pack('<BII',0,w,w*h)+raw;header=struct.pack('<II',kind,len(payload));changed_images+=1
  if kind!=0:digest.update(header);digest.update(payload)
  dst.write(header);dst.write(payload)
if image_file:image_file.close()
a.audit.write_text(json.dumps(dict(imu_source=str(a.imu_source),emitted_imu=emitted_imu,counts=counts,skipped_tail_images=skipped_tail_images,changed_imu=changed,changed_images=changed_images,image_and_extrinsic_sha256=digest.hexdigest(),image_bytes_unchanged=not bool(image_file),source=str(a.source),overrides=str(a.imu_overrides),image_source=str(a.image_source)),indent=2)+'\n')
