#!/usr/bin/env python3
"""Recorded-data regression: zero gimbal rotation must preserve old ROVIO output."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import time
import xmlrpc.client
import numpy as np
import yaml
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent;WS=NATIVE.parents[3]
OUT=NATIVE/'logs/active-static-regression'

def prepare():
    import cv2
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    OUT.mkdir(exist_ok=True)
    root=NATIVE/'logs/zab-offline/monocular_startup/bias/run1_tail_b1'
    for name in ['rovio.info','cam0.yaml']:shutil.copyfile(root/name,OUT/name)
    c=yaml.safe_load((OUT/'cam0.yaml').read_text());P=np.array(c['camera_matrix']['data']).reshape(3,3)
    K=P.copy();K[0,:]*=1.5;K[1,:]*=1.5
    maps=cv2.initUndistortRectifyMap(K,np.zeros(5),np.eye(3),P,(480,360),cv2.CV_32FC1)
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(NATIVE/'logs/zab-local-bags/run1_tail_b1/rosbag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    topics=['/dragon/imu/raw','/dragon/cam_link3/image'];r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in topics}
    records=[];counts=[0,0]
    while r.has_next():
        topic,data,_=r.read_next();m=deserialize_message(data,types[topic]);t=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        if t>20000000000:break
        if t<2003000000:continue
        if topic.endswith('imu/raw'):
            a=m.linear_acceleration;w=m.angular_velocity;kind=0;payload=struct.pack('<6d',a.x,a.y,a.z,w.x,w.y,w.z)
        else:
            kind=1;im=np.ndarray((m.height,m.width,3),dtype=np.uint8,buffer=m.data,strides=(m.step,3,1))
            gray=cv2.cvtColor(im,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
            payload=cv2.remap(gray,*maps,interpolation=cv2.INTER_LINEAR).tobytes()
        records.append((t,kind,payload));counts[kind]+=1
    with (OUT/'input.bin').open('wb') as f:
        for t,k,payload in sorted(records,key=lambda x:(x[0],x[1])):f.write(struct.pack('<IQ',k,t)+payload)
    (OUT/'input.json').write_text(json.dumps(dict(counts=counts,source='local run1_tail_b1, 2.003–20 s; no truth',bytes=(OUT/'input.bin').stat().st_size),indent=2)+'\n')

def replay():
    env={k:v for k,v in os.environ.items() if k in ('HOME','USER','PATH','LANG')}
    env['PATH']='/usr/local/bin:/usr/bin:/bin'
    prefix=f'source {NATIVE}/env_ros1.sh\nsource {WS}/slam_ros1_ws/devel/setup.bash\nexport ROS_MASTER_URI=http://127.0.0.1:11380\nexec '
    def cmd(args):return ['bash','--noprofile','--norc','-c',prefix+shlex.join(map(str,args))]
    import socket
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',11380))==0:raise RuntimeError('Test master port in use')
    with (OUT/'master.log').open('w') as log:master=subprocess.Popen(cmd(['roscore','-p','11380']),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        proxy=xmlrpc.client.ServerProxy('http://127.0.0.1:11380')
        for _ in range(100):
            try:proxy.getPid('/regression');break
            except OSError:time.sleep(.1)
        for label,binary in [('baseline',NATIVE/'logs/zab-offline/livo_comparison/audit25'),('gimbal',WS/'build/active_perception/rovio_static_replay')]:
            with (OUT/(label+'.log')).open('w') as log,(OUT/'input.bin').open('rb') as f:
                subprocess.run(cmd([binary,OUT/'rovio.info',OUT,OUT/label]),env=env,stdin=f,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
        checks={}
        for suffix in ['vio.csv','state.csv','features.csv']:
            a=(OUT/('baseline_'+suffix)).read_bytes();b=(OUT/('gimbal_'+suffix)).read_bytes()
            checks[suffix]=dict(byte_identical=a==b,baseline_sha256=hashlib.sha256(a).hexdigest(),gimbal_sha256=hashlib.sha256(b).hexdigest())
            if suffix=='features.csv':
                # Old single-camera diagnostic advertised three status columns
                # but wrote only status0. Compare every actual feature row.
                assert a.splitlines()[0] in (b.splitlines()[0],b.splitlines()[0]+b',status1,status2')
                checks[suffix]['data_rows_identical']=a.splitlines()[1:]==b.splitlines()[1:]
        (OUT/'validation.json').write_text(json.dumps(checks,indent=2)+'\n')
        if not all(v.get('data_rows_identical',v['byte_identical']) for v in checks.values()):raise AssertionError(checks)
        (OUT/'input.bin').unlink();print(json.dumps(checks,indent=2))
    finally:
        os.killpg(master.pid,signal.SIGINT)
        try:master.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','replay']);a=p.parse_args()
    globals()[a.action]()
