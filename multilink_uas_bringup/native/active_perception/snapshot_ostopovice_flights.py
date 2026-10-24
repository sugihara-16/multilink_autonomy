#!/usr/bin/env python3
"""Read-only transport observer: one image per view and zone, no continuous video."""
import argparse,json,os,time,signal,threading
from pathlib import Path
import numpy as np,cv2
from PIL import Image as PILImage
os.environ['GZ_PARTITION']='uas_speed_85'
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.odometry_pb2 import Odometry

def main():
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args();out=a.campaign.parent/'flight_snapshots';out.mkdir(exist_ok=True)
    state=dict(key=None,x=None);pending={};saved={};lock=threading.Lock();node=Node();callbacks=[];stop=False
    def ending(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,ending);signal.signal(signal.SIGINT,ending)
    def odom(m):state['x']=m.pose.position.x
    node.subscribe(Odometry,'/model/rmf_unipilot/odometry',odom)
    topics={n:'/view_array/'+n+'/raw_image' for n in ['yaw_m50','yaw_p50','pitch_p90','pitch_m90']};topics.update(front='/rmf_unipilot/cam_fixed/image',active='/rmf_unipilot/cam_gimbal/image')
    for name,topic in topics.items():
        def callback(msg,name=name):
            key=state['key'];x=state['x']
            if not key or not key.endswith('20262501') or x is None:return
            distances=np.abs(np.array([15,38,62,85])-x);zone=int(np.argmin(distances))
            if distances[zone]>.25:return
            label=f'{key}_zone{zone}_{name}'
            with lock:
                if label in pending or label in saved:return
                rgb=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
                pending[label]=(rgb,dict(trial=key,zone=zone,view=name,x_m=x,stamp_s=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9))
        callbacks.append(callback);node.subscribe(Image,topic,callback)
    while not stop:
        try:m=json.loads((a.campaign/'campaign.json').read_text())
        except (FileNotFoundError,json.JSONDecodeError):time.sleep(.2);continue
        active=next((k for k,v in m['runs'].items() if v['status']=='running'),None)
        if active!=state['key']:state.update(key=active,x=None)
        with lock:items=list(pending.items());pending.clear()
        for label,(rgb,metadata) in items:
            PILImage.fromarray(rgb).save(out/(label+'.png'));gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);metadata['fast20_corners']=len(cv2.FastFeatureDetector_create(20,True).detect(gray));saved[label]=metadata
        if items:(out/'manifest.json').write_text(json.dumps(saved,indent=2)+'\n');print('snapshots',len(saved),flush=True)
        if m['status'] in ['complete','failed','interrupted']:break
        time.sleep(.2)

if __name__=='__main__':main()
