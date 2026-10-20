#!/usr/bin/env python3
"""Save at most two live images to inspect airframe occlusion at high pitch."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import time
import threading
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--flight',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,required=True)
    a=p.parse_args();a.output.mkdir(exist_ok=False);os.environ['ROS_MASTER_URI']=f'http://127.0.0.1:{a.port}'
    import rospy,cv2
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64MultiArray
    rospy.init_node('extreme_pitch_inspection',disable_signals=True)
    lock=threading.Lock();encoders=deque(maxlen=3000);images=deque(maxlen=5);saved={}
    def encoder(m):
        with lock:encoders.append(tuple(m.data))
    def image(m):
        with lock:images.append(m)
    subscriptions=[rospy.Subscriber('/active/encoder',Float64MultiArray,encoder,queue_size=100),
                   rospy.Subscriber('/active/image',Image,image,queue_size=5,buff_size=2000000)]
    start=time.monotonic()
    while not (a.flight/'runtime.json').exists() and len(saved)<2 and time.monotonic()-start<900:
        with lock:
            frames=list(images);angles=list(encoders)
        for m in frames:
            t=m.header.stamp.to_sec()
            if not angles:continue
            e=min(angles,key=lambda e:abs(e[0]-t))
            if abs(e[0]-t)>1e-7:continue
            if abs(e[2])<np.deg2rad(75):continue
            key='pitch_positive' if e[2]>0 else 'pitch_negative'
            if key in saved:continue
            if m.encoding!='mono8':raise RuntimeError('Expected the unchanged mono8 estimator image')
            im=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width]
            path=a.output/(key+'.png')
            if not cv2.imwrite(str(path),im):raise RuntimeError('Could not save inspection image')
            saved[key]=dict(stamp_ns=m.header.stamp.to_nsec(),encoder_stamp_ns=round(e[0]*1e9),
                yaw_deg=float(np.rad2deg(e[1])),pitch_deg=float(np.rad2deg(e[2])),width=m.width,height=m.height)
            (a.output/'frames.json').write_text(json.dumps(saved,indent=2)+'\n')
        time.sleep(.05)
    for sub in subscriptions:sub.unregister()
    print(json.dumps(saved),flush=True)


if __name__=='__main__':main()
