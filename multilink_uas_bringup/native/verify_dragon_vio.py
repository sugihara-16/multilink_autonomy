#!/usr/bin/env python3
"""Read-only check of actual camera/IMU transforms, raw images and timestamps."""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=10)
    args=parser.parse_args()
    manifest=json.loads((args.output/'camera_manifest.json').read_text())
    rclpy.init();node=Node('verify_dragon_vio');buffer=Buffer();listener=TransformListener(buffer,node)
    bridge=CvBridge();stamps=[set(),set(),set()];images={};states=set();errors=[[],[],[]]
    def receive(msg,i):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        stamps[i].add(stamp)
        if i not in images:
            a=bridge.imgmsg_to_cv2(msg,'mono8')
            images[i]=dict(shape=list(a.shape),std=float(a.std()),frame=msg.header.frame_id)
            cv2.imwrite(str(args.output/f'dragon_cam{i}.png'),a)
    subs=[node.create_subscription(Image,c['topic'],lambda msg,i=i:receive(msg,i),qos_profile_sensor_data)
          for i,c in enumerate(manifest['cameras'])]
    subs.append(node.create_subscription(UInt8,'/dragon/flight_state',lambda msg:states.add(msg.data),10))
    deadline=time.monotonic()+args.duration
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        for i,c in enumerate(manifest['cameras']):
            try:
                t=buffer.lookup_transform('fc',f"cam_{c['link']}_optical",Time()).transform
                actual=Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
                expected=Rotation.from_quat(c['qCM']).inv()
                errors[i].append([float(np.linalg.norm(np.array([t.translation.x,t.translation.y,t.translation.z])-c['MrMC'])),
                                  float((expected.inv()*actual).magnitude())])
            except TransformException:
                pass
    common=sorted(set.intersection(*stamps))
    tf_errors=[dict(samples=len(e),max_translation_m=float(np.max(e,axis=0)[0]),max_rotation_rad=float(np.max(e,axis=0)[1])) if e else {} for e in errors]
    passed=(len(common)>20 and states=={5} and len(images)==3 and
            all(d['std']>5 and d['frame']==f"cam_{manifest['cameras'][i]['link']}_optical" for i,d in images.items()) and
            all(e and e['max_translation_m']<.015 and e['max_rotation_rad']<.03 for e in tf_errors))
    report=dict(passed=passed,flight_states=sorted(states),images=images,image_counts=[len(s) for s in stamps],
                identical_stamp_sets=len(common),median_interval_s=float(np.median(np.diff(common))) if len(common)>1 else None,
                calibration_vs_actual_tf=tf_errors)
    (args.output/'dragon_vio_sensors.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()
    if not passed:raise SystemExit(1)


if __name__=='__main__':main()
