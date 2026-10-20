#!/usr/bin/env python3
"""Rectify the three calibrated raw images to the pinhole model used by ROVIO.
Preserve acquisition stamps exactly; no inference, motion or ground truth input.
"""
import argparse
from pathlib import Path
import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

parser=argparse.ArgumentParser();parser.add_argument('--calibration',type=Path,required=True)
parser.add_argument('--topics', nargs=3, default=[f'/rmf_unipilot/cam_{name}/image' for name in ('front','left','right')])
args=parser.parse_args(rospy.myargv()[1:])
cv2.setNumThreads(1)
rospy.init_node('vio_images');bridge=CvBridge();publishers=[];maps=[];subscribers=[]
for i,topic in enumerate(args.topics):
    raw=yaml.safe_load((args.calibration/f'raw_cam{i}.yaml').read_text())
    rect=yaml.safe_load((args.calibration/f'cam{i}.yaml').read_text())
    K=np.array(raw['camera_matrix']['data']).reshape(3,3)
    D=np.array(raw['distortion_coefficients']['data'])
    P=np.array(rect['camera_matrix']['data']).reshape(3,3)
    size=(rect['image_width'],rect['image_height'])
    fn=cv2.fisheye.initUndistortRectifyMap if raw['distortion_model']=='equidistant' else cv2.initUndistortRectifyMap
    maps.append(fn(K,D,np.eye(3),P,size,cv2.CV_32FC1))
    publishers.append(rospy.Publisher(f'/vio/cam{i}/image',Image,queue_size=2))
    def receive(msg,i=i):
        try:
            raw_img=bridge.imgmsg_to_cv2(msg,'mono8')
            rect_img=cv2.remap(raw_img,*maps[i],interpolation=cv2.INTER_LINEAR)
            out=bridge.cv2_to_imgmsg(rect_img,'mono8');out.header=msg.header
            publishers[i].publish(out)
        except Exception as exc:rospy.logerr_throttle(5,str(exc))
    subscribers.append(rospy.Subscriber(topic,Image,receive,queue_size=2,buff_size=4*1024*1024))
rospy.spin()
