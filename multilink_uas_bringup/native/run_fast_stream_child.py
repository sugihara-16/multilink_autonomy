#!/usr/bin/env python3
"""Load frozen parameters in ROS1, then replace this process with the stream adapter."""
import os,sys,yaml,rospy
from pathlib import Path
os.environ['ROS_MASTER_URI']=os.environ.get('FAST_STREAM_MASTER_URI','http://127.0.0.1:11376')
rospy.init_node('fast_stream_config',anonymous=True,disable_signals=True)
cfg=yaml.safe_load(Path(sys.argv[1]).read_text())
if rospy.has_param('/multicamera'):rospy.delete_param('/multicamera')
for k,v in cfg['parameters'].items():rospy.set_param('/'+k,v)
rospy.set_param('/laserMapping',cfg['camera'])
rospy.set_param('/comparison_disable_visual_update',False)
rospy.set_param('/comparison_disable_visual_after_s',-1.0)
binary='/root/multilink_autonomy_ws/comparison_ws/devel/lib/fast_livo/fastlivo_stream'
os.execv(binary,[binary,sys.argv[2]])
