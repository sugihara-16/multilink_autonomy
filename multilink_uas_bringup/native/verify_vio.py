"""ROS1 observation-only camera/VIO probe; save images, rates and feature IDs."""
import argparse
import rospy,time,json,cv2
from pathlib import Path
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--duration',type=float,default=12);args=parser.parse_args()
rospy.init_node('probe_vio');out=args.output;d={};cams=set();bridge=CvBridge()
def cb(m,t):
 d[t]=dict(n=d.get(t,{}).get('n',0)+1,t=m.header.stamp.to_sec())
 if isinstance(m,Image):
  a=bridge.imgmsg_to_cv2(m,'mono8');d[t].update(mean=float(a.mean()),std=float(a.std()),shape=list(a.shape));
  if t not in cams:cv2.imwrite(str(out/(t.split('/')[-2]+'.png')),a);cams.add(t)
 if isinstance(m,PointCloud2):
  ids=set(int(p[0]) for p in point_cloud2.read_points(m,field_names=['camId','x','y','z'],skip_nans=True));d[t]['ids']=list(ids)
 if isinstance(m,Odometry):d[t]['p']=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]
for name in ['front','left','right']:
 t='/rmf_unipilot/cam_'+name+'/image';rospy.Subscriber(t,Image,cb,callback_args=t,queue_size=2,buff_size=4*1024*1024)
for t,cls in [('/rovio/odometry',Odometry),('/rovio/pcl',PointCloud2),('/slam/odometry',Odometry)]:rospy.Subscriber(t,cls,cb,callback_args=t,queue_size=5)
time.sleep(args.duration);print(json.dumps(d,indent=2));(out/'probe.json').write_text(json.dumps(d,indent=2))
