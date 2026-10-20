#!/usr/bin/env python3
"""Read-only, 1 Hz uncertainty samples; exit when the owning flight stops."""
import argparse,csv,json,time
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry


def main():
    parser=argparse.ArgumentParser();parser.add_argument('flight',type=Path)
    parser.add_argument('--max-wall-s',type=float,default=1800.);args=parser.parse_args()
    rospy.init_node('vio_uncertainty_probe',anonymous=True)
    files=[];subs=[];last={};counts={};start=time.monotonic()
    try:
        for name,path,topic in [('active',args.flight,'/rovio/odometry'),('fixed',args.flight/'fixed','/fixed/rovio/odometry')]:
            f=(path/'vio_uncertainty.csv').open('x');files.append(f);w=csv.writer(f)
            w.writerow(['stamp_ns','vx','vy','vz']+['pose_var_'+s for s in ['x','y','z','rx','ry','rz']]+['velocity_cov_'+str(i)+str(j) for i in range(3) for j in range(3)])
            def callback(msg,name=name,w=w,f=f):
                t=msg.header.stamp.to_nsec()
                if t-last.get(name,-10**18)<1000000000:return
                last[name]=t;counts[name]=counts.get(name,0)+1
                v=msg.twist.twist.linear
                w.writerow([t,v.x,v.y,v.z,*[msg.pose.covariance[7*i] for i in range(6)],*[msg.twist.covariance[6*i+j] for i in range(3) for j in range(3)]]);f.flush()
            subs.append(rospy.Subscriber(topic,Odometry,callback,queue_size=100,tcp_nodelay=True))
        while not rospy.is_shutdown() and not (args.flight/'runtime.json').exists() and time.monotonic()-start<args.max_wall_s:
            time.sleep(.5)
    finally:
        rospy.signal_shutdown('Owner finished or probe deadline reached')
        for sub in subs:sub.unregister()
        for f in files:f.close()
        (args.flight/'vio_uncertainty_probe.json').write_text(json.dumps(dict(scope='Read-only diagnostic started after flight initialization. One sample per sensor-second; no estimator/planner input, no truth subscription. Velocity and covariance use the published odometry child frame.',counts=counts,wall_s=time.monotonic()-start),indent=2)+'\n')


if __name__=='__main__':main()
