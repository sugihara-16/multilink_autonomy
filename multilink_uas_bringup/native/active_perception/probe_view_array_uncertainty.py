#!/usr/bin/env python3
"""Read-only scalar covariance audit; no truth subscription or publications."""
import argparse,csv,json,time
from pathlib import Path
import numpy as np
import rospy
from nav_msgs.msg import Odometry

def main():
    p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(exist_ok=False)
    rospy.init_node('array_covariance_probe',anonymous=True);topics={n:'/view_array/'+n+'/rovio/odometry' for n in ['yaw_m50','yaw_p50','pitch_p90','pitch_m90']};topics.update(front='/fixed/rovio/odometry',active='/rovio/odometry');subs=[];files=[];last={};start=time.monotonic();counts={}
    try:
        for name,topic in topics.items():
            f=(a.output/(name+'.csv')).open('x');files.append(f);writer=csv.writer(f);writer.writerow(['stamp_ns','pose_d_opt','pose_cov_sign','vx','vy','vz'])
            def callback(msg,name=name,writer=writer,f=f):
                stamp=msg.header.stamp.to_nsec()
                if stamp-last.get(name,-10**18)<1000000000:return
                last[name]=stamp;C=np.array(msg.pose.covariance).reshape(6,6);sign,logdet=np.linalg.slogdet(C);dopt=np.exp(logdet/6) if sign>0 else float('nan');v=msg.twist.twist.linear;writer.writerow([stamp,dopt,sign,v.x,v.y,v.z]);f.flush();counts[name]=counts.get(name,0)+1
            subs.append(rospy.Subscriber(topic,Odometry,callback,queue_size=20,tcp_nodelay=True))
        while not rospy.is_shutdown() and not (a.flight/'runtime.json').exists() and time.monotonic()-start<1200:time.sleep(.5)
    finally:
        rospy.signal_shutdown('Owning flight stopped')
        for s in subs:s.unregister()
        for f in files:f.close()
        (a.output/'status.json').write_text(json.dumps(dict(counts=counts,read_only=True,stopped=True,wall_s=time.monotonic()-start),indent=2)+'\n')

if __name__=='__main__':main()
