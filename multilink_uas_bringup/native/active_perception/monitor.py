#!/usr/bin/env python3
"""ROS1 validation only: truth is recorded here and never sent to estimators/planner."""
import argparse
import csv
import json
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--truth-topic',default='/dragon/ground_truth');a=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('active_validation_monitor');files=[];subs=[];counts={};startup={'checked':False,'max_displacement_m':0.};initial=[None];first_time=[None]
    for name,topic in [('rovio','/rovio/odometry'),('mimosa','/slam/odometry'),('truth',a.truth_topic)]:
        f=(a.output/(name+'.csv')).open('w',buffering=1);files.append(f);w=csv.writer(f)
        w.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw']);counts[name]=0
        def cb(m,w=w,name=name):
            p=m.pose.pose.position;q=m.pose.pose.orientation
            w.writerow([m.header.stamp.to_nsec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w]);counts[name]+=1
            if name=='rovio' and not startup['checked']:
                xyz=[p.x,p.y,p.z]
                if initial[0] is None:initial[0]=xyz;first_time[0]=m.header.stamp.to_sec()
                displacement=sum((x-y)**2 for x,y in zip(xyz,initial[0]))**.5
                startup['max_displacement_m']=max(startup['max_displacement_m'],displacement)
                if m.header.stamp.to_sec()-first_time[0]>=3.:startup['checked']=True
                startup['valid']=startup['max_displacement_m']<1.
        subs.append(rospy.Subscriber(topic,Odometry,cb,queue_size=100))
    f=(a.output/'encoders.csv').open('w',buffering=1);files.append(f);w=csv.writer(f);w.writerow(['t','yaw','pitch'])
    subs.append(rospy.Subscriber('/active/encoder',Float64MultiArray,lambda m:w.writerow(m.data),queue_size=200))
    def close():
        for f in files:f.close()
        (a.output/'monitor_counts.json').write_text(json.dumps(counts,indent=2)+'\n')
    def status(event):
        tmp=a.output/'evaluation_status.tmp';tmp.write_text(json.dumps(dict(counts=counts,startup=startup)));tmp.replace(a.output/'evaluation_status.json')
    timer=rospy.Timer(rospy.Duration(1),status)
    rospy.on_shutdown(close);rospy.spin()

if __name__=='__main__':main()
