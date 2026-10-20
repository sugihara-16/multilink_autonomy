#!/usr/bin/env python3
"""Share real reference frames while views coincide, then permanently separate.

Experimental common-input control, not a change to the estimator. Images are
neither modified nor retimed. The switch uses measured camera extrinsics only.
"""
import argparse,hashlib,json,math,threading
from pathlib import Path

def coincident(a,b,tolerance=1e-6):
    pa,pb=a.pose.position,b.pose.position;qa,qb=a.pose.orientation,b.pose.orientation
    translation=math.sqrt(sum((getattr(pa,k)-getattr(pb,k))**2 for k in ('x','y','z')))
    # Unit quaternion chord, sign invariant, avoids acos precision near zero.
    va=[getattr(qa,k) for k in ('x','y','z','w')];vb=[getattr(qb,k) for k in ('x','y','z','w')]
    chord=math.sqrt(min(sum((x-y)**2 for x,y in zip(va,vb)),sum((x+y)**2 for x,y in zip(va,vb))))
    return translation<=1e-8 and 4*math.asin(min(1.,chord/2))<=tolerance

class Selector:
    def __init__(self):self.cache=[{} for _ in range(4)];self.separated=False;self.last=-1
    def add(self,k,stamp,msg):
        if stamp<=self.last:return []
        self.cache[k][stamp]=msg;ready=[]
        for stamp in sorted(set.intersection(*(set(c) for c in self.cache))):
            image,pose,reference,reference_pose=[c.pop(stamp) for c in self.cache]
            if not coincident(pose,reference_pose):self.separated=True
            ready.append((stamp,image if self.separated else reference,pose if self.separated else reference_pose,self.separated))
            self.last=stamp
        for c in self.cache:
            for stamp in list(c):
                if stamp<=self.last:del c[stamp]
            if len(c)>100:raise RuntimeError('Paired image/extrinsic streams failed to synchronize')
        return ready

def main():
    import rospy
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('paired_initialization');lock=threading.RLock();selector=Selector();digest=hashlib.sha256()
    report=dict(scope=__doc__,shared_frames=0,separate_frames=0,switch_stamp_ns=None,last_stamp_ns=None,first_stamp_ns=None,missing_periods=0,max_gap_ns=0)
    image_pub=rospy.Publisher('/paired/active/image',Image,queue_size=30)
    pose_pub=rospy.Publisher('/paired/active/extrinsic',PoseStamped,queue_size=30)
    def callback(m,k):
        with lock:
            for stamp,image,pose,separate in selector.add(k,m.header.stamp.to_nsec(),m):
                last=report['last_stamp_ns']
                if last is not None:
                    delta=stamp-last;report['max_gap_ns']=max(report['max_gap_ns'],delta);report['missing_periods']+=max(0,round(delta/50000000)-1)
                else:report['first_stamp_ns']=stamp
                if separate:
                    report['separate_frames']+=1
                    if report['switch_stamp_ns'] is None:report['switch_stamp_ns']=stamp
                else:
                    report['shared_frames']+=1;digest.update(bytes(image.data))
                report['last_stamp_ns']=stamp
                pose_pub.publish(pose);image_pub.publish(image)
    subs=[rospy.Subscriber(topic,typ,lambda m,k=k:callback(m,k),queue_size=100,buff_size=20000000,tcp_nodelay=True) for k,(topic,typ) in enumerate([
        ('/active/image',Image),('/active/extrinsic',PoseStamped),('/fixed/active/image',Image),('/fixed/active/extrinsic',PoseStamped)])]
    def save(_=None):
        with lock:
            data=dict(report,shared_image_sha256=digest.hexdigest(),pending=[len(c) for c in selector.cache]);tmp=a.output/'shared_initialization.tmp';tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(a.output/'shared_initialization.json')
    timer=rospy.Timer(rospy.Duration(1),save);rospy.on_shutdown(save);rospy.spin()

if __name__=='__main__':main()
