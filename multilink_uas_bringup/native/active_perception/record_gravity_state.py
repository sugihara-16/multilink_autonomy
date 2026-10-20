#!/usr/bin/env python3
"""Keep one small graph-state sample per second and verify fixed-gravity mode."""
import argparse,csv,struct
from pathlib import Path
import rospy
from mimosa_msgs.msg import GraphManagerDebug


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path)
    p.add_argument('--gravity',required=True,type=float);p.add_argument('--paired',action='store_true');a=p.parse_args()
    # MIMOSA stores gravity_magnitude as float, then promotes it to double.
    gravity=struct.unpack('f',struct.pack('f',a.gravity))[0]
    rospy.init_node('gravity_state_audit');files=[];subscriptions=[];last={}
    try:
        for name,directory,topic in [('active',a.output,'/mimosa/graph/debug')]+([('fixed',a.output/'fixed','/fixed_mimosa/graph/debug')] if a.paired else []):
            f=(directory/'gravity_state.csv').open('w');files.append(f);writer=csv.writer(f)
            writer.writerow(['stamp_ns','acc_x','acc_y','acc_z','gyro_x','gyro_y','gyro_z','gravity_x','gravity_y','gravity_z'])
            def callback(m,name=name,writer=writer,f=f):
                g=m.gravity
                if abs(g.x)>1e-8 or abs(g.y)>1e-8 or abs(g.z+gravity)>1e-8:
                    rospy.logfatal('Fixed-gravity configuration was not honored: %s %s',name,g)
                    # Leave an explicit diagnostic and stop; owner detects exit.
                    (a.output/'gravity_validation_failure.txt').write_text(name+' '+str(g)+'\n')
                    rospy.signal_shutdown('Gravity validation failed');return
                t=m.header.stamp.to_nsec()
                if t-last.get(name,-10**18)<1000000000:return
                last[name]=t
                writer.writerow([t,*[format(getattr(v,k),'.17g') for v in (m.acc_bias,m.gyro_bias,g) for k in 'xyz']]);f.flush()
            subscriptions.append(rospy.Subscriber(topic,GraphManagerDebug,callback,queue_size=200,tcp_nodelay=True))
        rospy.spin()
    finally:
        for f in files:f.close()
    if (a.output/'gravity_validation_failure.txt').exists():raise RuntimeError('Fixed-gravity invariant failed')


if __name__=='__main__':main()
