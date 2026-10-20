#!/usr/bin/env python3
"""Read-only local information-scale diagnosis; no estimator or control output."""
import argparse,csv,json,os,time,threading
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from compare_fusion_sidecar import owner_environment
from core import lidar_model


def main():
    p=argparse.ArgumentParser();p.add_argument('--flight',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,required=True)
    p.add_argument('--max-wall-s',type=float,default=900.)
    p.add_argument('--vio-topic',default='/rovio/odometry')
    p.add_argument('--lidar-topic',default='/active/lidar_information')
    a=p.parse_args();start=time.monotonic()
    while True:
        env=owner_environment(a.port)
        if env and (a.flight/'mimosa.yaml').exists():break
        if time.monotonic()-start>120:raise RuntimeError('No live reference')
        time.sleep(.1)
    os.environ.update(env)
    import rospy
    from std_msgs.msg import Float64MultiArray
    from nav_msgs.msg import Odometry
    c=yaml.safe_load((a.flight/'mimosa.yaml').read_text())
    pc=yaml.safe_load((a.flight/'planner.yaml').read_text())
    e=c['lidar']['T_B_S'];T=np.eye(4);T[:3,3]=e[:3];T[:3,:3]=Rotation.from_quat(e[3:]).as_matrix()
    scale=c['odometry']['manager']['velocity_covariance_scale'];h=c['odometry']['manager']['velocity_period_s']
    lock=threading.Lock();state={'odom':None,'last':-np.inf};counts={'written':0,'unmatched':0,'invalid':0}
    a.output.parent.mkdir(exist_ok=True,parents=True)
    with a.output.open('x') as file:
        writer=csv.writer(file);writer.writerow(['stamp_s','vio_age_s',*[f'lidar_position_information_{i}' for i in range(3)],*[f'lidar_over_visual_information_{i}' for i in range(3)],'max_weak_probability'])
        def odom(m):
            with lock:state['odom']=m
        def information(m):
            with lock:
                x=np.asarray(m.data);stamp=x[1]
                if stamp-state['last']<1.:return
                v=state['odom']
                if v is None or not 0<=stamp-v.header.stamp.to_sec()<=.15:
                    counts['unmatched']+=1;return
                try:
                    H,U,weak=lidar_model(x[18:54].reshape(6,6),x[54:90].reshape(6,6),x[90:],T)
                    H+=np.diag(1/np.square(pc['prior_std']))
                    S=H[3:,3:]-H[3:,:3]@np.linalg.solve(H[:3,:3],H[:3,3:])
                    C=np.asarray(v.twist.covariance).reshape(6,6)[:3,:3]*scale*h*h
                    L=np.linalg.cholesky(C);eig=np.linalg.eigvalsh(L.T@S@L)
                    row=[stamp,stamp-v.header.stamp.to_sec(),*np.linalg.eigvalsh(S),*eig,float(max(weak))]
                    if not np.isfinite(row).all():raise ValueError('Nonfinite information')
                    writer.writerow(row);file.flush();state['last']=stamp;counts['written']+=1
                except (ValueError,np.linalg.LinAlgError):counts['invalid']+=1
        rospy.init_node('lidar_visual_scale_observer',anonymous=True,disable_signals=True)
        subs=[rospy.Subscriber(a.vio_topic,Odometry,odom,queue_size=10),
              rospy.Subscriber(a.lidar_topic,Float64MultiArray,information,queue_size=5)]
        try:
            while not (a.flight/'runtime.json').exists() and time.monotonic()-start<a.max_wall_s:time.sleep(.2)
        finally:
            for s in subs:s.unregister()
            rospy.signal_shutdown('Observation finished')
    a.output.with_suffix('.json').write_text(json.dumps(dict(counts=counts,scope=__doc__,
        source_flight=str(a.flight),finished_reference=(a.flight/'runtime.json').exists(),
        caveat='Instantaneous local planning information, with prior and DRPM probabilities; not actual fused posterior. Velocity covariance uses existing fixed calibration and one-second interval.'),indent=2)+'\n')
    print(counts,flush=True)


if __name__=='__main__':main()
