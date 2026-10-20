#!/usr/bin/env python3
"""Compare covariance-export ROVIO to the untouched binary on the same input."""
import argparse
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import yaml
from compare_fusion_sidecar import owner_environment
from joint_velocity_view import decode


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--flight',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,required=True);a=p.parse_args()
    a.output.mkdir(exist_ok=False)
    start=time.monotonic();proc=None;result=dict(valid=False)
    def stop(*args):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while True:
            env=owner_environment(a.port)
            if env and (a.flight/'protocol.json').exists():break
            if time.monotonic()-start>180:raise RuntimeError('Reference did not start')
            time.sleep(.1)
        os.environ.update(env)
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Float64MultiArray
        import sensor_msgs.point_cloud2 as pc2
        rospy.init_node('joint_information_audit',disable_signals=True)
        odoms={};old={};clouds={};joints={};errors=[]
        def odom(m,target):
            target[m.header.stamp.to_nsec()]=m
        def cloud(m):
            rows=[r for r in pc2.read_points(m,field_names=['id','status','x','y','z'],skip_nans=True) if r[1]==4]
            clouds[m.header.stamp.to_nsec()]=np.array([[r[0],*r[2:]] for r in rows]).reshape(-1,4)
        def joint(m):
            try:
                t,ids,points,C=decode(m.data)
                joints[t]=(ids,points,C)
            except Exception as e:errors.append(str(e))
        subs=[rospy.Subscriber('/rovio/odometry',Odometry,lambda m:odom(m,odoms),queue_size=100),
              rospy.Subscriber('/joint_reference/odometry',Odometry,lambda m:odom(m,old),queue_size=100),
              rospy.Subscriber('/rovio/pcl',PointCloud2,cloud,queue_size=100),
              rospy.Subscriber('/rovio_gimbal/joint_information',Float64MultiArray,joint,queue_size=100)]
        proto=json.loads((a.flight/'protocol.json').read_text())
        if not proto['joint_rovio_information']:raise RuntimeError('Expected instrumented reference')
        config={'joint_audit_reference':proto['imu_runtime_parameters']}
        path=a.output/'runtime.yaml';path.write_text(yaml.safe_dump(config))
        subprocess.run(['rosparam','load',str(path)],env=env,check=True,stdout=subprocess.DEVNULL)
        cmd=['/root/multilink_autonomy_ws/build/active_perception/rovio_gimbal',
             '__name:=joint_audit_reference',f'_config:={a.flight}/rovio.info',f'_calibration:={a.flight}/cam0.yaml',
             f'_stationary_initialization_duration:={proto["stationary_initialization_duration"]}',
             f'_stationary_mode:={proto["stationary_mode"]}',f'_diagnostic_csv:={a.output}/vio_diagnostic.csv',
             '/slam/input/imu:=/vio/initialized_imu',
             '/active/image:=/paired/active/image','/active/extrinsic:=/paired/active/extrinsic']
        for topic in ['/rovio/odometry','/rovio/pcl','/tf','/tf_static']:
            cmd.append(topic+':='+('/joint_reference/odometry' if topic=='/rovio/odometry' else '/joint_reference'+topic))
        (a.output/'command.json').write_text(json.dumps(cmd,indent=2))
        with (a.output/'node.log').open('w') as log:
            proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        while not (a.flight/'runtime.json').exists():
            if proc.poll() is not None:raise RuntimeError('Original ROVIO exited early')
            if time.monotonic()-start>900:raise RuntimeError('Diagnostic timeout')
            time.sleep(.2)
        for sub in subs:sub.unregister()
        def vector(m):
            p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear;w=m.twist.twist.angular
            return [p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z,*m.pose.covariance,*m.twist.covariance]
        common=sorted(odoms.keys()&old.keys());joint_times=sorted(joints.keys()&odoms.keys()&clouds.keys())
        pose_equal=all(np.array_equal(vector(odoms[t]),vector(old[t])) for t in common)
        max_cov=max_point=max_sym=0.;min_eig=1.
        ids_equal=True
        for t in joint_times:
            ids,points,C=joints[t];P=np.array(odoms[t].twist.covariance).reshape(6,6)[:3,:3];cl=clouds[t]
            ids_equal &= np.array_equal(ids,cl[:,0])
            if len(ids):max_point=max(max_point,float(np.max(np.abs(points-cl[:,1:]))))
            max_cov=max(max_cov,float(np.max(np.abs(P-C[:3,:3]))))
            max_sym=max(max_sym,float(np.max(np.abs(C-C.T))))
            min_eig=min(min_eig,float(np.linalg.eigvalsh((C+C.T)*.5).min()))
        runtime=json.loads((a.flight/'runtime.json').read_text())
        checks=dict(reference_complete=runtime['status']=='route_complete' and runtime['stopped'],
            all_odometry_times_equal=bool(common) and set(odoms)==set(old),all_odometry_values_exact=pose_equal,
            covariance_covers_all_odometry=bool(joint_times) and set(joint_times)==set(odoms),
            feature_ids_match=ids_equal,point_coordinates_match=max_point<2e-5,
            velocity_covariance_exact=max_cov==0.,covariance_symmetric=max_sym<1e-9,
            covariance_positive_semidefinite=min_eig>=-1e-9,no_decode_errors=not errors)
        result.update(checks=checks,counts=dict(reference=len(odoms),original=len(old),joint=len(joints),common=len(common)),
                      max_velocity_covariance_error=max_cov,max_point_error_m=max_point,
                      max_symmetry_error=max_sym,min_joint_eigenvalue=min_eig,errors=errors)
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        if proc is not None:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            result['original_exit_code']=proc.returncode
        result['valid']=bool(result.get('checks')) and all(result['checks'].values()) and result.get('original_exit_code')==0
        (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()
