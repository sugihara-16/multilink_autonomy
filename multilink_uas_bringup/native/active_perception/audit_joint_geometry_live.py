#!/usr/bin/env python3
"""Audit an isolated geometry-export ROVIO against the running original inputs.

Only summary diagnostics are saved. No images, point clouds, or joint covariance
stream is recorded. The duplicate never controls the camera or vehicle.
"""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import yaml
from compare_fusion_sidecar import owner_environment
from joint_geometry import decode


def main():
    p=argparse.ArgumentParser();p.add_argument('--flight',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False)
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
        import sensor_msgs.point_cloud2 as pc2
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Float64MultiArray
        rospy.init_node('joint_geometry_audit',disable_signals=True)
        original={};copy={};clouds={};geometry={};errors=[]
        def odom(m,target):target[m.header.stamp.to_nsec()]=m
        def cloud(m):
            names=['id','status','x','y','z','c_00','c_01','c_02','c_11','c_12','c_22']
            rows=[r for r in pc2.read_points(m,field_names=names,skip_nans=True) if r[1]==4]
            clouds[m.header.stamp.to_nsec()]=np.asarray(rows).reshape(-1,11)
        def joint(m):
            try:
                t,ids,p,v,C=decode(m.data);geometry[t]=(ids,p,v,C)
            except Exception as e:errors.append(str(e))
        subs=[rospy.Subscriber('/rovio/odometry',Odometry,lambda m:odom(m,original),queue_size=100),
              rospy.Subscriber('/geometry_audit/odometry',Odometry,lambda m:odom(m,copy),queue_size=100),
              rospy.Subscriber('/rovio/pcl',PointCloud2,cloud,queue_size=100),
              rospy.Subscriber('/geometry_audit/joint_geometry',Float64MultiArray,joint,queue_size=100)]
        proto=json.loads((a.flight/'protocol.json').read_text());config=a.output/'runtime.yaml'
        config.write_text(yaml.safe_dump({'geometry_audit':proto['imu_runtime_parameters']}))
        subprocess.run(['rosparam','load',str(config)],env=env,check=True,stdout=subprocess.DEVNULL)
        cmd=['/root/multilink_autonomy_ws/build/rovio-joint-geometry/rovio_gimbal','__name:=geometry_audit',
             f'_config:={a.flight}/rovio.info',f'_calibration:={a.flight}/cam0.yaml',
             f'_stationary_initialization_duration:={proto["stationary_initialization_duration"]}',f'_stationary_mode:={proto["stationary_mode"]}',
             '/slam/input/imu:=/vio/initialized_imu','/active/image:=/paired/active/image','/active/extrinsic:=/paired/active/extrinsic',
             '/rovio/odometry:=/geometry_audit/odometry','/rovio/pcl:=/geometry_audit/pcl','/tf:=/geometry_audit/tf','/tf_static:=/geometry_audit/tf_static']
        (a.output/'command.json').write_text(json.dumps(cmd,indent=2)+'\n')
        with (a.output/'node.log').open('w') as log:proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        while not (a.flight/'runtime.json').exists():
            if proc.poll() is not None:raise RuntimeError('Geometry exporter exited early')
            if time.monotonic()-start>900:raise RuntimeError('Audit timeout')
            time.sleep(.2)
        for sub in subs:sub.unregister()
        def vector(m):
            p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear;w=m.twist.twist.angular
            return [p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z,*m.pose.covariance,*m.twist.covariance]
        common=sorted(original.keys()&copy.keys());times=sorted(geometry.keys()&original.keys()&clouds.keys())
        checks=dict(all_odometry_times_equal=bool(common) and set(original)==set(copy),
                    all_odometry_values_exact=all(np.array_equal(vector(original[t]),vector(copy[t])) for t in common),
                    geometry_covers_all_odometry=bool(times) and set(times)==set(original),no_decode_errors=not errors)
        ids_ok=points_ok=cov_ok=True;velocity_exact=True;velocity_cov_exact=True;psd=True
        maximum_point_normalized_error=maximum_covariance_normalized_error=0.
        for t in times:
            ids,p,v,C=geometry[t];cl=clouds[t];m=original[t]
            ids_ok&=np.array_equal(ids,cl[:,0]);measured=m.twist.twist.linear
            velocity_exact&=np.array_equal(v,[measured.x,measured.y,measured.z])
            velocity_cov_exact&=np.array_equal(C[:3,:3],np.asarray(m.twist.covariance).reshape(6,6)[:3,:3])
            if len(ids):
                ep=np.max(np.abs(p-cl[:,2:5])/np.maximum(1.,np.abs(p)));maximum_point_normalized_error=max(maximum_point_normalized_error,float(ep));points_ok&=ep<2e-7
                for i in range(len(ids)):
                    cp=C[3+3*i:6+3*i,3+3*i:6+3*i];expected=cp[np.triu_indices(3)]
                    ec=np.max(abs(expected-cl[i,5:])/np.maximum(1.,abs(expected)));maximum_covariance_normalized_error=max(maximum_covariance_normalized_error,float(ec));cov_ok&=ec<2e-7
            psd&=np.linalg.eigvalsh((C+C.T)*.5).min()>=-1e-10*max(1.,np.linalg.norm(C,2))
        runtime=json.loads((a.flight/'runtime.json').read_text())
        checks.update(reference_complete=runtime['stopped'] and runtime['status']=='route_complete',feature_ids_match=bool(ids_ok),
            coordinates_match_official=bool(points_ok),covariance_blocks_match_official=bool(cov_ok),velocity_exact=bool(velocity_exact),
            velocity_covariance_exact=bool(velocity_cov_exact),joint_covariance_positive_semidefinite=bool(psd))
        result.update(checks=checks,counts=dict(original=len(original),duplicate=len(copy),geometry=len(geometry),matched=len(times)),errors=errors,
            maximum_point_normalized_error=maximum_point_normalized_error,maximum_covariance_normalized_error=maximum_covariance_normalized_error)
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        if proc is not None:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            result['duplicate_exit_code']=proc.returncode
        result['valid']=bool(result.get('checks')) and all(result['checks'].values()) and result.get('duplicate_exit_code')==0
        (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)


if __name__=='__main__':main()
