#!/usr/bin/env python3
"""Run a foreground, bounded UniPilot check of the imported tunnel."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from run_lio_failure_comparisons import INPUTS, NATIVE, run_one, verify_selected

ASSET=NATIVE.parents[3]/'assets/futatsugoya'

def observe(output):
    import numpy as np
    from PIL import Image
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image as RosImage, PointCloud2
    from sensor_msgs_py import point_cloud2
    rclpy.init();node=rclpy.create_node('futatsugoya_snapshot');done={}
    def ready():
        p=output/'campaign/smoke_active/route_status.json'
        try:d=json.loads(p.read_text());return d.get('stage')=='tracking' and d.get('route_time_s',0)>5
        except (FileNotFoundError,json.JSONDecodeError):return False
    def image(m,key):
        if key in done or not ready():return
        assert m.encoding in ('rgb8','bgr8','mono8'),m.encoding
        channels=1 if m.encoding=='mono8' else 3
        a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*channels].reshape(m.height,m.width,channels)
        if m.encoding=='bgr8':a=a[:,:,::-1]
        if channels==1:a=a[:,:,0]
        Image.fromarray(a).save(output/(key+'.png'))
        done[key]=dict(stamp_sec=m.header.stamp.sec,stamp_nanosec=m.header.stamp.nanosec,width=m.width,height=m.height,encoding=m.encoding,mean=float(a.mean()),std=float(a.std()))
    def points(m):
        if 'lidar' in done or not ready():return
        raw=point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True);a=np.column_stack([raw[k] for k in ('x','y','z')])
        a=a[np.isfinite(a).all(axis=1)];r=np.linalg.norm(a,axis=1);a=a[r>0];r=r[r>0]
        done['lidar']=dict(stamp_sec=m.header.stamp.sec,stamp_nanosec=m.header.stamp.nanosec,points=len(r),frame_id=m.header.frame_id, xyz_bounds=[np.nanmin(a,axis=0).tolist(),np.nanmax(a,axis=0).tolist()],range_percentiles_m=np.percentile(r,[0,10,50,90,100]).tolist())
    node.create_subscription(RosImage,'/rmf_unipilot/cam_gimbal/image',lambda m:image(m,'active_camera'),qos_profile_sensor_data)
    node.create_subscription(RosImage,'/rmf_unipilot/cam_fixed/image',lambda m:image(m,'fixed_camera'),qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/rmf_unipilot/lidar/points',points,qos_profile_sensor_data)
    deadline=time.monotonic()+240
    try:
        while len(done)<3 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
    finally:
        (output/'sensor_snapshots.json').write_text(json.dumps(done,indent=2)+'\n');node.destroy_node();rclpy.shutdown()
    if len(done)!=3:raise RuntimeError('Incomplete sensor snapshots')

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--route',choices=['smoke','interior'],default='smoke');p.add_argument('--prepare-only',action='store_true');p.add_argument('--observe',action='store_true');p.add_argument('--view-array',action='store_true',help='Compare five independent fixed views and the active camera in one flight');args=p.parse_args()
    if args.observe:observe(args.output);return
    root=args.output.resolve()
    if root.exists():raise RuntimeError('Use a new output directory')
    root.mkdir(parents=True);entries=json.loads((ASSET/'routes.json').read_text());entry=entries[args.route]
    template=json.loads((INPUTS/'templates/paired_flight.json').read_text())
    extension_checks={}
    if args.view_array:
        template[1]=str(Path(__file__).with_name('run_view_array.py'))
        extension_paths=[Path(__file__).with_name(n) for n in ['fixed_view_array.py','run_view_array.py','audit_view_array_inputs.py','report_view_array.py']] + [NATIVE/'unipilot_view_array.launch.py']
        extension_checks={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in extension_paths}
        (root/'array_source_checks.json').write_text(json.dumps(extension_checks,indent=2)+'\n')
    asset_checks={str(p.relative_to(ASSET)):hashlib.sha256(p.read_bytes()).hexdigest() for p in ASSET.rglob('*') if p.is_file()};(root/'asset_checks.json').write_text(json.dumps(asset_checks,indent=2)+'\n')
    checks=verify_selected();(root/'source_checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    (root/'entry.json').write_text(json.dumps(entry,indent=2)+'\n')
    if args.prepare_only:print(json.dumps(entry,indent=2));return
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    snapshot=None
    try:
        # Snapshot observer has no control or estimator subscriptions/publications.
        if args.route=='smoke':
            env=os.environ.copy();env['SNAPSHOT_SCRIPT']=str(Path(__file__).resolve());env['SNAPSHOT_OUTPUT']=str(root/'smoke')
            with (root/'snapshots.log').open('w') as f:
                snapshot=subprocess.Popen(['bash','-c','source /root/uas_native/env.sh; export ROS_DOMAIN_ID=85; exec /usr/bin/python3 "$SNAPSHOT_SCRIPT" --observe --output "$SNAPSHOT_OUTPUT"'],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        result=run_one(root,args.route,entry,template,20261021)
        if snapshot and snapshot.wait(timeout=20)!=0:raise RuntimeError('Sensor snapshot failed; flight evidence retained')
        if args.view_array:
            subprocess.run([sys.executable,str(Path(__file__).with_name('report_view_array.py')),'--trial',str(root/args.route)],check=True)
            array=json.loads((root/args.route/'array_comparison.json').read_text())
            result.update(array_valid=array['valid'],array_metrics=array['metrics'])
            if not all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in extension_checks.items()):
                raise RuntimeError('Array implementation changed during flight')
            if not array['valid']:
                (root/'result.json').write_text(json.dumps(result,indent=2)+'\n')
                raise RuntimeError('Array audit failed; evidence retained')
        (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    finally:
        if snapshot and snapshot.poll() is None:
            os.killpg(snapshot.pid,signal.SIGINT)
            try:snapshot.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(snapshot.pid,signal.SIGKILL);snapshot.wait()

if __name__=='__main__':main()
