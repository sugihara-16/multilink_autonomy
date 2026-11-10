"""Read-only near-robot return statistics; never changes estimator inputs."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
import yaml

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--min-time',type=float,default=50.);a=p.parse_args()
    rospy.init_node('lidar_clearance_audit',anonymous=True)
    pose=yaml.safe_load((a.output/'mimosa.yaml').read_text())['lidar']['T_B_S']
    R=Rotation.from_quat(pose[3:]).as_matrix();samples=[];deadline=time.monotonic()+120
    while len(samples)<10 and time.monotonic()<deadline:
        m=rospy.wait_for_message('/dragon/lidar/points',PointCloud2,timeout=15)
        if m.header.stamp.to_sec()<a.min_time:continue
        f={x.name:x for x in m.fields};order='>' if m.is_bigendian else '<'
        dt=np.dtype(dict(names=['x','y','z'],formats=[order+'f4']*3,offsets=[f[k].offset for k in ('x','y','z')],itemsize=m.point_step))
        raw=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step))
        q=np.column_stack([raw[k].ravel() for k in ('x','y','z')]);q=q[np.isfinite(q).all(axis=1)]
        radius=np.linalg.norm(q,axis=1);body=q@R.T+pose[:3]
        samples.append(dict(stamp_ns=m.header.stamp.to_nsec(),finite=len(q),below_point_filter=int((radius<.8).sum()),
                            retained=int(((radius>=.8)&(radius<=30)).sum()),
                            retained_near_body_height=int(((radius>=.8)&(radius<1.5)&(abs(body[:,2])<.3)).sum())))
    if len(samples)!=10:raise RuntimeError('Incomplete clearance audit')
    result=dict(samples=samples,median_near_fraction=float(np.median([s['below_point_filter']/s['finite'] for s in samples])),
                median_retained_points=float(np.median([s['retained'] for s in samples])),
                scope='At hover/preparation in the known wide tunnel. Counts describe visibility, not injected filtering or ground-truth estimator correction.')
    (a.output/'lidar_clearance.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
