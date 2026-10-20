#!/usr/bin/env python3
"""Read-only, one-Hz summaries of sparse ROVIO feature geometry.

No images, point coordinates, or truth are saved or fed back. Angular scatter
describes spatial distribution; it is not an accuracy or observability proof.
"""
import argparse
import csv
import json
from pathlib import Path
import time
import threading
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


def main():
    p = argparse.ArgumentParser()
    p.add_argument('flight', type=Path)
    p.add_argument('--wall-limit', type=float, default=1800.)
    a = p.parse_args()
    output = a.flight/'feature_spread.csv'
    if output.exists():raise ValueError('Preserve existing observer output')
    mount = json.loads((a.flight/'camera_manifest.json').read_text())
    tbc = np.array(mount['cameras'][0]['T_FC_camera'])[:3, 3]
    lock = threading.Lock();history = {m:{} for m in ['fixed','active']}
    last = {m:-np.inf for m in history};counts = {m:0 for m in history}
    rospy.init_node('readonly_feature_spread', anonymous=True, disable_signals=True)
    with output.open('w', buffering=1) as f:
        writer = csv.writer(f)
        writer.writerow(['stamp_ns','mode','tracked','mature','median_depth_m','median_relative_sigma',
                         'scatter_min','scatter_middle','scatter_max'])
        def callback(msg, mode):
            stamp = msg.header.stamp.to_sec()
            rows = [r for r in point_cloud2.read_points(msg,
                field_names=['id','status','x','y','z','c_00','c_01','c_02','c_11','c_12','c_22'],
                skip_nans=True) if r[1] == 4]
            with lock:
                h = history[mode]
                for row in rows:
                    ident = int(row[0]);born, seen = h.get(ident, (stamp,stamp))
                    h[ident] = (stamp if stamp-seen>.25 else born, stamp)
                history[mode] = {i:v for i,v in h.items() if stamp-v[1]<=.25}
                if stamp-last[mode] < 1.-1e-8:return
                last[mode] = stamp;counts[mode] += 1
                if not rows:
                    writer.writerow([msg.header.stamp.to_nsec(),mode,0,0,*([float('nan')]*5)])
                    return
                rays = np.array([r[2:5] for r in rows])-tbc
                ranges = np.linalg.norm(rays,axis=1)
                unit = rays/np.maximum(ranges[:,None],1e-12)
                cov = np.array([[[r[5],r[6],r[7]],[r[6],r[8],r[9]],[r[7],r[9],r[10]]] for r in rows])
                relative = np.sqrt(np.maximum(0.,np.einsum('ni,nij,nj->n',unit,cov,unit)))/np.maximum(ranges,1e-12)
                age = np.array([stamp-h[int(r[0])][0] for r in rows])
                mature = (age>=1.)&(relative<=.3)&(ranges>.3)
                scatter = np.linalg.eigvalsh(unit.T@unit/len(unit))
                writer.writerow([msg.header.stamp.to_nsec(),mode,len(rows),int(mature.sum()),
                                 float(np.median(ranges)),float(np.median(relative)),*scatter.tolist()])
        subs = [rospy.Subscriber(topic,PointCloud2,callback,callback_args=mode,queue_size=1)
                for mode,topic in [('fixed','/fixed/rovio/pcl'),('active','/rovio/pcl')]]
        start = time.monotonic();reason = 'wall_limit'
        try:
            while time.monotonic()-start<a.wall_limit and not rospy.is_shutdown():
                if (a.flight/'runtime.json').exists():reason='owner_stopped';break
                time.sleep(.2)
        finally:
            for sub in subs:sub.unregister()
            rospy.signal_shutdown('Observer stopped')
            with lock:
                (a.flight/'feature_spread_observer.json').write_text(json.dumps(
                    dict(scope=__doc__,reason=reason,samples=counts,truth_input=False,
                         image_or_point_recording=False),indent=2)+'\n')


if __name__ == '__main__':main()
