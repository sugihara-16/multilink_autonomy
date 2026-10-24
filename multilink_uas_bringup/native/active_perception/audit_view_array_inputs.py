#!/usr/bin/env python3
"""Read-only image/extrinsic audit; one illustrative frame per fixed direction."""
import argparse
import hashlib
import json
from pathlib import Path
import threading
import numpy as np
import cv2
import rospy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from fixed_view_array import VIEWS, OPTICAL


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('array_input_audit')
    out = args.output
    geometry = json.loads((out/'fixed_view_geometry.json').read_text())
    mount = json.loads((out/'camera_mount.json').read_text())
    geometry['front'] = dict(
        R_body_optical=(Rotation.from_euler('xyz', mount['mount_rpy']).as_matrix()@OPTICAL).tolist(),
        position=mount['mount_xyz'])
    lock = threading.RLock()
    report = {name: dict(image_stamps=[], extrinsic_stamps=[], rotation_error_max_rad=0.,
                         position_error_max_m=0., image_sha256=None, snapshot=None) for name in geometry}
    hashes = {name: hashlib.sha256() for name in geometry}
    snapshots = out/'view_snapshots'
    snapshots.mkdir()
    tracking = False
    def image(msg, name):
        with lock:
            r = report[name]
            r['image_stamps'].append(msg.header.stamp.to_nsec())
            hashes[name].update(bytes(msg.data))
            if tracking and r['snapshot'] is None:
                if msg.encoding != 'mono8':
                    raise RuntimeError('Expected existing adapter mono8 output')
                pixels = np.ndarray((msg.height, msg.width), np.uint8, buffer=bytes(msg.data),
                                    strides=(msg.step, 1)).copy()
                path = snapshots/(name+'.png')
                if not cv2.imwrite(str(path), pixels):
                    raise RuntimeError('Cannot save example image')
                r['snapshot'] = dict(path=str(path), stamp_ns=msg.header.stamp.to_nsec())
    def extrinsic(msg, name):
        p, q = msg.pose.position, msg.pose.orientation
        actual = Rotation.from_quat([q.x, q.y, q.z, q.w])
        expected = Rotation.from_matrix(geometry[name]['R_body_optical'])
        with lock:
            r = report[name]
            r['extrinsic_stamps'].append(msg.header.stamp.to_nsec())
            r['rotation_error_max_rad'] = max(r['rotation_error_max_rad'], float((expected.inv()*actual).magnitude()))
            r['position_error_max_m'] = max(r['position_error_max_m'],
                float(np.linalg.norm(np.array([p.x, p.y, p.z])-geometry[name]['position'])))
    subs = []
    for name in geometry:
        prefix = '/fixed' if name == 'front' else '/view_array/'+name
        subs.extend([
            rospy.Subscriber(prefix+'/active/image', Image, lambda m, n=name: image(m, n),
                             queue_size=100, buff_size=20000000, tcp_nodelay=True),
            rospy.Subscriber(prefix+'/active/extrinsic', PoseStamped, lambda m, n=name: extrinsic(m, n),
                             queue_size=100, tcp_nodelay=True)])
    def save(_=None):
        nonlocal tracking
        try:
            status = json.loads((out/'route_status.json').read_text())
            tracking = status['stage'] == 'tracking' and status['route_time_s'] >= 3.
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        with lock:
            for name in report:
                report[name]['image_sha256'] = hashes[name].hexdigest()
            tmp = out/'array_input_audit.tmp'
            tmp.write_text(json.dumps(report, indent=2)+'\n')
            tmp.replace(out/'array_input_audit.json')
    timer = rospy.Timer(rospy.Duration(1), save)
    rospy.on_shutdown(save)
    rospy.spin()


if __name__ == '__main__':
    main()
