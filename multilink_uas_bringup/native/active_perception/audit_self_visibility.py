#!/usr/bin/env python3
"""Project known robot geometry into a saved camera view, without sensor truth.

The body mesh is exact; rotors use conservative swept cylinders. This computes
a geometric visibility mask, not an edited camera image or a SLAM observation.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import trimesh
import yaml
from scipy.spatial.transform import Rotation
from core import camera_rotation


def transform(text):
    values = np.array([float(x) for x in (text or '0 0 0 0 0 0').split()])
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    T[:3, 3] = values[:3]
    return T


def near_clip(triangle, near):
    result = []
    previous = triangle[-1]
    for current in triangle:
        p_inside, c_inside = previous[2]>=near, current[2]>=near
        if p_inside != c_inside:
            result.append(previous+(current-previous)*(near-previous[2])/(current[2]-previous[2]))
        if c_inside:
            result.append(current)
        previous = current
    return np.asarray(result)


def project(vertices, faces, R, position, K, near, width, height):
    optical = (vertices-position)@R
    triangles = optical[faces]
    inside = triangles[:, :, 2]>=near
    polygons = list(triangles[inside.all(axis=1)])
    polygons.extend(near_clip(t, near) for t in triangles[inside.any(axis=1)&~inside.all(axis=1)])
    image_polygons = []
    for polygon in polygons:
        if len(polygon)<3:
            continue
        pixel = polygon@K.T
        pixel = pixel[:, :2]/pixel[:, 2:]
        if (pixel[:, 0].max()<0 or pixel[:, 0].min()>=width
                or pixel[:, 1].max()<0 or pixel[:, 1].min()>=height):
            continue
        image_polygons.append(np.rint(pixel).astype(np.int32))
    mask = np.zeros((height, width), np.uint8)
    if image_polygons:
        # Fill independently: batching polygons uses an even/odd hole rule
        # on overlapping triangles, which is incorrect for a silhouette.
        for polygon in image_polygons:
            cv2.fillConvexPoly(mask, polygon, 255)
    return mask


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--flight', required=True, type=Path)
    parser.add_argument('--snapshot', required=True, type=Path)
    args = parser.parse_args()
    info = json.loads((args.snapshot/'snapshot.json').read_text())
    stamp, yaw, pitch = info['matched_encoder_sample']
    model = ET.parse(args.flight/'unipilot_gimbal.sdf').getroot().find('model')
    camera = json.loads((args.flight/'camera_manifest.json').read_text())['cameras'][0]
    T = np.array(camera['T_FC_camera'])
    K = np.array(yaml.safe_load((args.flight/'cam0.yaml').read_text())['camera_matrix']['data']).reshape(3, 3)
    width, height = info['active']['width'], info['active']['height']
    near = float(model.findtext(".//sensor[@name='rgb_cam']/camera/clip/near"))
    pieces = []
    for link in model.findall('link'):
        if link.get('name').startswith('active_camera'):
            continue  # Housing is entirely behind its own optical center.
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            mesh = geometry.find('mesh')
            kind = 'body'
            if mesh is not None:
                part = trimesh.load(mesh.findtext('uri'), force='mesh')
                part.apply_scale([float(x) for x in mesh.findtext('scale', '1 1 1').split()])
                if link.get('name').startswith('rotor_'):
                    radius = np.linalg.norm(part.vertices[:, :2], axis=1).max()
                    low, high = part.bounds[:, 2]
                    part = trimesh.creation.cylinder(radius=radius, height=high-low, sections=64)
                    part.apply_translation([0, 0, (low+high)/2])
                    kind = 'rotor_swept_envelope'
            elif geometry.find('cylinder') is not None:
                part = trimesh.creation.cylinder(radius=float(geometry.findtext('cylinder/radius')),
                                                 height=float(geometry.findtext('cylinder/length')), sections=64)
            else:
                raise ValueError('Unsupported non-camera visual geometry')
            part.apply_transform(transform(link.findtext('pose'))@transform(visual.findtext('pose')))
            pieces.append((link.get('name')+'/'+visual.get('name'), kind, part))
    result = dict(scope=__doc__, image_stamp_ns=info['stamp_ns'], encoder_stamp_s=stamp,
                  angles_deg=np.rad2deg([yaw, pitch]).tolist(), views={})
    for view, angles in [('fixed', [0., 0.]), ('active', [yaw, pitch])]:
        R = camera_rotation(T[:3, :3], *angles)
        total = np.zeros((height, width), np.uint8)
        result['views'][view] = {}
        for name, kind, part in pieces:
            mask = project(part.vertices, part.faces, R, T[:3, 3], K, near, width, height)
            count = int(np.count_nonzero(mask))
            result['views'][view][name] = dict(kind=kind, pixels=count, fraction=count/(width*height))
            total = np.maximum(total, mask)
        cv2.imwrite(str(args.snapshot/(view+'_self_mask.png')), total)
        result['views'][view]['union_fraction'] = float(np.count_nonzero(total)/(width*height))
    (args.snapshot/'self_visibility.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
