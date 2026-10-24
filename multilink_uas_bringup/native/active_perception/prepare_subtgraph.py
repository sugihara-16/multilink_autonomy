#!/usr/bin/env python3
"""Build Gazebo wrappers without changing benchmark geometry or textures."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from subtgraph_assets import ROOT, REV


def prepare(family, start, end, spawn_z):
    directory = ROOT / family / '01'
    tree = ET.parse('/root/multilink_autonomy_ws/assets/zab/zab_harmonic.sdf')
    world = tree.getroot().find('world')
    # Retain the existing runtime namespace and physics/lighting settings.
    # The provenance and model name identify the actual environment.
    for child in list(world):
        if child.tag in ('model', 'include', 'gui') or (
                child.tag == 'light' and child.get('name') != 'sun'):
            world.remove(child)
    model = ET.SubElement(world, 'model', name='subtgraph_' + family + '_01')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'0 0 0 {math.pi/2:.17g} 0 0'
    link = ET.SubElement(model, 'link', name='environment')
    for tag in ['visual', 'collision']:
        shape = ET.SubElement(link, tag, name=tag)
        mesh = ET.SubElement(ET.SubElement(shape, 'geometry'), 'mesh')
        ET.SubElement(mesh, 'uri').text = str(directory / 'subtgraph.obj')
        ET.SubElement(mesh, 'scale').text = '1 1 1'
    ET.indent(tree, space='  ')
    path = directory / 'world.sdf'
    tree.write(path, encoding='utf-8', xml_declaration=True)
    route = directory / 'route'
    route.mkdir(exist_ok=True)
    delta = np.array(end) - start
    length = float(np.linalg.norm(delta))
    yaw = math.atan2(delta[1], delta[0])
    duration = length / .5 + 4.
    segment = dict(kind='trapezoid', t0=0., duration=duration, start=start,
                   end=end, yaw0=yaw, yaw1=yaw, ramp_s=4., speed_mps=.5)
    (route / 'segments.json').write_text(json.dumps({'segments': [segment]}, indent=2) + '\n')
    (route / 'route.json').write_text(json.dumps(dict(
        waypoints=[start, end], tracking_slow_m=.4, tracking_stop_m=.8,
        selection='Geometry-only selection before estimator measurements. No tuning on results.',
        source_revision=REV), indent=2) + '\n')
    entry = dict(world=str(path), route=str(route), route_end_s=duration,
                 spawn=[*start[:2], spawn_z], length_m=length,
                 source_revision=REV, scale=1., obj_to_world='Rx(+90 degrees)',
                 illumination='ZaB sun + ambient 0.4; removed unrelated ZaB local point lights')
    (directory / 'experiment.json').write_text(json.dumps(entry, indent=2) + '\n')
    return entry


if __name__ == '__main__':
    print(json.dumps(prepare('operational', [0., -510., 39.], [0., -650., 39.], 32.75), indent=2))
