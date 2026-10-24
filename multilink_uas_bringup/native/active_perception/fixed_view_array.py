"""Additional co-located virtual cameras; independent monocular estimators."""
import copy
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

VIEWS = {'yaw_m50': (-50., 0.), 'yaw_p50': (50., 0.),
         'pitch_p90': (0., 90.), 'pitch_m90': (0., -90.)}
OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def orientation(home, yaw, pitch):
    return (home @ Rotation.from_rotvec([0., 0., math.radians(yaw)]).as_matrix()
            @ Rotation.from_rotvec([0., math.radians(pitch), 0.]).as_matrix())


def prepare_array(original, source, output, **kwargs):
    """Preserve mass/physics; publish the model readiness file atomically."""
    output = Path(output)
    base = output/'array_model_base'
    original(source, base, **kwargs)
    for path in base.iterdir():
        if path.name != 'unipilot_gimbal.sdf':
            shutil.copy2(path, output/path.name)
    tree = ET.parse(base/'unipilot_gimbal.sdf')
    body = tree.getroot().find("model/link[@name='base_link']")
    reference = body.find("sensor[@name='fixed_rgb_cam']")
    if reference is None:
        raise ValueError('Array requires paired reference camera')
    mount = json.loads((output/'camera_mount.json').read_text())
    home = Rotation.from_euler('xyz', mount['mount_rpy']).as_matrix()
    geometry = {}
    for name, (yaw, pitch) in VIEWS.items():
        sensor = copy.deepcopy(reference)
        sensor.set('name', name+'_rgb_cam')
        rotation = orientation(home, yaw, pitch)
        rpy = Rotation.from_matrix(rotation).as_euler('xyz')
        sensor.find('pose').text = ' '.join(format(x, '.17g') for x in [*mount['mount_xyz'], *rpy])
        sensor.find('topic').text = f'/view_array/{name}/raw_image'
        sensor.find('gz_frame_id').text = name+'_optical'
        body.append(sensor)
        geometry[name] = dict(yaw_deg=yaw, pitch_deg=pitch, position=mount['mount_xyz'],
                              R_body_camera=rotation.tolist(),
                              R_body_optical=(rotation@OPTICAL).tolist(),
                              raw_topic=sensor.findtext('topic'))
    (output/'fixed_view_geometry.json').write_text(json.dumps(geometry, indent=2)+'\n')
    tmp = output/'unipilot_gimbal.sdf.tmp'
    tree.write(tmp, encoding='unicode', xml_declaration=True)
    tmp.replace(output/'unipilot_gimbal.sdf')
    return output/'unipilot_gimbal.sdf'


def add_bridge_topics(types):
    for name in VIEWS:
        prefix = '/view_array/'+name
        for suffix, typ in [('/active/image', 'sensor_msgs/msg/Image'),
                            ('/active/extrinsic', 'geometry_msgs/msg/PoseStamped'),
                            ('/active/encoder', 'std_msgs/msg/Float64MultiArray')]:
            types[prefix+suffix] = typ


def start_views(out, start, encoder, monitor, rovio_binary, imu_remap,
                launch_path, build, ws, fast_extra, args):
    """Use unchanged binaries/configs, except camera pose and output namespaces."""
    geometry = json.loads((out/'fixed_view_geometry.json').read_text())
    params = yaml.safe_load((out/'rovio_runtime.yaml').read_text())['rovio_gimbal']
    for name in VIEWS:
        prefix = '/view_array/'+name
        directory = out/'views'/name
        directory.mkdir(parents=True)
        (directory/'mimosa').mkdir()
        for filename in ['cam0.yaml', 'raw_cam0.yaml', 'rovio.info', 'mimosa.yaml',
                         'camera_manifest.json', 'protocol.json']:
            shutil.copy2(out/filename, directory/filename)
        camera = json.loads((directory/'camera_manifest.json').read_text())
        transform = np.array(camera['cameras'][0]['T_FC_camera'])
        transform[:3, :3] = geometry[name]['R_body_camera']
        camera['cameras'][0].update(T_FC_camera=transform.tolist(), topic=geometry[name]['raw_topic'])
        (directory/'camera_manifest.json').write_text(json.dumps(camera, indent=2)+'\n')
        # ROVIO's Camera0 parameters describe body->camera (inverse rotation).
        q = Rotation.from_matrix(np.array(geometry[name]['R_body_optical'])).inv().as_quat()
        text = (directory/'rovio.info').read_text()
        for axis, value in zip('xyzw', q):
            text, count = re.subn(rf'(qCM_{axis}\s+)[^;\s]+;',
                                 lambda m: m[1]+format(value, '.17g')+';', text)
            if count != 1:
                raise ValueError('Expected one Camera0 quaternion component')
        (directory/'rovio.info').write_text(text)
        config = yaml.safe_load((directory/'mimosa.yaml').read_text())
        config['logs_directory'] = str(directory/'mimosa')+'/'
        (directory/'mimosa.yaml').write_text(yaml.safe_dump(config))
        protocol = json.loads((directory/'protocol.json').read_text())
        protocol.update(mode='fixed', fixed_angles_deg=list(VIEWS[name]), shared_flight=str(out),
                        independent_monocular=True, initialization='own view from initialization onward')
        (directory/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
        for filename in ['route', 'route_tracking.csv', 'route_status.json']:
            (directory/filename).symlink_to(out/filename, target_is_directory=filename=='route')
        node_name = 'array_'+name+'_rovio'
        runtime = directory/'rovio_runtime.yaml'
        runtime.write_text(yaml.safe_dump({node_name: params}))
        process = start(name+'_parameters', ['rosparam', 'load', runtime], True, True)
        if process.wait(timeout=20):
            raise RuntimeError('Array ROVIO parameter loading failed')
        remaps = [f'{topic}:={prefix}{topic}' for topic in
                  ['/active/image', '/active/extrinsic', '/active/encoder',
                   '/rovio/odometry', '/rovio/pcl', '/slam/odometry']]
        encoder_remaps = [f'/fixed/active/{key}:={prefix}/active/{key}'
                          for key in ['image', 'extrinsic', 'encoder']]
        start(name+'_encoder', [*encoder, '--calibration', directory, '--fixed', '--reliable',
                                '--ros-args', '-r', '__node:=array_'+name+'_encoder',
                                *[v for r in encoder_remaps for v in ['-r', r]]])
        start(name+'_rovio', [rovio_binary, f'_config:={directory}/rovio.info',
                              f'_calibration:={directory}/cam0.yaml',
                              f'_stationary_initialization_duration:={args.stationary_duration}',
                              f'_stationary_mode:={args.stationary_mode}',
                              f'_diagnostic_csv:={directory}/vio_diagnostic.csv',
                              '__name:='+node_name, *remaps, *imu_remap], True)
        launch = ET.parse(launch_path)
        node = launch.getroot().find('node')
        node.set('name', 'array_'+name+'_mimosa')
        for remap in node.findall('remap'):
            if remap.get('to') in ['/slam/odometry', '/slam/map', '/slam/path']:
                remap.set('to', prefix+remap.get('to'))
        for topic in ['/active_lidar_diagnostics', '/active/lidar_information', '/tf', '/tf_static']:
            ET.SubElement(node, 'remap', {'from': topic, 'to': prefix+topic})
        launch.write(directory/'mimosa.launch')
        start(name+'_mimosa', ['roslaunch', directory/'mimosa.launch',
                               f'config:={directory}/mimosa.yaml', f'logs:={directory}/mimosa',
                               'external_odometry:='+prefix+'/rovio/odometry'], True,
              extra=f'export LD_LIBRARY_PATH={build}:{ws}/comparison_ws/deps/lib:${{LD_LIBRARY_PATH:-}}'+fast_extra)
        start(name+'_monitor', [*monitor, '--output', directory,
                                '--truth-topic', '/rmf_unipilot/odom',
                                '__name:=array_'+name+'_monitor', *remaps], True)
