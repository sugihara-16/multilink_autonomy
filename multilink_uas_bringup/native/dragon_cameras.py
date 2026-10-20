"""Three identical outward cameras and fixed-square FC-relative calibration."""
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

LINKS = ('link1', 'link3', 'link4')
# Optical center on the outer side of the link, level with its axis.
# Housing extends inward by 25 mm; a short saddle joins it to the link tube.
MOUNT_XYZ = (.10, -.045, 0.)
MOUNT_RPY = (0., 0., math.radians(-127.5))
OPTICAL = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    t[:3, 3] = xyz
    return t


def link_transforms(root, angles):
    """URDF forward kinematics; fixed origins followed by joint rotations."""
    frames = {'root': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            origin = joint.find('origin')
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            t = transform(xyz, rpy)
            if joint.get('type') in ('revolute', 'continuous'):
                axis = np.array([float(x) for x in joint.find('axis').get('xyz').split()])
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis*angles.get(joint.get('name'), 0.)).as_matrix()
                t = t @ rotation
            frames[joint.find('child').get('link')] = frames[parent] @ t
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError('Disconnected URDF joint tree')
    return frames


def add_cameras(root, output, gimbal=False):
    output = Path(output)
    angles = {}
    for joint in root.findall('./ros2_control/joint'):
        initial = joint.find("state_interface[@name='position']/param[@name='initial_value']")
        if initial is not None:
            angles[joint.get('name')] = float(initial.text)
    expected = {f'joint{i}_{axis}': angles[f'joint{i}_{axis}'] for i in (1, 2, 3) for axis in ('pitch', 'yaw')}
    frames = link_transforms(root, angles)
    fc_inv = np.linalg.inv(frames['fc'])
    template = Path('/root/unified_autonomy_stack/workspaces/robot_bringup/config/ros1/rovio.info').read_text()
    template = re.sub(r'Camera\d+\s*\{[^}]*\}', '', template)
    template = template.replace('doFrameVisualisation true', 'doFrameVisualisation false')
    cameras = []
    center = np.mean([(frames[f'link{i}'] @ np.array([.263, 0, 0, 1]))[:3] for i in range(1, 5)], axis=0)
    for i, link in enumerate(('link3',) if gimbal else LINKS):
        name = f'cam_{link}'
        body_camera = frames[link] @ transform(MOUNT_XYZ, MOUNT_RPY)
        forward = body_camera[:3, 0]
        outward_dot = float(np.dot(forward[:2], (body_camera[:3, 3]-center)[:2]))
        if outward_dot <= 0:
            raise ValueError(f'{link} camera points inward')
        fc_camera = fc_inv @ body_camera
        q = Rotation.from_matrix((fc_camera[:3, :3] @ OPTICAL).T).as_quat()
        template += f'\nCamera{i}\n{{\n  CalibrationFile\n'
        template += ''.join(f'  qCM_{k} {v:.15g};\n' for k, v in zip('xyzw', q))
        template += ''.join(f'  MrMC_{k} {v:.15g};\n' for k, v in zip('xyz', fc_camera[:3, 3])) + '}\n'
        for prefix, width, height in [('raw_', 720, 540), ('', 480, 360)]:
            fx = width/(2*math.tan(math.radians(120)/2))
            cfg = dict(image_width=width, image_height=height, camera_name=name,
                       camera_matrix=dict(rows=3, cols=3, data=[fx,0,width/2,0,fx,height/2,0,0,1]),
                       distortion_model='plumb_bob', distortion_coefficients=dict(rows=1,cols=5,data=[0.]*5))
            (output/f'{prefix}cam{i}.yaml').write_text(yaml.safe_dump(cfg))
        sensor = ET.SubElement(ET.SubElement(root, 'gazebo', reference=link), 'sensor', name=name, type='camera')
        for key, value in [('pose', ' '.join(map(str, MOUNT_XYZ+MOUNT_RPY))), ('always_on', 'true'),
                           ('update_rate', '20'), ('topic', f'/dragon/{name}/image'), ('gz_frame_id', f'{name}_optical')]:
            ET.SubElement(sensor, key).text = value
        c = ET.SubElement(sensor, 'camera')
        ET.SubElement(c, 'horizontal_fov').text = str(math.radians(120))
        im = ET.SubElement(c, 'image')
        for key, value in [('width','720'), ('height','540'), ('format','R8G8B8')]:
            ET.SubElement(im, key).text = value
        clip = ET.SubElement(c, 'clip')
        ET.SubElement(clip, 'near').text = '.04'
        ET.SubElement(clip, 'far').text = '60'
        # Visible ideal sensor housing; no added payload dynamics assumed.
        visual = ET.SubElement(root.find(f"link[@name='{link}']"), 'visual', name=name)
        housing_xyz = np.array(MOUNT_XYZ) - .0125*transform(rpy=MOUNT_RPY)[:3,0]
        ET.SubElement(visual, 'origin', xyz=' '.join(map(str, housing_xyz)), rpy=' '.join(map(str, MOUNT_RPY)))
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size='.025 .03 .03')
        ET.SubElement(ET.SubElement(visual, 'material', name=name+'_material'), 'color', rgba='0.1 0.3 0.9 1')
        bracket = ET.SubElement(root.find(f"link[@name='{link}']"), 'visual', name=name+'_bracket')
        ET.SubElement(bracket, 'origin', xyz=f'{MOUNT_XYZ[0]} -.017 0')
        ET.SubElement(ET.SubElement(bracket, 'geometry'), 'box', size='.03 .014 .012')
        ET.SubElement(ET.SubElement(bracket, 'material', name=name+'_bracket_material'), 'color', rgba='.25 .25 .25 1')
        ET.SubElement(root, 'link', name=name+'_optical')
        joint = ET.SubElement(root, 'joint', name=name+'_mount', type='fixed')
        ET.SubElement(joint, 'parent', link=link)
        ET.SubElement(joint, 'child', link=name+'_optical')
        optical_rpy = Rotation.from_matrix(transform(rpy=MOUNT_RPY)[:3,:3] @ OPTICAL).as_euler('xyz')
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, MOUNT_XYZ)), rpy=' '.join(map(str,optical_rpy)))
        cameras.append(dict(id=i, link=link, topic=f'/dragon/{name}/image', mount_xyz=MOUNT_XYZ,
            mount_rpy=MOUNT_RPY, T_FC_camera=fc_camera.tolist(), qCM=q.tolist(),
            MrMC=fc_camera[:3,3].tolist(), outward_dot=outward_dot, resolution=[720,540],
            rovio_resolution=[480,360], horizontal_fov_deg=120))
        if gimbal:
            # Ideal two-axis gimbal: axes intersect at the optical centre.
            # Body remains in the calibrated fixed-square configuration.
            # Several gazebo elements can have the same reference.
            g = next(e for e in root.findall('gazebo') if sensor in list(e))
            g.remove(sensor)
            sensor.find('pose').text = '0 0 0 0 0 0'
            ET.SubElement(root,'gazebo',reference='active_camera_pitch').append(sensor)
            for axis in ('yaw','pitch'):
                body=ET.SubElement(root,'link',name='active_camera_'+axis)
                inertia=ET.SubElement(body,'inertial')
                ET.SubElement(inertia,'mass',value='.002')
                ET.SubElement(inertia,'inertia',ixx='0.000001',iyy='0.000001',izz='0.000001',ixy='0',ixz='0',iyz='0')
                j=ET.SubElement(root,'joint',name='active_camera_'+axis+'_joint',type='revolute')
                ET.SubElement(j,'parent',link=link if axis=='yaw' else 'active_camera_yaw')
                ET.SubElement(j,'child',link='active_camera_'+axis)
                ET.SubElement(j,'origin',xyz=' '.join(map(str,MOUNT_XYZ)) if axis=='yaw' else '0 0 0',
                              rpy=' '.join(map(str,MOUNT_RPY)) if axis=='yaw' else '0 0 0')
                ET.SubElement(j,'axis',xyz='0 0 1' if axis=='yaw' else '0 1 0')
                ET.SubElement(j,'limit',lower='-2.0943951' if axis=='yaw' else '-.5235988',
                              upper='2.0943951' if axis=='yaw' else '.5235988',effort='.05',velocity='.34906585')
                plugin=ET.SubElement(ET.SubElement(root,'gazebo'),'plugin',
                    filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
                for k,v in [('joint_name','active_camera_'+axis+'_joint'),('topic','/active/gimbal/'+axis),
                            ('use_velocity_commands','true'),('cmd_max','.2468268299')]:
                    ET.SubElement(plugin,k).text=v
            root.find(f"link[@name='{link}']").remove(visual)
            root.find("link[@name='active_camera_pitch']").append(visual)
            visual.find('origin').set('xyz','-.0125 0 0');visual.find('origin').set('rpy','0 0 0')
            joint.find('parent').set('link','active_camera_pitch')
            joint.find('origin').set('xyz','0 0 0')
            joint.find('origin').set('rpy',' '.join(map(str,Rotation.from_matrix(OPTICAL).as_euler('xyz'))))
            plugin=ET.SubElement(ET.SubElement(root,'gazebo'),'plugin',filename='gz-sim-joint-state-publisher-system',
                                 name='gz::sim::systems::JointStatePublisher')
            ET.SubElement(plugin,'topic').text='/active/gimbal/states'
            for axis in ('yaw','pitch'): ET.SubElement(plugin,'joint_name').text='active_camera_'+axis+'_joint'
            cameras[-1]['gimbal']=True
    (output/'rovio.info').write_text(template)
    (output/'camera_manifest.json').write_text(json.dumps(dict(cameras=cameras, expected_joints=expected,
        calibration='Fixed-square body; home camera rotation overridden by exposure-time encoders' if gimbal else
                    'Nominal simulation mounts; fixed square only', imu_frame='fc'), indent=2)+'\n')
