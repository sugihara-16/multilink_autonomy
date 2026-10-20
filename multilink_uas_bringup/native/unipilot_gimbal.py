"""Derive a single gimballed front camera from the UAS UniPilot SDF."""
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

# Relative to UniPilot base_link, metres and radians. Both axes intersect here.
MOUNT_XYZ = (0.180, 0.0, -0.025)
MOUNT_RPY = (0.0, 0.0, 0.0)
GIMBAL_LIMITS_DEG = {'yaw': 50.0, 'pitch': 90.0}
CAMERA_RATE = 20  # Original disabled preview camera is only 1 Hz.


def prepare(source, output, paired=False, camera_far=None, sensor_profile="native", diagnostic_sensors=False, imu_sampling="resolved", diagnostic_imu_only=False):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    text = source.read_text()
    root = ET.fromstring(text)
    model = root.find('model')
    body = model.find("link[@name='base_link']")
    imu = ET.fromstring(re.search(r'<sensor name="imu" type="imu">.*?</sensor>', text, re.S).group())
    ET.SubElement(imu, 'topic').text = '/rmf_unipilot/imu'
    ET.SubElement(imu, 'gz_frame_id').text = 'rmf_unipilot/base_link'
    body.append(imu)
    lidar = body.find("sensor[@name='lidar']")
    ET.SubElement(lidar, 'topic').text = '/rmf_unipilot/lidar/scan'
    ET.SubElement(lidar, 'gz_frame_id').text = 'rmf_unipilot/lidar'
    # Two upstream controllers publish to the same motors. Only the standard
    # velocity controller owns them in this manual inspection mode. Retain
    # the vertical gain used by the existing UniPilot flight demo.
    for plugin in list(model.findall('plugin')):
        if 'AccelerationControl' in plugin.get('name', ''):
            model.remove(plugin)
        elif 'MulticopterVelocityControl' in plugin.get('name', ''):
            plugin.find('velocityGain').text = '4 4 10'
    camera = ET.fromstring(re.search(r'<sensor name="rgb_cam" type="camera">.*?</sensor>', text, re.S).group())
    if sensor_profile not in ('native', 'dragon'):
        raise ValueError('Unknown sensor profile: '+sensor_profile)
    profile = None
    if sensor_profile == 'dragon':
        profile = ET.parse(Path(__file__).parent/'sensor_profiles/dragon.xml').getroot()
        for sensor, kind in ((imu,'imu'),(lidar,'lidar'),(camera,'camera')):
            reference = profile.find(f"sensor[@name='{kind}']")
            for tag in ('update_rate',kind):
                old = sensor.find(tag)
                if old is not None: sensor.remove(old)
                sensor.append(ET.fromstring(ET.tostring(reference.find(tag))))
        (output/'sensor_profile.xml').write_text(ET.tostring(profile,encoding='unicode'))
    if imu_sampling not in ('resolved', 'legacy'):
        raise ValueError('Unknown IMU sampling mode')
    original_rate = float(imu.findtext('update_rate'))
    resolved_rate = max(original_rate, 1000.) if imu_sampling == 'resolved' else original_rate
    # Preserve continuous-time white-noise density, rather than silently
    # improving the sensor by taking five times as many equally noisy samples.
    # Bias distribution, driving noise and correlation times stay unchanged.
    for noise in imu.findall('.//noise'):
        for tag in ('stddev', 'precision'):
            value = noise.find(tag)
            if value is not None:
                value.text = str(float(value.text)*math.sqrt(resolved_rate/original_rate))
    imu.find('update_rate').text = str(resolved_rate)
    (output/'imu_sampling.json').write_text(json.dumps(dict(mode=imu_sampling,original_rate_hz=original_rate,rate_hz=resolved_rate,white_noise_density_preserved=True,bias_parameters_preserved=True),indent=2)+'\n')
    camera.find('pose').text = '0 0 0 0 0 0'
    camera.find('update_rate').text = str(CAMERA_RATE)
    if camera_far is not None:
        if not math.isfinite(camera_far) or camera_far<=float(camera.findtext('camera/clip/near')):
            raise ValueError('Camera far clip must be finite and beyond the near clip')
        camera.find('camera/clip/far').text=str(camera_far)
    ET.SubElement(camera, 'topic').text = '/rmf_unipilot/cam_gimbal/image'
    ET.SubElement(camera, 'gz_frame_id').text = 'rmf_unipilot/cam_gimbal_optical'
    diagnostics = []
    if diagnostic_sensors:
        clean_imu = ET.fromstring(ET.tostring(imu))
        clean_imu.set('name','diagnostic_clean_imu')
        clean_imu.find('topic').text='/diagnostic/imu_clean'
        for parent in clean_imu.iter():
            for child in list(parent):
                if child.tag=='noise':parent.remove(child)
        body.append(clean_imu)
        fast_imu = ET.fromstring(ET.tostring(clean_imu))
        fast_imu.set('name','diagnostic_fast_imu')
        fast_imu.find('topic').text='/diagnostic/imu_fast'
        fast_imu.find('update_rate').text='1000'
        body.append(fast_imu)
        for name in (() if diagnostic_imu_only else ('clean','wide','far')):
            diagnostic = ET.fromstring(ET.tostring(camera))
            diagnostic.set('name','diagnostic_'+name+'_cam')
            diagnostic.find('topic').text='/diagnostic/image_'+name
            noise=diagnostic.find('camera/noise')
            if noise is not None:diagnostic.find('camera').remove(noise)
            if name=='wide':
                width=int(camera.findtext('camera/image/width'))
                focal=width/(2*math.tan(float(camera.findtext('camera/horizontal_fov'))/2))
                # Keep angular pixel resolution and vertical field of view.
                diagnostic.find('camera/image/width').text=str(width+400)
                diagnostic.find('camera/horizontal_fov').text=str(2*math.atan((width+400)/(2*focal)))
            if name=='far':diagnostic.find('camera/clip/far').text='60'
            diagnostics.append(diagnostic)
    if paired:
        # Counterfactual fixed view of the SAME physical flight. This is a
        # virtual sensor only: no duplicate housing, mass or collision shape.
        fixed = ET.fromstring(ET.tostring(camera))
        fixed.set('name', 'fixed_rgb_cam')
        fixed.find('pose').text = ' '.join(map(str, MOUNT_XYZ + MOUNT_RPY))
        fixed.find('topic').text = '/rmf_unipilot/cam_fixed/image'
        fixed.find('gz_frame_id').text = 'rmf_unipilot/cam_fixed_optical'
        body.append(fixed)
    for axis, limit in GIMBAL_LIMITS_DEG.items():
        name = 'active_camera_' + axis
        link = ET.SubElement(model, 'link', name=name)
        ET.SubElement(link, 'pose', relative_to='base_link' if axis == 'yaw' else 'active_camera_yaw').text = (
            ' '.join(map(str, MOUNT_XYZ + MOUNT_RPY)) if axis == 'yaw' else '0 0 0 0 0 0')
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '.002'
        inertia = ET.SubElement(inertial, 'inertia')
        for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
            ET.SubElement(inertia, key).text = '.000001' if key in ('ixx','iyy','izz') else '0'
        joint = ET.SubElement(model, 'joint', name=name+'_joint', type='revolute')
        ET.SubElement(joint, 'parent').text = 'base_link' if axis == 'yaw' else 'active_camera_yaw'
        ET.SubElement(joint, 'child').text = name
        a = ET.SubElement(joint, 'axis')
        ET.SubElement(a, 'xyz').text = '0 0 1' if axis == 'yaw' else '0 1 0'
        limits = ET.SubElement(a, 'limit')
        for key,value in [('lower',-math.radians(limit)),('upper',math.radians(limit)),('effort',.05),('velocity',math.radians(20))]:
            ET.SubElement(limits, key).text = str(value)
        plugin = ET.SubElement(model, 'plugin', filename='gz-sim-joint-position-controller-system', name='gz::sim::systems::JointPositionController')
        for key,value in [('joint_name',name+'_joint'),('topic','/active/gimbal/'+axis),('use_velocity_commands','true'),('cmd_max',math.radians(20)/math.sqrt(2))]:
            ET.SubElement(plugin, key).text = str(value)
        if axis == 'pitch':
            link.append(camera)
            for diagnostic in diagnostics:link.append(diagnostic)
            visual = ET.SubElement(link, 'visual', name='camera_housing')
            ET.SubElement(visual, 'pose').text = '-.0125 0 0 0 0 0'
            ET.SubElement(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size').text = '.025 .03 .03'
            material = ET.SubElement(visual,'material')
            ET.SubElement(material,'diffuse').text = '.1 .3 .9 1'
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-joint-state-publisher-system', name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(plugin,'topic').text = '/active/gimbal/states'
    for axis in ('yaw','pitch'):
        ET.SubElement(plugin,'joint_name').text = 'active_camera_'+axis+'_joint'
    for uri in root.iter('uri'):
        if uri.text.startswith('meshes/'):
            uri.text = str(source.parent / uri.text)
    ET.indent(root)
    path = output / 'unipilot_gimbal.sdf'
    (output/'camera_mount.json').write_text(json.dumps(dict(
        parent='base_link', mount_xyz=MOUNT_XYZ, mount_rpy=MOUNT_RPY,
        gimbal_limits_deg=GIMBAL_LIMITS_DEG,
        frame='rmf_unipilot/cam_gimbal_optical', model_source=str(source),
        diagnostic_sensors=diagnostic_sensors,sensor_profile=sensor_profile, processed_image_size=([int(profile.get('processed_width')),int(profile.get('processed_height'))] if profile is not None else None),
        camera_rate=CAMERA_RATE, camera_far=float(camera.findtext('camera/clip/far')), ideal_intersecting_axes=True),indent=2)+'\n')
    # The experiment owner uses the model's existence as its readiness signal.
    # Publish it only after both the XML and auxiliary calibration are complete.
    temporary=path.with_suffix('.sdf.tmp')
    ET.ElementTree(root).write(temporary, encoding='unicode', xml_declaration=True)
    temporary.replace(path)
    return path
