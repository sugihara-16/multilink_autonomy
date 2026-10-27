"""End-mounted DRAGON sensing, copied from the current UniPilot generator.

The flight computer and LiDAR are rigid on link1; the sole camera is on link4.
No legacy DRAGON camera, LiDAR or IMU noise settings enter this model.
"""
from copy import deepcopy
import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from dragon_cameras import link_transforms, transform, OPTICAL
from unipilot_gimbal import prepare

NAMES = [f'joint{i}_{axis}' for i in (1, 2, 3) for axis in ('pitch', 'yaw')]
CAMERA_XYZ = (.10, -.08, 0.)
CAMERA_RPY = (0., 0., -math.pi/2)


def sensor_spec(sensor):
    """Compare sensor physics, excluding attachment, names and transport."""
    s = deepcopy(sensor)
    s.attrib.clear()
    for tag in ('pose', 'topic', 'gz_frame_id', 'plugin'):
        for child in s.findall(tag): s.remove(child)
    def canonical(e):
        return [e.tag, sorted(e.attrib.items()), (e.text or '').strip(),
                [canonical(c) for c in e]]
    return canonical(s)


def configure(root, source, output):
    output = Path(output)
    generated = prepare(source, output, camera_far=1000., sensor_profile='native', imu_sampling='resolved')
    reference = ET.parse(generated).getroot()
    sensors = {kind: deepcopy(reference.find(f".//sensor[@type='{kind}']"))
               for kind in ('imu', 'gpu_ray', 'camera')}
    # Move the controller's reference frame with its IMU to the first end.
    root.find("joint[@name='fc_joint']/parent").set('link', 'link1')
    for j in root.findall('joint'):
        if j.find('child').get('link') == 'livox_mid360': j.find('parent').set('link', 'link1')
    # The FC mass is represented partly by the baselink's inertial element.
    a, b = [root.find(f"link[@name='link{i}']") for i in (1, 2)]
    ia, ib = a.find('inertial'), b.find('inertial')
    a.remove(ia); b.remove(ib); a.append(ib); b.append(ia)
    removed = []
    for g in root.findall('gazebo'):
        for s in list(g.findall('sensor')):
            if s.get('type') in ('imu', 'gpu_lidar', 'gpu_ray', 'ray', 'camera', 'magnetometer', 'navsat'):
                removed.append(s.get('name')); g.remove(s)
    imu = sensors['imu']; imu.set('name', 'spinal_imu')
    for tag, value in [('topic','/dragon/imu/raw'),('gz_frame_id','fc'),('pose','0 0 0 0 0 0')]:
        e=imu.find(tag)
        if e is None: e=ET.SubElement(imu,tag)
        e.text=value
    ET.SubElement(root,'gazebo',reference='fc').append(imu)
    # The flight controller requires a magnetometer interface even in GT mode.
    # Use UniPilot's definition, not the removed aerial_robot_base noise model.
    mag_source=ET.fromstring(re.search(r'<sensor name="magnetometer" type="magnetometer">.*?</sensor>',Path(source).read_text(),re.S).group())
    mag=deepcopy(mag_source);mag.set('name','spinal_mag')
    ET.SubElement(root,'gazebo',reference='fc').append(mag)
    lidar=sensors['gpu_ray']; lidar.set('name','end_lidar')
    lidar.find('topic').text='/dragon/lidar/scan';lidar.find('gz_frame_id').text='dragon/lidar'
    # Preserve UniPilot's LiDAR-to-IMU extrinsics as well as sensor physics.
    ET.SubElement(root,'gazebo',reference='fc').append(lidar)
    camera=sensors['camera'];camera.set('name','end_camera')
    camera.find('pose').text=' '.join(map(str,CAMERA_XYZ+CAMERA_RPY))
    camera.find('topic').text='/dragon/end_camera/image';camera.find('gz_frame_id').text='dragon/camera_optical'
    ET.SubElement(root,'gazebo',reference='link4').append(camera)
    optical=ET.SubElement(root,'link',name='camera_optical')
    j=ET.SubElement(root,'joint',name='camera_optical_mount',type='fixed')
    ET.SubElement(j,'parent',link='link4');ET.SubElement(j,'child',link='camera_optical')
    R=transform(rpy=CAMERA_RPY)[:3,:3]@OPTICAL
    ET.SubElement(j,'origin',xyz=' '.join(map(str,CAMERA_XYZ)),rpy=' '.join(map(str,Rotation.from_matrix(R).as_euler('xyz'))))
    visual=ET.SubElement(root.find("link[@name='link4']"),'visual',name='end_camera_housing')
    ET.SubElement(visual,'origin',xyz='.10 -.0675 0',rpy='0 0 0')
    ET.SubElement(ET.SubElement(visual,'geometry'),'box',size='.03 .025 .03')
    ET.SubElement(ET.SubElement(visual,'material',name='end_camera_blue'),'color',rgba='.1 .3 .9 1')
    plugin=ET.SubElement(ET.SubElement(root,'gazebo'),'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(plugin,'topic').text='/dragon/articulated/states'
    for name in NAMES: ET.SubElement(plugin,'joint_name').text=name
    angles={j.get('name'):float(j.find("state_interface[@name='position']/param[@name='initial_value']").text)
            for j in root.findall('ros2_control/joint') if j.get('name') in NAMES}
    frames=link_transforms(root,angles)
    T=np.linalg.inv(frames['fc'])@frames['link4']@transform(CAMERA_XYZ,CAMERA_RPY)
    report={'sensor_source':str(source),'profile':'native','imu_sampling':'resolved',
            'removed_default_sensors':removed,'reference_link':'link1','camera_link':'link4',
            'parity':{kind:sensor_spec(s)==sensor_spec(reference.find(f".//sensor[@type='{kind}']")) for kind,s in sensors.items()},
            'joint_names':NAMES,'initial_joints':angles,'T_FC_camera':T.tolist(),
            'camera_xyz':CAMERA_XYZ,'camera_rpy':CAMERA_RPY}
    report['flight_controller_magnetometer']={'source':'UniPilot original SDF','used_by_slam':False,'parity':sensor_spec(mag)==sensor_spec(mag_source)}
    if not all(report['parity'].values()): raise ValueError('Sensor settings differ from UniPilot')
    (output/'articulated_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


class Kinematics:
    def __init__(self, path):
        self.root=ET.parse(path).getroot()
        self.names=NAMES+[j.get('name') for j in self.root.findall('joint') if j.get('name') in ('active_camera_yaw_joint','active_camera_pitch_joint')]
        self.home=np.array([0.,1.56]*3)
        # Actual URDF limits, with a small stop margin; shape feasibility is
        # checked independently instead of imposing the old near-square box.
        limits=[self.root.find(f"joint[@name='{n}']/limit") for n in NAMES]
        self.lower=np.array([float(e.get('lower'))+.01 for e in limits])
        self.upper=np.array([float(e.get('upper'))-.01 for e in limits])
        self.speed=np.deg2rad(10.)
        self.acceleration=np.deg2rad(20.)
        self._chain=[];known={'root'};pending=list(self.root.findall('joint'))
        while pending:
            progressed=False
            for joint in pending[:]:
                parent=joint.find('parent').get('link')
                if parent not in known:continue
                child=joint.find('child').get('link');o=joint.find('origin')
                xyz=list(map(float,o.get('xyz','0 0 0').split())) if o is not None else [0]*3
                rpy=list(map(float,o.get('rpy','0 0 0').split())) if o is not None else [0]*3
                name=joint.get('name');index=self.names.index(name) if name in self.names else None
                axis=np.array(list(map(float,joint.find('axis').get('xyz').split()))) if index is not None else None
                self._chain.append((parent,child,transform(xyz,rpy),index,axis))
                known.add(child);pending.remove(joint);progressed=True
            if not progressed:raise ValueError('Disconnected URDF')
        self._last_q=None;self._last_frames=None

    def frames(self, q):
        q=np.asarray(q)
        if self._last_q is not None and np.array_equal(q,self._last_q):return self._last_frames
        frames={'root':np.eye(4)}
        for parent,child,origin,index,axis in self._chain:
            T=frames[parent]@origin
            if index is not None:
                T=T.copy();T[:3,:3]=T[:3,:3]@Rotation.from_rotvec(axis*q[index]).as_matrix()
            frames[child]=T
        self._last_q=q.copy();self._last_frames=frames
        return frames

    def camera(self, q):
        f=self.frames(q)
        return np.linalg.inv(f['fc'])@f['camera_optical']

    def valid(self,q):
        if not np.isfinite(q).all() or np.any(q<self.lower-1e-8) or np.any(q>self.upper+1e-8): return False
        f=self.frames(q)
        # Swept rotor-module bounds, independent of rotor-gimbal angle.
        centers=np.array([f[f'thrust{i}'][:3,3] for i in range(1,5)])
        if any(np.linalg.norm(centers[i]-centers[j])<.36 for i in range(4) for j in range(i+1,4)):return False
        # Exclude nearly collinear support and extreme vertical folding before
        # invoking the flight controller. This is a geometric feasibility gate,
        # not a complete thrust-allocation guarantee.
        xy=centers[:,:2]-centers[:,:2].mean(0)
        if np.linalg.svd(xy,compute_uv=False)[-1]<.10 or np.ptp(centers[:,2])>1.25:return False
        samples=[(f[f'link{i}']@np.array([[x,0,0,1] for x in np.linspace(.07,.44,8)]).T).T[:,:3] for i in range(1,5)]
        for i in range(4):
            for j in range(i+1,4):
                if j-i>1 and np.min(np.linalg.norm(samples[i][:,None]-samples[j][None],axis=-1))<.10:return False
                if np.min(np.linalg.norm(samples[i]-centers[j],axis=-1))<.23:return False
                if np.min(np.linalg.norm(samples[j]-centers[i],axis=-1))<.23:return False
        return True

    def path_valid(self,a,b,resolution=np.deg2rad(3.)):
        n=max(2,int(np.ceil(np.max(abs(np.asarray(b)-a))/resolution))+1)
        return all(self.valid(q) for q in np.linspace(a,b,n))

    def collision_samples(self,q):
        f=self.frames(q);inv=np.linalg.inv(f['fc'])
        centers=[(inv@f[f'thrust{i}'])[:3,3] for i in range(1,5)]
        tubes=[(inv@f[f'link{i}']@np.array([x,0,0,1]))[:3] for i in range(1,5) for x in np.linspace(0,.5,6)]
        return np.array(centers+tubes),np.array([.23]*4+[.06]*len(tubes))

    def visible(self,q,points):
        """Conservative body/rotor bounding spheres along camera-to-feature rays.

        Excludes the housing, whose optical centre lies on its front face.
        Rotor spheres bound the swept disc; false exclusions are intentional.
        """
        f=self.frames(q);inv=np.linalg.inv(f['fc']);T=self.camera(q)
        origin=T[:3,3];rays=np.asarray(points)-origin;length=np.linalg.norm(rays,axis=1)
        unit=rays/np.maximum(length[:,None],1e-12);visible=np.ones(len(points),bool)
        for i in range(1,5):
            for x,radius,z in [(.05,.035,0),(.15,.035,0),(.25,.035,0),(.35,.035,0),(.45,.035,0),(.263,.19,.12)]:
                center=(inv@f[f'link{i}']@np.array([x,0,z,1]))[:3]
                v=center-origin;along=unit@v
                distance2=np.maximum(0.,v@v-along**2)
                visible&=~((along>0)&(along<length)&(distance2<radius**2))
        return visible
