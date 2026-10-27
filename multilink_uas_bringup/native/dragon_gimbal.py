"""UniPilot sensors on DRAGON's opposite ends, with an outward two-axis camera.

Body joints stay square. The optical centre and both ideal gimbal axes coincide;
the support is behind the complete camera viewing envelope, not hidden by a mask.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from dragon_articulated import NAMES, Kinematics
from dragon_cameras import OPTICAL, transform, link_transforms

GIMBAL_NAMES=['active_camera_yaw_joint','active_camera_pitch_joint']
MOUNT_XYZ=(.10,-.60,0.)
MOUNT_RPY=(0.,0.,-math.pi/2)
LIMITS=np.deg2rad([50.,90.])


def configure(root,output):
    """Apply after dragon_articulated.configure: do not change sensor physics."""
    output=Path(output)
    camera=root.find(".//sensor[@name='end_camera']")
    for g in root.findall('gazebo'):
        if camera in list(g):g.remove(camera)
    camera.find('pose').text='0 0 0 0 0 0'
    ET.SubElement(root,'gazebo',reference='active_camera_pitch').append(camera)
    link4=root.find("link[@name='link4']")
    link4.remove(link4.find("visual[@name='end_camera_housing']"))
    for axis,limit in zip(('yaw','pitch'),LIMITS):
        name='active_camera_'+axis
        body=ET.SubElement(root,'link',name=name)
        inertial=ET.SubElement(body,'inertial')
        ET.SubElement(inertial,'mass',value='.002')
        ET.SubElement(inertial,'inertia',ixx='.000001',iyy='.000001',izz='.000001',ixy='0',ixz='0',iyz='0')
        j=ET.SubElement(root,'joint',name=name+'_joint',type='revolute')
        ET.SubElement(j,'parent',link='link4' if axis=='yaw' else 'active_camera_yaw')
        ET.SubElement(j,'child',link=name)
        ET.SubElement(j,'origin',xyz=' '.join(map(str,MOUNT_XYZ)) if axis=='yaw' else '0 0 0',
                      rpy=' '.join(map(str,MOUNT_RPY)) if axis=='yaw' else '0 0 0')
        ET.SubElement(j,'axis',xyz='0 0 1' if axis=='yaw' else '0 1 0')
        ET.SubElement(j,'limit',lower=str(-limit),upper=str(limit),effort='.05',velocity=str(math.radians(20)))
        plugin=ET.SubElement(ET.SubElement(root,'gazebo'),'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
        for key,value in [('joint_name',name+'_joint'),('topic','/active/gimbal/'+axis),('use_velocity_commands','true'),('cmd_max',math.radians(20)/math.sqrt(2))]:
            ET.SubElement(plugin,key).text=str(value)
    housing=ET.SubElement(root.find("link[@name='active_camera_pitch']"),'visual',name='end_camera_housing')
    ET.SubElement(housing,'origin',xyz='-.0125 0 0')
    ET.SubElement(ET.SubElement(housing,'geometry'),'box',size='.025 .03 .03')
    ET.SubElement(ET.SubElement(housing,'material',name='gimbal_blue'),'color',rgba='.1 .3 .9 1')
    # An actual visible support, with its inner end at link4. No height mast.
    bracket=ET.SubElement(link4,'visual',name='gimbal_boom')
    ET.SubElement(bracket,'origin',xyz=f'{MOUNT_XYZ[0]} {MOUNT_XYZ[1]/2} 0')
    ET.SubElement(ET.SubElement(bracket,'geometry'),'box',size=f'.018 {-MOUNT_XYZ[1]-.12} .018')
    ET.SubElement(ET.SubElement(bracket,'material',name='gimbal_boom_gray'),'color',rgba='.25 .25 .25 1')
    j=root.find("joint[@name='camera_optical_mount']")
    j.find('parent').set('link','active_camera_pitch')
    j.find('origin').set('xyz','0 0 0')
    j.find('origin').set('rpy',' '.join(map(str,Rotation.from_matrix(OPTICAL).as_euler('xyz'))))
    plugin=root.find("gazebo/plugin[@name='gz::sim::systems::JointStatePublisher']")
    for name in GIMBAL_NAMES:ET.SubElement(plugin,'joint_name').text=name
    q={n:v for n,v in zip(NAMES,[0.,1.56]*3)};frames=link_transforms(root,q)
    T=np.linalg.inv(frames['fc'])@frames['link4']@transform(MOUNT_XYZ,MOUNT_RPY)
    report=json.loads((output/'articulated_manifest.json').read_text())
    report.update(camera_actuation='gimbal',camera_xyz=MOUNT_XYZ,camera_rpy=MOUNT_RPY,T_FC_camera=T.tolist(),
                  gimbal_limits_deg=np.rad2deg(LIMITS).tolist(),body_configuration='fixed square',
                  gimbal_axes='intersect at optical centre',gimbal_added_mass_kg=.004,
                  support_model='visible ideal boom; support mass and compliance not modeled')
    (output/'articulated_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


class GimbalKinematics:
    """Two controllable axes, with measured six-joint body geometry."""
    def __init__(self,path):
        self.full=Kinematics(path);self.body=self.full.home.copy()
        self.home=np.zeros(2);self.lower=-LIMITS;self.upper=LIMITS
        self.speed=math.radians(20)/math.sqrt(2)
    def camera(self,q):return self.full.camera(np.r_[self.body,q])
    def valid(self,q):return bool(np.isfinite(q).all() and np.all(q>=self.lower-1e-8) and np.all(q<=self.upper+1e-8))
    def path_valid(self,a,b,resolution=None):return self.valid(a) and self.valid(b)
    def collision_samples(self,q):return self.full.collision_samples(np.r_[self.body,q])
    def visible(self,q,points):return self.full.visible(np.r_[self.body,q],points)


class GimbalRoadmap:
    def __init__(self,kin):
        self.kin=kin
        self.q=np.deg2rad([[y,p] for y in np.linspace(-50,50,11) for p in np.linspace(-90,90,19)])
    def paths(self,current):return {i:(float(np.max(abs(q-current))/self.kin.speed),[np.array(current),q]) for i,q in enumerate(self.q)}
    def audit(self):return dict(nodes=len(self.q),reachable=len(self.q),joint_min_deg=np.rad2deg(self.kin.lower).tolist(),joint_max_deg=np.rad2deg(self.kin.upper).tolist(),body='fixed square; camera yaw/pitch only')
