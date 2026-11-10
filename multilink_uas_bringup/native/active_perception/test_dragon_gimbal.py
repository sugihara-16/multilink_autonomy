"""Regression checks on generated physical model and exposure-time transforms."""
import json,sys,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory as share
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import configure,Kinematics,NAMES,sensor_spec
from dragon_gimbal import configure as add_gimbal,MOUNT_XYZ,MOUNT_RPY,GIMBAL_NAMES
from dragon_cameras import link_transforms,transform,OPTICAL
from audit_dragon_gimbal import audit

class Gimbal(unittest.TestCase):
    profile='legacy'
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.out=Path(cls.tmp.name)
        cls.root=ET.fromstring(xacro.process_file(share('dragon')+'/robots/quad/dragon.gazebo.xacro',mappings={'robot_name':'dragon','visual_detail':'balanced','body_joint_velocity':'1.0'}).toxml())
        configure(cls.root,Path(share('rmf_gz'))/'resources/robots/rmf_unipilot/model.sdf',cls.out)
        cls.before={s.get('type'):sensor_spec(s) for s in cls.root.findall('gazebo/sensor')}
        cls.report=add_gimbal(cls.root,cls.out,profile=cls.profile);ET.ElementTree(cls.root).write(cls.out/'model.urdf');cls.kin=Kinematics(cls.out/'model.urdf')
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def test_sensor_physics_unchanged(self):
        after={s.get('type'):sensor_spec(s) for s in self.root.findall('gazebo/sensor')}
        self.assertEqual(self.before,after);self.assertTrue(all(self.report['parity'].values()))
    def test_full_camera_pose_uses_both_measured_gimbal_angles(self):
        for yaw,pitch in [(0,0),(-50,90),(50,-90),(13,-27)]:
            q=np.r_[self.kin.home,np.deg2rad([yaw,pitch])]
            frames=link_transforms(self.root,dict(zip(NAMES+GIMBAL_NAMES,q)))
            np.testing.assert_allclose(self.kin.camera(q),np.linalg.inv(frames['fc'])@frames['camera_optical'],atol=1e-12)
        home=self.kin.camera(np.r_[self.kin.home,0,0])
        rotated=self.kin.camera(np.r_[self.kin.home,.7,1.2])
        np.testing.assert_allclose(home[:3,3],rotated[:3,3],atol=1e-12)
        self.assertGreater(np.linalg.norm(home[:3,:3]-rotated[:3,:3]),1.)
    def test_full_image_clearance(self):
        result=audit(self.root);self.assertTrue(result['passed'],result['examples']);self.assertEqual(result['checked_views'],4641)

class MatchedGimbal(Gimbal):
    profile='unipilot'
    def test_forward_pose_and_nominal_offset(self):
        T=np.array(self.report['T_FC_camera'])
        np.testing.assert_allclose(T[:3,:3],np.eye(3),atol=1e-12)
        np.testing.assert_allclose(T[:3,3],[.8,0,-.025],atol=1e-12)
    def test_lidar_origin_outside_robot_and_rotor_bounds(self):
        from audit_dragon_gimbal import visual_bounds
        boxes,_=visual_bounds(self.root)
        frames=link_transforms(self.root,dict(zip(NAMES,[0.,1.56]*3)))
        T=np.linalg.inv(frames['fc'])@frames['link4']
        boxes=boxes@T[:3,:3].T+T[:3,3]
        p=np.array(self.report['lidar_pose_fc'][:3])
        self.assertFalse(np.any(np.all(p>=boxes.min(axis=1),axis=1)&np.all(p<=boxes.max(axis=1),axis=1)))

if __name__=='__main__':unittest.main()
