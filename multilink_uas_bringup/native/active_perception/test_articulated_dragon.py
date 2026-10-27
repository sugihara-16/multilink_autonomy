"""Sensor transplant and moving-camera regression checks against actual URDF."""
import sys,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory as share
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import configure,Kinematics

class Articulated(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.out=Path(cls.tmp.name)
        cls.root=ET.fromstring(xacro.process_file(share('dragon')+'/robots/quad/dragon.gazebo.xacro',
            mappings={'robot_name':'dragon','visual_detail':'balanced','body_joint_velocity':'1.0'}).toxml())
        cls.report=configure(cls.root,Path(share('rmf_gz'))/'resources/robots/rmf_unipilot/model.sdf',cls.out)
        ET.ElementTree(cls.root).write(cls.out/'model.urdf');cls.k=Kinematics(cls.out/'model.urdf')
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def test_sensor_parity_and_count(self):
        self.assertTrue(all(self.report['parity'].values()))
        sensors=self.root.findall('gazebo/sensor')
        self.assertEqual(len(sensors),4)
        self.assertEqual(sorted(s.get('type') for s in sensors),['camera','gpu_ray','imu','magnetometer'])
        self.assertTrue(self.report['flight_controller_magnetometer']['parity'])
    def test_lidar_imu_rigid_on_end(self):
        self.assertEqual(self.root.find("joint[@name='fc_joint']/parent").get('link'),'link1')
        for q in [self.k.home,self.k.home+np.array([.1,-.1,-.1,-.2,.1,-.2])]:
            f=self.k.frames(q)
            np.testing.assert_allclose(np.linalg.inv(f['link1'])@f['fc'],self.k.frames(self.k.home)['fc'])
    def test_camera_translation_and_rotation(self):
        a=self.k.camera(self.k.home);q=self.k.home.copy();q[4]=.15;q[5]-=.2;b=self.k.camera(q)
        self.assertGreater(np.linalg.norm(a[:3,3]-b[:3,3]),.02)
        self.assertGreater(np.linalg.norm(a[:3,:3]-b[:3,:3]),.2)
        self.assertTrue(self.k.valid(q))
    def test_unsafe_joint_limit_rejected(self):
        q=self.k.home.copy();q[0]=2.;self.assertFalse(self.k.valid(q))
    def test_occlusion_ray_points_towards_body(self):
        f=self.k.frames(self.k.home);T=self.k.camera(self.k.home)
        inv=np.linalg.inv(f['fc']);center=(inv@f['link2']@np.array([.263,0,.12,1]))[:3]
        point=T[:3,3]+3*(center-T[:3,3]);self.assertFalse(self.k.visible(self.k.home,[point])[0])

if __name__=='__main__':unittest.main()
