"""Matched initial geometry and moving-camera transform regression."""
import tempfile, unittest, json, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from test_dragon_gimbal import Gimbal
from dragon_gimbal import configure_fixed_mount, configure as add_gimbal
from dragon_articulated import Kinematics, configure, sensor_spec, NAMES
from dragon_cameras import link_transforms
from ament_index_python.packages import get_package_share_directory as share
import xacro

class Mechanisms(unittest.TestCase):
    def test_geometry_sensor_and_motion_parity(self):
        with tempfile.TemporaryDirectory() as tmp:
            models=[]
            for name in ('gimbal','joint'):
                out=Path(tmp)/name;out.mkdir()
                root=ET.fromstring(xacro.process_file(share('dragon')+'/robots/quad/dragon.gazebo.xacro',mappings={'robot_name':'dragon','visual_detail':'balanced','body_joint_velocity':'1.0'}).toxml())
                configure(root,Path(share('rmf_gz'))/'resources/robots/rmf_unipilot/model.sdf',out)
                if name=='gimbal':add_gimbal(root,out,profile='unipilot')
                else:configure_fixed_mount(root,out)
                path=out/'model.urdf';ET.ElementTree(root).write(path)
                models.append((root,Kinematics(path),json.loads((out/'articulated_manifest.json').read_text())))
            g,j=models
            self.assertEqual(len(g[1].names),8);self.assertEqual(len(j[1].names),6)
            np.testing.assert_allclose(g[1].camera(np.r_[g[1].home,0.,0.]),j[1].camera(j[1].home),atol=1e-12)
            self.assertEqual(g[2]['lidar_pose_fc'],j[2]['lidar_pose_fc'])
            for sensor in ('end_camera','end_lidar','spinal_imu'):
                self.assertEqual(sensor_spec(g[0].find(f".//sensor[@name='{sensor}']")),sensor_spec(j[0].find(f".//sensor[@name='{sensor}']")))
            for q in [j[1].home,j[1].home+np.array([.1,-.2,-.1,.2,.1,-.1])]:
                frames=link_transforms(j[0],dict(zip(NAMES,q)))
                np.testing.assert_allclose(j[1].camera(q),np.linalg.inv(frames['fc'])@frames['camera_optical'],atol=1e-12)
                np.testing.assert_allclose(g[1].camera(np.r_[q,0.,0.]),j[1].camera(q),atol=1e-12)
            self.assertGreater(np.linalg.norm(j[1].camera(q)[:3,3]-j[1].camera(j[1].home)[:3,3]),.01)

if __name__=='__main__':unittest.main()
