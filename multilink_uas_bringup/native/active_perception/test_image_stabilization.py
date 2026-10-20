import unittest
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from core import camera_rotation
from image_stabilization import ImageStabilization


class Tests(unittest.TestCase):
    def setUp(self):
        self.cfg=yaml.safe_load((Path(__file__).parent/'config.yaml').read_text())
        self.cfg['gaze_pitch_limit']=np.deg2rad(30.)
        self.controller=ImageStabilization(None,self.cfg)
        self.rays=np.array([[1,y,z] for y in [-.5,0,.5] for z in [-.2,.2]])
        self.rays/=np.linalg.norm(self.rays,axis=1)[:,None]

    def record(self,rays=None,motion=None,age=1.):
        rays=self.rays if rays is None else rays
        return (np.arange(len(rays)),rays*5,np.tile(np.eye(3),(len(rays),1,1)),
                np.full(len(rays),age),np.zeros_like(rays) if motion is None else motion)

    def test_stationary_view_stays_stationary(self):
        cur=np.array([.3,.1]);cmd,_=self.controller.choose(self.record(),np.eye(3),np.zeros(3),cur,.2,np.zeros(3))
        np.testing.assert_allclose(cmd,cur,atol=1e-12)

    def test_gyro_cancels_body_yaw(self):
        cmd,r=self.controller.choose(self.record(age=0),np.eye(3),np.zeros(3),np.zeros(2),.2,np.array([0,0,.2]))
        self.assertEqual(r['reason'],'stabilize_gyro_only')
        np.testing.assert_allclose(cmd,[-.04,0],atol=2e-5)

    def test_flow_correction_reduces_measured_bearing_motion(self):
        rates=np.cross(np.array([0,0,.1]),self.rays)
        cmd,_=self.controller.choose(self.record(motion=rates),np.eye(3),np.zeros(3),np.zeros(2),.2,np.zeros(3))
        future=self.rays+rates*.2;future/=np.linalg.norm(future,axis=1)[:,None]
        old=self.rays@camera_rotation(np.eye(3),0,0)
        held=future@camera_rotation(np.eye(3),0,0)
        corrected=future@camera_rotation(np.eye(3),*cmd)
        self.assertLess(np.linalg.norm(corrected-old),.01*np.linalg.norm(held-old))

    def test_metric_scale_does_not_change_command(self):
        record=list(self.record(motion=np.cross([0,.1,0],self.rays)))
        a,_=self.controller.choose(record,np.eye(3),np.zeros(3),np.zeros(2),.2,np.zeros(3))
        record[1]*=50;record[2]*=2500
        b,_=self.controller.choose(record,np.eye(3),np.zeros(3),np.zeros(2),.2,np.zeros(3))
        np.testing.assert_allclose(a,b,atol=1e-12)

    def test_outlier_does_not_steer_all_tracks(self):
        rays=np.tile(self.rays,(4,1));motion=np.cross([0,0,.1],rays);motion[0]=[0,-2,1]
        cmd,_=self.controller.choose(self.record(rays,motion),np.eye(3),np.zeros(3),np.zeros(2),.2,np.zeros(3))
        self.assertLess(np.linalg.norm(cmd-[.02,0]),.001)

    def test_joint_speed_and_limits(self):
        cur=np.array([self.cfg['limits_max'][0],.5]);cmd,_=self.controller.choose(self.record(),np.eye(3),np.zeros(3),cur,.2,np.array([0,-5,-5]))
        self.assertLessEqual(np.linalg.norm(cmd-cur),self.cfg['max_speed']*.2+1e-9)
        self.assertLessEqual(cmd[0],self.cfg['limits_max'][0]);self.assertLessEqual(cmd[1],np.deg2rad(30.))

    def test_empty_tracks_use_gyro_without_nan(self):
        cmd,r=self.controller.choose(self.record(np.empty((0,3))),np.eye(3),np.zeros(3),np.zeros(2),.2,np.array([0,.1,0]))
        self.assertTrue(np.isfinite(cmd).all());self.assertEqual(r['reason'],'stabilize_gyro_only')

    def test_follow_mode_returns_without_jump(self):
        self.cfg['stabilization_return_s']=5.
        controller=ImageStabilization(None,self.cfg);cur=np.array([.5,-.2])
        cmd,_=controller.choose(self.record(),np.eye(3),np.zeros(3),cur,.2,np.zeros(3))
        np.testing.assert_allclose(cmd,cur*np.exp(-.2/5),atol=1e-12)
        self.assertLess(np.linalg.norm(cmd-cur),self.cfg['max_speed']*.2)

    def test_follow_mode_preserves_small_body_turn_compensation(self):
        self.cfg['stabilization_return_s']=5.
        cmd,_=ImageStabilization(None,self.cfg).choose(self.record(age=0),np.eye(3),np.zeros(3),np.zeros(2),.2,np.array([0,0,.2]))
        np.testing.assert_allclose(cmd,[-.04,0],atol=2e-5)

if __name__=='__main__':unittest.main()
