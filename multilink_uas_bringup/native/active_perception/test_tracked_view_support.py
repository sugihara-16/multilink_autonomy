import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from tracked_view_support import retained_support,segment_position

class Camera:
    speed=1.
    def camera(self,q):
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('y',q[0]).as_matrix();return T
    def visible(self,q,points):return np.ones(len(points),bool)

class Support(unittest.TestCase):
    def setUp(self):
        self.kin=Camera();self.cfg=dict(tan_half_hfov=.9,tan_half_vfov=.6,retain_fraction=.9)
        self.points=np.c_[np.linspace(-.5,.5,20),np.zeros(20),np.full(20,3.)]
    def test_map_cannot_license_losing_current_tracks(self):
        valid,selected,hold=retained_support(self.kin,np.zeros(1),np.ones(1),self.points,np.zeros(3),np.zeros(3),self.cfg,2.)
        self.assertFalse(valid);self.assertEqual(hold[-1],20);self.assertLess(selected[-1],18)
    def test_small_turn_preserves_tracks(self):
        self.assertTrue(retained_support(self.kin,np.zeros(1),np.array([.05]),self.points,np.zeros(3),np.zeros(3),self.cfg,2.)[0])
    def test_holding_always_feasible_even_when_translation_loses_tracks(self):
        self.assertTrue(retained_support(self.kin,np.zeros(1),np.zeros(1),self.points,np.array([5.,0,0]),np.zeros(3),self.cfg,2.)[0])
    def test_stop_included_and_no_overshoot(self):
        for t in np.linspace(0,3,31):
            q=segment_position(np.zeros(1),np.array([6.]),4.,8.,t)
            self.assertTrue(0<=q[0]<=6.)
        self.assertAlmostEqual(segment_position(np.zeros(1),np.array([6.]),4.,8.,2.)[0],6.)
    def test_diagonal_gimbal_command_matches_vector_limiter(self):
        q=segment_position(np.zeros(2),np.array([20.,20.]),14.,None,1.,norm=2)
        self.assertAlmostEqual(np.linalg.norm(q),14.)

if __name__=='__main__':unittest.main()
