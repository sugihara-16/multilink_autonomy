import unittest
import numpy as np
from retention import protect_tracks
from bearing_motion import unit_rows

class RetentionTest(unittest.TestCase):
    def setUp(self):
        self.cfg=dict(tan_half_hfov=1.,tan_half_vfov=.6,border_fraction=.2,retain_fraction=.9,prediction_horizon_s=.8)
        self.limits=(np.deg2rad([-50.,-30.]),np.deg2rad([50.,30.]))
    def run_guard(self,points,cmd,omega=None):
        u=unit_rows(points)
        return protect_tracks(u,np.zeros_like(u),np.zeros(3) if omega is None else omega,np.eye(3),np.zeros(2),np.deg2rad(cmd),.2,self.cfg,self.limits)
    def test_rejects_slew_which_preserves_one_anchor_but_loses_other_tracks(self):
        cmd,r=self.run_guard([[1.,.2,0.],[1.,-.85,0.],[1.,-.84,.1],[1.,-.83,-.1]],[4.,0.])
        self.assertLess(r['retention_scale'],1.)
        self.assertGreaterEqual(cmd[0],0.)
        self.assertLessEqual(cmd[0],np.deg2rad(4.))
        self.assertLess(r['retention_full_slew_future'][-1],.9*r['retention_hold_future'][-1])
    def test_slew_that_keeps_all_tracks_is_unchanged(self):
        cmd,r=self.run_guard([[1.,.1,0.],[1.,.2,.1],[1.,0.,-.1]],[1.,0.])
        self.assertEqual(r['retention_scale'],1.)
        np.testing.assert_allclose(cmd,np.deg2rad([1.,0.]))
    def test_counter_rotation_is_allowed_when_it_improves_retention(self):
        cmd,r=self.run_guard([[1.,-.85,0.],[1.,-.84,.1],[1.,-.83,-.1]],[-4.,0.],np.deg2rad([0.,0.,20.]))
        self.assertEqual(r['retention_scale'],1.)
        self.assertGreater(r['retention_full_slew_future'][-1],r['retention_hold_future'][-1])
    def test_zero_motion_and_empty_tracks_remain_finite(self):
        for points in [[],[[1.,0.,0.]]]:
            cmd,r=self.run_guard(points,[0.,0.]);np.testing.assert_array_equal(cmd,[0.,0.])
    def test_metric_scale_does_not_change_result(self):
        points=np.array([[1.,.2,0.],[1.,-.85,0.],[1.,-.84,.1],[1.,-.83,-.1]])
        a,_=self.run_guard(points,[4.,0.]);b,_=self.run_guard(points*100,[4.,0.]);np.testing.assert_allclose(a,b)

if __name__=='__main__':unittest.main()
