import unittest
import numpy as np
from correlated_tracking_view import CorrelatedTrackingView
from test_joint_information_view import JointViewTest
from core import camera_rotation


class CorrelatedTrackingTest(unittest.TestCase):
    def setUp(self):
        f=JointViewTest();f.setUp();self.fixture=f
        self.cfg=dict(f.cfg);self.cfg['gaze_pitch_limit']=np.pi/2
        self.view=CorrelatedTrackingView(f.fisher,self.cfg)
        self.r=f.mature();n=len(self.r[0]);C=np.eye(3+2*n)
        C[:3,3:]=.05;C[3:,:3]=.05
        self.view.set_joint_information(self.r[0],C)

    def call(self, r, current=None):
        return self.fixture.call(r,self.view,current=current)

    def test_depth_precision_does_not_discard_observed_bearing_information(self):
        self.r[1][:,2]=1.3;self.r[4][:]=[0,0,.45]
        a,da=self.call(self.r)
        self.r[2][:]=np.eye(3)*1e12
        b,db=self.call(self.r)
        np.testing.assert_array_equal(a,b);self.assertEqual(da,db)
        self.assertFalse(db['depth_precision_gate']);self.assertLess(b[1],0.)

    def test_metric_scale_does_not_change_bearing_based_decision(self):
        self.r[1][:,2]=1.3;self.r[4][:]=[0,0,.45]
        a,da=self.call(self.r)
        self.r[1][:]*=10.;self.r[2][:]*=100.
        b,db=self.call(self.r)
        np.testing.assert_allclose(a,b,atol=1e-14)
        self.assertAlmostEqual(da['selected_score'],db['selected_score'])

    def test_full_pitch_range_respects_physical_limits_and_track_retention(self):
        for pitch in [-89.,-60.,60.,89.]:
            current=np.deg2rad([40.,pitch]);R=camera_rotation(np.eye(3),*current)
            self.r[1][:]=np.array([[-.2,-.1,3.],[.2,.1,3.],[.3,-.2,3.],[-.3,.2,3.]])@R.T
            self.r[4][:]=0.
            command,d=self.call(self.r,current)
            lo,hi=self.view.motion.limits()
            self.assertTrue(np.all(command>=lo));self.assertTrue(np.all(command<=hi))
            self.assertLessEqual(np.linalg.norm(command-current),self.cfg['max_speed']*.2+1e-12)
            self.assertTrue(np.all(np.array(d['retention_selected_future'])>=self.cfg['retain_fraction']*np.array(d['retention_hold_future'])-1e-12))

    def test_unseen_points_do_not_gain_visibility_from_the_covariance(self):
        self.r[1][:,0]*=-1
        command,d=self.call(self.r)
        self.assertFalse(d['reason'].startswith('correlated_tracking'))
        np.testing.assert_array_equal(command,[0.,0.])


if __name__=='__main__':unittest.main()
