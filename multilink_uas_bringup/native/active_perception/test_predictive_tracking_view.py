import unittest
import numpy as np
from test_joint_information_view import JointViewTest
from predictive_tracking_view import PredictiveTrackingView


class PredictiveTrackingTest(unittest.TestCase):
    def setUp(self):
        f=JointViewTest();f.setUp();self.fixture=f;self.r=f.mature()
        self.cfg=dict(f.cfg,fusion_velocity_covariance_scale=7.9,fusion_velocity_period_s=1.,gaze_pitch_limit=np.pi/2,
                      axis_max_speed=[np.deg2rad(20)/np.sqrt(2)]*2,joint_stop_margin=[.0005]*2)
        self.view=PredictiveTrackingView(f.fisher,self.cfg)
        self.view.set_joint_information(self.r[0],np.eye(3+2*len(self.r[0])))

    def run_case(self,velocity):
        self.view.set_joint_geometry(self.r[0],self.r[1],np.asarray(velocity),np.eye(3+3*len(self.r[0])))
        return self.fixture.call(self.r,self.view)

    def test_forecast_has_information_even_without_current_bearing_correlation(self):
        command,d=self.run_case([.5,0.,0.])
        self.assertGreater(d['hold_score'],0.)
        self.assertTrue(d['selection_score_kind'].startswith('predicted_geometry'))
        self.assertLessEqual(np.linalg.norm(command),self.cfg['max_speed']*.2+1e-12)
        self.assertTrue(np.all(abs(command)<=np.asarray(self.cfg['axis_max_speed'])*.2+1e-12))
        self.assertTrue(np.all(np.asarray(d['retention_selected_future'])>=self.cfg['retain_fraction']*np.asarray(d['retention_hold_future'])-1e-12))

    def test_zero_motion_prefers_hold_when_all_tracks_are_central(self):
        command,d=self.run_case([0.,0.,0.]);np.testing.assert_array_equal(command,[0.,0.])
        self.assertEqual(d['reason'],'correlated_tracking_hold')

    def test_identity_mismatch_rejected(self):
        self.view.set_joint_geometry(self.r[0]+1,self.r[1],np.zeros(3),np.eye(3+3*len(self.r[0])))
        with self.assertRaises(ValueError):self.fixture.call(self.r,self.view)


if __name__=='__main__':unittest.main()
