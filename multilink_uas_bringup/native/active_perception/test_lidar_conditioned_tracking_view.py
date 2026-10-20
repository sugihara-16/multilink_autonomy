import unittest
import numpy as np
from lidar_conditioned_tracking_view import LidarConditionedTrackingView
from lidar_conditioned_velocity_gain import fused_position_gain


class LidarTrackingScoreTest(unittest.TestCase):
    def view(self):
        v=LidarConditionedTrackingView.__new__(LidarConditionedTrackingView)
        v.cfg=dict(prior_std=[.2,.2,.2,10.,10.,10.],bearing_sigma=.1,
            fusion_velocity_covariance_scale=7.9,fusion_velocity_period_s=1.)
        return v

    def test_uses_existing_calibration_on_velocity_and_cross_terms(self):
        v=self.view();C=np.eye(7);C[0,3]=C[3,0]=.5;C[2,5]=C[5,2]=.5
        H=np.eye(6);H[3,3]=1000.;w=np.eye(2);score,kind=v.information_score(H,C)
        A=np.eye(7);A[:3,:3]*=np.sqrt(7.9)
        expected=fused_position_gain(H+np.diag(1/np.square(v.cfg['prior_std'])),A@C@A,w,.1,1.)
        np.testing.assert_allclose(score(w),expected,atol=1e-12)
        self.assertGreater(score(w)[1],100*score(w)[0])
        self.assertIn('lidar',kind)

    def test_no_lidar_is_the_velocity_information_limit(self):
        from joint_velocity_gain import velocity_gain
        v=self.view();v.cfg['prior_std']=[1e8]*6
        C=np.eye(7);C[0,3]=C[3,0]=.5;C[2,5]=C[5,2]=.5
        score,_=v.information_score(np.zeros((6,6)),C)
        np.testing.assert_allclose(score(np.eye(2)),velocity_gain(C,np.eye(2),.1),atol=1e-12)

    def test_actual_selection_uses_conditioned_score_and_retains_tracks(self):
        from test_correlated_tracking_view import CorrelatedTrackingTest
        fixture=CorrelatedTrackingTest();fixture.setUp()
        cfg=dict(fixture.cfg,fusion_velocity_covariance_scale=7.9,fusion_velocity_period_s=1.)
        view=LidarConditionedTrackingView(fixture.fixture.fisher,cfg)
        view.set_joint_information(fixture.view.joint_ids,fixture.view.joint_covariance)
        fixture.r[1][:,2]=1.3;fixture.r[4][:]=[0.,0.,.45]
        command,d=fixture.fixture.call(fixture.r,view)
        self.assertEqual(d['selection_score_kind'],'lidar_conditioned_position_increment_information_gain')
        self.assertLessEqual(np.linalg.norm(command),cfg['max_speed']*.2+1e-12)
        self.assertTrue(np.all(np.asarray(d['retention_selected_future'])>=cfg['retain_fraction']*np.asarray(d['retention_hold_future'])-1e-12))


if __name__=='__main__':unittest.main()
