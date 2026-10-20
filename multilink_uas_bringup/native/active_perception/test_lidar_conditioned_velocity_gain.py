import unittest
import numpy as np
from joint_velocity_gain import conditional_velocity_covariance
from lidar_conditioned_velocity_gain import fused_position_gain


class LidarConditionedVelocityTest(unittest.TestCase):
    def fixture(self):
        C=np.eye(7);C[0,3]=C[3,0]=.5;C[2,5]=C[5,2]=.5
        return C,np.eye(2)

    def test_prefer_constraint_missing_from_lidar(self):
        C,w=self.fixture();H=np.eye(6);H[3,3]=1000.
        score=fused_position_gain(H,C,w,.2,1.)
        self.assertGreater(score[1],10*score[0])

    def test_rotation_correlation_is_marginalized(self):
        C,w=self.fixture();H=np.eye(6);H[0,3]=H[3,0]=.99
        score=fused_position_gain(H,C,w,.2,1.)
        self.assertGreater(score[0],score[1])

    def test_matches_full_six_dimensional_inverse(self):
        rng=np.random.default_rng(915)
        for unused in range(20):
            A=rng.normal(size=(6,6));H=A@A.T+np.eye(6)
            B=rng.normal(size=(11,11));C=B@B.T+np.eye(11)
            w=rng.uniform(size=(5,4));h=.7
            P=conditional_velocity_covariance(C,w,.3)
            def logdet_position(V):
                full=H.copy();full[3:,3:]+=np.linalg.inv(V*h*h)
                return np.linalg.slogdet(np.linalg.inv(full)[3:,3:])[1]
            expected=np.array([logdet_position(C[:3,:3])-logdet_position(V) for V in P])
            np.testing.assert_allclose(fused_position_gain(H,C,w,.3,h),expected,atol=3e-12)

    def test_position_units_do_not_change_score(self):
        C,w=self.fixture();H=np.eye(6);pose=np.diag([1,1,1,.001,.001,.001]);vel=np.eye(7);vel[:3,:3]*=1000.
        np.testing.assert_allclose(fused_position_gain(H,C,w,.2,1.),
            fused_position_gain(pose@H@pose,vel@C@vel,w,.2,1.),atol=1e-12)

    def test_no_new_measurements_no_gain_and_invalid_horizon_rejected(self):
        C,w=self.fixture()
        np.testing.assert_allclose(fused_position_gain(np.eye(6),C,w*0.,.2,1.),0.,atol=1e-12)
        with self.assertRaises(ValueError):fused_position_gain(np.eye(6),C,w,.2,0.)


if __name__=='__main__':unittest.main()
