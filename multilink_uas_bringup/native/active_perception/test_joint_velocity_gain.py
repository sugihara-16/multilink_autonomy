import unittest
import numpy as np
from joint_velocity_gain import conditional_velocity_covariance, velocity_gain


class JointVelocityGainTest(unittest.TestCase):
    def setUp(self):
        self.rng = np.random.default_rng(7601)

    def covariance(self, n):
        A = self.rng.normal(size=(3+2*n, 3+2*n))
        return A@A.T+.2*np.eye(len(A))

    def test_matches_full_gaussian_information_update(self):
        for n in [1, 3, 12]:
            C = self.covariance(n)
            weights = self.rng.uniform(size=(5, n))
            actual = conditional_velocity_covariance(C, weights, .7)
            for i, w in enumerate(weights):
                information = np.linalg.inv(C)
                information[3:, 3:] += np.diag(np.repeat(w, 2)/.7**2)
                expected = np.linalg.inv(information)[:3, :3]
                np.testing.assert_allclose(actual[i], expected, rtol=2e-12, atol=2e-12)

    def test_no_observations_or_no_cross_correlation_gives_zero_gain(self):
        C = self.covariance(4)
        np.testing.assert_allclose(velocity_gain(C, np.zeros((2, 4)), .1), 0., atol=1e-12)
        C[:3, 3:] = 0.; C[3:, :3] = 0.
        np.testing.assert_allclose(velocity_gain(C, np.ones((2, 4)), .1), 0., atol=1e-12)
        np.testing.assert_allclose(velocity_gain(np.eye(3), np.zeros((2, 0)), .1), 0.)

    def test_more_visible_information_cannot_increase_covariance(self):
        C = self.covariance(5)
        P = conditional_velocity_covariance(C, np.array([np.zeros(5), np.full(5, .3), np.ones(5)]), .4)
        for i in range(2):
            self.assertGreaterEqual(np.linalg.eigvalsh(P[i]-P[i+1]).min(), -1e-12)

    def test_rotating_velocity_or_bearing_bases_does_not_change_gain(self):
        C = self.covariance(4)
        A = np.eye(len(C)); A[:3, :3] = np.linalg.qr(self.rng.normal(size=(3, 3)))[0]
        for i in range(4):
            A[3+2*i:5+2*i, 3+2*i:5+2*i] = np.linalg.qr(self.rng.normal(size=(2, 2)))[0]
        w = self.rng.uniform(size=(7, 4))
        np.testing.assert_allclose(velocity_gain(C, w, .3), velocity_gain(A@C@A.T, w, .3), atol=1e-12)

    def test_velocity_units_do_not_change_gain(self):
        C = self.covariance(3); A = np.eye(len(C)); A[:3, :3] *= 1000.
        w = self.rng.uniform(size=(4, 3))
        np.testing.assert_allclose(velocity_gain(C, w, .5), velocity_gain(A@C@A.T, w, .5), atol=1e-12)

    def test_rejects_invalid_joint_covariance(self):
        C = np.eye(5); C[0, 3] = C[3, 0] = 2.
        with self.assertRaises(ValueError): conditional_velocity_covariance(C, [[1.]], .1)
        with self.assertRaises(ValueError): conditional_velocity_covariance(np.eye(5), [[-1.]], .1)


if __name__ == '__main__':
    unittest.main()
