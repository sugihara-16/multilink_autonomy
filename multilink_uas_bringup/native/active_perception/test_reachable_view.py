import unittest
import numpy as np
from reachable_view import reachable_endpoint,future_points

class Reachable(unittest.TestCase):
    def test_trapezoid_needs_braking_time(self):
        p=reachable_endpoint(np.zeros(6),np.ones(6),4.,8.,2.)
        np.testing.assert_allclose(p,1.)
        p=reachable_endpoint(np.zeros(6),np.ones(6)*100,4.,8.,2.)
        np.testing.assert_allclose(p,6.)
        self.assertEqual(p[0]/4.+4./8.,2.)
    def test_short_triangular_motion(self):
        p=reachable_endpoint(np.zeros(6),np.ones(6),4.,8.,.5)
        np.testing.assert_allclose(p,.5)
        self.assertAlmostEqual(2*np.sqrt(p[0]/8),.5)
    def test_coordinated_and_gimbal(self):
        goal=np.array([100.,-50.])
        np.testing.assert_allclose(reachable_endpoint(np.zeros(2),goal,14.,None,2.),[28.,-14.])
    def test_future_frame_translation_rotation(self):
        p=future_points([[3.,0,0]],[1.,0,0],[0,0,np.pi/2],1.)
        np.testing.assert_allclose(p,[[0,-2,0]],atol=1e-12)
    def test_gimbal_diagonal_uses_combined_speed_limit(self):
        p=reachable_endpoint(np.zeros(2),np.array([100.,100.]),14.,None,2.,norm=2)
        self.assertAlmostEqual(np.linalg.norm(p),28.)
        self.assertLess(np.max(abs(p)),28.)
    def test_stationary_is_identity(self):
        p=np.arange(12.).reshape(4,3)
        np.testing.assert_allclose(future_points(p,[0,0,0],[0,0,0],3),p)

if __name__=='__main__':unittest.main()
