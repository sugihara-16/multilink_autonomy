import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from window_metrics import local_displacement_error,translation_axis,stats
class TestMetrics(unittest.TestCase):
 def test_frame_and_start_offset_invariance(self):
  p0=np.array([3.,2.,1.]);p1=np.array([8.,2.,1.]);R=Rotation.from_euler('z',1.).as_matrix();offset=np.array([40.,10.,-7.])
  e=local_displacement_error(p0,p1,np.eye(3),R@p0+offset,R@p1+offset,R)
  np.testing.assert_allclose(e,0,atol=1e-12)
 def test_scale_error_retained(self):
  e=local_displacement_error(np.zeros(3),np.array([5.,0,0]),np.eye(3),np.zeros(3),np.array([6.,0,0]),np.eye(3))
  np.testing.assert_allclose(e,[1,0,0])
 def test_weak_axis_rotation(self):
  H=np.diag([1,2,3,.001,1,2]);R=Rotation.from_euler('z',np.pi/2).as_matrix()
  u,eig,ratio=translation_axis(H,R,np.eye(3),np.array([0,1,0]))
  np.testing.assert_allclose(u,[0,1,0],atol=1e-12);self.assertAlmostEqual(ratio,.0005)
 def test_signed_bias_cannot_cancel_rms(self):
  d=stats([-2,2]);self.assertEqual(d['bias'],0);self.assertEqual(d['rmse'],2)
  self.assertAlmostEqual(d['rmse']**2,d['bias']**2+d['std']**2)
if __name__=='__main__':unittest.main()
