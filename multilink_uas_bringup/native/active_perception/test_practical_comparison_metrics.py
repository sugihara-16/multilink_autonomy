import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from practical_comparison_metrics import evaluate


def poses(rate, scale=1., frame=None):
    t=np.arange(round(4*rate)+1)/rate
    p=np.column_stack((scale*t, np.zeros_like(t), np.zeros_like(t)))
    r=Rotation.identity(len(t))
    if frame is not None:
        p=frame.apply(p)+[8,-4,2];r=frame*r
    return np.column_stack((t,p,r.as_quat()))


class Metrics(unittest.TestCase):
    def test_different_rates_and_rigid_frame(self):
        _,e,_,m=evaluate(poses(100),poses(20,frame=Rotation.from_euler('z',.7)),0,4)
        self.assertEqual(m['samples'],39)
        np.testing.assert_allclose(e,0,atol=1e-12)
    def test_scale_error_is_not_fitted_away(self):
        t,e,_,m=evaluate(poses(100),poses(20,scale=2),0,4)
        np.testing.assert_allclose(e[:,0],t-t[0],atol=1e-12)
        self.assertAlmostEqual(m['position_rmse_m'],np.sqrt(np.mean((t-t[0])**2)))
    def test_missing_tail_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'Incomplete'):
            evaluate(poses(100),poses(20)[:-10],0,4)
    def test_missing_middle_is_rejected(self):
        a=poses(20);a=np.delete(a,np.arange(20,30),axis=0)
        with self.assertRaisesRegex(ValueError,'Pose gap'):
            evaluate(poses(100),a,0,4)
    def test_conflicting_duplicate_is_rejected(self):
        a=poses(20);a=np.insert(a,20,a[20],axis=0);a[20,1]+=1
        with self.assertRaisesRegex(ValueError,'Conflicting'):
            evaluate(poses(100),a,0,4)

if __name__=='__main__':unittest.main()
