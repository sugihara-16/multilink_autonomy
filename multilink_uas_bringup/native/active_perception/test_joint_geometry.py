import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from joint_geometry import decode,subset,advance


class JointGeometryTest(unittest.TestCase):
    def fixture(self):
        a=np.r_[2.,123456.,2.,.5,.1,0.,7.,2.,3.,4.,5.,3.,2.,1.,np.eye(9).ravel()]
        return a

    def test_decode_and_subset_preserve_identity_order(self):
        stamp,ids,p,v,C=decode(self.fixture());self.assertEqual(stamp,123456)
        np.testing.assert_array_equal(ids,[7,5]);np.testing.assert_array_equal(v,[.5,.1,0.])
        C=np.arange(81).reshape(9,9);columns=[0,1,2,6,7,8,3,4,5]
        np.testing.assert_array_equal(subset(ids,C,[5,7]),C[np.ix_(columns,columns)])
        with self.assertRaises(ValueError):subset(ids,C,[8])

    def test_malformed_messages_rejected(self):
        for index,value in [(0,1),(1,.3),(2,26),(6,-1),(10,7)]:
            a=self.fixture();a[index]=value
            with self.assertRaises(ValueError):decode(a)
        with self.assertRaises(ValueError):decode(self.fixture()[:-1])

    def test_sequential_frame_advances_match_one_combined_motion(self):
        p=np.array([[5.,2.,1.],[6.,-2.,0.]]);v=np.array([.5,-.1,.1]);rng=np.random.default_rng(56);B=rng.normal(size=(9,9));C=B@B.T
        R1=Rotation.from_rotvec([.1,-.2,.3]).as_matrix();R2=Rotation.from_rotvec([-.2,.1,.3]).as_matrix()
        p1,v1,C1=advance(p,v,C,R1,.1);p2,v2,C2=advance(p1,v1,C1,R2,.2)
        p3,v3,C3=advance(p,v,C,R1@R2,.3)
        np.testing.assert_allclose(p2,p3,atol=1e-14);np.testing.assert_allclose(v2,v3,atol=1e-14);np.testing.assert_allclose(C2,C3,atol=1e-13)


if __name__=='__main__':unittest.main()
