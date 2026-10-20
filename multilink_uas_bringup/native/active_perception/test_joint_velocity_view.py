import unittest
import numpy as np
from joint_velocity_view import decode, subset, JointVelocityView
from test_joint_information_view import JointViewTest


class JointCovarianceMessageTest(unittest.TestCase):
    def test_decode_reorder_and_body_rotation(self):
        rng=np.random.default_rng(35); A=rng.normal(size=(7,7));C=A@A.T+np.eye(7)
        data=[1.,12300000000.,2.,77.,2.,3.,4.,22.,-4.,1.,6.,*C.ravel()]
        t,ids,points,cov=decode(data)
        self.assertEqual(t,12300000000);np.testing.assert_array_equal(ids,[77,22])
        np.testing.assert_array_equal(points,[[2,3,4],[-4,1,6]])
        R=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        idx=[0,1,2,5,6,3,4];transform=np.eye(7);transform[:3,:3]=R.T
        np.testing.assert_allclose(subset(ids,cov,[22,77],R),transform@C[np.ix_(idx,idx)]@transform.T)
        np.testing.assert_array_equal(C,cov)

    def test_invalid_or_missing_identity_is_rejected(self):
        with self.assertRaises(ValueError):decode([1.,3.,1.])
        with self.assertRaises(ValueError):subset([1],np.eye(5),[2])
        with self.assertRaises(ValueError):subset([1,1],np.eye(7),[1])


class JointVelocityControllerTest(JointViewTest):
    def setUp(self):
        super().setUp();self.controller=JointVelocityView(self.fisher,self.cfg)

    def mature(self):
        r=super().mature();n=len(r[0]);C=np.eye(3+2*n)
        C[:3,3:]=.05;C[3:,:3]=.05
        self.controller.set_joint_information(r[0],C)
        return r

    def test_missing_covariance_does_not_invent_information(self):
        r=self.mature();del self.controller.joint_covariance
        # Fixation's public wrapper safely holds on missing inputs.
        with self.assertRaises(ValueError):self.controller.score_model(np.eye(6),None,r[0])

    def test_joint_velocity_objective_selected(self):
        r=self.mature();_,d=self.call(r)
        self.assertEqual(d['selection_score_kind'],'conditional_joint_rovio_velocity_log_determinant_gain')


if __name__=='__main__':unittest.main()
