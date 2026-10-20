import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from predictive_bearing_covariance import forecast
from joint_velocity_gain import velocity_gain


class PredictiveBearingTest(unittest.TestCase):
    def test_motion_creates_visual_velocity_information(self):
        p=np.array([[5.,0.,0.]]);C=np.eye(6);C[3:,3:]*=.01
        at_zero,*_=forecast(p,np.zeros(3),C,np.zeros(3),np.zeros(3),0.)
        at_one,*_=forecast(p,np.zeros(3),C,np.zeros(3),np.zeros(3),1.)
        self.assertAlmostEqual(velocity_gain(at_zero,np.ones((1,1)),.01)[0],0.)
        self.assertGreater(velocity_gain(at_one,np.ones((1,1)),.01)[0],5.)

    def test_projection_jacobian_by_independent_finite_difference(self):
        p=np.array([[4.,2.,1.],[-2.,5.,.4]]);v=np.array([.5,-.2,.1]);w=np.array([.1,-.2,.3]);t=np.array([.18,0.,-.025]);h=.8
        C,rays,valid,J=forecast(p,v,np.eye(9),t,w,h)
        R=Rotation.from_rotvec(-w*h).as_matrix();u=rays/np.linalg.norm(rays,axis=1)[:,None]
        # Reconstruct the tangent axes from the reported point Jacobian.
        bases=[J[3+2*i:5+2*i,3+3*i:6+3*i]@R.T*np.linalg.norm(rays[i]) for i in range(2)]
        x=np.r_[v,p.ravel()]
        def observation(x):
            r=(x[3:].reshape(2,3)-x[:3]*h)@R.T-t;n=r/np.linalg.norm(r,axis=1)[:,None]
            return np.r_[x[:3],np.concatenate([b@n[i] for i,b in enumerate(bases)])]
        numeric=np.zeros_like(J)
        for i in range(9):
            d=np.zeros(9);d[i]=1e-6
            numeric[:,i]=(observation(x+d)-observation(x-d))/2e-6
        np.testing.assert_allclose(numeric,J,atol=2e-10,rtol=0)

    def test_retains_all_correlations(self):
        rng=np.random.default_rng(731);p=rng.uniform(2.,5.,(4,3));A=rng.normal(size=(15,15));C=A@A.T+np.eye(15)
        result,rays,valid,J=forecast(p,np.array([.5,0.,0.]),C,np.zeros(3),np.array([0.,0.,.2]),.8)
        np.testing.assert_allclose(result,J@C@J.T,atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(result).min(),0.)
        Csmall=C.copy();Csmall[3:,:3]=0.;Csmall[:3,3:]=0.
        other,*_=forecast(p,np.array([.5,0.,0.]),Csmall,np.zeros(3),np.array([0.,0.,.2]),.8)
        self.assertGreater(np.max(abs(result-other)),.1)

    def test_metric_units_do_not_change_gain(self):
        p=np.array([[5.,2.,1.],[4.,-2.,1.]]);v=np.array([.5,0.,0.]);t=np.array([.18,0.,-.025]);C=np.eye(9)
        original,*_=forecast(p,v,C,t,np.zeros(3),.8)
        scaled,*_=forecast(p*1000,v*1000,C*1e6,t*1000,np.zeros(3),.8)
        np.testing.assert_allclose(velocity_gain(original,np.eye(2),.01),velocity_gain(scaled,np.eye(2),.01),atol=1e-12)

    def test_near_optical_center_is_explicitly_excluded(self):
        C,rays,valid,J=forecast(np.array([[.5,0.,0.]]),np.array([.5,0.,0.]),np.eye(6),np.zeros(3),np.zeros(3),1.)
        self.assertFalse(valid[0]);self.assertTrue(np.isfinite(C).all())
        np.testing.assert_array_equal(J[3:],0.)


if __name__=='__main__':unittest.main()
