import unittest
import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from core import camera_rotation
from retinal_motion_candidates import bounded_least_squares,bearing_jacobians,candidate_steps


class RetinalMotionTest(unittest.TestCase):
    def test_jacobians_match_finite_differences_including_vertical_views(self):
        rng=np.random.default_rng(55);u=rng.normal(size=(12,3));u/=np.linalg.norm(u,axis=1)[:,None]
        home=Rotation.from_rotvec([.2,-.1,.3]).as_matrix()
        for pitch in [-90.,-89.,0.,89.,90.]:
            angles=np.deg2rad([40.,pitch]);camera,J,R=bearing_jacobians(u,home,angles)
            for k in range(2):
                d=np.eye(2)[k]*1e-6
                numeric=(u@camera_rotation(home,*(angles+d))-u@camera_rotation(home,*(angles-d)))/2e-6
                np.testing.assert_allclose(J[:,:,k],numeric,atol=3e-10)

    def test_box_solver_matches_independent_optimizer(self):
        rng=np.random.default_rng(155)
        for i in range(100):
            J=rng.normal(size=(12,2));b=rng.normal(size=12);lo=-rng.uniform(.01,.1,size=2);hi=rng.uniform(.01,.1,size=2)
            x=bounded_least_squares(J,b,lo,hi);ref=lsq_linear(J,b,bounds=(lo,hi),tol=1e-13).x
            np.testing.assert_allclose(x,ref,atol=1e-9)

    def test_vertical_view_compensates_image_roll(self):
        current=np.deg2rad([0.,-90.]);R=camera_rotation(np.eye(3),*current)
        rays=np.array([[-.2,-.15,1.],[.2,.15,1.],[-.2,.15,1.],[.2,-.15,1.]])@R.T
        rays/=np.linalg.norm(rays,axis=1)[:,None]
        commands=candidate_steps(rays,np.zeros_like(rays),np.eye(3),current,[0.,0.,.1],.2,0.,[-.05,0.],[.05,.05])
        np.testing.assert_allclose(commands[-1],[-.02,0.],atol=1e-12)
        future=Rotation.from_rotvec([0.,0.,-.02]).apply(rays)
        np.testing.assert_allclose(future@camera_rotation(np.eye(3),*(current+commands[-1])),rays@R,atol=1e-12)

    def test_unobservable_central_ray_does_not_spin_at_vertical(self):
        angles=np.deg2rad([20.,-90.]);u=camera_rotation(np.eye(3),*angles)[:,2][None]
        steps=candidate_steps(u,np.zeros_like(u),np.eye(3),angles,[0.,0.,0.],.2,.1,[-.05,0.],[.05,.05])
        np.testing.assert_allclose(steps,0.,atol=1e-15)

    def test_bounds_hold_for_unachievable_motion_and_input_stays_unchanged(self):
        rng=np.random.default_rng(255);u=rng.normal(size=(20,3));u/=np.linalg.norm(u,axis=1)[:,None];before=u.copy()
        steps=candidate_steps(u,rng.normal(size=(20,3)),np.eye(3),np.zeros(2),[2.,-3.,4.],.2,.1,[-.02,-.01],[.04,.03])
        self.assertTrue(np.all(steps>=[-.02,-.01]));self.assertTrue(np.all(steps<=[.04,.03]));np.testing.assert_array_equal(before,u)


if __name__=='__main__':unittest.main()
