import unittest
import numpy as np
from occlusion_queries import unoccluded


class OcclusionTest(unittest.TestCase):
    def test_original_predicate_random_geometry_and_boundary(self):
        rng=np.random.default_rng(17)
        rays=rng.normal(size=(24000,3));rays/=np.linalg.norm(rays,axis=1)[:,None]
        directions=rng.normal(size=(500,3));directions/=np.linalg.norm(directions,axis=1)[:,None]
        ranges=rng.uniform(.4,30,len(rays));distance=rng.uniform(.4,30,len(directions));eligible=rng.random(len(directions))>.2
        cos=np.cos(np.deg2rad(1.));expected=eligible.copy()
        for i in np.flatnonzero(eligible):
            if np.any(ranges[rays@directions[i]>cos]+.5<distance[i]):expected[i]=False
        np.testing.assert_array_equal(unoccluded(directions,distance,eligible,rays,ranges,cos,.5),expected)
        for delta in [-1e-9,0,1e-9]:
            angle=np.deg2rad(1.)+delta;ray=np.array([[np.cos(angle),np.sin(angle),0.]])
            actual=unoccluded(np.array([[1.,0,0]]),np.array([5.]),np.array([True]),ray,np.array([2.]),cos,.5)
            self.assertEqual(bool(actual[0]),not (ray[0,0]>cos))


if __name__=='__main__':unittest.main()
