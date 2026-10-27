"""Independent kinematic and information checks for large multi-joint moves."""
import sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import Kinematics,NAMES
from dragon_cameras import link_transforms
from articulated_search import Roadmap,visual_information,interpolate_path
URDF=Path('/root/uas_native/logs/dragon-articulated-20261026/flight10m_prepared_v2/dragon_slam.urdf')

class Search(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.k=Kinematics(URDF);cls.road=Roadmap(cls.k)
    def test_cached_fk_matches_independent_original_tree(self):
        rng=np.random.default_rng(124)
        for q in rng.uniform(self.k.lower,self.k.upper,(30,6)):
            f=link_transforms(self.k.root,dict(zip(NAMES,q)))
            expected=np.linalg.inv(f['fc'])@f['camera_optical']
            np.testing.assert_allclose(self.k.camera(q),expected,atol=1e-12)
    def test_reachable_views_cover_all_azimuth_quadrants(self):
        a=self.road.audit();az=np.array(a['azimuth_deg'])
        self.assertGreater(a['reachable'],100)
        for low in [-180,-90,0,90]:self.assertTrue(np.any((az>=low)&(az<low+90)))
    def test_all_route_edges_have_feasible_interiors(self):
        # Include large coordinated changes, not merely endpoints or +/-1deg.
        for i in list(self.road.home_paths)[::12]:
            _,path=self.road.home_paths[i]
            for a,b in zip(path,path[1:]):
                self.assertTrue(all(self.k.valid(q) for q in interpolate_path(a,b,np.deg2rad(2.))))
    def test_information_matches_finite_difference_with_lever_arm(self):
        T=self.k.camera(np.array([.3,1.1,-.2,.6,.2,.8]));p=np.array([[2.,1.,3.]])
        C=np.zeros((1,3,3));sigma=.01
        def bearing(x):
            pp=Rotation.from_rotvec(-x[:3]).apply(p[0]-x[3:]);r=T[:3,:3].T@(pp-T[:3,3]);return r/np.linalg.norm(r)
        eps=1e-6;J=np.column_stack([(bearing(np.eye(6)[i]*eps)-bearing(-np.eye(6)[i]*eps))/(2*eps) for i in range(6)])
        np.testing.assert_allclose(visual_information(p,C,T,sigma)[0],J.T@J/sigma**2,rtol=1e-6,atol=1e-5)

if __name__=='__main__':unittest.main()
