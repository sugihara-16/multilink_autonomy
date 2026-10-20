import unittest
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from image_geometry_map import depth_at_pixels,ImageGeometryMap,triangulate,triangulate_many
from view_motion import RotationHistory


class GeometryTest(unittest.TestCase):
    def setUp(self):
        self.K=np.array([[250.,0,240],[0,250,135],[0,0,1.]])
        u,v=np.meshgrid(np.arange(0,481,5),np.arange(0,271,5))
        self.cloud=np.c_[(u.ravel()-240)/250*8,(v.ravel()-135)/250*8,np.full(u.size,8.)]

    def test_plane_and_frame_transform(self):
        uv=np.array([[120.,90.],[350,200]])
        R=Rotation.from_euler('xyz',[.1,-.4,.8]).as_matrix();t=np.array([.2,.1,-.1])
        p,C=depth_at_pixels(uv,self.cloud@R.T+t,R,t,self.K)
        expected=np.c_[(uv[:,0]-240)/250*8,(uv[:,1]-135)/250*8,np.full(2,8.)]@R.T+t
        np.testing.assert_allclose(p,expected,atol=1e-10)
        self.assertTrue(np.all(np.linalg.eigvalsh(C)>0))

    def test_missing_depth_is_not_invented(self):
        p,C=depth_at_pixels([[100,100]],self.cloud[:3],np.eye(3),np.zeros(3),self.K)
        self.assertTrue(np.isnan(p).all())

    def test_optical_tracks_keep_ids(self):
        rng=np.random.default_rng(71);gray=np.zeros((270,480),np.uint8)
        for x,y in rng.integers([20,20],[460,250],size=(100,2)):cv2.rectangle(gray,(int(x),int(y)),(int(x)+5,int(y)+5),255,-1)
        m=ImageGeometryMap(self.K)
        ids,p,C,health=m.update(gray,self.cloud,np.eye(3),np.zeros(3),30.)
        next_ids,*_=m.update(gray,self.cloud,np.eye(3),np.zeros(3),30.)
        self.assertGreater(len(ids),50);np.testing.assert_array_equal(ids,next_ids)

    def test_temporal_triangulation_without_lidar_overlap(self):
        point=np.array([1.,.5,5.]);a=np.zeros(3);b=np.array([.8,0,0])
        da=(point-a)/np.linalg.norm(point-a);db=(point-b)/np.linalg.norm(point-b)
        p,C=triangulate(a,da,b,db,2./250,30.)
        np.testing.assert_allclose(p,point,atol=1e-12);self.assertTrue(np.all(np.linalg.eigvalsh(C)>0))
        self.assertIsNone(triangulate(a,da,a,db,2./250,30.))
        self.assertIsNone(triangulate(a,da,b,da,2./250,30.))

    def test_batched_triangulation_matches_independent_ray_solution(self):
        rng=np.random.default_rng(42);points=rng.uniform([-3,-2,3],[3,2,10],(500,3))
        anchors=rng.normal(0,.2,(500,3));origin=np.array([1.,0,0])
        first=points-anchors;first/=np.linalg.norm(first,axis=1)[:,None]
        rays=points-origin;rays/=np.linalg.norm(rays,axis=1)[:,None]
        p,C=triangulate_many(anchors,first,origin,rays,.004,30.)
        good=np.isfinite(p).all(axis=1);self.assertGreater(good.sum(),400)
        reference=[]
        for a,d0,d1 in zip(anchors[good],first[good],rays[good]):
            depths=np.linalg.lstsq(np.column_stack([d0,-d1]),origin-a,rcond=None)[0]
            reference.append((a+depths[0]*d0+origin+depths[1]*d1)*.5)
        np.testing.assert_allclose(p[good],reference,atol=2e-11)

    def test_image_sequence_recovers_metric_depth_from_translation(self):
        rng=np.random.default_rng(18);gray=np.zeros((270,480),np.uint8)
        for x,y in rng.integers([30,30],[450,240],size=(120,2)):cv2.rectangle(gray,(int(x),int(y)),(int(x)+5,int(y)+5),255,-1)
        m=ImageGeometryMap(self.K);empty=np.empty((0,3))
        for i in range(20):
            distance=.04*i;image=cv2.warpAffine(gray,np.array([[1.,0,-250*distance/8],[0,1.,0]]),(480,270))
            T=np.eye(4);T[0,3]=distance
            ids,p,C,health=m.update(image,empty,np.eye(3),np.zeros(3),30.,T)
        self.assertGreater(health['triangulated_depth'],30)
        self.assertLess(abs(np.median(p[:,2])-8.),.15)
        # A changing camera direction with fixed optical centre is not a
        # translation baseline and must never create metric map points.
        m=ImageGeometryMap(self.K)
        for angle in np.linspace(0,.15,20):
            R=Rotation.from_euler('y',angle).as_matrix()
            image=cv2.warpPerspective(gray,self.K@R.T@np.linalg.inv(self.K),(480,270))
            ids,p,C,health=m.update(image,empty,R,np.zeros(3),30.,np.eye(4))
            self.assertEqual(health['triangulated_depth'],0)


class MotionTest(unittest.TestCase):
    def test_constant_and_causal_bounds(self):
        h=RotationHistory();w=np.array([.1,-.2,.3])
        for t in np.arange(0,1.001,.001):h.append(t,w)
        np.testing.assert_allclose(h.rotation(.1334,.6789),Rotation.from_rotvec(w*(.6789-.1334)).as_matrix(),atol=2e-13)
        np.testing.assert_allclose(h.average_rate(.8),w,atol=1e-13)
        self.assertIsNone(h.rotation(.9,1.1))

    def test_oscillation_does_not_become_constant_turn(self):
        h=RotationHistory()
        for t in np.arange(0,1.001,.001):h.append(t,[0,0,.2*np.sin(2*np.pi*10*t)])
        np.testing.assert_allclose(h.average_rate(.97,.3),np.zeros(3),atol=1e-12)


if __name__=='__main__':unittest.main()
