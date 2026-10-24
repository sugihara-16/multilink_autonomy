import unittest
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from motion_view_planner import motion_information,spatial_budget,Planner
from core import Fisher,camera_rotation,skew

class MotionViewTests(unittest.TestCase):
 def test_landmark_jacobian_of_bearing_difference(self):
  p=np.array([5.,2.,1.]);v=np.array([.5,.1,0.]);h=2.
  def normalized(q):return q/np.linalg.norm(q)
  def f(q):return normalized(q-v*h)-normalized(q)
  B=lambda q:(np.eye(3)-np.outer(normalized(q),normalized(q)))/np.linalg.norm(q)
  D=B(p-v*h)-B(p)
  numeric=np.column_stack([(f(p+np.eye(3)[i]*1e-5)-f(p-np.eye(3)[i]*1e-5))/2e-5 for i in range(3)])
  np.testing.assert_allclose(D,numeric,atol=1e-10)
 def test_psd_and_uncertainty_monotonicity(self):
  rng=np.random.default_rng(7);p=rng.uniform([3,-2,-2],[8,2,2],(20,3));C=np.tile(np.eye(3)*.02,(20,1,1))
  a=motion_information(p,C,np.zeros(3),[.5,0,0],2.,.007)
  b=motion_information(p,C*100,np.zeros(3),[.5,0,0],2.,.007)
  self.assertGreater(np.linalg.eigvalsh(a).min(),-1e-8)
  self.assertGreater(np.linalg.eigvalsh(a-b).min(),-1e-8)
 def test_rotation_equivariance(self):
  p=np.array([[5.,2.,1.],[3.,-2.,2.]]);C=np.tile(np.diag([.1,.2,.3]),(2,1,1));t=np.array([.1,.2,0.]);v=np.array([.5,0,.1]);R=Rotation.from_euler('xyz',[.3,.5,.7]).as_matrix()
  a=motion_information(p,C,t,v,2.,.007);b=motion_information(p@R.T,R@C@R.T,R@t,R@v,2.,.007)
  A=np.zeros((6,6));A[:3,:3]=A[3:,3:]=R
  np.testing.assert_allclose(b,A@a@A.T,atol=1e-8)
 def test_pose_jacobian_and_length_units(self):
  p=np.array([5.,2.,1.]);v=np.array([.5,.1,0.]);t=np.array([.18,0,-.025]);h=2.
  q=p-v*h-t;n=q/np.linalg.norm(q);B=(np.eye(3)-np.outer(n,n))/np.linalg.norm(q)
  J=np.column_stack([B@skew(p-v*h),-B])
  def f(x):
   q=Rotation.from_rotvec(-x[:3]).apply(p-v*h)-t-x[3:];return q/np.linalg.norm(q)
  numeric=np.column_stack([(f(np.eye(6)[i]*1e-5)-f(-np.eye(6)[i]*1e-5))/2e-5 for i in range(6)])
  np.testing.assert_allclose(J,numeric,atol=1e-10)
  C=np.diag([.1,.2,.3])[None];A=np.diag([1,1,1,.01,.01,.01])
  a=motion_information(p[None],C,t,v,h,.007)
  b=motion_information(p[None]*100,C*10000,t*100,v*100,h,.007)
  np.testing.assert_allclose(b,A@a@A.T,atol=1e-8)
 def test_spatial_budget_duplicate_invariance(self):
  c={'tan_half_hfov':1.7,'tan_half_vfov':1.3,'feature_budget':25}
  R=camera_rotation(np.eye(3),0,0)[None];p=np.array([[5.,0.,0.],[5.,2.,0.]])
  a=spatial_budget(p,np.zeros(3),R,np.ones((1,2),bool),np.ones((1,2)),c)
  b=spatial_budget(np.repeat(p,3,axis=0),np.zeros(3),R,np.ones((1,6),bool),np.ones((1,6)),c)
  np.testing.assert_allclose(a,[[b[0,:3].sum(),b[0,3:].sum()]])
  self.assertTrue(np.all((b>=0)&(b<=1)));self.assertLessEqual(b.sum(),25)
 def test_candidate_limits_and_retention(self):
  here=Path(__file__).parent;c=yaml.safe_load((here/'config.yaml').read_text());c.update(yaml.safe_load((here/'motion_view_v1.yaml').read_text()));c.update(limits_min=[-.872,-1.57],limits_max=[.872,1.57])
  p=np.array([[5.,y,z] for y in [-3.,-1.,1.,3.] for z in [-1.,1.]]);C=np.tile(np.eye(3)*.01,(len(p),1,1));unit=p/np.linalg.norm(p,axis=1)[:,None]
  planner=Planner(None,c);cur=np.zeros(2)
  for _ in range(30):
   cmd,r=planner.choose(p,C,unit,np.diag([100]*3+[.01,100,100]),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),cur,np.array([.5,0,0]),.2,track_points=p)
   self.assertLessEqual(np.linalg.norm(cmd-cur),c['max_speed']*.2+1e-10)
   self.assertTrue(np.isfinite(cmd).all());self.assertGreaterEqual(r['selected']['retained'],c['retain_fraction']*r['current_tracks']-1e-8)
   cur=cmd
if __name__=='__main__':unittest.main()
