import tempfile
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from core import Fisher,VisualMap,Planner,adjoint,lidar_model,projected_uncertainty,camera_rotation
HERE=Path(__file__).resolve().parent
WS=HERE.parent.parents[3]

class Tests(unittest.TestCase):
    def setUp(self):self.f=Fisher(WS/'build/active_perception/libfif_active.so')

    def test_official_jacobian_against_perturbed_projection(self):
        R=Rotation.from_euler('xyz',[.2,-.3,.8]).as_matrix();t=np.array([.4,-.2,.1]);p=np.array([[2.,3.,4.]])
        H,J=self.f.information(p,np.zeros((1,3,3)),R,t,.01)
        expected=np.zeros((3,6));eps=1e-6
        for k in range(6):
            values=[]
            for sign in [-1,1]:
                d=np.zeros(6);d[k]=sign*eps
                # (T_W_B Exp(delta) T_B_C)^-1 p_W at T_W_B=identity.
                pc=R.T@(Rotation.from_rotvec(d[:3]).as_matrix().T@(p[0]-d[3:])-t)
                values.append(pc/np.linalg.norm(pc))
            expected[:,k]=(values[1]-values[0])/(2*eps)
        np.testing.assert_allclose(J[0],expected,atol=1e-9)
        np.testing.assert_allclose(H,J[0].T@J[0]/.0001,atol=1e-8)

    def test_map_uncertainty_reduces_information(self):
        p=np.array([[2.,1.,5.]])
        exact,_=self.f.information(p,np.zeros((1,3,3)),np.eye(3),np.zeros(3),.01)
        uncertain,_=self.f.information(p,np.eye(3)[None]*.25,np.eye(3),np.zeros(3),.01)
        self.assertGreater(np.linalg.eigvalsh(exact-uncertain).min(),-1e-9)
        self.assertLess(np.trace(uncertain),np.trace(exact))

    def test_coupling_changes_ranking(self):
        H=np.diag([.01,1.]);u=np.array([[1.],[0.]])
        a=100*np.ones((2,2));b=np.diag([10.,0.])
        self.assertGreater(a[0,0],b[0,0])
        self.assertGreater(projected_uncertainty(H+a,u,[1]),projected_uncertainty(H+b,u,[1]))

    def test_lidar_change_of_origin_preserves_score(self):
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.3,-.8,.4]).as_matrix();T[:3,3]=[.4,-.7,.2]
        H=np.diag([2,3,4,5,6,7]);U=np.eye(6);p=np.linspace(.1,.9,6)
        hb,ub,w=lidar_model(H,U,p,T)
        self.assertAlmostEqual(projected_uncertainty(H,U,1-p),projected_uncertainty(hb,ub,w),places=12)

    def test_persistent_map_no_double_count_and_save(self):
        m=VisualMap(max_age=1)
        for stamp in [1,2,3]:m.update(stamp,[5],[[0,0,5]],[np.eye(3)*.01],np.zeros(3))
        self.assertEqual(len(m.points),1);self.assertEqual(len(m.body_points(np.eye(4))[0]),1)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'map.npz';m.save(path);n=VisualMap();n.load(path)
            np.testing.assert_array_equal(n.body_points(np.eye(4))[0],[[0,0,5]])
        m.update(2000000000,[],[],[],np.zeros(3));self.assertFalse(m.points)

    def test_rate_limit_visibility_and_hold(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text());planner=Planner(self.f,c)
        rng=np.random.default_rng(3);points=rng.uniform([3,-5,-2],[8,5,2],(80,3));C=np.tile(np.eye(3)*.001,(80,1,1))
        views=points/np.linalg.norm(points,axis=1)[:,None]
        args=(points,C,views,np.diag([100]*3+[.01,100,100]),np.eye(6),np.array([0,0,0,1,0,0]),np.eye(3),np.zeros(3),np.zeros(2),np.array([1,0,0]),.2)
        target,report=planner.choose(*args)
        self.assertLessEqual(np.linalg.norm(target),c['max_speed']*.2+1e-10)
        self.assertEqual(report['reason'],'evaluated')
        self.assertTrue(all(x['gain']>=-1e-9 for x in report['candidates']))
        empty=list(args);empty[0]=np.empty((0,3));empty[1]=np.empty((0,3,3));empty[2]=np.empty((0,3))
        target,report=planner.choose(*empty);np.testing.assert_array_equal(target,[0,0])
        self.assertEqual(report['reason'],'no_weak_direction_or_map')

    def test_negative_covariance_rejected(self):
        with self.assertRaises(ValueError):self.f.information(np.array([[1.,2.,3.]]),-np.eye(3)[None],np.eye(3),np.zeros(3),.01)

    def test_tracking_policy_retains_current_features_and_rejects_stale_gain(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text())
        c.update(yaml.safe_load((HERE/'tracking_aware_v1.yaml').read_text()))
        p=np.array([[5.,y,z] for y in [-3.,-1.,1.,3.] for z in [-1.,1.]])
        v=p/np.linalg.norm(p,axis=1)[:,None];C=np.tile(np.eye(3)*.001,(len(p),1,1))
        planner=Planner(self.f,c)
        args=(p,C,v,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),np.zeros(2),np.zeros(3),.2)
        target,report=planner.choose(*args,ages=np.zeros(len(p)))
        self.assertGreaterEqual(report['selected']['retained'],.8*report['candidates'][0]['retained']-1e-9)
        _,stale=planner.choose(*args,ages=np.full(len(p),40.))
        self.assertLess(stale['selected']['gain'],1e-4)

    def test_predicted_turn_keeps_landmarks_in_view(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text());c.update(yaml.safe_load((HERE/'tracking_aware_v2.yaml').read_text()))
        p=np.array([[5.,y,z] for y in [-6.,-6.5,-7.,-7.5] for z in [-1.,1.]])
        views=p/np.linalg.norm(p,axis=1)[:,None];C=np.tile(np.eye(3)*.001,(len(p),1,1))
        target,r=Planner(self.f,c).choose(p,C,views,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),np.zeros(2),np.zeros(3),.2,ages=np.zeros(len(p)),angular_velocity=np.array([0,0,.4]))
        self.assertLess(target[0],0)
        self.assertGreater(r['selected']['predicted_retained'],r['candidates'][0]['predicted_retained'])
        self.assertLessEqual(np.linalg.norm(target),c['max_speed']*.2+1e-10)

    def test_scan_speed_separate_from_body_compensation_and_recovery(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text());c.update(yaml.safe_load((HERE/'tracking_aware_v3.yaml').read_text()));planner=Planner(self.f,c)
        cmd=planner.command(np.zeros(2),np.array([1.,1.]),np.eye(3),np.zeros(3),.2)
        self.assertLessEqual(np.linalg.norm(cmd),c['scan_speed']*.2+1e-10)
        w=np.array([0,0,.2]);cmd=planner.command(np.zeros(2),np.array([-.04,0]),np.eye(3),w,.2)
        np.testing.assert_allclose(cmd,[-.04,0],atol=1e-12)
        args=(np.empty((0,3)),np.empty((0,3,3)),np.empty((0,3)),np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),np.array([.4,.5]),np.zeros(3),.2)
        for _ in range(6):cmd,r=planner.choose(*args)
        self.assertEqual(r['reason'],'recover_forward_no_map');self.assertLess(np.linalg.norm(cmd),np.linalg.norm(args[8]))

    def test_vertical_view_rate_compensation_is_not_singular(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text());c.update(yaml.safe_load((HERE/'tracking_aware_v4.yaml').read_text()))
        c.update(limits_min=[-.9,-np.pi/2],limits_max=[.9,np.pi/2]);p=Planner(self.f,c)
        current=np.array([.3,np.pi/2]);omega=np.array([0.,0.,.2]);target=current+[-.04,0]
        cmd=p.command(current,target,np.eye(3),omega,.2)
        np.testing.assert_allclose(cmd,target,atol=1e-12)
        # Full camera orientation, not just its viewing ray, stays fixed.
        old=camera_rotation(np.eye(3),*current)
        new=Rotation.from_rotvec(omega*.2).as_matrix()@camera_rotation(np.eye(3),*cmd)
        np.testing.assert_allclose(new,old,atol=1e-12)

    def test_gaze_trust_region_keeps_mechanical_limits_and_rate(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text());c.update(yaml.safe_load((HERE/'tracking_aware_v5.yaml').read_text()))
        c.update(limits_min=[-.9,-np.pi/2],limits_max=[.9,np.pi/2]);p=Planner(self.f,c)
        current=np.array([0.,.52]);cmd=p.command(current,np.array([0.,1.]),np.eye(3),np.zeros(3),.2)
        self.assertLessEqual(cmd[1],c['gaze_pitch_limit']);self.assertEqual(c['limits_max'][1],np.pi/2)
        # A controller started with an encoder outside its search region must
        # return gradually, without violating the actuator speed bound.
        current=np.array([0.,.9]);cmd=p.command(current,np.zeros(2),np.eye(3),np.zeros(3),.2)
        self.assertLess(cmd[1],current[1]);self.assertLessEqual(np.linalg.norm(cmd-current),c['max_speed']*.2+1e-12)

    def test_uncertain_depth_does_not_mean_image_tracking_is_lost(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text());c.update(yaml.safe_load((HERE/'tracking_aware_v6.yaml').read_text()))
        c.update(tan_half_hfov=.95,tan_half_vfov=.535,limits_min=[-.873,-1.571],limits_max=[.873,1.571]);p=Planner(self.f,c)
        tracks=np.array([[5.,y,z] for y in [-3.5,-3.,-2.5,-2.] for z in [-1.,1.]])
        args=(np.empty((0,3)),np.empty((0,3,3)),np.empty((0,3)),np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),np.zeros(2),np.zeros(3),.2)
        for _ in range(8):
            cmd,r=p.choose(*args,angular_velocity=np.array([0,0,.4]),track_points=tracks)
        self.assertEqual(r['reason'],'evaluated');self.assertEqual(r['current_tracks'],8)
        self.assertLess(cmd[0],0)
        self.assertTrue(all(abs(v['gain'])<1e-12 for v in r['candidates']))
        # Losing the actual image tracks still triggers a bounded recovery.
        for _ in range(8):cmd,r=p.choose(*args,track_points=np.empty((0,3)))
        self.assertEqual(r['reason'],'recover_forward_no_map')

    def test_retention_uses_reachable_transition_not_distant_endpoint(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text());c.update(yaml.safe_load((HERE/'tracking_aware_v7.yaml').read_text()))
        c.update(limits_min=np.deg2rad([-50,-90]).tolist(),limits_max=np.deg2rad([50,90]).tolist(),tan_half_hfov=.95,tan_half_vfov=.535)
        a=np.deg2rad(np.linspace(-30,30,15))
        points=np.array([[5*np.cos(x),5*np.sin(x),z] for x in a for z in [-.3,.3]])
        cov=np.tile(np.eye(3)*.01,(len(points),1,1));views=points/np.linalg.norm(points,axis=1)[:,None]
        args=(points,cov,views,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),np.zeros(2),np.zeros(3),.2)
        _,new=Planner(self.f,c).choose(*args,track_points=points)
        c['transition_aware']=False
        _,old=Planner(self.f,c).choose(*args,track_points=points)
        match=lambda report:next(r for r in report['candidates'] if np.allclose(r['angles'],np.deg2rad([37.5,0])))
        n,o=match(new),match(old)
        self.assertFalse(o['allowed']);self.assertTrue(n['allowed'])
        self.assertLess(n['retained_target'],.9*len(points));self.assertAlmostEqual(n['retained'],len(points))
        self.assertGreater(n['predicted_retained'],n['retained_target'])
        np.testing.assert_allclose(np.rad2deg(n['command_angles']),[1.,0.],atol=1e-10)
        self.assertTrue(new['retention_constraint_feasible'])

    def test_cached_information_orientation_invariance(self):
        p=np.array([[2.,1.,5.],[-3.,2.,4.]])
        C=np.array([np.diag([.01,.02,.03]),np.diag([.03,.04,.05])])
        H,J,parts=self.f.information(p,C,np.eye(3),np.array([.2,.1,0]),.01,per_point=True)
        other,_=self.f.information(p,C,Rotation.from_euler('xyz',[1.,-.4,.3]).as_matrix(),np.array([.2,.1,0]),.01)
        np.testing.assert_allclose(parts.sum(axis=0),H,atol=1e-10)
        np.testing.assert_allclose(H,other,atol=1e-9)

    def test_current_occluder_rejects_landmarks(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text());planner=Planner(self.f,c)
        p=np.array([[5.,y,z] for y in [-1,1] for z in [-1,1]])
        views=p/np.linalg.norm(p,axis=1)[:,None];C=np.tile(np.eye(3)*.001,(4,1,1))
        _,report=planner.choose(p,C,views,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),
            np.zeros(2),np.zeros(3),.2,obstacles=p/2)
        self.assertTrue(all(v['features']==0 and v['gain']==0 for v in report['candidates']))

    def test_indexed_occlusion_preserves_view_decisions(self):
        c=yaml.safe_load((HERE/'config.yaml').read_text());c.update(yaml.safe_load((HERE/'tracking_aware_v8c.yaml').read_text()))
        rng=np.random.default_rng(76);p=rng.uniform([2,-5,-2],[12,5,2],(80,3))
        views=p/np.linalg.norm(p,axis=1)[:,None];C=np.tile(np.eye(3)*.01,(len(p),1,1))
        obstacles=np.r_[p[:40]*.6,rng.normal(size=(8000,3))*8]
        args=(p,C,views,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),np.zeros(2),np.array([.5,0,0]),.2)
        old,report=Planner(self.f,c).choose(*args,obstacles=obstacles)
        c['indexed_occlusion']=True;new,fast=Planner(self.f,c).choose(*args,obstacles=obstacles)
        np.testing.assert_array_equal(old,new)
        self.assertEqual(report,fast)

    def test_optical_frame_matches_generated_robot(self):
        import sys,json,xml.etree.ElementTree as ET
        sys.path.insert(0,str(HERE.parent))
        from dragon_cameras import link_transforms
        samples=sorted((HERE.parent/'logs').glob('active-gimbal-*/dragon_slam.urdf'))
        if not samples:self.skipTest('Generate the gimbal model first')
        root=ET.parse(samples[-1]).getroot()
        manifest=json.loads((samples[-1].parent/'camera_manifest.json').read_text())
        base=manifest['expected_joints'];T=np.array(manifest['cameras'][0]['T_FC_camera'])
        for yaw,pitch in [(0,0),(.7,-.4),(-1.2,.3)]:
            fk=link_transforms(root,{**base,'active_camera_yaw_joint':yaw,'active_camera_pitch_joint':pitch})
            actual=np.linalg.inv(fk['fc'])@fk['cam_link3_optical']
            np.testing.assert_allclose(actual[:3,:3],camera_rotation(T[:3,:3],yaw,pitch),atol=1e-12)
            np.testing.assert_allclose(actual[:3,3],T[:3,3],atol=1e-12)

if __name__=='__main__':unittest.main()
