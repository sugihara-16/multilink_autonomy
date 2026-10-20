import unittest
from pathlib import Path
import numpy as np
import yaml
from core import Fisher
from fixation import Fixation


class FixationTest(unittest.TestCase):
    def setUp(self):
        here=Path(__file__).parent
        self.cfg=yaml.safe_load((here/'config.yaml').read_text())
        self.cfg.update(yaml.safe_load((here/'tracking_aware_v13.yaml').read_text()))
        # Generated UniPilot calibration overrides the generic DRAGON limits.
        self.cfg.update(limits_min=np.deg2rad([-50.,-90.]).tolist(),limits_max=np.deg2rad([50.,90.]).tolist(),
                        tan_half_hfov=.950451460608803,tan_half_vfov=.5346289465924517)
        self.f=Fixation(Fisher('/root/multilink_autonomy_ws/build/active_perception/libfif_active.so'),self.cfg)

    def step(self,ids,points,ages=None,covs=None,now=1.,current=None):
        n=len(ids);record=(np.array(ids),np.array(points).reshape(-1,3),
            np.tile(np.eye(3)*.01,(n,1,1)) if covs is None else covs,
            np.full(n,2.) if ages is None else np.array(ages))
        return self.f.choose(record,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),
            np.zeros(2) if current is None else current,np.array([.5,0.,0.]),.2,np.zeros(3),now)

    def test_degeneracy_gate_holds_reference_in_well_constrained_scene(self):
        self.f.cfg['fixation_degeneracy_gate']=True
        record=(np.array([1]),np.array([[6.,2.,0.]]),np.array([np.eye(3)*.01]),np.array([2.]))
        for j in range(30):
            cmd,r=self.f.choose(record,np.eye(6),np.eye(6),np.zeros(6),np.eye(3),np.zeros(3),np.zeros(2),np.zeros(3),.2,np.zeros(3),j*.2)
            np.testing.assert_array_equal(cmd,[0.,0.])
            self.assertFalse(r['degeneracy_gate_active'])

    def test_degeneracy_gate_opens_for_sustained_evidence_and_resists_single_drop(self):
        self.f.cfg['fixation_degeneracy_gate']=True
        self.step([1],[[6.,2.,0.]],now=0.)
        self.assertFalse(self.f.gate_active)
        for j in range(1,5):cmd,r=self.step([1],[[6.,2.,0.]],now=j*.2)
        self.assertTrue(self.f.gate_active);self.assertGreater(cmd[0],0.)
        record=(np.array([1]),np.array([[6.,2.,0.]]),np.array([np.eye(3)*.01]),np.array([2.]))
        cmd,r=self.f.choose(record,np.eye(6),np.eye(6),np.zeros(6),np.eye(3),np.zeros(3),np.zeros(2),np.zeros(3),.2,np.zeros(3),1.)
        self.assertTrue(r['degeneracy_gate_active'])
        for j in range(20):
            current=np.deg2rad([30.,-10.])
            cmd,r=self.f.choose(record,np.eye(6),np.eye(6),np.zeros(6),np.eye(3),np.zeros(3),current,np.zeros(3),.2,np.zeros(3),1.2+j*.2)
        self.assertFalse(r['degeneracy_gate_active']);self.assertEqual(self.f.anchors,[])
        self.assertLess(np.linalg.norm(cmd),np.linalg.norm(current))
        self.assertLessEqual(np.linalg.norm(cmd-current),self.cfg['scan_speed']*.2+1e-12)

    def test_visual_quality_can_request_recovery_before_lidar_degeneracy(self):
        self.f.cfg.update(fixation_degeneracy_gate=True,fixation_quality_recovery=True,
            fixation_recover_quality=True,fixation_keep_recovered_view=True,
            fixation_recovery_angles_deg=[0.,60.],gaze_pitch_limit=np.deg2rad(60.))
        record=(np.array([],dtype=int),np.empty((0,3)),np.empty((0,3,3)),np.empty(0))
        def call(rec,current,now):
            return self.f.choose(rec,np.eye(6),np.eye(6),np.zeros(6),np.eye(3),np.zeros(3),current,np.zeros(3),.2,np.zeros(3),now)
        cmd,r=call(record,np.zeros(2),0.)
        np.testing.assert_array_equal(cmd,[0.,0.])
        cmd,r=call(record,np.zeros(2),1.1)
        self.assertFalse(r['degeneracy_gate_active']);self.assertTrue(r['view_control_enabled'])
        self.assertEqual(r['reason'],'fixation_recover_view');self.assertGreater(cmd[1],0.)
        self.assertLessEqual(np.linalg.norm(cmd),self.cfg['scan_speed']*.2+1e-12)
        good=(np.arange(4),np.array([[2.,-.1,-2.],[2.,.1,-2.],[2.,0.,-1.9],[2.,0.,-2.1]]),np.tile(np.eye(3)*.001,(4,1,1)),np.ones(4)*2)
        ref=np.deg2rad([0.,60.])
        call(good,ref,2.)
        cmd,r=call(good,ref,3.1)
        self.assertEqual(r['reason'],'fixation_recovery_completed')
        cmd,r=call(good,ref,3.3)
        np.testing.assert_allclose(cmd,ref,atol=1e-12)
        self.assertFalse(r['view_control_enabled'])
        np.testing.assert_allclose(r['rest_angles'],ref)

    def test_recovery_view_must_fit_planning_limits(self):
        self.f.cfg.update(fixation_recover_quality=True,fixation_recovery_angles_deg=[0.,60.])
        self.step([],[],now=0.)
        with self.assertRaisesRegex(ValueError,'Recovery view exceeds'):
            self.step([],[],now=1.1)

    def test_measured_bearing_sets_target_and_rate_bound(self):
        cmd,r=self.step([1],[[6.,2.,-1.]])
        np.testing.assert_allclose(r['target_angles'],[np.arctan2(2.,6.),np.arctan2(1.,np.sqrt(40.))])
        self.assertGreater(cmd[0],0);self.assertGreater(cmd[1],0)
        self.assertLessEqual(np.linalg.norm(cmd),self.cfg['max_speed']*.2+1e-12)

    def test_keep_established_identity_despite_new_nearer_feature(self):
        self.step([1],[[6.,2.,0.]])
        cmd,r=self.step([1,2],[[6.,1.8,0.],[1.,-.1,0.]],now=1.2)
        self.assertEqual(r['anchors'],[1]);self.assertEqual(r['reason'],'follow_mature_landmarks')
        self.assertGreater(cmd[0],0)

    def test_young_or_uncertain_depth_cannot_start_anchor(self):
        cmd,r=self.step([1,2],[[6.,2.,0.],[6.,-2.,0.]],ages=[.2,2.],covs=np.array([np.eye(3)*.01,np.eye(3)*100.]))
        self.assertEqual(r['reason'],'fixation_wait_for_depth');np.testing.assert_array_equal(cmd,[0.,0.])

    def test_track_loss_wait_then_handover(self):
        self.step([1],[[6.,2.,0.]])
        cmd,r=self.step([2],[[6.,-2.,0.]],now=1.2)
        self.assertEqual(r['reason'],'fixation_wait_for_track')
        cmd,r=self.step([2],[[6.,-2.,0.]],now=1.5)
        self.assertEqual(r['anchors'],[2]);self.assertLess(cmd[0],0)

    def test_unreachable_landmark_is_not_selected(self):
        cmd,r=self.step([1],[[-3.,0.,0.]])
        self.assertEqual(r['anchors'],[]);np.testing.assert_array_equal(cmd,[0.,0.])

    def test_empty_observations_do_not_produce_invalid_target(self):
        cmd,r=self.step([],[])
        self.assertEqual(r['mature_tracks'],0);np.testing.assert_array_equal(cmd,[0.,0.])

    def test_horizontal_fixation_follows_visible_off_axis_feature(self):
        self.cfg.update(fixation_yaw_only=True,fixation_motion_prediction=True)
        self.f=Fixation(self.f.fisher,self.cfg)
        cmd,r=self.step([1],[[6.,2.,-1.]])
        self.assertEqual(r['anchors'],[1]);self.assertGreater(cmd[0],0.)
        self.assertEqual(cmd[1],0.)
        self.assertEqual(r['target_angles'][1],0.)

    def test_horizontal_fixation_does_not_target_unreachable_vertical_feature(self):
        self.cfg['fixation_yaw_only']=True
        self.f=Fixation(self.f.fisher,self.cfg)
        cmd,r=self.step([1],[[6.,2.,-8.]])
        self.assertEqual(r['anchors'],[])
        np.testing.assert_array_equal(cmd,[0.,0.])

    def test_translation_stabilization_is_not_limited_as_exploration(self):
        self.f.cfg['fixation_motion_prediction']=True
        current=np.array([np.arctan2(1.5,2.),0.])
        cmd,r=self.step([1],[[2.,1.5,0.]],current=current)
        # A centered static point must stay centered after a known 0.1m
        # forward translation, even if its bearing moves faster than 5deg/s.
        self.assertAlmostEqual(cmd[0],np.arctan2(1.5,1.9))
        self.assertGreater(np.linalg.norm(cmd-current),self.cfg['scan_speed']*.2)
        self.assertLessEqual(np.linalg.norm(cmd-current),self.cfg['max_speed']*.2)

    def test_visible_anchor_survives_centred_gaze_limit(self):
        self.f.cfg['fixation_retain_visible']=True
        self.step([1],[[6.,0.,-3.]])
        current=np.array([0.,np.deg2rad(30)])
        cmd,r=self.step([1,2],[[6.,0.,-3.6],[6.,0.,0.]],now=1.5,current=current)
        self.assertEqual(r['anchors'],[1])
        self.assertAlmostEqual(r['target_angles'][1],np.deg2rad(30))
        self.assertLessEqual(cmd[1],np.deg2rad(30)+1e-12)

    def test_anchor_outside_actual_image_is_released(self):
        self.f.cfg['fixation_retain_visible']=True
        self.step([1],[[6.,2.,0.]])
        self.step([1,2],[[6.,7.,0.],[6.,0.,0.]],now=1.2)
        cmd,r=self.step([1,2],[[6.,7.,0.],[6.,0.,0.]],now=1.5)
        self.assertEqual(r['anchors'],[2])

    def test_clamped_pitch_does_not_consume_yaw_stabilization_budget(self):
        self.f.cfg.update(fixation_retain_visible=True,fixation_motion_prediction=True)
        self.step([1],[[6.,2.,-3.]])
        cmd,r=self.step([1],[[6.,2.,-5.]],now=1.2,current=np.array([0.,np.deg2rad(30)]))
        self.assertEqual(r['anchors'],[1]);self.assertGreater(cmd[0],np.deg2rad(1.))
        self.assertAlmostEqual(cmd[1],np.deg2rad(30))

    def test_deadband_keeps_a_well_visible_point_without_recentering(self):
        self.f.cfg.update(fixation_deadband=True,fixation_retain_visible=True)
        cmd,r=self.step([1],[[6.,1.,-.5]])
        np.testing.assert_array_equal(cmd,[0.,0.])
        self.assertTrue(r['deadband_hold'])

    def test_deadband_moves_near_edge_and_preserves_inner_view(self):
        from core import camera_rotation
        self.f.cfg.update(fixation_deadband=True,fixation_retain_visible=True)
        point=np.array([6.,3.5,0.])
        cmd,r=self.step([1],[point])
        self.assertGreater(cmd[0],0.)
        self.assertLess(cmd[0],np.arctan2(point[1],point[0]))
        future=point-np.array([.1,0,0])
        pc=future@camera_rotation(np.eye(3),*cmd)
        self.assertLessEqual(abs(pc[0]/pc[2]),.6*self.cfg['tan_half_hfov']+1e-12)
        self.assertLessEqual(np.linalg.norm(cmd),self.cfg['max_speed']*.2+1e-12)

    def test_deadband_can_wait_at_joint_limit_without_replacing_visible_anchor(self):
        self.f.cfg.update(fixation_deadband=True,fixation_retain_visible=True)
        self.step([1],[[6.,2.,-3.]])
        current=np.deg2rad([20.,30.])
        cmd,r=self.step([1],[[6.,2.,-5.]],now=1.2,current=current)
        self.assertEqual(r['anchors'],[1])
        self.assertTrue(np.isfinite(cmd).all())
        self.assertLessEqual(cmd[1],np.deg2rad(30)+1e-12)
        self.assertLessEqual(np.linalg.norm(cmd-current),self.cfg['max_speed']*.2+1e-12)

    def test_invalid_future_point_holds_without_nonfinite_diagnostics(self):
        import json
        self.f.cfg.update(fixation_deadband=True,fixation_retain_visible=True)
        record=(np.array([1]),np.array([[.4,0.,0.]]),np.array([np.eye(3)*.001]),np.array([2.]))
        cmd,r=self.f.choose(record,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),
            np.zeros(2),np.array([2.,0.,0.]),.2,np.zeros(3),1.)
        np.testing.assert_array_equal(cmd,[0.,0.])
        self.assertEqual(r['reason'],'fixation_invalid_motion_prediction')
        json.dumps(r,allow_nan=False)

    def test_quality_loss_waits_then_returns_toward_reference_view(self):
        self.f.cfg['fixation_recover_quality']=True
        current=np.deg2rad([40.,-20.])
        cmd,r=self.step([1,2],[[6.,2.,0.],[6.,1.8,0.]],current=current)
        np.testing.assert_array_equal(cmd,current)
        self.assertEqual(r['reason'],'fixation_wait_for_quality')
        cmd,r=self.step([1,2],[[6.,2.,0.],[6.,1.8,0.]],current=current,now=2.1)
        self.assertEqual(r['reason'],'fixation_recover_forward')
        self.assertLess(np.linalg.norm(cmd),np.linalg.norm(current))
        self.assertLessEqual(np.linalg.norm(cmd-current),self.cfg['scan_speed']*.2+1e-12)

    def test_depth_failure_triggers_recovery_even_with_many_image_tracks(self):
        self.f.cfg['fixation_recover_quality']=True
        current=np.deg2rad([30.,20.]);points=np.tile([6.,1.,0.],(8,1));C=np.tile(np.eye(3)*100.,(8,1,1))
        self.step(range(8),points,current=current,covs=C)
        cmd,r=self.step(range(8),points,current=current,covs=C,now=2.1)
        self.assertEqual(r['reason'],'fixation_recover_forward')
        self.assertEqual(r['tracked_points'],8);self.assertEqual(r['mature_tracks'],0)

    def test_recovery_requires_reference_angle_and_sustained_quality(self):
        self.f.cfg['fixation_recover_quality']=True
        current=np.deg2rad([30.,20.]);self.step([],[],current=current)
        self.step([],[],current=current,now=2.1)
        points=[[6.,1.,0.],[6.,1.1,0.],[6.,.9,.1],[6.,1.,-.1]]
        _,r=self.step(range(4),points,current=current,now=3.)
        self.assertEqual(r['reason'],'fixation_recover_forward')
        _,r=self.step(range(4),points,current=current,now=4.1)
        self.assertEqual(r['reason'],'fixation_recover_forward')
        _,r=self.step(range(4),points,current=np.zeros(2),now=4.3)
        self.assertEqual(r['reason'],'select_mature_landmarks')
        self.assertFalse(self.f.recovering)

    def test_single_bad_sample_does_not_trigger_recovery(self):
        self.f.cfg['fixation_recover_quality']=True
        points=[[6.,1.,0.],[6.,1.1,0.],[6.,.9,.1],[6.,1.,-.1]]
        self.step(range(4),points,now=1.)
        self.step([],[],now=1.2)
        _,r=self.step(range(4),points,now=1.4)
        self.assertFalse(self.f.recovering)
        self.assertEqual(r['reason'],'follow_mature_landmarks')

    def test_observed_motion_is_independent_of_fused_velocity_and_depth_scale(self):
        self.cfg.update(fixation_bearing_motion=True,fixation_motion_prediction=True)
        outputs=[]
        for scale,velocity in [(1.,[.5,0,0]),(100.,[50.,20.,-10.]),(.1,[-100.,10.,20.])]:
            f=Fixation(self.f.fisher,self.cfg)
            point=scale*np.array([[6.,2.,-.4]])
            record=(np.array([1]),point,np.array([np.eye(3)*.001*scale**2]),np.array([2.]),np.array([[-.01,.03,0.]]))
            current=np.array([np.arctan2(2.,6.),np.arctan2(.4,np.sqrt(40.))])
            cmd,_=f.choose(record,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),current,np.array(velocity),.2,np.zeros(3),1.)
            outputs.append(cmd)
        for output in outputs[1:]:np.testing.assert_allclose(output,outputs[0],atol=1e-13)


    def test_fixation_protects_nonanchor_tracks_during_slew(self):
        self.cfg.update(fixation_bearing_motion=True,fixation_motion_prediction=True,
                        fixation_protect_other_tracks=True)
        f=Fixation(self.f.fisher,self.cfg);f.anchors=[1]
        points=np.array([[6.,3.,0.],[6.,-5.1,0.],[6.,-5.04,.1],[6.,-4.98,-.1]])
        record=(np.arange(1,5),points,np.tile(np.eye(3)*.001,(4,1,1)),np.ones(4)*2,np.zeros((4,3)))
        cmd,r=f.choose(record,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),
                       np.zeros(2),np.array([.5,0.,0.]),.2,np.zeros(3),1.)
        self.assertEqual(r['anchors'],[1])
        self.assertLess(r['retention_scale'],1.)
        self.assertLessEqual(np.linalg.norm(cmd),self.cfg['max_speed']*.2)

    def test_bearing_deadband_ignores_metric_scale_and_fused_velocity(self):
        self.cfg.update(fixation_deadband=True,fixation_bearing_motion=True,
                        fixation_deadband_horizon_s=.8,fixation_retain_visible=True)
        outputs=[]
        for scale,velocity in [(1.,[.5,0,0]),(100.,[50.,20.,-10.]),(.1,[-100.,10.,20.])]:
            f=Fixation(self.f.fisher,self.cfg)
            record=(np.array([1]),scale*np.array([[6.,3.,0.]]),
                    np.array([np.eye(3)*.001*scale**2]),np.array([2.]),np.array([[-.05,.1,0.]]))
            cmd,r=f.choose(record,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),
                           np.zeros(2),np.array(velocity),.2,np.zeros(3),1.)
            self.assertGreater(cmd[0],0.)
            outputs.append(cmd)
        for output in outputs[1:]:np.testing.assert_allclose(output,outputs[0],atol=1e-13)

    def test_bearing_deadband_holds_when_future_anchor_has_margin(self):
        self.cfg.update(fixation_deadband=True,fixation_bearing_motion=True,
                        fixation_deadband_horizon_s=.8,fixation_retain_visible=True)
        f=Fixation(self.f.fisher,self.cfg)
        record=(np.array([1]),np.array([[6.,1.,0.]]),np.array([np.eye(3)*.001]),
                np.array([2.]),np.zeros((1,3)))
        cmd,r=f.choose(record,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),
                       np.zeros(2),np.array([100.,0.,0.]),.2,np.zeros(3),1.)
        np.testing.assert_array_equal(cmd,[0.,0.])
        self.assertTrue(r['deadband_hold'])


if __name__=='__main__':unittest.main()
