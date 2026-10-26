import unittest
from unittest.mock import patch
import numpy as np
from committed_tracking_view import CommittedTrackingView
from view_information_ablation import InformationAblationView
from lidar_conditioned_velocity_gain import fused_position_gain
from run_weathered_view_array import paced_guarded_sources
from run_information_ablation_array import ablation_sources


class AblationTests(unittest.TestCase):
    def test_only_information_inputs_change(self):
        args = (object(), np.eye(6)*17, np.eye(6)*2, np.arange(6),
                object(), object(), object(), object(), .2, object(), 1.)
        for mode in ['visual_only', 'lidar_conditioned']:
            view = object.__new__(InformationAblationView)
            view.cfg = dict(view_information_mode=mode)
            with patch.object(CommittedTrackingView, 'choose', return_value=('command', {})) as delegate:
                command, report = view.choose(*args)
            actual = delegate.call_args.args
            for i in [0,4,5,6,7,8,9,10]: self.assertIs(actual[i], args[i])
            if mode == 'visual_only':
                np.testing.assert_array_equal(actual[1], np.zeros((6,6)))
                np.testing.assert_array_equal(actual[2], np.eye(6))
                np.testing.assert_allclose(actual[3], [0,0,0,1/3,1/3,1/3])
            else:
                for i in [1,2,3]:self.assertIs(actual[i], args[i])
            self.assertEqual(command, 'command');self.assertEqual(report['view_information_mode'], mode)

    def test_conditioning_changes_direction_ranking(self):
        # Feature A constrains x more; B constrains y more. Both covariances valid.
        C=np.eye(7);C[0,3]=C[3,0]=.8;C[1,5]=C[5,1]=.6
        views=np.array([[1.,0.],[0.,1.]])
        prior=np.eye(6)*.01
        visual=fused_position_gain(prior,C,views,.1,1.)
        lidar=prior+np.diag([1,1,1,1000,0,1])
        conditioned=fused_position_gain(lidar,C,views,.1,1.)
        self.assertGreater(visual[0],visual[1])
        self.assertLess(conditioned[0],conditioned[1])

    def test_real_policy_pass_through_and_visual_independence(self):
        from test_predictive_tracking_view import PredictiveTrackingTest
        fixture=PredictiveTrackingTest();fixture.setUp();r=fixture.r;n=len(r[0])
        def controller(cls, mode):
            v=cls(fixture.fixture.fisher,dict(fixture.cfg,view_information_mode=mode))
            v.set_joint_information(r[0],np.eye(3+2*n))
            v.set_joint_geometry(r[0],r[1],np.array([.5,0,0]),np.eye(3+3*n))
            return v
        old=controller(CommittedTrackingView,'lidar_conditioned')
        new=controller(InformationAblationView,'lidar_conditioned')
        a,ra=fixture.fixture.call(r,old);b,rb=fixture.fixture.call(r,new)
        np.testing.assert_array_equal(a,b);rb.pop('view_information_mode');self.assertEqual(ra,rb)
        # Exercise actual predictive scoring and fallback through choose with different H.
        args=(r,np.eye(6),np.eye(6),np.ones(6),np.eye(3),np.zeros(3),np.zeros(2),np.array([.5,0,0]),.2,np.zeros(3),1.)
        a,ra=controller(InformationAblationView,'visual_only').choose(*args)
        varied=list(args);varied[1]=np.diag([99,2,3,.0001,1e6,44]);varied[2]=np.eye(6)[:,::-1];varied[3]=np.arange(6)
        b,rb=controller(InformationAblationView,'visual_only').choose(*varied)
        np.testing.assert_array_equal(a,b);self.assertEqual(ra,rb)

    def test_owner_and_pacer_are_unchanged(self):
        old,pacer,_=paced_guarded_sources();new,other,_=ablation_sources()
        self.assertEqual(pacer,other)
        self.assertEqual(new,old.replace("HERE/'weathered_planner_node.py'","HERE/'ablation_planner_node.py'"))

if __name__=='__main__':unittest.main()
