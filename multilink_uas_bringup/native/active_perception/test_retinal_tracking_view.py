import unittest
import numpy as np
from test_predictive_tracking_view import PredictiveTrackingTest
from retinal_tracking_view import RetinalTrackingView
from predictive_tracking_view import PredictiveTrackingView


class RetinalTrackingTest(unittest.TestCase):
    def setUp(self):
        f=PredictiveTrackingTest();f.setUp();self.f=f
        self.view=RetinalTrackingView(f.fixture.fisher,f.cfg)
        self.view.set_joint_information(f.r[0],np.eye(3+2*len(f.r[0])))
        self.view.set_joint_geometry(f.r[0],f.r[1],np.array([.5,0.,0.]),np.eye(3+3*len(f.r[0])))

    def test_augmented_family_preserves_all_existing_candidates(self):
        f=self.f;r=f.r;unit=r[1]/np.linalg.norm(r[1],axis=1)[:,None];good=np.ones(len(unit),bool)
        args=(unit,r[4],good,np.eye(3),np.zeros(2),np.array([.1,.2,.3]),.2)
        old=PredictiveTrackingView.candidate_commands(self.view,*args);new=self.view.candidate_commands(*args)
        np.testing.assert_array_equal(new[:len(old)],old)
        self.assertEqual(len(new),2*len(old))
        self.assertTrue(np.all(abs(new)<=np.array(f.cfg['axis_max_speed'])*.2+1e-12))

    def test_selection_uses_forecast_and_original_retention_limits(self):
        target,d=self.f.fixture.call(self.f.r,self.view)
        self.assertTrue(d['selection_score_kind'].startswith('predicted_geometry'))
        self.assertEqual(d['retinal_candidates_added'],len(self.f.r[0])+1)
        self.assertTrue(np.all(np.asarray(d['retention_selected_future'])>=self.f.cfg['retain_fraction']*np.asarray(d['retention_hold_future'])-1e-12))


if __name__=='__main__':unittest.main()
