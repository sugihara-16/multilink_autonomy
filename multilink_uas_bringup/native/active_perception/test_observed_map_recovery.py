"""Check recovery when a geometry map exists but the estimator loses its tracks."""
import unittest
from pathlib import Path
import numpy as np
import yaml
from core import Fisher, Planner, camera_rotation

HERE = Path(__file__).resolve().parent


class RecoveryTests(unittest.TestCase):
    def test_current_tracks_control_recovery_even_with_a_populated_map(self):
        cfg = yaml.safe_load((HERE/'config.yaml').read_text())
        cfg.update(yaml.safe_load((HERE/'tracking_aware_v58.yaml').read_text()))
        cfg['axis_max_speed'] = np.full(2, cfg['max_speed']/np.sqrt(2)).tolist()
        current = np.array([.4, .5])
        points = np.array([[x,y,5.] for x in [-.5,.5] for y in [-.4,-.2,.2,.4]])
        points = points @ camera_rotation(np.eye(3), *current).T
        covariance = np.tile(np.eye(3)*.001, (len(points),1,1))
        views = points/np.linalg.norm(points,axis=1)[:,None]
        args = (points,covariance,views,np.eye(6),np.eye(6),np.ones(6),
                np.eye(3),np.zeros(3),current,np.zeros(3),.2)
        fisher = Fisher(HERE.parent.parents[3]/'build/active_perception/libfif_active.so')
        planner = Planner(fisher,cfg)
        for _ in range(4):
            command,report = planner.choose(*args,track_points=np.empty((0,3)))
            self.assertEqual(report['reason'],'no_weak_direction_or_map')
            np.testing.assert_array_equal(command,current)
        command,report = planner.choose(*args,track_points=np.empty((0,3)))
        self.assertEqual(report['reason'],'recover_forward_no_map')
        self.assertLess(np.linalg.norm(command),np.linalg.norm(current))
        self.assertLessEqual(np.linalg.norm(command-current),cfg['max_speed']*.2+1e-12)
        self.assertTrue(np.all(np.abs(command-current)<=np.array(cfg['axis_max_speed'])*.2+1e-12))
        _,report = planner.choose(*args,track_points=points)
        self.assertEqual(report['reason'],'evaluated')
        _,report = planner.choose(*args,track_points=np.empty((0,3)))
        self.assertEqual(report['reason'],'no_weak_direction_or_map')
        old = Planner(fisher,dict(cfg,allow_map_recovery=True))
        for _ in range(6):
            _,report = old.choose(*args,track_points=np.empty((0,3)))
        self.assertEqual(report['reason'],'evaluated')


if __name__ == '__main__':
    unittest.main()
