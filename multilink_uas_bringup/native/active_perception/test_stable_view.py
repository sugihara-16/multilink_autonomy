import unittest
import numpy as np
import test_joint_information_view as fixture
from joint_information_view import JointInformationView
from stable_view import StableView


class StableViewTest(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.JointViewTest()
        self.fixture.setUp()
        self.cfg = dict(self.fixture.cfg)
        self.controller = StableView(self.fixture.fisher, self.cfg)

    def record(self):
        r = self.fixture.mature()
        r[1][:, 2] = [.1, -.1, .2, -.2]
        r[4][:] = [0., .15, 0.]
        return r

    def call(self, record, current=None, omega=None, controller=None):
        return self.fixture.call(record, controller=controller or self.controller,
                                 current=current, omega=omega)

    def test_useful_view_stays_still(self):
        current = np.deg2rad([5., 2.])
        c, d = self.call(self.record(), current=current)
        np.testing.assert_array_equal(c, current)
        self.assertEqual(d['reason'], 'stable_useful_view')

    def test_feature_loss_allows_tracking(self):
        r = self.fixture.mature()
        r[1][:, 2] = 1.3
        r[4][:] = [0., 0., .45]
        c, d = self.call(r)
        self.assertEqual(d['reason'], 'joint_information_retention')
        self.assertLess(c[1], 0.)

    def test_body_rotation_is_included_in_loss_prediction(self):
        c, d = self.call(self.record(), omega=np.array([0., 0., 2.]))
        self.assertNotEqual(d['reason'], 'stable_useful_view')
        self.assertLessEqual(np.linalg.norm(c), self.cfg['max_speed']*.2+1e-12)

    def test_unreliable_depth_does_not_enable_hold(self):
        r = self.record();r[2][:] = np.eye(3)*100.
        c, d = self.call(r)
        expected, report = self.call(r, controller=JointInformationView(self.fixture.fisher, self.cfg))
        np.testing.assert_array_equal(c, expected);self.assertEqual(d, report)

    def test_rotation_only_or_unmeasurable_parallax_does_not_enable_hold(self):
        r = self.record();r[4][:] = 0.
        _, d = self.call(r)
        self.assertNotEqual(d['reason'], 'stable_useful_view')

    def test_young_tracks_do_not_enable_hold(self):
        r = self.record();r[3][:] = .2
        _, d = self.call(r)
        self.assertNotEqual(d['reason'], 'stable_useful_view')

    def test_points_behind_camera_are_not_counted(self):
        r = self.record();r[1][:, 0] *= -1.
        _, d = self.call(r)
        self.assertNotEqual(d['reason'], 'stable_useful_view')

    def test_hold_clears_old_recovery_target(self):
        self.controller.anchors = [30]
        self.controller.recovering = True
        self.controller.missing_since = 0.
        _, d = self.call(self.record())
        self.assertEqual(d['reason'], 'stable_useful_view')
        self.assertFalse(self.controller.recovering)
        self.assertEqual(self.controller.anchors, [])
        self.assertIsNone(self.controller.missing_since)

    def test_no_input_mutation(self):
        r = self.record();old = [v.copy() for v in r]
        self.call(r)
        for before, after in zip(old, r):
            np.testing.assert_array_equal(before, after)


if __name__ == '__main__':
    unittest.main()
