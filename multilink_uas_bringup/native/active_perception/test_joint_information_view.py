import unittest
import numpy as np
from joint_information_view import JointInformationView, joint_gain
from spatial_parallax_fixation import SpatialParallaxFixation
from test_spatial_parallax_fixation import SpatialMotionTest


class JointGainTest(unittest.TestCase):
    def test_complementary_constraints_beat_redundant_ones(self):
        # Both alternatives have the same trace; observing an unobserved axis
        # reduces joint uncertainty more than repeating the same direction.
        parts = np.zeros((3, 6, 6))
        parts[0, 0, 0] = parts[1, 0, 0] = parts[2, 3, 3] = 10.
        scores = joint_gain(np.eye(6), parts, np.array([[1., 1., 0.], [1., 0., 1.]]))
        self.assertGreater(scores[1], scores[0])

    def test_pose_units_and_basis_do_not_change_gain(self):
        rng = np.random.default_rng(22)
        A = rng.normal(size=(6, 6)); prior = A.T@A+np.eye(6)
        J = rng.normal(size=(5, 2, 6)); parts = J.transpose(0, 2, 1)@J
        weights = rng.uniform(size=(4, 5))
        change = np.diag([.1, .1, .1, 100., 100., 100.])
        np.testing.assert_allclose(joint_gain(prior, parts, weights),
            joint_gain(change.T@prior@change, change.T@parts@change, weights), rtol=1e-12)

    def test_coupled_rotation_translation_is_not_scored_as_independent(self):
        prior = np.eye(6);prior[0, 3] = prior[3, 0] = .95
        J = np.array([[1., 0., 0., -1., 0., 0.], [1., 0., 0., 1., 0., 0.]])
        scores = joint_gain(prior, J[:, :, None]*J[:, None, :], np.eye(2))
        self.assertGreater(scores[0], scores[1])
        np.testing.assert_allclose(scores, np.log([41., 1.+2./1.95]), rtol=1e-12)


class JointViewTest(unittest.TestCase):
    def setUp(self):
        fixture = SpatialMotionTest();fixture.setUp()
        self.fisher, self.cfg = fixture.fisher, dict(fixture.cfg)
        self.controller = JointInformationView(self.fisher, self.cfg)
        self.record = fixture.vertical_record

    def call(self, record, controller=None, current=None, omega=None):
        return (controller or self.controller).choose(record, np.eye(6), np.eye(6), np.ones(6),
            np.eye(3), np.zeros(3), np.zeros(2) if current is None else current,
            np.array([.5, 0., 0.]), .2, np.zeros(3) if omega is None else omega, 2.)

    def mature(self):
        r = list(self.record());r[2][:] = np.eye(3)*.001
        r = tuple(np.concatenate([x, x[:1]], axis=0) for x in r);r[0][-1] = 3
        return r

    def test_low_quality_uses_unchanged_observed_parallax_fallback(self):
        r = self.record();a, da = self.call(r)
        b, db = self.call(r, SpatialParallaxFixation(self.fisher, self.cfg))
        np.testing.assert_array_equal(a, b);self.assertEqual(da, db)

    def test_still_visible_features_do_not_induce_unnecessary_slew(self):
        r = self.mature();r[4][:] = 0.
        c, d = self.call(r)
        np.testing.assert_array_equal(c, [0., 0.]);self.assertEqual(d['reason'], 'joint_information_hold')

    def test_leaving_image_selects_retaining_motion(self):
        r = self.mature();r[1][:, 2] = 1.30
        r[4][:] = [0., 0., .45]
        c, d = self.call(r)
        self.assertLess(c[1], 0.)
        self.assertEqual(d['reason'], 'joint_information_retention')
        self.assertGreater(d['selected_score'], d['hold_score'])
        self.assertGreater(d['retention_selected_future'][-1], d['retention_hold_future'][-1])

    def test_boundaries_rate_and_retention_are_respected(self):
        r = self.mature();current = np.deg2rad([49., -29.])
        from core import camera_rotation
        rays = np.array([[-.2, -.1, 3.], [.2, .1, 3.], [.3, -.2, 3.], [-.3, .2, 3.]])
        r[1][:] = rays@camera_rotation(np.eye(3), *current).T
        c, d = self.call(r, current=current, omega=np.array([0., .8, -1.]))
        lo, hi = self.controller.motion.limits()
        self.assertTrue(np.all(c >= lo) and np.all(c <= hi))
        self.assertLessEqual(np.linalg.norm(c-current), self.cfg['max_speed']*.2+1e-12)
        self.assertTrue(np.all(np.array(d['retention_selected_future']) >=
                               self.cfg['retain_fraction']*np.array(d['retention_hold_future'])-1e-12))

    def test_input_is_not_changed(self):
        r = self.mature();before = [x.copy() for x in r]
        self.call(r)
        for old, new in zip(before, r):np.testing.assert_array_equal(old, new)

    def test_unseen_mature_points_do_not_bypass_fallback(self):
        r = self.mature();r[1][:, 0] *= -1.
        c, d = self.call(r)
        self.assertFalse(d['reason'].startswith('joint_information'))
        np.testing.assert_array_equal(c, [0., 0.])


class TranslationViewTest(JointViewTest):
    def setUp(self):
        super().setUp()
        self.cfg['fixation_translation_information'] = True
        self.controller = JointInformationView(self.fisher, self.cfg)

    def test_translation_objective_is_selected(self):
        r = self.mature()
        _, d = self.call(r)
        self.assertEqual(d['selection_score_kind'], 'predicted_marginal_position_log_determinant_gain')


if __name__ == '__main__':unittest.main()
