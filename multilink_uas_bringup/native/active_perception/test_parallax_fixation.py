import unittest
from pathlib import Path
import numpy as np
import yaml
from core import Fisher
from fixation import Fixation
from parallax_fixation import ParallaxFixation


class ParallaxFallbackTest(unittest.TestCase):
    def setUp(self):
        here = Path(__file__).parent
        self.cfg = yaml.safe_load((here / 'config.yaml').read_text())
        self.cfg.update(yaml.safe_load((here / 'tracking_aware_v38.yaml').read_text()))
        self.cfg.update(limits_min=np.deg2rad([-50., -90.]).tolist(),
                        limits_max=np.deg2rad([50., 90.]).tolist(),
                        tan_half_hfov=.950451460608803, tan_half_vfov=.5346289465924517)
        self.fisher = Fisher('/root/multilink_autonomy_ws/build/active_perception/libfif_active.so')
        self.controller = ParallaxFixation(self.fisher, self.cfg)

    def record(self, covariance=100., age=2., rate=.15):
        points = np.array([[3., 1., 0.], [3., 1.1, .1], [3., -1., .1]])
        unit = points / np.linalg.norm(points, axis=1)[:, None]
        rates = np.tile([0., rate, 0.], (3, 1))
        rates -= (rates * unit).sum(axis=1)[:, None] * unit
        return np.arange(3), points, np.tile(np.eye(3) * covariance, (3, 1, 1)), np.full(3, age), rates

    def step(self, record, now=2., controller=None):
        weak = np.array([0., 0., 0., 1., 0., 0.])
        return (controller or self.controller).choose(record, np.eye(6), np.eye(6), weak,
            np.eye(3), np.zeros(3), np.zeros(2), np.array([.5, 0., 0.]), .2, np.zeros(3), now)

    def test_uncertain_depth_can_be_retained_without_inventing_depth(self):
        record = self.record()
        copies = [x.copy() for x in record]
        command, report = self.step(record)
        self.assertEqual(report['reason'], 'select_observed_parallax')
        self.assertEqual(report['mature_tracks'], 0)
        self.assertGreater(report['provisional_tracks'], 0)
        self.assertGreater(abs(command[0]), 0.)
        self.assertEqual(command[1], 0.)
        self.assertLessEqual(np.linalg.norm(command), self.cfg['max_speed'] * .2 + 1e-12)
        for before, after in zip(copies, record):
            np.testing.assert_array_equal(before, after)

    def test_unknown_metric_depth_does_not_change_fallback_ranking(self):
        a, ra = self.step(self.record(covariance=100.))
        other = ParallaxFixation(self.fisher, dict(self.cfg))
        record = list(self.record(covariance=100.))
        record[1] *= 10.
        record[2] *= 100.
        b, rb = self.step(tuple(record), controller=other)
        np.testing.assert_allclose(a, b, atol=1e-12)
        self.assertEqual(ra['anchors'], rb['anchors'])

    def test_larger_supported_bearing_motion_wins_and_anchor_is_retained(self):
        record = list(self.record())
        record[4][2] *= 3.
        command, report = self.step(tuple(record))
        self.assertEqual(report['anchors'], [2])
        self.assertLess(report['target_angles'][0], 0.)
        # The command also compensates the selected track's measured motion;
        # its sign need not match the instantaneous centering target.
        self.assertLessEqual(abs(command[0]), self.cfg['max_speed'] * .2 + 1e-12)
        record[4][0] *= 10.
        _, report = self.step(tuple(record), now=2.2)
        self.assertEqual(report['anchors'], [2])
        self.assertEqual(report['reason'], 'retain_observed_parallax')

    def test_pure_rotation_or_no_translation_signal_does_not_claim_parallax(self):
        command, report = self.step(self.record(rate=0.))
        self.assertNotIn('parallax', report['reason'])
        self.assertEqual(self.controller.anchors, [])
        np.testing.assert_array_equal(command, [0., 0.])

    def test_young_track_cannot_supply_unmeasured_motion_history(self):
        command, report = self.step(self.record(age=.05))
        self.assertNotIn('parallax', report['reason'])
        np.testing.assert_array_equal(command, [0., 0.])

    def test_remaining_mature_anchor_is_not_discarded_for_low_total_count(self):
        self.controller.anchors = [0]
        record = list(self.record(rate=0.))
        record[2][0] = np.eye(3) * .001
        command, report = self.step(tuple(record))
        self.assertEqual(report['reason'], 'retain_observed_parallax')
        self.assertEqual(report['anchors'], [0])
        self.assertEqual(report['mature_tracks'], 1)
        self.assertGreater(command[0], 0.)

    def test_no_visible_evidence_falls_back_without_new_anchor(self):
        record = list(self.record())
        record[1][:, 0] *= -1.
        command, report = self.step(tuple(record))
        self.assertNotIn('parallax', report['reason'])
        self.assertEqual(self.controller.anchors, [])
        np.testing.assert_array_equal(command, [0., 0.])

    def test_mature_normal_operation_is_identical(self):
        record = self.record(covariance=.001)
        # Four independent features meet the original recovery-quality rule.
        record = tuple(np.concatenate([x, x[:1]], axis=0) for x in record)
        record[0][-1] = 3
        a, ra = self.step(record)
        b, rb = self.step(record, controller=Fixation(self.fisher, dict(self.cfg)))
        np.testing.assert_array_equal(a, b)
        self.assertEqual(ra, rb)

    def test_empty_frame_is_safe(self):
        record = np.empty(0, int), np.empty((0, 3)), np.empty((0, 3, 3)), np.empty(0), np.empty((0, 3))
        command, report = self.step(record)
        self.assertEqual(report['mature_tracks'], 0)
        np.testing.assert_array_equal(command, [0., 0.])


if __name__ == '__main__':
    unittest.main()
