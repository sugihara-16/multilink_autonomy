import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np
import yaml
from core import camera_rotation
from retinal_tracking_view import RetinalTrackingView
from committed_tracking_view import CommittedTrackingView


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        here = Path(__file__).resolve().parent
        cfg = yaml.safe_load((here / 'config.yaml').read_text())
        cfg.update(yaml.safe_load((here / 'motion_view_v8.yaml').read_text()))
        cfg.update(limits_min=np.deg2rad([-50, -90]).tolist(), limits_max=np.deg2rad([50, 90]).tolist(),
                   axis_max_speed=[.24682683, .24682683], joint_stop_margin=[.0005, .0005], joint_stop_steps=2,
                   tan_half_hfov=.9504514606, tan_half_vfov=.5346289466)
        self.view = CommittedTrackingView(None, cfg)

    def step(self, count, current, now, age=1.):
        optical = np.column_stack([np.linspace(-.1, .1, count), np.zeros(count), np.full(count, 3.)])
        points = optical @ camera_rotation(np.eye(3), *current).T
        record = (np.arange(count), points, np.tile(np.eye(3), (count, 1, 1)), np.full(count, age), np.zeros((count, 3)))
        before = [value.copy() for value in record]
        with patch.object(RetinalTrackingView, 'choose', return_value=(current.copy(), {'reason': 'normal_policy'})) as delegate:
            command, report = self.view.choose(record, np.eye(6), np.eye(6), np.ones(6), np.eye(3), np.zeros(3),
                                               current, np.zeros(3), .2, np.zeros(3), now)
        for old, new in zip(before, record):
            np.testing.assert_array_equal(old, new)
        self.assertLessEqual(np.linalg.norm(command - current), self.view.cfg['max_speed'] * .2 + 1e-10)
        self.assertEqual(delegate.called, not report['committed_recovery_active'])
        return command, report

    def test_closed_loop_recovery_survives_transient_tracks_and_finishes_at_reference(self):
        current = np.deg2rad([40., -70.]);started = False;finished = False
        for i in range(65):
            # Loss, then single-feature reacquisition, then sufficient tracks.
            count = 0 if i < 8 else (1 if i < 20 else 6)
            old = current.copy();current, report = self.step(count, current, i * .2)
            if report['committed_recovery_active']:
                started = True
            elif started:
                self.assertLessEqual(np.linalg.norm(old), np.deg2rad(1.))
                self.assertGreaterEqual(count, 4)
                finished = True
                break
            if started and count < 4:
                self.assertTrue(report['committed_recovery_active'])
        self.assertTrue(started and finished)

    def test_brief_loss_does_not_trigger_recovery(self):
        current = np.deg2rad([10., 20.])
        for i, count in enumerate([0, 0, 0, 0, 6, 6, 6, 6]):
            _, report = self.step(count, current, i * .2)
            self.assertFalse(report['committed_recovery_active'])

    def test_new_features_do_not_count_as_established_tracking(self):
        current = np.zeros(2)
        for i in range(8):
            _, report = self.step(10, current, i * .2, age=.1)
        self.assertEqual(report['recovery_visible_aged_tracks'], 0)
        self.assertTrue(report['committed_recovery_active'])
        for i in range(8, 13):
            _, report = self.step(10, current, i * .2)
            self.assertTrue(report['committed_recovery_active'])
        _, report = self.step(10, current, 2.8)
        self.assertFalse(report['committed_recovery_active'])

    def test_unobserved_preparation_gap_is_not_continuous_loss_evidence(self):
        current = np.deg2rad([10., 20.])
        self.step(0, current, 0.)
        self.step(0, current, .2)
        _, report = self.step(0, current, 30.)
        self.assertFalse(report['committed_recovery_active'])
        for i in range(1, 8):
            _, report = self.step(0, current, 30. + i * .2)
        self.assertTrue(report['committed_recovery_active'])

    def test_healthy_tracking_preserves_real_covariance_based_policy(self):
        import test_predictive_tracking_view as fixtures
        fixture = fixtures.PredictiveTrackingTest();fixture.setUp()
        guarded = CommittedTrackingView(fixture.fixture.fisher, fixture.cfg)
        original = RetinalTrackingView(fixture.fixture.fisher, fixture.cfg)
        record = fixture.r;n = len(record[0])
        for controller in [guarded, original]:
            controller.set_joint_information(record[0], np.eye(3 + 2 * n))
            controller.set_joint_geometry(record[0], record[1], np.array([.5, 0., 0.]), np.eye(3 + 3 * n))
        a, report_a = fixture.fixture.call(record, guarded)
        b, report_b = fixture.fixture.call(record, original)
        np.testing.assert_array_equal(a, b)
        for key, value in report_b.items():
            self.assertEqual(report_a[key], value)
        self.assertFalse(report_a['committed_recovery_active'])

    def test_repeated_sensor_time_does_not_accumulate_loss_duration(self):
        current = np.deg2rad([10., 20.])
        for _ in range(20):
            _, report = self.step(0, current, 1.)
            self.assertFalse(report['committed_recovery_active'])
        _, report = self.step(0, current, 1.2)
        self.assertFalse(report['committed_recovery_active'])


if __name__ == '__main__':
    unittest.main()
