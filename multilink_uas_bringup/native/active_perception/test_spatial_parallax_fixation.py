import unittest
import numpy as np
from core import camera_rotation
from fixation import Fixation
from spatial_parallax_fixation import SpatialParallaxFixation
import test_parallax_fixation as baseline


class SpatialYawCompatibility(baseline.ParallaxFallbackTest):
    def setUp(self):
        super().setUp()
        self.controller = SpatialParallaxFixation(self.fisher, self.cfg)


class SpatialMotionTest(unittest.TestCase):
    def setUp(self):
        fixture = baseline.ParallaxFallbackTest()
        fixture.setUp()
        self.fisher = fixture.fisher
        self.cfg = dict(fixture.cfg, fixation_yaw_only=False)
        self.controller = SpatialParallaxFixation(self.fisher, self.cfg)

    step = baseline.ParallaxFallbackTest.step

    def vertical_record(self):
        points = np.array([[3., 0., .8], [3., .1, .9], [3., -.1, .7]])
        unit = points / np.linalg.norm(points, axis=1)[:, None]
        rates = np.tile([0., 0., .15], (3, 1))
        rates -= (rates * unit).sum(axis=1)[:, None] * unit
        return np.arange(3), points, np.tile(np.eye(3)*100., (3, 1, 1)), np.full(3, 2.), rates

    def test_vertical_motion_retained_with_pitch(self):
        record = self.vertical_record()
        command, report = self.step(record)
        self.assertIn('parallax', report['reason'])
        self.assertLess(command[1], 0.)
        self.assertLessEqual(np.linalg.norm(command), self.cfg['max_speed']*.2+1e-12)
        selected = np.isin(record[0], report['anchors'])
        unit = record[1]/np.linalg.norm(record[1], axis=1)[:, None]
        future = unit[selected]+record[4][selected]*.2
        def off_axis(angles):
            pc = future @ camera_rotation(np.eye(3), *angles)
            return np.mean(np.linalg.norm(pc[:, :2], axis=1)/pc[:, 2])
        self.assertLess(off_axis(command), off_axis(np.zeros(2)))

    def test_outside_image_cannot_be_invented_as_pitch_target(self):
        record = list(self.vertical_record())
        record[1][:, 2] = 10.
        command, report = self.step(tuple(record))
        self.assertNotIn('parallax', report['reason'])
        self.assertEqual(self.controller.anchors, [])
        np.testing.assert_array_equal(command, [0., 0.])

    def test_mature_two_axis_behavior_matches_existing_policy(self):
        record = list(self.vertical_record())
        record[2][:] = np.eye(3)*.001
        record = tuple(np.concatenate([x, x[:1]], axis=0) for x in record)
        record[0][-1] = 3
        a, ra = self.step(record)
        b, rb = self.step(record, controller=Fixation(self.fisher, self.cfg))
        np.testing.assert_array_equal(a, b)
        self.assertEqual(ra, rb)

    def test_two_axis_metric_scale_invariance(self):
        record = self.vertical_record()
        a, ra = self.step(record)
        scaled = tuple(x.copy() for x in record)
        scaled[1][:] *= 10.
        scaled[2][:] *= 100.
        b, rb = self.step(scaled, controller=SpatialParallaxFixation(self.fisher, self.cfg))
        np.testing.assert_allclose(a, b, atol=1e-12)
        self.assertEqual(ra['anchors'], rb['anchors'])


if __name__ == '__main__':
    unittest.main()
