import unittest
import numpy as np
from report_articulated_dragon import fixed_body_measurements
class FixedBody(unittest.TestCase):
    def test_measured_fixed_body_despite_timer_placeholder(self):
        actual=np.tile([0,1.56,0,1.56,0,1.56],(100,1))
        self.assertTrue(fixed_body_measurements(actual))
        # The discarded timer row is demonstrably not a valid measurement.
        self.assertFalse(fixed_body_measurements(np.vstack([np.zeros(6),actual])))
    def test_real_body_motion_rejected(self):
        actual=np.tile([0,1.56,0,1.56,0,1.56],(100,1));actual[-1,1]-=np.deg2rad(1.1)
        self.assertFalse(fixed_body_measurements(actual))
    def test_missing_and_nonfinite_rejected(self):
        for actual in [np.empty((0,6)),np.zeros((1,6)),np.zeros((5,8)),np.full((5,6),np.nan)]:
            self.assertFalse(fixed_body_measurements(actual))
if __name__=='__main__':unittest.main()
