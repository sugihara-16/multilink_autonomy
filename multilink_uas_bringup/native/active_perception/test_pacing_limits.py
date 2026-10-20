import unittest
import numpy as np
from pacing_limits import planner_lag_limit


class PacingTest(unittest.TestCase):
    def test_recorded_pause_can_resume_without_future_clock(self):
        # Actual failed flight: all sensors caught up; last command age .136 s.
        # The next 5 Hz decision is not due yet. Old resume limit .1 deadlocked.
        ages={'fixed_vio':.106,'vio':.106,'fixed_lidar':.106,'lidar':.106,'planner':.136}
        limits={k:(planner_lag_limit(True,.2) if k=='planner' else .23 if 'vio' in k else .26) for k in ages}
        self.assertFalse(any(ages[k]>limits[k] for k in ages))
        self.assertTrue(ages['planner']>.1)

    def test_all_healthy_schedule_phases_can_resume(self):
        for period in [.1,.2,.3]:
            self.assertTrue(all(age<=planner_lag_limit(True,period) for age in np.linspace(0,period,1001)))
            self.assertLess(planner_lag_limit(True,period),planner_lag_limit(False,period))
            self.assertLess(planner_lag_limit(False,period),.5)
            self.assertGreater(2*period,planner_lag_limit(False,period))

if __name__=='__main__':unittest.main()
