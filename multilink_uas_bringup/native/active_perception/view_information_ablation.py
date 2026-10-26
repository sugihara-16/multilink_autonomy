"""Remove LiDAR information from view scoring, never from state estimation.

The visual-only arm uses the identical regularizing prior, visual covariance,
forecast, candidates, retention/recovery, and actuator constraints. All scoring
paths (including fallback) see zero LiDAR information and equal translation
weights. The LiDAR-conditioned arm delegates without changing its inputs.
"""
import numpy as np
from committed_tracking_view import CommittedTrackingView


class InformationAblationView(CommittedTrackingView):
    def choose(self, record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now):
        mode = self.cfg['view_information_mode']
        if mode not in ('lidar_conditioned', 'visual_only'):
            raise ValueError('Unknown view information ablation mode')
        if mode == 'visual_only':
            H = np.zeros((6, 6))
            U = np.eye(6)
            weak = np.r_[np.zeros(3), np.ones(3) / 3.]
        command, report = super().choose(record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now)
        report['view_information_mode'] = mode
        return command, report
