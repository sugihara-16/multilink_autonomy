"""Select observed visual tracks for information missing from current LiDAR.

Only the planning score changes. The existing filter, sensor noise, feature
retention constraints, and actuator limits are kept. This local score does not
claim to be the actual fused covariance; shared-IMU correlation remains outside
this planning approximation.
"""
import numpy as np
from correlated_tracking_view import CorrelatedTrackingView
from lidar_conditioned_velocity_gain import fused_position_gain


class LidarConditionedTrackingView(CorrelatedTrackingView):
    def information_score(self, H, C, **forecast_context):
        prior=(H+H.T)*.5+np.diag(1/np.square(self.cfg['prior_std']))
        scale=float(self.cfg['fusion_velocity_covariance_scale'])
        horizon=float(self.cfg['fusion_velocity_period_s'])
        if not np.isfinite(scale) or scale<1.:raise ValueError('Invalid fusion covariance scale')
        # Scale velocity and its cross-covariance consistently. Bearing noise
        # and bearing covariance retain their original physical units.
        A=np.ones(len(C));A[:3]=np.sqrt(scale)
        calibrated=A[:,None]*C*A[None,:]
        return (lambda weights:fused_position_gain(prior,calibrated,weights,self.cfg['bearing_sigma'],horizon),
                'lidar_conditioned_position_increment_information_gain')
