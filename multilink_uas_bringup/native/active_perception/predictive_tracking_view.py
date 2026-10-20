"""Score retained tracks using future velocity/feature geometry covariance.

Candidates and visibility still use measured bearing motion, as in v53. Only
 the score forecasts constant velocity; no unseen features or route commands.
"""
import numpy as np
from lidar_conditioned_tracking_view import LidarConditionedTrackingView
from joint_geometry import subset
from predictive_bearing_covariance import forecast


class PredictiveTrackingView(LidarConditionedTrackingView):
    def set_joint_geometry(self, ids, points, velocity, covariance):
        self.geometry_ids=np.asarray(ids).copy();self.geometry_points=np.asarray(points).copy()
        self.geometry_velocity=np.asarray(velocity).copy();self.geometry_covariance=np.asarray(covariance).copy()

    def information_score(self, H, C, **context):
        if not hasattr(self,'geometry_ids'):raise ValueError('Missing matching geometry covariance')
        selected=context['selected_ids'];lookup={int(v):i for i,v in enumerate(self.geometry_ids)}
        if any(int(v) not in lookup for v in selected):raise ValueError('Geometry feature identity mismatch')
        points=self.geometry_points[[lookup[int(v)] for v in selected]]
        joint=subset(self.geometry_ids,self.geometry_covariance,selected)
        predicted,rays,valid,J=forecast(points,self.geometry_velocity,joint,
            context['camera_position'],context['angular_velocity'],context['horizon'])
        score,kind=super().information_score(H,predicted)
        return (lambda weights:score(weights*valid[None]),'predicted_geometry_'+kind)
