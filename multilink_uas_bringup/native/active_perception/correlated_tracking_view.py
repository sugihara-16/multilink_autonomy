"""Retain useful bearing tracks without pretending their depth is known.

ROVIO's marginal velocity/bearing covariance already integrates uncertainty in
depth and the other states. It is therefore used directly, without the depth
precision gate needed by the older independent 3D-landmark information proxy.
This remains a local additional-observation score, not a future EKF forecast.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from joint_information_view import visibility
from joint_velocity_view import JointVelocityView, subset
from joint_velocity_gain import velocity_gain


class CorrelatedTrackingView(JointVelocityView):
    def information_score(self, H, C, **forecast_context):
        return (lambda weights:velocity_gain(C, weights, self.cfg['bearing_sigma']),
                'conditional_joint_rovio_velocity_log_determinant_gain')

    def candidate_commands(self, unit, rates, eligible, R_home, current, omega, dt):
        aims = np.vstack([unit[eligible], unit[eligible].mean(axis=0)])
        motion = np.vstack([rates[eligible], rates[eligible].mean(axis=0)])
        future = Rotation.from_rotvec(-omega*dt).apply(aims)+motion*dt

        def bearings(v):
            v = v@R_home
            return np.column_stack([np.arctan2(v[:, 1], v[:, 0]),
                                    np.arctan2(-v[:, 2], np.linalg.norm(v[:, :2], axis=1))])

        angles = bearings(aims); change = bearings(future)-angles
        change = np.arctan2(np.sin(change), np.cos(change))
        lo, hi = self.motion.limits()
        center = np.clip(angles, lo, hi)-current
        center *= np.minimum(1., self.cfg.get('scan_speed', self.cfg['max_speed'])*dt/
                             np.maximum(np.linalg.norm(center, axis=1), 1e-12))[:, None]
        if self.cfg.get('fixation_yaw_only', False):change[:, 1] = 0.
        commands = self.limiter.command_batch(current, current+change+center, R_home, np.zeros(3), dt)
        return commands

    def _choose(self, record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now):
        if not 0 < dt <= self.cfg['max_dt']:
            raise ValueError('Invalid planning interval')
        ids, points, covs, ages, rates = record
        rays = points-tbc
        distance = np.linalg.norm(rays, axis=1)
        unit = rays/np.maximum(distance[:, None], 1e-12)
        initial = visibility(unit, np.asarray(current)[None], R_home, self.cfg)[0]
        eligible = ((ages >= self.cfg.get('motion_average_s', .3)) & (distance > .1)
                    & np.isfinite(rates).all(axis=1) & (initial > 0.))
        if int(eligible.sum()) < self.cfg['min_features']:
            return super()._choose(record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now)
        if not hasattr(self, 'joint_covariance'):
            raise ValueError('Missing matching ROVIO joint covariance')
        C = subset(self.joint_ids, self.joint_covariance, ids[eligible])
        self.recovering = False; self.missing_since = None; self.quality_bad_since = None
        commands = self.candidate_commands(unit, rates, eligible, R_home, current, omega, dt)
        commands = np.vstack([current, commands])
        lo, hi = self.motion.limits()
        scores = np.zeros(len(commands)); allowed = np.ones(len(commands), dtype=bool)
        horizons = sorted(set([float(dt), max(float(dt), self.cfg.get('prediction_horizon_s', .8))]))
        retained = []
        for horizon in horizons:
            score, score_kind = self.information_score(H, C, horizon=horizon,
                selected_ids=ids[eligible], angular_velocity=omega, camera_position=tbc)
            predicted = Rotation.from_rotvec(-omega*horizon).apply(unit)+rates*horizon
            predicted /= np.maximum(np.linalg.norm(predicted, axis=1)[:, None], 1e-12)
            projected = np.clip(current+(commands-current)*(horizon/dt), lo, hi)
            weights = np.minimum(visibility(predicted, projected, R_home, self.cfg), initial[None])
            count = weights.sum(axis=1); retained.append(count)
            allowed &= count >= self.cfg.get('retain_fraction', .9)*count[0]-1e-12
            scores += score(weights[:, eligible])/len(horizons)
        scores[~allowed] = -np.inf
        best = int(np.argmax(scores))
        if scores[best]-scores[0] <= max(1e-12, self.cfg.get('min_cost_improvement', .03)*abs(scores[0])):best = 0
        self.anchors = []
        return commands[best], dict(reason='correlated_tracking_hold' if best == 0 else 'correlated_tracking_retention',
            anchors=[], eligible_tracks=int(eligible.sum()), tracked_points=len(ids),
            selection_score_kind=score_kind,
            depth_precision_gate=False, hold_score=float(scores[0]), selected_score=float(scores[best]),
            candidate_count=len(commands), feasible_count=int(allowed.sum()), retention_horizons_s=horizons,
            retention_hold_future=[float(c[0]) for c in retained],
            retention_selected_future=[float(c[best]) for c in retained])
