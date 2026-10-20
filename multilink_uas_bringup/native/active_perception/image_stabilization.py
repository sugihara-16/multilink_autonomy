"""Keep the observed image bearings stable without relying on feature depth.

This is a visual-servo baseline, not a map-based view search. Fit a two-axis
camera correction to all tracked bearings with a robust loss, using measured
bearing motion and gyro. No body position, flight command, truth or image from
the fixed reference camera enters this controller.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from core import Planner, camera_rotation


class ImageStabilization:
    def __init__(self, fisher, config):
        self.config = config
        limit_config = dict(config)
        limit_config.pop('scan_speed', None)
        self.limiter = Planner(fisher, limit_config)

    def choose(self, record, R_home, tbc, current, dt, omega):
        if not 0 < dt <= self.config['max_dt']:
            raise ValueError('Invalid image stabilization interval')
        ids, points, covs, ages, motion = record
        rays = points - tbc
        length = np.linalg.norm(rays, axis=1)
        good = np.isfinite(rays).all(axis=1) & np.isfinite(motion).all(axis=1) & (length > .1)
        rays = rays[good] / length[good, None]
        motion = motion[good]
        # A newly detected point has no measured image motion yet. Fall back
        # to gyro stabilization when fewer than four established tracks exist.
        established = ages[good] >= .3
        use_flow = int(established.sum()) >= 4
        if use_flow:
            rays, motion = rays[established], motion[established]
        else:
            rays = np.eye(3)
            motion = np.zeros((3, 3))
        predicted = Rotation.from_rotvec(-np.asarray(omega)*dt).apply(rays) + motion*dt
        predicted /= np.linalg.norm(predicted, axis=1)[:, None]
        current = np.asarray(current, dtype=float)
        rotation = camera_rotation(R_home, *current)
        before = rays @ rotation
        after = predicted @ rotation
        # Camera coordinates change by -axis x bearing for a positive joint
        # increment. Axes expressed in the current optical coordinates.
        yaw_axis = R_home[:, 2] @ rotation
        pitch_axis = (R_home @ Rotation.from_euler('z', current[0]).as_matrix())[:, 1] @ rotation
        jacobian = np.stack((-np.cross(yaw_axis, after), -np.cross(pitch_axis, after)), axis=2)
        residual = before - after
        weights = np.ones(len(rays))
        delta = np.zeros(2)
        for _ in range(4):
            normal = np.einsum('nki,nkj,n->ij', jacobian, jacobian, weights)
            rhs = np.einsum('nki,nk,n->i', jacobian, residual, weights)
            delta = np.linalg.solve(normal + np.eye(2)*1e-8, rhs)
            error = np.linalg.norm(np.einsum('nki,i->nk', jacobian, delta)-residual, axis=1)
            # One pixel at this camera's focal length. Outliers do not choose
            # the viewing direction on behalf of the rest of the image.
            threshold = self.config.get('stabilization_huber_rad', .004)
            if threshold <= 0:raise ValueError('Invalid robust angular threshold')
            weights = np.minimum(1., threshold / np.maximum(error, 1e-12))
        return_time = self.config.get('stabilization_return_s', 0.)
        if not np.isfinite(return_time) or return_time < 0:
            raise ValueError('Invalid stabilization return time')
        correction = delta.copy()
        if return_time > 0:
            # A bounded follow mode: retain short-term scene stabilization,
            # but smoothly release a view before the joints stay saturated.
            delta -= current * (-np.expm1(-dt/return_time))
        command = self.limiter.command(current, current+delta, R_home, np.zeros(3), dt)
        return command, dict(reason='stabilize_observed_image' if use_flow else 'stabilize_gyro_only',
                             tracks_used=len(rays) if use_flow else 0,
                             correction_rad=correction.tolist(), return_time_s=float(return_time),
                             target_angles=(current+delta).tolist())
