"""Keep useful image tracks while their metric depth is still converging.

The normal mature-landmark policy is unchanged when enough mature tracks exist.
The fallback ranks measured, gyro-compensated bearing motion. It does not turn
that motion into a metric depth or an estimator measurement. This is a view
selection heuristic, not a calibrated visual information matrix.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from core import camera_rotation
from fixation import Fixation
from retention import protect_tracks


class ParallaxFixation(Fixation):
    def __init__(self, fisher, cfg):
        super().__init__(fisher, cfg)
        if not cfg.get('fixation_yaw_only') or not cfg.get('fixation_bearing_motion'):
            raise ValueError('Parallax fallback requires yaw-only measured bearing motion')

    def _choose(self, record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now):
        if not 0 < dt <= self.cfg['max_dt']:
            raise ValueError('Invalid fixation planning dt')
        ids, points, covs, ages, rates = record
        rays = points - tbc
        distances = np.linalg.norm(rays, axis=1)
        unit = rays / np.maximum(distances[:, None], 1e-12)
        relative_sigma = np.sqrt(np.maximum(0., np.einsum('ni,nij,nj->n', unit, covs, unit))) / np.maximum(distances, 1e-12)
        mature = (ages >= self.cfg.get('fixation_min_age', 1.)) & (relative_sigma <= self.cfg.get('fixation_max_relative_sigma', .3))
        mature &= (distances > .3) & np.isfinite(covs).all(axis=(1, 2))
        mature &= np.linalg.eigvalsh(covs).min(axis=1) >= -1e-9
        if int(mature.sum()) >= self.cfg['min_features']:
            # Once useful depth has recovered, do not discard it merely to
            # finish a return to the zero-angle reference.
            self.recovering = False
            return super()._choose(record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now)

        lo, hi = self.motion.limits()
        home = unit @ R_home
        yaw = np.arctan2(home[:, 1], home[:, 0])
        reachable = (yaw >= lo[0]) & (yaw <= hi[0])
        reachable &= np.abs(home[:, 2]) < np.linalg.norm(home[:, :2], axis=1) * self.cfg['tan_half_vfov']
        optical = rays @ camera_rotation(R_home, *current)
        visible = (optical[:, 2] > .1)
        visible &= np.abs(optical[:, 0]) < optical[:, 2] * self.cfg['tan_half_hfov']
        visible &= np.abs(optical[:, 1]) < optical[:, 2] * self.cfg['tan_half_vfov']
        window = self.cfg.get('motion_average_s', .3)
        if window <= 0:
            raise ValueError('Positive bearing-motion window required')
        # Two bearing observations contribute to a difference. Require a
        # two-standard-deviation angular displacement over that window.
        minimum_rate = 2. * np.sqrt(2.) * self.cfg['bearing_sigma'] / window
        speed = np.linalg.norm(rates, axis=1)
        provisional = (ages >= window) & (speed > minimum_rate)
        eligible = visible & reachable & (mature | provisional) & np.isfinite(rates).all(axis=1)
        if not eligible.any():
            return super()._choose(record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now)

        retained = [i for i in self.anchors if np.any((ids == i) & eligible)]
        reason = 'retain_observed_parallax'
        if not retained:
            # Directional support concerns translation only. Rotation is
            # removed by the IMU before the observed bearing rate is formed.
            directions = np.asarray(U)[3:, :]
            norms = np.linalg.norm(directions, axis=0)
            good = norms > 1e-12
            weights = np.asarray(weak)[good]
            if weights.sum() > 1e-12:
                axes = directions[:, good] / norms[good]
                support = np.maximum(0., 1. - (unit @ axes) ** 2) @ (weights / weights.sum())
            else:
                support = np.ones(len(ids))
            scores = speed ** 2 * support
            valid = np.flatnonzero(eligible)
            order = valid[np.lexsort((-ages[valid], -scores[valid]))]
            best = order[0]
            radius = .5 * np.arctan(self.cfg['tan_half_vfov'])
            cluster = [i for i in order if unit[i] @ unit[best] >= np.cos(radius)]
            retained = [int(ids[i]) for i in cluster[:3]]
            reason = 'select_observed_parallax'
        self.anchors = retained
        self.missing_since = None
        self.quality_bad_since = None
        self.recovering = False
        selected = np.isin(ids, retained)
        aim = np.mean(unit[selected], axis=0) @ R_home
        raw_yaw = np.arctan2(aim[1], aim[0])
        target = np.array([np.clip(raw_yaw, lo[0], hi[0]), 0.])
        future = Rotation.from_rotvec(-omega * dt).apply(unit[selected]) + rates[selected] * dt
        future /= np.maximum(np.linalg.norm(future, axis=1)[:, None], 1e-12)
        aim_future = np.mean(future, axis=0) @ R_home
        delta_yaw = np.arctan2(aim_future[1], aim_future[0]) - raw_yaw
        tracking_step = np.array([np.arctan2(np.sin(delta_yaw), np.cos(delta_yaw)), 0.])
        centering = target - current
        centering *= min(1., self.cfg.get('scan_speed', self.cfg['max_speed']) * dt / max(np.linalg.norm(centering), 1e-12))
        command = self.limiter.command(current, current + tracking_step + centering, R_home, np.zeros(3), dt)
        protection = {}
        if self.cfg.get('fixation_protect_other_tracks'):
            command, protection = protect_tracks(unit, rates, omega, R_home, current, command, dt, self.cfg, (lo, hi))
        return command, dict(reason=reason, anchors=retained, target_angles=target.tolist(),
            mature_tracks=int(mature.sum()), provisional_tracks=int((eligible & ~mature).sum()),
            tracked_points=len(ids), minimum_parallax_rate_rad_s=float(minimum_rate),
            anchor_bearing_rate_rad_s=speed[selected].tolist(),
            selection_score_kind='measured_rotation_compensated_bearing_motion', **protection)
