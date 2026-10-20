"""Keep a useful view still until measured tracks are at risk of leaving it.

This changes only the steering policy. It does not change estimator inputs,
fusion weights, feature depths, or physical limits. Unobserved scenery is
never treated as a known target.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from joint_information_view import JointInformationView, visibility


class StableView(JointInformationView):
    def _choose(self, record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now):
        if not 0 < dt <= self.cfg['max_dt']:
            raise ValueError('Invalid planning interval')
        ids, points, covs, ages, rates = record
        rays = points-tbc
        distance = np.linalg.norm(rays, axis=1)
        unit = rays/np.maximum(distance[:, None], 1e-12)
        finite = np.isfinite(covs).all(axis=(1, 2)) & np.isfinite(rates).all(axis=1)
        positive = np.zeros(len(ids), dtype=bool)
        positive[finite] = np.linalg.eigvalsh(covs[finite]).min(axis=1) >= -1e-9
        radial_variance = np.einsum('ni,nij,nj->n', unit, covs, unit)
        relative_sigma = np.sqrt(np.maximum(0., radial_variance))/np.maximum(distance, 1e-12)
        window = self.cfg.get('motion_average_s', .3)
        if window <= 0:
            raise ValueError('Positive bearing-motion window required')
        minimum_rate = 2.*np.sqrt(2.)*self.cfg['bearing_sigma']/window
        initial = visibility(unit, np.asarray(current)[None], R_home, self.cfg)[0]
        useful = (finite & positive & (distance > .3) & (initial > 0.)
                  & (ages >= self.cfg.get('fixation_min_age', 1.))
                  & (relative_sigma <= self.cfg.get('fixation_max_relative_sigma', .3))
                  & (np.linalg.norm(rates, axis=1) > minimum_rate))
        if int(useful.sum()) >= self.cfg['min_features']:
            horizon = max(float(dt), self.cfg.get('prediction_horizon_s', .8))
            predicted = Rotation.from_rotvec(-omega*horizon).apply(unit[useful])+rates[useful]*horizon
            predicted /= np.maximum(np.linalg.norm(predicted, axis=1)[:, None], 1e-12)
            future = visibility(predicted, np.asarray(current)[None], R_home, self.cfg)[0]
            before = initial[useful].sum()
            after = np.minimum(initial[useful], future).sum()
            if after >= self.cfg.get('retain_fraction', .9)*before:
                self.anchors = []
                self.recovering = False
                self.missing_since = None
                self.quality_bad_since = None
                return np.array(current, copy=True), dict(reason='stable_useful_view',
                    useful_tracks=int(useful.sum()), tracked_points=len(ids),
                    retention_hold_current=float(before), retention_hold_future=float(after),
                    retention_horizon_s=float(horizon), minimum_parallax_rate_rad_s=float(minimum_rate),
                    selection_score_kind='hold_mature_observed_parallax_until_track_loss_predicted')
        return super()._choose(record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now)
