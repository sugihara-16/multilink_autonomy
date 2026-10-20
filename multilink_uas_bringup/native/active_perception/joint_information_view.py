"""Retain joint pose information of currently tracked, mature landmarks.

Candidate slews are scored on the whole predicted visible set, including the
coupling between translation and rotation. This is a local information proxy:
landmark correlations and future newly detected features are not modeled.
No unseen texture, truth, route commands, or estimator weights are used.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from core import camera_rotation
from spatial_parallax_fixation import SpatialParallaxFixation


def visibility(rays, angles, home, cfg):
    rotations = np.array([camera_rotation(home, *a) for a in angles])
    p = np.einsum('ni,cij->cnj', rays, rotations)
    edge = np.maximum(np.abs(p[:, :, 0])/np.maximum(p[:, :, 2], 1e-12)/cfg['tan_half_hfov'],
                      np.abs(p[:, :, 1])/np.maximum(p[:, :, 2], 1e-12)/cfg['tan_half_vfov'])
    border = cfg.get('border_fraction', .2)
    if not 0 < border <= 1:
        raise ValueError('Invalid visibility margin')
    return np.clip((1.-edge)/border, 0., 1.)*(p[:, :, 2] > 0.)


def joint_gain(prior, contributions, weights):
    """Dimensionless log determinant ratio of two positive information matrices."""
    prior = (prior+prior.T)*.5
    L = np.linalg.cholesky(prior)
    additions = np.einsum('cn,nij->cij', weights, contributions)
    posterior = prior[None]+additions
    C = np.linalg.cholesky((posterior+posterior.transpose(0, 2, 1))*.5)
    return 2.*(np.log(np.diagonal(C, axis1=1, axis2=2)).sum(axis=1)-np.log(np.diag(L)).sum())


class JointInformationView(SpatialParallaxFixation):
    def score_model(self, prior, contributions, ids):
        gain = joint_gain
        score_kind = 'predicted_joint_pose_log_determinant_gain'
        if self.cfg.get('fixation_translation_information', False):
            from translation_information_gain import translation_gain
            gain = translation_gain
            score_kind = 'predicted_marginal_position_log_determinant_gain'
        return lambda weights: gain(prior, contributions, weights), score_kind

    def _choose(self, record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now):
        if not 0 < dt <= self.cfg['max_dt']:
            raise ValueError('Invalid planning interval')
        ids, points, covs, ages, rates = record
        rays = points-tbc
        distance = np.linalg.norm(rays, axis=1)
        unit = rays/np.maximum(distance[:, None], 1e-12)
        sigma = np.sqrt(np.maximum(0., np.einsum('ni,nij,nj->n', unit, covs, unit)))
        mature = ((ages >= self.cfg.get('fixation_min_age', 1.)) & (distance > .3)
                  & (sigma/np.maximum(distance, 1e-12) <= self.cfg.get('fixation_max_relative_sigma', .3))
                  & np.isfinite(covs).all(axis=(1, 2)) & np.isfinite(rates).all(axis=1))
        mature &= np.linalg.eigvalsh(covs).min(axis=1) >= -1e-9
        initial = visibility(unit, np.asarray(current)[None], R_home, self.cfg)[0]
        mature &= initial > 0.
        if int(mature.sum()) < self.cfg['min_features']:
            return super()._choose(record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now)

        self.recovering = False
        self.missing_since = None
        self.quality_bad_since = None
        # Generate the same bounded track-following motion as fixation, once
        # per mature point and once for their mean. Holding is always included.
        aims = np.vstack([unit[mature], unit[mature].mean(axis=0)])
        motion = np.vstack([rates[mature], rates[mature].mean(axis=0)])
        future = Rotation.from_rotvec(-omega*dt).apply(aims)+motion*dt
        def bearings(v):
            v = v@R_home
            return np.column_stack([np.arctan2(v[:, 1], v[:, 0]),
                                    np.arctan2(-v[:, 2], np.linalg.norm(v[:, :2], axis=1))])
        angles = bearings(aims)
        change = bearings(future)-angles
        change = np.arctan2(np.sin(change), np.cos(change))
        lo, hi = self.motion.limits()
        center = np.clip(angles, lo, hi)-current
        center *= np.minimum(1., self.cfg.get('scan_speed', self.cfg['max_speed'])*dt/
                            np.maximum(np.linalg.norm(center, axis=1), 1e-12))[:, None]
        if self.cfg.get('fixation_yaw_only', False):
            change[:, 1] = 0.
        commands = self.limiter.command_batch(current, current+change+center, R_home, np.zeros(3), dt)
        commands = np.vstack([current, commands])

        contributions = self.fisher.information(points[mature], covs[mature],
            camera_rotation(R_home, *current), tbc, self.cfg['bearing_sigma'], per_point=True)[2]
        prior = (H+H.T)*.5+np.diag(1/np.square(self.cfg['prior_std']))
        scores = np.zeros(len(commands))
        score, score_kind = self.score_model(prior, contributions, ids[mature])
        allowed = np.ones(len(commands), dtype=bool)
        horizons = sorted(set([float(dt), max(float(dt), self.cfg.get('prediction_horizon_s', .8))]))
        retained = []
        for horizon in horizons:
            predicted = Rotation.from_rotvec(-omega*horizon).apply(unit)+rates*horizon
            predicted /= np.maximum(np.linalg.norm(predicted, axis=1)[:, None], 1e-12)
            projected = np.clip(current+(commands-current)*(horizon/dt), lo, hi)
            weights = np.minimum(visibility(predicted, projected, R_home, self.cfg), initial[None])
            count = weights.sum(axis=1)
            retained.append(count)
            allowed &= count >= self.cfg.get('retain_fraction', .9)*count[0]-1e-12
            scores += score(weights[:, mature])/len(horizons)
        scores[~allowed] = -np.inf
        # Require a real improvement over holding. Avoid movement for ties.
        best = int(np.argmax(scores))
        improvement = scores[best]-scores[0]
        if improvement <= max(1e-12, self.cfg.get('min_cost_improvement', .03)*abs(scores[0])):
            best = 0
        self.anchors = []
        return commands[best], dict(reason='joint_information_hold' if best == 0 else 'joint_information_retention',
            anchors=[], mature_tracks=int(mature.sum()), tracked_points=len(ids),
            selection_score_kind=score_kind,
            hold_score=float(scores[0]), selected_score=float(scores[best]),
            candidate_count=len(commands), feasible_count=int(allowed.sum()),
            retention_horizons_s=horizons,
            retention_hold_future=[float(c[0]) for c in retained],
            retention_selected_future=[float(c[best]) for c in retained])
