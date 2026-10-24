"""Finish recovery from lost tracks before resuming local view selection.

This changes camera motion only. It does not reset or correct an estimator.
The local policy may use a few provisional features during a short loss, but
cannot cancel a committed recovery on the first newly observed feature.
"""
import numpy as np
from joint_information_view import visibility
from retinal_tracking_view import RetinalTrackingView


class CommittedTrackingView(RetinalTrackingView):
    def __init__(self, fisher, cfg):
        super().__init__(fisher, cfg)
        self.track_recovery = False
        self.track_bad_since = None
        self.track_good_since = None
        self.track_last_time = None
        self.track_delay = float(cfg.get('recovery_delay_s', 1.))
        self.track_settle = float(cfg.get('track_recovery_settle_deg', 1.)) * np.pi / 180
        self.track_reference = np.deg2rad(cfg.get('fixation_recovery_angles_deg', [0., 0.]))
        lo, hi = self.motion.limits()
        if not (self.track_delay > 0 and self.track_settle > 0 and
                self.track_reference.shape == (2,) and np.isfinite(self.track_reference).all() and
                np.all(self.track_reference >= lo) and np.all(self.track_reference <= hi)):
            raise ValueError('Invalid committed track recovery configuration')

    def choose(self, record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now):
        if not 0 < dt <= self.cfg['max_dt'] or not np.isfinite(now):
            raise ValueError('Invalid tracking observation time')
        if self.track_last_time is not None:
            if now < self.track_last_time:
                raise ValueError('Tracking observation time must not decrease')
            if now - self.track_last_time > self.cfg['max_dt']:
                # A held preparation phase is not evidence of continuous
                # good or bad tracking; require fresh observations afterward.
                self.track_bad_since = self.track_good_since = None
        self.track_last_time = now
        ids, points, covs, ages, rates = record
        rays = points - tbc
        distances = np.linalg.norm(rays, axis=1)
        unit = rays / np.maximum(distances[:, None], 1e-12)
        weights = visibility(unit, np.asarray(current)[None], R_home, self.cfg)[0]
        eligible = ((distances > .1) & (ages >= self.cfg.get('motion_average_s', .3)) &
                    np.isfinite(rates).all(axis=1) & np.isfinite(points).all(axis=1) & (weights > 0))
        count = int(eligible.sum())
        enough = count >= self.cfg['min_features']
        if enough:
            self.track_bad_since = None
            if self.track_good_since is None:
                self.track_good_since = now
        else:
            self.track_good_since = None
            if self.track_bad_since is None:
                self.track_bad_since = now
            if now - self.track_bad_since >= self.track_delay:
                self.track_recovery = True
        settled = np.linalg.norm(np.asarray(current) - self.track_reference) <= self.track_settle
        if self.track_recovery and enough and settled and now - self.track_good_since >= self.track_delay:
            self.track_recovery = False
            self.recovering = False
            self.quality_bad_since = self.quality_good_since = self.missing_since = None
        if self.track_recovery:
            self.anchors = []
            command = self.limiter.command(current, self.track_reference, R_home, np.zeros(3), dt)
            report = dict(reason='committed_track_recovery', anchors=[], target_angles=self.track_reference.tolist())
        else:
            command, report = super().choose(record, H, U, weak, R_home, tbc, current, velocity, dt, omega, now)
        report.update(committed_recovery_active=self.track_recovery,
                      recovery_visible_aged_tracks=count, recovery_settled=bool(settled))
        return command, report
