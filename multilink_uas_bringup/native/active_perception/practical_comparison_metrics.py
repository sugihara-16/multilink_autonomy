"""Full-interval common-clock evaluation, never truncate to estimator coverage."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def resample_pose(samples, times, max_gap_s):
    samples = np.asarray(samples, dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 8 or len(samples) < 2:
        raise ValueError('Expected time, XYZ, quaternion XYZW with at least two poses')
    if not np.isfinite(samples).all() or np.any(np.diff(samples[:, 0]) < 0):
        raise ValueError('Non-finite or out-of-order pose data')
    # Repeated publications of one timestamp are permitted only if identical.
    duplicate = np.flatnonzero(np.diff(samples[:, 0]) == 0)
    if any(not np.array_equal(samples[i, 1:], samples[i + 1, 1:]) for i in duplicate):
        raise ValueError('Conflicting poses at one timestamp')
    samples = samples[np.r_[True, np.diff(samples[:, 0]) > 0]]
    ts = samples[:, 0]
    if times[0] < ts[0] - 1e-7 or times[-1] > ts[-1] + 1e-7:
        raise ValueError('Incomplete evaluation interval; extrapolation is forbidden')
    i = np.searchsorted(ts, times, side='right').clip(1, len(ts) - 1)
    gap = ts[i] - ts[i - 1]
    if gap.max() > max_gap_s + 1e-7:
        raise ValueError(f'Pose gap {gap.max():.6f}s exceeds {max_gap_s}s')
    norms = np.linalg.norm(samples[:, 4:8], axis=1)
    if not np.allclose(norms, 1., atol=1e-3, rtol=0):
        raise ValueError('Invalid pose quaternion')
    pos = np.column_stack([np.interp(times, ts, samples[:, j]) for j in range(1, 4)])
    rot = Slerp(ts, Rotation.from_quat(samples[:, 4:8]))(times)
    return pos, rot, float(gap.max())


def evaluate(truth, estimate, start, duration=182., rate=10., max_gap_s=.151):
    elapsed = np.arange(1, round(duration * rate), dtype=float) / rate
    times = start + elapsed
    if len(times) < 2:
        raise ValueError('Evaluation interval too short')
    tp, tr, truth_gap = resample_pose(truth, times, max_gap_s)
    ep, er, estimate_gap = resample_pose(estimate, times, max_gap_s)
    alignment = tr[0] * er[0].inv()
    error = alignment.apply(ep - ep[0]) + tp[0] - tp
    angular = (tr.inv() * alignment * er).as_rotvec() * 180 / np.pi
    magnitude = np.linalg.norm(error, axis=1)
    metrics = dict(position_rmse_m=float(np.sqrt(np.mean(magnitude**2))),
                   position_max_m=float(magnitude.max()), position_end_m=float(magnitude[-1]),
                   rotation_rmse_deg=float(np.sqrt(np.mean(np.sum(angular**2, axis=1)))),
                   samples=len(times), elapsed_start_s=float(elapsed[0]), elapsed_end_s=float(elapsed[-1]),
                   truth_max_gap_s=truth_gap, estimate_max_gap_s=estimate_gap,
                   alignment='First common sample position and orientation; no scale fitting')
    return elapsed, error, angular, metrics
