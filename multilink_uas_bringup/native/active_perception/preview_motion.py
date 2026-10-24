"""Bounded out-and-back motion on the already validated first route polyline.

Used only by the common body controller. No reference or truth is supplied to
the camera planner. The return follows the identical polyline in reverse.
"""
import numpy as np

MAX_DISTANCE_M = 15.0
MAX_SPEED_MPS = 0.5
RAMP_S = 2.0


def profile(elapsed, distance, speed=MAX_SPEED_MPS, ramp=RAMP_S):
    if not all(np.isfinite(x) for x in (elapsed, distance, speed, ramp)):
        raise ValueError('Finite preview motion parameters required')
    if distance <= 0 or speed <= 0 or ramp <= 0:
        raise ValueError('Positive preview distance, speed and ramp required')
    peak = min(speed, distance / ramp)
    cruise = max(0.0, distance / peak - ramp)
    duration = 2 * ramp + cruise
    t = float(np.clip(elapsed, 0, duration))
    if t < ramp:
        phase = np.pi * t / ramp
        return .5 * peak * (t - ramp / np.pi * np.sin(phase)), .5 * peak * (1 - np.cos(phase)), duration
    if t <= ramp + cruise:
        return .5 * peak * ramp + peak * (t - ramp), peak, duration
    remaining = duration - t
    phase = np.pi * remaining / ramp
    return distance - .5 * peak * (remaining - ramp / np.pi * np.sin(phase)), .5 * peak * (1 - np.cos(phase)), duration


def reference(segment, elapsed, returning=False):
    if segment['kind'] != 'polyline':
        raise ValueError('Preview requires a first polyline segment')
    points = np.asarray(segment['points'], float)
    arc = np.asarray(segment['arc'], float)
    if points.shape != (len(arc), 3) or len(arc) < 2:
        raise ValueError('Invalid preview polyline')
    if not np.isfinite(points).all() or not np.isfinite(arc).all() or arc[0] != 0 or np.any(np.diff(arc) <= 0):
        raise ValueError('Invalid preview arc lengths')
    if abs(segment['yaw1'] - segment['yaw0']) > 1e-9:
        raise ValueError('Preview requires constant route yaw on the first segment')
    length = min(float(arc[-1]), MAX_DISTANCE_M)
    progress, velocity, duration = profile(elapsed, length)
    if returning:
        progress = length - progress
        velocity = -velocity
    i = int(np.clip(np.searchsorted(arc, progress, side='right') - 1, 0, len(arc) - 2))
    fraction = (progress - arc[i]) / (arc[i + 1] - arc[i])
    delta = points[i + 1] - points[i]
    position = points[i] + fraction * delta
    tangent = delta / (arc[i + 1] - arc[i])
    return position, velocity * tangent, duration
