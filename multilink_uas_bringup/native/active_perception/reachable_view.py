"""Finite-horizon view commands, with no route, environment labels or truth."""
import numpy as np


def reachable_endpoint(current, goal, speed, acceleration, horizon, norm=np.inf):
    """A rest-to-rest coordinated segment that finishes within the horizon.

    This is the inverse of the actuator's triangular/trapezoidal profile, not
    just speed * horizon: the joints must also accelerate and stop.
    """
    delta=np.asarray(goal)-current
    distance=float(np.linalg.norm(delta,ord=norm))
    if acceleration is None:
        budget=speed*horizon
    elif horizon<=2*speed/acceleration:
        budget=acceleration*horizon*horizon/4
    else:
        budget=speed*(horizon-speed/acceleration)
    return np.asarray(current)+delta*min(1.,budget/max(distance,1e-12))


def future_points(points, velocity, omega, duration):
    """Static landmarks in a future body frame, using estimated body motion."""
    from scipy.spatial.transform import Rotation
    R=Rotation.from_rotvec(np.asarray(omega)*duration).as_matrix()
    return (np.asarray(points)-np.asarray(velocity)*duration)@R
