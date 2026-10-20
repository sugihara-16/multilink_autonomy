"""Small common preparation motion, excluded from the evaluation route.

Six smooth out-and-back pulses excite metric scale before constant-speed flight.
No estimator receives the commanded motion or the controller's ground truth.
"""
import numpy as np

DURATION = 26.0

def reference(t):
    p = np.zeros(3); v = np.zeros(3)
    if not 0 <= t < 24.0:
        return p, v
    phase = int(t // 4.0)
    axis = phase // 2
    amplitude = (0.25 if axis < 2 else 0.10) * (1 if phase % 2 == 0 else -1)
    angle = np.pi * (t % 4.0) / 4.0
    p[axis] = amplitude * np.sin(angle)**4
    v[axis] = amplitude * np.pi * np.sin(angle)**3 * np.cos(angle)
    return p, v
