#!/usr/bin/env python
"""
| File: reference.py
| License: BSD-3-Clause
| Description: Closed-form attitude reference for the attitude-only
|   validation -- a sequence of single-axis "doublet" maneuvers (smooth
|   ramp to a peak angle about a FIXED axis, hold, smooth ramp back to
|   zero), one per body axis (roll, pitch, yaw), run back to back.

Unlike Automatica2026Simulation/quad_attitude_sim/reference.py (which had to
derive R_d(t) from a commanded LIFT VECTOR via attitude_extraction, and so
needed a numerically-filtered "dirty derivative" to get omega_d/dot_omega_d
-- see that module's docstring), there is no lift vector here: the attitude
reference IS the thing being specified directly. For a FIXED rotation axis
n, R_d(t) = expm(skew(n)*angle(t)) has an exact closed form for both
derivatives with no filtering needed at all:

    dot(R_d) = R_d @ skew(n) * angle_dot(t)      (property of a
    omega_d    = n * angle_dot(t)                  one-parameter rotation
    dot_omega_d = n * angle_ddot(t)                 subgroup, holds for ANY
                                                     R_base right-multiplied
                                                     in -- see module code
                                                     comment on why segments
                                                     can be composed simply)

Each doublet's angle(t) ramps 0 -> angle_max -> 0 using the same raised-
cosine profile (0.5*(1-cos(pi*s))) Automatica2026Simulation's own reference
profiles use, so it starts and ends at angle=0 (R_d=I) -- meaning each
segment is self-contained (no need to track/compose a running base attitude
across segments; every segment already starts from identity).
"""
import numpy as np
from scipy.linalg import expm


def skew(v):
    v = np.asarray(v).flatten()
    return np.array([[0., -v[2], v[1]],
                      [v[2], 0., -v[0]],
                      [-v[1], v[0], 0.]])


def _smooth_ramp(s):
    """s in [0,1] -> (profile, dprofile/ds, ddprofile/ds^2), profile: 0->1."""
    profile = 0.5 * (1. - np.cos(np.pi * s))
    dprofile = 0.5 * np.pi * np.sin(np.pi * s)
    ddprofile = 0.5 * np.pi ** 2 * np.cos(np.pi * s)
    return profile, dprofile, ddprofile


def _doublet_angle(t_local, angle_max, t_ramp, t_hold):
    """t_local measured from the segment's own start. Returns
    (angle, angle_dot, angle_ddot)."""
    if t_local < 0. or t_local > 2 * t_ramp + t_hold:
        return 0., 0., 0.
    if t_local < t_ramp:
        s = t_local / t_ramp
        p, dp, ddp = _smooth_ramp(s)
        return angle_max * p, angle_max * dp / t_ramp, angle_max * ddp / t_ramp ** 2
    if t_local < t_ramp + t_hold:
        return angle_max, 0., 0.
    s = (t_local - t_ramp - t_hold) / t_ramp
    p, dp, ddp = _smooth_ramp(s)
    return angle_max * (1. - p), -angle_max * dp / t_ramp, -angle_max * ddp / t_ramp ** 2


# (axis, angle_max [rad], t_ramp [s], t_hold [s], t_start [s])
DEFAULT_MANEUVER = [
    (np.array([1., 0., 0.]), np.radians(20.), 0.6, 1.0, 1.0),    # roll doublet
    (np.array([0., 1., 0.]), np.radians(20.), 0.6, 1.0, 4.0),    # pitch doublet
    (np.array([0., 0., 1.]), np.radians(30.), 0.8, 1.0, 7.0),    # yaw doublet
]


def compute_reference(t, maneuver=None):
    maneuver = DEFAULT_MANEUVER if maneuver is None else maneuver
    for axis, angle_max, t_ramp, t_hold, t_start in maneuver:
        t_local = t - t_start
        if 0. <= t_local <= 2 * t_ramp + t_hold:
            angle, angle_dot, angle_ddot = _doublet_angle(t_local, angle_max, t_ramp, t_hold)
            R_d = expm(skew(axis) * angle)
            return {'R_d': R_d, 'omega_d': axis * angle_dot, 'dot_omega_d': axis * angle_ddot}
    return {'R_d': np.eye(3), 'omega_d': np.zeros(3), 'dot_omega_d': np.zeros(3)}
