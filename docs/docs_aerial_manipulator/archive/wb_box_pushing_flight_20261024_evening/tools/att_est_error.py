#!/usr/bin/env python3
"""PX4's attitude estimate (vehicle_attitude, the law's attitude) vs the mocap rigid body's attitude:
the tilt the law cannot see. Per window: the mean difference (the rigid-body frame offset), its
variation in the 0.05-1 Hz band (what misdirects 37 N of thrust), and the true pitch / roll motion.
Both rotated into the push frame (along = the nose) so pitch = the along-push tilt.
Usage: att_est_error.py <tag> <name:t0:t1> ..."""
import sys
import numpy as np
from bp_common import Bag

tag = sys.argv[1]
b = Bag(tag)

def R_of(q):    # w x y z
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

tv = b.t("vatt"); qv = b.f("vatt", "q")                       # PX4: NED / FRD, [w x y z]
tm = b.t("mocap"); qm = np.column_stack([b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"])   # ENU / FLU
P = np.diag([1.0, -1.0, -1.0])
NE = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])           # NED <-> ENU
T = np.arange(max(tv[0], tm[0]) + 1, min(tv[-1], tm[-1]) - 1, 0.02)
iv = np.clip(np.searchsorted(tv, T), 0, len(tv) - 1); im = np.clip(np.searchsorted(tm, T), 0, len(tm) - 1)
def tilt_vec(R):            # body z in world: (x, y) components = the tilt direction, rad
    return R[:, 2][:2]
d_px = np.zeros((len(T), 2)); d_mc = np.zeros((len(T), 2)); yaw = np.zeros(len(T))
for k in range(len(T)):
    Rp = NE @ R_of(qv[iv[k]]) @ P                              # PX4 -> ENU/FLU
    Rm = R_of(qm[im[k]])
    d_px[k] = tilt_vec(Rp); d_mc[k] = tilt_vec(Rm)
    yaw[k] = np.arctan2(Rm[1, 0], Rm[0, 0])
dt = 0.02
def band(x, f1=0.05, f2=1.0):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), dt)
    X[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(X, len(x))
for spec in sys.argv[2:]:
    name, a, c = spec.split(":"); a, c = float(a), float(c)
    m = (T >= a) & (T <= c)
    ps = np.mean(yaw[m])
    u = np.array([np.cos(ps), np.sin(ps)]); n = np.array([-u[1], u[0]])
    e = d_px[m] - d_mc[m]                                     # what PX4 thinks minus the truth (tilt vector)
    e_al, e_ac = e @ u, e @ n
    t_al, t_ac = d_mc[m] @ u, d_mc[m] @ n                     # the true tilt: + along = nose down (thrust forward)
    print(f"{tag} {name:16s} {a:6.1f}-{c:6.1f}s yaw {np.degrees(ps):6.1f} | PX4-mocap tilt err: mean along {np.degrees(e_al.mean()):+5.2f} "
          f"across {np.degrees(e_ac.mean()):+5.2f} deg; 0.05-1 Hz rms along {np.degrees(np.std(band(e_al))):.2f} across "
          f"{np.degrees(np.std(band(e_ac))):.2f} deg (x 36.6 N = {36.6*np.std(band(e_al)):.2f} / {36.6*np.std(band(e_ac)):.2f} N) | "
          f"TRUE tilt 0.05-1 Hz rms along {np.degrees(np.std(band(t_al))):.2f} across {np.degrees(np.std(band(t_ac))):.2f} deg")
