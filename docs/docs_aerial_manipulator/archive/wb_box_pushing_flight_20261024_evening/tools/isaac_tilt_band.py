#!/usr/bin/env python3
"""Tilt (true, base_truth) of a pl_mission.py run in two bands -- 0.05-1 Hz (the contact wander) and
1.2-1.8 Hz (the attitude mode H1b's gains were once seen to excite with a payload, 10-02) -- in the
open-jaw hold, clamped (close + push), and the way home (go_to_land_start). rms deg, max of along/across.
Usage: isaac_tilt_band.py <run.npz> ..."""
import sys, os
import numpy as np

def R_of(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
def band(t, x, f1, f2):
    T = np.arange(t[0], t[-1], 0.01); y = np.interp(T, t, x)
    X = np.fft.rfft(y - y.mean()); f = np.fft.rfftfreq(len(y), 0.01); X[(f < f1) | (f > f2)] = 0
    return np.degrees(np.std(np.fft.irfft(X, len(y))))
for p in sys.argv[1:]:
    z = np.load(p, allow_pickle=True); B = z["base_truth"]; t, st = B[:, 0], B[:, 8]
    v = np.array([R_of(q)[:, 2][:2] for q in B[:, 4:8]])
    out = []
    for nm, steps in (("hold", [1]), ("clamped", [2, 3]), ("way home", [6])):
        m = np.isin(st, steps)
        if m.sum() < 200:
            out.append(f"{nm}: --"); continue
        lo = max(band(t[m], v[m, k], 0.05, 1.0) for k in (0, 1)); hi = max(band(t[m], v[m, k], 1.2, 1.8) for k in (0, 1))
        out.append(f"{nm} {lo:4.2f} / {hi:4.2f}")
    print(f"{os.path.basename(p):20s} tilt rms [deg] 0.05-1 Hz / 1.2-1.8 Hz: " + " | ".join(out))
