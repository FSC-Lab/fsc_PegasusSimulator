#!/usr/bin/env python3
"""Is the arm stick-slipping in contact? Per joint (q2, q3): the share of time STUCK (|qdot| < 1 deg/s,
from the encoder position, 50 Hz, 0.1 s smoothing), the number of slips (> 20 deg/s), their peak speed,
and the time spent on q2's 45 deg wall. Hardware: <tag> <t0> <t1>; Isaac: <run.npz> (the clamped steps).
Usage: arm_stickslip.py 194310 49.0 70.3 | arm_stickslip.py ../../x/runs/sg_s1.npz"""
import sys, os
import numpy as np

def stats(name, t, q2, q3):
    T = np.arange(t[0], t[-1], 0.02)
    out = [name]
    for nm, q in (("q2", q2), ("q3", q3)):
        y = np.convolve(np.interp(T, t, q), np.ones(5) / 5, "same")
        v = np.gradient(y, 0.02)[5:-5]
        stuck = np.mean(np.abs(v) < 1.0)
        slips = np.diff((np.abs(v) > 20).astype(int)); n = int(np.sum(slips == 1))
        out.append(f"{nm}: stuck {stuck:4.0%}, {n:2d} slips > 20 deg/s, peak {np.abs(v).max():4.0f} deg/s")
    out.append(f"q2 on its 45 deg wall {np.mean(np.interp(T, t, q2) > 44.5):4.0%}")
    print(" | ".join(out))

a = sys.argv[1]
if a.endswith(".npz"):
    for p in sys.argv[1:]:
        z = np.load(p, allow_pickle=True); J = z["joints"]; m = np.isin(J[:, 9], [2, 3]); jn = list(z["joint_names"])
        stats(f"Isaac {os.path.basename(p):12s}", J[m, 0], np.degrees(J[m, 1 + jn.index("joint2")]),
              np.degrees(J[m, 1 + jn.index("joint3")]))
else:
    from bp_common import Bag
    b = Bag(a); t = b.t("js"); P = np.degrees(b.f("js", "position")); t0, t1 = float(sys.argv[2]), float(sys.argv[3])
    m = (t >= t0) & (t <= t1)
    stats(f"hardware {a} {t0}-{t1}", t[m], P[m, 0], P[m, 1])
