#!/usr/bin/env python3
"""Push-step timeline of a pl_mission.py recording, in 0.5 s bins:

    /usr/bin/python3 pl_timeline.py ../runs/pl_9.npz [step]

Per bin: tilt (odometry, max), |e_R| (max), the rendered task force |F_hat_y|
and the raw reading |F_hat| (max), the box speed and its travel since the
step began, peak arm torque, rotor-saturation ticks. For locating WHEN a push
goes wrong (break-away, slide, stop) rather than how badly.
"""
import sys

import numpy as np
from pl_score import STEPS, D_ER, D_FY, D_FRAW, D_TAU, D_NSAT, col, tilt_deg

path = sys.argv[1]
step = sys.argv[2] if len(sys.argv) > 2 else "push"
k = STEPS.index(step)
z = np.load(path, allow_pickle=True)
o, d, b = z["odom"], z["dbg"], z["box"]
o, d, b = o[o[:, -1] == k], d[d[:, 1] == k], b[b[:, -1] == k]
t0 = o[0, 0]
print(f"{path} -- {step}, t0 = {t0:.2f} s (driver clock)")
print(f"{'t':>5s} {'tilt':>5s} {'|eR|':>6s} {'Fy':>5s} {'Fraw':>5s} {'vbox':>6s} {'box':>6s} {'tau':>5s} {'sat':>4s}")
print(f"{'s':>5s} {'deg':>5s} {'':>6s} {'N':>5s} {'N':>5s} {'m/s':>6s} {'mm':>6s} {'N.m':>5s}")
tb = b[:, 0]
vb = np.r_[0.0, np.linalg.norm(np.diff(b[:, 1:3], axis=0), axis=1) / np.maximum(np.diff(tb), 1e-3)]
travel = np.linalg.norm(b[:, 1:3] - b[0, 1:3], axis=1) * 1e3
T = o[-1, 0] - t0
for a in np.arange(0.0, T, 0.5):
    so = (o[:, 0] >= t0 + a) & (o[:, 0] < t0 + a + 0.5)
    sd = (d[:, 0] >= t0 + a) & (d[:, 0] < t0 + a + 0.5)
    sb = (tb >= t0 + a) & (tb < t0 + a + 0.5)
    if not so.any():
        continue
    tl = tilt_deg(o[so, 7], o[so, 8], o[so, 9], o[so, 10]).max()
    line = f"{a:5.1f} {tl:5.2f}"
    if sd.any():
        dd = d[sd]
        line += (f" {np.linalg.norm(col(dd, D_ER), axis=1).max():6.3f}"
                 f" {np.linalg.norm(col(dd, D_FY)[:, :3], axis=1).max():5.2f}"
                 f" {np.linalg.norm(col(dd, D_FRAW)[:, :3], axis=1).max():5.1f}")
    else:
        line += f" {'-':>6s} {'-':>5s} {'-':>5s}"
    line += (f" {vb[sb].max():6.3f} {travel[sb].max():6.1f}" if sb.any() else f" {'-':>6s} {'-':>6s}")
    if sd.any():
        line += f" {np.abs(col(dd, D_TAU)).max():5.2f} {int(np.count_nonzero(col(dd, D_NSAT) > 0)):4d}"
    print(line)
