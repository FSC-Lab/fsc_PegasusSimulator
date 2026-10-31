#!/usr/bin/env python3
"""push_timeline.py -- the push phase of an arbitrary-time box-pushing flight,
every --step s from the grasp: the base's tracking error, the law's force terms
(feedback m(r1+N), the J_E feed-forward -g*F_hat, the TSKFE D_hat), the observer's
estimate against the true contact force, the arm's admittance offset and
tracking error, and the box against its plan (the planner's lag guard).

    PYTHONNOUSERSITE=1 /usr/bin/python3 push_timeline.py runs/<tag>.npz [--step 1.0] [--ff 1.0]
"""
import argparse
import re

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("npz")
ap.add_argument("--step", type=float, default=1.0)
ap.add_argument("--from", dest="t_from", type=float, default=-1.0)
ap.add_argument("--to", type=float, default=12.0)
a = ap.parse_args()
z = np.load(a.npz, allow_pickle=True)
yaml = a.npz.replace(".npz", ".yaml")
try:
    ff = float(re.search(r"^\s+atc_quad_ff_gain:\s*([-0-9.eE]+)", open(yaml).read(), re.M).group(1))
except Exception:
    ff = 1.0
d = z["dbg"]
t = d[:, 0]
D = d[:, 2:]
mk = dict(zip(z["marks_name"], z["marks_t"]))
tp = float(mk["push:start"])
box, ct, arr = z["box"], z["contact"], z["arrival"]
m_ = 3.74617
print(f"ff gain {ff}; push starts at {tp - float(mk['direct']):.1f} s after DIRECT")
print("  t    | e_p [mm]           | feedback  -g*F_hat  D_hat (y) [N] | F_hat world        -F_true            | qc-qd [deg]          | box y   box behind plan [mm]")
for s in np.arange(tp + a.t_from, tp + a.to, a.step):
    m = (t >= s) & (t < s + a.step)
    if m.sum() < 5:
        continue
    mb = (box[:, 0] >= s) & (box[:, 0] < s + a.step)
    mc = (ct[:, 0] >= s) & (ct[:, 0] < s + a.step)
    Fd, Fh, Dh, ad = D[m, 90:93].mean(0), D[m, 32:35].mean(0), D[m, 16:19].mean(0), D[m, 69:72].mean(0)
    fb = Fd - m_ * ad - np.array([0, 0, m_ * 9.81]) + ff * Fh - Dh
    Ft = (ct[mc, 1:4] + ct[mc, 4:7]).mean(0) if mc.sum() else np.full(3, np.nan)
    lag = np.nan
    ma = (arr[:, 0] >= s) & (arr[:, 0] < s + a.step) if arr.size > 1 else None
    if ma is not None and ma.sum() and arr.shape[1] > 13:
        lag = 1e3 * np.nanmean(arr[ma, 13])   # [11] box lag in the arrival record (recorder cols +2)
    print(f"{s - tp:+5.1f} | {np.round(1e3 * D[m, 0:3].mean(0), 0)} | {fb[1]:+5.2f}    {-ff * Fh[1]:+5.2f}    {Dh[1]:+5.2f}   | "
          f"{np.round(Fh, 2)} {np.round(-Ft, 2)} | {np.round(np.degrees((D[m, 54:58] - D[m, 50:54]).mean(0)), 2)} | "
          f"{box[mb, 2].mean() if mb.sum() else np.nan:.3f}   {lag:6.1f}")
