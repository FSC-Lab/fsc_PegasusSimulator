#!/usr/bin/env python3
"""The grasp's torsional stiffness about the vertical, per run: the true heading couple on the claw
(contact truth, resisting +) against the claw's twist on the fin, theta = (true base yaw + q1) - beam yaw,
over the turn. A rigid grasp would show no twist; a soft one a slope k = dM / dtheta.
    grip_stiffness.py tag [tag ...] [--plot out.png]"""
import os, sys, argparse
import numpy as np
from scipy.signal import butter, filtfilt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import yaw_score as YS
ap = argparse.ArgumentParser(); ap.add_argument("tags", nargs="+"); ap.add_argument("--plot")
a = ap.parse_args()
out = []
for tag in a.tags:
    z = np.load(os.path.join(YS.RUNS, tag + ".npz"), allow_pickle=True)
    row, S = YS.score(tag)
    bt, box, js = z["base_truth"], z["box"], z["joints"]
    names = [str(n) for n in z["joint_names"]]
    j1 = names.index("joint1") if "joint1" in names else 0
    t = S["t"]
    w = (t >= S["a"]) & (t <= S["b"])
    psi = np.interp(t, bt[:, 0], np.unwrap(YS.yaw_q(bt[:, 4:8])))
    q1 = np.interp(t, js[:, 0], js[:, 1 + j1])
    yb = np.interp(t, box[:, 0], np.unwrap(YS.yaw_q(box[:, 4:8])))
    th = psi + q1 - yb
    bb, aa = butter(2, 5.0 / (0.5 / np.median(np.diff(t))))
    th = filtfilt(bb, aa, th)
    # the couple ON the claw, about +z: F_ext[:, 3] (the fin's reaction to the claw twisting CCW is CW)
    Mz = S["F_ext"][:, 3]
    thw, Mw = th[w] - th[w].mean(), Mz[w] - Mz[w].mean()
    k = -float(np.polyfit(thw, Mw, 1)[0])            # N.m/rad: the fin's restoring couple per rad of twist
    r = float(np.corrcoef(thw, Mw)[0, 1])
    out.append((tag, row["K_psi"], k, r, float(np.degrees(thw.std())), float(Mw.std())))
    print(f"{tag:22s} K_psi {row['K_psi']:6.3g}  twist std {np.degrees(thw.std()):5.1f} deg  couple std {Mw.std():.3f} N.m  "
          f"k_grip = {k:+.3f} N.m/rad (corr {r:+.2f})")
if a.plot:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 4.5))
    for tag in a.tags[:4]:
        z = np.load(os.path.join(YS.RUNS, tag + ".npz"), allow_pickle=True)
    fig.savefig(a.plot)
