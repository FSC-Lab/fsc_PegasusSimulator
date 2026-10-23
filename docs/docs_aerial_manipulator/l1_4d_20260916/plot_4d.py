#!/usr/bin/env python3
"""6-D vs 4-D attribution: tracking and phantom-force traces, one figure.

    PYTHONNOUSERSITE=1 /usr/bin/python3 plot_4d.py --six a.npz [b.npz] --four c.npz [d.npz] --out fig.png

PYTHONNOUSERSITE=1 because this machine's ~/.local numpy is 2.x while the apt
matplotlib is built against 1.x (the wb_compare_plot.py trap). Only the numeric
arrays are read, so the npz files written by a numpy-2 driver still load.

Rows, all against DIRECT-relative time:
  1. CoM tracking error |x_c - x_cd|                                      [mm]
  2. EE POSITION error |e_y[0:3]| -- position only, never the 4-norm of e_y,
     whose fourth component is sin(heading error) and is not a length     [mm]
  3. EE HEADING error asin|e_y[3]| -- the channel the designs differ on   [deg]
  4. the force the impedance was rendering against: |F_hat_y| consumed
     (6-D: the attributed phantom; 4-D: gated to 0) and, for the 4-D runs, the
     RAW |F_hat| the gate hides (dashed)                                   [N]
  5. peak |tau_joint| over the four joints                               [N.m]
"""
import argparse

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D_MODE, D_U1 = 0, 17
D_TAU = slice(13, 17)
D_EY = slice(24, 28)
D_XCD = slice(45, 48)
D_XC = slice(48, 51)
D_FY = slice(58, 62)
D_FRAW = slice(97, 101)


def load(p):
    z = np.load(p, allow_pickle=False)
    dbg = z["dbg"]
    t, d = dbg[:, 0], dbg[:, 1:]
    m = np.nan_to_num(d[:, D_MODE]) > 0.5
    t0 = t[m][0]
    d = d[m]; t = t[m] - t0
    com = np.linalg.norm(d[:, D_XC] - d[:, D_XCD], axis=1) * 1e3
    ey = np.linalg.norm(d[:, D_EY][:, :3], axis=1) * 1e3
    head = np.degrees(np.arcsin(np.clip(np.abs(d[:, D_EY.start + 3]), 0.0, 1.0)))
    fy = np.linalg.norm(d[:, D_FY][:, :3], axis=1)
    fraw = np.linalg.norm(d[:, D_FRAW][:, :3], axis=1) if d.shape[1] > D_FRAW.stop else np.zeros_like(fy)
    # the raw reading is a 250 Hz momentum difference through J_yq^-T: show
    # its 1 s running mean, which is what a collision test would threshold
    n = max(1, int(round(1.0 / np.median(np.diff(t)))))
    fraw = np.convolve(fraw, np.ones(n) / n, mode="same")
    tau = np.abs(d[:, D_TAU]).max(axis=1)
    return dict(t=t, com=com, ey=ey, head=head, fy=fy, fraw=fraw, tau=tau, name=p.split("/")[-1].replace(".npz", ""))


ap = argparse.ArgumentParser()
ap.add_argument("--six", nargs="+", required=True)
ap.add_argument("--four", nargs="+", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--title", default="Whole-body DIRECT, config A plant: 6-D vs 4-D attribution")
a = ap.parse_args()

fig, ax = plt.subplots(5, 1, figsize=(12, 13.5), sharex=True)
cols6 = ["#2a78d6", "#86b6ef"]
cols4 = ["#eb6834", "#f3a889"]
for i, p in enumerate(a.six):
    r = load(p)
    c = cols6[i % 2]
    ax[0].plot(r["t"], r["com"], c, lw=0.8, label=f"6-D {r['name']}")
    ax[1].plot(r["t"], r["ey"], c, lw=0.8)
    ax[2].plot(r["t"], r["head"], c, lw=0.8)
    ax[3].plot(r["t"], r["fy"], c, lw=0.8, label=f"6-D |F_hat_y| consumed ({r['name']})")
    ax[4].plot(r["t"], r["tau"], c, lw=0.8)
for i, p in enumerate(a.four):
    r = load(p)
    c = cols4[i % 2]
    ax[0].plot(r["t"], r["com"], c, lw=0.8, label=f"4-D {r['name']}")
    ax[1].plot(r["t"], r["ey"], c, lw=0.8)
    ax[2].plot(r["t"], r["head"], c, lw=0.8)
    ax[3].plot(r["t"], r["fraw"], c, lw=0.8, ls="--", label=f"4-D RAW |F_hat|, 1 s mean (gated, not consumed) ({r['name']})")
    ax[3].plot(r["t"], r["fy"], c, lw=1.2, label=f"4-D |F_hat_y| consumed = 0 ({r['name']})")
    ax[4].plot(r["t"], r["tau"], c, lw=0.8)
ax[0].set_ylabel("CoM position error\n|x_c - x_cd|  [mm]"); ax[0].set_ylim(0, 350)
ax[1].set_ylabel("EE POSITION error\n|e_y[0:3]|  [mm]"); ax[1].set_ylim(0, 40)
ax[2].set_ylabel("EE HEADING error\nasin|e_y[3]|  [deg]"); ax[2].set_ylim(0, 30)
ax[3].set_ylabel("phantom force  [N]")
ax[4].set_ylabel("max |tau_joint|  [N.m]"); ax[4].set_xlabel("time since DIRECT entry  [s]")
ax[0].legend(fontsize=8, ncol=2); ax[3].legend(fontsize=7, ncol=2)
for x in ax:
    x.grid(alpha=0.3)
fig.suptitle(a.title)
fig.tight_layout()
fig.savefig(a.out, dpi=130)
print("wrote", a.out)
