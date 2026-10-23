#!/usr/bin/env python3
"""Tracking and error curves for a GMO vs L1 whole-body comparison.

    PYTHONNOUSERSITE=1 /usr/bin/python3 wb_compare_plot.py \
        --gmo gmo.npz --l1 l1.npz --out fig.png [--title "..."]

Run it under PYTHONNOUSERSITE=1: this machine's ~/.local numpy is 2.x while
the apt matplotlib is built against numpy 1.x. The npz files are written by a
numpy 2 process, though, so the OBJECT arrays (leg_marks, events) cannot be
unpickled under numpy 1 -- this script therefore reads only the plain numeric
arrays and takes the leg boundaries from a sidecar the metrics pass writes.

Four rows, all against DIRECT-relative time:
  1. CoM tracking    x_c vs x_cd, all three axes, both observers
  2. CoM error norm  |x_c - x_cd|
  3. EE task error   |e_y| (the impedance law's own task error)
  4. Phantom force   |F_hat_y| -- zero is the truth in free flight
"""

import argparse

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D_MODE, D_U1 = 0, 17
D_EY = slice(24, 28)
D_XCD = slice(45, 48)
D_XC = slice(48, 51)
D_FY = slice(58, 62)

C_GMO, C_L1, C_REF = "#B4451F", "#0E6F79", "#888888"


def load(path):
    z = np.load(path, allow_pickle=False)
    dbg = z["dbg"]
    t, d = dbg[:, 0], dbg[:, 1:]
    if d.shape[1] <= D_FY.stop:
        d = np.hstack([d, np.full((d.shape[0], D_FY.stop + 1 - d.shape[1]),
                                  np.nan)])
    m = np.nan_to_num(d[:, D_MODE]) > 0.5
    if not m.any():
        raise SystemExit(f"{path}: never entered DIRECT")
    return t[m] - t[m][0], d[m]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gmo", required=True)
    ap.add_argument("--l1", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="Whole-body DIRECT: GMO vs L1")
    ap.add_argument("--marks", default="",
                    help="comma-separated 'name:t_rel' leg boundaries")
    a = ap.parse_args()

    tg, dg = load(a.gmo)
    tl, dl = load(a.l1)
    marks = []
    for tok in filter(None, a.marks.split(",")):
        n, v = tok.rsplit(":", 1)
        marks.append((n, float(v)))

    fig, ax = plt.subplots(4, 1, figsize=(13, 13.5), sharex=True,
                           gridspec_kw=dict(hspace=0.16))
    fig.suptitle(a.title, fontsize=12.5, y=0.965)

    # 1. tracking, x and y (z is nearly constant and would flatten the scale).
    # EACH RUN IS PLOTTED AGAINST ITS OWN REFERENCE. The whole_body_planner
    # captures the hold from odometry at DIRECT entry, so the two runs' x_cd
    # differ by wherever the vehicle happened to be -- drawing one run's
    # reference under both makes the other look like it has a huge steady
    # offset when its error is millimetres.
    for k in (0, 1):
        ax[0].plot(tg, dg[:, D_XCD][:, k], color=C_GMO, lw=2.6, alpha=.30,
                   zorder=1, label="GMO reference" if k == 0 else None)
        ax[0].plot(tl, dl[:, D_XCD][:, k], color=C_L1, lw=2.6, alpha=.30,
                   zorder=1, label="L1 reference" if k == 0 else None)
        ax[0].plot(tg, dg[:, D_XC][:, k], color=C_GMO, lw=1.0,
                   label="GMO" if k == 0 else None)
        ax[0].plot(tl, dl[:, D_XC][:, k], color=C_L1, lw=1.0,
                   label="L1" if k == 0 else None)
    ax[0].set_ylabel("CoM $x$, $y$  [m]")
    ax[0].legend(loc="upper right", ncol=2, fontsize=8.5, framealpha=.9)
    # pad clears the rotated leg labels drawn above this axes
    ax[0].set_title("CoM tracking (reference is the law's own $x_{cd}$)",
                    fontsize=10, loc="left", pad=30)

    eg = np.linalg.norm(dg[:, D_XC] - dg[:, D_XCD], axis=1) * 1e3
    el = np.linalg.norm(dl[:, D_XC] - dl[:, D_XCD], axis=1) * 1e3
    ax[1].plot(tg, eg, color=C_GMO, lw=1.0, label="GMO")
    ax[1].plot(tl, el, color=C_L1, lw=1.0, label="L1")
    ax[1].set_ylabel("$|x_c - x_{cd}|$  [mm]")
    ax[1].set_yscale("log")
    ax[1].set_ylim(0.1, None)
    ax[1].legend(loc="upper right", fontsize=9)
    ax[1].set_title("CoM tracking error", fontsize=10, loc="left")

    yg = np.linalg.norm(dg[:, D_EY], axis=1) * 1e3
    yl = np.linalg.norm(dl[:, D_EY], axis=1) * 1e3
    ax[2].plot(tg, yg, color=C_GMO, lw=1.0, label="GMO")
    ax[2].plot(tl, yl, color=C_L1, lw=1.0, label="L1")
    ax[2].set_ylabel("$|e_y|$  [mm]")
    ax[2].set_yscale("log")
    ax[2].set_ylim(0.5, None)
    ax[2].legend(loc="upper right", fontsize=9)
    ax[2].set_title("End-effector task error", fontsize=10, loc="left")

    fg = np.linalg.norm(dg[:, D_FY][:, :3], axis=1)
    fl = np.linalg.norm(dl[:, D_FY][:, :3], axis=1)
    ax[3].plot(tg, fg, color=C_GMO, lw=1.0, label="GMO")
    ax[3].plot(tl, fl, color=C_L1, lw=1.0, label="L1")
    ax[3].axhline(0.0, color="k", lw=.8, ls=":")
    ax[3].set_ylabel(r"$|\hat F_y|$  [N]")
    ax[3].set_xlabel("time since DIRECT entry  [s]")
    ax[3].legend(loc="upper right", fontsize=9)
    ax[3].set_title("Phantom end-effector force — nothing is touching the "
                    "arm, so the truth is 0", fontsize=10, loc="left")

    for x in ax:
        x.grid(alpha=.25, lw=.5)
        for n, tm in marks:
            x.axvline(tm, color="k", lw=.6, alpha=.28, ls="--")
    for n, tm in marks:
        ax[0].annotate(n, (tm, 1.03), xycoords=("data", "axes fraction"),
                       fontsize=7.5, rotation=38, ha="left", va="bottom")

    fig.savefig(a.out, dpi=125, bbox_inches="tight")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
