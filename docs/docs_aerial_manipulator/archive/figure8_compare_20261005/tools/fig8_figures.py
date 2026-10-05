#!/usr/bin/env python3
"""fig8_figures.py -- the comparison report's figure-8 figures (2026-10-05).

  --top   top view per flight (the 8 is planar, so a plan view reads better than
          the circle figure's 3-D): EE reference (dashed black), EE measured
          (controller colour), base reference (dashed grey), base measured (dark
          grey), and the claw's horizontal direction as arrows (black reference,
          red measured) -- the same arrow convention as the circle figure.
  --series  for one flight per controller, against time since Start:
          the reference base yaw rate, then the EE position error, then the EE
          heading error (three stacked panels, one y-axis each).

    PYTHONNOUSERSITE=1 /usr/bin/python3 fig8_figures.py --top out_top.png --series out_series.png \
        --run WB-1 wb wb_f8a.npz --run GEO-1 geo decoupled_f8a.npz ...
    (--series uses the first run of each controller)
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..",
                                 "application", "robotic_arm", "utils"))
import am_ee_compare_score as SC  # noqa: E402

COLORS = {"wb": "#2a78d6", "geo": "#1baf7a", "mod": "#eb6834"}   # the report's controller colours
NAMES = {"wb": "Whole-body L1", "geo": "Geometric L1 adaptive", "mod": "Modular adaptive"}
REF_INK, BASE_REF, BASE_MEAS, MEAS_ARROW = "#16191d", "#9aa1ab", "#4a5059", "#c8312b"


def claw(b1):
    """Rz(+90) of the horizontal EE heading b_1e, unit length: the claw's horizontal direction."""
    h = b1[:, :2] / np.linalg.norm(b1[:, :2], axis=1)[:, None]
    return np.column_stack([-h[:, 1], h[:, 0]])


def rms(x):
    return float(math.sqrt(np.mean(np.asarray(x) ** 2)))


def top(runs, out, arrows, length):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    n = len(runs)
    ncol = 2 if n > 1 else 1
    nrow = -(-n // ncol)
    fig, axs = plt.subplots(nrow, ncol, figsize=(7.2 * ncol, 4.6 * nrow), squeeze=False)
    lims = None
    data = []
    for label, rig, path in runs:
        res, ser = SC.analyse(path)
        data.append((label, rig, res, ser))
        e, b = ser["ee"], ser["base"]
        pts = np.vstack([e[:, 1:3], e[:, 4:6], b[3][:, :2], b[4][:, :2]])
        lo, hi = pts.min(0), pts.max(0)
        lims = (lo, hi) if lims is None else (np.minimum(lims[0], lo), np.maximum(lims[1], hi))
    pad = 0.08
    for i, (label, rig, res, ser) in enumerate(data):
        ax = axs[i // ncol][i % ncol]
        e, (th, az, b1m, b1r), b = ser["ee"], ser["heading"], ser["base"]
        col = COLORS[rig]
        ax.plot(b[4][:, 0], b[4][:, 1], "--", color=BASE_REF, lw=1.0)
        ax.plot(b[3][:, 0], b[3][:, 1], "-", color=BASE_MEAS, lw=1.2)
        ax.plot(e[:, 4], e[:, 5], "--", color=REF_INK, lw=1.2)
        ax.plot(e[:, 1], e[:, 2], "-", color=col, lw=1.8)
        idx = np.linspace(0, len(th) - 1, arrows).astype(int)[1:-1]
        pe = SC.interp_rows(e[:, 0], e[:, 1:3], th[idx]); pr = SC.interp_rows(e[:, 0], e[:, 4:6], th[idx])
        cr, cm = claw(b1r[idx]), claw(b1m[idx])
        kw = dict(angles="xy", scale_units="xy", scale=1.0 / length, width=0.004, headwidth=4, headlength=5)
        ax.quiver(pr[:, 0], pr[:, 1], cr[:, 0], cr[:, 1], color=REF_INK, **kw)
        ax.quiver(pe[:, 0], pe[:, 1], cm[:, 0], cm[:, 1], color=MEAS_ARROW, **kw)
        s0 = e[0, 4:6]
        ax.plot(*s0, "o", ms=6, mfc="white", mec=REF_INK, mew=1.4)
        ax.set_title(f"{label}: EE {res['ee_pos_rms_mm']:.1f} mm / {rms(az):.2f}° rms", fontsize=11)
        ax.set_xlim(lims[0][0] - pad, lims[1][0] + pad); ax.set_ylim(lims[0][1] - pad, lims[1][1] + pad)
        ax.set_aspect("equal"); ax.grid(color="#e8ebef", lw=0.8); ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    for j in range(len(data), nrow * ncol):
        axs[j // ncol][j % ncol].axis("off")
    handles = [Line2D([], [], color=REF_INK, ls="--", label="EE reference"),
               Line2D([], [], color=COLORS["wb"], label="EE measured, whole-body"),
               Line2D([], [], color=COLORS["geo"], label="EE measured, geometric"),
               Line2D([], [], color=BASE_REF, ls="--", label="base reference"),
               Line2D([], [], color=BASE_MEAS, label="base measured"),
               Line2D([], [], color=REF_INK, marker=r"$\rightarrow$", ls="", ms=12, label="claw, reference"),
               Line2D([], [], color=MEAS_ARROW, marker=r"$\rightarrow$", ls="", ms=12, label="claw, measured"),
               Line2D([], [], color=REF_INK, marker="o", mfc="white", ls="", ms=6, label="start / end")]
    fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=9, frameon=False)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print("wrote", out)


def series(runs, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    first = {}
    for label, rig, path in runs:
        first.setdefault(rig, (label, path))
    fig, axs = plt.subplots(3, 1, figsize=(10, 7.6), sharex=True,
                            gridspec_kw=dict(height_ratios=[1, 1.3, 1.3]))
    yaw_done = False
    for rig, (label, path) in first.items():
        res, ser = SC.analyse(path)
        d, _ = SC.load(path)
        t0 = ser["t0"]
        if not yaw_done:
            w = d["wbref"]; w = w[(w[:, 0] >= ser["t0"]) & (w[:, 0] <= ser["t1"])]
            tg = np.arange(w[0, 0], w[-1, 0], 0.02)
            yaw = np.interp(tg, w[:, 0], np.unwrap(np.arctan2(w[:, 11], w[:, 10])))
            axs[0].plot(tg - t0, np.degrees(np.gradient(yaw, tg)), color=REF_INK, lw=1.4)
            yaw_done = True
        e = ser["ee"]
        axs[1].plot(e[:, 0] - t0, 1e3 * np.linalg.norm(e[:, 1:4] - e[:, 4:7], axis=1), color=COLORS[rig], lw=1.5,
                    label=f"{NAMES[rig]} ({label})")
        th, az = ser["heading"][0], ser["heading"][1]
        axs[2].plot(th - t0, az, color=COLORS[rig], lw=1.5, label=f"{NAMES[rig]} ({label})")
    axs[0].set_ylabel("reference yaw\nrate [deg/s]")
    axs[1].set_ylabel("EE position\nerror [mm]")
    axs[2].set_ylabel("EE heading\nerror [deg]")
    axs[2].set_xlabel("time since Start [s]")
    for a in axs:
        a.grid(color="#e8ebef", lw=0.8); a.set_axisbelow(True)
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
    axs[0].axhline(0, color=BASE_REF, lw=0.8)
    axs[2].axhline(0, color=BASE_REF, lw=0.8)
    axs[1].set_ylim(bottom=0)
    axs[1].legend(fontsize=9, frameon=False, loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print("wrote", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", nargs=3, action="append", metavar=("LABEL", "RIG", "NPZ"), required=True)
    ap.add_argument("--top", default="")
    ap.add_argument("--series", default="")
    ap.add_argument("--arrows", type=int, default=16)
    ap.add_argument("--length", type=float, default=0.12)
    a = ap.parse_args()
    if a.top:
        top(a.run, a.top, a.arrows, a.length)
    if a.series:
        series(a.run, a.series)


if __name__ == "__main__":
    main()
