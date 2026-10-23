#!/usr/bin/env python3
"""plot_3d_tangent.py -- the comparison report's 3-D trajectory figure, with the
end-effector heading drawn as the CLAW direction (tangent to the circle).

Same data and window as am_ee_compare_score.analyse(); differs from that tool's
own 3-D plot only in the arrows. The tracked heading b_1e (the end-effector frame's
first axis in the controller model, i.e. the wrist x-axis) is perpendicular to the
claw, so on this circle it points radially outward. Here every arrow is that
heading rotated +90 deg about the vertical, projected horizontally and unit-length:
it is the horizontal direction the claw points (checked against the claw from FK,
-R0m re[:,2], to dot product 1.000) and, for the reference, the direction of travel.
The angle between a black and a red arrow is unchanged by the rotation, so it is
still exactly the EE heading error in the table. The claw also tilts ~35-40 deg
down, which is not drawn.

    PYTHONNOUSERSITE=1 /usr/bin/python3 plot_3d_tangent.py --out fig.png \
        --run WB-1 wb wb_rt1a.npz --run MOD-1 mod modular_nu10a.npz ...
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..",
                                 "application", "robotic_arm", "utils"))
import am_ee_compare_score as SC  # noqa: E402

COLORS = {"wb": "#2a78d6", "geo": "#1baf7a", "mod": "#eb6834"}   # the report's controller colours


def tangent(b1):
    """Rz(+90) of the horizontal heading, unit length: the claw's horizontal direction."""
    h = b1[:, :2] / np.linalg.norm(b1[:, :2], axis=1)[:, None]
    return np.column_stack([-h[:, 1], h[:, 0], np.zeros(len(h))])


def rms(x):
    return float(math.sqrt(np.mean(np.asarray(x) ** 2)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", nargs=3, action="append", metavar=("LABEL", "RIG", "NPZ"), required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--arrows", type=int, default=12)
    ap.add_argument("--length", type=float, default=0.15)
    ap.add_argument("--cols", type=int, default=0, help="panels per row (0 = all in one row)")
    a = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    n = len(a.run)
    ncol = a.cols or n
    nrow = -(-n // ncol)
    fig = plt.figure(figsize=(7 * ncol, 6.2 * nrow))
    for i, (label, rig, path) in enumerate(a.run):
        res, ser = SC.analyse(path)
        e, (th, az, b1m, b1r), b = ser["ee"], ser["heading"], ser["base"]
        col = COLORS[rig]
        ax = fig.add_subplot(nrow, ncol, i + 1, projection="3d")
        ax.plot(e[:, 4], e[:, 5], e[:, 6], "k--", lw=1.2)
        ax.plot(e[:, 1], e[:, 2], e[:, 3], "-", color=col, lw=1.8)
        ax.plot(b[4][:, 0], b[4][:, 1], b[4][:, 2], "--", color="#8a8a86", lw=1.0)
        ax.plot(b[3][:, 0], b[3][:, 1], b[3][:, 2], "-", color="#4b4b48", lw=1.4)
        idx = np.linspace(0, len(th) - 1, a.arrows).astype(int)
        pe = SC.interp_rows(e[:, 0], e[:, 1:4], th[idx]); pr = SC.interp_rows(e[:, 0], e[:, 4:7], th[idx])
        tr, tm = tangent(b1r[idx]), tangent(b1m[idx])
        ax.quiver(pr[:, 0], pr[:, 1], pr[:, 2], tr[:, 0], tr[:, 1], tr[:, 2], length=a.length,
                  color="k", arrow_length_ratio=0.45, lw=2.0)
        ax.quiver(pe[:, 0], pe[:, 1], pe[:, 2], tm[:, 0], tm[:, 1], tm[:, 2], length=a.length,
                  color="tab:red", arrow_length_ratio=0.45, lw=2.0)
        ax.set_title(f"{label}: EE {res['ee_pos_rms_mm']:.1f} mm / {rms(az):.2f}° rms", fontsize=12)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
        pts = np.vstack([e[:, 1:4], e[:, 4:7]])
        c = pts.mean(axis=0); r = max(0.5, np.abs(pts - c).max())
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        handles = [Line2D([], [], color="k", ls="--", label="EE reference"),
                   Line2D([], [], color=col, label="EE measured"),
                   Line2D([], [], color="#8a8a86", ls="--", label="base reference"),
                   Line2D([], [], color="#4b4b48", label="base measured"),
                   Line2D([], [], color="k", marker=r"$\rightarrow$", ls="", ms=12, label="claw, reference"),
                   Line2D([], [], color="tab:red", marker=r"$\rightarrow$", ls="", ms=12, label="claw, measured")]
        ax.legend(handles=handles, loc="upper left", fontsize=8)
        print(f"{label}: EE {res['ee_pos_rms_mm']:.2f} mm, heading {rms(az):.2f} deg rms")
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    plt.close(fig)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
