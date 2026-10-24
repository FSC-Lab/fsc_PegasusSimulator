#!/usr/bin/env python3
"""(2026-10-22) Mocap marker layout for the valve rotor, drawn on the CAD (Valve_Assembly.stl).

    PYTHONNOUSERSITE=1 /usr/bin/python3 marker_layout.py

Frame = the valve asset's (stl_to_usda_valve.py): origin on the bearing axis at the plate-bottom
plane, +x along the beam toward the fin, +z up; millimetres. Writes ../marker_layout.png and
prints the markers' pairwise distances (they must all differ for Motive to label them).
"""
import itertools
import os
import struct

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(HERE, "..", "..", "..", "..", "..", "extensions", "fsc_aerial_manipulation",
                   "fsc_aerial_manipulation", "rotorcraft", "assets", "Valve_Assembly.stl")
OUT = os.path.join(HERE, "..", "marker_layout.png")
AXIS_MM, PLATE_BOTTOM_MM = np.array([-1.1, 3.0]), -4.5        # the converter's CAD -> asset shift
BEAM_TOP = 50.0                                                # the flat top the markers sit on

# (name, x, y, post height above the beam top) -- centre of each marker's base, mm
# from marker_plan_search.py --joint (2026-10-22): every base has >= 2.5 mm of flat top around it;
# 4-marker plan = M1..M4 (distances >= 10.2 mm apart), 5-marker plan = M1..M5 (>= 4.6 mm apart)
MARKERS = [("M1", 16.0, 15.0, 0.0),
           ("M2", 42.0, -14.0, 30.0),
           ("M3", 98.0, 14.0, 0.0),
           ("M4", 129.0, -9.0, 0.0)]
M5 = ("M5", 74.0, -6.0, 0.0)                                   # the fifth, 5-marker plan only
BASE_D = 12.0                                                  # marker base diameter assumed


def load():
    raw = open(STL, "rb").read()
    n = struct.unpack("<I", raw[80:84])[0]
    a = np.frombuffer(raw[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    V = a["v"].reshape(-1, 3, 3).astype(float)
    if abs(V).max() < 5:
        V *= 1000.0
    V[:, :, :2] -= AXIS_MM
    V[:, :, 2] -= PLATE_BOTTOM_MM
    return V


def main():
    V = load()
    N = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0])
    N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    up = (N[:, 2] > 0.99) & (V[:, :, 2].mean(1) > 45)          # the rotor's upward faces
    z = V[up, :, 2].mean(1)
    order = np.argsort(z)
    fig, ax = plt.subplots(figsize=(13, 5.2))
    colors = {50.0: "#d9d4c7", 52.0: "#b8b0a0", 54.0: "#9c9384", 57.4: "#7d7466", 74.0: "#6b6255", 204.0: "#3f3a33"}
    fc = [colors.get(round(v, 1), "#cccccc") for v in z[order]]
    ax.add_collection(PolyCollection(V[up][order][:, :, :2], facecolors=fc, edgecolors="none"))
    # keep-out: the bracket pad + fin (x 142-178) and the beam end beside the jaws
    ax.add_patch(plt.Rectangle((142, -25), 58, 50.7, fill=False, hatch="///", ec="#b23b3b", lw=1.0))
    ax.text(171, 27.5, "keep clear: bracket, fin, the claw's jaws", color="#b23b3b", ha="center", fontsize=9)
    ax.add_patch(plt.Circle((0, 0), 11, fill=False, ec="#555", lw=1, ls="--"))
    ax.text(0, -31, "axis hole", ha="center", fontsize=8, color="#555")
    ax.plot([0], [0], "+", color="k", ms=12)
    ax.annotate("", xy=(60, -40), xytext=(0, -40), arrowprops=dict(arrowstyle="->", lw=1.4))
    ax.text(30, -45, "obj_0 +x (toward the fin)", ha="center", fontsize=9)
    ax.annotate("", xy=(-40, 25), xytext=(-40, -5), arrowprops=dict(arrowstyle="->", lw=1.4))
    ax.text(-43, 10, "+y", ha="right", fontsize=9)
    ax.text(160, 0, "fin", ha="center", va="center", fontsize=8, color="w",
            bbox=dict(boxstyle="round,pad=0.15", fc="#3f3a33", ec="none"))
    for name, x, y, h in MARKERS + [M5]:
        ax.add_patch(plt.Circle((x, y), BASE_D / 2, fc="#2a78d6" if h == 0 else "#eb6834", ec="k", lw=0.8,
                                ls="--" if name == "M5" else "-", alpha=0.55 if name == "M5" else 1.0))
        lab = f"{name}  ({x:+.0f}, {y:+.0f})" + (f"\non a {h:.0f} mm post" if h else "")
        ax.text(x, y + (10 if y > 0 else -11), lab, ha="center", va="bottom" if y > 0 else "top", fontsize=9)
    ax.set_xlim(-55, 210)
    ax.set_ylim(-52, 34)
    ax.set_aspect("equal")
    ax.set_xlabel("x [mm] from the bearing axis, along the beam")
    ax.set_ylabel("y [mm]")
    ax.set_title("Valve rotor, top view: mocap markers M1-M4 (4-marker plan) + M5 dashed (5-marker plan), base centres on the 50 mm flat top")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(OUT, dpi=150)
    print("wrote", os.path.abspath(OUT))
    for label, plan in (("4-marker", MARKERS), ("5-marker", MARKERS + [M5])):
        P = {n: np.array([x, y, BEAM_TOP + h]) for n, x, y, h in plan}
        d = sorted((np.linalg.norm(P[a] - P[b]), a, b) for a, b in itertools.combinations(P, 2))
        print(f"{label} pairwise distances (base centres, mm):", ", ".join(f"{a}-{b} {v:.1f}" for v, a, b in d))
        print("  closest two distances differ by", f"{min(np.diff([v for v, _, _ in d])):.1f} mm")


if __name__ == "__main__":
    main()
