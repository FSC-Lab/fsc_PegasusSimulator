#!/usr/bin/env python3
"""The mail-delivery design drawing (2026-10-23): the envelope (top view), the red
mailbox's front face with its mouth, and the side view at the release -- every
dimension from mail_geometry.py's constants.

    PYTHONNOUSERSITE=1 /usr/bin/python3 mail_drawing.py <out.png>
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mail_geometry as G                            # noqa: E402

RED, BLUE, DARK, GREY = "#c62828", "#1e40af", "#222222", "#888888"


def dim(ax, p0, p1, text, off=(0, 0), fs=8, color=DARK):
    ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="<->", lw=0.8, color=color))
    ax.text(0.5 * (p0[0] + p1[0]) + off[0], 0.5 * (p0[1] + p1[1]) + off[1], text, ha="center", va="center",
            fontsize=fs, color=color, bbox=dict(fc="white", ec="none", pad=0.5))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "mail_design.png"
    fig, axes = plt.subplots(1, 3, figsize=(17, 6.2), gridspec_kw={"width_ratios": [1.15, 1.0, 1.6]})
    L, W = G.ENV_L * 1e3, G.ENV_W * 1e3
    # ---- (a) the envelope, top view ----
    ax = axes[0]
    ax.add_patch(Rectangle((0, 0), L, W, fc="#e57373", ec=RED, lw=1.5))
    grip = (G.GRASP_IN + G.JAW_TIP_BEYOND_CLAW) * 1e3
    ax.add_patch(Rectangle((0, 0), grip, W, fc="none", ec=DARK, hatch="//", lw=0.6))
    ax.text(0.5 * grip, W + 6, "jaws\n(31 mm)", ha="center", fontsize=8)
    ax.add_patch(Rectangle((L - 30, 5), 25, W - 10, fc="#555555", ec="none", alpha=0.85))
    ax.text(L - 17.5, 0.5 * W, "steel\nstrip\n~35 g", ha="center", va="center", color="white", fontsize=7)
    for (mx, my) in ((50, 18), (72, 66), (88, 30)):
        ax.add_patch(Circle((mx, my), 3.2, fc="#bbbbbb", ec=DARK, lw=0.6))
    ax.text(72, W - 9, "markers (<= 4 mm tall), 40-90 mm from the edge", ha="center", fontsize=7)
    com = L - G.ENV_COM_FROM_FAR * 1e3
    ax.plot([com], [0.5 * W], marker="+", ms=14, mew=2, color=DARK)
    ax.text(com, 0.5 * W - 9, "CoM", ha="center", fontsize=8)
    ax.plot([G.GRASP_IN * 1e3], [0.5 * W], marker="o", ms=5, color=BLUE)
    ax.text(G.GRASP_IN * 1e3 + 3, 0.5 * W - 15, "grasp point\n(EE Offset x -0.06)", fontsize=7, color=BLUE)
    dim(ax, (0, -10), (L, -10), f"{L:.0f} mm")
    dim(ax, (-10, 0), (-10, W), f"{W:.0f}", off=(-6, 0))
    dim(ax, (com, -22), (L, -22), f"{G.ENV_COM_FROM_FAR * 1e3:.0f}")
    ax.annotate("near edge: overhangs the\nhat rim by 70 mm; the claw\ncomes in from here", xy=(0, W * 0.25),
                xytext=(-12, -55), fontsize=7, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax.annotate("obj_0 pivot = geometric centre,\nx toward the far (weighted) end", xy=(0.5 * L, 0.5 * W),
                xytext=(60, W + 28), fontsize=7, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax.set_title(f"ENVELOPE, top view: {L:.0f} x {W:.0f} x {G.ENV_T * 1e3:.0f} mm, "
                 f"{G.ENV_MASS * 1e3:.0f} g, red", fontsize=10)
    ax.set_xlim(-30, L + 20)
    ax.set_ylim(-70, W + 50)
    ax.set_aspect("equal")
    ax.axis("off")
    # ---- (b) the mailbox front face ----
    ax = axes[1]
    bw, bh = G.BOX_W * 1e3, G.BOX_H * 1e3
    mw, mh, mz = G.MOUTH_W * 1e3, G.MOUTH_H * 1e3, G.MOUTH_Z * 1e3
    ax.add_patch(Rectangle((-0.5 * bw, 0), bw, bh, fc="#ef5350", ec=RED, lw=1.5))
    ax.add_patch(Rectangle((-0.5 * mw, mz - 0.5 * mh), mw, mh, fc="#1b1b1b", ec=DARK))
    dim(ax, (-0.5 * bw, -12), (0.5 * bw, -12), f"{bw:.0f} mm")
    dim(ax, (0.5 * bw + 14, 0), (0.5 * bw + 14, bh), f"{bh:.1f}", off=(10, 0))
    dim(ax, (-0.5 * mw, mz + 0.5 * mh + 12), (0.5 * mw, mz + 0.5 * mh + 12), f"mouth {mw:.0f}")
    dim(ax, (-0.5 * mw - 10, mz - 0.5 * mh), (-0.5 * mw - 10, mz + 0.5 * mh), f"{mh:.0f}", off=(-9, 0))
    dim(ax, (0.5 * mw + 6, mz + 0.5 * mh), (0.5 * mw + 6, bh), f"{bh - mz - 0.5 * mh:.0f}", off=(9, 0))
    dim(ax, (0.5 * mw + 6, 0), (0.5 * mw + 6, mz - 0.5 * mh), f"{mz - 0.5 * mh:.1f}", off=(9, 0))
    ax.text(0, mz, "centred across", color="white", ha="center", va="center", fontsize=8)
    ax.text(0, 60, f"RED mailbox (the target)\n{G.BOX_D * 1e3:.0f} mm deep, {G.WALL * 1e3:.0f} mm walls\n"
            "front flush with the table edge", ha="center", fontsize=8, color="white")
    ax.set_title("MAILBOX front face: the mouth", fontsize=10)
    ax.set_xlim(-0.5 * bw - 40, 0.5 * bw + 45)
    ax.set_ylim(-30, bh + 30)
    ax.set_aspect("equal")
    ax.axis("off")
    # ---- (c) side view at the release (x forward, z up), metres ----
    ax = axes[2]
    xf, tt = G.TABLE_FRONT_X, G.TABLE_TOP
    ax.add_patch(Rectangle((xf, tt - 0.018), 0.150, 0.018, fc="#a1887f", ec=DARK, lw=0.6))
    ax.add_patch(Rectangle((xf, tt), G.BOX_D, G.BOX_H, fc="none", ec=RED, lw=1.5))
    ax.add_patch(Rectangle((xf, tt + G.MOUTH_Z - 0.5 * G.MOUTH_H), G.WALL, G.MOUTH_H, fc="white", ec="none"))
    for z0, z1 in ((tt, tt + G.MOUTH_Z - 0.5 * G.MOUTH_H), (tt + G.MOUTH_Z + 0.5 * G.MOUTH_H, tt + G.BOX_H)):
        ax.add_patch(Rectangle((xf, z0), G.WALL, z1 - z0, fc=RED, ec="none"))
    claw = (xf - G.JAW_CLEAR - G.JAW_TIP_BEYOND_CLAW, G.Z_CLAW_PLACE)
    body = (claw[0] - 0.2469, claw[1] + 0.0675)
    ax.add_patch(Rectangle((body[0] - 0.10, body[1] - 0.03), 0.20, 0.06, fc="#90a4ae", ec=DARK, lw=0.6))
    ax.plot([body[0] - 0.16, body[0] + 0.16], [body[1] - 0.2425] * 2, color=DARK, lw=2)
    zp = body[1] + G.PROP_BOTTOM_ABOVE_BODY
    ax.plot([body[0] + 0.037, body[0] + G.PROP_REACH_FWD], [zp, zp], color="#1565c0", lw=3)
    ax.text(body[0] + 0.23, zp + 0.012, "front props", color="#1565c0", fontsize=8, ha="center")
    pitch = math.radians(14.0)
    a = (math.cos(pitch), -math.sin(pitch))
    n0 = (claw[0] - G.GRASP_IN * a[0], claw[1] - G.GRASP_IN * a[1])
    n1 = (n0[0] + G.ENV_L * a[0], n0[1] + G.ENV_L * a[1])
    ax.plot([n0[0], n1[0]], [n0[1], n1[1]], color="#e53935", lw=4, solid_capstyle="butt")
    ax.plot([body[0] + 0.05, claw[0]], [body[1] - 0.02, claw[1]], color="#6d4c41", lw=3)
    ax.plot([claw[0]], [claw[1]], "o", color=BLUE, ms=5)
    ax.text(claw[0] - 0.005, claw[1] + 0.02, "claw (q = [0, 36, 44, 90] deg,\n10 deg down, jaws vertical)",
            fontsize=7, color=BLUE, ha="right")
    ax.text(n1[0] + 0.004, n1[1] - 0.03, "envelope,\n14 deg in the sim", fontsize=7, color="#e53935")
    dim(ax, (claw[0] + G.JAW_TIP_BEYOND_CLAW, claw[1] - 0.055), (xf, claw[1] - 0.055), "35 mm", fs=7)
    dim(ax, (xf + 0.075, tt + G.BOX_H), (xf + 0.075, zp), f"{(zp - tt - G.BOX_H) * 1e3:.0f} mm", fs=7)
    ax.text(xf + 0.013, tt + G.MOUTH_Z, "mouth", fontsize=7, rotation=90, va="center")
    ax.text(body[0], body[1] - 0.27, "gear", fontsize=7, ha="center")
    ax.text(xf + 0.06, tt - 0.04, "table top 0.80 m", fontsize=7, ha="center")
    ax.set_title("SIDE VIEW at the release (red mailbox)", fontsize=10)
    ax.set_xlim(body[0] - 0.22, xf + 0.20)
    ax.set_ylim(tt - 0.08, body[1] + 0.13)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (the drone faces +x)")
    ax.set_ylabel("z [m]")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print("wrote", out)


if __name__ == "__main__":
    main()
