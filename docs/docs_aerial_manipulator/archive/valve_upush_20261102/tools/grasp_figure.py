#!/usr/bin/env python3
"""The U-push grasp in the fin's plane (2026-11-02, archive/valve_upush_20261102): today's grasp (25 mm under the
fin top, at its centre) against the deep one (80 mm under, 25 mm behind the centre), the claw 60 deg below
level (the turn pose [0, 5, 25, 0]). Red = claw material crossing the fin's mid-plane (the bracket, hub and rods
-- what can push the fin along the turn); grey = the claws' outline (beside the fin, never crossing it while
open); blue = the fin; the star = where the fin's back edge meets the red material as the claw moves forward."""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import push_geom_screen as G
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

p = np.radians(60.0)
m = np.abs(G.S[:, 1]) < 0.0025
fw, up = (v * 1e3 for v in G.side(G.S[m], p))
L = G.load("gripper_left.stl") + np.array([0.0, 0.03708, 0.04934])
k = 20; u = G.rng.random((k, 2)); u = np.where(u.sum(1, keepdims=True) > 1, 1 - u, u)
CL = (L[:, None, 0] + u[None, :, :1] * (L[:, None, 1] - L[:, None, 0]) + u[None, :, 1:] * (L[:, None, 2] - L[:, None, 0])).reshape(-1, 3)
cf, cu = (v * 1e3 for v in G.side(CL, p))
fig, axs = plt.subplots(1, 2, figsize=(13, 6.5), sharey=True)
for ax, (dmm, smm, title) in zip(axs, ((25, 0, "today: 25 mm under the fin top, at its centre"),
                                       (80, -25, "deep U: 80 mm under the top, 25 mm behind the centre"))):
    ax.scatter(cf, cu, s=0.5, c="0.75", label="claw (beside the fin)")
    ax.scatter(fw, up, s=1, c="tab:red", label="bracket + hub + rods in the fin's plane")
    back = -20 - smm
    ax.add_patch(plt.Rectangle((back, dmm - 130), 40, 130, fc=(0.2, 0.4, 0.9, 0.18), ec="tab:blue", lw=1.5))
    sel = (up <= dmm) & (up >= dmm - 130) & (fw <= back)
    if sel.any():
        i = np.argmax(np.where(sel, fw, -1e9))
        ax.plot(fw[i], up[i], "k*", ms=14)
        ax.annotate(f"push contact: {dmm - up[i]:.0f} mm under the fin top,\n{back - fw[i]:.0f} mm behind its back edge",
                    (fw[i], up[i]), (fw[i] - 95, up[i] + 25), fontsize=8, arrowprops=dict(arrowstyle="->"))
    else:
        ax.text(-100, dmm + 8, f"nothing of the claw crosses the fin below its top\n(lowest: {up.min() - dmm:+.0f} mm above it)", fontsize=8)
    ax.plot([0], [0], "k+", ms=12); ax.text(2, -9, "claw point", fontsize=8)
    ax.annotate("", (40, -60), (5, -60), arrowprops=dict(arrowstyle="->", lw=1.5)); ax.text(6, -70, "the turn", fontsize=8)
    ax.set_title(title, fontsize=10); ax.set_xlim(-115, 70); ax.set_ylim(-80, 100); ax.set_aspect("equal")
    ax.set_xlabel("along the turn [mm]"); ax.grid(alpha=0.3)
axs[0].set_ylabel("up [mm]"); axs[0].legend(loc="lower left", fontsize=7, markerscale=6)
fig.suptitle("The claw seen in the fin's plane (claw 60 deg below level, the valve-turn pose)", fontsize=11)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures", "grasp_today_vs_deep.png")
fig.savefig(out, dpi=110, bbox_inches="tight"); print(out)
