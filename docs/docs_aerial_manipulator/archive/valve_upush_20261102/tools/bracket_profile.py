#!/usr/bin/env python3
"""Side view of the claw where it crosses the fin's mid-plane (2026-11-02, archive/valve_upush_20261102):
the bracket+hub+rods (link5) and the two claws at a given claw joint position, for a claw pitch,
with the fin drawn for several grasp depths. Writes ../figures/bracket_profile_<pitch>.png."""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import push_geom_screen as G
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

def claw_samples(fn, yoff, c):
    T = G.load(fn) + np.array([0.0, yoff, 0.049340])
    T[:, :, 1] += np.sign(yoff) * c
    k = 30; u = G.rng.random((k, 2)); u = np.where(u.sum(1, keepdims=True) > 1, 1 - u, u)
    return (T[:, None, 0] + u[None, :, :1] * (T[:, None, 1] - T[:, None, 0]) + u[None, :, 1:] * (T[:, None, 2] - T[:, None, 0])).reshape(-1, 3)

def draw(pitch_deg, c_claw, fname, depths=(25, 45, 65), slab=0.0025):
    p = np.radians(pitch_deg)
    fig, ax = plt.subplots(figsize=(8, 7))
    m = np.abs(G.S[:, 1]) < slab
    fw, up = G.side(G.S[m], p)
    ax.scatter(fw * 1e3, up * 1e3, s=1, c="tab:red", label="bracket + hub + rods crossing the fin plane")
    for fn, yo in (("gripper_left.stl", 0.03708), ("gripper_right.stl", -0.03708)):
        Q = claw_samples(fn, yo, c_claw)
        mm = np.abs(Q[:, 1]) < 0.012
        f2, u2 = G.side(Q[mm], p)
        ax.scatter(f2 * 1e3, u2 * 1e3, s=1, c="tab:blue", alpha=0.3)
    ax.scatter([], [], s=6, c="tab:blue", label=f"claw material within 12 mm of the plane (claw joint {c_claw*1e3:+.0f} mm)")
    for dmm, col in zip(depths, ("k", "tab:green", "tab:purple")):
        ax.add_patch(plt.Rectangle((-20, dmm - 130), 40, 130, fill=False, ec=col, lw=1.2))
        ax.text(21, dmm - 4, f"fin, grasp {dmm} mm under its top", color=col, fontsize=8)
    ax.plot([0], [0], "k+", ms=12); ax.text(2, -8, "claw point", fontsize=8)
    ax.set_xlim(-110, 60); ax.set_ylim(-80, 90); ax.set_aspect("equal")
    ax.set_xlabel("along the turn [mm] (the claw moves +)"); ax.set_ylabel("up [mm]")
    ax.set_title(f"claw {pitch_deg:.0f} deg below level: what crosses the fin's mid-plane")
    ax.legend(loc="lower left", fontsize=7, markerscale=4); ax.grid(alpha=0.3)
    os.makedirs(os.path.dirname(fname), exist_ok=True); fig.savefig(fname, dpi=110, bbox_inches="tight")

if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    for pd in (60, 50, 45, 40):
        draw(pd, 0.0, os.path.join(here, "..", "figures", f"bracket_profile_{pd}.png"))
    print("written")
