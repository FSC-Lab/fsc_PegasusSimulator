"""Figures for the 2026-10-16 report (drawn light; the page inverts them onto black).
  ../figures/carry.png  the loaded carry, grasp -> drop click: 10-15 D-1 (grey) vs 10-16 D-1 / D-2. Rows: vehicle pitch
                        (0.3-2 Hz), wrist roll q4, j2 error, j2 Present Current (1 s mean).
  ../figures/drop.png   top view of the basket from release to rest, relative to the drop point.
usage: fig16.py"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pd15_common as P
from pd15_common import band, rosout, wbdebug, px4_rpy
from drop16 import drop_point
from release import vel

OLD = os.path.join(P.NPZ, "..", "..", "wb_pick_drop_flight_20261015", "npz")
RUNS = [("10-15 D-1", os.path.join(OLD, "d1.npz"), "#8a8984", 1.2),
        ("10-16 D-1", os.path.join(P.NPZ, "d1.npz"), "#2a78d6", 1.4),
        ("10-16 D-2", os.path.join(P.NPZ, "d2.npz"), "#eb6834", 1.4)]
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3dd"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False})


def load(path):
    d = np.load(path)
    return d, min(d[k][0] for k in d.files if k.endswith("__recv"))


def carry():
    fig, ax = plt.subplots(4, 1, figsize=(10, 9.2), sharex=True)
    for name, path, col, lw in RUNS:
        d, t0 = load(path)
        tg = [t for t, n, m in rosout(d, t0, contains="Exit To Pick at once")][0]
        tc = [t for t, n, m in rosout(d, t0, contains="gripper OPEN") if t > tg][0]
        a, b = tg - 1.0, tc + 1.0
        ta, rr, pp, yy = px4_rpy(d, t0)
        m = (ta >= a) & (ta <= b)
        tu, pb = band(np.degrees(pp[m]), ta[m], 0.3, 2.0)
        ax[0].plot(tu - tg, pb, color=col, lw=lw, label=name)
        tw, D = wbdebug(d, t0)
        mw = (tw >= a) & (tw <= b)
        ax[1].plot(tw[mw] - tg, np.degrees(D[mw, 8]), color=col, lw=lw)
        ax[2].plot(tw[mw] - tg, np.degrees(D[mw, 6] - D[mw, 10]), color=col, lw=lw)
        tj = d["js__recv"] - t0; I = np.abs(d["js__effort"][:, 0])          # joint_states column 0 = joint2
        mj = (tj >= a - 1) & (tj <= b)
        w = max(1, int(round(1.0 / np.median(np.diff(tj)))))
        Im = np.convolve(I[mj], np.ones(w) / w, mode="same")
        ax[3].plot(tj[mj] - tg, Im, color=col, lw=lw)
        for k in range(4):
            ax[k].axvline(tc - tg, color=col, lw=0.8, ls=":")
    ax[3].axhline(250, color=INK, lw=0.8, ls="--")
    ax[3].text(0.2, 256, "≈250 counts: j2 overload shutdown, 2026-08-26", color=INK, fontsize=9)
    ax[2].annotate("D-2: j2 held 1.1° low for 18 s,\nthen broke loose (+12.7°)", xy=(22.5, 12.5), xytext=(8, 9.0),
                   color=INK, fontsize=9, arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax[0].set_ylabel("pitch, 0.3–2 Hz (°)"); ax[1].set_ylabel("wrist roll q4 (°)")
    ax[2].set_ylabel("j2 error q2 − q2,d (°)"); ax[3].set_ylabel("j2 |current|, 1 s mean\n(counts, 2.69 mA)")
    ax[3].set_xlabel("time since the grasp (s); dotted line = drop click")
    ax[0].legend(loc="upper right", ncol=3)
    ax[3].set_xlim(-1, 39)
    fig.align_ylabels(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(P.FIG, "carry.png"), dpi=130)


def drop():
    fig, ax = plt.subplots(figsize=(7.6, 5.4))
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(25 * np.cos(th), 25 * np.sin(th), color=MUTED, lw=0.9, ls="--")
    ax.text(21, 18, "25 mm payload gate", ha="left", va="bottom", color=MUTED, fontsize=8.5)
    ax.plot(0, 0, marker="+", ms=14, color=INK, mew=1.4)
    for name, path, col, lw in RUNS:
        d, t0 = load(path)
        pt, ee, yaw = drop_point(d)
        to, Po, Qo = P.obj_pose(d, t0)
        tg = [t for t, n, m in rosout(d, t0, contains="Exit To Pick at once")][0]
        tc = [t for t, n, m in rosout(d, t0, contains="gripper OPEN") if t > tg][0]
        i = np.searchsorted(to, tc)
        k0 = i + np.argmax(Po[i:, 2] < Po[i, 2] - 0.01)
        rel = (Po[k0:, :2] - pt[:2]) * 1e3
        ax.plot(rel[:, 0], rel[:, 1], color=col, lw=lw * 0.8)
        ax.plot(*rel[0], marker="o", mfc="white", mec=col, mew=1.6, ms=9, ls="")
        ax.plot(*rel[-1], marker="s", color=col, ms=9, ls="")
        tilt = P.tilt_of(Qo[-1:])[0]
        ax.annotate(f"{name}\nrest {np.linalg.norm(rel[-1]):.0f} mm off, {tilt:.0f}° tilt", xy=rel[-1],
                    xytext={"10-15 D-1": (rel[-1][0] - 95, rel[-1][1] + 14), "10-16 D-1": (rel[-1][0] - 40, rel[-1][1] + 22),
                            "10-16 D-2": (rel[-1][0] - 95, rel[-1][1] - 50)}[name],
                    color=INK, fontsize=9)
    ax.plot([], [], marker="o", mfc="white", mec=INK, ls="", label="basket leaves the jaws")
    ax.plot([], [], marker="s", color=INK, ls="", label="basket at rest")
    ax.annotate("", xy=(-110, 135), xytext=(-40, 135), arrowprops=dict(arrowstyle="->", color=INK, lw=1.2))
    ax.text(-75, 142, "vehicle nose (yaw −180°)", ha="center", color=INK, fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlim(-130, 180); ax.set_ylim(-70, 160)
    ax.set_xlabel("world x − drop point (mm)"); ax.set_ylabel("world y − drop point (mm)")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(P.FIG, "drop.png"), dpi=130)


if __name__ == "__main__":
    carry()
    drop()
    print("wrote", P.FIG)
