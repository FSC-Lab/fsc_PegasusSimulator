#!/usr/bin/env python3
"""Failure timeline of flight_wb_l1_4d_pick_and_drop_20261019_170849 -> ../figures/timeline.png
usage: PYTHONNOUSERSITE=1 /usr/bin/python3 fig_timeline.py   (after timeline_prep.py)"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
S = np.load(os.path.join(HERE, "..", "npz", "timeline.npz"))
OUT = os.path.join(HERE, "..", "figures")
os.makedirs(OUT, exist_ok=True)

INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
DARK = [(44.889, 47.265), (47.759, 65.746)]       # PX4 stream absent at the recorder
EVENTS = [(43.0, "lift starts"), (45.89, "node: feedback lost\n-> SAFETY"), (57.2, "flip"), (62.3, "righted\nby hand")]
XL = (38, 70)

plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.spines.top": False,
                     "axes.spines.right": False})
fig, ax = plt.subplots(4, 1, figsize=(9.5, 8.2), sharex=True,
                       gridspec_kw={"height_ratios": [1.5, 1, 1, 1], "hspace": 0.12})
for a in ax:
    for lo, hi in DARK:
        a.axvspan(lo, hi, color="#d9d8d3", alpha=0.55, lw=0, zorder=0)
    for t, _ in EVENTS:
        a.axvline(t, color=MUTED, lw=0.8, ls=(0, (3, 3)), zorder=1)
    a.grid(axis="y", color=GRID, lw=0.6)
    a.set_axisbelow(True)

bt = S["bin_t"]
for key, col, lab, dy in (("px4", BLUE, "PX4 -> Agent -> ROS (vehicle_attitude)", 0.0),
                          ("orin_to_laptop", ORANGE, "Orin -> laptop over WiFi (joint_states)", 0.0),
                          ("laptop_to_orin", AQUA, "laptop -> Orin over WiFi (mocap relayed by the estimator)", 0.0)):
    ax[0].plot(bt, S[key] + dy, color=col, lw=2, label=lab, drawstyle="steps-mid")
ax[0].set_ylim(-0.08, 1.35)
ax[0].set_ylabel("rate / nominal")
ax[0].legend(loc="lower left", bbox_to_anchor=(0.0, 1.02), frameon=False, fontsize=8, ncol=1, labelcolor=INK2, borderaxespad=0)
ax[0].text(56.3, 0.12, "PX4 <-> Agent link down\n(both directions)", ha="center", va="bottom", color=INK, fontsize=8.5)
for t, lab in EVENTS:
    ax[0].text(t + 0.25, 1.30, lab, va="top", ha="left", color=INK2, fontsize=7.5)

mt = S["mc_t"]
ax[1].plot(mt, S["z"], color=INK, lw=1.6)
ax[1].set_ylabel("height z [m]\n(mocap)")
ax[2].plot(mt, S["tilt"], color=INK, lw=1.6)
ax[2].set_ylabel("tilt [deg]\n(mocap)")
ax[2].set_ylim(-5, 185)
ax[3].plot(S["js_t"], S["q2_deg"], color=INK, lw=1.6)
ax[3].set_ylabel("joint 2 [deg]")
ax[3].set_xlabel("time since bag start [s]  (bag start 17:08:49.575 EDT)")
ax[3].set_xlim(*XL)
ax[3].annotate("j2 slips 12 deg as the basket\nweight comes on (cut at 44.89 s)", xy=(44.85, 27), xytext=(49.5, 24),
               fontsize=7.5, color=INK2, arrowprops={"arrowstyle": "-", "color": MUTED, "lw": 0.8})
fig.suptitle("1019 pick-and-drop: the PX4 stream died, WiFi did not", x=0.08, y=0.995, ha="left", fontsize=11, color=INK)
fig.savefig(os.path.join(OUT, "timeline.png"), dpi=150, bbox_inches="tight", facecolor="white")
print("wrote", os.path.join(OUT, "timeline.png"))
