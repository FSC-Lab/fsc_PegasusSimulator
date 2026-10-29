#!/usr/bin/env python3
"""Figure: where the 10-28 contact sway gets its energy (mean power at the sway frequency, from
energy_flow.py / energy_flow2.py).  Red = feeds the motion, blue = damps it."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
RED, BLUE, INK, INK2, GRID, SURF = "#e34948", "#2a78d6", "#0b0b0b", "#52514e", "#e1e0d9", "#fcfcfb"
TERMS = ["base, law COMMAND (u1 x commanded tilt)", "base, ROLL TRACKING error (-u1 e_R,nose)",
         "base, PX4 attitude ESTIMATE error", "base, observer -d_hat_t consumed",
         "q1, law joint torque (all terms)", "   of which the observer-driven share",
         "q4, law joint torque (all terms)", "   of which the observer-driven share"]
DATA = {"180652  (sway 59 mm at 0.49 Hz)": [-203.3, +263.7, +10.7, +6.1, +39.0, +7.6, +27.1, +14.4],
        "181833  (sway 14 mm at 0.63 Hz)": [-20.5, +37.0, -0.5, +0.5, +8.2, +1.6, +9.1, +2.3]}
fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6), facecolor=SURF, sharey=True)
for ax, (title, vals) in zip(axes, DATA.items()):
    ax.set_facecolor(SURF)
    y = list(range(len(TERMS)))[::-1]
    ax.barh(y, vals, height=0.62, color=[RED if v > 0 else BLUE for v in vals], edgecolor=SURF, linewidth=2)
    lim = max(abs(v) for v in vals) * 1.35
    ax.set_xlim(-lim, lim); ax.axvline(0, color="#c3c2b7", lw=1)
    for yy, v in zip(y, vals):
        ax.text(v + (0.02 * lim if v >= 0 else -0.02 * lim), yy, f"{v:+.1f}", va="center",
                ha="left" if v >= 0 else "right", color=INK2, fontsize=9)
    ax.set_title(title, color=INK, fontsize=11, loc="left")
    ax.set_xlabel("mean power at the sway frequency [mW]", color=INK2, fontsize=9)
    ax.grid(axis="x", color=GRID, lw=0.6); ax.set_axisbelow(True)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color("#c3c2b7"); ax.tick_params(colors=INK2, labelsize=9, length=0)
axes[0].set_yticks(list(range(len(TERMS)))[::-1]); axes[0].set_yticklabels(TERMS, color=INK, fontsize=9)
fig.suptitle("10-28 valve contact sway: the energy enters through the base ROLL tracking error, not the observer",
             color=INK, fontsize=12, x=0.01, ha="left")
fig.text(0.01, 0.01, "red = feeds the motion, blue = damps it.  Lock-in at the sway frequency; power = 1/2 x force amplitude x velocity amplitude x cos(phase).", color=INK2, fontsize=8.5)
fig.tight_layout(rect=(0, 0.04, 1, 0.94))
out = os.path.join(HERE, "..", "figures", "energy_split.png")
fig.savefig(out, dpi=150, facecolor=SURF); print(out)
