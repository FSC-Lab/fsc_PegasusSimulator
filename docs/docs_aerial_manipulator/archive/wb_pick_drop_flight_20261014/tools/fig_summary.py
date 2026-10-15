"""Summary figure for the 2026-10-14 pick-and-drop free flights -> ../figures/pick_drop_1014_summary.png
(a) PD-1 pitch attitude through the drop with the bin spans shaded, (b) the gain/lag-fitted external pitch moment per
hover window (moment_residual.json robust pass), (c) the standing yaw lever per hardware flight since 2026-09-18
(yaw_history.json).  Run moment_residual.py (ROBUST=1) and yaw_history.py first."""
import os, json, datetime
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pd_common import load, px4_rpy, OUT, FIG

BLUE, ORANGE = "#2a78d6", "#eb6834"
INK, INK2, GRID, SURF, SHADE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb", "#fbe3d6"
plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.facecolor": SURF, "figure.facecolor": SURF, "axes.grid": True,
                     "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
                     "lines.linewidth": 1.4})

fig = plt.figure(figsize=(11, 10.5))
gs = fig.add_gridspec(3, 1, height_ratios=[1.0, 1.05, 1.15], hspace=0.55)

# (a) pitch through the drop, flight 1
ax = fig.add_subplot(gs[0])
d, t0 = load("d1")
tp, ro, pi_, ya = px4_rpy(d, t0)
s = (tp > 66) & (tp < 136)
ax.axvspan(86.0, 120.5, color=SHADE, lw=0, zorder=0)
ax.plot(tp[s], np.degrees(pi_[s]), color=BLUE, label="pitch (PX4 attitude)")
for x, lab in [(70.4, "hold\n0.70 m"), (76.4, "drop leg:\nclimb, fly to bin"), (86.0, "front rotors over the bin:\n1.35 m, then 1.65 m (abort, from 92 s)"),
               (120.5, "transit"), (128.6, "open hover\n(0, 0) 1.68 m")]:
    ax.axvline(x, color=INK2, lw=0.6, ls=":")
    ax.text(x + 0.4, 1.02, lab, transform=ax.get_xaxis_transform(), ha="left", va="bottom", color=INK2, fontsize=7.5)
ax.set_xlim(66, 136); ax.set_ylabel("pitch [deg]"); ax.set_xlabel("time since bag start [s]  (PD-1, 2026-10-14 17:28, full bin)")
ax.set_title("(a) The oscillation starts over the bin and dies out at the same height away from it", loc="left", color=INK, fontsize=10, pad=34)

# (b) external pitch moment per window
ax = fig.add_subplot(gs[1])
rows = json.load(open(os.path.join(OUT, "moment_residual_robust.json")))
SHORT = {"PD-1 pick hover": "PD-1\npick hover\n1.15 m", "PD-1 Drop Start": "PD-1\nDrop Start\n0.70 m",
         "PD-1 OVER FULL BIN": "PD-1\nover bin\n1.35 m", "PD-1 over full bin 1.65": "PD-1\nover bin\n1.65 m",
         "PD-1 open hover": "PD-1\nopen (0,0)\n1.68 m", "PP-1 10-09 same point 1.32": "10-09 PP-1\ndrop point\n1.32 m, hat",
         "PP-1 10-09 same point 1.31 m #2": "10-09 PP-1\ndrop point\n1.31 m, hat", "PP-2 10-09": "10-09 PP-2\ndrop point\nhat+basket",
         "PP-4 10-10": "10-10 PP-4\ndrop point\nhat+basket", "PD-2 hover": "PD-2\nhover\n1.0 m"}
def short(w):
    for k in sorted(SHORT, key=len, reverse=True):
        if w.startswith(k):
            return SHORT[k]
    return w
lab = [short(r["window"]) for r in rows]
v = [r["minres_pitch"] for r in rows]
binw = ["BIN" in r["window"].upper() for r in rows]
cols = [ORANGE if b else BLUE for b in binw]
ax.bar(range(len(v)), v, color=cols, width=0.62, edgecolor=SURF, linewidth=2)
for i, x in enumerate(v):
    ax.text(i, x + 0.0015, f"{x:.3f}", ha="center", va="bottom", color=INK, fontsize=7.5)
ax.set_xticks(range(len(v))); ax.set_xticklabels(lab, fontsize=7.5); ax.grid(axis="x", visible=False)
ax.set_ylabel("external pitch moment\n0.3-3 Hz rms [N m]")
from matplotlib.patches import Patch
ax.legend(handles=[Patch(color=ORANGE, label="front rotors over the full bin"), Patch(color=BLUE, label="elsewhere / same point over the hat (no bin)")],
          loc="upper right", frameon=False, fontsize=8, labelcolor=INK)
ax.set_title("(b) Moment the airframe felt, back-solved from gyro and rotor model (gain, lag and shift fitted per window)",
             loc="left", color=INK, fontsize=10)

# (c) yaw lever history
ax = fig.add_subplot(gs[2])
rows = json.load(open(os.path.join(OUT, "yaw_history.json")))
def when(r):
    s = r["stamp"].replace("ps4", "")
    return datetime.datetime.strptime(s, "%Y%m%d_%H%M%S")
for rig, c, name in [("WB", BLUE, "whole-body rig"), ("DEC", ORANGE, "decoupled rig")]:
    rr = [r for r in rows if r["rig"] == rig]
    ax.plot([when(r) for r in rr], [r["lever_mm"] for r in rr], "o", color=c, ms=7, mec=SURF, mew=1.5, label=name, zorder=3)
for when_s, txt, ha, y in [("20260924_120400", "09-24 F1\nfly-away crash", "center", 9.6), ("20261009_183000", "10-09 PP-3\ntipped 57 deg ", "right", 9.6),
                          ("20261010_191300", " 10-10 PP-4\n 3.8 m/s impact", "left", 7.6)]:
    w = datetime.datetime.strptime(when_s, "%Y%m%d_%H%M%S")
    ax.axvline(w, color=INK2, lw=0.6, ls=":")
    ax.text(w, y, txt, ha=ha, va="top", color=INK2, fontsize=7)
for r in rows[-2:]:
    ax.annotate("PD-1" if "172845" in r["stamp"] else "PD-2", (when(r), r["lever_mm"]), xytext=(6, -3), textcoords="offset points", color=INK, fontsize=8)
ax.set_ylim(0, 10); ax.set_ylabel("standing yaw lever tau_z / T [mm]\n(clockwise torque the rotors hold off)")
ax.legend(loc="upper left", frameon=False, fontsize=8, labelcolor=INK)
import matplotlib.dates as mdates
ax.xaxis.set_major_formatter(mdates.DateFormatter("%m-%d"))
ax.set_title("(c) Standing yaw torque per hardware flight (one fixed rotor model; T ~ 37 N, so 1 mm = 0.037 N m)", loc="left", color=INK, fontsize=10)

os.makedirs(FIG, exist_ok=True)
fig.savefig(os.path.join(FIG, "pick_drop_1014_summary.png"), dpi=150, bbox_inches="tight")
print("wrote", os.path.join(FIG, "pick_drop_1014_summary.png"))
