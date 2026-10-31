#!/usr/bin/env python3
"""Figure: the 10-24 evening 280 g push (old config) beside the two 10-27 pushes (push pose
[0, 28, 32, 0], push axis at natural inertia, straight-up abort), time from the jaws' clamp.
Run with PYTHONNOUSERSITE=1 /usr/bin/python3 (the apt matplotlib).  -> ../fig_compare.png"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
BLUE, ORANGE, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"
COLS = [("194310", "../../wb_box_pushing_flight_20261024_evening/npz",
         "10-24 19:43, old config: 0.50 m push", 48.99, (-3, 30),
         [(48.99, "A"), (70.31, "B"), (71.0, "C"), (75.0, "D")],
         "A clamp  B box-tilt guard  C jaws open\nD box falls off the table"),
        ("171114", "../npz", "10-27 17:11 (A): 0.40 m push", 47.25, (-3, 30),
         [(47.25, "A"), (67.84, "B"), (71.13, "C"), (71.57, "D")],
         "A clamp  B push complete  C exit after the 3 s calm\nwait (release + open)  D q3 guard -> abort"),
        ("172227", "../npz", "10-27 17:22 (B): 0.40 m push", 52.64, (-3, 30),
         [(52.64, "A"), (71.65, "B"), (71.88, "C"), (72.06, "D")],
         "A clamp  B q3 guard -> abort (348 of 400 mm)\nC jaws open  D q3 < 0 for 0.64 s")]
plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
fig, ax = plt.subplots(4, 3, figsize=(16, 10), sharex=True, sharey="row")
for c, (tag, d, title, tc, (a, b), ev, key) in enumerate(COLS):
    S = np.load(os.path.join(HERE, d, f"push_{tag}.npz"))
    T = S["T"] - tc; m = (T >= a) & (T <= b); t = T[m]
    i0 = np.searchsorted(T, -1.0)
    box = 1e3 * (S["box_al"] - S["box_al"][i0])
    pl = 1e3 * S["mon_plan"]; pl = np.where(S["step"] == 2, pl, np.nan)
    ax[0, c].plot(t, box[m], color=BLUE, lw=1.6, label="box (mocap)")
    ax[0, c].plot(t, pl[m], color=MUTED, lw=1.2, ls="--", label="plan (monitor)")
    ax[1, c].plot(t, 1e3 * S["ecom_al"][m], color=BLUE, lw=1.4)
    ax[2, c].plot(t, S["q2"][m], color=BLUE, lw=1.4, label="q2")
    ax[2, c].plot(t, S["q3"][m], color=ORANGE, lw=1.4, label="q3")
    ax[2, c].axhline(45, color=BLUE, lw=0.8, ls=":"); ax[2, c].axhline(10, color=ORANGE, lw=0.8, ls=":")
    ax[2, c].axhline(0, color=INK, lw=0.6)
    ax[2, c].text(a + 0.3, 46.5, "q2 wall 45", color=MUTED, fontsize=8)
    ax[2, c].text(a + 0.3, 11.5, "q3 guard 10", color=MUTED, fontsize=8)
    ax[3, c].plot(t, S["veh_tilt"][m], color=BLUE, lw=1.4)
    ax[3, c].set_xlabel("time from the jaws' clamp [s]")
    ax[0, c].set_title(title + "\n" + key, color=INK, fontsize=10, loc="left", pad=16)
    for r in range(4):
        ax[r, c].grid(True, color=GRID, lw=0.6)
        for te, lab in ev:
            ax[r, c].axvline(te - tc, color=MUTED, lw=0.8, ls="--")
            if r == 0:
                ax[r, c].text(te - tc, 1.01, lab, transform=ax[r, c].get_xaxis_transform(),
                              fontsize=8, color=INK, ha="center", va="bottom", fontweight="bold")
ax[0, 0].set_ylabel("box travel along\nthe push [mm]")
ax[1, 0].set_ylabel("vehicle CoM error\nalong the push [mm]")
ax[2, 0].set_ylabel("joint [deg]")
ax[3, 0].set_ylabel("vehicle tilt (mocap) [deg]")
ax[0, 0].legend(loc="upper left", frameon=False, fontsize=8)
ax[2, 0].legend(loc="lower left", frameon=False, fontsize=8, ncol=2)
fig.tight_layout(h_pad=1.2)
fig.subplots_adjust(top=0.90)
fig.savefig(os.path.join(HERE, "..", "fig_compare.png"), dpi=120)
print("wrote ../fig_compare.png")
