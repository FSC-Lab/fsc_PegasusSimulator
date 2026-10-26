#!/usr/bin/env python3
"""Figure: the 280 g push (left) and the 510 g grasp + trip (right), four stacked panels each, one
axis per panel. Run with PYTHONNOUSERSITE=1 /usr/bin/python3 (the apt matplotlib).  -> ../fig_flights.png"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE, ORANGE, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"
COLS = [("194310", "280 g box (19:43): push 0.50 m at 21 mm/s", (46.0, 79.0),
         [(48.99, "A"), (70.31, "B"), (71.0, "C"), (75.0, "D")],
         "A jaws clamp   B box-tilt guard -> abort   C jaws open   D box falls off the table"),
        ("195657", "510 g box (19:56): grasp, then the trip", (50.0, 60.5),
         [(52.22, "A"), (53.68, "B"), (53.95, "C"), (55.05, "D"), (58.66, "E")],
         "A jaws close  B q3 guard -> abort  C q3 = 0  D watchdog -> SAFETY  E pilot STAB")]
plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
fig, ax = plt.subplots(4, 2, figsize=(13, 10), sharex="col")
for c, (tag, title, (a, b), ev, key) in enumerate(COLS):
    S = np.load(f"../npz/push_{tag}.npz")
    T = S["T"]; m = (T >= a) & (T <= b); t = T[m]
    i0 = np.searchsorted(T, ev[0][0])
    box = 1e3 * (S["box_al"] - S["box_al"][i0])
    pl = 1e3 * S["mon_plan"]; pl = np.where(S["step"] == 2, pl, np.nan)
    ax[0, c].plot(t, box[m], color=BLUE, lw=1.6, label="box (mocap)")
    ax[0, c].plot(t, pl[m], color=MUTED, lw=1.2, ls="--", label="plan")
    ax[0, c].set_ylabel("box travel along\nthe push [mm]")
    ax[1, c].plot(t, 1e3 * S["ecom_al"][m], color=BLUE, lw=1.4)
    ax[1, c].set_ylabel("vehicle CoM error\nalong the push [mm]")
    ax[2, c].plot(t, S["q2"][m], color=BLUE, lw=1.4, label="q2")
    ax[2, c].plot(t, S["q3"][m], color=ORANGE, lw=1.4, label="q3")
    ax[2, c].axhline(45, color=BLUE, lw=0.8, ls=":"); ax[2, c].axhline(10, color=ORANGE, lw=0.8, ls=":")
    ax[2, c].text(a + 0.2, 46.5, "q2 wall 45", color=MUTED, fontsize=8)
    ax[2, c].text(a + 0.2, 11.5, "q3 guard 10", color=MUTED, fontsize=8)
    ax[2, c].set_ylabel("joint [deg]")
    ax[3, c].plot(t, 1e3 * S["eR0"][m], color=BLUE, lw=1.4)
    ax[3, c].set_ylabel("pitch error [mrad]\n(+ = nose up)")
    ax[3, c].set_xlabel("time in the bag [s]")
    ax[0, c].set_title(title + "\n" + key, color=INK, fontsize=10, loc="left", pad=16)
    for r in range(4):
        ax[r, c].grid(True, color=GRID, lw=0.6)
        for k, (te, lab) in enumerate(ev):
            ax[r, c].axvline(te, color=MUTED, lw=0.8, ls="--")
            if r == 0:
                ax[r, c].text(te, 1.01, lab, transform=ax[r, c].get_xaxis_transform(),
                              fontsize=8, color=INK, ha="center", va="bottom", fontweight="bold")
    ax[0, c].legend(loc="lower right", frameon=False, fontsize=8)
    ax[2, c].legend(loc="lower left", frameon=False, fontsize=8, ncol=2)
ax[2, 1].set_ylim(-45, 55)
ax[1, 1].text(0.99, 0.05, "law signals end at D (SAFETY)", transform=ax[1, 1].transAxes, ha="right", fontsize=8, color=MUTED)
fig.tight_layout(h_pad=1.2)
fig.subplots_adjust(top=0.92)
fig.savefig("../fig_flights.png", dpi=130)
print("wrote ../fig_flights.png")
