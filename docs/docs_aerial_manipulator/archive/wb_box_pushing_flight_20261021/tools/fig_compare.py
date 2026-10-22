#!/usr/bin/env python3
"""Figure: the formal hardware push (170123) against an Isaac run of the same task (v21_2), aligned at
the push start: q2, q3 (with the hardware limits and the planner's q3 guard), the box's travel along
the push against the plan, and the CoM error along the arm (world y).
Run with the plotting python: PYTHONNOUSERSITE=1 /usr/bin/python3 fig_compare.py"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from phases import hw_run, sim_run, win
from bp_common import Bag

HW_C, SIM_C = "#2a78d6", "#eb6834"
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fig_hw_vs_sim.png")


def series(r, ref):
    m = r["marks"]
    t0 = m["push:start"]
    a, c = m["ready:end"] - 1.0, m["release:start"] + (1.0 if r["name"].startswith("HW") else 0.0)
    j = win(r["joints"], a, c)
    d = win(r["dbg"], a, c)
    bx = win(r["box"], a, c)
    s = -(bx[:, 2] - np.interp(t0, r["box"][:, 0], r["box"][:, 2])) * 1e3
    return dict(tj=j[:, 0] - t0, q2=np.degrees(j[:, 2]), q3=np.degrees(j[:, 3]),
                tb=bx[:, 0] - t0, box=s, td=d[:, 0] - t0,
                ecy=(d[:, 2 + 49] - d[:, 2 + 46]) * 1e3,
                tr=ref[0] - t0, plan=ref[1],
                ph=[(m["ready:end"] - t0, "claw world-held\n(jaws open)"), (m["gripper_close:end"] - t0, "settle"),
                    (1.5, "slide"), (m["push:end"] - t0, "post-push")], end=m["release:start"] - t0)


def plan_hw():
    b = Bag("170123")
    t = b.t("wbref")
    y = b.f("wbref", "r_ed.y")
    y0 = np.interp(39.58, t, y)
    return t, -(y - y0) * 1e3


def plan_sim(tag):
    z = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "box_pushing_speed_20261021", "runs", tag + ".npz"))
    ref = z["ref"]
    mk = {str(n): float(v) for n, v in zip(z["marks_name"], z["marks_t"])}
    y0 = np.interp(mk["push:start"], ref[:, 0], ref[:, 2])
    return ref[:, 0], -(ref[:, 2] - y0) * 1e3


hw = series(hw_run(), plan_hw())
sm = series(sim_run("v21_2"), plan_sim("v21_2"))

plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False,
                     "axes.spines.right": False})
fig, ax = plt.subplots(4, 1, figsize=(9.5, 9.0), sharex=True, constrained_layout=True)
for a in ax:
    a.grid(True, color=GRID, lw=0.6)
    a.set_facecolor("#fcfcfb")
fig.patch.set_facecolor("#fcfcfb")
# phases (hardware timing) as boundaries
for a in ax:
    for x, _ in hw["ph"]:
        a.axvline(x, color=MUTED, lw=0.6, ls=":")
    a.axvline(hw["end"], color="#b0342c", lw=0.9, ls="-")
for x, lab in hw["ph"]:
    ax[0].text(x + 0.05, 1.02, lab, transform=ax[0].get_xaxis_transform(), color=MUTED, fontsize=8, va="bottom")
ax[0].text(hw["end"] + 0.05, 0.05, "q3 guard trip\n-> abort", transform=ax[0].get_xaxis_transform(), color="#b0342c", fontsize=8)

ax[0].plot(hw["tj"], hw["q2"], color=HW_C, lw=1.6, label="hardware 10-21 (170123)")
ax[0].plot(sm["tj"], sm["q2"], color=SIM_C, lw=1.6, ls="--", label="Isaac, same task (v21_2)")
ax[0].axhline(45, color=MUTED, lw=0.8, ls="-.")
ax[0].text(ax[0].get_xlim()[0] if False else -4.2, 45.8, "q2 hardware limit 45°", color=MUTED, fontsize=8)
ax[0].set_ylabel("q2 [deg]")
ax[0].legend(loc="lower left", frameon=False, fontsize=8)

ax[1].plot(hw["tj"], hw["q3"], color=HW_C, lw=1.6)
ax[1].plot(sm["tj"], sm["q3"], color=SIM_C, lw=1.6, ls="--")
ax[1].axhline(50, color=MUTED, lw=0.8, ls="-.")
ax[1].axhline(10, color="#b0342c", lw=0.8, ls="-.")
ax[1].text(-4.2, 51, "q3 stop 50°", color=MUTED, fontsize=8)
ax[1].text(-4.2, 11, "planner q3 guard 10°", color="#b0342c", fontsize=8)
ax[1].set_ylabel("q3 [deg]")

ax[2].plot(hw["tr"], hw["plan"], color=MUTED, lw=1.0, ls=":", label="claw reference (plan)")
ax[2].plot(hw["tb"], hw["box"], color=HW_C, lw=1.6)
ax[2].plot(sm["tb"], sm["box"], color=SIM_C, lw=1.6, ls="--")
ax[2].set_ylabel("box travel\nalong the push [mm]")
ax[2].set_ylim(-25, 120)
ax[2].legend(loc="upper left", frameon=False, fontsize=8)

ax[3].plot(hw["td"], hw["ecy"], color=HW_C, lw=1.4)
ax[3].plot(sm["td"], sm["ecy"], color=SIM_C, lw=1.4, ls="--")
ax[3].axhline(0, color=MUTED, lw=0.6)
ax[3].set_ylabel("CoM error along\nthe arm (world y) [mm]")
ax[3].set_xlabel("time from the push start [s]")
ax[3].set_xlim(-4.3, 8.5)
fig.suptitle("Box pushing 240 g / 0.10 m / 21 mm/s: hardware vs Isaac", color=INK, fontsize=11, x=0.02, ha="left")
fig.savefig(OUT, dpi=140)
print("wrote", OUT)
