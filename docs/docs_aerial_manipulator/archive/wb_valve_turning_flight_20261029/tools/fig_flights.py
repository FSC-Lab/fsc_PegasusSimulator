#!/usr/bin/env python3
"""Figure: both 10-28 valve flights, contact phase.  (a) valve angle vs plan vs plan + 0.75 x the
fore-aft CoM error / r (the free-axis anchor), (b) CoM error fore-aft / lateral, (c) the guard's tilt
reading against its 2 deg limit and the frame artifact, (d) the arm joints.  Run with the system
python (PYTHONNOUSERSITE=1, apt matplotlib)."""
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from vt_common import Bag, BAGS
from valve_axis import obj, step_times
GAM = np.radians(1.50)
fig, ax = plt.subplots(4, 2, figsize=(13, 11), sharex="col")
for c, tag in enumerate(BAGS):
    b = Bag(tag); st = step_times(b); T = np.load(f"../npz/turn_{tag}.npz"); g = T["g"]
    tz = st["push"] + 1.5; x = g - tz
    a = ax[0, c]
    a.plot(x, T["th_plan"], "k--", lw=1.5, label="plan")
    a.plot(x, T["th_v"], color="#c0392b", lw=1.6, label="valve (obj_0 yaw)")
    a.plot(x, T["th_plan"] + 0.75 * np.degrees(T["ec_t"] / 0.160) + (T["th_v"] - T["th_plan"] - 0.75*np.degrees(T["ec_t"]/0.16))[(g > st["push"]) & (g < st["push"] + 0.4)].mean(),
           color="#2e86c1", lw=1.0, alpha=0.9, label="plan + 0.75 x fore-aft CoM err / r")
    a.set_ylabel("valve angle [deg]"); a.set_title(f"{tag}  (abort {st['abort']-tz:+.1f} s into the turn)")
    a = ax[1, c]
    a.plot(x, T["ec_t"] * 1e3, color="#2e86c1", label="fore-aft (along nose = tangent)")
    a.plot(x, T["ec_r"] * 1e3, color="#8e44ad", label="lateral (radial at the fin)")
    a.set_ylabel("CoM error [mm]")
    a = ax[2, c]
    t, R, p = obj(b); i0 = np.searchsorted(t, st["push"])
    tl = np.degrees(np.arccos(np.clip(R[:, :, 2] @ R[i0][:, 2], -1, 1)))
    yaw = np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0])); art = 2*np.degrees(np.arcsin(np.sin(GAM)*np.abs(np.sin((yaw - yaw[i0])/2))))
    m = (t > st["push"]) & (t < st["abort"] + 0.05)
    a.plot(t[m] - tz, tl[m], color="#7f8c8d", lw=0.6, label="guard's tilt reading (obj_0 z vs push start)")
    a.plot(t[m] - tz, art[m], color="#e67e22", lw=1.4, label="frame artifact (gamma 1.5 deg)")
    a.axhline(2.0, color="#c0392b", ls=":", label="limit 2 deg / 0.2 s")
    a.set_ylabel("tilt [deg]"); a.set_ylim(0, 8)
    a = ax[3, c]
    for j, (lab, col) in enumerate((("q1", "#16a085"), ("q2", "#d35400"), ("q3", "#2c3e50"), ("q4", "#9b59b6"))):
        a.plot(x, T["q"][:, j], color=col, lw=1.1, label=lab)
    a.axhline(3, color="#2c3e50", ls=":", lw=0.8); a.set_ylabel("joint [deg]"); a.set_xlabel("time from the turn start [s]")
    for r in range(4):
        ax[r, c].axvline(st["push"] - tz, color="0.6", lw=0.8); ax[r, c].axvline(0, color="0.6", lw=0.8)
        ax[r, c].axvline(st["abort"] - tz, color="#c0392b", lw=0.8); ax[r, c].set_xlim(st["push"] - tz - 1.5, st["abort"] - tz + 0.5)
        ax[r, c].grid(alpha=0.3)
for r in range(4): ax[r, 0].legend(fontsize=7, loc="upper left")
fig.tight_layout(); fig.savefig("../fig_flights.png", dpi=110)
print("wrote ../fig_flights.png")
