#!/usr/bin/env python3
"""fig_flight.png: flight 145526 from Go To Start's end to the pilot's takeover -- the valve against its plan,
the CoM error along the nose / sideways, the arm joints, the vehicle tilt and attitude error, the rotor
commands; events marked.  Run with the numpy-1 system python (PYTHONNOUSERSITE=1 /usr/bin/python3)."""
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from vt_common import Bag, tilt_deg
from valve_axis import obj
b = Bag("145526"); T0, T1 = 31.0, 50.0
tw = b.t("wb"); D = b.f("wb", "data"); ok = np.isfinite(D[:, 50]) & (D[:, 0] == 1); tw, D = tw[ok], D[ok]
ta = b.t("vatt"); qa = b.f("vatt", "q"); tilt = tilt_deg(qa[:, 0], qa[:, 1], qa[:, 2], qa[:, 3])
psi = np.interp(tw, ta, np.unwrap(np.pi/2 - np.arctan2(2*(qa[:, 0]*qa[:, 3]+qa[:, 1]*qa[:, 2]), 1-2*(qa[:, 2]**2+qa[:, 3]**2))))
ec = D[:, 48:51] - D[:, 45:48]
ef = (ec[:, 0]*np.cos(psi) + ec[:, 1]*np.sin(psi))*1e3; es = (-ec[:, 0]*np.sin(psi) + ec[:, 1]*np.cos(psi))*1e3
to, R, p = obj(b); vy = np.degrees(np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0])))
tp = 38.84; vy0 = np.interp(tp, to, vy)
tr = b.t("wbref"); red = np.stack([b.f("wbref", f"r_ed.{c}") for c in "xy"], 1)
az = np.degrees(np.unwrap(np.arctan2(red[:, 1]-0.004, red[:, 0]-0.855))); az0 = np.interp(tp, tr, az)
tj = b.t("js"); P = np.degrees(b.f("js", "position"))   # joint2, joint3, joint1, joint4
EV = [(33.40, "jaws open"), (38.84, "grasp"), (40.34, "turn starts"), (45.63, "rotor sat. / q3 guard"),
      (46.30, "watchdog: SAFETY"), (46.99, "mocap guard: jump"), (48.00, "LOST")]
fig, ax = plt.subplots(5, 1, figsize=(11, 12.5), sharex=True)
m = (to > T0) & (to < T1)
ax[0].plot(to[m], vy[m] - vy0, label="valve angle (obj_0)"); mr = (tr > tp) & (tr < 46.3)
ax[0].plot(tr[mr], az[mr] - az0, "k--", label="planned turn"); ax[0].set_ylabel("deg"); ax[0].set_ylim(-40, 40)
mw = (tw > T0) & (tw < 46.3)
ax[1].plot(tw[mw], ef[mw], label="CoM error along the nose"); ax[1].plot(tw[mw], es[mw], label="CoM error sideways"); ax[1].set_ylabel("mm"); ax[1].set_ylim(-300, 300)
mj = (tj > T0) & (tj < T1)
for i, (k, n) in enumerate(((2, "q1 arm yaw"), (0, "q2"), (1, "q3 elbow"), (3, "q4 wrist roll"))):
    ax[2].plot(tj[mj], P[mj, k], label=n)
ax[2].axhline(3, color="r", lw=0.8, ls=":"); ax[2].set_ylabel("deg (encoder)"); ax[2].set_ylim(-50, 60)
mt = (ta > T0) & (ta < T1)
ax[3].plot(ta[mt], tilt[mt], label="tilt (PX4)"); ax[3].plot(tw[mw], np.degrees(np.linalg.norm(D[mw, 28:31], axis=1)), label="|attitude error| (law)")
ax[3].axhline(15, color="r", lw=0.8, ls=":"); ax[3].set_ylabel("deg"); ax[3].set_ylim(0, 60)
ax[4].plot(tw[mw], D[mw, 41:45].min(1), label="lowest rotor"); ax[4].plot(tw[mw], D[mw, 41:45].max(1), label="highest rotor")
ax[4].set_ylabel("rotor command"); ax[4].set_ylim(-0.05, 1.05); ax[4].set_xlabel("time in bag (s)")
for a in ax:
    for t, n in EV:
        a.axvline(t, color="0.5", lw=0.7)
    a.legend(loc="upper left", fontsize=8); a.grid(alpha=0.3)
for t, n in EV:
    ax[0].text(t, 41, n, rotation=90, fontsize=7, va="bottom", ha="center")
ax[0].set_title("2026-10-29 valve turn (bag 145526): contact sway -> arm collapse -> SAFETY could not recover", pad=55)
fig.tight_layout(); fig.savefig("../fig_flight.png", dpi=110)
print("wrote ../fig_flight.png")
