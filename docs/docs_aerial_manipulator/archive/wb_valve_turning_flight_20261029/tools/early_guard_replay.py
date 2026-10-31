#!/usr/bin/env python3
"""When would an earlier contact guard have fired on the three hardware valve contacts?  Criteria, each
persisting 0.2 s, measured from the grasp: sideways / along-nose CoM error, the arm yaw q1 and wrist roll q4
leaving their value at the grasp, and the law's attitude error.  Compared with what actually ended each run
(10-28: the false tilt trip; 10-29: the q3 rate guard 0.2 s before q3 crossed 0)."""
import os, numpy as np
import vt_common as V
from valve_axis import step_times
HERE = os.path.dirname(os.path.abspath(__file__))
def bag(tag):
    V._c.NPZ = os.path.join(HERE, "..", "..", "wb_valve_turning_flight_20261028", "npz") if tag in ("180652", "181833") else os.path.join(HERE, "..", "npz")
    return V.Bag(tag)
CRIT = [("|e_side| > 60 mm", lambda S: np.abs(S["es"]) > 60), ("|e_fwd| > 60 mm", lambda S: np.abs(S["ef"]) > 60),
        ("|dq1| > 15 deg", lambda S: np.abs(S["q1"]) > 15), ("|dq4| > 20 deg", lambda S: np.abs(S["q4"]) > 20),
        ("|e_R| > 6 deg", lambda S: S["eR"] > 6), ("|e_R| > 10 deg", lambda S: S["eR"] > 10)]
for tag in ("180652", "181833", "145526"):
    b = bag(tag); st = step_times(b)
    tw = b.t("wb"); D = b.f("wb", "data"); ok = np.isfinite(D[:, 50]) & (D[:, 0] == 1); tw, D = tw[ok], D[ok]
    ta = b.t("vatt"); qa = b.f("vatt", "q")
    psi = np.interp(tw, ta, np.unwrap(np.pi/2 - np.arctan2(2*(qa[:, 0]*qa[:, 3]+qa[:, 1]*qa[:, 2]), 1-2*(qa[:, 2]**2+qa[:, 3]**2))))
    ec = D[:, 48:51] - D[:, 45:48]
    S = {"ef": (ec[:, 0]*np.cos(psi) + ec[:, 1]*np.sin(psi))*1e3, "es": (-ec[:, 0]*np.sin(psi) + ec[:, 1]*np.cos(psi))*1e3,
         "eR": np.degrees(np.linalg.norm(D[:, 28:31], axis=1))}
    tg = st["push"]; tab = st["abort"]
    i0 = np.searchsorted(tw, tg + 0.3)
    S["q1"] = np.degrees(D[:, 5] - D[i0, 5]); S["q4"] = np.degrees(D[:, 8] - D[i0, 8])
    w = (tw >= tg + 0.3) & (tw <= tab)
    print(f"===== {tag}: grasp {tg:.2f}, run ended {tab:.2f} (+{tab-tg:.2f} s): {st['abort_msg'][12:70]}")
    for name, f in CRIT:
        m = f(S) & w; tt = tw; hit = None
        run = 0.0
        for k in np.where(w)[0]:
            run = run + (tw[k] - tw[k-1]) if m[k] else 0.0
            if run >= 0.2: hit = tw[k]; break
        print(f"   {name:18s} " + (f"fires at +{hit-tg:5.2f} s after the grasp, {tab-hit:4.2f} s before the end" if hit else "never"))
