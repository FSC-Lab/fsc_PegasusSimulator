#!/usr/bin/env python3
"""Per planner HOLD (>= 2 s, first 1 s dropped), whole-flight-filtered signals (ab.py's): location, arm pose, the law's
slow moment estimate M_hat (0.05-1 Hz), translational estimate F_hat (0.1-1 Hz), the true tilt (0.1-1 Hz), the CoM
wander about the hold's mean."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import ab
H = ab.H
FL = [("144602", "wb_hover_wander_history_20261103", "circle 10-28", 0), ("150821", "wb_hover_wander_history_20261103", "circle 10-28", 0),
      ("182844", "wb_sidewall_ab_20261103", "circle 11-02", 1),
      ("161640", "wb_hover_wander_history_20261103", "PnD 10-28", 0), ("162711", "wb_hover_wander_history_20261103", "PnD 10-28", 0),
      ("164816", "wb_hover_wander_history_20261103", "PnD 10-28", 0), ("165603", "wb_hover_wander_history_20261103", "PnD 10-28", 0),
      ("192323", "wb_sidewall_ab_20261103", "PnD 11-02", 1)]
for tag, d, lab, wall in FL:
    H._c.NPZ = os.path.join(ab.ARCH, d, "npz"); b = H._c.Bag(tag)
    md = b.edges("wbmode"); t_dir = [t for t, s in md if s == "DIRECT"]; t_saf = [t for t, s in md if s == "SAFETY" and t > t_dir[0]]
    T0, T1 = t_dir[0] + 2.0, (t_saf[0] if t_saf else b.t("wbmode")[-1]) - 0.5
    T = np.arange(T0, T1, ab.DT)
    tw, D = H.wb(b)
    Mh = np.stack([ab.bp(np.interp(T, tw, D[:, k]), 0.05, 1.0) for k in (34, 35)], 1)
    Fh = np.stack([ab.bp(np.interp(T, tw, D[:, k]), 0.1, 1.0) for k in (31, 32)], 1)
    tv = b.t("vrpn"); Q = np.stack([b.f("vrpn", f"pose.orientation.{k}") for k in "wxyz"], 1)
    q = np.stack([np.interp(T, tv, Q[:, k]) for k in range(4)], 1); w_, x_, y_, z_ = q.T
    tilt = np.stack([ab.bp(2 * (x_ * z_ + w_ * y_), 0.1, 1.0), ab.bp(2 * (y_ * z_ - w_ * x_), 0.1, 1.0)], 1)
    ecom = np.stack([np.interp(T, tw, D[:, 48 + k] - D[:, 45 + k]) for k in range(2)], 1)
    xc = np.stack([np.interp(T, tw, D[:, 48 + k]) for k in range(2)], 1); qd = np.degrees(np.stack([np.interp(T, tw, D[:, 5 + k]) for k in range(4)], 1))
    st = b.edges("pl_status")
    for i, (ts, v) in enumerate(st):
        te = st[i + 1][0] if i + 1 < len(st) else T1
        if not (v.startswith("HOLD") and te - ts >= 2.0): continue
        m = (T >= ts + 1.0) & (T <= te)
        if m.sum() < 80: continue
        r = lambda s_: np.sqrt((s_[m] ** 2).mean(0).sum())
        e = ecom[m] - ecom[m].mean(0)
        qq = qd[m].mean(0)
        print(f"{lab:12s} {tag} {'W' if wall else ' '} hold {ts:5.1f}+{te-ts-1:3.1f}s at ({xc[m,0].mean():+.2f},{xc[m,1].mean():+.2f}) q[{qq[0]:.0f},{qq[1]:.0f},{qq[2]:.0f},{qq[3]:.0f}] | "
              f"M_hat {r(Mh)*1e3:4.0f} mN.m  F_hat {r(Fh):4.2f} N  tilt {np.degrees(r(tilt)):4.2f} deg  wander {np.sqrt((e**2).sum(1).mean())*1e3:5.1f} mm")
