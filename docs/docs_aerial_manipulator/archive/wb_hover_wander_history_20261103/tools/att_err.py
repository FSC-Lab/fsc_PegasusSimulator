#!/usr/bin/env python3
"""Per still hold (holds.csv): PX4's attitude estimate (vehicle_attitude -- the law's attitude) minus the mocap rigid
body's tilt (the 10-27 att_est_error.py conversion), detrended in the hold -- the tilt the law cannot see, which turns
36.7 N of thrust sideways -- and the TRUE tilt's variation (mocap). Appends att_err_x/y, tilt_x/y [deg] to holds.csv."""
import os, csv
import numpy as np
import history as H
HERE = os.path.dirname(os.path.abspath(__file__))
P = np.diag([1.0, -1.0, -1.0]); NE = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
def e3_of(q):
    w, x, y, z = q.T
    return np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], 1)
def detr(t, x):
    return x - np.polyval(np.polyfit(t, x, 1), t)
rows = list(csv.DictReader(open(os.path.join(HERE, "..", "holds.csv"))))
cur = None
for r in rows:
    ts = r["ts"]
    if cur != ts:
        p = H.find_npz(ts); H._c.NPZ = os.path.dirname(p); b = H._c.Bag(ts); cur = ts
        tv = b.t("vatt"); qv = np.stack(b.f("vatt", "q")).astype(float)
        tm = b.t("vrpn"); qm = np.stack([b.f("vrpn", f"pose.orientation.{k}") for k in "wxyz"], 1)
        # PX4 body z in ENU: R_enu = NE R_ned P -> third column = NE @ (R_ned @ P[:,2]) = NE @ (-R_ned[:,2])
        e3p_ned = e3_of(qv)                       # R_ned[:,2]
        e3p = -(e3p_ned @ NE.T)                   # ENU body-z of the FLU body
        e3m = e3_of(qm)
    a0 = float(r["t0"]); a1 = a0 + float(r["dur"])
    T = np.arange(a0, a1, 0.01)
    if len(T) < 100: continue
    ep = np.stack([np.interp(T, tv, e3p[:, k]) for k in range(2)], 1)
    em = np.stack([np.interp(T, tm, e3m[:, k]) for k in range(2)], 1)
    d = ep - em
    r.update(att_err_x=round(np.degrees(detr(T, d[:, 0]).std()), 3), att_err_y=round(np.degrees(detr(T, d[:, 1]).std()), 3),
             att_off=round(np.degrees(np.linalg.norm(d.mean(0))), 2),
             tilt_x=round(np.degrees(detr(T, em[:, 0]).std()), 3), tilt_y=round(np.degrees(detr(T, em[:, 1]).std()), 3))
keys = list(rows[0].keys()) + [k for k in ("att_err_x", "att_err_y", "att_off", "tilt_x", "tilt_y") if k not in rows[0]]
with open(os.path.join(HERE, "..", "holds.csv"), "w", newline="") as f:
    wr = csv.DictWriter(f, fieldnames=keys); wr.writeheader(); wr.writerows(rows)
print("done", len(rows))
