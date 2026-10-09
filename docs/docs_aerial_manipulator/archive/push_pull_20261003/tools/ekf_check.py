#!/usr/bin/env python3
"""EKF2 vs ground truth through a push run: the fused position offset and PX4's
attitude error (world-frame small-angle vector), per step.
    /usr/bin/python3 ekf_check.py ../runs/pl_34   (needs pl_34.npz + pl_34_px4.npz)"""
import sys
import numpy as np
from pl_score import STEPS

def R_q(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
M = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])      # NED <-> ENU
F = np.diag([1.0, -1.0, -1.0])                           # FRD <-> FLU

def logR(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1); a = np.arccos(c)
    if a < 1e-9: return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])

base = sys.argv[1]
z = np.load(base + ".npz", allow_pickle=True); p = np.load(base + "_px4.npz", allow_pickle=True)
o, bt, att, odo = z["odom"], z["base_truth"], p["att"], p["odo"]
best = None                                              # align the two recorders' clocks on z
for dt in np.arange(-200, 200, 0.05):
    e = np.mean(np.abs(-np.interp(o[:, 0], odo[:, 0] + dt, odo[:, 3]) - o[:, 3]))
    if best is None or e < best[1]: best = (dt, e)
dt = best[0]
print(f"{base.split('/')[-1]}: clock offset {dt:.2f} s (|dz| {best[1]*1e3:.1f} mm)")
print(f"{'step':12s} {'pos est-truth mean xyz mm':>28s} {'std':>16s} | {'att err mean x/y/z deg':>24s} {'|tilt err| max':>15s} | corr(pos_y, tilt err)")
for st in ["go_to_start", "ready", "close", "push", "release"]:
    k = STEPS.index(st); oo = o[o[:, -1] == k]
    if len(oo) < 20: continue
    t = oo[:, 0]
    tr = np.column_stack([np.interp(t, bt[:, 0], bt[:, 1+i]) for i in range(3)])
    e = (oo[:, 1:4] - tr) * 1e3
    ta = att[:, 0] + dt; m = (ta >= t[0]) & (ta <= t[-1])
    errs = []
    for row in att[m]:
        Rp = M @ R_q(*row[1:5]) @ F
        tq = [np.interp(row[0] + dt, bt[:, 0], bt[:, 4+i]) for i in range(4)]
        tq = np.array(tq) / np.linalg.norm(tq)
        errs.append(logR(Rp @ R_q(*tq).T))
    errs = np.degrees(np.array(errs))
    tilt = np.linalg.norm(errs[:, :2], axis=1)
    ey = np.interp(ta[m], t, e[:, 1]); ex = np.interp(ta[m], t, e[:, 0])
    c1 = np.corrcoef(ey, errs[:, 0])[0, 1]; c2 = np.corrcoef(ey, errs[:, 1])[0, 1]
    print(f"{st:12s} [{e[:,0].mean():+6.1f} {e[:,1].mean():+6.1f} {e[:,2].mean():+6.1f}]  [{e[:,0].std():4.1f} {e[:,1].std():4.1f} {e[:,2].std():4.1f}] | "
          f"[{errs[:,0].mean():+5.2f} {errs[:,1].mean():+5.2f} {errs[:,2].mean():+5.2f}]  {tilt.max():6.2f} | x-err {c1:+.2f}  y-err {c2:+.2f}")
