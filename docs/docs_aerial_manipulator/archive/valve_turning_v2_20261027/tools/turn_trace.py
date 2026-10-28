#!/usr/bin/env python3
"""The push step of a run, every 0.5 s from 1 s before it: joints q1..q4 [deg], the true tilt, and
the valve's rotation since the start (Isaac truth). Ends at the abort / the step's end + 2 s.
    turn_trace.py tag"""
import os, sys, numpy as np
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")
def yaw(q): return np.degrees(np.unwrap(np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]), 1-2*(q[:,2]**2+q[:,3]**2))))
for tag in sys.argv[1:]:
    d = np.load(os.path.join(R, tag + ".npz"), allow_pickle=True)
    m = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    j, bt, bx = d["joints"], d["base_truth"], d["box"]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (bt[:, 5] ** 2 + bt[:, 6] ** 2), -1, 1)))
    yb = yaw(bx[:, 4:8]); t0 = m["push:start"]
    t1 = min(m.get("push:end", t0 + 14) + 2, j[-1, 0])
    if bool(d["aborted"]): print(f"{tag}: {str(d['reason'])[:150]}")
    for tt in np.arange(t0 - 1, t1, 0.5):
        k = np.argmin(abs(j[:, 0] - tt)); i = np.argmin(abs(bt[:, 0] - tt)); b = np.argmin(abs(bx[:, 0] - tt))
        print(f"  t+{tt - t0:5.1f}  q {np.round(np.degrees(j[k, 1:5]), 1)}  tilt {tilt[i]:4.1f}  valve {yb[b] - yb[0]:6.1f}")
