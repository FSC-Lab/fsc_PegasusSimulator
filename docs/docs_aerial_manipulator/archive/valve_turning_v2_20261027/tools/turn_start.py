#!/usr/bin/env python3
"""The turn's arm margin, per run: from the push step's start (the 1.5 s settle) to 4 s later -- the
lowest q3 and the largest BACKWARD slip of the valve (against the turn); and over the whole push
step until 2 s after it (or the abort) -- the lowest q3 and when, as a share of the step.
    turn_start.py <runs dir> tag [tag ...]"""
import os, sys, numpy as np
def yaw(q): return np.degrees(np.unwrap(np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]), 1-2*(q[:,2]**2+q[:,3]**2))))
R = sys.argv[1]
for tag in sys.argv[2:]:
    d = np.load(os.path.join(R, tag + ".npz"), allow_pickle=True)
    m = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    if "push:start" not in m: print(f"{tag:16s} no turn"); continue
    t0 = m["push:start"]; j, bx = d["joints"], d["box"]
    k = (j[:, 0] >= t0) & (j[:, 0] <= t0 + 4.0); b = (bx[:, 0] >= t0) & (bx[:, 0] <= t0 + 4.0)
    yb = yaw(bx[:, 4:8]); y0 = yb[np.argmin(abs(bx[:, 0] - t0))]
    out = "ABORT" if bool(d["aborted"]) else "ok"
    t1 = m.get("push:end", t0 + 1e9) + 2.0
    w = (j[:, 0] >= t0) & (j[:, 0] <= t1); qa = np.degrees(j[w, 3]); ia = int(np.argmin(qa))
    T = m.get("push:end", j[w, 0][-1]) - t0
    print(f"{tag:16s} {out:5s}  start: q3 min {np.degrees(j[k, 3]).min():6.1f}  valve back {min(0.0, (yb[b] - y0).min()):6.1f} deg"
          f"  | whole: q3 min {qa[ia]:6.1f} at {100 * (j[w, 0][ia] - t0) / T:4.0f} % of the step")
