#!/usr/bin/env python3
"""Score an injected push-and-pull abort (pl_mission.py --inject abort@S): from the injection to 3 s
after the abort's climb leg -- how high the box was lifted, how far it moved, its peak tilt, when the
jaws were open, the claw against the box (did the open jaws drag the fin?), the vehicle's tilt and q3.
Usage: abort_score.py <run.npz> ..."""
import os
import sys

import numpy as np


def tilt_deg(q):
    w, x, y, z = q
    return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


for p in sys.argv[1:]:
    z = np.load(p, allow_pickle=True)
    mk = dict(zip([str(s) for s in z["marks_name"]], z["marks_t"]))
    ti = mk.get("inject_abort:start")
    if ti is None:
        print(f"{os.path.basename(p)}: no injected abort"); continue
    ev = [str(e) for e in z["events"]]
    t_done = next((float(e[1:e.index("s]")]) for e in ev if "push_pull: ABORTED" in e), ti + 10.0)
    B, C, G, J = z["box"], z["claw"], z["grip"], z["joints"]
    BT = z["base_truth"]
    jn = list(z["joint_names"])
    w = (B[:, 0] >= ti - 0.5) & (B[:, 0] <= t_done + 3.0)
    b0 = B[np.searchsorted(B[:, 0], ti), 1:4]
    dz = 1e3 * (B[w, 3] - b0[2]); dxy = 1e3 * np.linalg.norm(B[w, 1:3] - b0[:2], axis=1)
    btilt = np.array([tilt_deg(q) for q in B[w, 4:8]])
    g = G[(G[:, 0] >= ti) & (G[:, 0] <= t_done)]
    t_open = next((t - ti for t, pos in zip(g[:, 0], g[:, 1]) if pos > -np.radians(3.0)), float("nan"))   # Isaac: joint radians, open = 0
    wc = (C[:, 0] >= ti) & (C[:, 0] <= t_done)
    cb = []
    for t, x, y, zz in C[wc, :4]:
        k = np.searchsorted(B[:, 0], t); cb.append([x - B[k, 1], y - B[k, 2], zz - B[k, 3]])
    cb = 1e3 * np.array(cb)
    wj = (J[:, 0] >= ti) & (J[:, 0] <= t_done)
    q3 = np.degrees(J[wj, 1 + jn.index("joint3")])
    wb = (BT[:, 0] >= ti) & (BT[:, 0] <= t_done)
    vt = np.array([tilt_deg(q) for q in BT[wb, 4:8]])
    print(f"{os.path.basename(p):22s} abort {t_done - ti:4.1f} s | jaws open {t_open:4.2f} s after | box: lift max "
          f"{dz.max():+5.1f} mm, moved {dxy.max():5.1f} mm (end {dxy[-1]:5.1f}), tilt max {btilt.max():4.1f} deg | "
          f"claw-box: z {cb[0, 2]:+5.0f} -> {cb[-1, 2]:+5.0f} mm, horizontal max {np.linalg.norm(cb[:, :2], axis=1).max():4.0f} mm | "
          f"vehicle tilt max {vt.max():4.1f} deg | q3 min {q3.min():4.1f}")
