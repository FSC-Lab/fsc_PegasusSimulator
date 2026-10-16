#!/usr/bin/env python3
"""The 1.2-1.8 Hz pitch band after the RELEASE, in 1 s steps (does the ring decay?), and over the two quiet
hovers (over the pick before the grasp; at Drop Start before the drop), and over the carry (4 s after the
lift to the release) -- a mode that grows or sustains itself would show there.  PYTHONNOUSERSITE=1 /usr/bin/python3 wobble_decay.py ../runs/*.npz"""
import sys
import numpy as np
from scipy.signal import butter, filtfilt

b, a = butter(2, [1.2 / 50, 1.8 / 50], btype="band")
print(f"{'run':4s} | ring after the open, 1.2-1.8 Hz pitch rms per 1 s (deg)          | quiet hovers: pick / drop start | carry")
for path in sys.argv[1:]:
    d = np.load(path, allow_pickle=True)
    m = {str(n): float(t) for n, t in zip(d["marks_name"], d["marks_t"])}
    if "gripper_open:start" not in m:
        print(f"{path.split('/')[-1][:-4]:4s} | (did not reach the release)"); continue
    o = d["odom"]; t = o[:, 0]; w, x, y, z = o[:, 7:11].T
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    tu = np.arange(m["direct"], m.get("execute_land:end", t[-1]), 0.01)
    bp = filtfilt(b, a, np.interp(tu, t, pitch))
    rms = lambda t0, t1: np.sqrt(np.mean(bp[(tu >= t0) & (tu < t1)] ** 2))
    t0 = m["gripper_open:start"]
    ring = " ".join(f"{rms(t0 + k, t0 + k + 1):4.2f}" for k in range(10))
    q1 = rms(m["go_to_start:end"], m["pick:start"]) if "pick:start" in m else np.nan
    q2 = rms(m["go_to_place_start:end"], m["ready_place:start"]) if "ready_place:start" in m else np.nan
    # the carry: from 4 s after the lift (its own transient) to the release
    qc = rms(m["exit_pick:start"] + 4.0, t0) if "exit_pick:start" in m else np.nan
    print(f"{path.split('/')[-1][:-4]:4s} | {ring} | {q1:4.2f} / {q2:4.2f}                   | {qc:4.2f}")
