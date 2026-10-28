#!/usr/bin/env python3
"""The approach (Go To Start's end -> the gripper's close command): the joints' ranges (q1..q4)
and the true tilt peak. The box push rejected a room-held descent on 2026-10-07 for a 13-25 deg
arm swing (and a bobbing claw, not measured here).
    approach_arm.py tag [tag ...]"""
import os, sys, numpy as np
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")
for tag in sys.argv[1:]:
    d = np.load(os.path.join(R, tag + ".npz"), allow_pickle=True)
    m = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    t0 = m.get("go_to_start:end"); t1 = m.get("gripper_close:start", m.get("gripper_abort:start"))
    if t0 is None or t1 is None: print(f"{tag:14s} --"); continue
    j = d["joints"]; kj = (j[:, 0] >= t0) & (j[:, 0] <= t1); q = np.degrees(j[kj, 1:5])
    bt = d["base_truth"]; kb = (bt[:, 0] >= t0) & (bt[:, 0] <= t1); qq = bt[kb, 4:8]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (qq[:, 1] ** 2 + qq[:, 2] ** 2), -1, 1)))
    rng = " ".join(f"q{i+1} {q[:, i].min():5.1f}..{q[:, i].max():5.1f}" for i in range(4))
    print(f"{tag:14s} {t1 - t0:4.1f} s  {rng} | tilt max {tilt.max():4.1f}")
