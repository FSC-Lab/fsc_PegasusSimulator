#!/usr/bin/env python3
"""From Ready's start to the abort: the planner's own readings (push_pull/arrival_error: [2] |claw error|, [3..5] error,
[9] along the box x (the settle's 'across'), [10] |F| low-passed, [34] depth under the fin top, [35] fin across the claw,
[36] gap ahead of the bracket, [37]/[38] clearances), the law's CoM error, the valve's yaw (obj_0), the gripper."""
import sys, os, re
import numpy as np
from vt_common import Bag, yaw_deg
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from claw_motion import phases
for tag in sys.argv[1:]:
    b = Bag(tag); ph = phases(b)
    a0 = (ph["ready"] or ph["gts_done"]) - 0.5; a1 = (ph["abort"] or a0 + 8) + 0.6
    ta = b.t("pl_arr"); A = b.f("pl_arr", "data")
    to = b.t("obj"); yo = np.degrees(np.unwrap(np.radians(yaw_deg(*[b.f("obj", f"pose.orientation.{c}") for c in "wxyz"]))))
    tw = b.t("wb"); W = b.f("wb", "data")
    tj = b.t("js"); jn = list(b.f("js", "name")[0]); J = np.stack(b.f("js", "position")).astype(float)
    gi = jn.index("gripper_left_joint") if "gripper_left_joint" in jn else None
    print(f"===== {tag}  ready {ph['ready']}, ready_done {ph['ready_done']}, abort {ph['abort']}")
    print("    t     |err|  ex    ey    ez  axis |  depth across  gap   box   ball | CoM err x  y   | valve yaw | grip  |F|")
    for t, r in zip(ta, A):
        if t < a0 or t > a1: continue
        r = np.asarray(r, float)
        g = lambda k: r[k] * 1e3 if len(r) > k and np.isfinite(r[k]) else np.nan
        iw = np.searchsorted(tw, t); w = np.asarray(W[min(iw, len(W) - 1)], float)
        ce = (w[48:51] - w[45:48]) * 1e3 if len(w) > 50 else [np.nan] * 3
        io = min(np.searchsorted(to, t), len(to) - 1)
        ij = min(np.searchsorted(tj, t), len(tj) - 1)
        gr = J[ij, gi] * 1e3 if gi is not None else np.nan
        print(f"  {t:6.2f} {g(2):5.0f} {g(3):5.0f} {g(4):5.0f} {g(5):5.0f} {g(9):5.0f} | {g(34):5.0f} {g(35):6.1f} {g(36):5.0f} {g(37):5.0f} {g(38):5.0f} |"
              f" {ce[0]:+5.0f} {ce[1]:+5.0f} | {yo[io]:8.2f} | {gr:5.1f} {r[10] if len(r)>10 else np.nan:4.2f}")
