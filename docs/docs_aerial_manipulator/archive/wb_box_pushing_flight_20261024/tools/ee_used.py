#!/usr/bin/env python3
"""Which EE Offset did each flight use, and where was the fin? While Ready / Push fly, the planner's
push_pull/arrival_error [3..5] = (its grasp target) - (its FK claw), so target - obj_0, turned into the
box frame, IS the EE Offset in use. With the jaws closed on the fin (a push that moved the box), the FK
claw - obj_0 in the box frame is where the fin is (+- the few-mm FK error).
Usage: /usr/bin/python3 ee_used.py [tag ...]"""
import sys
import numpy as np
from bp_common import Bag, yaw_deg

P = ["pose.position.x", "pose.position.y", "pose.position.z"]


def box_frame(b, t, vec_world_fn):
    to = b.t("obj")
    O = np.column_stack([b.f("obj", k) for k in P])
    oy = np.radians(yaw_deg(*[b.f("obj", f"pose.orientation.{k}") for k in "wxyz"]))
    j = min(np.searchsorted(to, t), len(to) - 1)
    c, s = np.cos(oy[j]), np.sin(oy[j])
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return R.T @ (vec_world_fn(t) - O[j])


for tag in (sys.argv[1:] or ["150423", "151159", "162850", "163413", "164419"]):
    b = Bag(tag)
    tc = b.t("pl_current_ee")
    C = np.column_stack([b.f("pl_current_ee", k) for k in P])
    claw = lambda t: np.array([np.interp(t, tc, C[:, i]) for i in range(3)])
    ta, A = b.t("pl_arr"), b.f("pl_arr", "data")
    ready = [i for i in range(len(ta)) if A[i, 0] == 1 and A[i, 1] == 1]        # step 1 = ready
    if ready:
        E = []
        for i in ready:
            E.append(box_frame(b, ta[i], lambda t, i=i: claw(t) + A[i, 3:6]))
        E = np.array(E) * 1e3
        print(f"{tag}: EE Offset in use (Ready target - obj_0, box frame) = across {np.median(E[:,0]):+6.1f}, "
              f"along {np.median(E[:,1]):+6.1f}, up {np.median(E[:,2]):+6.1f} mm  (n={len(E)})")
    push = [i for i in range(len(ta)) if A[i, 0] == 2]
    to = b.t("obj")
    O = np.column_stack([b.f("obj", k) for k in P])
    moved = np.linalg.norm(O[-1, :2] - O[0, :2])
    if push and moved > 0.03:
        t0p, t1p = ta[push[0]] + 0.5, ta[push[-1]]
        F = np.array([box_frame(b, t, claw) for t in np.arange(t0p, t1p, 0.05)]) * 1e3
        print(f"{tag}: jaws on the fin {t0p:.1f}-{t1p:.1f} s: claw - obj_0 = across {np.median(F[:,0]):+6.1f}, "
              f"along {np.median(F[:,1]):+6.1f}, up {np.median(F[:,2]):+6.1f} mm  -> the fin is "
              f"{np.median(F[:,0]):+.1f} mm across from obj_0")
