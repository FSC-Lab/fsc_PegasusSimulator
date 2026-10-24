#!/usr/bin/env python3
"""Where is the fin really, relative to the grasp point the planner computes from obj_0? While the
jaws hold the fin, the claw IS on the fin, so (planner FK claw) - (obj_0 origin + push_pull_ee_offset
in the box frame) = the fin's offset from the assumed grasp point plus the FK model error. Box frame:
x across the fin (the jaws' closing axis), y along the push, z up.
Usage: /usr/bin/python3 fin_offset.py"""
import numpy as np
from bp_common import Bag, yaw_deg

EE = np.array([0.0, 0.0, 0.183])   # push_pull_ee_offset of the hardware box yaml
GRIPPED = {"170123": (40.0, 45.6, "10-21"), "172853": (48.40, 48.80, "10-23"), "150423": (49.7, 51.0, "10-24 #1")}
for tag, (a, c, day) in GRIPPED.items():
    b = Bag(tag)
    tc = b.t("pl_current_ee")
    C = np.column_stack([b.f("pl_current_ee", f"pose.position.{k}") for k in "xyz"])
    to = b.t("obj")
    O = np.column_stack([b.f("obj", f"pose.position.{k}") for k in "xyz"])
    oyaw = np.radians(yaw_deg(*[b.f("obj", f"pose.orientation.{k}") for k in "wxyz"]))
    D = []
    for t in np.arange(a, c, 0.05):
        i = min(np.searchsorted(tc, t), len(tc) - 1)
        j = min(np.searchsorted(to, t), len(to) - 1)
        cs, sn = np.cos(oyaw[j]), np.sin(oyaw[j])
        R = np.array([[cs, -sn, 0], [sn, cs, 0], [0, 0, 1]])
        D.append(R.T @ (C[i] - (O[j] + R @ EE)) * 1e3)
    D = np.array(D)
    rest_yaw = np.degrees(np.median(oyaw[to < a - 10]))
    print(f"{day:9s} gripped {a:.1f}-{c:.1f} s: claw - assumed grasp point = across {np.median(D[:,0]):+6.1f}, "
          f"along {np.median(D[:,1]):+6.1f}, up {np.median(D[:,2]):+6.1f} mm (median); box yaw at rest {rest_yaw:+6.1f} deg")
