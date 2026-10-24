#!/usr/bin/env python3
"""The arm's distance from its singular pose, sigma_nd (the planner's measure: sigma_min of J_3y with
the translational rows scaled by 1/Lchar; keep-out 0.10), over q2 x q3 at q1 = q4 = 0, T650 model,
gripper EE. Shows that it depends on q3 ONLY, and where the singular pose and the keep-out sit.
Usage: /usr/bin/python3 sigma_map.py"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "..", "extensions", "fsc_aerial_manipulation",
                                "fsc_aerial_manipulation", "robotic_arm", "utils_planner"))
import transition_planner as TP  # noqa: E402

P = TP.make_params_t650()
D = np.pi / 180


def s(q2, q3):
    return TP._sigma_nd(np.array([0.0, q2 * D, q3 * D, 0.0]), P)


q3s = [50, 40, 30, 20, 15, 10, 5, 0, -5, -10, -20, -30, -45, -60]
print("sigma_nd at q1 = q4 = 0 (planner keep-out 0.10)")
print("q2\\q3 " + " ".join(f"{q:6d}" for q in q3s))
for q2 in (-20, 10, 20, 32, 40, 45, 50):
    print(f"{q2:5d} " + " ".join(f"{s(q2, q):6.3f}" for q in q3s))
g = np.arange(-80, 50.01, 0.25)
v = np.array([s(32.0, q) for q in g])
k = int(np.argmin(v))
print(f"singular pose: sigma_nd {v[k]:.3f} at q3 {g[k]:.1f} deg (any q2)")
cross = g[np.argmin(np.abs(np.where(g > g[k], v, 9) - 0.10))]
print(f"keep-out 0.10 at q3 {cross:.1f} deg; the static guard's 10 deg = sigma_nd {s(32, 10):.3f}")
