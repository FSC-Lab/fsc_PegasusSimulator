#!/usr/bin/env python3
"""Why joint 1 stuck 5-6 mm short of the across gate in the 10-23 open-jaw hold, and what the claw
was doing when the gripper closed.

Joint 1's torque budget from wb_control_debug (layout of the 10-21 tools): u3 [13..16] (the law's
arm torques), the arm's slow internal feed-forward u3_est [89..92] (C_x of the deadbeat residual,
omega_x 0.1 rad/s in the box yaml) and the raw joint-row residual d_c [68..71]; e_y [24..27]; the
measured joint from joint_states (order [q2, q3, q1, q4]). u3 - u3_est = what the task spring and
damper ask of the joint; u3_est = what the observer has learned and subtracts.
Usage: /usr/bin/python3 grasp_q1.py [t0 t1 dt]"""
import sys
import numpy as np
from bp_common import Bag

t0, t1, dt = (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 \
    else (43.6, 48.6, 0.2)
b = Bag("172853")
tw = b.t("wb")
W = np.vstack(b.f("wb", "data")).astype(float)
tj = b.t("js")
P = np.vstack(b.f("js", "position")).astype(float)
V = np.vstack(b.f("js", "velocity")).astype(float)
E = np.vstack(b.f("js", "effort")).astype(float)
q1, v1, e1 = np.degrees(P[:, 2]), np.degrees(V[:, 2]), E[:, 2]

print(f"{'t':>6s} {'q1':>6s} {'q1dot':>6s} {'ey_x':>6s} {'ey_y':>6s} | {'u3_j1':>7s} {'ff_j1':>7s} "
      f"{'task':>7s} {'d_c j1':>7s} | {'eff j1':>6s}   [deg, deg/s, mm, mm | N.m | counts]")
for t in np.arange(t0, t1 + 1e-9, dt):
    k = min(np.searchsorted(tw, t), len(tw) - 1)
    j = min(np.searchsorted(tj, t), len(tj) - 1)
    u3, ff, dc = W[k, 13], W[k, 89], W[k, 68]
    print(f"{t:6.1f} {q1[j]:6.1f} {v1[j]:6.1f} {W[k,24]*1e3:6.1f} {W[k,25]*1e3:6.1f} | "
          f"{u3:7.3f} {ff:7.3f} {u3-ff:7.3f} {dc:7.3f} | {e1[j]:6.0f}")
