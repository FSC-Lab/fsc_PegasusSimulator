#!/usr/bin/env python3
"""The claw (FK: joint_states + mocap pose on the planner's model, R0_model = R0_actual Rz(-90)) against
the box (obj_0) in the push frame, through a window: did the claw touch the box?
Usage: claw_box.py <tag> <t0> <t1> [dt]"""
import sys, os
import numpy as np
from bp_common import Bag, tilt_deg
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), *[".."] * 5))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP   # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C          # noqa: E402

tag, a, c = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]); dt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25
b = Bag(tag)
def R_of(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
model = TP.make_params_t650(); Rz90 = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])
tm = b.t("mocap"); P = np.column_stack([b.f("mocap", f"pose.position.{k}") for k in "xyz"])
Q = np.column_stack([b.f("mocap", f"pose.orientation.{k}") for k in "wxyz"])
tj = b.t("js"); J = b.f("js", "position")
to = b.t("obj"); O = np.column_stack([b.f("obj", f"pose.position.{k}") for k in "xyz"])
OQ = np.column_stack([b.f("obj", f"pose.orientation.{k}") for k in "wxyz"])
yaw = np.radians(b.f("pl_info", "data")[-1, 81]); u = np.array([np.cos(yaw), np.sin(yaw), 0]); n = np.array([-u[1], u[0], 0])
print(f"{tag}: claw - box centre in the push frame (along = the push, across = left of it, up), mm; the fin is ~"
      f"[0, 0, +183] from the box centre")
for tt in np.arange(a, c, dt):
    i = np.searchsorted(tm, tt); j = np.searchsorted(tj, tt); o = min(np.searchsorted(to, tt), len(to) - 1)
    R = R_of(Q[i]); q = np.array([-J[j, 2], J[j, 0], J[j, 1], -J[j, 3]])
    X = np.zeros(12 + 3 * 4 + 6); X[3:12] = (R @ Rz90).flatten(order="F"); X[12:16] = q
    claw = P[i] + (R @ Rz90) @ C.dynamics(X, model)["r_0e_0"]
    d = claw - O[o]
    print(f"{tt:6.2f} | along {1e3*d@u:+6.0f} across {1e3*d@n:+6.0f} up {1e3*d[2]:+6.0f} | vehicle z {P[i,2]:.3f} | "
          f"box tilt {tilt_deg(*OQ[o]):5.1f} z {O[o,2]:.3f} | grip {J[j,4]:+.3f} m | q {np.round(np.degrees(q), 1)}")
