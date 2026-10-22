#!/usr/bin/env python3
"""The 10-21 q3-guard abort (bag _170123) second by second: what the arm, the claw, the box and
the gripper did while the abort held the MEASURED (buckled) pose. Pure numpy.

    /usr/bin/python3 abort_window.py [t0 t1 dt]     (default 45.8 60.0 0.25)
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools"))
from bp_common import Bag, tilt_deg  # noqa: E402


def at(t, a, ti):
    i = int(np.clip(np.searchsorted(t, ti), 1, len(t) - 1))
    return a[i - 1] if abs(t[i - 1] - ti) < abs(t[i] - ti) else a[i]


def main():
    t0, t1, dt = (float(x) for x in (sys.argv[1:4] if len(sys.argv) >= 4 else (45.8, 60.0, 0.25)))
    b = Bag("170123")
    tj = b.t("js")
    js = np.vstack(b.f("js", "position")).astype(float)
    q = np.degrees(np.column_stack([js[:, 2], js[:, 0], js[:, 1], js[:, 3]]))   # hw q1..q4
    grip = js[:, 4] * 1e3                                                      # left finger [mm]
    tr = b.t("wbref")
    qd = np.degrees(np.vstack(b.f("wbref", "q_d")).astype(float))               # MODEL q1..q4
    xcd = np.column_stack([b.f("wbref", f"x_cd.{c}") for c in "xyz"])
    red = np.column_stack([b.f("wbref", f"r_ed.{c}") for c in "xyz"])
    tw = b.t("wb")
    W = np.vstack(b.f("wb", "data")).astype(float)
    tm = b.t("mocap")
    P = np.column_stack([b.f("mocap", f"pose.position.{c}") for c in "xyz"])
    Q = np.column_stack([b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"])
    te = b.t("pl_current_ee")
    E = np.column_stack([b.f("pl_current_ee", f"pose.position.{c}") for c in "xyz"])
    tb = b.t("obj")
    B = np.column_stack([b.f("obj", f"pose.position.{c}") for c in "xyz"])
    print(" t [s] | q1 q2 q3 q4 meas (hw, deg) | q2_d q3_d | finger mm | base z | x_cd z | claw (FK) xyz | "
          "r_ed xyz | |e_y| mm | tilt | box y")
    for ti in np.arange(t0, t1 + 1e-9, dt):
        qq = at(tj, q, ti)
        qdd = at(tr, qd, ti)
        w = at(tw, W, ti)
        ey = np.linalg.norm(w[24:27]) * 1e3 if len(w) > 27 else np.nan
        p = at(tm, P, ti)
        tl = tilt_deg(*at(tm, Q, ti))
        e = at(te, E, ti)
        print(f"{ti:6.2f} | {qq[0]:6.1f} {qq[1]:5.1f} {qq[2]:5.1f} {qq[3]:6.1f} | {qdd[1]:5.1f} {qdd[2]:5.1f} | "
              f"{at(tj, grip, ti):5.1f} | {p[2]:.3f} | {at(tr, xcd, ti)[2]:.3f} | "
              f"{e[0]:.3f} {e[1]:.3f} {e[2]:.3f} | {at(tr, red, ti)[0]:.3f} {at(tr, red, ti)[1]:.3f} {at(tr, red, ti)[2]:.3f} | "
              f"{ey:5.1f} | {tl:4.1f} | {at(tb, B, ti)[1]:.3f}")


if __name__ == "__main__":
    main()
