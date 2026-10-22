#!/usr/bin/env python3
"""The 10-21 abort (bag _170123): the vehicle along the push axis (world y; the push is -y) while
the abort held the trip pose -- the base, the CoM reference, the claw (planner FK on measured
base + joints), the claw reference r_ed, and the box. Pure numpy.

    /usr/bin/python3 lurch_window.py [t0 t1 dt]     (default 46.4 52.0 0.1)
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools"))
from bp_common import Bag  # noqa: E402
from abort_window import at  # noqa: E402


def main():
    t0, t1, dt = (float(x) for x in (sys.argv[1:4] if len(sys.argv) >= 4 else (46.4, 52.0, 0.1)))
    b = Bag("170123")
    tm, P = b.t("mocap"), np.column_stack([b.f("mocap", f"pose.position.{c}") for c in "xyz"])
    tr = b.t("wbref")
    xcd = np.column_stack([b.f("wbref", f"x_cd.{c}") for c in "xyz"])
    red = np.column_stack([b.f("wbref", f"r_ed.{c}") for c in "xyz"])
    tw, W = b.t("wb"), np.vstack(b.f("wb", "data")).astype(float)
    te, E = b.t("pl_current_ee"), np.column_stack([b.f("pl_current_ee", f"pose.position.{c}") for c in "xyz"])
    tb, B = b.t("obj"), np.column_stack([b.f("obj", f"pose.position.{c}") for c in "xyz"])
    tj, js = b.t("js"), np.vstack(b.f("js", "position")).astype(float)
    print(" t [s] | base y | x_c y  x_cd y | claw y  r_ed y | claw z | box y | finger mm | dhat_t (wb[?])")
    for ti in np.arange(t0, t1 + 1e-9, dt):
        w = at(tw, W, ti)
        print(f"{ti:6.2f} | {at(tm, P, ti)[1]:+.3f} | {w[49]:+.3f} {at(tr, xcd, ti)[1]:+.3f} | "
              f"{at(te, E, ti)[1]:+.3f} {at(tr, red, ti)[1]:+.3f} | {at(te, E, ti)[2]:.3f} | "
              f"{at(tb, B, ti)[1]:+.4f} | {at(tj, js[:, 4], ti) * 1e3:5.1f}")


if __name__ == "__main__":
    main()
