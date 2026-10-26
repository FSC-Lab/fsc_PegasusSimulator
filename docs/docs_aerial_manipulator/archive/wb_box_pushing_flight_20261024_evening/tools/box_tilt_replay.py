#!/usr/bin/env python3
"""Replay the planner's box-tilt guard on the full-rate box mocap: the tilt of the box z axis against
its pose when the claw first sat on the handle, the 0.2 s persistence (reset by any sample under the
limit), and two variants: the tilt low-passed (tau) instead of raw, and a lower limit.
Usage: box_tilt_replay.py <tag> <t_ref> <t0> <t1>"""
import sys
import numpy as np
from bp_common import Bag

tag, tr, a, c = sys.argv[1], *(float(x) for x in sys.argv[2:5])
b = Bag(tag)
def R_of(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
t = b.t("obj"); Q = np.column_stack([b.f("obj", f"pose.orientation.{k}") for k in "wxyz"])
i0 = np.searchsorted(t, tr); z0 = R_of(Q[i0])[:, 2]
m = (t >= a) & (t <= c); tt = t[m]
til = np.array([np.degrees(np.arccos(np.clip(z0 @ R_of(q)[:, 2], -1, 1))) for q in Q[m]])
def replay(x, lim, persist=0.2):
    since = None
    for ti, xi in zip(tt, x):
        if xi > lim:
            since = ti if since is None else since
            if ti - since >= persist:
                return ti
        else:
            since = None
    return None
def lp(x, tau):
    y = np.empty_like(x); y[0] = x[0]
    for k in range(1, len(x)):
        g = 1 - np.exp(-(tt[k] - tt[k - 1]) / tau); y[k] = y[k - 1] + g * (x[k] - y[k - 1])
    return y
print(f"{tag}: box tilt vs t={tr}: max {til.max():.2f} deg at {tt[np.argmax(til)]:.2f} s; time above 2 deg {np.sum(np.diff(tt)[til[:-1] > 2.0]):.2f} s")
for nm, x, lim, per in (("as flown: raw > 2.0 for 0.2 s", til, 2.0, 0.2), ("raw > 2.0 for 0.1 s", til, 2.0, 0.1),
                        ("LP 0.1 s > 2.0 for 0.2 s", lp(til, 0.1), 2.0, 0.2), ("LP 0.1 s > 1.5 for 0.2 s", lp(til, 0.1), 1.5, 0.2),
                        ("LP 0.2 s > 1.5 for 0.1 s", lp(til, 0.2), 1.5, 0.1)):
    r = replay(x, lim, per)
    print(f"   {nm:28s}: {'no trip' if r is None else f'TRIP at {r:.2f} s'}")
