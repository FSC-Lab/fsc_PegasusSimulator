#!/usr/bin/env python3
"""Dense table of the 10-23 grasp and q3-guard trip: joints (MODEL, deg), gripper, the planner's
q3 rate guard replayed, base pose (mocap), box pose, the law's CoM / EE errors and force reading.
Usage: /usr/bin/python3 trip_window.py [t0 t1 dt]"""
import sys
import numpy as np
from bp_common import Bag, tilt_deg, yaw_deg

b = Bag("172853")
t0, t1, dt = (float(x) for x in (sys.argv[1:4] if len(sys.argv) > 3 else (44.5, 50.5, 0.04)))
wt, W = b.t("wb"), b.f("wb", "data")
tj = b.t("js")
th = b.f("js", "hdr")
P = np.degrees(b.f("js", "position"))
q = {1: -P[:, 2] * 0 + P[:, 2], 2: P[:, 0], 3: P[:, 1], 4: P[:, 3]}
# MODEL convention from the law's own q (wb[5..8]) has q1, q4 sign-flipped vs joint_states
q[1], q[4] = -P[:, 2], -P[:, 3]
grip = P[:, 4]


def rate_guard(ts, q3, tau=0.2, lim=10.0, vmin=5.0, persist=0.05, tc=0.02):
    """The planner's plQ3RateGuard on every sample (header stamps), never disarmed: returns
    filtered rate v [deg/s], prediction p [deg], and the 'bad since' persistence flag."""
    v = np.zeros_like(q3)
    since, trip = None, np.zeros(len(q3), bool)
    for i in range(1, len(q3)):
        d = ts[i] - ts[i - 1]
        if 0 < d < 0.1:
            v[i] = v[i - 1] + (1 - np.exp(-d / tc)) * ((q3[i] - q3[i - 1]) / d - v[i - 1])
        elif d <= 0:
            v[i] = v[i - 1]
        p = q3[i] + tau * v[i]
        if p < lim and v[i] < -vmin:
            since = ts[i] if since is None else since
            trip[i] = ts[i] - since >= persist - 1e-9
        else:
            since = None
    return v, q3 + tau * v, trip


if __name__ == "__main__":
    v3, p3, tr = rate_guard(th, q[3])
    first = np.flatnonzero(tr & (tj > 44.9))
    print(f"replayed rate guard first trip (engaged from 45.00): t={tj[first[0]]:.3f} q3={q[3][first[0]]:.1f} "
          f"v={v3[first[0]]:.1f} p={p3[first[0]]:.1f}" if len(first) else "no trip")
    tm = b.t("mocap")
    bp = np.column_stack([b.f("mocap", f"pose.position.{c}") for c in "xyz"])
    bq = [b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"]
    byaw, btilt = yaw_deg(*bq), tilt_deg(*bq)
    to = b.t("obj")
    op = np.column_stack([b.f("obj", f"pose.position.{c}") for c in "xyz"])
    oyaw = yaw_deg(*[b.f("obj", f"pose.orientation.{c}") for c in "wxyz"])
    i0 = np.searchsorted(tm, t0)
    ib0 = np.searchsorted(to, t0)
    print(f"{'t':>6s} {'q1':>6s} {'q2':>6s} {'q3':>6s} {'q4':>6s} {'grip':>6s} {'v3':>6s} {'p3':>6s} | "
          f"{'dbx':>6s} {'dby':>6s} {'dbz':>6s} {'yaw':>6s} {'tilt':>5s} | {'box dx':>6s} {'dy':>6s} {'yaw':>6s} | "
          f"{'eCoM x':>6s} {'y':>6s} {'z':>6s} | {'ey x':>6s} {'y':>6s} {'z':>6s} {'psi':>6s} | {'Fx':>5s} {'Fy':>5s} {'Fz':>5s} | tau1..4")
    for tk in np.arange(t0, t1, dt):
        j = np.searchsorted(tj, tk)
        m = np.searchsorted(tm, tk)
        o = np.searchsorted(to, tk)
        w = np.searchsorted(wt, tk)
        ec = (W[w, 48:51] - W[w, 45:48]) * 1e3
        ey = W[w, 24:28] * np.array([1e3, 1e3, 1e3, 57.2958])
        F = W[w, 97:100]
        tau = W[w, 13:17]
        print(f"{tk:6.2f} {q[1][j]:6.1f} {q[2][j]:6.1f} {q[3][j]:6.1f} {q[4][j]:6.1f} {grip[j]:6.3f} {v3[j]:6.0f} {p3[j]:6.1f} | "
              f"{1e3*(bp[m,0]-bp[i0,0]):6.0f} {1e3*(bp[m,1]-bp[i0,1]):6.0f} {1e3*(bp[m,2]-bp[i0,2]):6.0f} {byaw[m]:6.1f} {btilt[m]:5.1f} | "
              f"{1e3*(op[o,0]-op[ib0,0]):6.1f} {1e3*(op[o,1]-op[ib0,1]):6.1f} {oyaw[o]:6.1f} | "
              f"{ec[0]:6.0f} {ec[1]:6.0f} {ec[2]:6.0f} | {ey[0]:6.1f} {ey[1]:6.1f} {ey[2]:6.1f} {ey[3]:6.1f} | "
              f"{F[0]:5.1f} {F[1]:5.1f} {F[2]:5.1f} | {' '.join(f'{x:5.2f}' for x in tau)}")
