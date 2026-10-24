#!/usr/bin/env python3
"""Usable stretch room (claw travel from the push pose, relative to the airframe) before each q3 rule
fires, as a function of how fast the claw is being pulled out. Two stretch directions:
  horizontal  the claw moves straight out at constant height (the fold q2+q3 shrinks 2.6 deg/cm)
  fold-held   q2 up / q3 down with q2+q3 = 60 deg (the claw moves out AND up) -- the 10-23 stretch
Rules (onset; the rate rules also need q3 falling > 5 deg/s, and persist 0.05 s, the static 0.2 s):
  static      q3 < 10
  rate        q3 + tau*q3dot < 10                                 (flown 10-23, tau = 0.2 s)
  q2-aware    q3 + tau*q3dot + max(0, q2 + tau*q2dot - 45) < 10   (proposed)
Each path ends where q2 meets its 45 deg wall (the arm stops stretching there).
Usage: /usr/bin/python3 room_vs_speed.py"""
import io
import contextlib
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import joint_window as J

TAU, LIM, WALL = 0.2, 10.0, 45.0
POSE = (32.0, 28.0)


def path_horizontal(ds=0.0002):
    p0, q, pts = J.claw(*POSE), POSE, [(0.0, *POSE)]
    s = 0.0
    while q[0] < WALL and s < 0.06:
        s += ds
        q = J.ik(p0 + np.array([0, s, 0]), *q)
        pts.append((s, *q))
    return np.array(pts)


def path_fold_held(du=0.02):
    p0, pts, s, prev = J.claw(*POSE), [], 0.0, J.claw(*POSE)
    for u in np.arange(0.0, WALL - POSE[0] + 1e-9, du):
        c = J.claw(POSE[0] + u, POSE[1] - u)
        s += np.linalg.norm(c - prev)
        prev = c
        pts.append((s, POSE[0] + u, POSE[1] - u))
    return np.array(pts)


def trip(P, v):
    """first travel s [mm] at which each rule's condition holds, the claw moving at v [mm/s]"""
    s, q2, q3 = P[:, 0], P[:, 1], P[:, 2]
    dq2 = np.gradient(q2, s) * v * 1e-3          # deg/s
    dq3 = np.gradient(q3, s) * v * 1e-3
    falling = dq3 < -5.0
    rules = {"static": q3 < LIM,
             "rate": falling & (q3 + TAU * dq3 < LIM),
             "q2-aware": falling & (q3 + TAU * dq3 + np.maximum(0.0, q2 + TAU * dq2 - WALL) < LIM)}
    out = {}
    for k, c in rules.items():
        i = np.flatnonzero(c)
        out[k] = f"{s[i[0]]*1e3:5.1f}" if len(i) else "never"
    return out


for name, P in (("horizontal", path_horizontal()), ("fold-held (10-23)", path_fold_held())):
    end = P[-1]
    print(f"\n{name}: path to the q2 wall = {end[0]*1e3:.1f} mm (q2 {end[1]:.1f}, q3 {end[2]:.1f} there); "
          f"q3 falls {abs(np.polyfit(P[:,0]*1e3, P[:,2], 1)[0]):.2f} deg per mm")
    print(f"  {'claw speed':>12s} | {'static':>7s} | {'rate (flown)':>12s} | {'q2-aware':>9s}   [mm of travel before the rule fires]")
    for v in (5, 20, 50, 88, 150):
        r = trip(P, v)
        print(f"  {v:7d} mm/s | {r['static']:>7s} | {r['rate']:>12s} | {r['q2-aware']:>9s}")
