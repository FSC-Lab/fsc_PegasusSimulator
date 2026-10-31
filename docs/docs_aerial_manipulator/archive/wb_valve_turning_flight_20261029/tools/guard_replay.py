#!/usr/bin/env python3
"""Replay the planner's handle-tilt guard over the recorded obj_0 stream (the sample gate 20 mm / 10 deg
from the last good sample, reference = obj_0 at the push step's start, trip = above the limit for the
persistence without a break), for a grid of limits and persistences, raw and low-passed.  Only the
contact phase that was actually flown (push step -> the real abort) can be replayed."""
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import obj, step_times

def gate(t, R, p, jm=0.020, jd=10.0):
    keep = np.zeros(len(t), bool); lp_, lR = None, None
    for i in range(len(t)):
        if lp_ is None: keep[i] = True; lp_, lR = p[i], R[i]; continue
        d = np.linalg.norm(p[i] - lp_); a = np.degrees(np.arccos(np.clip((np.trace(lR.T @ R[i]) - 1) / 2, -1, 1)))
        if d <= jm and a <= jd: keep[i] = True; lp_, lR = p[i], R[i]
    return keep

def first_trip(t, x, lim, persist):
    since = None
    for ti, xi in zip(t, x):
        if xi > lim:
            since = ti if since is None else since
            if ti - since >= persist: return ti
        else: since = None
    return None

print("first trip, seconds after the TURN start ('-' = none before the real abort);  real abort: 180652 +7.67 s, 181833 +2.12 s")
for tag in BAGS:
    b = Bag(tag); st = step_times(b); t, R, p = obj(b)
    k = gate(t, R, p); t, R = t[k], R[k]
    w = (t >= st["push"]) & (t <= st["abort"]); i0 = np.searchsorted(t, st["push"])
    tl = np.degrees(np.arccos(np.clip(R[:, :, 2] @ R[i0][:, 2], -1, 1)))
    a = np.zeros_like(tl); a[0] = tl[0]
    for i in range(1, len(tl)):   # 1st-order low-pass, 2 rad/s
        a[i] = a[i-1] + (1 - np.exp(-2.0 * (t[i] - t[i-1]))) * (tl[i] - a[i-1])
    t0 = st["push"] + 1.5
    print(f"===== {tag}: contact phase {st['abort']-st['push']:.1f} s; raw tilt max {tl[w].max():.2f}, low-passed max {a[w].max():.2f} deg")
    for lab, x in (("raw", tl), ("LP 2 rad/s", a)):
        for lim in (2, 3, 4, 5, 6, 8):
            cells = []
            for per in (0.2, 0.5, 1.0):
                tt = first_trip(t[w], x[w], lim, per)
                cells.append(f"{tt - t0:+6.2f}" if tt is not None else "     -")
            print(f"  {lab:10s} > {lim} deg   persist 0.2/0.5/1.0 s: {'  '.join(cells)}")
