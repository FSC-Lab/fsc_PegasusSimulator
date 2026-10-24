#!/usr/bin/env python3
"""The standing yaw torque of every box push: rotor commands of the two diagonal pairs
(PX4 Quad-X: 1 front-right + 2 rear-left spin CCW, 3 front-left + 4 rear-right spin CW) in the
hover before Go To Start and in the open-jaw hold / push, and how often a rotor sits at its
limit (<= 0.02 or >= 0.98). A CW-minus-CCW difference near the mean means most of the yaw
budget is spent holding the airframe straight (the loose front-left arm, 10-14 analysis).
Usage: /usr/bin/python3 yaw_balance.py"""
import numpy as np
from bp_common import Bag


def edges(b):
    t, s = b.t("pl_pstatus"), [str(x) for x in b.f("pl_pstatus", "data")]
    ev = {}
    for ti, si in zip(t, s):
        for key, name in (("FLYING go_to_start", "gts"), ("DONE ready", "hold"), ("FLYING push", "push"),
                          ("ABORTING", "abort"), ("DONE push", "pushed"), ("FLYING exit", "exit")):
            if si.startswith(key) and name not in ev:
                ev[name] = ti
    return ev


print(f"{'flight':16s} {'window':10s} {'CCW 1+2':>8s} {'CW 3+4':>8s} {'CW-CCW':>7s} {'at a limit %':>12s}")
for tag, day in (("170123", "10-21"), ("172853", "10-23"), ("150423", "10-24 #1"), ("151159", "10-24 #2"),
                 ("151634", "10-24 #3")):
    b = Bag(tag)
    ev = edges(b)
    tm = b.t("motors")
    M = np.vstack(b.f("motors", "control")).astype(float)[:, :4]
    end = ev.get("abort", ev.get("pushed", ev.get("exit", np.nan)))
    for name, a, c in (("hover", ev["gts"] - 8.0, ev["gts"]), ("hold+push", ev.get("hold", np.nan), end)):
        m = (tm >= a) & (tm < c)
        if not m.any():
            continue
        R = M[m]
        ccw, cw = R[:, [0, 1]].mean(), R[:, [2, 3]].mean()
        lim = 100 * np.mean((R.min(axis=1) <= 0.02) | (R.max(axis=1) >= 0.98))
        print(f"{day:16s} {name:10s} {ccw:8.3f} {cw:8.3f} {cw - ccw:+7.3f} {lim:12.1f}")
