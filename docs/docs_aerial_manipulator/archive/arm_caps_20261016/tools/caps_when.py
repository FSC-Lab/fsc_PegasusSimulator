#!/usr/bin/env python3
"""WHEN each joint came near its caps on the 10-15 flights: per mission leg (the planner's leg marks), the
peak DIRECT torque term (% of max_effort), the peak total duty (% of the PWM Limit), the peak SAFETY-hold duty.
PYTHONNOUSERSITE=1 /usr/bin/python3 caps_when.py [run ...]   (default e1 d1 d2)"""
import os
import sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "wb_pick_drop_flight_20261015", "tools")))
from pd15_common import load, leg_windows  # noqa: E402
from caps_audit import MAXEFF, PWMLIM, KPWM  # noqa: E402

for run in (sys.argv[1:] or ["e1", "d1", "d2"]):
    d, t0 = load(run)
    tl = d["law__recv"] - t0; L = d["law__data"]; pt = L[:, 0] > 0.5
    T = L[:, 9:13] * KPWM + L[:, 18:22] + L[:, 22:26]; duty = L[:, 13:17]
    win = dict(leg_windows(d, t0))
    tdir = tl[pt]
    win = {"SAFETY before DIRECT": (tl[0], tdir[0]), **win, "SAFETY after DIRECT": (tdir[-1], tl[-1])}
    print(f"\n=== {run}: per leg  [DIRECT term % of max_effort | total duty % of PWM Limit]  j1..j4")
    for lab, (a, b) in win.items():
        m = (tl >= a) & (tl < b)
        if m.sum() < 20:
            continue
        mt = m & pt
        tt = (np.abs(T[mt]).max(axis=0) / MAXEFF * 100) if mt.any() else np.full(4, np.nan)
        dd = np.abs(duty[m]).max(axis=0) / PWMLIM * 100
        f = lambda v: " ".join(f"{x:4.0f}" for x in v)
        flag = "  <- term over a cap" if np.nanmax(tt) > 100 else ("  <- duty at a PWM Limit" if dd.max() >= 99.5 else "")
        print(f"  {lab[:32]:32s} {b-a:6.1f} s  term [{f(tt)}]  duty [{f(dd)}]{flag}")
