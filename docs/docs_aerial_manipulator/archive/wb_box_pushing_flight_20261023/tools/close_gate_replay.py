#!/usr/bin/env python3
"""Close-gate candidates replayed on the open-jaw room holds of both hardware pushes: when would
the gripper have closed, and how fast were the claw and the base moving then?

Gate today (arm GS gripWhenThere): |claw error| (push_pull/arrival_error [2]) <= 20 mm and
|error across the fin| ([9], the jaws' closing axis) <= 3 mm -- the 3 mm sized for the SIM claw
(20 mm grip shim, 8.75 mm clearance a side). The real gripper: fingers +19 mm open, -7.5 mm on
the 5 mm fin (10-23), i.e. ~58 mm open gap, ~25 mm clearance a side. Speeds = displacement over
0.3 s (raw mocap velocity is noise), claw = planner FK (current_ee), base = mocap.
Usage: /usr/bin/python3 close_gate_replay.py"""
import numpy as np
from bp_common import Bag

WIN = 0.3
HOLDS = {"172853": (45.00, 47.79), "170123": (36.50, 39.55)}   # Ready done -> the flown close


def speed(b, key, t):
    tt = b.t(key)
    p = np.column_stack([b.f(key, f"pose.position.{c}") for c in "xy"])
    a = np.array([np.interp(t - WIN, tt, p[:, i]) for i in range(2)])
    c = np.array([np.interp(t, tt, p[:, i]) for i in range(2)])
    return np.hypot(*(c - a)) / WIN * 1e3


GATES = [  # name, |e| max mm, |across| max mm, claw speed max mm/s (None = no speed condition)
    ("today: |e|<=20, across<=3", 20, 3, None),
    ("|e|<=20, across<=8", 20, 8, None),
    ("|e|<=25, across<=8", 25, 8, None),
    ("|e|<=20, across<=8, claw<=25 mm/s", 20, 8, 25),
    ("|e|<=25, across<=8, claw<=25 mm/s", 25, 8, 25),
    ("|e|<=25, across<=10, claw<=30 mm/s", 25, 10, 30),
]
for tag, (h0, close) in HOLDS.items():
    b = Bag(tag)
    if not b.has("pl_arr"):
        print(f"{tag}: no push_pull/arrival_error recorded")
        continue
    ta = b.t("pl_arr")
    A = np.vstack(b.f("pl_arr", "data")).astype(float)
    g = ta[(ta >= h0) & (ta <= close + 2.0)]
    rows = A[(ta >= h0) & (ta <= close + 2.0)]
    vc = np.array([speed(b, "pl_current_ee", t) for t in g])
    vb = np.array([speed(b, "mocap", t) for t in g])
    print(f"\n{tag}: open-jaw room hold from {h0:.2f} s, flown close at {close:.2f} s "
          f"({close - h0:.2f} s of hold), arrival_error at {1/np.median(np.diff(g)):.0f} Hz")
    for name, emax, xmax, vmax in GATES:
        ok = (rows[:, 2] * 1e3 <= emax) & (np.abs(rows[:, 9]) * 1e3 <= xmax)
        if vmax is not None:
            ok &= vc <= vmax
        if not ok.any():
            print(f"  {name:38s} never within the window")
            continue
        i = int(np.argmax(ok))
        print(f"  {name:38s} closes at {g[i]:6.2f} s (hold {g[i]-h0:4.2f} s): |e| {rows[i,2]*1e3:4.1f}, "
              f"across {rows[i,9]*1e3:+5.1f} mm; claw {vc[i]:4.0f}, base {vb[i]:4.0f} mm/s")
