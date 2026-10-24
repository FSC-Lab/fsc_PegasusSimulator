#!/usr/bin/env python3
"""How fast does the vehicle drift while it just holds? Base speed from mocap (over 0.3 s) in the
planner's HOLD windows of every 10-24 push flight (DIRECT entry + 3 s to the next step, and every
'ABORT complete' / 'DONE <step>' hold longer than 4 s, skipping its first 3 s), plus the fraction of
time a 'slower than V for 0.5 s' condition is met.
Usage: /usr/bin/python3 hold_speed.py"""
import numpy as np
from bp_common import Bag

speeds = []
for tag in ("150423", "151159", "162850", "163413", "164419", "170123", "172853"):
    b = Bag(tag)
    tm = b.t("mocap"); M = np.column_stack([b.f("mocap", f"pose.position.{k}") for k in "xy"])
    st = b.t("pl_status"); S = [str(x) for x in b.f("pl_status", "data")]
    # HOLD windows of the planner
    win, start = [], None
    for t, s in zip(st, S):
        if s.startswith("HOLD") and start is None:
            start = t
        elif not s.startswith("HOLD") and start is not None:
            if t - start > 4.0:
                win.append((start + 3.0, t))
            start = None
    for a, c in win:
        tg = np.arange(a, c, 0.05)
        P = np.column_stack([np.interp(tg, tm, M[:, i]) for i in range(2)])
        v = np.linalg.norm(P[6:] - P[:-6], axis=1) / 0.3 * 1e3
        speeds.append(v)
v = np.concatenate(speeds)
print(f"hold windows: {len(speeds)}, {len(v) * 0.05:.0f} s; base speed (0.3 s) median {np.median(v):.0f}, "
      f"p90 {np.percentile(v, 90):.0f}, p99 {np.percentile(v, 99):.0f} mm/s")
for V in (20, 30, 40, 50):
    ok = v <= V
    # fraction of time inside a run of >= 0.5 s
    run, good = 0, 0
    for x in ok:
        run = run + 1 if x else 0
        good += 1 if run >= 10 else 0
    print(f"  slower than {V} mm/s: {100 * ok.mean():.0f} % of the time; held 0.5 s: {100 * good / len(ok):.0f} %")
