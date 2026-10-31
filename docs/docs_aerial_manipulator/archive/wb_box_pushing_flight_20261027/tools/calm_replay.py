#!/usr/bin/env python3
"""Replay the arm GS's calm-exit gate (push_pull_panel.cpp exitWhenCalm/baseSpeed) on the recorded
whole_body_planner/current_base: speed = |p(now) - p(now - 0.3 s)| / 0.3, calm = <= 40 mm/s for
0.3 s, checked every 100 ms, at most 3 s.  Usage: calm_replay.py <tag> <push DONE t>"""
import sys
import numpy as np
from bp_common import Bag

tag, tdone = sys.argv[1], float(sys.argv[2])
b = Bag(tag)
t = b.t("pl_current_base")
p = np.column_stack([b.f("pl_current_base", f"pose.position.{c}") for c in "xyz"])
t0 = tdone + 0.2                         # push_pull_auto_exit_delay_s
since, out = None, None
for tc in np.arange(t0, t0 + 3.05, 0.1):
    k = np.searchsorted(t, tc) - 1
    j = np.searchsorted(t, t[k] - 0.3 + 1e-3, side="right") - 1
    v = np.linalg.norm(p[k] - p[j]) / (t[k] - t[j])
    calm = v <= 0.04
    since = (since if since is not None else tc) if calm else None
    print(f"  {tc:6.2f}  v {1e3 * v:5.0f} mm/s  {'calm' if calm else ''}")
    if since is not None and tc - since >= 0.3 - 1e-6:
        out = f"calm at {tc:.2f}"; break
print(out or f"NOT calm within 3 s -> exit at {t0 + 3.0:.2f}")
