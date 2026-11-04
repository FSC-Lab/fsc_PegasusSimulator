#!/usr/bin/env python3
"""Event timeline of each valve flight: planner / push-pull status edges, WB mode, PX4 nav state +
arming, mocap guard state, and every /rosout line from the planner, the flight node and the arm GS."""
import sys
import numpy as np
from vt_common import Bag, BAGS, NAV, GUARD
for tag in (sys.argv[1:] or BAGS):
    b = Bag(tag)
    ev = []
    for key, lab in (("pl_pstatus", "PP"), ("pl_status", "PL"), ("wbmode", "MODE"), ("ctype", "CTRL")):
        for t, s in b.edges(key):
            ev.append((t, lab, s))
    t = b.t("vstatus"); nav = b.f("vstatus", "nav_state"); arm = b.f("vstatus", "arming_state")
    last = None
    for ti, n, a in zip(t, nav, arm):
        s = f"{NAV.get(int(n), int(n))} arm={int(a)}"
        if s != last: ev.append((ti, "PX4", s)); last = s
    if b.has("guard"):
        g = b.f("guard", "data"); last = None
        for ti, row in zip(b.t("guard"), g):
            s = GUARD.get(int(row[0]), row[0])
            if s != last: ev.append((ti, "GUARD", s)); last = s
    if b.has("rosout"):
        names = b.f("rosout", "name"); msgs = b.f("rosout", "msg")
        for ti, n, m in zip(b.t("rosout"), names, msgs):
            n = str(n)
            if any(k in n for k in ("planner", "fsc_autopilot", "ground", "panel", "joint_plot", "arm_gs", "custom")):
                ev.append((ti, "LOG " + n.split("/")[-1][:22], str(m)[:300]))
    ev.sort(key=lambda e: e[0])
    print(f"===== {tag}   duration {max(b.t(k.replace('__recv',''))[-1] for k in b.z.files if k.endswith('__recv')):.1f} s")
    for ti, lab, s in ev:
        print(f"{ti:8.2f}  {lab:28s} {s}")
