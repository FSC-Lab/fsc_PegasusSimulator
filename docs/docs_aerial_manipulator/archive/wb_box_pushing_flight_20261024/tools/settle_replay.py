#!/usr/bin/env python3
"""Replay candidate settle conditions on the recorded hardware pushes (planner telemetry
push_pull/arrival_error, 10 Hz): for the hold after Go To Start (step 0, target = the point above
the handle), when the claw would first have been within TOL and slower than VMAX (over 0.3 s) for
DWELL -- the automatic Ready's condition -- against when Ready really started.
Usage: /usr/bin/python3 settle_replay.py [tol_mm vmax_mm_s dwell_s]"""
import sys
import numpy as np
from bp_common import Bag

TOL, VMAX, DWELL = (float(x) for x in (sys.argv[1:4] if len(sys.argv) > 3 else (50, 30, 0.5)))
FLIGHTS = [("170123", "10-21"), ("172853", "10-23"), ("150423", "10-24 #1"), ("151159", "10-24 #2"),
           ("162850", "10-24 A"), ("163413", "10-24 B"), ("164419", "10-24 C")]
print(f"condition: |e| <= {TOL:.0f} mm, speed <= {VMAX:.0f} mm/s, held {DWELL:.1f} s")
for tag, day in FLIGHTS:
    b = Bag(tag)
    st = b.edges("pl_pstatus")
    t_gs = [t for t, s in st if s.startswith("DONE go_to_start")][0]
    t_r0 = [t for t, s in st if s.startswith("FLYING ready")][0]
    ta, A = b.t("pl_arr"), b.f("pl_arr", "data")
    m = (A[:, 0] == 0) & (ta >= t_gs) & (ta < t_r0)
    t, E = ta[m], A[m, 3:6] * 1e3
    if len(t) < 3:
        print(f"{day:9s} no hold samples")
        continue
    since, met = None, None
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] - 0.3)
        v = np.linalg.norm(E[i] - E[j]) / max(t[i] - t[j], 1e-3) if t[i] - t[j] > 0.2 else np.inf
        ok = np.linalg.norm(E[i]) <= TOL and v <= VMAX
        if ok:
            since = t[i] if since is None else since
            if t[i] - since >= DWELL:
                met = t[i]
                break
        else:
            since = None
    e_r0 = np.linalg.norm(E[-1])
    if met is None:
        print(f"{day:9s} hold {t_r0 - t_gs:.1f} s: NOT met before Ready started (|e| {e_r0:.0f} mm at Ready)")
    else:
        k = np.searchsorted(t, met)
        print(f"{day:9s} hold {t_r0 - t_gs:.1f} s: met {met - t_gs:.1f} s after Go To Start (|e| {np.linalg.norm(E[k]):.0f} mm); "
              f"Ready really started at {t_r0 - t_gs:.1f} s with |e| {e_r0:.0f} mm")
