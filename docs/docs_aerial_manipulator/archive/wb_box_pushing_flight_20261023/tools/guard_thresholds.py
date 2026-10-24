#!/usr/bin/env python3
"""Lower q3 thresholds for the two q3 guards, replayed on the same runs as guard_variants.py
(both hardware pushes + the Isaac runs in box_pushing_contact_stability_tune_20261021/runs).

Rule replayed = the shipped 2026-10-24 planner: static q3 < S for 0.2 s (10 Hz tick), and the
q2-aware rate rule q3 + 0.2 q3_dot + max(0, q2 + 0.2 q2_dot - 45) < R while q3 falls > 5 deg/s for
0.05 s. Today S = R = 10 (one key). For each (S, R): false trips on completed runs, and how early
each buckle (left DIRECT) is caught.
Usage: /usr/bin/python3 guard_thresholds.py"""
import io
import contextlib
import numpy as np

with contextlib.redirect_stdout(io.StringIO()):
    import guard_variants as G

TICK, PERSIST = 0.1, 0.2


def static_trip(t, q3, t0, t1, lim):
    since = None
    for tk in np.arange(t0, t1, TICK):
        if np.interp(tk, t, q3) < lim:
            since = tk if since is None else since
            if tk - since >= PERSIST - 1e-9:
                return tk
        else:
            since = None
    return None


_score = G.score


def _score_keep_t0(name, t, q2, q3, t0, t_end, outcome):
    out = _score(name, t, q2, q3, t0, t_end, outcome)
    out["win0"] = t0
    return out


G.score = _score_keep_t0
runs = list(G.hw_runs()) + list(G.sim_runs())
COMBOS = [(10, 10), (5, 10), (0, 10), (5, 5), (0, 0)]
print(f"{len(runs)} runs; buckles = runs that left DIRECT; 'false' = a trip on a run that completed")
print(f"{'static S / rate R':18s} {'false trips':>11s}  {'buckles caught: lead s @ q3 deg':40s}  "
      f"{'HW 172853 (10-23)':>18s} {'HW 170123 (10-21)':>18s}")
for S, R in COMBOS:
    false, caught, hw = [], [], {}
    for r in runs:
        t, q2, q3 = r["q"]
        tend = r["t_end"]
        t1 = tend + (1.0 if r["outcome"] == "buckle" else 0.0)
        v2, v3 = G.lpf_rate(t, q2), G.lpf_rate(t, q3)
        pred = q3 + 0.2 * v3 + np.maximum(0.0, q2 + 0.2 * v2 - 45.0)
        win0 = r["win0"]
        ts = static_trip(t, q3, win0, t1, S)
        tr = G.rate_trip(t, pred, v3, win0, t1, R)
        trip = G.first(ts, tr)
        name = r["name"]
        if name.startswith("HW"):
            hw[name] = "-" if trip is None else f"{trip - tend:+.2f}s q3 {np.interp(trip, t, q3):4.1f}"
        if r["outcome"] == "completed" and trip is not None:
            false.append(name)
        if r["outcome"] == "buckle":
            caught.append("miss" if trip is None or trip > tend else
                          f"{tend - trip:.2f}@{np.interp(trip, t, q3):.1f}")
    print(f"S {S:2d} / R {R:2d}        {len(false):11d}  {', '.join(caught):40s}  "
          f"{hw.get('HW 172853', ''):>18s} {hw.get('HW 170123', ''):>18s}")
    if false:
        print(f"{'':20s}false: {', '.join(false)}")
