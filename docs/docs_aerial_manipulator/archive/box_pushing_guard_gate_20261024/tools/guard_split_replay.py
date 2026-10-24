#!/usr/bin/env python3
"""The q3 guards replayed on this campaign's flights (and the hardware pushes), for the threshold
combinations: static floor S (q3 < S for 0.2 s, 10 Hz tick) and the q2-aware rate rule predicting
against R (q3 + 0.2 q3_dot + max(0, q2 + 0.2 q2_dot - 45) < R while q3 falls > 5 deg/s for 0.05 s).
Flown 2026-10-24: S = R = 5. For each flight: when each combination would first trip after Ready's
end, q3 / q2 / q4 there, and the time to what actually happened (the flown abort / the end).
    /usr/bin/python3 guard_split_replay.py"""
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")
TICK, PERSIST = 0.1, 0.2


def lpf_rate(t, q, tc=0.02):
    v = np.zeros_like(q)
    for i in range(1, len(q)):
        d = t[i] - t[i - 1]
        if 0 < d < 0.1:
            v[i] = v[i - 1] + (1 - np.exp(-d / tc)) * ((q[i] - q[i - 1]) / d - v[i - 1])
        else:
            v[i] = v[i - 1] if d <= 0 else 0.0
    return v


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


def rate_trip(t, q2, q3, t0, t1, lim, vmin=5.0, hold=0.05):
    v2, v3 = lpf_rate(t, q2), lpf_rate(t, q3)
    pred = q3 + 0.2 * v3 + np.maximum(0.0, q2 + 0.2 * v2 - 45.0)
    since = None
    for i in np.flatnonzero((t >= t0) & (t <= t1)):
        if pred[i] < lim and v3[i] < -vmin:
            since = t[i] if since is None else since
            if t[i] - since >= hold - 1e-9:
                return t[i]
        else:
            since = None
    return None


COMBOS = [(5, 5), (5, 10), (10, 10)]
print(f"{'run':8s} {'flown end':28s} | " + " | ".join(f"S{s:>2d}/R{r:>2d}: t q3 q2 q4".ljust(30) for s, r in COMBOS))
for tag in [f"gg_s{k}" for k in range(1, 9)]:
    p = os.path.join(RUNS, tag + ".npz")
    if not os.path.exists(p):
        continue
    z = np.load(p, allow_pickle=True)
    mk = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
    J = z["joints"]
    t, q = J[:, 0], np.degrees(J[:, 1:5])
    t0 = mk.get("ready:end")
    if t0 is None:
        continue
    ev = []
    for ln in open(p[:-4] + ".log", errors="replace"):
        m = re.match(r"\[\s*([0-9.]+)s\] (.*)", ln)
        if m:
            ev.append((float(m.group(1)), m.group(2)))
    t_ab = min([tt for tt, x in ev if x.startswith("push_pull: ABORTING")] or [np.inf])
    t_end = min(t_ab, mk.get("exit:end", np.inf), t[-1])
    t1 = min(t_end, mk.get("push:end", np.inf))      # READY + the push (the abort's own motion excluded)
    what = (f"abort {t_ab - t0:5.2f} s after Ready" if np.isfinite(t_ab) else "no abort")
    cells = []
    for s, r in COMBOS:
        tr = [x for x in (static_trip(t, q[:, 2], t0, t1, s), rate_trip(t, q[:, 1], q[:, 2], t0, t1, r)) if x]
        if not tr:
            cells.append("-".ljust(30))
            continue
        tt = min(tr)
        k = np.searchsorted(t, tt)
        cells.append(f"{tt - t0:5.2f}s {q[k, 2]:5.1f} {q[k, 1]:5.1f} {q[k, 3]:6.1f}".ljust(30))
    print(f"{tag:8s} {what:28s} | " + " | ".join(cells))
