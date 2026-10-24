#!/usr/bin/env python3
"""How close the box-pushing arm comes to the q3 = +50 deg wall, on both hardware pushes and the
Isaac runs of box_pushing_contact_stability_tune_20261021 (window: arm on the handle -> exit /
abort / DIRECT loss, as guard_variants.py scores it). Prefix 'fw' = the free axis from the room
hold (shipped 2026-10-22).
Usage: /usr/bin/python3 q3_upper_usage.py"""
import io
import re
import contextlib
import numpy as np

with contextlib.redirect_stdout(io.StringIO()):
    import guard_variants as G

_score = G.score


def keep(name, t, q2, q3, t0, t_end, outcome):
    out = _score(name, t, q2, q3, t0, t_end, outcome)
    out["win0"] = t0
    return out


G.score = keep
rows = list(G.hw_runs()) + list(G.sim_runs())
fam = {}
print(f"{'run':18s} {'outcome':9s} {'q3 max':>7s} {'s>45':>6s} {'s>48':>6s} {'q3 min':>7s} {'q2 max':>7s}")
for r in rows:
    t, q2, q3 = r["q"]
    m = (t >= r["win0"]) & (t <= r["t_end"])
    dt = np.median(np.diff(t[m])) if m.sum() > 2 else 0.0
    hi = q3[m].max()
    a45, a48 = (q3[m] > 45).sum() * dt, (q3[m] > 48).sum() * dt
    print(f"{r['name']:18s} {r['outcome']:9s} {hi:7.1f} {a45:6.2f} {a48:6.2f} {q3[m].min():7.1f} {q2[m].max():7.1f}")
    key = "HW" if r["name"].startswith("HW") else ("fw (shipped)" if "_fw" in r["name"] or "fw" in r["name"].split()[1][:4]
                                                  else "older configs")
    fam.setdefault(key, []).append((hi, a48))
print("\nsummary: runs, q3 max (median / max), runs reaching > 48 deg")
for k, v in fam.items():
    a = np.array(v)
    print(f"  {k:14s} {len(a):3d}  {np.median(a[:,0]):5.1f} / {a[:,0].max():5.1f}  {(a[:,0] > 48).sum()}")
