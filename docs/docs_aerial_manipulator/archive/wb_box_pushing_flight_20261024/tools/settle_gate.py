#!/usr/bin/env python3
"""Would a 'settled' close gate have closed the jaws in the 10-23 open-jaw hold (45.00-48.84 s)?
Speeds are displacement over a 0.3 s window (raw mocap velocity is 40+ mm/s of noise, so no
instantaneous velocity). Claw = planner FK (current_ee); error = push_pull/arrival_error e(0..1)
(world x = across the fin, y = along). Also the same speeds in the hover above the handle.
Usage: /usr/bin/python3 settle_gate.py"""
import numpy as np
from bp_common import Bag

b = Bag("172853")
W = 0.3


def speed(key, t):
    tt = b.t(key)
    p = np.column_stack([b.f(key, f"pose.position.{c}") for c in "xy"])
    a = np.array([np.interp(t - W, tt, p[:, i]) for i in range(2)])
    c = np.array([np.interp(t, tt, p[:, i]) for i in range(2)])
    return np.hypot(*(c - a)) / W * 1e3


ta, A = b.t("pl_arr"), b.f("pl_arr", "data")
grid = np.arange(40.5, 48.84, 0.05)
for name, t0, t1 in (("hover above the handle", 40.5, 41.8), ("Ready descent", 41.8, 45.0),
                     ("open-jaw room hold", 45.0, 48.84)):
    g = grid[(grid >= t0) & (grid < t1)]
    vb = np.array([speed("mocap", t) for t in g])
    vc = np.array([speed("pl_current_ee", t) for t in g])
    print(f"{name:24s} base speed median {np.median(vb):5.1f} / p90 {np.percentile(vb, 90):5.1f} mm/s, "
          f"claw median {np.median(vc):5.1f} / p90 {np.percentile(vc, 90):5.1f} mm/s")
g = grid[grid >= 45.0]
vb = np.array([speed("mocap", t) for t in g])
vc = np.array([speed("pl_current_ee", t) for t in g])
ex = np.interp(g, ta, A[:, 3]) * 1e3
ey = np.interp(g, ta, A[:, 4]) * 1e3
pos_ok = (np.abs(ex) <= 3.0) & (np.abs(ey) <= 20.0)
print(f"\nopen-jaw hold, {len(g)} samples at 20 Hz:")
print(f"  position gate (|across| <= 3, |along| <= 20 mm) met {pos_ok.mean()*100:4.1f} % "
      f"(first at {g[pos_ok][0]:.2f} s)" if pos_ok.any() else "  position gate never met")
for vcl, vbl in ((10, 15), (10, 25), (20, 30)):
    ok = pos_ok & (vc < vcl) & (vb < vbl)
    print(f"  + claw < {vcl} mm/s and base < {vbl} mm/s: met {ok.mean()*100:4.1f} %"
          + (f" (first at {g[ok][0]:.2f} s)" if ok.any() else ""))
k = np.argmin(np.abs(g - 47.79))
print(f"  at the close command 47.79 s: across {ex[k]:+.1f} mm, along {ey[k]:+.1f} mm, "
      f"claw {vc[k]:.0f} mm/s, base {vb[k]:.0f} mm/s")
