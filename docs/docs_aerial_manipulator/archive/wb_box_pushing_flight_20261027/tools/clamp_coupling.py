#!/usr/bin/env python3
"""Does the clamp tie the box's tilt to the vehicle's? Box (obj_0) and vehicle (mocap) tilt vectors in
the push frame (along = the nose, + = nose down / box top forward), relative to their pre-clamp mean,
compared over a window; plus the box's tilt-guard reading. Usage: clamp_coupling.py <tag> <t_ref0> <t_ref1> <t0> <t1>"""
import sys
import numpy as np
from bp_common import Bag

tag = sys.argv[1]; r0, r1, a, c = (float(x) for x in sys.argv[2:6])
b = Bag(tag)
def R_of(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
def tilts(key):
    t = b.t(key); Q = np.column_stack([b.f(key, f"pose.orientation.{c}") for c in "wxyz"])
    v = np.array([R_of(q)[:, 2][:2] for q in Q])
    return t, v
T = np.arange(r0, c, 0.02)
tv, vv = tilts("mocap"); to, vo = tilts("obj")
V = np.column_stack([np.interp(T, tv, vv[:, k]) for k in range(2)])
O = np.column_stack([np.interp(T, to, vo[:, k]) for k in range(2)])
I = b.f("pl_info", "data"); ps = np.radians(I[-1, 81]); u = np.array([np.cos(ps), np.sin(ps)]); n = np.array([-u[1], u[0]])
ref = (T >= r0) & (T < r1); m = (T >= a) & (T <= c)
V = V - V[ref].mean(0); O = O - O[ref].mean(0)
for nm, ax in (("along (pitch)", u), ("across (roll)", n)):
    v, o = np.degrees(V[m] @ ax), np.degrees(O[m] @ ax)
    k = np.polyfit(v, o, 1)[0]; r = np.corrcoef(v, o)[0, 1]
    print(f"{tag} {a:.1f}-{c:.1f}s {nm:14s}: vehicle tilt rms {np.std(v):.2f} deg (range {v.min():+.1f}..{v.max():+.1f}), "
          f"box tilt rms {np.std(o):.2f} (range {o.min():+.2f}..{o.max():+.2f}); box/vehicle slope {k:+.3f}, corr {r:+.2f}")
