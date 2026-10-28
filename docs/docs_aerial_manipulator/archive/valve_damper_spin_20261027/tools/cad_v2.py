#!/usr/bin/env python3
"""Valve_Assembly_v2.stl: list the closed shells (bbox, volume, CoM) beside v1's,
and the new half-rings' radii about the bearing axis (the pad's gap to the 25 mm hub). Reuses the 10-15 converter's mesh functions."""
import os, collections, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
CONV = os.path.join(HERE, "..", "..", "valve_turning_20261015", "tools", "stl_to_usda_valve.py")
code = open(CONV).read(); ns = {"np": np}                   # the mesh functions only (no pxr import)
for name in ("read_stl", "shells", "solid_mass_props"):
    s = code.index(f"def {name}("); e = code.index("\ndef ", s + 5)
    exec(code[s:e], ns)
A = os.path.join(HERE, *[".."] * 5, "extensions", "fsc_aerial_manipulation", "fsc_aerial_manipulation", "rotorcraft", "assets")
for f in ("Valve_Assembly_v1.stl", "Valve_Assembly_v2.stl"):
    T = ns["read_stl"](os.path.join(A, f)) * 1e3
    lab = ns["shells"](T); cnt = collections.Counter(lab.tolist())
    print(f"== {f}: {len(T)} triangles, {len(cnt)} shells")
    for k, n in cnt.most_common():
        S = T[lab == k]; v = S.reshape(-1, 3)
        vol, com, I = ns["solid_mass_props"](S)
        print(f"  shell {k:6d} tris {n:6d} bbox x[{v[:,0].min():7.1f},{v[:,0].max():7.1f}] y[{v[:,1].min():6.1f},{v[:,1].max():6.1f}] "
              f"z[{v[:,2].min():6.1f},{v[:,2].max():6.1f}] mm  vol {vol/1e3:8.2f} cm3  com {np.round(com,1)}")

# the two new half-rings (v2 only): radial extent about the bearing axis, and the hub's radius
T = ns["read_stl"](os.path.join(A, "Valve_Assembly_v2.stl")) * 1e3
lab = ns["shells"](T); cnt = collections.Counter(lab.tolist())
ax = np.array([-1.1, 3.0])
for k, n in cnt.most_common():
    v = T[lab == k].reshape(-1, 3)
    r = np.hypot(v[:, 0] - ax[0], v[:, 1] - ax[1])
    if n == 916:
        print(f"half-ring {k}: r from axis {r.min():.2f} .. {r.max():.2f} mm, z {v[:,2].min():.1f}..{v[:,2].max():.1f}; "
              f"distinct r (rounded 0.1): {np.unique(np.round(r,1))[:12]} ... {np.unique(np.round(r,1))[-6:]}")
    if n == 11082:
        h = v[(v[:, 2] > 5) & (v[:, 2] < 26) & (r < 30)]
        rh = np.hypot(h[:, 0] - ax[0], h[:, 1] - ax[1])
        print(f"rotor below z 26 within r 30: r {rh.min():.2f}..{rh.max():.2f} mm, z {h[:,2].min():.1f}..{h[:,2].max():.1f}")
