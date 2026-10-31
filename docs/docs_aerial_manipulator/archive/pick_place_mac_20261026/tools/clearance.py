#!/usr/bin/env python3
"""clearance.py <run.npz>... -- the hook basket's clearance over the PLACE hat on the Ready To Place
approach (2026-10-26). The basket (110 x 115 x 65 mm, centre = obj_0) can strike the hat's rim when
its footprint first overlaps the platform (centre within 100 + 57.5 mm of the pillar axis) while its
bottom is below the platform top (1.008 m). Prints, per run: the bottom's height over the platform
top at that first overlap, the lowest over the overlap until the place descent starts, the base and
basket standing offsets in the carry, and whether the basket stayed on the hook (payload - claw)."""
import sys
import numpy as np
PLACE = np.array([-1.0, -1.0]); TOP = 1.008; HALF_H = 0.0325; R_OVL = 0.100 + 0.0575
for f in sys.argv[1:]:
    d = np.load(f, allow_pickle=True)
    p, c, o = d["payload"], d["claw"], d["odom"]
    mk = dict(zip(d["marks_name"], d["marks_t"]))
    t0, t1 = mk.get("ready_place:start"), mk.get("place:start", mk.get("ready_place:start", 0) + 8.0)
    if t0 is None:
        print(f"{f}: never reached Ready To Place"); continue
    w = (p[:, 0] >= t0) & (p[:, 0] <= t1)
    r = np.hypot(p[w, 1] - PLACE[0], p[w, 2] - PLACE[1])
    bot = p[w, 3] - HALF_H - TOP
    ov = r < R_OVL
    first = bot[ov][0] if ov.any() else np.nan
    low = bot[ov].min() if ov.any() else np.nan
    cl = np.interp(p[w, 0], c[:, 0], c[:, 3])
    hang = (cl - p[w, 3])
    off_hook = np.nanmax(hang) > 0.25          # basket left behind (hangs ~0.19 m below the claw)
    print(f"{f.split('/')[-1]:28s} clearance at first overlap {first * 1e3:+6.0f} mm, lowest {low * 1e3:+6.0f} mm, "
          f"claw-basket max {np.nanmax(hang) * 1e3:4.0f} mm{'  BASKET LEFT THE HOOK' if off_hook else ''}")
