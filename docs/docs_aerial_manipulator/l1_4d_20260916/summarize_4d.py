#!/usr/bin/env python3
"""Score the 4-D vs 6-D campaign: whole-run metrics + per-leg tracking, to text and JSON.

    /usr/bin/python3 summarize_4d.py [run.npz ...]     (default: every npz here)

Wraps wb_l1_metrics.score() and wb_compare_metrics.{load,legs,score_leg}, so the
numbers are the same the rest of 7.15 quotes. Writes metrics.txt, legs.txt and
summary.json beside the runs.
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
UTILS = os.path.abspath(os.path.join(HERE, "..", "..", "..", "application", "robotic_arm", "utils"))
sys.path.insert(0, UTILS)
import wb_l1_metrics as M          # noqa: E402
import wb_compare_metrics as CM    # noqa: E402

paths = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "*.npz")))
whole, legs = {}, {}
for p in paths:
    name = os.path.basename(p).replace(".npz", "")
    s = M.score(p)
    whole[name] = {k: (float(v) if isinstance(v, (int, float, np.floating, np.integer)) and not isinstance(v, bool) else v)
                   for k, v in s.items()}
    try:
        run = CM.load(p)
        legs[name] = {}
        for lname, t0, t1 in CM.legs(run):
            sl = CM.score_leg(run, t0, t1)
            if sl:
                legs[name][lname] = sl
    except SystemExit as e:
        legs[name] = {"error": str(e)}

# whole-run table
names = list(whole)
w = max(20, max(len(n) for n in names) + 2)
lines = [f"{'metric':<26s}" + "".join(f"{n:>{w}s}" for n in names), "-" * (26 + w * len(names))]
for key, fmt, unit in M.ROWS:
    if not any(key in whole[n] for n in names):
        continue
    cells = []
    for n in names:
        v = whole[n].get(key)
        cells.append("-" if v is None else (fmt.format(v) if not isinstance(v, str) else v))
    label = f"{key} [{unit}]" if unit else key
    lines.append(f"{label:<26s}" + "".join(f"{c:>{w}s}" for c in cells))
open(os.path.join(HERE, "metrics.txt"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))

# per-leg table: peak / rms / settled CoM, settled EE, |Fy|, |Fraw|
leg_names = []
for n in names:
    for l in legs.get(n, {}):
        if l not in leg_names and l != "error":
            leg_names.append(l)
out = []
for l in leg_names:
    out.append(f"\n== {l}: peakCoM / rmsCoM / settledCoM [mm] | settledEE [mm] | tilt [deg] | tau [N.m] | clamp% | |Fy| | |Fraw| [N]")
    for n in names:
        s = legs.get(n, {}).get(l)
        if not s:
            out.append(f"  {n:<20s}  (no leg)")
            continue
        out.append(f"  {n:<20s} {s['peak_mm']:7.1f} / {s['rms_mm']:6.1f} / {s['settled_mm']:6.1f} | "
                   f"{s['ee_settled_mm']:6.1f} | {s['tilt_max']:5.2f} | {s['tau_max']:5.2f} | "
                   f"{s['clamp_pct']:5.1f} | {s['fy']:6.3f} | {s['fraw']:6.3f}")
open(os.path.join(HERE, "legs.txt"), "w").write("\n".join(out) + "\n")
print("\n".join(out))
json.dump({"whole": whole, "legs": legs}, open(os.path.join(HERE, "summary.json"), "w"), indent=1, default=str)
print(f"\nwrote {HERE}/metrics.txt legs.txt summary.json")
