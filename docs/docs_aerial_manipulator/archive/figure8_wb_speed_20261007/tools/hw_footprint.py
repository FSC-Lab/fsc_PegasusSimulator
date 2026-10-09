#!/usr/bin/env python3
"""hw_footprint.py -- how far the 1005 hardware figure-8 runs left their PLANNED box: per side, the measured
airframe (odometry) and EE extents beyond the planned airframe/EE reference box over the EXECUTING span.
    AM_NPZ=<dir> PYTHONNOUSERSITE=1 /usr/bin/python3 hw_footprint.py
"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_vs_decoupled_figure8_flight_20261005", "tools"))
import f1005 as F  # noqa
import metrics as M  # noqa
box = lambda P: np.array([P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max()])
uni = lambda a, b: np.array([min(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), max(a[3], b[3])])
out = {}
for nm in ("w3", "w4", "w5", "w6", "r5", "r6"):
    o, S = M.analyse(nm)
    plan = uni(box(S["xb"]), box(S["red"])); meas = uni(box(S["Pu"]), box(S["re"]))
    over = np.array([plan[0] - meas[0], meas[1] - plan[1], plan[2] - meas[2], meas[3] - plan[3]])
    out[nm] = dict(tag=F.RUNS[nm][0], v=F.RUNS[nm][3], plan=plan.tolist(), meas=meas.tolist(), over=over.tolist(),
                   ee_peak=o["metrics"]["ee_pos"]["max_norm"], base_peak=o["metrics"]["base_pos"]["max_norm"])
    print(f"{F.RUNS[nm][0]:6s} {F.RUNS[nm][3]:.2f}  planned {plan[1]-plan[0]:.3f} x {plan[3]-plan[2]:.3f} m  measured {meas[1]-meas[0]:.3f} x {meas[3]-meas[2]:.3f} m"
          f"  overshoot -x/+x/-y/+y {1e3*over[0]:+5.0f}/{1e3*over[1]:+5.0f}/{1e3*over[2]:+5.0f}/{1e3*over[3]:+5.0f} mm  (EE peak {out[nm]['ee_peak']:.0f}, airframe peak {out[nm]['base_peak']:.0f})")
json.dump(out, open(os.path.join(HERE, "..", "analysis", "hw_footprint.json"), "w"), indent=1)
