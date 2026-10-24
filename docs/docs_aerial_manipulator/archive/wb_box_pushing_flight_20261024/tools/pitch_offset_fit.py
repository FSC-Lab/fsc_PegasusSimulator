#!/usr/bin/env python3
"""Is the base's along-the-push error the structural T*e_R/k_x of its pitch attitude error?
For each afternoon flight, from Go To Start to the end of Ready (every 0.1 s): the CoM error along the
push (x_cd - x_c in the box frame, + = the base SHORT of its reference) against the attitude error about
the model x axis (wb_control_debug [28], = body pitch on this airframe), and the slope fitted through
zero against the prediction u1/k_x (k_x 50.03).
Usage: /usr/bin/python3 pitch_offset_fit.py"""
import json, os
import numpy as np
from bp_common import Bag, yaw_deg

KX = 50.03
out = {}
for tag in ("162850", "163413", "164419"):
    b = Bag(tag)
    st = b.edges("pl_pstatus")
    t0 = [t for t, s in st if s.startswith("FLYING go_to_start")][0]
    t1 = [t for t, s in st if s.startswith("DONE ready")][0]
    tw, W = b.t("wb"), b.f("wb", "data")
    oy = np.radians(np.median(yaw_deg(*[b.f("obj", f"pose.orientation.{k}") for k in "wxyz"])[b.t("obj") < t0]))
    ex, ey = [], []
    for t in np.arange(t0, t1, 0.1):
        k = np.searchsorted(tw, t)
        sl = slice(max(k - 12, 0), k + 13)
        d = (W[sl, 45:47] - W[sl, 48:50]).mean(0)
        short = -(-np.sin(oy) * d[0] + np.cos(oy) * d[1]) * 1e3
        ex.append(W[sl, 28].mean() * 1e3)
        ey.append(short)
    ex, ey = np.array(ex), np.array(ey)
    slope = float(ex @ ey / (ex @ ex))
    r = float(np.corrcoef(ex, ey)[0, 1])
    u1 = float(np.median(W[(tw > t0) & (tw < t1), 17]))
    print(f"{tag}: short-along vs pitch error: slope {slope:.2f} mm/mrad (r {r:.2f}, n {len(ex)}); u1/k_x = {u1/KX:.2f} mm/mrad")
    out[tag] = {"eR": [round(float(v), 1) for v in ex], "short": [round(float(v), 1) for v in ey],
                "slope": round(slope, 3), "r": round(r, 3), "pred": round(u1 / KX, 3)}
json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pitch_offset_fit.json"), "w"))
