#!/usr/bin/env python3
"""fig8_envelope.py -- will a figure-8 flight stay inside a 2 x 2 m grid? (2026-10-05)

For every flight (am_ee_compare_driver npz), over the EXECUTING window:
  * the PLANNED footprint: bounding box of the base reference x_b (the airframe body
    origin, what the mocap tracks) and of the end-effector reference;
  * the simulated tracking errors (am_ee_compare_score.analyse(): EE / base rms and peak);
  * the PREDICTED FLIGHT errors = simulated / RATIO (default 0.6, the measured sim/flight
    rmse ratio of the 09-28 / 10-02 circle flights), applied to the PEAKS as well;
  * the predicted footprint: base-reference box grown by the predicted base peak on every
    side, unioned with the EE-reference box grown by the predicted EE peak, and whether it
    fits GRID x GRID m (default 2.0). Also where that box is centred relative to the
    takeoff point, i.e. where the grid centre must be.
Grouped by (controller, shape, speed): rms = mean over flights, peaks = max over flights.

    /usr/bin/python3 fig8_envelope.py --run wb A0.60 0.10 data/wb_A060_v010_a.npz ... --json out.json
"""
import argparse
import json
import math
import os
import sys
from collections import OrderedDict

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..",
                                 "application", "robotic_arm", "utils"))
import am_ee_compare_score as SC  # noqa: E402


def box(P):
    return np.array([P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max()])


def grow(b, m):
    return b + np.array([-m, m, -m, m])


def union(a, b):
    return np.array([min(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), max(a[3], b[3])])


def one(path):
    res, ser = SC.analyse(path)
    d, marks = SC.load(path)
    t0, t1 = ser["t0"], ser["t1"]
    wb = d["wbref"]; w = wb[(wb[:, 0] >= t0) & (wb[:, 0] <= t1)]
    e = ser["ee"]
    log = d["log"]
    takeoff = log[0, 1:3]
    return dict(ee_rms=res["ee_pos_rms_mm"], ee_peak=res["ee_pos_max_mm"],
                head_rms=float(math.sqrt(np.mean(np.asarray(ser["heading"][1]) ** 2))),
                head_peak=res["ee_head_max_deg"],
                base_rms=res["base_pos_rms_mm"], base_peak=res["base_pos_max_mm"],
                tilt=res["tilt_max_deg"], aborted=res["aborted"],
                base_box=box(w[:, 30:32]), ee_box=box(e[:, 4:6]),
                meas_box=union(box(log[(log[:, 0] >= t0) & (log[:, 0] <= t1)][:, 1:3]), box(e[:, 1:3])),
                takeoff=takeoff)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", nargs=4, action="append", metavar=("RIG", "SHAPE", "SPEED", "NPZ"), required=True)
    ap.add_argument("--ratio", type=float, default=0.6, help="simulated / flight rmse")
    ap.add_argument("--grid", type=float, default=2.0)
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    groups = OrderedDict()
    for rig, shape, speed, path in a.run:
        groups.setdefault((rig, shape, speed), []).append(one(path))
    out = []
    for (rig, shape, speed), runs in groups.items():
        g = dict(rig=rig, shape=shape, speed=float(speed), n=len(runs), aborted=sum(r["aborted"] for r in runs),
                 ee_rms=float(np.mean([r["ee_rms"] for r in runs])), ee_peak=max(r["ee_peak"] for r in runs),
                 head_rms=float(np.mean([r["head_rms"] for r in runs])), head_peak=max(r["head_peak"] for r in runs),
                 base_rms=float(np.mean([r["base_rms"] for r in runs])), base_peak=max(r["base_peak"] for r in runs),
                 tilt=max(r["tilt"] for r in runs))
        g["ee_rms_flight"] = g["ee_rms"] / a.ratio
        g["ee_peak_flight"] = g["ee_peak"] / a.ratio
        g["base_peak_flight"] = g["base_peak"] / a.ratio
        plan = union(runs[0]["base_box"], runs[0]["ee_box"])
        pred = union(grow(runs[0]["base_box"], g["base_peak_flight"] / 1e3), grow(runs[0]["ee_box"], g["ee_peak_flight"] / 1e3))
        meas = runs[0]["meas_box"]
        for r in runs[1:]:
            meas = union(meas, r["meas_box"])
        to = runs[0]["takeoff"]
        g.update(plan_box=[float(plan[1] - plan[0]), float(plan[3] - plan[2])],
                 sim_box=[float(meas[1] - meas[0]), float(meas[3] - meas[2])],
                 pred_box=[float(pred[1] - pred[0]), float(pred[3] - pred[2])],
                 pred_center_from_takeoff=[float((pred[0] + pred[1]) / 2 - to[0]), float((pred[2] + pred[3]) / 2 - to[1])])
        g["fits"] = bool(max(g["pred_box"]) <= a.grid)
        g["margin_m"] = float((a.grid - max(g["pred_box"])) / 2)
        out.append(g)
        print(f"{rig:9s} {shape:6s} v {float(speed):.2f}: EE rms {g['ee_rms']:5.1f} peak {g['ee_peak']:5.1f} mm"
              f" | flight-predicted rms {g['ee_rms_flight']:5.1f} peak {g['ee_peak_flight']:5.1f} (base {g['base_peak_flight']:5.1f})"
              f" | box plan {g['plan_box'][0]:.2f}x{g['plan_box'][1]:.2f} sim {g['sim_box'][0]:.2f}x{g['sim_box'][1]:.2f}"
              f" predicted {g['pred_box'][0]:.2f}x{g['pred_box'][1]:.2f} m -> {'FITS' if g['fits'] else 'DOES NOT FIT'}"
              f" {a.grid:.0f}x{a.grid:.0f} (margin {g['margin_m']*1e3:+.0f} mm/side), centre "
              f"({g['pred_center_from_takeoff'][0]:+.2f}, {g['pred_center_from_takeoff'][1]:+.2f}) m from takeoff"
              + (f" | {g['aborted']} ABORTED" if g["aborted"] else ""))
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
