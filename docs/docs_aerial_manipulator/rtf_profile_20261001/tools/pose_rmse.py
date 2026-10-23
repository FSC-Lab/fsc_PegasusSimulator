#!/usr/bin/env python3
"""pose_rmse.py -- pose-level tracking RMSE table for the RTF-1 comparison report.

Reuses am_ee_compare_score.analyse() (same EXECUTING window, same conventions)
and adds RMS for the two series it only reports mean/p95/max for: EE heading
error (the `az` array in the heading series) and base yaw error (`ey` in the
base series). EE/base POSITION rms and joint rms are already in `res`.

    PYTHONNOUSERSITE=1 /usr/bin/python3 pose_rmse.py LABEL=run.npz [...] [--json out.json]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..",
                                 "application", "robotic_arm", "utils"))
import am_ee_compare_score as SC  # noqa: E402


def rms(x):
    return float(math.sqrt(np.mean(np.asarray(x) ** 2)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", help="LABEL=path.npz")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    out = {}
    for spec in a.runs:
        lab, path = spec.split("=", 1)
        res, ser = SC.analyse(path)
        row = dict(ee_pos_rms_mm=res["ee_pos_rms_mm"],
                   ee_head_rms_deg=rms(ser["heading"][1]) if ser["heading"] is not None else None,
                   base_pos_rms_mm=res["base_pos_rms_mm"],
                   base_yaw_rms_deg=rms(ser["base"][2]) if ser["base"] is not None else None,
                   joint_rms_deg=res["joint_rms_deg"], joint_rms_all_deg=res["joint_rms_all_deg"],
                   aborted=res["aborted"])
        out[lab] = row
        print(f"{lab:10s} EE pos {row['ee_pos_rms_mm']:6.2f} mm | EE head {row['ee_head_rms_deg']:5.2f} deg | "
              f"base pos {row['base_pos_rms_mm']:6.2f} mm | base yaw {row['base_yaw_rms_deg']:5.2f} deg | "
              f"joints {np.round(row['joint_rms_deg'], 2)} deg")
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
