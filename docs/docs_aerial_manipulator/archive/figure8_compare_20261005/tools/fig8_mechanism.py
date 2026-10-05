#!/usr/bin/env python3
"""fig8_mechanism.py -- what the figure-8's yaw rate does to each controller (2026-10-05).

For every flight, on a uniform 50 Hz grid over the EXECUTING window:
  * EE heading error vs the reference base yaw rate: least-squares slope [s] and R^2.
    For the geometric law (omega_d = 0) the attitude loop's steady yaw lag is
    (K_omega,z / K_R,z) * psi_dot = (0.411 / 1.737) * psi_dot = 0.237 s * psi_dot.
  * EE position error where the yaw rate is high (|psi_dot| > 35 deg/s, the lobe
    tops and bottoms) vs low (|psi_dot| < 20 deg/s, the crossing and lobe ends).
  * the hover offset before Start (base position error over the 4 s before the run).

    /usr/bin/python3 fig8_mechanism.py LABEL=npz [...] --json out.json
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..",
                                 "application", "robotic_arm", "utils"))
import am_ee_compare_score as SC  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    out = {}
    for spec in a.runs:
        lab, path = spec.split("=", 1)
        res, ser = SC.analyse(path)
        d, marks = SC.load(path)
        t0, t1 = ser["t0"], ser["t1"]
        w = d["wbref"]; w = w[(w[:, 0] >= t0 - 1) & (w[:, 0] <= t1 + 1)]
        tg = np.arange(t0 + 0.5, t1 - 0.5, 0.02)
        yaw = np.interp(tg, w[:, 0], np.unwrap(np.arctan2(w[:, 11], w[:, 10])))
        yr = np.degrees(np.gradient(yaw, tg))
        th, az = ser["heading"][0], ser["heading"][1]
        h = np.interp(tg, th, az)
        A = np.column_stack([yr, np.ones_like(yr)])
        (k, c), *_ = np.linalg.lstsq(A, h, rcond=None)
        r2 = 1 - np.sum((h - A @ [k, c]) ** 2) / np.sum((h - h.mean()) ** 2)
        e = ser["ee"]
        ee = np.interp(tg, e[:, 0], 1e3 * np.linalg.norm(e[:, 1:4] - e[:, 4:7], axis=1))
        hi, lo = np.abs(yr) > 35, np.abs(yr) < 20
        # hover offset before Start: base error vs the converted reference
        L = d["log"]; wb = d["wbref"]
        pre = L[(L[:, 0] > t0 - 4.0) & (L[:, 0] < t0)]
        off = float("nan")
        if pre.shape[0] > 5:
            xb = SC.interp_rows(wb[:, 0], wb[:, 30:33], pre[:, 0])
            off = 1e3 * float(np.linalg.norm(pre[:, 1:3] - xb[:, :2], axis=1).mean())
        row = dict(head_slope_s=float(k), head_bias_deg=float(c), head_r2=float(r2),
                   ee_rms_highrate_mm=float(math.sqrt(np.mean(ee[hi] ** 2))),
                   ee_rms_lowrate_mm=float(math.sqrt(np.mean(ee[lo] ** 2))),
                   hover_offset_mm=off)
        out[lab] = row
        print(f"{lab:8s} heading error = {k:+.3f} s x yaw rate {c:+.2f} deg (R2 {r2:.2f}) | EE rms at |yaw rate|>35: "
              f"{row['ee_rms_highrate_mm']:.1f} mm, <20: {row['ee_rms_lowrate_mm']:.1f} mm | hover offset before Start {off:.1f} mm")
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
