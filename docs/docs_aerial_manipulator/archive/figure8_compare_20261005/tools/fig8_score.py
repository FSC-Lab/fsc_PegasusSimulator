#!/usr/bin/env python3
"""fig8_score.py -- the figure-8 comparison's per-flight numbers (2026-10-05).

Same window and conventions as the circle report (am_ee_compare_score.analyse(),
the EXECUTING window of the run): EE/base position and heading RMS, joint RMS
against the streamed joint reference. Adds what the figure-8 needs:

  * the EE error split along the reference path: ALONG-track (positive = ahead)
    and CROSS-track (positive = to the left of the direction of travel), mean and
    rms, plus the vertical error -- the circle's radial/tangential split does not
    exist on an 8;
  * the reference's own numbers over the run: path length, mean speed over the
    constant-rate lap window, peak speed, peak base yaw rate;
  * actuator checks from the controller's debug array: joint-torque clamp count
    and peak |tau_joint| (whole-body [13..16], [51]); rotor saturation (whole-body
    unallocated wrench [52..54]; geometric motor commands [32..35] at 0 or 1);
  * peak tilt.

    PYTHONNOUSERSITE=1 /usr/bin/python3 fig8_score.py LABEL=rig:run.npz [...] --json out.json
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


def rms(x):
    x = np.asarray(x, float)
    return float(math.sqrt(np.mean(x ** 2))) if x.size else float("nan")


def path_split(e):
    """EE error resolved on the reference path: along (+ ahead), cross (+ left), vertical."""
    t, m, r = e[:, 0], e[:, 1:4], e[:, 4:7]
    tan = np.gradient(r[:, :2], t, axis=0)
    sp = np.linalg.norm(tan, axis=1)
    ok = sp > 0.02                     # the direction of travel is undefined at the rests
    tan = tan / (sp[:, None] + 1e-12)
    left = np.column_stack([-tan[:, 1], tan[:, 0]])
    d = m - r
    along = np.sum(d[:, :2] * tan, 1)[ok]
    cross = np.sum(d[:, :2] * left, 1)[ok]
    return dict(along_mean_mm=1e3 * float(along.mean()), along_rms_mm=1e3 * rms(along),
                cross_mean_mm=1e3 * float(cross.mean()), cross_rms_mm=1e3 * rms(cross),
                z_mean_mm=1e3 * float(d[:, 2].mean()), z_rms_mm=1e3 * rms(d[:, 2]))


def reference_numbers(d, t0, t1):
    """The streamed reference over the run: EE path, its speed, the base yaw rate."""
    ee = d["ee"]
    e = ee[(ee[:, 0] >= t0) & (ee[:, 0] <= t1)]
    r = e[:, 4:7]
    seg = np.linalg.norm(np.diff(r, axis=0), axis=1)
    out = dict(ref_path_m=float(seg.sum()))
    wb = d["wbref"]
    w = wb[(wb[:, 0] >= t0) & (wb[:, 0] <= t1)]
    # uniform 50 Hz grid before differencing (receive stamps are bursty)
    tg = np.arange(w[0, 0], w[-1, 0], 0.02)
    red = SC.interp_rows(w[:, 0], w[:, 13:16], tg)
    v = np.linalg.norm(np.gradient(red, tg, axis=0), axis=1)
    out["ref_ee_speed_peak"] = float(np.percentile(v, 99.5))
    yaw = np.interp(tg, w[:, 0], np.unwrap(np.arctan2(w[:, 11], w[:, 10])))
    out["ref_yaw_rate_peak_degs"] = float(np.degrees(np.percentile(np.abs(np.gradient(yaw, tg)), 99.5)))
    out["ref_yaw_span_deg"] = float(np.degrees(yaw.max() - yaw.min()))
    return out


def actuators(d, rig, t0, t1):
    raw = d["dbg"]
    # an object array: SAFETY ticks publish a shorter prefix than DIRECT ones,
    # so keep the rows of the longest (DIRECT) length only
    if raw.dtype == object:
        n = max(len(x) for x in raw)
        dbg = np.array([np.asarray(x, float) for x in raw if len(x) == n])
    else:
        dbg = np.asarray(raw, float)
    if dbg.ndim != 2 or dbg.shape[0] < 10:
        return {}
    g = dbg[(dbg[:, 0] >= t0) & (dbg[:, 0] <= t1)]
    out = {"dbg_samples": int(g.shape[0])}
    if rig == "wb" and g.shape[1] > 56:
        tau = g[:, 1 + 13:1 + 17]
        out["tau_joint_peak"] = float(np.abs(tau).max())
        out["joint_clamp_pct"] = 100.0 * float(np.mean(g[:, 1 + 51] > 0))
        unal = np.linalg.norm(g[:, 1 + 52:1 + 55], axis=1)
        out["rotor_sat_pct"] = 100.0 * float(np.mean(unal > 1e-6))
    elif rig == "decoupled" and g.shape[1] > 36:
        mot = g[:, 1 + 32:1 + 36]
        out["motor_min"] = float(mot.min()); out["motor_max"] = float(mot.max())
        out["rotor_sat_pct"] = 100.0 * float(np.mean(np.any((mot <= 1e-4) | (mot >= 1 - 1e-4), axis=1)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", help="LABEL=rig:path.npz")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    out = {}
    for spec in a.runs:
        lab, rest = spec.split("=", 1)
        rig, path = rest.split(":", 1)
        res, ser = SC.analyse(path)
        d, marks = SC.load(path)
        t0, t1 = ser["t0"], ser["t1"]
        row = dict(rig=rig, file=os.path.basename(path), aborted=res["aborted"], reason=res["reason"],
                   s=res.get("s"), s_max=res.get("s_max"), T_lap=res.get("T_lap"), run_s=res.get("run_duration"),
                   ee_pos_rms_mm=res["ee_pos_rms_mm"], ee_pos_max_mm=res["ee_pos_max_mm"],
                   ee_lag_s=res["ee_lag_s"], ee_resid_after_lag_mm=res["ee_residual_after_lag_mm"],
                   ee_head_rms_deg=rms(ser["heading"][1]), ee_head_max_deg=res["ee_head_max_deg"],
                   base_pos_rms_mm=res["base_pos_rms_mm"], base_pos_max_mm=res["base_pos_max_mm"],
                   base_yaw_rms_deg=rms(ser["base"][2]), base_yaw_max_deg=res["base_yaw_max_deg"],
                   tilt_max_deg=res["tilt_max_deg"],
                   joint_rms_deg=res["joint_rms_deg"], joint_max_deg=res["joint_max_deg"])
        row.update(path_split(ser["ee"]))
        row.update(reference_numbers(d, t0, t1))
        row.update(actuators(d, rig, t0, t1))
        out[lab] = row
        print(f"{lab:8s} EE {row['ee_pos_rms_mm']:6.2f} mm (max {row['ee_pos_max_mm']:5.1f}) | head {row['ee_head_rms_deg']:5.2f} deg "
              f"(max {row['ee_head_max_deg']:5.1f}) | base {row['base_pos_rms_mm']:6.2f} mm, yaw {row['base_yaw_rms_deg']:5.2f} deg | "
              f"along {row['along_mean_mm']:+6.1f}/{row['along_rms_mm']:5.1f} cross {row['cross_mean_mm']:+6.1f}/{row['cross_rms_mm']:5.1f} "
              f"z {row['z_mean_mm']:+6.1f} mm | joints {np.round(row['joint_rms_deg'], 2)} | tilt {row['tilt_max_deg']:.1f} | "
              f"ref path {row['ref_path_m']:.3f} m, peak yaw {row['ref_yaw_rate_peak_degs']:.1f} deg/s | "
              + " ".join(f"{k} {v:.3g}" for k, v in row.items() if k in ("tau_joint_peak", "joint_clamp_pct", "rotor_sat_pct", "motor_min", "motor_max"))
              + (f" | ABORTED: {row['reason']}" if row["aborted"] else ""))
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)
        print("wrote", a.json)


if __name__ == "__main__":
    main()
