#!/usr/bin/env python3
"""The 0928 flight report's error budget, applied to an Isaac circle flight
(am_ee_compare_driver.py npz), so a sim run and the hardware flights are split
the SAME way (wb_vs_decoupled_flight_20260928/tools/ee_budget.py, sweep_band.py,
com_spectrum.py):

    r_e - r_ed = (x_b - x_b,ref)                  airframe position
               + (R0 - R0,ref) r_0e(q_d)          airframe attitude
               + R0 (r_0e(q) - r_0e(q_d))         arm joints

plus the airframe error's share of variance in the arm-sweep band (0.13-0.21 Hz
for the 6 s q2 period) along-track / radial, and the detrended CoM xy error by
band (<0.1, 0.1-0.6, >0.6 Hz). Everything on a uniform 100 Hz grid over the
EXECUTING window (marks run_start..run_end); spectra skip 3 s at each end.

    PYTHONNOUSERSITE=1 /usr/bin/python3 sim_budget.py LABEL=run.npz [LABEL=run.npz ...] [--json out.json]
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.signal import periodogram, welch

HERE = os.path.dirname(os.path.abspath(__file__))
PEG = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(PEG, "application", "robotic_arm", "utils"))
import am_ee_compare_driver as DRV  # noqa: E402

FS = 100.0
G = 9.81


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def interp_rows(ts, X, td):
    return np.column_stack([np.interp(td, ts, X[:, i]) for i in range(X.shape[1])])


def rms(x):
    return float(math.sqrt(np.mean(np.asarray(x) ** 2)))


def analyse(path):
    d = np.load(path, allow_pickle=True)
    marks = dict((str(m).split("=")[0], float(str(m).split("=")[1])) for m in d["marks"])
    t0, t1 = marks["run_start"], marks["run_end"]
    base_com = [float(v) for v in d["base_com"]] if "base_com" in d.files else [0.0, -0.017854, 0.0]
    B, TP, params = DRV._bridge_helpers(base_com)
    Rzm90 = TP._Rz(-0.5 * math.pi)
    t = np.arange(t0, t1, 1.0 / FS)
    log = d["log"]; js = d["js"]; wb = d["wbref"]; ee = d["ee"]
    js = js[np.all(np.isfinite(js), axis=1)]
    Q = log[:, 7:11].copy()
    for k in range(1, len(Q)):
        if Q[k] @ Q[k - 1] < 0:
            Q[k] = -Q[k]
    P = interp_rows(log[:, 0], log[:, 1:4], t)
    Qu = interp_rows(log[:, 0], Q, t); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    q = interp_rows(js[:, 0], js[:, 1:5], t)
    W = interp_rows(wb[:, 0], wb[:, 1:], t)
    x_cd, xdd, b1d, r_ed, q_d, x_b = W[:, 0:3], W[:, 6:9], W[:, 9:12], W[:, 12:15], W[:, 21:25], W[:, 29:32]
    n = len(t)
    eb = P - x_b; ea = np.zeros((n, 3)); ej = np.zeros((n, 3)); re = np.zeros((n, 3)); xc = np.zeros((n, 3))
    for k in range(n):
        R0m = quat_to_R(*Qu[k]) @ Rzm90
        R0r = B.build_r0(xdd[k] + np.array([0, 0, G]), b1d[k] / np.linalg.norm(b1d[k]))
        r0c_m, r0e_m, _ = TP.arm_fk_model(q[k], params)
        _, r0e_d, _ = TP.arm_fk_model(q_d[k], params)
        ea[k] = (R0m - R0r) @ r0e_d
        ej[k] = R0m @ (r0e_m - r0e_d)
        re[k] = P[k] + R0m @ r0e_m
        xc[k] = P[k] + R0m @ r0c_m
    tot = re - r_ed
    nr = lambda X: 1e3 * rms(np.linalg.norm(X, axis=1))  # noqa: E731
    res = dict(file=os.path.basename(path), total=nr(tot), airframe=nr(eb), attitude=nr(ea), joints=nr(ej),
               closure_mm=float(1e3 * np.abs(tot - (eb + ea + ej)).max()))
    # planner-reported EE error over the same window, for the cross-check
    e = ee[(ee[:, 0] >= t0) & (ee[:, 0] <= t1)]
    res["planner_ee_rms"] = nr(e[:, 1:4] - e[:, 4:7])
    m = (t - t0 > 3) & (t1 - t > 3)
    rad = x_b[:, :2] - x_b[m][:, :2].mean(0)
    rad /= np.linalg.norm(rad, axis=1)[:, None]
    tan = np.column_stack([-rad[:, 1], rad[:, 0]])
    for nm, x in (("radial", np.sum(eb[:, :2] * rad, 1)), ("along", np.sum(eb[:, :2] * tan, 1))):
        y = x[m] - x[m].mean(); f, Pp = periodogram(y, fs=FS, window="hann")
        res[f"sweep_{nm}_pct"] = float(100 * Pp[(f >= 0.13) & (f < 0.21)].sum() / Pp[f > 0].sum())
        res[f"{nm}_pp_mm"] = float(1e3 * np.ptp(y)); res[f"{nm}_mean_mm"] = float(1e3 * x[m].mean())
    ec = (xc - x_cd)[m]
    f, Px = welch(ec[:, 0] - ec[:, 0].mean(), fs=FS, nperseg=800)
    _, Py = welch(ec[:, 1] - ec[:, 1].mean(), fs=FS, nperseg=800)
    Pc = Px + Py; df = f[1] - f[0]
    band = lambda a, b: float(1e3 * math.sqrt(Pc[(f >= a) & (f < b)].sum() * df))  # noqa: E731
    res.update(com_rms=float(1e3 * math.sqrt(Pc.sum() * df)), com_lt01=band(0, 0.1), com_b01_06=band(0.1, 0.6),
               com_gt06=band(0.6, 50), com_peak_hz=float(f[np.argmax(Pc[1:]) + 1]))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    out = {}
    for spec in a.runs:
        lab, path = spec.split("=", 1)
        r = analyse(path); out[lab] = r
        print(f"{lab:16s} EE {r['total']:5.1f} mm rms (planner {r['planner_ee_rms']:5.1f}) = airframe {r['airframe']:5.1f}"
              f" + attitude {r['attitude']:4.1f} + joints {r['joints']:4.1f}  (closure {r['closure_mm']:.1e} mm)")
        print(f"{'':16s} sweep band 0.13-0.21 Hz: along {r['sweep_along_pct']:3.0f} % ({r['along_pp_mm']:4.0f} mm p-p)"
              f" | radial {r['sweep_radial_pct']:3.0f} % ({r['radial_pp_mm']:4.0f} mm p-p, mean {r['radial_mean_mm']:+.0f})")
        print(f"{'':16s} CoM xy detrended {r['com_rms']:5.1f} | <0.1 Hz {r['com_lt01']:5.1f} | 0.1-0.6 Hz {r['com_b01_06']:5.1f}"
              f" | >0.6 Hz {r['com_gt06']:4.1f} | peak {r['com_peak_hz']:.3f} Hz")
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
