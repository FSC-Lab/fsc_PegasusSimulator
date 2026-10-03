#!/usr/bin/env python3
"""Score an Isaac circle replay (or a hardware circle flight) for the gain study.

    /usr/bin/python3 score_run.py data/sim_<tag>.npz [--real] [--json out.json]

The EE-trajectory run is the planner's longest EXECUTING leg. EE ABSOLUTE
error = CoM error + task error, `|(x_c - x_cd) + e_y[0:3]|` from the law's own
debug array (the relative EE reference is r_ed + e_x, so r_e - r_ed = e_y + e_x;
exact to < 1 mm against the 0924 flights). Plant-time windows (RTF from
sensor_combined) like compare.py, whose loader this reuses.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compare as CP  # noqa: E402


def score(path, sim=True):
    r = CP.load(path, sim)
    legs = r["legs"]
    if not legs:
        return {"verdict": "no planner legs"}
    ta, tb, Tp = max(legs, key=lambda x: x[1] - x[0])
    st = CP.stats(r, ta, tb)
    t, D = r["t"], r["D"]
    m = (t >= ta) & (t <= tb) & r["direct"]
    Dm = D[m]
    ep = Dm[:, CP.D_XC] - Dm[:, CP.D_XCD]
    ey = Dm[:, CP.D_EY][:, :3]
    ee = np.linalg.norm(ep + ey, axis=1)
    a, b = r["t_direct"]
    aborted = bool(np.isfinite(b) and b < tb - 0.5)
    rms = lambda v: float(np.sqrt(np.mean(v ** 2)))  # noqa: E731
    out = dict(path=os.path.basename(path), rtf=r["rtf"], run=(float(ta), float(tb)), run_T_plan=Tp,
               aborted_in_run=aborted,
               ee_abs_rms_mm=rms(ee) * 1e3, ee_abs_peak_mm=float(ee.max()) * 1e3,
               com_rms_mm=st["com_err_norm_rms_mm"], com_peak_mm=st["com_err_norm_peak_mm"],
               ee_task_rms_mm=st["ee_task_norm_rms_mm"], heading_rms_deg=st["heading_err_rms_deg"],
               eR_mean=st["eR_norm_mean"], tilt_peak_deg=st["tilt_peak_deg"],
               joint_err_rms_deg=st["joint_err_rms_deg"], tau_cmd_peak_nm=st["tau_cmd_peak_nm"],
               n_sat=st["n_sat"], n_clamp=st["n_clamp"], dhat_t_mean_n=st["dhat_t_mean_n"])
    # the DIRECT-entry handover: the first 8 plant-seconds of DIRECT
    me = (t >= a) & (t <= a + 8.0) & r["direct"]
    if me.any():
        ee0 = np.linalg.norm(D[me][:, CP.D_XC] - D[me][:, CP.D_XCD], axis=1)
        out["entry_com_peak_mm"] = float(ee0.max()) * 1e3
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz", nargs="+")
    ap.add_argument("--real", action="store_true")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    res = {}
    for p in a.npz:
        s = score(p, sim=not a.real)
        res[os.path.basename(p)] = s
        if "ee_abs_rms_mm" not in s:
            print(p, s); continue
        print(f"{s['path']:34} RTF {s['rtf']:.3f} run {s['run'][1]-s['run'][0]:5.1f} s{' ABORTED' if s['aborted_in_run'] else ''} | "
              f"EEabs {s['ee_abs_rms_mm']:5.1f} (pk {s['ee_abs_peak_mm']:4.0f}) | CoM {s['com_rms_mm']:5.1f} (pk {s['com_peak_mm']:4.0f}) | "
              f"task {s['ee_task_rms_mm']:4.1f} | head {s['heading_rms_deg']:4.2f} | |eR| {s['eR_mean']:.4f} | tilt pk {s['tilt_peak_deg']:4.1f} | "
              f"sat {s['n_sat']} clamp {s['n_clamp']} | entry pk {s.get('entry_com_peak_mm', float('nan')):4.0f}")
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
