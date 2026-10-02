#!/usr/bin/env python3
"""report_data.py -- compact JSON for the comparison report's charts.

    /usr/bin/python3 report_data.py --run WB ../data/wb_c24.npz --run MOD ../data/modular_c24.npz \
        --score ../analysis/score_isaac.json --bench ../analysis/final_eval.json --out ../analysis/report_data.json

Per Isaac run, over the EXECUTING window (marks run_start..run_end), in PLAN time
(wall seconds x the requested time scale, i.e. seconds of the 24 s lap):
  ee      t, |cur_ee - ref_ee| [mm]                     (planner's own topics)
  xy      measured EE and reference EE top-view path [m] (every ~0.05 s of plan time)
  base    t, |odom - x_b_ref| [mm]                       (driver's conversion of the same stream)
  joints  t, q2/q3 measured and referenced [deg]
"""
import argparse
import json
import math

import numpy as np


def marks(d):
    return {m.split("=")[0]: float(m.split("=")[1]) for m in d["marks"]}


def interp_rows(t, T, Y):
    return np.column_stack([np.interp(t, T, Y[:, i]) for i in range(Y.shape[1])])


def one(path, label, n=360):
    d = np.load(path, allow_pickle=True)
    mk = marks(d)
    t0, t1 = mk["run_start"], mk["run_end"]
    s = float(d["time_scale"])
    ee = d["ee"]
    m = (ee[:, 0] >= t0) & (ee[:, 0] <= t1)
    e = ee[m]
    err = np.linalg.norm(e[:, 1:4] - e[:, 4:7], axis=1) * 1e3
    tp = (e[:, 0] - t0) * s
    keep = np.linspace(0, len(tp) - 1, min(n, len(tp))).astype(int)
    out = {"label": label, "s": s,
           "ee": {"t": np.round(tp[keep], 3).tolist(), "err": np.round(err[keep], 2).tolist()},
           "xy": {"mx": np.round(e[keep, 1], 4).tolist(), "my": np.round(e[keep, 2], 4).tolist(),
                  "rx": np.round(e[keep, 4], 4).tolist(), "ry": np.round(e[keep, 5], 4).tolist()}}
    log = d["log"]; wr = d["wbref"]
    ml = (log[:, 0] >= t0) & (log[:, 0] <= t1)
    L = log[ml]
    xb = interp_rows(L[:, 0], wr[:, 0], wr[:, 30:33])   # [30..32] x_b (driver layout)
    berr = np.linalg.norm(L[:, 1:4] - xb, axis=1) * 1e3
    k2 = np.linspace(0, len(L) - 1, min(n, len(L))).astype(int)
    out["base"] = {"t": np.round((L[k2, 0] - t0) * s, 3).tolist(), "err": np.round(berr[k2], 2).tolist()}
    js = d["js"]; mj = (js[:, 0] >= t0) & (js[:, 0] <= t1); J = js[mj]
    qref = interp_rows(J[:, 0], wr[:, 0], wr[:, 22:26])  # [22..25] q_d
    k3 = np.linspace(0, len(J) - 1, min(n, len(J))).astype(int)
    out["joints"] = {"t": np.round((J[k3, 0] - t0) * s, 3).tolist(),
                     "q2": np.round(np.degrees(J[k3, 2]), 2).tolist(), "q3": np.round(np.degrees(J[k3, 3]), 2).tolist(),
                     "q2r": np.round(np.degrees(qref[k3, 1]), 2).tolist(), "q3r": np.round(np.degrees(qref[k3, 2]), 2).tolist()}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", nargs=2, action="append", metavar=("LABEL", "NPZ"), required=True)
    ap.add_argument("--score", default=None)
    ap.add_argument("--bench", default=None)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    data = {"runs": [one(p, lbl) for lbl, p in a.run]}
    if a.score:
        data["score"] = json.load(open(a.score))
    if a.bench:
        data["bench"] = json.load(open(a.bench))
    json.dump(data, open(a.out, "w"))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
