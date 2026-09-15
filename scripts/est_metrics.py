#!/usr/bin/env python3
"""Summarise a scripts/est_sweep.sh directory: one row per (estimator kind, predictor), mean +- sd
over seeds, from each run's stdout (solve fails, RTH stop, worst roll/yaw, the GUIDE status times)
and its per-tick --log CSV (the plant's TRUE bob force per phase -- the number that says whether
the loop is fighting the tank -- roll sd in the orbit, u_ad rms, the raw reading's and the
predictor's correlation with the truth).

    ~/envs/isaac_env/bin/python3 scripts/est_metrics.py <sweep dir> [--csv OUT.csv]

Phases of the turning mission (guide "20:wp 500 0 500 400 100 400;150:rth"): turns = from the wp
command to the loiter hand-over (the two 90 deg fly-by turns), orbit = loiter to rth, stop = rth to
the RTH STOP line. A run with no summary line is reported as MISSING (a core-dumped start).
"""
import csv
import glob
import os
import re
import sys

import numpy as np


def parse_txt(path):
    s = open(path, errors="replace").read()
    o = {}
    m = re.search(r"solve fails (\d+)/(\d+)", s); o["fails"] = int(m.group(1)) if m else None; o["ticks"] = int(m.group(2)) if m else None
    m = re.search(r"worst \|roll\| ([\d.]+) deg\s+worst \|yaw err\| ([\d.]+)", s)
    o["worst_roll"], o["worst_yaw"] = (float(m.group(1)), float(m.group(2))) if m else (None, None)
    m = re.search(r"RTH STOP t=([\d.]+): ([\d.]+) m from home", s); o["t_stop"], o["stop_m"] = (float(m.group(1)), float(m.group(2))) if m else (None, None)
    m = re.search(r"GUIDE t=([\d.]+): legs done: loitering", s); o["t_orbit"] = float(m.group(1)) if m else None
    m = re.search(r"GUIDE t=([\d.]+): 'rth'", s); o["t_rth"] = float(m.group(1)) if m else None
    m = re.search(r"GUIDE t=([\d.]+): rth: braking", s); o["t_brake"] = float(m.group(1)) if m else None
    m = re.search(r"peak bob displacement ([\d.]+) mm", s); o["bob_peak_mm"] = float(m.group(1)) if m else None
    m = re.search(r"peak moment on the airframe ([\d.]+) N\.m", s); o["tank_M_peak"] = float(m.group(1)) if m else None
    m = re.search(r"F_x: truth rms ([\d.]+) N \| STESO reading corr ([+-][\d.]+) \(lags the truth by ~(\d+) ms\) \| predictor now corr ([+-][\d.]+) err ([\d.]+) \| at \+0.25 s: pred err ([\d.]+) vs hold ([\d.]+)", s)
    if m:
        o.update(px_truth=float(m.group(1)), px_raw_corr=float(m.group(2)), px_raw_lag=float(m.group(3)), px_pred_corr=float(m.group(4)), px_pred_err=float(m.group(5)), px_pred25=float(m.group(6)), px_hold25=float(m.group(7)))
    m = re.search(r"F_y: truth rms ([\d.]+) N \| STESO reading corr ([+-][\d.]+) \(lags the truth by ~(\d+) ms\) \| predictor now corr ([+-][\d.]+) err ([\d.]+) \| at \+0.25 s: pred err ([\d.]+) vs hold ([\d.]+)", s)
    if m:
        o.update(py_truth=float(m.group(1)), py_raw_corr=float(m.group(2)), py_raw_lag=float(m.group(3)), py_pred_corr=float(m.group(4)), py_pred_err=float(m.group(5)), py_pred25=float(m.group(6)), py_hold25=float(m.group(7)))
    m = re.search(r"pitch\s+([+-]?[\d.]+)\s+deg.*hover", s)
    # hover case: the standing pitch and the sigma read are in the final lines
    m = re.search(r"hover pitch ([+-]?[\d.]+) deg", s); o["hover_pitch"] = float(m.group(1)) if m else None
    return o


def parse_csv(path, o):
    rows = list(csv.reader(open(path))); h = rows[0]
    A = np.array([[float(x) for x in r] for r in rows[1:]]); t = A[:, h.index("t")]
    col = lambda n: A[:, h.index(n)]
    rms = lambda x: float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")
    phases = {}
    if o.get("t_orbit") and o.get("t_rth"):
        # turns = the two fly-by turns; orbit = the STEADY orbit (the last 40 s before rth: the
        # entry's bank reversal is excluded); stop = the braking onset to the RTH stop
        phases["turns"] = (22.0, o["t_orbit"]); phases["orbit"] = (o["t_rth"] - 40.0, o["t_rth"])
        phases["stop"] = (o["t_brake"] or o["t_rth"], o["t_stop"] if o.get("t_stop") else t[-1])
    else:
        phases["all"] = (max(5.0, t[0] + 5.0), t[-1])
    for ph, (a, b) in phases.items():
        w = (t > a) & (t < b)
        if w.sum() < 50:
            continue
        if "Ftrue_x" in h:
            Fx, Fy = col("Ftrue_x")[w], col("Ftrue_y")[w]
            o[f"{ph}_bobF"] = rms(np.hypot(Fx, Fy)); o[f"{ph}_bobFy"] = rms(Fy); o[f"{ph}_bobFx"] = rms(Fx)
            o[f"{ph}_hits"] = int(np.sum(np.hypot(Fx, Fy) > 1.5))
        o[f"{ph}_roll_sd"] = float(col("roll_deg")[w].std()); o[f"{ph}_pitch_sd"] = float(col("pitch_deg")[w].std())
        o[f"{ph}_uad"] = rms(np.hypot(np.hypot(col("u_x")[w], col("u_y")[w]), col("u_z")[w]))
        if "ud_z" in h:
            o[f"{ph}_udz"] = rms(col("ud_z")[w])
        o[f"{ph}_sigma_hf"] = float(np.std(np.diff(col("sigma_z")[w])) / np.sqrt(2.0))
    if o.get("hover_pitch") is None and "pitch_deg" in h and t[-1] < 60:
        w = t > 20.0; o["hover_pitch"] = float(col("pitch_deg")[w].mean()); o["hover_sigma_y"] = float(col("sigma_y")[w].mean()); o["hover_sigma_sd"] = float(col("sigma_y")[w].std())
    return o


def load_dir(d):
    runs = {}
    for txt in sorted(glob.glob(os.path.join(d, "*_p*_s*.txt"))):
        name = os.path.basename(txt)[:-4]
        m = re.match(r"(.+)_p(on|off)_s(\d+)$", name)
        if not m:
            continue
        kind, pred, sd = m.group(1), m.group(2), int(m.group(3))
        o = parse_txt(txt); o["missing"] = o["fails"] is None
        csvp = txt[:-4] + ".csv"
        if os.path.exists(csvp) and not o["missing"]:
            try:
                parse_csv(csvp, o)
            except Exception as e:      # noqa: BLE001
                o["csv_error"] = str(e)
        runs.setdefault((kind, pred), []).append((sd, o))
    return runs


def ms(vals):
    v = [x for x in vals if x is not None and not (isinstance(x, float) and np.isnan(x))]
    if not v:
        return "   --   "
    return f"{np.mean(v):.3f}" + (f"±{np.std(v):.3f}" if len(v) > 1 else "")


def main():
    d = sys.argv[1]; runs = load_dir(d)
    order = ["steso", "leso", "mdob", "mdob22", "pwc", "imu-steso", "imu-leso", "imu-mdob"]
    keys = sorted(runs, key=lambda k: (order.index(k[0]) if k[0] in order else 9, k[1]))
    cfg = open(os.path.join(d, "config")).read().strip() if os.path.exists(os.path.join(d, "config")) else ""
    print(f"sweep {d}\n  {cfg}\n  controller {open(os.path.join(d, 'ctrl_commit')).read().strip() if os.path.exists(os.path.join(d, 'ctrl_commit')) else '?'}")
    turning = any("t_orbit" in o and o["t_orbit"] for rs in runs.values() for _, o in rs)
    if turning:
        cols = [("seeds", lambda rs: f"{sum(1 for _, o in rs if not o['missing'])}/{len(rs)}"),
                ("fails", lambda rs: ms([o["fails"] for _, o in rs])),
                ("stop m", lambda rs: ms([o.get("stop_m") for _, o in rs])),
                ("bob|F| turns", lambda rs: ms([o.get("turns_bobF") for _, o in rs])),
                ("bob|F| orbit", lambda rs: ms([o.get("orbit_bobF") for _, o in rs])),
                ("bob|F| stop", lambda rs: ms([o.get("stop_bobF") for _, o in rs])),
                ("hits>1.5N", lambda rs: ms([(o.get("turns_hits", 0) + o.get("orbit_hits", 0) + o.get("stop_hits", 0)) for _, o in rs])),
                ("roll sd orbit", lambda rs: ms([o.get("orbit_roll_sd") for _, o in rs])),
                ("roll sd stop", lambda rs: ms([o.get("stop_roll_sd") for _, o in rs])),
                ("u_ad orbit", lambda rs: ms([o.get("orbit_uad") for _, o in rs])),
                ("raw corr y", lambda rs: ms([o.get("py_raw_corr") for _, o in rs])),
                ("pred corr y", lambda rs: ms([o.get("py_pred_corr") for _, o in rs])),
                ("pred25 y", lambda rs: ms([o.get("py_pred25") for _, o in rs])),
                ("worst roll", lambda rs: ms([o.get("worst_roll") for _, o in rs]))]
    else:
        cols = [("seeds", lambda rs: f"{sum(1 for _, o in rs if not o['missing'])}/{len(rs)}"),
                ("fails", lambda rs: ms([o["fails"] for _, o in rs])),
                ("hover pitch", lambda rs: ms([o.get("hover_pitch") for _, o in rs])),
                ("sigma_y read", lambda rs: ms([o.get("hover_sigma_y") for _, o in rs])),
                ("sigma_y sd", lambda rs: ms([o.get("hover_sigma_sd") for _, o in rs])),
                ("u_ad", lambda rs: ms([o.get("all_uad") for _, o in rs])),
                ("pitch sd", lambda rs: ms([o.get("all_pitch_sd") for _, o in rs])),
                ("worst roll", lambda rs: ms([o.get("worst_roll") for _, o in rs]))]
    hdr = f"{'kind':7s} {'pred':4s} " + " ".join(f"{c[0]:>14s}" for c in cols)
    print(hdr); print("-" * len(hdr))
    table = []
    for k in keys:
        rs = runs[k]
        vals = [c[1](rs) for c in cols]
        print(f"{k[0]:7s} {k[1]:4s} " + " ".join(f"{v:>14s}" for v in vals))
        table.append((k, dict(zip([c[0] for c in cols], vals))))
    missing = [(k, sd) for k, rs in runs.items() for sd, o in rs if o["missing"]]
    if missing:
        print("MISSING (no summary line -- a core-dumped start? re-run serially):", missing)
    if "--csv" in sys.argv:
        out = sys.argv[sys.argv.index("--csv") + 1]
        with open(out, "w", newline="") as f:
            wri = csv.writer(f); wri.writerow(["kind", "pred"] + [c[0] for c in cols])
            for k, row in table:
                wri.writerow([k[0], k[1]] + [row[c[0]] for c in cols])
        print("->", out)


if __name__ == "__main__":
    main()
