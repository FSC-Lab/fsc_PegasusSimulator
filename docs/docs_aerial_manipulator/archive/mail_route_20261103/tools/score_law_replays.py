#!/usr/bin/env python3
"""Score replay_bag_law.py runs (2026-11-03): per signal, rms(difference) / rms(recorded) between
the recorded flight and each replay, and between two replays of the SAME bag (the floor: what
the replay's own tick timing and the open-loop observer give). Raw and low-passed at --lp Hz
(zero-phase), since the attitude loop's derivative term on gyro noise is uncorrelated sample
to sample and says nothing about the law.

    score_law_replays.py RUNS_DIR [--lp 2.0]
"""
import argparse
import glob
import json
import os

import numpy as np
from scipy.signal import butter, filtfilt

NAMES = {"tau": [f"tau_j{j + 1}" for j in range(4)], "motors": [f"motor{j + 1}" for j in range(4)],
         "dbg": ["u1", "tau_body_x", "tau_body_y", "tau_body_z", "e_y_x", "e_y_y", "e_y_z", "e_y_psi",
                 "d_t_x", "d_t_y", "d_t_z", "d_r_x", "d_r_y", "d_r_z", "d_q1", "d_q2", "d_q3", "d_q4",
                 "tau_q1", "tau_q2", "tau_q3", "tau_q4"]}
SHOW = ["tau_j1", "tau_j2", "tau_j3", "tau_j4", "motor1", "motor2", "motor3", "motor4", "u1", "tau_body_x",
        "tau_body_y", "tau_body_z", "e_y_x", "e_y_y", "e_y_z", "e_y_psi", "d_t_z", "d_r_x", "d_r_y", "d_r_z", "d_q4"]


def grid(A, t):
    return np.column_stack([np.interp(t, A[:, 0], A[:, k]) for k in range(1, A.shape[1])])


def ratio(X, Y, lp, fs=250.0):
    if lp:
        b, a = butter(2, lp / (fs / 2))
        X, Y = filtfilt(b, a, X, axis=0), filtfilt(b, a, Y, axis=0)
    return np.sqrt(np.mean((X - Y) ** 2, 0)) / np.maximum(np.sqrt(np.mean(X ** 2, 0)), 1e-9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs")
    ap.add_argument("--lp", type=float, default=2.0)
    a = ap.parse_args()
    runs = {os.path.basename(p)[:-4]: np.load(p) for p in sorted(glob.glob(os.path.join(a.runs, "law_*.npz")))}
    win = {k: json.load(open(os.path.join(a.runs, k + ".json")))["window"] for k in runs}
    out = {}
    pairs = [(k, "rec", k) for k in runs]
    if "law_flown_3cm_a" in runs and "law_flown_3cm_b" in runs:
        pairs.append(("law_flown_3cm_a", "rep_vs_rep", "law_flown_3cm_b"))
    if "law_flown_3cm_a" in runs and "law_route_3cm" in runs:
        pairs.append(("law_flown_3cm_a", "rep_vs_rep", "law_route_3cm"))
    for k1, kind, k2 in pairs:
        t = np.arange(win[k1][0], win[k1][1], 1 / 250.0)
        row = {}
        for sig in ("tau", "motors", "dbg"):
            A = runs[k1]["rec_" + sig] if kind == "rec" else runs[k1]["rep_" + sig]
            B = runs[k2]["rep_" + sig]
            if len(A) < 10 or len(B) < 10:
                continue
            X, Y = grid(A, t), grid(B, t)
            for nm, r0, r1 in zip(NAMES[sig], ratio(X, Y, 0), ratio(X, Y, a.lp)):
                row[nm] = (r0, r1)
        label = f"{k1.replace('law_', '')} vs {'recorded' if kind == 'rec' else k2.replace('law_', '')}"
        out[label] = row
    print(f"rms(difference) / rms(signal) over DIRECT -- raw | low-passed {a.lp:g} Hz")
    print(f"{'':34s}" + "".join(f"{s:>13s}" for s in SHOW))
    for label, row in out.items():
        print(f"{label:34s}" + "".join(f"{row[s][0]:6.3f}|{row[s][1]:5.3f} " if s in row else f"{'-':>13s}" for s in SHOW))
    json.dump({k: {s: list(v) for s, v in r.items()} for k, r in out.items()},
              open(os.path.join(a.runs, "law_score.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
