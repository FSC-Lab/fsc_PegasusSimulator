#!/usr/bin/env python3
"""(2026-10-30) Items 6 and 9 in Isaac, against the 10-28 runs of the same scene without them
(wb_valve_turning_flight_20261028/sim/runs: nm_nom = nominal, qg_s2..s4 = drift seeds 2..4):
  wrist:   q4 at the close against q4 at the start of Ready (deg), and q4's range in the turn (deg).
           (Isaac's gripper is OPEN from the spawn on, so the jaw-opening kick of the 10-29 flights
           cannot happen here: item 6's early open is a no-op in the sim.)
  heading: the law's EE heading error asin(debug[27]) in the turn, rms / max (deg), and the new
           free-axis heading offset debug[119] in the turn, max (deg)
    PYTHONNOUSERSITE=1 /usr/bin/python3 sim_compare.py
"""
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NEW = os.path.join(HERE, "..", "sim", "runs")
OLD = os.path.join(HERE, "..", "..", "wb_valve_turning_flight_20261028", "sim", "runs")
D = 2   # dbg column = debug index + 2


def row(path, tag):
    z = np.load(os.path.join(path, tag + ".npz"), allow_pickle=True)
    M = dict(zip([str(n) for n in z["marks_name"]], z["marks_t"]))
    js, dbg = z["joints"], z["dbg"]
    q4 = np.degrees(js[:, 4])
    q4_at = lambda t: np.interp(t, js[:, 0], q4)
    dq4 = q4_at(M.get("ready:end", np.nan)) - q4_at(M.get("ready:start", np.nan))
    t0, t1 = M.get("push:start", np.nan), M.get("push:end", np.nan)
    if not np.isfinite(t1):   # an abort cuts the turn: its first ABORT event
        ab = [str(e) for e in z["events"] if "ABORT" in str(e)]
        t1 = float(ab[0].split("s]")[0].strip("[ ")) if ab else dbg[-1, 0]
    p = (dbg[:, 0] >= t0) & (dbg[:, 0] <= t1)
    wq = (js[:, 0] >= t0) & (js[:, 0] <= t1)
    he = np.degrees(np.arcsin(np.clip(dbg[p, 27 + D], -1, 1)))
    fh = np.degrees(dbg[p, 119 + D]) if dbg.shape[1] > 119 + D else np.full(p.sum(), np.nan)
    return (f"{tag:10s} q4 Ready->close {dq4:+5.1f}  q4 in the turn {q4[wq].min():+6.1f}..{q4[wq].max():+6.1f} | "
            f"heading err rms {np.sqrt(np.mean(he ** 2)):4.1f} max {np.abs(he).max():4.1f}  "
            f"free-axis heading max {np.nanmax(np.abs(fh)) if np.isfinite(fh).any() else np.nan:4.1f}")


if __name__ == "__main__":
    print("10-28, before (no open at Go To Start, no heading gate, no free-axis heading):")
    for t in ("nm_nom", "qg_s2", "qg_s3", "qg_s4"):
        if os.path.exists(os.path.join(OLD, t + ".npz")):
            print("  " + row(OLD, t))
    print("10-30, with items 6 and 9:")
    for f in sorted(os.listdir(NEW)):
        if f.endswith(".npz"):
            try:
                print("  " + row(NEW, f[:-4]))
            except Exception as exc:
                print(f"  {f[:-4]:10s} ERROR {exc}")
