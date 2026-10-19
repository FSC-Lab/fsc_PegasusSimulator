#!/usr/bin/env python3
"""2026-10-17 push-profile campaign: the CRUISE push against the 10-16 min-snap push, one line
per run, matched case by case.

    PYTHONNOUSERSITE=1 /usr/bin/python3 compare.py [--csv out.csv]

Columns: the 10-16 scorer (box_pushing_campaign_20261016/tools/table.py -- outcome, the
impedance residual over the SLIDE, the release peak, tilt, q2 / q3 extremes, the box's
travel), plus, over the push's MOVE (after the 1.5 s settle, to the push's end or the trip):
  stop%    the share of the time the box stood still (|v| < 3 mm/s) after it first broke loose
  restarts how many times it started again after standing still >= 0.1 s
  vref/vbox the reference's peak speed / the box's peak speed [mm/s]
  lagpk    the planner's box-lag reading's peak (push_pull/arrival_error [11]) [mm]
  trip     guard trip time after the Push press [s] and the guard's name
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "..", "..")
NEW = os.path.join(HERE, "..", "runs")
OLD = os.path.join(A, "box_pushing_campaign_20261016", "runs")
sys.path.insert(0, os.path.join(A, "box_pushing_campaign_20261016", "tools"))
import table as TB        # noqa: E402
from pl_metrics import mark  # noqa: E402

P = np.array([0.0, -1.0, 0.0])    # the push, world -y
# case: (min-snap run(s) of 10-16, cruise run(s) of 10-17)
CASES = [
    ("normal 500 g", ["s3_P_1", "s3_P_2", "s3_P_3"], ["c_nom_1", "c_nom_2", "c_nom_3"]),
    ("normal 500 g, min-snap flown 10-17 (same-day control)", [], ["m_nom_1", "m_nom_2"]),
    # the worst cases below were planned but NOT flown: the campaign stopped after
    # c_nom_3 crashed (README); kept so a re-flight of the cruise scores against them
    ("sticky box (mu_s 0.5 / mu_k 0.29)", ["g_stick"], ["c_stick"]),
    ("600 g", ["g_m600"], ["c_m600"]),
    ("jammed after 0.2 m", ["h_stop", "i_stop"], ["c_stop"]),
    ("1 kg, mu 0.6", ["h_heavy", "h_heavy2"], ["c_heavy"]),
    ("ABORT button 10 s in", ["h_abort", "i_abort"], ["c_abort"]),
]


def use(runs):
    TB.RUNS = TB.TT.RUNS = TB.S.RUNS = runs


def extra(runs, tag):
    z = np.load(os.path.join(runs, tag + ".npz"), allow_pickle=True)
    ps = mark(z, "push:start")
    pe = mark(z, "push:end")
    trip, why = None, ""
    for e in z["events"]:
        s = str(e)
        t = float(s[1:s.index("s]")])
        if ps is not None and t >= ps and ("GUARD" in s or "ABORTING" in s or "INJECT" in s):
            trip = t - ps
            why = "inject" if "INJECT" in s else s.split("SAFETY GUARD:")[-1].split(")")[0].split("--")[0].strip()[:26]
            break
    t0 = ps + 1.5
    t1 = min(x for x in (pe, (ps + trip) if trip is not None else None) if x is not None) if (pe or trip) else ps + 25
    b, rd, ar = z["box"], z["refd"], z["arrival"]
    tg = np.arange(t0, t1, 0.02)
    y = np.interp(tg, b[:, 0], b[:, 1:4] @ P)
    vb = np.gradient(np.convolve(y, np.ones(5) / 5, mode="same"), tg)
    vr = np.interp(tg, rd[:, 0], rd[:, 1:4] @ P)
    ok = np.arange(len(tg)) > max(2, np.argmax(np.abs(y - y[0]) > 3e-3))
    ok &= tg < t1 - 0.1
    stopped = ok & (np.abs(vb) < 3e-3)
    # restarts: a run of >= 5 samples (0.1 s) standing still, then moving
    restarts, run = 0, 0
    for i in range(len(tg)):
        if not ok[i]:
            continue
        if stopped[i]:
            run += 1
        else:
            if run >= 5:
                restarts += 1
            run = 0
    m = (ar[:, 0] >= ps) & (ar[:, 0] <= (ps + trip if trip is not None else t1))
    lag = np.abs(ar[m, 12]).max() * 1e3 if m.any() and ar.shape[1] > 13 else np.nan
    return dict(stop=100 * stopped.sum() / max(ok.sum(), 1), restarts=restarts,
                vref=np.abs(vr).max() * 1e3, vbox=np.abs(vb[ok]).max() * 1e3 if ok.any() else np.nan,
                lagpk=lag, trip=trip, why=why)


def row(runs, tag):
    if not os.path.isfile(os.path.join(runs, tag + ".npz")):
        return None
    use(runs)
    r = TB.one(tag)
    r.update(extra(runs, tag))
    return r


def fmt(r):
    if r is None:
        return "(not flown)"
    trip = f"{r['trip']:5.1f} s {r['why']}" if r["trip"] is not None else "-"
    sl = f"{r['slide']:5.2f}" if r["ok"] else "    -"
    return (f"{r['tag']:9s} {'y' if r['ok'] else 'N'} {r['trav']:4.0f} | stop {r['stop']:4.1f}% rs {r['restarts']:2d}"
            f" | v {r['vref']:4.1f}/{r['vbox']:5.1f} | lag {r['lagpk']:5.1f} | e_imp {sl} | tilt {r['tilt']:4.1f}"
            f" q2<= {r['q2max']:4.1f} q3 {r['q3min']:4.1f}..{r['q3max']:4.1f} tq3 {r['tq3']:.1f} | {trip}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv")
    a = ap.parse_args()
    out = []
    for name, old, new in CASES:
        print(f"== {name}")
        for lab0, runs, tags in (("min-snap 24 s", OLD, old), ("cruise 20 s  ", NEW, new)):
            for t in tags:
                lab = lab0
                if runs == NEW:
                    use(NEW)
                    if float(TB.args_of(t).get("RAMP", "3")) == 0.0:
                        lab = "min-snap 10-17"
                r = row(runs, t)
                print(f"  {lab}  {fmt(r) if r else t + ' (not flown)'}")
                if r:
                    out.append(dict(case=name, profile=lab.strip(), **{k: v for k, v in r.items() if k not in ("change", "setting")}))
    if a.csv and out:
        import csv
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
            w.writeheader()
            w.writerows(out)


if __name__ == "__main__":
    main()
