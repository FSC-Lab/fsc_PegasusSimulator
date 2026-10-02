#!/usr/bin/env python3
"""delay_sweep.py -- why the paper's law flies 2/2 in Isaac at RTF 0.48 and 1/3 at
RTF 1, on the RTF-1 bench (2026-10-01).

A memoryless PD law behaves identically in plant time at any RTF; what RTF changes
is every lag around it. Isaac at RTF 1 carries more transport delay than the
bench's nominal 16 ms (PX4/DDS/EKF), so the sweep raises the bench's rotor
transport delay until each law stops completing the circle, and then tries
softer attitude modules for the paper's law (same CMA tune otherwise).

    OMP_NUM_THREADS=1 /usr/bin/python3 delay_sweep.py [--jobs 30]
Writes ../analysis/delay_sweep.json.
"""
import argparse
import json
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MODT = os.path.join(HERE, "..", "..", "modular_adaptive_20260930", "tools")
sys.path.insert(0, MODT)
import modular_bench as MB  # noqa: E402
import mapped as MP  # noqa: E402

DELAYS = [16.0, 28.0, 40.0, 52.0, 64.0]


def gains(over=None):
    p = dict(MP.TABLE2_MAPPED)
    p.update(json.load(open(os.path.join(MODT, "..", "analysis", "modular_final_gains.json")))["best"])
    p.update(over or {})
    return p


# attitude-module variants: (label, overrides). Shipped: kp_rp 81.5 / kd_rp 22.3
# (M_bar 0.095 -> wn 9.0 rad/s on a 10 rad/s rotor-lag pole, zeta 1.24).
VARIANTS = [("MOD tuned (wn 9.0)", {}),
            ("MOD att wn 6.0", dict(q_kp_rp=36.0, q_kd_rp=14.9)),
            ("MOD att wn 4.5", dict(q_kp_rp=20.25, q_kd_rp=11.2)),
            ("MOD att wn 4.5, pos wn 2.5", dict(q_kp_rp=20.25, q_kd_rp=11.2, p_kp_xy=6.25, p_kd_xy=6.0))]


def job(args):
    who, over, delay = args
    t0 = time.time()
    if who == "WB H1b":
        r = MB.run_wb(None, delay_ms=delay)
    else:
        r = MB.run(MP.make(gains(over)), delay_ms=delay)
    for k in ("H", "Hq", "Hqd"):
        r.pop(k, None)
    return who, delay, r, time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=30)
    a = ap.parse_args()
    tasks = [("WB H1b", None, d) for d in DELAYS] + [(lbl, ov, d) for lbl, ov in VARIANTS for d in DELAYS]
    res = {}
    with mp.get_context("fork").Pool(a.jobs) as pool:
        for who, d, r, dt in pool.imap_unordered(job, tasks):
            res.setdefault(who, {})[str(d)] = r
            print(f"  {who:28s} delay {d:4.0f} ms {r['verdict']:9s} [{dt:.0f}s]", flush=True)
    print()
    whos = ["WB H1b"] + [v[0] for v in VARIANTS]
    print(f"{'controller':28s} | " + " | ".join(f"{d:>4.0f} ms" for d in DELAYS) + "   (EE rms mm, or abort time)")
    for w in whos:
        cells = []
        for d in DELAYS:
            r = res[w][str(d)]
            cells.append(f"{r['ee_rms']*1e3:7.1f}" if r["verdict"] == "completed" else f"X@{r['t_end']:4.1f}s")
        print(f"{w:28s} | " + " | ".join(f"{c:>7s}" for c in cells))
    json.dump({"delays": DELAYS, "variants": dict(VARIANTS), "res": res},
              open(os.path.join(HERE, "..", "analysis", "delay_sweep.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
