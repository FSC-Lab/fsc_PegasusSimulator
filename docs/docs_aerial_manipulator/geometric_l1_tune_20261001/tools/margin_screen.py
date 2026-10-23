#!/usr/bin/env python3
"""margin_screen.py -- re-screen already-evaluated CMA candidates for transport-delay margin
(2026-10-01). The first search held candidates to the shared 28 ms bar; its optimum (F1)
then diverged in Isaac in a growing 1.53 Hz attitude mode, which the bench reproduces at
30 ms (the hardware set: at 60 ms). This flies the top-N distinct logged candidates at 36
and 44 ms and lists those that complete, ranked by their logged A-run EE error.

    OMP_NUM_THREADS=1 /usr/bin/python3 margin_screen.py --top 60
"""
import argparse
import json
import multiprocessing as mp
import os

import numpy as np

import geo_bench as GB
from geo_finalists import distinct

HERE = os.path.dirname(os.path.abspath(__file__))
AN = os.path.join(HERE, "..", "analysis")


def job(a):
    i, p, d = a
    return i, d, GB.run(p, delay_ms=d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="cma_geo_log.jsonl")
    ap.add_argument("--top", type=int, default=60)
    ap.add_argument("--delays", default="36,44")
    ap.add_argument("--jobs", type=int, default=30)
    ap.add_argument("--out", default="margin_screen.json")
    a = ap.parse_args()
    L = [json.loads(l) for l in open(os.path.join(AN, a.log))]
    L = [d for d in L if d["R"]["A"]["verdict"] == "completed"]
    L.sort(key=lambda d: d["R"]["A"]["ee_rms"])
    C = distinct(L, a.top, tol=0.08)
    D = [float(x) for x in a.delays.split(",")]
    tasks = [(i, d["p"], dl) for i, d in enumerate(C) for dl in D]
    R = {}
    with mp.get_context("fork").Pool(a.jobs) as pool:
        for i, dl, r in pool.imap_unordered(job, tasks):
            R.setdefault(i, {})[dl] = r
    rows = []
    for i, d in enumerate(C):
        ok = [dl for dl in D if R[i][dl]["verdict"] == "completed"]
        rows.append(dict(p=d["p"], ee_A=d["R"]["A"]["ee_rms"] * 1e3, ok=ok,
                         ee_at={str(dl): (R[i][dl]["ee_rms"] * 1e3 if R[i][dl]["verdict"] == "completed" else None)
                                for dl in D}))
    for r in rows:
        if max(D) in r["ok"]:
            print(f"A {r['ee_A']:6.1f} mm | ok {r['ok']} | " + " ".join(f"{k}={r['p'][k]:.3g}" for k in GB.GAIN_KEYS))
    print(f"{sum(max(D) in r['ok'] for r in rows)} of {len(rows)} complete at {max(D):.0f} ms")
    json.dump(rows, open(os.path.join(AN, a.out), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
