#!/usr/bin/env python3
"""margin_sweep.py -- the transport-delay margin of all three laws on ONE bench, at a
finer step than rtf_profile_20261001/tools/delay_sweep.py (16/28/40/52/64 ms), so the
geometric finalists' 28-ok/36-abort can be read against the whole-body and modular
tunes like-for-like (2026-10-01).

    OMP_NUM_THREADS=1 /usr/bin/python3 margin_sweep.py [--cands F1,F3,F6]
"""
import argparse
import json
import multiprocessing as mp
import os
import sys

import geo_bench as GB

HERE = os.path.dirname(os.path.abspath(__file__))
AN = os.path.join(HERE, "..", "analysis")
MODT = os.path.join(GB.REPO, "docs", "docs_aerial_manipulator", "modular_adaptive_20260930", "tools")
sys.path.insert(0, MODT)
import modular_bench as MB  # noqa: E402
import mapped as MP  # noqa: E402

DELAYS = [28.0, 30.0, 32.0, 34.0, 36.0, 40.0]


def job(a):
    law, name, p, d = a
    if law == "wb":
        r = MB.run_wb(None, delay_ms=d)
    elif law == "mod":
        r = MB.run(MP.make(dict(MP.TABLE2_MAPPED, **p)), delay_ms=d)
    else:
        r = GB.run(p, delay_ms=d)
    return name, d, r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cands", default="F1,F2,F3,F4,F5,F6,F7,F8")
    ap.add_argument("--jobs", type=int, default=30)
    a = ap.parse_args()
    fin = {c["name"]: c["p"] for c in json.load(open(os.path.join(AN, "finalists.json")))}
    modp = json.load(open(os.path.join(MODT, "..", "analysis", "modular_final_gains_rt1.json")))
    modp = modp.get("best", modp)
    entries = [("wb", "WB H1b", None), ("mod", "MOD rt1", modp), ("geo", "HW", fin["HW"])]
    entries += [("geo", n, fin[n]) for n in a.cands.split(",")]
    tasks = [(law, n, p, d) for law, n, p in entries for d in DELAYS]
    R = {}
    with mp.get_context("fork").Pool(a.jobs) as pool:
        for n, d, r in pool.imap_unordered(job, tasks):
            R.setdefault(n, {})[d] = r
    for _, n, _ in entries:
        row = " ".join((f"{d:.0f}:{R[n][d]['ee_rms']*1e3:5.1f}" if R[n][d]["verdict"] == "completed"
                        else f"{d:.0f}:ABORT") for d in DELAYS)
        print(f"{n:8} {row}", flush=True)
    json.dump({n: {str(d): R[n][d] for d in DELAYS} for _, n, _ in entries},
              open(os.path.join(AN, "margin_sweep.json"), "w"), default=float, indent=1)


if __name__ == "__main__":
    main()
