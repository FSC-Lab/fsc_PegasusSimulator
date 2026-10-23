#!/usr/bin/env python3
"""Finalist gate for the CMA-ES stage-2 candidates (2026-09-27).

Top-K fully-passing, mutually distinct candidates of analysis/cma2_log.jsonl +
the shipped law, each through:
  seeds    3 fresh noise seeds on the showcase circle and the flown circle
  delay    showcase at 16 / 24 / 32 / 36 ms transport delay
  gust     showcase + a 3 N world-x step force for 3 s, 10 s into the run
  robust   the robustness plant (the _sim_robustness stress) at 16 and 24 ms
    /usr/bin/python3 finalists.py [--k 5]  -> analysis/finalists.json
"""
import argparse
import json
import multiprocessing as mp
import os

import numpy as np

import circle_bench as CB

HERE = os.path.dirname(os.path.abspath(__file__))
AN = os.path.join(HERE, "..", "analysis")
SHOW, FLOWN = "r075_L24_sync15_cw", "r050_L24_half10"
KEYS = ["k_x", "k_v", "k_R", "k_w", "mrd_s", "ky", "dy", "ky_psi", "dy_psi",
        "omega_c_t", "omega_c_r", "omega_c_q", "omega_x"]


def tests():
    T = []
    for sd in (11, 12, 13):
        T += [(f"seed{sd}_show", dict(s=SHOW, seed=sd)), (f"seed{sd}_flown", dict(s=FLOWN, seed=sd))]
    for d in (16.0, 24.0, 28.0, 30.0, 32.0, 36.0):
        T.append((f"delay{int(d)}", dict(s=SHOW, delay_ms=d)))
    T.append(("gust", dict(s=SHOW, gust=(10.0, 3.0, 3.0, 0.0))))
    T += [("robust16", dict(s=SHOW, profile="robustness")), ("robust24", dict(s=SHOW, profile="robustness", delay_ms=24.0))]
    return T


def job(a):
    name, p, tn, kw = a
    kw = dict(kw); s = kw.pop("s")
    return name, tn, CB.simulate(p, CB.stream(s), **kw)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--k", type=int, default=5)
    ap.add_argument("--log", default="cma2_log.jsonl"); ap.add_argument("--jobs", type=int, default=32)
    ap.add_argument("--out", default="finalists.json")
    a = ap.parse_args()
    recs = [json.loads(l) for l in open(os.path.join(AN, a.log))]
    ok = sorted([r for r in recs if r["gen"] >= 0 and all(r["R"][t]["verdict"] == "completed" for t in "ABCD")],
                key=lambda r: r["J"])
    fins = []
    for r in ok:   # distinct: > 10 % apart (log space) from every finalist already taken
        z = np.log([r["p"][k] for k in KEYS])
        if all(np.max(np.abs(z - np.log([f["p"][k] for k in KEYS]))) > 0.10 for f in fins):
            fins.append(r)
        if len(fins) == a.k:
            break
    cands = {"shipped": dict(CB.BASE)}
    for i, r in enumerate(fins):
        p = dict(CB.BASE); p.update(r["p"]); cands[f"F{i+1} (J {r['J']:.1f})"] = p
    jobs = [(n, p, tn, kw) for n, p in cands.items() for tn, kw in tests()]
    out = {n: {} for n in cands}
    with mp.get_context("fork").Pool(a.jobs) as pool:
        for n, tn, r in pool.imap_unordered(job, jobs):
            out[n][tn] = {k: v for k, v in r.items() if k not in ("H", "Hq", "Hqd")}
    rows = []
    for n, R in out.items():
        def ee(t):
            return R[t]["ee_rms"] * 1e3 if R[t]["verdict"] == "completed" else float("nan")
        sh = [ee(f"seed{s}_show") for s in (11, 12, 13)]; fl = [ee(f"seed{s}_flown") for s in (11, 12, 13)]
        dl = " ".join(f"{ee(f'delay{d}'):.1f}" if R[f"delay{d}"]["verdict"] == "completed" else "ABORT" for d in (16, 24, 28, 30, 32, 36))
        g = R["gust"]
        rb = " ".join(f"{ee(t):.1f}/{R[t]['entry_pk']*1e3:.0f}" if R[t]["verdict"] == "completed" else "ABORT" for t in ("robust16", "robust24"))
        line = (f"{n:14} show {np.mean(sh):5.1f}±{np.std(sh):.1f} | flown {np.mean(fl):5.1f}±{np.std(fl):.1f} | "
                f"delay 16/24/28/30/32/36: {dl} | gust +{g.get('gust_rise', float('nan'))*1e3:.0f} mm rec {g.get('gust_rec', float('nan')):.1f} s | "
                f"robust EE/entry 16,24: {rb} | hold|eR| {R['seed11_show'].get('hold_eR', float('nan')):.4f}")
        print(line, flush=True); rows.append(line)
    json.dump({"cands": cands, "res": out, "table": rows}, open(os.path.join(AN, a.out), "w"),
              indent=1, default=float)


if __name__ == "__main__":
    main()
