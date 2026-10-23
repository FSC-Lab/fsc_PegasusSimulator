#!/usr/bin/env python3
"""finalists.py -- the robustness battery for the geometric+L1 CMA finalists (2026-10-01).

The CMA cost scores ONE noise seed at 16/28 ms and one robustness run, which rewards
candidates that sit on a stability edge. Before anything flies in Isaac every finalist
also faces:
  seeds   the comparison circle on 3 noise seeds (mirror plant, hardware-like feedback)
  isaac   Isaac's feedback (60 Hz raw mocap, no EKF lag)
  delay   transport delay 28 / 36 / 44 / 52 ms (the whole-body and modular laws: ok 28, abort 30 / 36)
  rob     the _sim_robustness stress plant
  gust    a 1.5 N world-x force for 2 s, 8 s into the lap
and is ranked on the WORST of its seeds, with margin and gust as tie-breakers.

    OMP_NUM_THREADS=1 /usr/bin/python3 geo_finalists.py --top 8 [--extra cands.json]
"""
import argparse
import json
import multiprocessing as mp
import os

import numpy as np

import geo_bench as GB

HERE = os.path.dirname(os.path.abspath(__file__))
AN = os.path.join(HERE, "..", "analysis")
TESTS = [("s0", dict(seed=0)), ("s1", dict(seed=1)), ("s2", dict(seed=2)),
         ("isaac", dict(fb=GB.FB_ISAAC)),
         ("d28", dict(delay_ms=28.0)), ("d36", dict(delay_ms=36.0)), ("d44", dict(delay_ms=44.0)),
         ("d52", dict(delay_ms=52.0)),
         ("rob", dict(profile="robustness")),
         ("gust", dict(gust=(8.0, 2.0, 1.5, 0.0)))]


def job(a):
    name, g, tag, kw = a
    return name, tag, GB.run(g, **kw)


def distinct(L, k, tol=0.15):
    """top-k by J, skipping candidates within tol (log-distance per gain) of one already kept."""
    out = []
    for d in L:
        z = np.log([d["p"][n] for n in GB.GAIN_KEYS])
        if all(np.max(np.abs(z - zz)) > tol for _, zz in out):
            out.append((d, z))
        if len(out) == k:
            break
    return [d for d, _ in out]


def summarize(R):
    s = {}
    ok = lambda r: r["verdict"] == "completed"  # noqa: E731
    seeds = [R[t] for t in ("s0", "s1", "s2")]
    s["seeds_ok"] = all(ok(r) for r in seeds)
    s["ee_worst"] = max(r["ee_rms"] for r in seeds) * 1e3 if s["seeds_ok"] else float("inf")
    s["ee_mean"] = float(np.mean([r["ee_rms"] for r in seeds])) * 1e3 if s["seeds_ok"] else float("inf")
    s["head"] = float(np.mean([r["head_rms"] for r in seeds])) if s["seeds_ok"] else float("nan")
    s["tilt"] = max(r["tilt_pk"] for r in seeds) if s["seeds_ok"] else float("nan")
    s["isaac"] = R["isaac"]["ee_rms"] * 1e3 if ok(R["isaac"]) else float("inf")
    DL = [t for t in ("d28", "d36", "d44", "d52") if t in R]
    s["margin_ms"] = max([0.0] + [float(t[1:]) for t in DL if ok(R[t])])
    s["delay_ok"] = [t for t in DL if ok(R[t])]
    s["rob"] = R["rob"]["ee_rms"] * 1e3 if ok(R["rob"]) else float("inf")
    s["gust_rise"] = R["gust"].get("gust_rise", float("nan")) * 1e3 if ok(R["gust"]) else float("inf")
    s["gust_rec"] = R["gust"].get("gust_rec", float("nan")) if ok(R["gust"]) else float("inf")
    s["dw"] = float(np.mean([r["dw_rms"] for r in seeds])) if s["seeds_ok"] else float("nan")
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="cma_geo_log.jsonl")
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--extra", default="", help="json list of {name, p}")
    ap.add_argument("--jobs", type=int, default=30)
    ap.add_argument("--out", default="finalists.json")
    a = ap.parse_args()
    L = [json.loads(l) for l in open(os.path.join(AN, a.log))]
    L.sort(key=lambda d: d["J"])
    cands = [(f"F{i+1}", d["p"]) for i, d in enumerate(distinct(L, a.top))]
    cands.insert(0, ("HW", dict(GB.GEO_HW)))
    if a.extra:
        cands += [(c["name"], c["p"]) for c in json.load(open(a.extra))]
    tasks = [(nm, g, tag, kw) for nm, g in cands for tag, kw in TESTS]
    R = {}
    with mp.get_context("fork").Pool(a.jobs) as pool:
        for nm, tag, r in pool.imap_unordered(job, tasks):
            R.setdefault(nm, {})[tag] = r
    out = []
    for nm, g in cands:
        s = summarize(R[nm])
        out.append(dict(name=nm, p=g, summary=s, runs=R[nm]))
        print(f"{nm:5} worst {s['ee_worst']:6.1f} mean {s['ee_mean']:6.1f} mm | head {s['head']:4.2f}° | tilt {s['tilt']:4.1f} | "
              f"isaac-fb {s['isaac']:6.1f} | delay ok {','.join(s['delay_ok']) or '-'} | rob {s['rob']:6.1f} | "
              f"gust +{s['gust_rise']:.0f} mm rec {s['gust_rec']:.1f} s | dω {s['dw']:.1f}", flush=True)
        print("      " + " ".join(f"{k}={g[k]:.3g}" for k in GB.GAIN_KEYS), flush=True)
    json.dump(out, open(os.path.join(AN, a.out), "w"), default=float, indent=1)


if __name__ == "__main__":
    main()
