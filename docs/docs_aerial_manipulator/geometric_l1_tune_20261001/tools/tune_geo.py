#!/usr/bin/env python3
"""CMA-ES over the geometric+L1 law's gains on the circle bench (2026-10-01).

The SAME machinery, runs and guards as modular_adaptive_20260930/tools/tune_modular.py
(itself circle_tune_20260927/tools/tune_cma.py, the search behind the whole-body H1b),
applied to the Cai et al. law, so all three controllers of the comparison report are
tuned the same way, on the same bench, against the same plant, on the comparison task
itself (r 0.50 m, 24 s lap, q2 25 +- 15 deg at 6 s, fold 55).

Search space (log, hard bounds) -- every gain the law has: position K_p/K_v (xy, z),
attitude K_R/K_omega (roll-pitch, yaw), the L1 predictor poles A_s (velocity, rate)
and the L1 filter bandwidth omega_c. Model terms (mass, inertia, arm model, r_os trim)
and safety limits (error saturations, tilt clamp, torque/L1 clamps) stay as flown.

Each candidate flies three times (realistic feedback, common random numbers):
  A  the comparison circle, mirror plant, 16 ms           -- performance
  B  same, 28 ms transport delay (x1.75)                   -- margin, must complete
     STAGE 2 (--delay-b 44): the stage-1 optimum met 28 ms and diverged in Isaac in a
     growing 1.53 Hz attitude mode that the bench reproduces at 30 ms (the hardware set:
     at 60 ms), so stage 2 holds every candidate to 44 ms instead.
  C  same, ROBUSTNESS plant (_sim_robustness stress)       -- must stay sane
Cost (mm): EE rms A + 0.25 EE peak A + 0.15 EE rms C
         + 1000 per abort + 100 x tilt over 10 deg (A, B) + 50 x rotor-saturation %
         + effort above 1.3 x the WHOLE-BODY law's on the same run (rotor and
           joint-torque tick-to-tick rms) -- the ceiling H1b and the modular law met.

    OMP_NUM_THREADS=1 /usr/bin/python3 tune_geo.py --gens 40 --jobs 30
"""
import argparse
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np

import geo_bench as GB

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(GB.REPO, "docs", "docs_aerial_manipulator", "modular_adaptive_20260930", "tools"))
SPACE = [
    ("kp_xy", 2.0, 80.0), ("kv_xy", 2.0, 50.0), ("kp_z", 4.0, 80.0), ("kv_z", 4.0, 50.0),
    ("kr_xy", 0.5, 8.0), ("kw_xy", 0.2, 3.0), ("kr_z", 0.3, 8.0), ("kw_z", 0.1, 2.5),
    ("as_v", 0.5, 60.0), ("as_w", 0.5, 60.0), ("omega_c", 1.0, 25.0),
]
NAMES = [s[0] for s in SPACE]
LO = np.log([s[1] for s in SPACE]); HI = np.log([s[2] for s in SPACE])
REF = {"dw": 20.898742751041297, "dtau": 0.031045804584922055}   # whole-body H1b on A (modular tune's REF)
RUNS = ["A", "B", "C"]
DELAY_B = [28.0]                  # --delay-b (ms); stage 2 uses 44 (see below)


def to_params(z):
    x = np.exp(np.clip(z, LO, HI))
    return {n: float(v) for n, v in zip(NAMES, x)}


def run_one(args):
    tag, p = args
    if tag == "A":
        return tag, GB.run(p)
    if tag == "B":
        return tag, GB.run(p, delay_ms=DELAY_B[0])
    if tag == "C":
        return tag, GB.run(p, profile="robustness")


def cost(R):
    J = 0.0; notes = []
    for t in RUNS:
        if R[t]["verdict"] != "completed":
            J += 1000.0 + 50.0 * max(0.0, 40.0 - R[t]["t_end"]); notes.append(f"{t}:ABORT@{R[t]['t_end']:.0f}")
    if R["A"]["verdict"] != "completed":
        return J + 500.0, notes
    A = R["A"]
    J += 1e3 * (A["ee_rms"] + 0.25 * A["ee_pk"])
    if R["C"]["verdict"] == "completed":
        J += 0.15e3 * R["C"]["ee_rms"]
    for t in ("A", "B"):
        if R[t]["verdict"] == "completed":
            J += 100.0 * max(0.0, R[t]["tilt_pk"] - 10.0) + 50.0 * R[t]["sat_pct"]
    dwr = A["dw_rms"] / REF["dw"]; dtr = A["dtau_rms"] / REF["dtau"]
    J += 30.0 * max(0.0, dwr - 1.3) + 30.0 * max(0.0, dtr - 1.3)
    notes.append(f"A {A['ee_rms']*1e3:.0f} mm dw x{dwr:.2f} dtau x{dtr:.2f}")
    return J, notes


def evaluate(pool, Z, gen, log):
    tasks = [(i, t, to_params(z)) for i, z in enumerate(Z) for t in RUNS]
    res = {}
    for (i, t, _), (tag, r) in zip(tasks, pool.imap(run_one, [(t, p) for _, t, p in tasks])):
        res.setdefault(i, {})[t] = r
    out = []
    with open(log, "a") as f:
        for i, z in enumerate(Z):
            J, notes = cost(res[i])
            f.write(json.dumps({"gen": gen, "J": J, "notes": notes, "p": to_params(z),
                                "R": {t: res[i][t] for t in RUNS}}, default=float) + "\n")
            out.append(J)
    return np.array(out), res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gens", type=int, default=40)
    ap.add_argument("--jobs", type=int, default=30)
    ap.add_argument("--lam", type=int, default=16)
    ap.add_argument("--sigma", type=float, default=0.5)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--init", default="", help="json with a 'best' gain dict (default: a bench-stable start)")
    ap.add_argument("--log", default="cma_geo_log.jsonl")
    ap.add_argument("--best-out", default="cma_geo_best.json")
    ap.add_argument("--delay-b", type=float, default=28.0)
    ap.add_argument("--start", default="", help="json gain dict overriding the default start")
    a = ap.parse_args()
    DELAY_B[0] = a.delay_b
    log = os.path.join(HERE, "..", "analysis", a.log)
    rng = np.random.default_rng(a.seed)
    n = len(SPACE)
    # start: the hand-picked bench-stable point of explore.py (the hardware set aborts nothing
    # but sits far from the optimum; CMA from it wastes generations re-finding the coupling)
    start = dict(GB.GEO_HW, kp_xy=16.0, kv_xy=12.0, kr_xy=2.0, kw_xy=0.8, kr_z=2.0, kw_z=0.45)
    if a.init:
        start.update(json.load(open(a.init))["best"])
    if a.start:
        start.update(json.loads(a.start))
    m = np.clip(np.log([start[nm] for nm in NAMES]), LO, HI)
    lam = a.lam; mu = lam // 2
    w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1)); w /= w.sum(); mueff = 1 / np.sum(w ** 2)
    cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n); cs = (mueff + 2) / (n + mueff + 5)
    c1 = 2 / ((n + 1.3) ** 2 + mueff); cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0, np.sqrt((mueff - 1) / (n + 1)) - 1) + cs
    chiN = np.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n ** 2))
    pc = np.zeros(n); ps = np.zeros(n); Cm = np.eye(n); sigma = a.sigma
    with mp.get_context("fork").Pool(a.jobs) as pool:
        J0, R0 = evaluate(pool, [m], -1, log)
        print(f"[cma] start J = {J0[0]:.1f}  {cost(R0[0])[1]}", flush=True)
        best = (float(J0[0]), m.copy())
        for g in range(a.gens):
            t0 = time.time()
            evals, Bm = np.linalg.eigh(Cm); Dm = np.sqrt(np.maximum(evals, 1e-12))
            Y = rng.standard_normal((lam, n)) @ np.diag(Dm) @ Bm.T
            X = np.clip(m + sigma * Y, LO, HI); Y = (X - m) / sigma
            J, _ = evaluate(pool, X, g, log)
            idx = np.argsort(J)
            if J[idx[0]] < best[0]:
                best = (float(J[idx[0]]), X[idx[0]].copy())
            yw = w @ Y[idx[:mu]]
            m = m + sigma * yw
            Cinvsqrt = Bm @ np.diag(1 / Dm) @ Bm.T
            ps = (1 - cs) * ps + np.sqrt(cs * (2 - cs) * mueff) * (Cinvsqrt @ yw)
            hsig = np.linalg.norm(ps) / np.sqrt(1 - (1 - cs) ** (2 * (g + 1))) / chiN < 1.4 + 2 / (n + 1)
            pc = (1 - cc) * pc + hsig * np.sqrt(cc * (2 - cc) * mueff) * yw
            Ymu = Y[idx[:mu]]
            Cm = ((1 - c1 - cmu) * Cm + c1 * (np.outer(pc, pc) + (not hsig) * cc * (2 - cc) * Cm)
                  + cmu * (Ymu.T @ np.diag(w) @ Ymu))
            sigma *= np.exp((cs / damps) * (np.linalg.norm(ps) / chiN - 1))
            pb = to_params(best[1])
            print(f"[cma] gen {g:2d} best-of-gen {J[idx[0]]:.1f} median {np.median(J):.1f} | best {best[0]:.1f} "
                  f"sigma {sigma:.3f} [{time.time()-t0:.0f} s] | " + " ".join(f"{k}={pb[k]:.3g}" for k in NAMES),
                  flush=True)
            json.dump({"best_J": best[0], "best": to_params(best[1]), "gen": g, "REF": REF},
                      open(os.path.join(HERE, "..", "analysis", a.best_out), "w"), indent=1)


if __name__ == "__main__":
    main()
