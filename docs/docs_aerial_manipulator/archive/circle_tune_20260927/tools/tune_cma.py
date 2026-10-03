#!/usr/bin/env python3
"""CMA-ES search over the whole-body 4-D law's gains on the circle bench (2026-09-27).

All four gain groups JOINTLY (the 2026-09-26 study moved one group at a time):
  translational k_x, k_v | orientational k_R, k_w, M_r_d scale |
  arm channel K_y, D_y, K_psi, D_psi | L1 omega_c_t, omega_c_r, omega_c_q, omega_x
searched in LOG space inside hard bounds.

Each candidate is flown FOUR times on circle_bench (realistic feedback, common
random numbers):
  A  r=0.50 m circle, mirror plant, 16 ms            -- performance
  B  r=0.75 m circle (0.2 m/s), mirror plant, 16 ms  -- performance
  C  r=0.75 m circle, ROBUSTNESS plant (the _sim_robustness stress: +17.6 % kf
     belief, mass/inertia x1.10, CoM 10/10/5 mm, arm x1.05)  -- must stay sane
  D  r=0.75 m circle, mirror plant, 28 ms transport delay (x1.75) -- margin
Cost (mm): mean(EE rms A,B) + 0.25 mean(EE peak A,B)
         + 0.15 EE rms C + max(0, entry peak C - 0.55 m) x 200
         + effort: 10 x max(0, d_omega/d_omega_ship - 1.3) + 10 x max(0, d_tau/d_tau_ship - 1.3)
         + 1000 per abort (any run), + 100 x tilt excess over 10 deg (A,B,D), + 50 x sat %.
Log: ../analysis/cma_log.jsonl (one line per evaluation, resumable).

    OMP_NUM_THREADS=1 /usr/bin/python3 tune_cma.py --gens 40 --jobs 32
"""
import argparse
import json
import multiprocessing as mp
import os
import time

import numpy as np

import circle_bench as CB

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "..", "analysis", "cma_log.jsonl")
PERF = ["r050", "r075"]          # streams for A and B (performance)
STRESS = "r075"                  # stream for C (robustness plant) and D (28 ms)
DELAY_D = [28.0]                 # the margin run's transport delay [ms] (--delay-d)
ISAAC_GATE = [False]             # stage 4: run E = robustness plant under the ISAAC clock (RTF 0.48)
TAGS = ["ABCD"]
RELAXED = [False]
SPACE = [  # name, lower, upper
    ("k_x", 10.0, 100.0), ("k_v", 5.0, 50.0),
    ("k_R", 0.8, 6.0), ("k_w", 0.3, 2.5), ("mrd_s", 0.6, 1.8),
    ("ky", 10.0, 300.0), ("dy", 4.0, 80.0), ("ky_psi", 0.08, 2.0), ("dy_psi", 0.08, 2.0),
    ("omega_c_t", 0.5, 10.0), ("omega_c_r", 0.15, 3.0), ("omega_c_q", 0.15, 3.0), ("omega_x", 0.08, 2.0),
]
NAMES = [s[0] for s in SPACE]
LO = np.log([s[1] for s in SPACE]); HI = np.log([s[2] for s in SPACE])
REF = {}   # shipped effort, filled at start


def to_params(z):
    x = np.exp(np.clip(z, LO, HI))
    p = dict(CB.BASE); p.update({n: float(v) for n, v in zip(NAMES, x)})
    return p


def run_one(args):
    tag, p = args
    if tag == "A":
        return tag, CB.simulate(p, CB.stream(PERF[0]))
    if tag == "B":
        return tag, CB.simulate(p, CB.stream(PERF[1]))
    if tag == "C":
        return tag, CB.simulate(p, CB.stream(STRESS), profile="robustness")
    if tag == "D":
        return tag, CB.simulate(p, CB.stream(STRESS), delay_ms=DELAY_D[0])
    if tag == "E":
        return tag, CB.simulate(p, CB.stream(STRESS), profile="robustness", rtf=0.48, t_settle=20.0)


def cost(R):
    J = 0.0; notes = []
    for t in TAGS[0]:
        if R[t]["verdict"] != "completed":
            J += 1000.0; notes.append(f"{t}:ABORT")
    if "E" in TAGS[0] and R["E"]["verdict"] == "completed":
        E = R["E"]
        if RELAXED[0]:
            # stage 5: the flight must COMPLETE like the shipped law's does --
            # hover (the 5 cm start gate), entry, circle error and tilt no worse
            # than shipped (+10 %), attitude bounded (no sustained ringing)
            J += 5.0 * 1e3 * max(0.0, E["hold_com_rms"] - 1.10 * REF["E_hold"])
            J += 3.0 * 1e3 * max(0.0, E["ee_rms"] - 1.10 * REF["E_ee"])
            J += 1.0 * 1e3 * max(0.0, E["entry_pk"] - 1.10 * REF["E_entry"])
            J += 100.0 * max(0.0, E["tilt_pk"] - 6.0)
            J += 2000.0 * max(0.0, E["eR_mean"] - 0.13)
        else:
            # stage 4: no worse than the shipped law there (+10 %)
            J += 3.0 * 1e3 * max(0.0, E["ee_rms"] - 1.10 * REF["E_ee"])
            J += 2000.0 * max(0.0, E["eR_mean"] - 1.10 * REF["E_eR"])
        notes.append(f"E {E['ee_rms']*1e3:.0f} mm hold {E['hold_com_rms']*1e3:.0f} |eR| {E['eR_mean']:.3f}")
    if any(R[t]["verdict"] != "completed" for t in "AB"):
        return J + 500.0, notes
    J += 1e3 * (0.5 * (R["A"]["ee_rms"] + R["B"]["ee_rms"]) + 0.25 * 0.5 * (R["A"]["ee_pk"] + R["B"]["ee_pk"]))
    for t in "ABD":
        if R[t]["verdict"] == "completed":
            J += 100.0 * max(0.0, R[t]["tilt_pk"] - 10.0) + 50.0 * R[t]["sat_pct"]
    if R["C"]["verdict"] == "completed":
        J += 0.15 * 1e3 * R["C"]["ee_rms"] + 200.0 * max(0.0, R["C"]["entry_pk"] - 0.55)
    # HOVER QUALITY (stage 2): the pre-run hold must not get worse than the shipped
    # law's -- stage 1 found circle gains with zeta ~0.33 that rang in hover.
    for t in "AB":
        J += 1500.0 * max(0.0, R[t]["hold_eR"] - 1.15 * REF["hold_eR"])
        J += 1e3 * max(0.0, R[t]["hold_com_rms"] - 1.15 * REF["hold_com"])
    for t in "ABD":
        if R[t]["verdict"] == "completed":
            J += 100.0 * max(0.0, R[t]["tilt_pk"] - 6.0)
    dwr = 0.5 * (R["A"]["dw_rms"] + R["B"]["dw_rms"]) / REF["dw"]
    dtr = 0.5 * (R["A"]["dtau_rms"] + R["B"]["dtau_rms"]) / REF["dtau"]
    J += 10.0 * max(0.0, dwr - 1.3) + 10.0 * max(0.0, dtr - 1.3)
    notes.append(f"dw x{dwr:.2f} dtau x{dtr:.2f}")
    return J, notes


def evaluate(pool, Z, gen):
    tasks = []
    for i, z in enumerate(Z):
        p = to_params(z)
        for t in TAGS[0]:
            tasks.append((i, t, p))
    res = {}
    for (i, t, p), (tag, r) in zip(tasks, pool.imap(run_one, [(t, p) for i, t, p in tasks])):
        res.setdefault(i, {})[t] = r
    out = []
    with open(LOG, "a") as f:
        for i, z in enumerate(Z):
            J, notes = cost(res[i])
            rec = {"gen": gen, "J": J, "notes": notes, "z": list(map(float, z)),
                   "p": {n: to_params(z)[n] for n in NAMES},
                   "R": {t: {k: v for k, v in res[i][t].items() if k not in ("H", "Hq", "Hqd")} for t in TAGS[0]}}
            f.write(json.dumps(rec) + "\n")
            out.append(J)
    return np.array(out), res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gens", type=int, default=40)
    ap.add_argument("--jobs", type=int, default=32)
    ap.add_argument("--lam", type=int, default=12)
    ap.add_argument("--sigma", type=float, default=0.35)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--perf", default="r050,r075", help="two streams for the performance runs A,B")
    ap.add_argument("--stress", default="r075")
    ap.add_argument("--init", default="", help="json with a 'best' gain dict to start from")
    ap.add_argument("--log", default="cma_log.jsonl")
    ap.add_argument("--best-out", default="cma_best.json")
    ap.add_argument("--delay-d", type=float, default=28.0)
    ap.add_argument("--bound", action="append", default=[], help="name=lo:hi (overrides the search box)")
    ap.add_argument("--isaac-gate", action="store_true", help="add run E (robustness plant, Isaac clock)")
    ap.add_argument("--relaxed", action="store_true", help="stage-5 pass definition for run E")
    a = ap.parse_args()
    global LOG
    LOG = os.path.join(HERE, "..", "analysis", a.log)
    PERF[:] = a.perf.split(","); globals()["STRESS"] = a.stress; DELAY_D[0] = a.delay_d
    if a.isaac_gate:
        TAGS[0] = "ABCDE"
    RELAXED[0] = a.relaxed
    for b in a.bound:
        nm, rng_ = b.split("="); lo, hi = map(float, rng_.split(":")); i = NAMES.index(nm)
        LO[i], HI[i] = np.log(lo), np.log(hi)
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    rng = np.random.default_rng(a.seed)
    n = len(SPACE)
    start = dict(CB.BASE)
    if a.init:
        start.update(json.load(open(a.init))["best"])
    m = np.log([start[nm] for nm in NAMES])
    lam = a.lam; mu = lam // 2
    w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1)); w /= w.sum(); mueff = 1 / np.sum(w ** 2)
    cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n); cs = (mueff + 2) / (n + mueff + 5)
    c1 = 2 / ((n + 1.3) ** 2 + mueff); cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0, np.sqrt((mueff - 1) / (n + 1)) - 1) + cs
    chiN = np.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n ** 2))
    pc = np.zeros(n); ps = np.zeros(n); Cm = np.eye(n); sigma = a.sigma
    with mp.get_context("fork").Pool(a.jobs) as pool:
        # the shipped law: reference effort and cost
        # the SHIPPED law sets every reference level (effort, hover quality)
        ship = np.log([CB.BASE[nm] for nm in NAMES])
        Rs = {t: r for t, r in pool.imap(run_one, [(t, to_params(ship)) for t in TAGS[0]])}
        REF["dw"] = 0.5 * (Rs["A"]["dw_rms"] + Rs["B"]["dw_rms"])
        REF["dtau"] = 0.5 * (Rs["A"]["dtau_rms"] + Rs["B"]["dtau_rms"])
        REF["hold_eR"] = 0.5 * (Rs["A"]["hold_eR"] + Rs["B"]["hold_eR"])
        REF["hold_com"] = 0.5 * (Rs["A"]["hold_com_rms"] + Rs["B"]["hold_com_rms"])
        if "E" in Rs:
            REF["E_ee"] = Rs["E"]["ee_rms"]; REF["E_eR"] = Rs["E"]["eR_mean"]
            REF["E_hold"] = Rs["E"]["hold_com_rms"]; REF["E_entry"] = Rs["E"]["entry_pk"]
        Js, _ = cost(Rs)
        print(f"[cma] shipped J = {Js:.2f} (A {Rs['A']['ee_rms']*1e3:.1f} mm, B {Rs['B']['ee_rms']*1e3:.1f} mm, "
              f"hold |eR| {REF['hold_eR']:.4f})", flush=True)
        J0, R0 = evaluate(pool, [m], -1)
        J0 = float(J0[0])
        print(f"[cma] start J = {J0:.2f}  (A {R0[0]['A']['ee_rms']*1e3:.1f} mm, B {R0[0]['B']['ee_rms']*1e3:.1f} mm)", flush=True)
        best = (J0, m.copy())
        for g in range(a.gens):
            t0 = time.time()
            B, D = None, None
            evals, Bm = np.linalg.eigh(Cm); evals = np.maximum(evals, 1e-12); Dm = np.sqrt(evals)
            Zs = rng.standard_normal((lam, n))
            Y = Zs @ np.diag(Dm) @ Bm.T
            X = m + sigma * Y
            X = np.clip(X, LO, HI)
            Y = (X - m) / sigma
            J, _ = evaluate(pool, X, g)
            idx = np.argsort(J)
            if J[idx[0]] < best[0]:
                best = (J[idx[0]], X[idx[0]].copy())
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
            print(f"[cma] gen {g:2d} best-of-gen {J[idx[0]]:.2f} median {np.median(J):.2f} | overall best {best[0]:.2f} "
                  f"sigma {sigma:.3f} [{time.time()-t0:.0f} s] | " +
                  " ".join(f"{k}={pb[k]:.3g}" for k in NAMES), flush=True)
    json.dump({"J0": J0, "J_shipped": Js, "REF": REF, "best_J": best[0], "best": to_params(best[1])},
              open(os.path.join(HERE, "..", "analysis", a.best_out), "w"), indent=1)


if __name__ == "__main__":
    main()
