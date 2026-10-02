#!/usr/bin/env python3
"""CMA-ES over the modular adaptive law's free parameters on the circle bench (2026-09-30).

The same machinery and guards as circle_tune_20260927/tools/tune_cma.py (the
search that produced the whole-body law's shipped H1b), applied to the
paper's law -- so both controllers are tuned the same way, on the same
bench, against the same plant. Differences, all in the competitor's favour:
it is tuned on THE comparison task itself (r 0.50 m, 24 s lap, q2 25 +- 15 deg
at 6 s, fold 55), where H1b was tuned on r 0.50/0.75 circles with slower arm
motion and only CHECKED on this one.

Search space (log, hard bounds) -- every user-defined constant the paper has,
per module: the closed-loop poles K1 = Lambda lambda1, K2 = Lambda lambda2
(per axis group), the shape of Q (q_v / q_e; q_e = 1 fixes the scale, which
varpi already sets), the leakage nu (one per module, as in Table II), the
boundary layer varpi and the adaptive gains' initial value K_hat(0). Mbar is
the nominal inertia (Remark 3's natural choice; a scale on it is redundant
with K1/K2 here), eps and zeta(0) are Table II's.

Each candidate flies three times (realistic feedback, common random numbers):
  A  the comparison circle, mirror plant, 16 ms           -- performance
  B  same, 28 ms transport delay (x1.75)                   -- margin, must complete
  C  same, ROBUSTNESS plant (_sim_robustness stress)       -- must stay sane
Cost (mm): EE rms A + 0.25 EE peak A + 0.15 EE rms C
         + 1000 per abort + 100 x tilt over 10 deg (A, B) + 50 x joint-clamp %
         + effort above 1.3 x the WHOLE-BODY law's on the same run (rotor and
           joint-torque tick-to-tick rms) -- the same ceiling H1b was held to.

    OMP_NUM_THREADS=1 /usr/bin/python3 tune_modular.py --gens 50 --jobs 20
"""
import argparse, json, multiprocessing as mp, os, time
import numpy as np
import modular_bench as MB
import circle_bench as CB
import mapped as MP

HERE = os.path.dirname(os.path.abspath(__file__))
STREAM = "r050_L24_f55_a15_p6"
SPACE = [
    ("p_kp_xy", 1.0, 30.0), ("p_kd_xy", 0.5, 15.0), ("p_kp_z", 2.0, 40.0), ("p_kd_z", 1.0, 15.0),
    ("p_q", 0.1, 300.0), ("p_qv", 0.01, 3.0), ("p_nu0", 1e-4, 30.0), ("p_nu123", 1e-2, 300.0),
    ("p_varpi", 1e-3, 3.0), ("p_k0", 1e-4, 5.0),
    ("q_kp_rp", 5.0, 150.0), ("q_kd_rp", 2.0, 30.0), ("q_kp_y", 3.0, 80.0), ("q_kd_y", 1.0, 25.0),
    ("q_q", 0.1, 300.0), ("q_qv", 0.01, 3.0), ("q_nu0", 1e-4, 50.0), ("q_nu123", 1e-2, 300.0),
    ("q_varpi", 1e-3, 3.0), ("q_k0", 1e-5, 5.0),
    ("a_kp", 10.0, 3000.0), ("a_kd", 3.0, 120.0), ("a_q", 0.1, 300.0), ("a_qv", 0.01, 3.0),
    ("a_nu0", 1e-4, 10.0), ("a_nu123", 1e-2, 300.0), ("a_varpi", 1e-3, 1.0), ("a_k0", 1e-5, 5.0),
    ("acc_hz", 0.5, 20.0),
]
NAMES = [s[0] for s in SPACE]
LO = np.log([s[1] for s in SPACE]); HI = np.log([s[2] for s in SPACE])
FIXED = dict()
REF = {}
RUNS = ["A", "B", "C"]          # --runs
ARM_DELAY_D = 32.0             # run D: arm torque transport delay [ms] (hardware clock)


SUBSET = [None]   # --dims: names searched; None = all
BASEP = {}


def to_params(z):
    names = SUBSET[0] or NAMES
    lo = np.array([LO[NAMES.index(n)] for n in names]); hi = np.array([HI[NAMES.index(n)] for n in names])
    x = np.exp(np.clip(z, lo, hi))
    p = dict(MP.TABLE2_MAPPED); p.update(FIXED); p.update(BASEP); p.update({n: float(v) for n, v in zip(names, x)})
    return p


def run_one(args):
    tag, p = args
    cfg = MP.make(p)
    if tag == "A":
        return tag, MB.run(cfg, STREAM)
    if tag == "B":
        return tag, MB.run(cfg, STREAM, delay_ms=28.0)
    if tag == "C":
        return tag, MB.run(cfg, STREAM, profile="robustness")
    if tag == "D":   # arm torque-path delay margin, Isaac's arm armature (0.020)
        return tag, MB.run(cfg, STREAM, arm_delay_ms=ARM_DELAY_D, plant_armature=[0.02] * 4)
    if tag == "E":   # Isaac-like: wall clock RTF 0.48, 24 ms arm delay, 0.020 armature
        return tag, MB.run(cfg, STREAM, rtf=0.48, t_settle=20.0, arm_delay_ms=24.0, plant_armature=[0.02] * 4)


def cost(R):
    J = 0.0; notes = []
    for t in RUNS:
        if R[t]["verdict"] != "completed":
            J += 1000.0 + 50.0 * max(0.0, 40.0 - R[t]["t_end"]); notes.append(f"{t}:ABORT@{R[t]['t_end']:.0f}")
    if R["A"]["verdict"] != "completed":
        return J + 500.0, notes
    A = R["A"]
    J += 1e3 * (A["ee_rms"] + 0.25 * A["ee_pk"])
    if "C" in RUNS and R["C"]["verdict"] == "completed":
        J += 0.15e3 * R["C"]["ee_rms"]
    if "E" in RUNS and R["E"]["verdict"] == "completed":
        J += 0.25e3 * R["E"]["ee_rms"]
    for t in [x for x in RUNS if x in "ABDE"]:
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
            f.write(json.dumps({"gen": gen, "J": J, "notes": notes, "p": {n: to_params(z)[n] for n in NAMES},
                                "R": {t: res[i][t] for t in RUNS}}, default=float) + "\n")
            out.append(J)
    return np.array(out), res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gens", type=int, default=50)
    ap.add_argument("--jobs", type=int, default=20)
    ap.add_argument("--lam", type=int, default=16)
    ap.add_argument("--sigma", type=float, default=0.5)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--init", default="")
    ap.add_argument("--log", default="cma_modular_log.jsonl")
    ap.add_argument("--best-out", default="cma_modular_best.json")
    ap.add_argument("--runs", default="ABC")
    ap.add_argument("--dims", default="", help="comma list of names to search (default all)")
    a = ap.parse_args()
    RUNS[:] = list(a.runs)
    log = os.path.join(HERE, "..", "analysis", a.log)
    rng = np.random.default_rng(a.seed)
    n = len(SPACE)
    start = dict(MP.TABLE2_MAPPED, p_q=1.0, q_q=1.0, a_q=1.0, acc_hz=10.0)
    st1 = json.load(open(os.path.join(HERE, "..", "analysis", "cma_modular_stage1_best.json")))["best"]
    start.update(st1)
    for m_ in ("p", "q", "a"):
        start[f"{m_}_nu0"] = start[f"{m_}_nu"]; start[f"{m_}_nu123"] = start[f"{m_}_nu"]
    if a.init:
        start.update(json.load(open(a.init))["best"])
    BASEP.update({k: v for k, v in start.items()})
    if a.dims:
        SUBSET[0] = a.dims.split(",")
    names = SUBSET[0] or NAMES
    n = len(names)
    LO_s = np.array([LO[NAMES.index(nm)] for nm in names]); HI_s = np.array([HI[NAMES.index(nm)] for nm in names])
    m = np.clip(np.log([start[nm] for nm in names]), LO_s, HI_s)
    lam = a.lam; mu = lam // 2
    w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1)); w /= w.sum(); mueff = 1 / np.sum(w ** 2)
    cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n); cs = (mueff + 2) / (n + mueff + 5)
    c1 = 2 / ((n + 1.3) ** 2 + mueff); cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0, np.sqrt((mueff - 1) / (n + 1)) - 1) + cs
    chiN = np.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n ** 2))
    pc = np.zeros(n); ps = np.zeros(n); Cm = np.eye(n); sigma = a.sigma
    with mp.get_context("fork").Pool(a.jobs) as pool:
        wb = MB.run_wb(None, STREAM)          # the whole-body law's effort on the same run = the ceiling
        REF["dw"], REF["dtau"] = wb["dw_rms"], wb["dtau_rms"]
        print(f"[cma] whole-body H1b on A: EE {wb['ee_rms']*1e3:.1f} mm, dw {REF['dw']:.2f}, dtau {REF['dtau']*1e3:.1f} m",
              flush=True)
        J0, R0 = evaluate(pool, [m], -1, log)
        print(f"[cma] start J = {J0[0]:.1f}", flush=True)
        best = (float(J0[0]), m.copy())
        for g in range(a.gens):
            t0 = time.time()
            evals, Bm = np.linalg.eigh(Cm); Dm = np.sqrt(np.maximum(evals, 1e-12))
            Y = rng.standard_normal((lam, n)) @ np.diag(Dm) @ Bm.T
            X = np.clip(m + sigma * Y, LO_s, HI_s); Y = (X - m) / sigma
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
                  f"sigma {sigma:.3f} [{time.time()-t0:.0f} s] | " + " ".join(f"{k}={pb[k]:.3g}" for k in names),
                  flush=True)
            json.dump({"best_J": best[0], "best": to_params(best[1]), "gen": g, "REF": REF},
                      open(os.path.join(HERE, "..", "analysis", a.best_out), "w"), indent=1)


if __name__ == "__main__":
    main()
