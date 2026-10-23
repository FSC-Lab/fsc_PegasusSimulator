#!/usr/bin/env python3
"""Where does the stage-2 improvement come from? Each gain group of a finalist
applied ALONE to the shipped law, and REVERTED alone from the finalist; plus
the finalist with M_r_d restored, and a fine delay scan (28/30 ms).
    /usr/bin/python3 group_ablation.py [finalist-name]  -> analysis/group_ablation.json"""
import json, multiprocessing as mp, os, sys
import circle_bench as CB
HERE = os.path.dirname(os.path.abspath(__file__)); AN = os.path.join(HERE, "..", "analysis")
G = {"translational": ["k_x", "k_v"], "orientational": ["k_R", "k_w", "mrd_s"],
     "arm channel": ["ky", "dy", "ky_psi", "dy_psi"], "L1": ["omega_c_t", "omega_c_r", "omega_c_q", "omega_x"]}
SHOW, FLOWN = "r075_L24_sync15_cw", "r050_L24_half10"
def job(a):
    name, p, s, kw = a
    return name, s, kw.get("delay_ms", 16.0), CB.simulate(p, CB.stream(s), seed=11, **kw)
def main():
    fins = json.load(open(os.path.join(AN, "finalists.json")))["cands"]
    fname = sys.argv[1] if len(sys.argv) > 1 else [k for k in fins if k.startswith("F2")][0]
    F = fins[fname]; S = dict(CB.BASE)
    cfg = {"shipped": S, fname: F}
    for g, ks in G.items():
        cfg[f"shipped + {g}"] = dict(S, **{k: F[k] for k in ks})
        cfg[f"{fname.split()[0]} - {g}"] = dict(F, **{k: S[k] for k in ks})
    cfg[f"{fname.split()[0]}, M_r_d x1.0"] = dict(F, mrd_s=1.0)
    jobs = [(n, p, s, {}) for n, p in cfg.items() for s in (SHOW, FLOWN)]
    for n in ("shipped", fname, f"{fname.split()[0]}, M_r_d x1.0"):
        jobs += [(n, cfg[n], SHOW, {"delay_ms": d}) for d in (28.0, 30.0)]
    res = {}
    with mp.get_context("fork").Pool(32) as pool:
        for n, s, d, r in pool.imap_unordered(job, jobs):
            res.setdefault(n, {})[f"{s}@{d:g}"] = {k: v for k, v in r.items() if k not in ("H", "Hq", "Hqd")}
    ee = lambda r: f"{r['ee_rms']*1e3:5.1f}" if r["verdict"] == "completed" else "ABORT"
    for n in cfg:
        R = res[n]
        extra = ""
        if f"{SHOW}@28" in R:
            extra = f" | show @28 ms {ee(R[f'{SHOW}@28'])} @30 ms {ee(R[f'{SHOW}@30'])}"
        print(f"{n:34} show {ee(R[f'{SHOW}@16'])} | flown {ee(R[f'{FLOWN}@16'])} | hold|eR| {R[f'{SHOW}@16'].get('hold_eR', float('nan')):.4f}{extra}", flush=True)
    json.dump({"cfg": cfg, "res": res}, open(os.path.join(AN, "group_ablation.json"), "w"), indent=1, default=float)
main()
