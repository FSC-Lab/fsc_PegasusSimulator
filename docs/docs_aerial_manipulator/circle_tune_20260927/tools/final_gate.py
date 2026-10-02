#!/usr/bin/env python3
"""Final gate (2026-09-27): shipped vs candidates, the bar being NO WORSE than
the shipped law on every robustness test, better on the hardware-clock circles.
    /usr/bin/python3 final_gate.py  -> analysis/final_gate.json"""
import json, multiprocessing as mp, os
import numpy as np
import circle_bench as CB
AN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "analysis")
SHOW, FLOWN = "r075_L24_sync15_cw", "r050_L24_half10"
S = dict(CB.BASE)
import sys
LOGN = sys.argv[1] if len(sys.argv) > 1 else "cma4_log.jsonl"
OUTN = sys.argv[2] if len(sys.argv) > 2 else "final_gate.json"
recs = [json.loads(l) for l in open(os.path.join(AN, LOGN))]
TH = dict(hold=0.0863, ee=0.1184, tilt=4.0, entry=0.524)
def eok(E):
    return (E["verdict"] == "completed" and E["hold_com_rms"] <= 1.1 * TH["hold"] and E["ee_rms"] <= 1.1 * TH["ee"]
            and E["tilt_pk"] <= 6.0 and E["entry_pk"] <= 1.1 * TH["entry"] and E["eR_mean"] <= 0.13)
KEYS = ["k_x", "k_v", "k_R", "k_w", "mrd_s", "ky", "dy", "ky_psi", "dy_psi", "omega_c_t", "omega_c_r", "omega_c_q", "omega_x"]
pas = [r for r in recs if r["gen"] >= 0 and all(r["R"][t]["verdict"] == "completed" for t in "ABCDE") and eok(r["R"]["E"])]
pas.sort(key=lambda r: r["J"])
cand = []
for r in pas:     # distinct (> 10 % apart in log space)
    z = np.log([r["p"][k] for k in KEYS])
    if all(np.max(np.abs(z - np.log([c["p"][k] for k in KEYS]))) > 0.10 for c in cand):
        cand.append(r)
    if len(cand) == 3:
        break
C = {"shipped": S, "F3": dict(S, **json.load(open(os.path.join(AN, "tuned_F3.json")))["best"])}
for i, r in enumerate(cand):
    C[f"H{i+1}"] = dict(S, **r["p"])
for extra in (sys.argv[3].split(",") if len(sys.argv) > 3 else []):     # name=json
    nm, fp = extra.split("=")
    C[nm] = dict(S, **json.load(open(os.path.join(AN, fp)))["best"])
if len(sys.argv) > 4 and sys.argv[4] == "only-extra":
    C = {k: v for k, v in C.items() if k == "shipped" or k in [e.split("=")[0] for e in sys.argv[3].split(",")]}
T = []
for sd in (11, 12, 13):
    T += [(f"show_s{sd}", dict(s=SHOW, seed=sd)), (f"flown_s{sd}", dict(s=FLOWN, seed=sd))]
for d in (24.0, 28.0, 30.0):
    T.append((f"delay{int(d)}", dict(s=SHOW, delay_ms=d)))
T.append(("gust", dict(s=SHOW, gust=(10.0, 3.0, 3.0, 0.0))))
T += [("rob16", dict(s=SHOW, profile="robustness")), ("rob24", dict(s=SHOW, profile="robustness", delay_ms=24.0))]
for sd in (0, 21, 22):
    T.append((f"isaac_rob_s{sd}", dict(s=SHOW, profile="robustness", rtf=0.48, t_settle=20.0, seed=sd)))
T += [("isaac_show", dict(s=SHOW, rtf=0.48)), ("isaac_flown", dict(s=FLOWN, rtf=0.48))]
def job(a):
    n, tn, kw = a; kw = dict(kw); s = kw.pop("s")
    return n, tn, CB.simulate(C[n], CB.stream(s), **kw)
out = {n: {} for n in C}
with mp.get_context("fork").Pool(32) as pool:
    for n, tn, r in pool.imap_unordered(job, [(n, tn, kw) for n in C for tn, kw in T]):
        out[n][tn] = {k: v for k, v in r.items() if k not in ("H", "Hq", "Hqd")}
e = lambda R, t: (f"{R[t]['ee_rms']*1e3:.1f}" if R[t]["verdict"] == "completed" else "ABORT")
for n, R in out.items():
    sh = np.mean([R[f"show_s{s}"]["ee_rms"] for s in (11, 12, 13)]) * 1e3
    fl = np.mean([R[f"flown_s{s}"]["ee_rms"] for s in (11, 12, 13)]) * 1e3
    ir = []
    for sd in (0, 21, 22):
        x = R[f"isaac_rob_s{sd}"]
        ir.append(f"{x['ee_rms']*1e3:.0f}/{x['hold_com_rms']*1e3:.0f}/{x['eR_mean']:.3f}" if x["verdict"] == "completed" else "ABORT")
    g = R["gust"]
    print(f"{n:8} show {sh:5.1f} flown {fl:5.1f} | delay 24/28/30 {e(R,'delay24')}/{e(R,'delay28')}/{e(R,'delay30')} | gust +{g['gust_rise']*1e3:.0f}/{g['gust_rec']:.1f}s | "
          f"rob16 {e(R,'rob16')}/{R['rob16']['entry_pk']*1e3:.0f} rob24 {e(R,'rob24')} | ISAAC-rob ee/hold/|eR| {' '.join(ir)} | ISAAC show {e(R,'isaac_show')} flown {e(R,'isaac_flown')}", flush=True)
json.dump({"cands": C, "res": out}, open(os.path.join(AN, OUTN), "w"), indent=1, default=float)
