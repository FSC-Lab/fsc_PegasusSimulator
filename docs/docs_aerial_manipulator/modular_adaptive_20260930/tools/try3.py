import sys, json, time, multiprocessing as mp, numpy as np
import modular_bench as MB, circle_bench as CB, mapped as MP
recs=[json.loads(l) for l in open("../analysis/cma_modular_log.jsonl")]
best=min(recs, key=lambda r:r["J"])["p"]
B = dict(MP.TABLE2_MAPPED); B.update(dict(p_q=1.0, q_q=1.0, a_q=1.0)); B.update(best)
cands = {"best": B}
for qs in (30.0, 300.0, 3000.0):
    cands[f"Q x{qs:g} (p,q)"] = dict(B, p_q=qs, q_q=qs)
cands["Q x300 (p,q,a)"] = dict(B, p_q=300.0, q_q=300.0, a_q=300.0)
cands["Q x300 nu/10"] = dict(B, p_q=300.0, q_q=300.0, p_nu=B["p_nu"]/10, q_nu=B["q_nu"]/10)
def job(k):
    t0=time.time(); r = MB.run(MP.make(cands[k]), keep=True); return k, r, time.time()-t0
with mp.get_context("fork").Pool(len(cands)) as pool:
    for k, r, dtw in pool.imap(job, list(cands)):
        print(CB.fmt(k, r), f"[{dtw:.0f}s]", flush=True)
