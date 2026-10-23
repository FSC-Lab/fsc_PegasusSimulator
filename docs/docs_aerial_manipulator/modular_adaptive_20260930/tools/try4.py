import sys, json, time, multiprocessing as mp, numpy as np
import modular_bench as MB, circle_bench as CB, mapped as MP
recs=[json.loads(l) for l in open("../analysis/cma_modular_log.jsonl")]
best=min(recs, key=lambda r:r["J"])["p"]
B = dict(MP.TABLE2_MAPPED); B.update(dict(p_q=1.0, q_q=1.0, a_q=1.0)); B.update(best)
cands = {}
for qs, nu0, nu123, acc in ((30, 0.01, 30, 10), (30, 0.01, 30, 2), (100, 0.01, 100, 2), (100, 0.001, 100, 1),
                            (300, 0.01, 300, 1), (10, 0.001, 20, 2), (1000, 0.01, 1000, 1)):
    cands[f"q{qs} nu0 {nu0} nu123 {nu123} acc {acc}"] = dict(B, p_q=qs, q_q=qs, p_nu0=nu0, q_nu0=nu0,
                                                              p_nu123=nu123, q_nu123=nu123, acc_hz=acc)
def job(k):
    t0=time.time(); r = MB.run(MP.make(cands[k])); return k, r, time.time()-t0
with mp.get_context("fork").Pool(len(cands)) as pool:
    for k, r, dtw in pool.imap(job, list(cands)):
        print(CB.fmt(k, r), f"[{dtw:.0f}s]", flush=True)
