import sys, time, multiprocessing as mp, numpy as np
import modular_bench as MB, circle_bench as CB, mapped as MP
S = dict(MP.TABLE2_MAPPED)
cands = {
 "mapped T2": S,
 "arm stiff 300/35": dict(S, a_kp=300.0, a_kd=35.0),
 "arm stiff 600/50": dict(S, a_kp=600.0, a_kd=50.0),
 "arm 300/35 pos 6/4": dict(S, a_kp=300.0, a_kd=35.0, p_kp_xy=6.0, p_kd_xy=4.0, p_kp_z=10, p_kd_z=6),
 "arm 300/35 adapt nu.01": dict(S, a_kp=300.0, a_kd=35.0, a_nu=0.01, p_nu=0.1, q_nu=0.1),
 "arm 300/35 adapt q10": dict(S, a_kp=300.0, a_kd=35.0, a_q=10, p_q=10, q_q=10),
}
def job(k):
    t0=time.time(); r = MB.run(MP.make(cands[k])); return k, r, time.time()-t0
with mp.get_context("fork").Pool(len(cands)) as pool:
    for k, r, dtw in pool.imap(job, list(cands)):
        print(CB.fmt(k, r), f"[{dtw:.0f}s]", flush=True)
