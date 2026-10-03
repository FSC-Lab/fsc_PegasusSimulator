import sys, time, multiprocessing as mp, numpy as np
import modular_bench as MB, circle_bench as CB, mapped as MP
S = dict(MP.TABLE2_MAPPED)
H = dict(S, q_kp_rp=40.0, q_kd_rp=10.0, q_kp_y=20.0, q_kd_y=8.0, p_kp_xy=4.0, p_kd_xy=3.5, p_kp_z=9.0, p_kd_z=5.0,
         a_kp=600.0, a_kd=40.0)
cands = {
 "H": H,
 "H qv.05": dict(H, p_qv=0.05, q_qv=0.05, a_qv=0.05),
 "H qv.05 nu.5": dict(H, p_qv=0.05, q_qv=0.05, a_qv=0.05, p_nu=0.5, q_nu=0.5, a_nu=0.5),
 "H att 60/12": dict(H, q_kp_rp=60.0, q_kd_rp=12.0),
 "H att 25/8": dict(H, q_kp_rp=25.0, q_kd_rp=8.0),
 "H pos 8/5": dict(H, p_kp_xy=8.0, p_kd_xy=5.0),
 "H ideal": H,
}
def job(k):
    t0=time.time(); r = MB.run(MP.make(cands[k]), fb=CB.FB_IDEAL if "ideal" in k else None); return k, r, time.time()-t0
with mp.get_context("fork").Pool(len(cands)) as pool:
    for k, r, dtw in pool.imap(job, list(cands)):
        print(CB.fmt(k, r), f"[{dtw:.0f}s]", flush=True)
