#!/usr/bin/env python3
"""j4's true back-EMF constant and the observer's gain on the 2026-10-31 flights, by the 10-16 method
(q4_damping_20261016/tools/j4_net_damping.py): V - R*I = Ke*qdot with R pinned at the 09-11 calibration, on
the samples where j4 moves; and v_obs / qdot.  The feed-forward cancels s*Ke_cfg*v_obs, so its over-
cancellation is Ke_cfg*gain/Ke_true - 1, in N*m*s/rad (Kt/R)*(Ke_cfg*gain - Ke_true)."""
import sys, numpy as np
from scipy.signal import butter, filtfilt
from vt_common import Bag
VS, FULL, LSB = 12.0, 885.0, 0.00269
KE_CFG, R_CFG, KT = 2.570, 4.88, 1.0/(153.4*0.00269)
def lp(x, fs, fc=10.0):
    b, a = butter(2, fc/(fs/2)); return filtfilt(b, a, x)
for tag in (sys.argv[1:] or ["174943", "175725", "180423"]):
    b = Bag(tag); fs = 250.0
    names = [x.strip(" '[]") for x in str(b.f("js","name")[0]).split(",")]; c = names.index("joint4")
    tl = b.t("law"); L = np.array([r if len(r) >= 32 else np.full(32, np.nan) for r in b.f("law","data")])
    direct = L[:, 0] > 0.5
    tu = np.arange(tl[direct][0] + 1, tl[direct][-1] - 1, 1/fs)
    tj = b.t("js"); q = np.interp(tu, tj, b.f("js","position")[:, c]); qd = lp(np.gradient(q, tu), fs)
    I = lp(np.interp(tu, tj, b.f("js","effort")[:, c]) * LSB, fs)
    V = lp(np.interp(tu, tl, L[:, 16]), fs) * VS / FULL
    vo = lp(np.interp(tu, b.t("velobs"), b.f("velobs","velocity")[:, 3]), fs)
    best = None
    for lag in range(0, 8):
        Vl, Il, ql, vl = (V[:-lag], I[lag:], qd[lag:], vo[:-lag]) if lag else (V, I, qd, vo)
        m = np.abs(ql) > 0.1
        if m.sum() < 500: continue
        ke = np.linalg.lstsq(np.c_[ql[m], np.ones(m.sum())], Vl[m] - R_CFG*Il[m], rcond=None)[0][0]
        res = Vl[m] - R_CFG*Il[m] - ke*ql[m]; r2 = 1 - res.var()/(Vl[m]-R_CFG*Il[m]).var()
        if best is None or r2 > best[1]: best = (lag, r2, ke)
    m = np.abs(qd) > 0.1
    gain = np.linalg.lstsq(qd[m, None], vo[m], rcond=None)[0][0]
    lag, r2, ke = best
    over = KT/R_CFG*(KE_CFG*gain - ke)
    print(f"{tag}: moving {m.mean()*100:4.1f} % of DIRECT | Ke_true {ke:.3f} (cfg {KE_CFG}, lag {lag} ticks, R2 {r2:.3f}) | "
          f"v_obs/qdot {gain:.3f} | feed-forward over-cancels {100*(KE_CFG*gain/ke-1):+.1f} % = {over:+.3f} N*m*s/rad")
