#!/usr/bin/env python3
"""Reconstruct j4's current-loop trim (not logged) and score every j4 duty term's work on the swing.
  written duty  law_debug[16] = clip(tau_ext*kappa + aux + gcorr, +-max_effort) + ke_duty*v_obs + errd + trim
  ke_duty = Ke_cfg*885/12 = 189.5 counts per rad/s (back_emf_ff_scale 1.0)
Work of a duty term on the joint = sum(term/kappa * qdot) dt, in mJ per cycle; positive = pumps the swing.
Usage: j4_trim.py <tag> <t0> <t1>"""
import sys, numpy as np
from scipy.signal import butter, filtfilt
from vt_common import Bag
KAPPA, KE_DUTY, MAXEFF = 148.51, 2.570*885/12, 57.6117
def lp(x, fs, fc=8.0):
    b, a = butter(2, fc/(fs/2)); return filtfilt(b, a, x)
tag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
b = Bag(tag); fs = 250.0
tl = b.t("law"); L = np.array([r if len(r) >= 32 else np.full(32, np.nan) for r in b.f("law","data")])
m = (tl >= t0) & (tl <= t1) & np.isfinite(L[:, 16]); tl, L = tl[m], L[m]
vo = np.interp(tl, b.t("velobs"), b.f("velobs","velocity")[:, 3])
tauterm = np.clip(L[:, 12]*KAPPA + L[:, 21] + L[:, 25], -MAXEFF, MAXEFF)
trim = L[:, 16] - tauterm - KE_DUTY*vo - L[:, 31]
names = [x.strip(" '[]") for x in str(b.f("js","name")[0]).split(",")]; c = names.index("joint4")
q = np.interp(tl, b.t("js"), b.f("js","position")[:, c])
tu = np.arange(tl[0], tl[-1], 1/fs); qu = np.interp(tu, tl, q); qd = lp(np.gradient(qu, tu), fs)
ncyc = max(1, np.sum(np.diff(np.sign(qd[np.abs(qd) > 0.02])) != 0)/2)
print(f"== {tag} {t0}-{t1} s: {ncyc:.0f} cycles, q4 p-p {np.degrees(np.ptp(q)):.0f} deg")
print(f"   trim (reconstructed): mean {trim.mean():+.1f} counts, std {trim.std():.1f}, range {trim.min():+.1f}..{trim.max():+.1f}"
      f"  (limit 0.25*57.6 = {0.25*MAXEFF:.1f}; anything beyond is timing noise in v_obs)")
for name, x in (("law tau_ext", L[:, 12]*KAPPA), ("relay", L[:, 21]), ("err damping", L[:, 31]), ("trim", trim),
                ("ff - true back-EMF (Ke 2.587)", KE_DUTY*vo - 2.587*885/12*np.interp(tl, tu, qd))):
    xu = lp(np.interp(tu, tl, x), fs) / KAPPA
    A = np.c_[qu, qd, np.sign(qd), np.ones(len(tu))]; mm = np.abs(qd) > 0.02
    coef, *_ = np.linalg.lstsq(A[mm], xu[mm], rcond=None)
    print(f"   {name:30s} work {np.sum(xu*qd)/fs/ncyc*1e3:+7.1f} mJ/cycle   fit: {coef[0]:+.3f} q {coef[1]:+.3f} qd {coef[2]:+.4f} sgn [N*m]")
