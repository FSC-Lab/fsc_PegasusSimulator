#!/usr/bin/env python3
"""Does j4's current trim integrate the error damping away? Over every stretch where j4 moves and the trim is
NOT gated (|tau term| above the 5-count deadband), regress the reconstructed trim on q4dot.  A coefficient
equal and opposite to the error damping's (-d * 1.276 N*m*s/rad) means the trim cancels it.
Usage: trim_vs_damping.py <npz> ...   (any 10-16+ whole-body bag npz with law_debug[28..31])"""
import sys, os, numpy as np
from scipy.signal import butter, filtfilt
KAPPA, KE_DUTY, MAXEFF, G = 148.51, 2.570*885/12, 57.6117, 153.4/148.51
def lp(x, fs, fc=8.0):
    b, a = butter(2, fc/(fs/2)); return filtfilt(b, a, x)
for f in sys.argv[1:]:
    z = np.load(f, allow_pickle=True)
    tl = z["law__recv"]; L = np.array([r if len(r) >= 32 else np.full(32, np.nan) for r in z["law__data"]])
    m = (L[:, 0] > 0.5) & np.isfinite(L[:, 31]); tl, L = tl[m], L[m]
    if len(tl) < 1000: print(os.path.basename(f), "no DIRECT"); continue
    vo = np.interp(tl, z["velobs__recv"], z["velobs__velocity"][:, 3])
    tau = np.clip(L[:, 12]*KAPPA + L[:, 21] + L[:, 25], -MAXEFF, MAXEFF)
    trim = L[:, 16] - tau - KE_DUTY*vo - L[:, 31]
    names = [x.strip(" '[]") for x in str(z["js__name"][0]).split(",")]
    q = np.interp(tl, z["js__recv"], z["js__position"][:, names.index("joint4")])
    fs = 250.0; tu = np.arange(tl[0], tl[-1], 1/fs); qd = lp(np.gradient(np.interp(tu, tl, q), tu), fs)
    tr = lp(np.interp(tu, tl, trim), fs)/KAPPA; ed = lp(np.interp(tu, tl, L[:, 31]), fs)/KAPPA
    active = np.abs(np.interp(tu, tl, tau))*G > 5.0
    for lab, mm in (("trim active", active & (np.abs(qd) > 0.05)), ("trim gated", ~active & (np.abs(qd) > 0.05))):
        if mm.sum() < 300: print(f"  {os.path.basename(f):12s} {lab:12s}: too few moving samples"); continue
        A = np.c_[qd[mm], np.ones(mm.sum())]
        bt = np.linalg.lstsq(A, tr[mm], rcond=None)[0][0]; be = np.linalg.lstsq(A, ed[mm], rcond=None)[0][0]
        print(f"  {os.path.basename(f):12s} {lab:12s}: moving {mm.sum()/fs:6.1f} s | err damping {be:+.3f} N*m*s/rad | trim {bt:+.3f} N*m*s/rad | net {be+bt:+.3f}")
