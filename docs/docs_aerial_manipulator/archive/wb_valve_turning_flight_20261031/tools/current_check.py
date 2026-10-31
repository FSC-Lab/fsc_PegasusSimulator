#!/usr/bin/env python3
"""Independent check of the trim-vs-damping finding, from the MEASURED current (no duty reconstruction):
  delivered current (counts, Present Current) - the torque term's current (tau term * g)
  = the error damping's current (-d*ke_duty*(v - qd_ref)*g) + the trim + back-EMF mismatch.
If the damping reaches the motor, the slope on q4dot is ~ -d*ke_duty*g = -9.8 counts per rad/s (d 0.05);
if the trim cancels it, the slope is ~0.  Split by whether the trim is active (|tau term*g| > 5 counts)."""
import sys, os, numpy as np
from scipy.signal import butter, filtfilt
KAPPA, KE_DUTY, MAXEFF, G = 148.51, 2.570*885/12, 57.6117, 153.4/148.51
def lp(x, fs, fc=4.0):
    b, a = butter(2, fc/(fs/2)); return filtfilt(b, a, x)
for f in sys.argv[1:]:
    z = np.load(f, allow_pickle=True)
    tl = z["law__recv"]; L = np.array([r if len(r) >= 32 else np.full(32, np.nan) for r in z["law__data"]])
    m = (L[:, 0] > 0.5) & np.isfinite(L[:, 31]); tl, L = tl[m], L[m]
    names = [x.strip(" '[]") for x in str(z["js__name"][0]).split(",")]; c = names.index("joint4")
    fs = 250.0; tu = np.arange(tl[0], tl[-1], 1/fs)
    tj = z["js__recv"]; q = np.interp(tu, tj, z["js__position"][:, c]); qd = lp(np.gradient(q, tu), fs)
    I = lp(np.interp(tu, tj, z["js__effort"][:, c].astype(float)), fs)
    tau = np.clip(L[:, 12]*KAPPA + L[:, 21] + L[:, 25], -MAXEFF, MAXEFF)
    taug = lp(np.interp(tu, tl, tau), fs)*G
    d = np.nanmedian(np.abs(L[:, 31])/np.maximum(np.abs(np.interp(tl, z["velobs__recv"], z["velobs__velocity"][:, 3])), 1e-3)/KE_DUTY)
    active = np.abs(taug) > 5.0
    for lab, mm in (("trim active", active & (np.abs(qd) > 0.05)), ("trim gated ", ~active & (np.abs(qd) > 0.05))):
        if mm.sum() < 300: continue
        A = np.c_[qd[mm], np.sign(qd[mm]), np.ones(mm.sum())]
        coef = np.linalg.lstsq(A, I[mm] - taug[mm], rcond=None)[0]
        print(f"  {os.path.basename(f):12s} {lab}: moving {mm.sum()/fs:5.1f} s | measured-current minus torque-term current: "
              f"{coef[0]:+6.1f} counts per rad/s (damping alone would give {-d*KE_DUTY*G:+.1f}; d = {d:.3f})")
