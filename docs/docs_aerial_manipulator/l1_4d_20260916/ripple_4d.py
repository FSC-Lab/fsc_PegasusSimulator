#!/usr/bin/env python3
"""Whole-DIRECT joint-torque ripple and tilt ripple, per run.

    /usr/bin/python3 ripple_4d.py [run.npz ...]

The soak window alone under-reads the 6-D runs: their 0.9 Hz joint-torque
cycle (Command.md 7.15.12) locks in only over the later legs. So this scores
the ripple over ALL of DIRECT after the 40 s entry allowance, as the std of the
band-passed (0.5-3 Hz) joint torques and of the logged tilt.
"""
import glob, sys
import numpy as np

def bp(x, fs, lo=0.5, hi=3.0):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1 / fs)
    X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n=len(x))

args = sys.argv[1:]
SOAK = "--soak" in args
args = [a for a in args if a != "--soak"]
paths = args or sorted(glob.glob("*.npz"))
T_LO, T_HI = (26.0, 43.0) if SOAK else (40.0, 1e9)
if SOAK:
    print("HOVER SOAK WINDOW ONLY (DIRECT +26..+43 s: after the entry settle, before the first step):")
print(f"{'run':<22s} {'tau_j1':>7s} {'tau_j2':>7s} {'tau_j3':>7s} {'tau_j4':>7s}  {'tilt':>6s}  {'|e_psi|':>7s} {'|Fy|':>6s} {'|Fraw|':>6s}   (band-passed 0.5-3 Hz std, mN.m / deg; EE heading mean deg; phantom N; all DIRECT t>40 s)")
for p in paths:
    z = np.load(p, allow_pickle=True); dbg = z["dbg"]; t = dbg[:, 0]; d = dbg[:, 1:]
    m = np.nan_to_num(d[:, 0]) > 0.5; t0 = t[m][0]; w = m & (t - t0 > T_LO) & (t - t0 < T_HI)
    d = d[w]; tt = t[w]; fs = 1 / np.median(np.diff(tt))
    log = z["log"]; lw = (log[:, 0] >= tt[0]) & (log[:, 0] <= tt[-1]); tilt = log[lw, 7]; fsl = 1 / np.median(np.diff(log[lw, 0]))
    taus = [bp(d[:, 13 + j], fs).std() * 1e3 for j in range(4)]
    epsi = np.abs(np.degrees(d[:, 27])).mean()
    fy = np.linalg.norm(d[:, 58:61], axis=1).mean()
    fraw = np.linalg.norm(d[:, 97:100], axis=1).mean() if d.shape[1] > 105 else float('nan')
    com = np.linalg.norm(d[:, 48:51] - d[:, 45:48], axis=1).mean() * 1e3
    ee = np.linalg.norm(d[:, 24:27], axis=1).mean() * 1e3
    print(f"{p.split('/')[-1].replace('.npz',''):<22s} " + " ".join(f"{v:7.1f}" for v in taus)
          + f"  {bp(tilt, fsl).std():6.3f}  {epsi:7.2f} {fy:6.3f} {fraw:6.3f}   CoM {com:5.1f} mm  EE {ee:4.1f} mm")
