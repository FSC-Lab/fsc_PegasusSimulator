#!/usr/bin/env python3
"""Score the DIRECT window of a wb_l1_tune_cycle npz: how long DIRECT lasted, the
tilt/attitude-error envelope, and the dominant attitude-error frequency.
    /usr/bin/python3 score_direct.py <npz> [...]"""
import sys
import numpy as np


def score(path):
    d = np.load(path, allow_pickle=True)
    dc = list(d["dbg_cols"]); D = d["dbg"]
    t = D[:, dc.index("t")]
    c = lambda i: D[:, dc.index(f"d{i}")]  # noqa: E731 -- wb_control_debug[i]
    direct = c(0) > 0.5
    L = d["log"]; lc = list(d["log_cols"])
    tl = L[:, lc.index("t")]; tilt = L[:, lc.index("tilt_deg")]
    if not direct.any():
        return f"{path}: never in DIRECT"
    i0 = np.argmax(direct); i1 = len(direct) - 1 - np.argmax(direct[::-1])
    ta, tb = t[i0], t[i1]
    m = (t >= ta) & (t <= tb) & direct
    eR = np.sqrt(c(28) ** 2 + c(29) ** 2 + c(30) ** 2)[m]
    ex = np.sqrt(sum((c(48 + k) - c(45 + k)) ** 2 for k in range(3)))[m]
    tm = t[m]
    mt = (tl >= ta) & (tl <= tb)
    # dominant frequency of e_R,x over the DIRECT window (wall seconds)
    x = c(28)[m] - c(28)[m].mean(); fs = len(x) / max(tb - ta, 1e-6)
    F = np.abs(np.fft.rfft(x * np.hanning(len(x)))); f = np.fft.rfftfreq(len(x), 1 / fs)
    k = np.argmax(F[2:]) + 2
    # growth: |e_R| rms in the first vs last 5 s
    a5 = np.sqrt(np.mean(eR[tm < ta + 5] ** 2)); b5 = np.sqrt(np.mean(eR[tm > tb - 5] ** 2))
    return (f"{path.split('/')[-1]:28} DIRECT {tb - ta:6.1f} s wall | aborted {bool(d['aborted'])} "
            f"| tilt max {tilt[mt].max():5.1f} deg | |e_R| rms first/last 5 s {a5:.4f}/{b5:.4f} "
            f"| CoM err rms {np.sqrt(np.mean(ex ** 2)) * 1e3:6.1f} mm, max {ex.max() * 1e3:6.0f} "
            f"| e_R,x peak freq {f[k]:.2f} Hz (wall)")


for p in sys.argv[1:]:
    print(score(p))
