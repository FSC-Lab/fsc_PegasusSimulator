#!/usr/bin/env python3
"""pattern_check.py -- does the Isaac whole-body error REPEAT the hardware's repeatable error?
Run phase grid tau = t - t_start (both plans are the same A 0.70 / B 0.35 figure-8 at the same speed);
reference paths aligned by subtracting the start point and rotating so the start tangents coincide
(mirroring if needed); errors resolved on the path (along / cross). Prints the reference-path match and
the correlation of the sim error with the hardware two-run mean.
  step 1 (numpy-1 python, hardware): AM_NPZ=<dir> PYTHONNOUSERSITE=1 /usr/bin/python3 pattern_check.py hw w3 w4 out_hw.npz
  step 2 (numpy-2 python, sim):      /usr/bin/python3 pattern_check.py cmp out_hw.npz <sim.npz> [...]
"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))


def pathframe(red, e, dt=0.01):
    tan = np.gradient(red[:, :2], dt, axis=0); sp = np.linalg.norm(tan, axis=1)
    tan = tan / (sp[:, None] + 1e-12); left = np.column_stack([-tan[:, 1], tan[:, 0]])
    return np.column_stack([np.sum(e[:, :2] * tan, 1), np.sum(e[:, :2] * left, 1)]), sp


if sys.argv[1] == "hw":
    sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_vs_decoupled_figure8_flight_20261005", "tools"))
    import f1005  # noqa  (puts the 0928 tools on the path)
    import metrics as M  # noqa
    S = [M.analyse(n)[1] for n in sys.argv[2:4]]
    n = min(len(s["t"]) for s in S)
    np.savez(sys.argv[4], red=np.array([s["red"][:n] for s in S]), e=np.array([(s["re"] - s["red"])[:n] for s in S]))
    sys.exit()

sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "..", "application", "robotic_arm", "utils"))
import am_ee_compare_score as SC  # noqa
H = np.load(sys.argv[2]); red_h = H["red"][0]; eh = H["e"]
pf_h = [pathframe(H["red"][i], eh[i])[0] for i in range(2)]
mean_h = (pf_h[0] + pf_h[1]) / 2; sp_h = pathframe(red_h, eh[0])[1]
for p in sys.argv[3:]:
    res, ser = SC.analyse(p)
    E = ser["ee"]; tg = np.arange(E[0, 0], E[-1, 0], 0.01)
    m = np.column_stack([np.interp(tg, E[:, 0], E[:, i]) for i in range(1, 7)])
    red_s, e_s = m[:, 3:6], m[:, 0:3] - m[:, 3:6]
    n = min(len(tg), len(red_h))
    # align reference paths: start point, start tangent, mirror
    A = red_h[:n, :2] - red_h[0, :2]; Bs = red_s[:n, :2] - red_s[0, :2]
    best = None
    for mir in (1, -1):
        Bm = Bs * np.array([1, mir])
        k = np.searchsorted(np.cumsum(np.r_[0, np.linalg.norm(np.diff(A, axis=0), axis=1)]), 0.05)
        ang = np.arctan2(A[k, 1], A[k, 0]) - np.arctan2(Bm[k, 1], Bm[k, 0])
        Rm = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
        dev = np.sqrt(np.mean(np.sum((A - Bm @ Rm.T) ** 2, 1)))
        if best is None or dev < best[0]: best = (dev, mir)
    pf_s, sp_s = pathframe(red_s[:n], e_s[:n])
    if best[1] < 0: pf_s[:, 1] *= -1                       # a mirrored traversal flips the cross-track sign
    ok = (sp_s > 0.02) & (sp_h[:n] > 0.02)
    c = [np.corrcoef(pf_s[ok, i], mean_h[:n][ok, i])[0, 1] for i in range(2)]
    ch = [np.corrcoef(pf_h[0][:n][ok, i], pf_h[1][:n][ok, i])[0, 1] for i in range(2)]
    rs = lambda x: 1e3 * np.sqrt(np.mean(x ** 2))
    print(f"{os.path.basename(p):20s} ref-path match {1e3*best[0]:.1f} mm (mirror {best[1] < 0})  "
          f"along: sim rms {rs(pf_s[ok,0]):.1f} hw-mean rms {rs(mean_h[:n][ok,0]):.1f} corr(sim, hw mean) {c[0]:+.2f} [hw run-vs-run {ch[0]:+.2f}]  "
          f"cross: sim {rs(pf_s[ok,1]):.1f} hw-mean {rs(mean_h[:n][ok,1]):.1f} corr {c[1]:+.2f} [hw {ch[1]:+.2f}]")
