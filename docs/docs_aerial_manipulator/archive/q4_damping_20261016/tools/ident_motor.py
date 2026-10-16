#!/usr/bin/env python3
"""Electrical identification of the arm motors from FLIGHT data: V = R*I + Ke*qdot (+ c), per joint.

    PYTHONNOUSERSITE=1 /usr/bin/python3 ident_motor.py [run ...]      (default d1 e1 d2: the 10-15 flights)

V   = the duty the arm controller WROTE (law_debug[13..16], hardware order j1..j4) * Vs / 885
I   = Present Current (joint_states.effort, order joint2, joint3, joint1, joint4) * 2.69 mA
qdot= d/dt of the encoder position (joint_states.position), central difference
All three zero-phase low-passed at 10 Hz (the 2026-09-08 bench fit's filter); the write -> read delay is
scanned (the current responds to a duty one or more control ticks later) and the best-R^2 lag kept.
Fitted over DIRECT, on samples where the joint actually moves (|qdot| > VMIN), so Ke is identified from
motion and not from the current sensor's offset. j2/j3 have bench-measured Ke: they validate the method.
"""
import os
import sys
import numpy as np
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.abspath(os.path.join(HERE, "..", "..", "wb_pick_drop_flight_20261015", "npz"))
VS, FULL, LSB = 12.0, 885.0, 0.00269
KE_CFG = np.array([2.427, 2.559, 2.623, 2.570])        # back_emf_ke, hardware yaml (j1..j4)
R_CFG = np.array([5.26, 4.90, 4.53, 4.88])             # motor_resistance_ohm
NM_TO_EFFORT = np.array([162.4, 154.0, 150.5, 153.4])  # current counts per N*m (calibrated 09-11)
KT = 1.0 / (NM_TO_EFFORT * LSB)                        # N*m/A
JS_OF = {0: 2, 1: 0, 2: 1, 3: 3}                       # joint j (0-based j1..j4) -> joint_states column
VMIN = 0.05                                            # rad/s


def lp(x, fs, fc=10.0):
    b, a = butter(2, fc / (fs / 2))
    return filtfilt(b, a, x)


def series(run):
    d = np.load(os.path.join(NPZ, f"{run}.npz"))
    tj = d["js__recv"]; P = d["js__position"]; C = d["js__effort"]
    tl = d["law__recv"]; L = d["law__data"]
    direct = L[:, 0] > 0.5                                  # pass-through = whole-body DIRECT owns the arm
    t0, t1 = tl[direct][0], tl[direct][-1]
    fs = 250.0
    tu = np.arange(t0 + 1.0, t1 - 1.0, 1.0 / fs)
    out = {}
    for j in range(4):
        c = JS_OF[j]
        q = np.interp(tu, tj, P[:, c]); i = np.interp(tu, tj, C[:, c]) * LSB
        duty = np.interp(tu, tl, L[:, 13 + j])
        qd = np.gradient(q, tu)
        out[j] = (lp(duty * VS / FULL, fs), lp(i, fs), lp(qd, fs))
    return tu, out


def fit(V, I, W, lag):
    """V(t) = R I(t+lag) + Ke W(t+lag) + c ; lag in samples (current read after the write)."""
    if lag > 0:
        V, I, W = V[:-lag], I[lag:], W[lag:]
    m = np.abs(W) > VMIN
    A = np.c_[I[m], W[m], np.ones(m.sum())]
    x, *_ = np.linalg.lstsq(A, V[m], rcond=None)
    res = V[m] - A @ x
    r2 = 1 - res.var() / V[m].var()
    return x, r2, m.mean(), np.sqrt(np.mean(W[m] ** 2))


def main(runs):
    print(f"{'run':3s} {'jt':3s} {'lag ms':>6s} {'R fit':>6s} {'(cfg)':>6s} {'Ke fit':>7s} {'(cfg)':>6s} "
          f"{'c [V]':>6s} {'R2':>5s} {'moving':>6s} {'|qd| rms':>8s}  residual damping Kt*(Ke-s*Ke_cfg)/R at s=1")
    for run in runs:
        tu, S = series(run)
        for j in range(4):
            V, I, W = S[j]
            best = max(((fit(V, I, W, lag), lag) for lag in range(0, 9)), key=lambda z: z[0][1])
            (x, r2, frac, wr), lag = best
            R, Ke, c = x
            bres = KT[j] * (Ke - KE_CFG[j]) / R
            print(f"{run:3s} j{j+1}  {lag*4:6.0f} {R:6.2f} {R_CFG[j]:6.2f} {Ke:7.3f} {KE_CFG[j]:6.3f} {c:6.3f} "
                  f"{r2:5.3f} {frac*100:5.1f}% {wr:8.3f}  {bres:+.3f} N*m*s/rad"
                  f"   (motor's own Kt*Ke/R = {KT[j]*Ke/R:.3f})")


if __name__ == "__main__":
    main(sys.argv[1:] or ["d1", "e1", "d2"])
