#!/usr/bin/env python3
"""Rank control gains, impedance-side knobs and the grasp pose on the reduced lateral-roll model
(lateral_roll_model.py). The model gets the DIRECTION of the 10-28 mechanism (the clamped fin's roll
coupling eats the sideways mode's damping) but never reaches the flown growth, so only the ORDERING is
used. Per candidate: the sideways mode's decay rate at a near-marginal coupling (k_s = 10 and 20 N.m/rad),
the free-flight sideways mode, and the 1.5 Hz roll mode with its delay margin (the known limit: the rotor
lag; every tune that ate this margin failed in flight or Isaac)."""
import numpy as np
from lateral_roll_model import BASE, A_of, with_

def slow(P):
    ev = np.linalg.eigvals(A_of(P)); o = [l for l in ev if 1.0 < l.imag < 6.0]
    return max(o, key=lambda v: v.real) if o else complex(np.nan, np.nan)

def fast(P):
    ev = np.linalg.eigvals(A_of(P)); o = [l for l in ev if 6.0 <= l.imag < 40.0]
    return max(o, key=lambda v: v.real) if o else complex(np.nan, np.nan)

def delay_margin(P):
    for T in np.arange(0.005, 0.12, 0.0025):
        if max(np.linalg.eigvals(A_of(dict(P, T=T, k_s=0.0))).real) > 0: return T
    return np.inf

C = BASE["c"]
cands = [("shipped (k_x 50, k_v 12.6, k_R 1.6, k_w 1.2)", {}),
         ("k_v 15", dict(k_v=15.0)), ("k_v 10", dict(k_v=10.0)),
         ("k_x 40", dict(k_x=40.0)), ("k_x 60", dict(k_x=60.0)),
         ("k_R 2.134 / k_w 1.567 (H1b, pick-and-drop)", dict(k_R=2.134, k_w=1.567)),
         ("k_R 2.0", dict(k_R=2.0)), ("k_w 1.5", dict(k_w=1.5)), ("k_w 1.0", dict(k_w=1.0)),
         ("M_r_d x0.85", dict(I_d=BASE["I_d"] * 0.85)), ("M_r_d x1.2", dict(I_d=BASE["I_d"] * 1.2)),
         ("omega_c_r 0.5", dict(w_r=0.5)), ("omega_c_r 1.0", dict(w_r=1.0)),
         ("pose coupling x0.75 (fold ~22 deg)", dict(c=0.75 * C)), ("pose coupling x0.5 (fold ~15 deg)", dict(c=0.5 * C)),
         ("handle free in roll (k_s -> 0)", dict(k_s_scale=0.0))]
print(f"{'candidate':44s} | sideways mode decay 1/s @ k_s 10 / 20 | free flight | 1.5 Hz mode decay | delay margin")
for name, kw in cands:
    sc = kw.pop("k_s_scale", 1.0)
    s10 = slow(with_(k_s=10 * sc, **kw)); s20 = slow(with_(k_s=20 * sc, **kw)); s0 = slow(with_(k_s=0, **kw))
    fm = fast(with_(k_s=0, **kw)); dm = delay_margin(with_(**kw))
    print(f"{name:44s} |   {s10.real:+.3f} ({s10.imag:.1f}) / {s20.real:+.3f} ({s20.imag:.1f} rad/s)   |"
          f" {s0.real:+.2f} ({s0.imag:.1f}) | {fm.real:+.3f} ({fm.imag/2/np.pi:.2f} Hz) | {dm*1e3:5.1f} ms")
