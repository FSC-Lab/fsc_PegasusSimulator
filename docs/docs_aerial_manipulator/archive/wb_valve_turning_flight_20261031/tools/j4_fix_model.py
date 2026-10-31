#!/usr/bin/env python3
"""j4 in the room-held valve hold, the 10-16 model (q4_damping_20261016/tools/j4_model.py) re-parameterised from the
2026-10-31 flights, to size the fixes before they fly.
CHANGED vs the 10-16 model:
  law     K 0.10 N*m/rad and D -0.02 N*m*s/rad (the room-hold fits, j4_energy.py: the law's j4 torque pushes WITH
          the motion through the base-lateral/q1 coupling; D_LAW = -0.02 is that, as a stand-in) -- 'cofree' uses
          the CoM-anchored fit (K 0.03, D +0.035) instead
  motor   Ke_true 2.587 (j4_ke.py, today) vs cfg 2.570; relay friction_ff[3] 4.455 counts = 0.030 N*m
  trim    the 0.2 Hz j4 current loop, deadband 5 counts, leak 0.25 s, limit 0.25*57.6 counts; its TARGET either the
          torque term alone ('flown', the code before 2026-10-31) or the torque term + the error damping's own
          current ('fixed')
  start   q4 30 deg off target (every flight broke loose; the model starts after the breakaway)
  forced  the heading target swings +-15 deg at 0.35 Hz (the measured q1 antiphase motion seen through the heading task)
Scores, 5-60 s: q4 p-p, the late amplitude (40-60 s p-p), time slipping, final error to the target."""
import numpy as np
DT, KT_R = 0.004, 2.4234/4.88
KE_TRUE, KE_CFG = 2.587, 2.570
F_S, F_K, B_V = 0.045, 0.0335, 0.020
ENC, W_OBS, KAPPA, J = np.radians(0.0879), 2*np.pi*20.0, 148.51, 0.0109
F_R = 4.455/KAPPA
def sim(d, trim_mode, K=0.10, D=-0.02, T=60.0, kick=0.0, relay=F_R, delay=2, trim_on=True, fk=F_K, q0=np.radians(30), force=0.0, ff=0.35):
    n = int(T/DT); q = q0; v = 0.0; qh = np.round(q0/ENC)*ENC; vh = 0.0; trim = 0.0; buf = [0.0]*(delay+1); stuck = True
    lim = 0.25*57.6117/KAPPA; ded = 5.0/153.4
    out = np.zeros((n, 3))
    for k in range(n):
        t = k*DT
        qd_tgt = kick*np.clip((t-2.0)/0.5, 0, 1) + force*np.sin(2*np.pi*ff*t)
        qd_tgt_dot = force*2*np.pi*ff*np.cos(2*np.pi*ff*t)
        qm = np.round(q/ENC)*ENC; e = qm - qh; qh += DT*(vh + 2*W_OBS*e); vh += DT*(W_OBS**2*e)
        buf.append(-K*(q - qd_tgt) - D*(vh - qd_tgt_dot)); tau_law = buf.pop(0)
        tau_int = tau_law + relay*np.tanh(vh/0.03)
        w_comp = vh - d*vh
        ed = -KT_R*KE_CFG*d*vh                                   # the error damping's torque (its current)
        motor_res = KT_R*(KE_CFG*w_comp - KE_TRUE*v)
        tau_em = tau_int + trim + motor_res
        if trim_on:
            target = tau_int + (ed if trim_mode == "fixed" else 0.0)
            err = target - tau_em
            if abs(target) < ded:
                t2 = trim*np.exp(-DT/0.25); cand = t2 + 2*np.pi*0.2*err*DT
                trim = cand if abs(cand) < abs(t2) else t2
            else:
                trim = np.clip(trim + 2*np.pi*0.2*err*DT, -lim, lim)
        drive = tau_em - B_V*v
        if stuck:
            if abs(drive) > F_S: stuck = False
            else: v = 0.0
        if not stuck:
            a = (drive - fk*np.sign(v if v != 0 else drive))/J; vn = v + a*DT
            if v != 0 and np.sign(vn) != np.sign(v) and abs(drive) <= F_S: vn, stuck = 0.0, True
            v = vn
        q += v*DT
        out[k] = (t, q, 1.0 if stuck else 0.0)
    m = out[:, 0] > 5; late = out[:, 0] > 40
    return dict(pp=np.degrees(np.ptp(out[m, 1])), late=np.degrees(np.ptp(out[late, 1])),
                slip=100*(1-out[m, 2].mean()), err=np.degrees(out[-1, 1]))
rows0 = [("FLOWN: d 0.05, trim target = torque term", 0.05, "flown", {}),
        ("trim target fixed, d 0.05", 0.05, "fixed", {}),
        ("d 0.2 (the Valve tab's raise), trim as flown", 0.20, "flown", {}),
        ("d 0.2 + trim target fixed", 0.20, "fixed", {}),
        ("d 0.2 + trim fixed, law anti-damping x2 (-0.04)", 0.20, "fixed", dict(D=-0.04)),
        ("d 0.2 + trim fixed, kinetic friction -25 %", 0.20, "fixed", dict(fk=F_K*0.75)),
        ("d 0.05, trim OFF", 0.05, "flown", dict(trim_on=False)),
        ("CoM-anchored law (K 0.03, D +0.035), as flown", 0.05, "flown", dict(K=0.03, D=0.035))]
rows = [(l + suf, d, tm, dict(kw, **extra)) for suf, extra in (("", {}), ("  [forced]", dict(force=np.radians(15)))) for l, d, tm, kw in rows0]
print(f"{'case':62s} {'p-p 5-60s':>10s} {'p-p 40-60s':>11s} {'slipping':>9s} {'final err':>10s}")
for lab, d, tm, kw in rows:
    r = sim(d, tm, **kw)
    print(f"{lab:62s} {r['pp']:8.1f}deg {r['late']:9.1f}deg {r['slip']:7.0f} % {r['err']:+8.1f}deg")
