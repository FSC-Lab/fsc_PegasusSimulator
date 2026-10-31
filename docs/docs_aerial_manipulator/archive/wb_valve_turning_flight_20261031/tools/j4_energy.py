#!/usr/bin/env python3
"""Who pumps the j4 swing: least-squares fit of each torque acting on j4 as a*q + b*qdot + c*sign(qdot) + d
over a window, plus the work each does per cycle (integral of tau * qdot dt / cycles).
  law     = the whole-body law's streamed j4 torque (law_debug[12], N*m)
  obs     = the observer's share of it (wb_control_debug[96], applied after the j4 cap)
  relay   = the arm controller's aux term (friction relay) law_debug[21] / kappa
  errd    = the error-damping duty law_debug[31] / kappa
  em      = the delivered electromagnetic torque Kt * Present Current
  resid   = em - (law + relay + errd): the motor's own residual (back-EMF over/under-cancel, current trim)
Usage: j4_energy.py <tag> <t0> <t1>   (numpy-1 /usr/bin/python3; scipy for the filter)"""
import sys, numpy as np
from scipy.signal import butter, filtfilt
from vt_common import Bag
LSB = 0.00269; NM_TO_EFFORT4 = 153.4; KT4 = 1.0/(NM_TO_EFFORT4*LSB); KPWM4 = 148.51
def lp(x, fs, fc=8.0):
    b, a = butter(2, fc/(fs/2)); return filtfilt(b, a, x)
def run(tag, t0, t1):
    b = Bag(tag); fs = 250.0
    names = [x.strip(" '[]") for x in str(b.f("js","name")[0]).split(",")]; c = names.index("joint4")
    tj = b.t("js"); tu = np.arange(t0, t1, 1/fs)
    q = np.interp(tu, tj, b.f("js","position")[:, c]); qd = lp(np.gradient(q, tu), fs)
    em = lp(np.interp(tu, tj, b.f("js","effort")[:, c]) * LSB * KT4, fs)
    tl = b.t("law"); L = np.array([r if len(r) >= 32 else np.full(32, np.nan) for r in b.f("law","data")])
    law = lp(np.interp(tu, tl, L[:, 12]), fs); relay = lp(np.interp(tu, tl, L[:, 21]), fs)/KPWM4
    errd = lp(np.interp(tu, tl, L[:, 31]), fs)/KPWM4; duty = lp(np.interp(tu, tl, L[:, 16]), fs)
    tw = b.t("wb"); W = b.f("wb","data"); ok = np.array([len(r) >= 120 for r in W]); Wm = np.array([r[:120] for r in W[ok]])
    obs = lp(np.interp(tu, tw[ok], Wm[:, 96]), fs); tau16 = lp(np.interp(tu, tw[ok], Wm[:, 16]), fs)
    hdg = lp(np.interp(tu, tw[ok], Wm[:, 119]), fs)
    resid = em - (law + relay + errd)
    m = np.abs(qd) > 0.02
    ncyc = max(1, np.sum(np.diff(np.sign(qd[m])) != 0) / 2)
    print(f"== {tag} {t0:.1f}-{t1:.1f} s: q4 p-p {np.degrees(np.ptp(q)):.0f} deg, {ncyc:.0f} cycles, |qd| max {np.degrees(np.abs(qd).max()):.0f} deg/s")
    print(f"   {'term':6s} {'a [Nm/rad]':>10s} {'b [Nms/rad]':>11s} {'c [Nm]':>8s} {'d [Nm]':>8s} {'R2':>5s} {'work/cyc mJ':>11s}")
    A = np.c_[q[m], qd[m], np.sign(qd[m]), np.ones(m.sum())]
    for name, x in (("law", law), ("obs", obs), ("relay", relay), ("errd", errd), ("em", em), ("resid", resid)):
        coef, *_ = np.linalg.lstsq(A, x[m], rcond=None); r2 = 1 - np.var(x[m] - A @ coef)/max(np.var(x[m]), 1e-12)
        work = np.sum(x * qd) / fs / ncyc * 1e3
        print(f"   {name:6s} {coef[0]:+10.4f} {coef[1]:+11.4f} {coef[2]:+8.4f} {coef[3]:+8.4f} {r2:5.2f} {work:+11.2f}")
    # the law torque against the heading-target offset too
    A2 = np.c_[q[m], qd[m], hdg[m], np.ones(m.sum())]
    coef, *_ = np.linalg.lstsq(A2, law[m], rcond=None)
    print(f"   law = {coef[0]:+.4f} q4 {coef[1]:+.4f} qd4 {coef[2]:+.4f} hdg_offset {coef[3]:+.4f}  (rad, rad/s)")
    # inertia check: J * qdd = em - friction  -> fit em = J qdd + fk sign(qd) + bv qd
    qdd = lp(np.gradient(qd, tu), fs, 4.0)
    A3 = np.c_[qdd[m], np.sign(qd[m]), qd[m], np.ones(m.sum())]
    coef, *_ = np.linalg.lstsq(A3, em[m], rcond=None); r2 = 1 - np.var(em[m] - A3 @ coef)/np.var(em[m])
    print(f"   em = {coef[0]:.4f} qdd + {coef[1]:.4f} sign + {coef[2]:.4f} qd + {coef[3]:+.4f}   R2 {r2:.2f}   (J, Coulomb, viscous)")
if __name__ == "__main__":
    run(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]))
