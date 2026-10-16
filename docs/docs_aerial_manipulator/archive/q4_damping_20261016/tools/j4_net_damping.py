#!/usr/bin/env python3
"""What the arm controller DELIVERS on a joint against what it INTENDED, as a function of joint velocity.

    PYTHONNOUSERSITE=1 /usr/bin/python3 j4_net_damping.py [run ...]      (default d1 d2 e1)

delivered = Kt * Present Current                                          (N*m, electromagnetic)
intended  = (tau_cmd - back-EMF FF - trim) / kappa_pwm
          = law_debug: the duty WRITTEN minus everything that is not torque intent. The written duty is
            tau_clamped + s*ke*v_obs + trim; the torque intent is tau_clamped (the streamed torque + friction
            relay + dither + gravity correction, clamped). We do not log the trim or v_obs*ke separately, so
            intended = (ext + aux + gravity_corr) * kappa, clamped -- reconstructed from law_debug [9..12],
            [18..21], [22..25].
residual  = delivered - intended = what the motor added on its own: back-EMF left over (Kt/R)(s*Ke_cfg*v_obs -
            Ke_true*qdot), the current trim, and the current sensor's offset.
The slope of residual on qdot is the joint's NET ELECTRICAL DAMPING (negative slope = damping, positive =
the motor pushes WITH the motion). Also: Ke_true from V - R_cfg*I = Ke*qdot with R pinned at the 09-11
calibration (R and Ke are collinear on j4, whose current is mostly velocity).
"""
import os
import sys
import numpy as np
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ident_motor import NPZ, VS, FULL, LSB, KE_CFG, R_CFG, NM_TO_EFFORT, KT, JS_OF, lp  # noqa: E402

KPWM = np.array([169.47, 149.70, 135.25, 148.51])        # duty counts per N*m (NM_TO_DUTY, servo_model.py)
MAXEFF = np.array([57.6117, 364.8743, 192.0391, 57.6117])


def run_one(run, j=3):
    d = np.load(os.path.join(NPZ, f"{run}.npz"))
    tj = d["js__recv"]; P = d["js__position"]; C = d["js__effort"]
    tl = d["law__recv"]; L = d["law__data"]
    to = d["velobs__recv"]; VO = d["velobs__velocity"]
    direct = L[:, 0] > 0.5
    fs = 250.0
    tu = np.arange(tl[direct][0] + 1.0, tl[direct][-1] - 1.0, 1.0 / fs)
    c = JS_OF[j]
    q = np.interp(tu, tj, P[:, c]); qd = lp(np.gradient(q, tu), fs)
    i_a = np.interp(tu, tj, C[:, c]) * LSB
    delivered = lp(KT[j] * i_a, fs)
    inten_cnt = np.clip((np.interp(tu, tl, L[:, 9 + j]) * KPWM[j] + np.interp(tu, tl, L[:, 18 + j])
                         + np.interp(tu, tl, L[:, 22 + j])), -MAXEFF[j], MAXEFF[j])
    intended = lp(inten_cnt / KPWM[j], fs)
    duty = lp(np.interp(tu, tl, L[:, 13 + j]), fs)
    vobs = lp(np.interp(tu, to, VO[:, j]), fs)
    resid = delivered - intended
    V = duty * VS / FULL; I = lp(i_a, fs)
    print(f"--- {run} j{j+1}: DIRECT {tu[-1]-tu[0]:.0f} s")
    for lo, hi in ((0.05, 0.2), (0.2, 0.5), (0.5, 5.0), (0.05, 5.0)):
        m = (np.abs(qd) > lo) & (np.abs(qd) <= hi)
        if m.sum() < 200:
            continue
        A = np.c_[qd[m], np.sign(qd[m]), np.ones(m.sum())]
        (b, f, c0), *_ = np.linalg.lstsq(A, resid[m], rcond=None)
        ke = np.linalg.lstsq(np.c_[qd[m], np.ones(m.sum())], V[m] - R_CFG[j] * I[m], rcond=None)[0][0]
        gain = np.linalg.lstsq(qd[m, None], vobs[m], rcond=None)[0][0]
        print(f"   |qd| {lo:4.2f}-{hi:4.2f} rad/s ({m.mean()*100:4.1f} %):  residual = {b:+.3f}*qd {f:+.3f}*sign(qd) "
              f"{c0:+.3f} N*m   | Ke_true (R pinned {R_CFG[j]}) {ke:.3f} vs cfg {KE_CFG[j]:.3f}"
              f"  | v_obs/qdot gain {gain:.3f}")


if __name__ == "__main__":
    for r in (sys.argv[1:] or ["d1", "d2", "e1"]):
        run_one(r)
