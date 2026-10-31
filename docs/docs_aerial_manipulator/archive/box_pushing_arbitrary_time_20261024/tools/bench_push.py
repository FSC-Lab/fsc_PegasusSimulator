#!/usr/bin/env python3
"""bench_push.py -- the push axis of the box-pushing task under the
arbitrary-time quadrotor law, to rank the J_E feed-forward gain against the
position stiffness and the fuzzy integral before an Isaac flight.

One axis (the push direction, y): the base is a point mass with this law's
position row (the backstepping with the floored time-to-go, the TSKFE as the
integral, the J_E feed-forward of the estimated contact force); the horizontal
force reaches the base through the attitude loop and the rotors (2nd order at
wn_att, zeta 1, then the 99.7 ms rotor lag). The arm is a spring-damper K_a/D_a
between the base's claw point and the clamped box (the admittance + the arm law's
tracking in series); the box is 0.5 kg with Coulomb friction mu (static = kinetic,
the scene's), regularised with a stick band. The observer's estimate is the arm
force through a 2nd-order filter at wn_obs plus the 36 ms arm actuation delay.
The grasp leaves a preload (the arm already pressing/pulling the fin).

The reference: hold for the 1.5 s settle, then the planner's min-snap 0.5 m in 24 s.
Scored like the planner's box-lag guard (box behind its plan > 40 mm aborts).

    PYTHONNOUSERSITE=1 /usr/bin/python3 bench_push.py
"""
import itertools
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, *[".."] * 5))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_controller import arbitrary_time_control as A  # noqa: E402

DT = 0.004


def minsnap(s):
    s = min(max(s, 0.0), 1.0)
    return s ** 5 * (126 - 420 * s + 540 * s ** 2 - 315 * s ** 3 + 70 * s ** 4)


def run(ff=1.0, eta=1.04, floor=0.6, c=20.9, Ka=144.0, Da=8.0, wn_obs=5.0, wn_att=7.0, mu=0.25, mb=0.5,
        preload=-0.5, T=30.0, settle=1.5, dist=0.5, Tpush=24.0, delay_obs=0.036):
    m = A.QuadConfig().mass
    y = v = 0.0                      # base (push axis; the push goes +y here)
    yb = vb = 0.0                    # box
    f1 = f1d = 0.0                   # attitude-loop state (2nd order on the commanded force)
    f_act = 0.0                      # after the rotor lag
    fo = fod = 0.0                   # observer filter state
    buf = []
    nd = int(round(delay_obs / DT))
    fz = A.TskFuzzyEstimator(A.FuzzyConfig(c=np.array([c, c, c, 0.1, 0.1, 0.1])))
    x1 = x2 = 0.0
    win = A.Window(1.2, floor)
    win.trigger(-10.0)
    off = preload / Ka              # the grasp preload as an initial spring stretch
    lag_max = 0.0
    trip = None
    back = 0.0
    for k in range(int(T / DT)):
        t = k * DT
        s = (t - settle) / Tpush
        yd = dist * minsnap(s)
        vd = dist * (minsnap(min(s + 1e-4, 1.0)) - minsnap(s)) / (1e-4 * Tpush) if 0 < s < 1 else 0.0
        # arm force on the base (spring stretched by box - base - off) == minus the force on the box
        stretch = (yb - y) - off
        F_arm_on_base = Ka * stretch + Da * (vb - v)
        F_env_on_ee = F_arm_on_base          # quasi-static: what the arm transmits to the base
        buf.append(F_env_on_ee)
        Fm = buf[max(0, len(buf) - 1 - nd)]
        fod += DT * (wn_obs ** 2 * (Fm - fo) - 2 * wn_obs * fod)
        fo += DT * fod
        # the law (the y row of quad_law, fuzzy on the last tick's errors)
        D, ph, X = fz.evaluate(x1, x2)
        tgo = win.tgo(t)
        r1 = yd - y
        r2 = vd - v
        delta = -eta * A.phi(np.array([r1]))[0] / tgo
        eps = r2 - delta
        N = eta * (A.dphi(np.array([r1]))[0] * r2 / tgo) + eta * A.phi(np.array([eps]))[0] / tgo
        Fcmd = m * (r1 + N) - ff * fo + D[1]
        g = np.zeros(6)
        g[1] = eps / m
        fz.adapt(DT, g, ph, X)
        x1, x2 = abs(r1), abs(r2)
        # force realisation through the attitude loop + the rotor lag
        f1d += DT * (wn_att ** 2 * (Fcmd - f1) - 2 * wn_att * f1d)
        f1 += DT * f1d
        f_act = f1 + (f_act - f1) * math.exp(-10.0265 * DT)
        # plant
        a = (f_act + F_arm_on_base) / m
        v += DT * a
        y += DT * v
        F_on_box = -F_arm_on_base
        Ffric_max = mu * mb * 9.81
        if abs(vb) < 1e-4 and abs(F_on_box) <= Ffric_max:
            vb = 0.0
        else:
            fr = -Ffric_max * (np.sign(vb) if abs(vb) >= 1e-4 else np.sign(F_on_box))
            vb_new = vb + DT * (F_on_box + fr) / mb
            vb = 0.0 if vb != 0.0 and np.sign(vb_new) != np.sign(vb) else vb_new
        yb += DT * vb
        if t > settle:
            lag = yd - yb
            lag_max = max(lag_max, lag)
            if trip is None and lag > 0.040:
                trip = t - settle
        back = max(back, -yb)
    return dict(lag_max=lag_max, trip=trip, back=back, end=yb)


def main():
    print("ff   eta   c     Ka   preload | box lag max [mm]  trip [s into push]  box backward [mm]  box end [mm]")
    for ff, eta, c, Ka, pre in itertools.product((1.0, 0.75, 0.5, 0.25, 0.0), (1.04, 1.5), (20.9, 40.0), (144.0,),
                                                 (-0.5, 0.0, 0.5)):
        r = run(ff=ff, eta=eta, c=c, Ka=Ka, preload=pre)
        print(f"{ff:4.2f} {eta:4.2f} {c:5.1f} {Ka:5.0f} {pre:+5.1f}   | {1e3 * r['lag_max']:8.1f}          "
              f"{('%.1f' % r['trip']) if r['trip'] is not None else '  -':>6}            {1e3 * r['back']:7.1f}         {1e3 * r['end']:7.1f}")


if __name__ == "__main__":
    main()
