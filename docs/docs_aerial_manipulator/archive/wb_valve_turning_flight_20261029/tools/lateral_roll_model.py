#!/usr/bin/env python3
"""Reduced linear model of the 10-28 contact sway, for RANKING knobs (not a prediction of the flight).

States: base sideways offset x (model x = right), its rate, base roll th about the nose (+th tilts the
thrust toward +x), its rate, the delivered roll torque (rotor lag), a first-order Pade delay state, the
translational observer's sideways estimate, the rotational observer's roll estimate.

  m x''  = u1 th                                          (thrust tilt; u1 = m g)
  th_c   = -(k_x x + k_v x' + dt_hat) / u1                (the CoM loop's commanded tilt)
  tau_c  = (I/I_d) (-k_R (th - th_c) - k_w (th' - th_c')) - dr_hat      (inertia-shaped attitude law)
  tau_a  = rotor lag lambda / (s + lambda) of the delayed tau_c
  I th'' = tau_a + M,     M = -k_s (th + c x)             (the clamped fin resists the claw's WORLD roll;
                                                            c = claw roll per metre of sideways offset)
  dt_hat' = w_t (u1 th - (-(k_x x + k_v x')) - dt_hat),   dr_hat' = w_r (tau_a + M - tau_c - dr_hat)

c comes from roll_coupling_screen.py (0.155 deg/mm at the flown pose). k_s, the grip + shaft roll
stiffness in series, is not measured: it is CALIBRATED so the model reproduces the flown mode
(3.1 rad/s growing at ln2 / ~2.5-5 s). Then every knob is ranked by the growth rate it gives."""
import numpy as np

BASE = dict(m=3.746, I=0.132, I_d=0.139992, k_x=50.03, k_v=12.58, k_R=1.6, k_w=1.2, lam=10.0265, T=0.025,
            w_t=2.0, w_r=0.7428, k_s=0.0, c=np.radians(0.155) / 1e-3)


def A_of(P):
    """Linearised about hover, sideways channel only, from wb_controller.cpp lines 128-189:
    f_d = -k_x e_x - k_v e_v (- d_t, ~0 here: no external sideways force); the model acceleration, jerk
    and the 2-level attitude feed-forward  u2 = M_r (th_c'' - (k_R e_R + k_w e_w)/M_r_d) - d_r."""
    g = 9.81; u1 = P["m"] * g; n = 7
    def f(s):
        x, xd, th, thd, ta, z, dr = s
        xdd = u1 * th / P["m"]
        thc = -(P["k_x"] * x + P["k_v"] * xd) / u1
        thcd = -(P["k_x"] * xd + P["k_v"] * xdd) / u1
        thcdd = -(P["k_x"] * xdd + P["k_v"] * (u1 / P["m"]) * thd) / u1
        tc = P["I"] * (thcdd - (P["k_R"] * (th - thc) + P["k_w"] * (thd - thcd)) / P["I_d"]) - dr
        tdel = 2 * z - tc
        M = -P["k_s"] * (th + P["c"] * x)
        return np.array([xd, xdd, thd, (ta + M) / P["I"], P["lam"] * (tdel - ta), (2 / P["T"]) * (tc - z),
                         P["w_r"] * (ta + M - tc - dr)])
    A = np.zeros((n, n)); e = 1e-6
    for j in range(n):
        d = np.zeros(n); d[j] = e
        A[:, j] = (f(d) - f(-d)) / (2 * e)
    return A


def mode(P):
    """(growth rate 1/s, frequency rad/s) of the least-damped oscillatory mode below 2 Hz, and the max real part."""
    ev = np.linalg.eigvals(A_of(P))
    osc = [l for l in ev if l.imag > 0.5 and l.imag < 12.0]
    l = max(osc, key=lambda v: v.real)
    return l.real, l.imag, max(ev.real)


def with_(**kw):
    P = dict(BASE); P.update(kw); return P


if __name__ == "__main__":
    print("free flight (k_s = 0): growth %+.3f 1/s at %.2f rad/s" % mode(BASE)[:2])
    print("\ncalibration: k_s (grip+shaft roll stiffness, N.m/rad) that reproduces the flown mode (3.1 rad/s, doubling 2.5-5 s = +0.14..+0.28 1/s)")
    for ks in (1, 2, 3, 4, 5, 6, 8, 10):
        s, w, _ = mode(with_(k_s=ks))
        print(f"   k_s {ks:4.1f}: growth {s:+.3f} 1/s ({'doubling %.1f s' % (np.log(2)/s) if s > 0 else 'decays'}) at {w:.2f} rad/s ({w/2/np.pi:.2f} Hz)")
