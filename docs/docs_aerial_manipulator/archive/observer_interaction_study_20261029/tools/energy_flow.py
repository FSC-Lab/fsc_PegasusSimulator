#!/usr/bin/env python3
"""WHERE does the energy of the hardware contact sway come from?  10-28 valve flights, the window where the
lateral sway grows.  Lock-in at the sway's own frequency (fitted), every driver's amplitude and its phase
against the velocity it acts on; mean power = 1/2 A_f A_v cos(phi) (> 0 feeds the motion).
  base, lateral:  the ACTUAL thrust force across the nose  u1 * (R_true e3) . n   (mocap attitude)
                  split into the law's terms: -k_x e, -k_v e_dot, -d_hat_t (the observer), the rest
  arm, q1 / q4:   the law's commanded joint torque, and its observer-driven share (debug [93..96])
  the contact estimate the law renders: F_hat_y consumed (debug [58..61]) across the nose."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_valve_turning_flight_20261028", "tools"))
from vt_common import Bag                                  # noqa: E402
from valve_axis import step_times                          # noqa: E402

def lockin(t, x, f):
    A = np.column_stack([np.cos(2*np.pi*f*t), np.sin(2*np.pi*f*t), np.ones_like(t), t - t.mean()])
    c, *_ = np.linalg.lstsq(A, x, rcond=None)
    return complex(c[0], -c[1])                            # x ~ Re(Z e^{j w t})

def fit_f(t, x):
    best = None
    for f in np.arange(0.35, 0.85, 0.002):
        a = abs(lockin(t, x, f))
        best = (a, f) if best is None or a > best[0] else best
    return best[1]

def quatR(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

for tag, win in (("180652", (41.0, 47.5)), ("181833", (41.0, 44.0))):
    b = Bag(tag); st = step_times(b)
    tw = b.t("wb"); W = b.f("wb", "data"); ok = np.isfinite(W[:, 100]); tw, W = tw[ok], W[ok]
    g = np.arange(win[0], win[1], 0.004)
    col = lambda i: np.interp(g, tw, W[:, i])
    # the nose from PX4 yaw (ENU)
    tv = b.t("vatt"); qv = b.f("vatt", "q")
    yaw = np.unwrap(np.pi/2 - np.arctan2(2*(qv[:, 0]*qv[:, 3] + qv[:, 1]*qv[:, 2]), 1 - 2*(qv[:, 2]**2 + qv[:, 3]**2)))
    psi = np.interp(g, tv, yaw).mean(); n = np.array([-np.sin(psi), np.cos(psi)]); u = np.array([np.cos(psi), np.sin(psi)])
    ex = np.stack([col(48) - col(45), col(49) - col(46)], 1); e = ex @ n
    def smooth_d(x):
        k = np.ones(25) / 25; xs = np.convolve(x, k, mode="same"); return np.gradient(xs, 0.004)
    v = smooth_d(e)
    f0 = fit_f(g - g[0], e)
    tt = g - g[0]
    Zv = lockin(tt, v, f0); Ze = lockin(tt, e, f0)
    def report(name, x, Zref, unit):
        Z = lockin(tt, x, f0); ph = np.degrees(np.angle(Z / Zref)); P = 0.5 * abs(Z) * abs(Zref) * np.cos(np.radians(ph))
        print(f"   {name:46s} amp {abs(Z):7.3f} {unit:6s} phase vs velocity {ph:+6.0f} deg  -> mean power {P*1e3:+7.2f} mW")
        return P
    print(f"===== {tag} window {win}: lateral sway {abs(Ze)*1e3:.1f} mm amplitude at {f0:.3f} Hz (velocity {abs(Zv)*1e3:.0f} mm/s)")
    # the actual lateral thrust force from the TRUE (mocap) attitude
    tm = b.t("mocap"); qm = np.stack([b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"], 1)
    zt = np.array([quatR(*qm[min(np.searchsorted(tm, t), len(tm)-1)])[:2, 2] for t in g])
    u1 = col(17); F_lat = u1 * (zt @ n)
    report("BASE: actual thrust force across (u1 * tilt)", F_lat, Zv, "N")
    report("  law term -k_x e   (k_x 50.03)", -50.03 * e, Zv, "N")
    report("  law term -k_v e_dot (k_v 12.58)", -12.58 * v, Zv, "N")
    dt_ = np.stack([col(31), col(32)], 1) @ n
    report("  OBSERVER: -d_hat_t across (filtered, consumed)", -dt_, Zv, "N")
    dc = np.stack([col(62), col(63)], 1) @ n
    report("  observer deadbeat d_hat^c across (unfiltered)", dc, Zv, "N")
    fy = np.stack([col(58), col(59)], 1) @ n
    report("  F_hat_y consumed across (rendered on the impedance)", fy, Zv, "N")
    # the arm: q1 / q4
    for j, nm in ((0, "q1"), (3, "q4")):
        qj = col(5 + j); qdj = col(107 + j)
        Zq = lockin(tt, qdj, f0)
        print(f"   ARM {nm}: amplitude {np.degrees(abs(lockin(tt, qj, f0))):.1f} deg; rate {np.degrees(abs(Zq)):.0f} deg/s; phase of {nm} vs the lateral velocity "
              f"{np.degrees(np.angle(Zq / Zv)):+.0f} deg")
        report(f"     law joint torque tau_{nm} (published)", col(13 + j), Zq, "N.m")
        report(f"     its observer-driven share (applied, [93..96])", col(93 + j), Zq, "N.m")
