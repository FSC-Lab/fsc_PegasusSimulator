#!/usr/bin/env python3
"""Second split of the 10-28 contact sway: the delivered lateral force = what the law COMMANDS
(u1 * commanded tilt) + its ATTITUDE TRACKING error (-u1 * e_R about the nose) + PX4's ATTITUDE
ESTIMATE error (u1 * (true - PX4) tilt).  Then the roll-moment balance that makes the tracking error:
the law's roll torque, the observer's consumed roll estimate d_hat_r.  Power = 1/2 |A_f||A_v| cos(phase)."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_valve_turning_flight_20261028", "tools"))
from energy_flow import lockin, fit_f, quatR      # noqa: E402  (runs energy_flow's report first)
from vt_common import Bag                          # noqa: E402

for tag, win in (("180652", (41.0, 47.5)), ("181833", (41.0, 44.0))):
    b = Bag(tag)
    tw = b.t("wb"); W = b.f("wb", "data"); ok = np.isfinite(W[:, 100]); tw, W = tw[ok], W[ok]
    g = np.arange(win[0], win[1], 0.004); tt = g - g[0]
    col = lambda i: np.interp(g, tw, W[:, i])
    tv = b.t("vatt"); qv = b.f("vatt", "q")
    yaw = np.unwrap(np.pi/2 - np.arctan2(2*(qv[:, 0]*qv[:, 3] + qv[:, 1]*qv[:, 2]), 1 - 2*(qv[:, 2]**2 + qv[:, 3]**2)))
    psi = np.interp(g, tv, yaw).mean(); n = np.array([-np.sin(psi), np.cos(psi)])
    e = np.stack([col(48) - col(45), col(49) - col(46)], 1) @ n
    k = np.ones(25) / 25; v = np.gradient(np.convolve(e, k, mode="same"), 0.004)
    f0 = fit_f(tt, e); Zv = lockin(tt, v, f0)
    tm = b.t("mocap"); qm = np.stack([b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"], 1)
    idx = lambda ts: np.clip(np.searchsorted(ts, g), 0, len(ts) - 1)
    zt = np.array([quatR(*qm[i])[:2, 2] for i in idx(tm)]) @ n
    # PX4 attitude is NED/FRD: convert its body z (down) in NED to the ENU thrust direction
    zp = []
    for i in idx(tv):
        Rn = quatR(*qv[i]); zned = -Rn[:, 2]                 # thrust (body -z_FRD) in NED
        zp.append([zned[1], zned[0]])                          # NED (N,E) -> ENU (E,N)
    zp = np.array(zp) @ n
    u1 = col(17); eRy = col(29)
    parts = {"delivered (true tilt, mocap)": u1 * zt,
             "  = law COMMAND  u1*(PX4 tilt + e_R,nose)": u1 * (zp + eRy),
             "  + attitude TRACKING error  -u1*e_R,nose": -u1 * eRy,
             "  + PX4 attitude ESTIMATE error u1*(true-PX4)": u1 * (zt - zp)}
    print(f"===== {tag} {win}: sway {abs(lockin(tt, e, f0))*1e3:.1f} mm at {f0:.3f} Hz")
    for nm, x in parts.items():
        Z = lockin(tt, x, f0); ph = np.degrees(np.angle(Z / Zv))
        print(f"   {nm:46s} amp {abs(Z):6.3f} N  phase {ph:+5.0f} deg  power {0.5*abs(Z)*abs(Zv)*np.cos(np.radians(ph))*1e3:+7.1f} mW")
    print(f"   check: amplitude of e_R,nose {np.degrees(abs(lockin(tt, eRy, f0))):.2f} deg, PX4-vs-mocap tilt error {np.degrees(abs(lockin(tt, zt - zp, f0))):.2f} deg")
    # roll moment balance (roll = about the nose: FLU x, model y)
    rr = np.gradient(np.convolve(np.arcsin(np.clip(zt, -1, 1)), k, mode="same"), 0.004)   # lateral tilt rate ~ -roll rate
    Zr = lockin(tt, rr, f0)
    print(f"   ROLL (lateral-tilt) rate amplitude {np.degrees(abs(Zr)):.1f} deg/s, phase vs lateral velocity {np.degrees(np.angle(Zr/Zv)):+.0f} deg")
    for nm, x in (("law roll torque  tau_FLU,x [21]", col(21)), ("observer roll estimate consumed d_hat_r,nose [35]", col(35)),
                  ("observer roll deadbeat d^c_r,nose [66]", col(66))):
        Z = lockin(tt, x, f0)
        print(f"   {nm:50s} amp {abs(Z):6.3f} N.m  phase vs lateral-tilt rate {np.degrees(np.angle(Z/Zr)):+5.0f} deg")
