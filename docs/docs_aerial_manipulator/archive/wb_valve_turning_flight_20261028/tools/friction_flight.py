#!/usr/bin/env python3
"""The valve's resisting torque, measured in flight.  Coulomb friction opposes the motion, so the
claw's tangential force (the law's attributed contact estimate F_hat, debug [97..99], and its
heading moment [100]) should flip sign with the valve's direction: tau_claw = r F_t + M_z ~
-tau_f sign(theta_dot) - I theta_ddot.  Fitted over the contact phase (jaws closed -> abort),
with a free constant (the estimate's slow bias).  Also the obj_0 centroid against its circle
(a real rocking would move it)."""
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import obj, step_times

R_CLAW = 0.160; I_ROT = 0.0031 * 204.0 / 243.0   # kg m^2: the CAD rotor at 204 g
AXIS = {"180652": (0.996, 0.011), "181833": (0.996, 0.009)}

def lp(t, y, wc):
    out = np.empty_like(y); out[0] = y[0]
    for i in range(1, len(y)):
        a = 1 - np.exp(-wc * (t[i] - t[i-1])); out[i] = out[i-1] + a * (y[i] - out[i-1])
    return out

rows = []
for tag in BAGS:
    b = Bag(tag); st = step_times(b); t, R, p = obj(b)
    T = np.load(f"../npz/turn_{tag}.npz"); g = T["g"]
    yaw = np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0]))
    th = np.interp(g, t, yaw); th_f = lp(g, th, 2*np.pi*4.0); th_f = lp(g, th_f[::-1], 2*np.pi*4.0)[::-1]   # backward pass on the uniform grid
    thd = np.gradient(th_f, g); thdd = np.gradient(lp(g, thd, 2*np.pi*3.0), g)
    tw = b.t("wb"); W = b.f("wb", "data"); ok = np.isfinite(W[:, 100]); tw, W = tw[ok], W[ok]
    F = np.stack([np.interp(g, tw, W[:, 97+i]) for i in range(3)], 1); M = np.interp(g, tw, W[:, 100])
    te = b.t("pl_current_ee"); ee = np.stack([b.f("pl_current_ee", f"pose.position.{c}") for c in "xy"], 1)
    ax = np.array(AXIS[tag]); phi = np.interp(g, te, np.unwrap(np.arctan2(ee[:, 1]-ax[1], ee[:, 0]-ax[0])))
    tang = np.stack([-np.sin(phi), np.cos(phi)], 1)
    Ft = np.sum(F[:, :2] * tang, 1)
    tau = R_CLAW * Ft + M                      # the claw's torque about the shaft (CCW +), external ON the claw
    t_on = st["push"] + 0.3; t_off = st.get("abort", g[-1])
    w = (g >= t_on) & (g <= t_off)
    for wc in (np.inf, 6.0, 2.0):
        # the SAME filter on both sides: a linear model survives it, and the filter's lag no
        # longer attenuates the sign regressor (25 reversals in 9 s)
        f = (lambda x: x) if not np.isfinite(wc) else (lambda x: lp(g, x, wc))
        y = f(tau); sg = f(np.sign(thd) * (np.abs(thd) > np.radians(0.5))); ad = f(thdd)
        mv = w & np.isfinite(y) & np.isfinite(ad)
        X = np.c_[sg[mv], ad[mv], np.ones(mv.sum())]
        c, res, *_ = np.linalg.lstsq(X, y[mv], rcond=None)
        e = y[mv] - X @ c; cov = np.linalg.inv(X.T @ X) * e.var()
        fwd = w & (thd > np.radians(2)); bwd = w & (thd < -np.radians(2)); stk = w & (np.abs(thd) < np.radians(1))
        # the same fit with the inertia FIXED at the CAD rotor (the free coefficient soaks up the
        # vehicle's own unmodelled claw dynamics)
        X2 = np.c_[sg[mv], np.ones(mv.sum())]; y2 = y[mv] + I_ROT * ad[mv]
        c2, *_ = np.linalg.lstsq(X2, y2, rcond=None); e2 = y2 - X2 @ c2
        cv2 = np.linalg.inv(X2.T @ X2) * e2.var()
        # effective sample count: the residual's correlation time (lag-1 autocorrelation)
        r1 = np.corrcoef(e2[:-1], e2[1:])[0, 1]; neff = mv.sum() * (1 - r1) / (1 + r1)
        se = np.sqrt(cv2[0, 0] * mv.sum() / max(neff, 1))
        print(f"    inertia fixed at CAD: tau_f = {-c2[0]*1e3:6.1f} mN.m  +- {np.sqrt(cv2[0,0])*1e3:4.1f} (white)  +- {se*1e3:5.1f} (correlated residual, n_eff {neff:.0f})")
        print(f"{tag} LP {wc:>4} rad/s: moving {mv.sum()*0.02:4.1f} s (fwd {fwd.sum()*0.02:.1f}, back {bwd.sum()*0.02:.1f}, stuck {stk.sum()*0.02:.1f})  "
              f"tau_f = {-c[0]*1e3:6.1f} +- {np.sqrt(cov[0,0])*1e3:5.1f} mN.m  inertia coef {-c[1]:.4f} (CAD {I_ROT:.4f})  bias {c[2]*1e3:+6.1f}  "
              f"mean tau fwd {tau[fwd].mean()*1e3:+6.1f} back {tau[bwd].mean()*1e3:+6.1f}  noise rms {e.std()*1e3:6.1f}")
    # the valve's own motion statistics
    print(f"   valve speed in contact: |theta_dot| p50 {np.degrees(np.median(np.abs(thd[w]))):.1f}, p95 {np.degrees(np.percentile(np.abs(thd[w]),95)):.1f}, "
          f"max {np.degrees(np.abs(thd[w]).max()):.1f} deg/s; reversals {int(np.sum(np.diff(np.sign(thd[w & (np.abs(thd)>np.radians(2))]))!=0))}")
    # centroid on its circle (real rocking)
    c0 = ax; rr = np.hypot(p[:, 0]-c0[0], p[:, 1]-c0[1]); wt = (t >= t_on) & (t <= t_off); wr = (t > 5) & (t < 12)
    print(f"   obj_0 centroid radius about the axis: rest {np.median(rr[wr])*1e3:.1f} mm (sd {rr[wr].std()*1e3:.2f}), "
          f"contact {np.median(rr[wt])*1e3:.1f} (sd {rr[wt].std()*1e3:.2f}, p-p {np.ptp(rr[wt])*1e3:.1f}); "
          f"z rest {np.median(p[wr,2])*1e3:.1f} sd {p[wr,2].std()*1e3:.2f}, contact sd {p[wt,2].std()*1e3:.2f} p-p {np.ptp(p[wt,2])*1e3:.1f} mm")
