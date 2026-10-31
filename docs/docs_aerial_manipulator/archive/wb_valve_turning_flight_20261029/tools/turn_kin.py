#!/usr/bin/env python3
"""The turn in the valve's own polar frame (about the captured axis): planned angle (the claw
reference's azimuth), the valve's angle (obj_0 yaw), the claw's measured azimuth/radius (planner FK),
the base (CoM) error split tangential/radial/vertical, attitude error, joints, joint torques, and the
contact force estimates along the tangent: low-passed F_hat (debug [97..99]) and -K_y e_y.
Writes ../npz/turn_<tag>.npz on a common 50 Hz grid for the figures."""
import sys
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import obj, step_times

AXIS = {"145526": (0.855, 0.004)}   # planner log, the captured axis
KY, DY = 120.0, 20.2

def interp(tq, t, y):
    return np.interp(tq, t, y)

def lp(t, y, wc):
    out = np.empty_like(y); out[0] = y[0]
    for i in range(1, len(y)):
        a = 1 - np.exp(-wc * (t[i] - t[i-1])); out[i] = out[i-1] + a * (y[i] - out[i-1])
    return out

for tag in (sys.argv[1:] or BAGS):
    b = Bag(tag); st = step_times(b); ax = np.array(AXIS[tag])
    t0, t1 = st["handle"] - 2.0, st.get("abort", st["push"] + 20) + 4.0
    g = np.arange(t0, t1, 0.02)
    # valve
    to, R, p = obj(b); yaw = np.degrees(np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0])))
    th_v = interp(g, to, yaw) - np.interp(st["push"], to, yaw)
    # claw reference (streamed) and measured (planner FK)
    tr = b.t("wbref"); red = np.stack([b.f("wbref", f"r_ed.{c}") for c in "xyz"], 1)
    rr = red[:, :2] - ax; az_r = np.degrees(np.unwrap(np.arctan2(rr[:, 1], rr[:, 0])))
    az_ref = interp(g, tr, az_r); az_ref0 = np.interp(st["push"], tr, az_r); th_plan = az_ref - az_ref0
    rad_ref = interp(g, tr, np.linalg.norm(rr, axis=1))
    te = b.t("pl_current_ee"); ee = np.stack([b.f("pl_current_ee", f"pose.position.{c}") for c in "xyz"], 1)
    re_ = ee[:, :2] - ax; az_m = np.degrees(np.unwrap(np.arctan2(re_[:, 1], re_[:, 0])))
    th_claw = interp(g, te, az_m) - az_ref0; rad_m = interp(g, te, np.linalg.norm(re_, axis=1))
    z_claw = interp(g, te, ee[:, 2]); z_ref = interp(g, tr, red[:, 2])
    # controller internals
    tw = b.t("wb"); W = b.f("wb", "data"); ok = np.isfinite(W[:, 100]); tw, W = tw[ok], W[ok]
    def col(i): return interp(g, tw, W[:, i])
    ey = np.stack([col(24), col(25), col(26)], 1); eyh = col(27)
    xc = np.stack([col(48), col(49), col(50)], 1); xcd = np.stack([col(45), col(46), col(47)], 1)
    ec = xc - xcd
    eR = np.stack([col(28), col(29), col(30)], 1)
    q = np.degrees(np.stack([col(5+i) for i in range(4)], 1)); qd = np.degrees(np.stack([col(9+i) for i in range(4)], 1))
    tau = np.stack([col(13+i) for i in range(4)], 1)
    Fh = np.stack([col(97), col(98), col(99)], 1); Mh = col(100)
    chi = col(105)
    # tangent at the valve's own angle (CCW positive) from the claw's measured azimuth
    phi = np.radians(interp(g, te, az_m)); tang = np.stack([-np.sin(phi), np.cos(phi)], 1); radl = np.stack([np.cos(phi), np.sin(phi)], 1)
    Fh_lp = np.stack([lp(g, Fh[:, i], 2.0) for i in range(3)], 1)
    Ft_hat = np.sum(Fh_lp[:, :2] * tang, 1); Fr_hat = np.sum(Fh_lp[:, :2] * radl, 1)
    Ft_spr = -KY * np.sum(ey[:, :2] * tang, 1)
    ec_t = np.sum(ec[:, :2] * tang, 1); ec_r = np.sum(ec[:, :2] * radl, 1)
    # base yaw (PX4 attitude), model
    tv = b.t("vatt"); qv = b.f("vatt", "q")   # px4 quaternion w,x,y,z NED/FRD
    tilt_b = None
    np.savez(f"../npz/turn_{tag}.npz", g=g, th_v=th_v, th_plan=th_plan, th_claw=th_claw, rad_ref=rad_ref, rad_m=rad_m,
             z_claw=z_claw, z_ref=z_ref, ey=ey, eyh=eyh, ec=ec, ec_t=ec_t, ec_r=ec_r, eR=eR, q=q, qd=qd, tau=tau,
             Ft_hat=Ft_hat, Fr_hat=Fr_hat, Fz_hat=Fh_lp[:, 2], Ft_spr=Ft_spr, Mh=Mh, chi=chi,
             t_handle=st["handle"], t_push=st["push"], t_turn=st["push"] + 1.5, t_abort=st.get("abort", np.nan))
    print(f"===== {tag}  push {st['push']:.2f}  turn {st['push']+1.5:.2f}  abort {st.get('abort', np.nan):.2f}")
    print("   t    plan  valve claw | rad_ref rad_m  dz_claw | e_y t/r/z mm      | CoM err t/r/z mm   |eR|  | q1 q2 q3 q4        | Ft_hat Ft_spr  Fz_hat")
    for tt in np.arange(st["handle"] - 0.5, min(t1, st.get("abort", t1) + 2.6), 0.25):
        i = np.searchsorted(g, tt)
        if i >= len(g): break
        ty = np.sum(ey[i, :2] * tang[i]) * 1e3; ry = np.sum(ey[i, :2] * radl[i]) * 1e3
        print(f"  {g[i]:6.2f} {th_plan[i]:6.1f} {th_v[i]:6.1f} {th_claw[i]:6.1f} | {rad_ref[i]*1e3:5.0f} {rad_m[i]*1e3:5.0f} {(z_claw[i]-z_ref[i])*1e3:+6.1f} |"
              f" {ty:+6.1f} {ry:+6.1f} {ey[i,2]*1e3:+6.1f} | {ec_t[i]*1e3:+6.1f} {ec_r[i]*1e3:+6.1f} {ec[i,2]*1e3:+6.1f} |"
              f" {np.linalg.norm(eR[i]):.3f} | {q[i,0]:+5.1f} {q[i,1]:+5.1f} {q[i,2]:+5.1f} {q[i,3]:+6.1f} | {Ft_hat[i]:+5.2f} {Ft_spr[i]:+5.2f} {Fh_lp[i,2]:+5.2f}")
