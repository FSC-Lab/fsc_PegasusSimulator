#!/usr/bin/env python3
"""The wrist roll j4 under the two 2026-10-16 law fixes: a stiffer EE heading (wb_ky_psi / wb_dy_psi) and a stuck-joint
hold on the L1 observer's j4 row. Built on ../../q4_damping_20261016/tools/j4_model.py (validated on both 10-15 wrist
failures), with three changes, nothing fitted:
  * the law's PD on j4 comes from the heading gains through the exact law (joint_gains.py, every pose):
    K_q = 0.1735 K_psi, D_q = 0.1781 D_psi;
  * the observer feed-forward is the 4-D d_int^f j4 row, the residual low-passed at omega_x = 0.2072 rad/s (hardware yaml);
  * HOLD: while the joint is stuck -- |q_dot low-passed at TAU_V| < V_TH -- the feed-forward row keeps its state.
The arm controller is today's: relay 0.030 N*m (friction_ff[3] 4.455), error damping d 0.05.
    PYTHONNOUSERSITE=1 /usr/bin/python3 j4_study.py [validate|sweep|robust|margin]"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "q4_damping_20261016", "tools"))
import j4_model as M  # noqa: E402

KQ_PER, DQ_PER = 0.0431 / 0.2484, 0.0517 / 0.2903
SHIP = (0.2484, 0.2903)
RELAY, DAMP = 4.455 / M.KAPPA, 0.05
W_X = 0.2072


def simulate(T, ky_psi, dy_psi, hold=False, v_th=0.02, tau_v=0.25, loaded=False, q0=0.0, q_d_of_t=None, qd_of_t=None,
             f_s=M.F_S, f_k=M.F_K, b_v=M.B_V, delay=M.DELAY, w_ff=W_X, bias=0.0, force=0.0, f_force=0.51, j_scale=1.0,
             vel_lag=0.0, relay=RELAY, enc=M.ENC):
    K, D = KQ_PER * ky_psi, DQ_PER * dy_psi
    J = (0.025 if loaded else 0.0109) * j_scale
    Kp = 0.20 if loaded else 0.0
    n = int(T / M.DT)
    q, v = q0, 0.0
    qh, vh = (np.round(q0 / enc) * enc if enc > 0 else q0), 0.0
    d_ff, trim_nm, v_lp = 0.0, 0.0, 0.0
    buf = [0.0] * (delay + 1)
    vbuf = [0.0] * (int(round(vel_lag / M.DT)) + 1)
    stuck = True
    out = np.zeros((n, 7))
    a_v = np.exp(-M.DT / tau_v)
    for k in range(n):
        t = k * M.DT
        q_d = q_d_of_t(t) if q_d_of_t else 0.0
        qd_d = qd_of_t(t) if qd_of_t else 0.0
        qm = np.round(q / enc) * enc if enc > 0 else q
        e = qm - qh
        qh += M.DT * (vh + 2 * M.W_OBS * e)
        vh += M.DT * (M.W_OBS ** 2 * e)
        vbuf.append(vh); vlaw = vbuf.pop(0)
        tau_nom = -K * (q - q_d) - D * (vlaw - qd_d)
        buf.append(tau_nom - d_ff)
        tau_law = buf.pop(0)
        relay_t = relay * np.tanh(vh / 0.03)
        tau_int = tau_law + relay_t
        w_comp = vh - DAMP * (vh - qd_d)
        motor_res = M.KT_R * (M.KE_CFG * w_comp - M.KE_TRUE * v)
        tau_em = tau_int + trim_nm + motor_res
        err = tau_int - tau_em
        if abs(tau_int) < 5.0 / 153.4:
            cand = trim_nm * np.exp(-M.DT / 0.25) + 2 * np.pi * 0.2 * err * M.DT
            held = trim_nm * np.exp(-M.DT / 0.25)
            trim_nm = cand if abs(cand) < abs(held) else held
        else:
            trim_nm = np.clip(trim_nm + 2 * np.pi * 0.2 * err * M.DT, -0.25 * 57.6 / M.KAPPA, 0.25 * 57.6 / M.KAPPA)
        tau_ext = -Kp * np.sin(q) + force * np.sin(2 * np.pi * f_force * t) + bias
        drive = tau_em + tau_ext - b_v * v
        if stuck:
            if abs(drive) > f_s:
                stuck = False
            else:
                v = 0.0
        if not stuck:
            a = (drive - f_k * np.sign(v if v != 0 else drive)) / J
            v_new = v + a * M.DT
            if v != 0 and np.sign(v_new) != np.sign(v) and abs(drive) <= f_s:
                v_new, stuck = 0.0, True
            v = v_new
        q += v * M.DT
        # the observer's stuck test, on the LAW's velocity
        v_lp = a_v * v_lp + (1 - a_v) * vlaw
        held_now = hold and abs(v_lp) < v_th
        resid = (tau_em - tau_law) + tau_ext - b_v * v - (f_k * np.sign(v) if not stuck else drive)
        if not held_now:
            d_ff += M.DT * w_ff * (resid - d_ff)
        out[k] = (t, q, v, tau_em, d_ff, 1.0 if stuck else 0.0, 1.0 if held_now else 0.0)
    return out


def hunt(o, t_from=10.0):
    m = o[:, 0] >= t_from
    q = np.degrees(o[m, 1])
    # stick-slip events after t_from
    slides = np.flatnonzero(np.diff(o[m, 5]) < 0)
    return np.ptp(q), np.abs(q[-int(5 / M.DT):]).max(), len(slides), np.abs(np.degrees(o[m, 2])).max()


def scores(ky, dy, **kw):
    r = {}
    ou = simulate(90, ky, dy, q0=np.radians(-30), **kw)
    r["hunt_pp"], r["hunt_end"], r["hunt_slides"], r["hunt_vmax"] = hunt(ou)
    ob = simulate(60, ky, dy, q0=0.0, bias=0.010, **kw)               # q1-coupling push ~10 mN.m (flown)
    r["bias_pp"], r["bias_end"], r["bias_slides"], _ = hunt(ob)
    ol = simulate(40, ky, dy, loaded=True, q0=np.radians(15), **kw)
    m = ol[:, 0] > 10
    r["load_pp"] = np.degrees(np.ptp(ol[m, 1])); r["load_end"] = np.abs(np.degrees(ol[-int(5 / M.DT):, 1])).max()
    of = simulate(40, ky, dy, loaded=True, q0=np.radians(15), force=0.02, **kw)
    r["load_forced"] = np.degrees(np.ptp(of[of[:, 0] > 10, 1]))
    ramp = lambda t: np.radians(20.0) * np.clip(t - 1.0, 0, 3.0)
    rate = lambda t: np.radians(20.0) if 1.0 <= t < 4.0 else 0.0
    otr = simulate(15, ky, dy, q_d_of_t=ramp, qd_of_t=rate, **kw)
    err = np.degrees(otr[:, 1]) - np.degrees([ramp(t) for t in otr[:, 0]])
    r["roll_lag"] = np.abs(err[(otr[:, 0] > 2.0) & (otr[:, 0] < 4.0)]).max()
    r["roll_end"] = np.abs(err[otr[:, 0] > 12.0]).max()
    r["roll_os"] = max(0.0, (np.degrees(otr[otr[:, 0] > 4.0, 1]) - 60.0).max())
    return r


HDR = (f"{'K_psi':>6s} {'D_psi':>6s} {'hold':>4s} | {'unloaded kick -30: p-p':>22s} {'end':>5s} {'slips':>5s} | "
       f"{'10 mN.m bias: p-p':>17s} {'end':>5s} {'slips':>5s} | {'loaded: p-p':>11s} {'end':>5s} {'forced':>6s} | "
       f"{'roll 20/s: lag':>14s} {'overshoot':>9s} {'end':>5s}")


def row(ky, dy, hold, **kw):
    r = scores(ky, dy, hold=hold, **kw)
    return (f"{ky:6.3f} {dy:6.3f} {'on' if hold else 'off':>4s} | {r['hunt_pp']:20.1f}deg {r['hunt_end']:5.1f} {r['hunt_slides']:5d} | "
            f"{r['bias_pp']:15.1f}deg {r['bias_end']:5.1f} {r['bias_slides']:5d} | {r['load_pp']:9.1f}deg {r['load_end']:5.1f} "
            f"{r['load_forced']:6.1f} | {r['roll_lag']:12.1f}deg {r['roll_os']:9.1f} {r['roll_end']:5.1f}")


def dpsi_for(ky, zeta=1.2, J=0.0109):
    return 2 * zeta * np.sqrt(KQ_PER * ky * J) / DQ_PER


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "validate"
    if mode == "validate":
        print("TODAY (shipped heading, no hold) against 10-16 E-1: hunting ~40-43 deg p-p, ~10.5 s period, re-breaks at ~35 mN.m")
        for fs in (0.035, 0.045):
            o = simulate(90, *SHIP, q0=np.radians(-23), f_s=fs)
            pp, end, sl, vmax = hunt(o)
            st = np.flatnonzero(np.diff(o[:, 5]) < 0) * M.DT
            per = np.diff(st).mean() if len(st) > 2 else np.nan
            print(f"  static {fs}: p-p {pp:.1f} deg, slips {sl} (mean spacing {per:.1f} s), slide speed max {vmax:.0f} deg/s")
    elif mode == "sweep":
        print(HDR)
        for ky in (SHIP[0], 1.0, 1.5, 2.0, 2.5, 3.0, 4.0):
            dy = SHIP[1] if ky == SHIP[0] else dpsi_for(ky)
            for hold in (False, True):
                print(row(ky, dy, hold), flush=True)


VARIANTS = [("nominal", {}), ("static 0.035", dict(f_s=0.035)), ("static 0.060", dict(f_s=0.060)),
            ("kinetic -25%", dict(f_k=0.75 * M.F_K)), ("kinetic +25%", dict(f_k=1.25 * M.F_K)),
            ("viscous 0.010", dict(b_v=0.010)), ("viscous 0.030", dict(b_v=0.030)),
            ("transport 5 ticks", dict(delay=5)), ("+20 ms on q_dot", dict(vel_lag=0.020)),
            ("inertia x0.7", dict(j_scale=0.7)), ("inertia x1.5", dict(j_scale=1.5)),
            ("omega_x 0.6", dict(w_ff=0.6)), ("v_th 0.01", dict(v_th=0.01)), ("v_th 0.04", dict(v_th=0.04)),
            ("tau_v 0.10", dict(tau_v=0.10))]


def margin(ky, dy, hold=False):
    """Largest extra delay on BOTH the torque path and q_dot (4 ms steps) before the wrist's LINEAR loop rings: no
    Coulomb friction, no relay, no encoder (all nonlinear), viscous friction + motor residual + error damping kept. A
    5-degree step; rings = the velocity still reverses (>= 4 sign changes above 0.1 deg/s) in the last 3 s of 10 s, which
    a slowly settling soft loop does not do. The arm's ~8 ms transport and the 12 ms observer are already in the model."""
    for extra in range(0, 61):
        o = simulate(10, ky, dy, hold=hold, q0=np.radians(5.0), f_s=0.0, f_k=0.0, relay=0.0, enc=0.0,
                     delay=M.DELAY + extra, vel_lag=extra * M.DT)
        v = np.degrees(o[o[:, 0] > 7, 2])
        if not np.all(np.isfinite(v)):
            return extra * 4
        s_ = np.sign(v[np.abs(v) > 0.1])
        if np.count_nonzero(np.diff(s_)) >= 4:
            return extra * 4
    return ">240"


if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "robust":
    for ky in (2.0, 2.5, 3.0):
        dy = dpsi_for(ky)
        print(f"\n=== K_psi {ky}  D_psi {dy:.3f}  hold on")
        print(f"{'variant':18s} " + HDR.split('|', 1)[1])
        for name, kw in VARIANTS:
            print(f"{name:18s} " + row(ky, dy, True, **kw).split('|', 1)[1], flush=True)
if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "margin":
    for ky in (SHIP[0], 1.0, 1.5, 2.0, 2.5, 3.0, 4.0):
        dy = SHIP[1] if ky == SHIP[0] else dpsi_for(ky)
        print(f"K_psi {ky:6.3f} D_psi {dy:.3f}: extra delay before the linear wrist loop rings {margin(ky, dy)} ms", flush=True)
