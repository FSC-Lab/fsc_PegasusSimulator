#!/usr/bin/env python3
"""Offline model of the WRIST ROLL j4 in whole-body DIRECT, built from the 10-15 flight identification, to
choose the arm controller's j4 compensation terms before they fly.

    PYTHONNOUSERSITE=1 /usr/bin/python3 j4_model.py            (validation + candidate table)

PLANT (measured, analysis/j4_friction_id.txt, ident_motor.txt, j4_net_damping.txt)
  J      0.0109 kg m^2 unloaded (armature 0.0097 + gripper); loaded 0.025 with K_pend 0.20 N*m/rad, which
         gives the flown 0.51 Hz swing (D-1)
  friction  static 0.045, kinetic 0.0335 N*m, viscous 0.020 N*m*s/rad (unloaded slides, R2 0.97)
  motor  Kt/R = 2.4234/4.88 per A/V; true Ke 2.535 vs configured 2.570 (the feed-forward over-cancels 1.4 %)
SENSING  encoder 0.0879 deg/count; the arm controller's 20 Hz, zeta 1 Luenberger observer (v_hat)
LAW (whole-body, as fitted on D-1, analysis of 10-15 j4_law_gain.txt)
  tau_law = -K_law*(q - q_d) - D_law*(v_hat - qd_d) - d_ff,   K_law 0.05 N*m/rad, D_law 0.045 N*m*s/rad,
  2 ticks (8 ms) of transport; d_ff = the L1 joint-row estimate of everything the law does not model
  (friction left over, the relay, the motor's residual, the payload), low-passed at W_FF rad/s (0.3: the
  ramp measured while j4 was stuck in D-2)
ARM CONTROLLER (external_torque_controller, pass-through)
  relay   f_r * tanh(v_hat / 0.03)                                   (friction_ff[3] / kappa_pwm)
  duty    tau + s*Ke_cfg*w_comp (+ trim),  w_comp = v_hat - d*(v_hat - qd_ref)   <- d = the NEW knob
  trim    0.2 Hz integral on the current error, deadband 0.0326 N*m (5 current counts), leak 0.25 s
  so the motor delivers tau_em = tau + trim + (Kt/R)*(s*Ke_cfg*w_comp - Ke_true*qdot)
"""
import numpy as np

DT = 0.004
KT_R = 2.4234 / 4.88
KE_TRUE, KE_CFG = 2.535, 2.570
F_S, F_K, B_V = 0.045, 0.0335, 0.020
K_LAW, D_LAW, DELAY = 0.05, 0.045, 2
W_FF = 0.3
ENC = np.radians(0.0879)
W_OBS = 2 * np.pi * 20.0
KAPPA = 148.51


def simulate(T=60.0, f_r=7.7776 / KAPPA, d=0.0, s=1.0, loaded=False, q0=0.0, qd_of_t=None, q_d_of_t=None,
             force=0.0, f_force=0.51, trim=True, seed=0):
    rng = np.random.default_rng(seed)
    J = 0.025 if loaded else 0.0109
    Kp = 0.20 if loaded else 0.0
    n = int(T / DT)
    q, v = q0, 0.0
    qh, vh = np.round(q0 / ENC) * ENC, 0.0   # observer, started on the encoder grid (no start-up spike)
    d_ff = 0.0
    trim_nm = 0.0
    buf = [0.0] * (DELAY + 1)
    stuck = True
    out = np.zeros((n, 6))
    for k in range(n):
        t = k * DT
        q_d = q_d_of_t(t) if q_d_of_t else 0.0
        qd_d = qd_of_t(t) if qd_of_t else 0.0
        # observer on the quantised encoder
        qm = np.round(q / ENC) * ENC
        e = qm - qh
        qh += DT * (vh + 2 * W_OBS * e)
        vh += DT * (W_OBS ** 2 * e)
        # law (with transport) and the arm controller
        tau_nom = -K_LAW * (q - q_d) - D_LAW * (vh - qd_d)
        buf.append(tau_nom - d_ff)
        tau_law = buf.pop(0)
        relay = f_r * np.tanh(vh / 0.03)
        tau_int = tau_law + relay
        w_comp = vh - d * (vh - qd_d)
        motor_res = KT_R * (s * KE_CFG * w_comp - KE_TRUE * v)
        tau_em = tau_int + trim_nm + motor_res
        if trim:
            err = tau_int - tau_em                     # current error, in N*m
            if abs(tau_int) < 5.0 / 153.4:
                cand = trim_nm * np.exp(-DT / 0.25) + 2 * np.pi * 0.2 * err * DT
                held = trim_nm * np.exp(-DT / 0.25)
                trim_nm = cand if abs(cand) < abs(held) else held
            else:
                trim_nm = np.clip(trim_nm + 2 * np.pi * 0.2 * err * DT, -0.25 * 57.6 / KAPPA, 0.25 * 57.6 / KAPPA)
        tau_ext = -Kp * np.sin(q) + force * np.sin(2 * np.pi * f_force * t)
        drive = tau_em + tau_ext - B_V * v
        # stick / slip
        if stuck:
            if abs(drive) > F_S:
                stuck = False
            else:
                v = 0.0
        if not stuck:
            a = (drive - F_K * np.sign(v if v != 0 else drive)) / J
            v_new = v + a * DT
            if v != 0 and np.sign(v_new) != np.sign(v) and abs(drive) <= F_S:
                v_new, stuck = 0.0, True
            v = v_new
        q += v * DT
        # the L1 joint feed-forward: everything not in the law's own model
        resid = (tau_em - tau_law) + tau_ext - B_V * v - (F_K * np.sign(v) if not stuck else drive)
        d_ff += DT * W_FF * (resid - d_ff)          # the estimate tracks the residual; the law subtracts it
        out[k] = (t, q, v, tau_em, d_ff, 1.0 if stuck else 0.0)
    return out


def hunt_stats(o, t_from=10.0):
    m = o[:, 0] >= t_from
    q = np.degrees(o[m, 1]); v = np.degrees(o[m, 2]); st = o[m, 5]
    return dict(pp=np.ptp(q), peak=np.abs(q).max(), vmax=np.abs(v).max(), stuck=st.mean() * 100)


def main():
    print("VALIDATION against the 10-15 flights (shipped terms: relay 0.052 N*m, d 0, s 1)")
    o = simulate(T=60, q0=np.radians(-30))
    h = hunt_stats(o)
    print(f"  unloaded, kicked to -30 deg: q4 p-p {h['pp']:.0f} deg, |q4| max {h['peak']:.0f}, slide speed max "
          f"{h['vmax']:.0f} deg/s, stuck {h['stuck']:.0f} % of the time")
    print("     flown (D-1 after the drop, D-2 after the pull-out): sticks for 0.6-3.4 s at +-20..60 deg, jumps"
          " 60-90 deg at up to ~47 deg/s")
    for F in (0.0, 0.01, 0.02):
        o = simulate(T=60, loaded=True, force=F, q0=np.radians(15))
        print(f"  loaded, kicked to 15 deg, forced {F} N*m at 0.51 Hz: q4 p-p over 20-60 s "
              f"{np.degrees(np.ptp(o[o[:,0]>20,1])):.0f} deg (flown D-1 38-58 s: 19-26 deg p-p per 2 s)")


if __name__ == "__main__":
    main()


def scores(f_r, d, **kw):
    """One candidate: the loaded swing, the unloaded hunting, and what the extra damping costs in tracking."""
    o = simulate(T=60, f_r=f_r, d=d, loaded=True, q0=np.radians(15), **kw)
    m = o[:, 0] > 20
    lp = np.degrees(np.ptp(o[m, 1]))
    big = np.where(np.abs(np.degrees(o[:, 1])) > 2.0)[0]
    t2 = o[big[-1], 0] if len(big) else 0.0
    of = simulate(T=60, f_r=f_r, d=d, loaded=True, q0=np.radians(15), force=0.02, **kw)
    lpf_ = np.degrees(np.ptp(of[of[:, 0] > 20, 1]))
    ou = simulate(T=60, f_r=f_r, d=d, q0=np.radians(-30), **kw)
    hu = hunt_stats(ou)
    # tracking: a teleop roll of 20 deg/s for 3 s from rest (the planner's reference ramps q_d and qd_d)
    ramp = lambda t: np.radians(20.0) * np.clip(t - 1.0, 0, 3.0)
    rate = lambda t: np.radians(20.0) if 1.0 <= t < 4.0 else 0.0
    otr = simulate(T=12, f_r=f_r, d=d, q_d_of_t=ramp, qd_of_t=rate, **kw)
    err = np.degrees(otr[:, 1]) - np.degrees([ramp(t) for t in otr[:, 0]])
    lag = np.abs(err[(otr[:, 0] > 2.0) & (otr[:, 0] < 4.0)]).max()
    fin = np.abs(err[otr[:, 0] > 9.0]).max()
    return dict(load_pp=lp, load_t2=t2, load_forced=lpf_, hunt_pp=hu["pp"], hunt_stuck=hu["stuck"],
                hunt_vmax=hu["vmax"], ramp_lag=lag, ramp_final=fin)


def table(cands, **kw):
    print(f"{'relay N*m':>9s} {'d':>5s} | {'loaded: p-p':>11s} {'<2deg after':>11s} {'forced p-p':>10s} | "
          f"{'unloaded hunt p-p':>17s} {'stuck%':>6s} | {'roll 20deg/s: max err':>21s} {'err at 9s':>9s}")
    for f_r, d in cands:
        r = scores(f_r, d, **kw)
        print(f"{f_r:9.4f} {d:5.2f} | {r['load_pp']:9.1f}deg {r['load_t2']:9.1f} s {r['load_forced']:8.1f}deg | "
              f"{r['hunt_pp']:15.1f}deg {r['hunt_stuck']:6.0f} | {r['ramp_lag']:19.1f}deg {r['ramp_final']:7.1f}deg")


if __name__ == "__main__":
    print("\nCANDIDATES (relay = friction_ff[3]/148.51; d = fraction of the motor's own damping kept on the "
          "velocity ERROR in the pass-through)")
    SHIP, FK = 7.7776 / KAPPA, F_K
    table([(SHIP, 0.0), (FK, 0.0), (SHIP, 0.05), (FK, 0.03), (FK, 0.05), (FK, 0.08), (FK, 0.12), (FK, 0.20)])
