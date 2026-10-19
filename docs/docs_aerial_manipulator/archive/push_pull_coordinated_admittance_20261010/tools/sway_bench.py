#!/usr/bin/env python3
"""sway_bench.py -- the coordinated-admittance rig's base loop on a rigid-body
plant: the real flight law (coordinated_admittance.flight_law with the integral),
a 3-D rigid body (m 3.746, the yaml's inertia), first-order rotor lag on thrust
and torque (lambda 10.03 1/s), the mirror plant's body-fixed bias force, mocap
noise, the planner node's MPC in its flown 'full' mode at 20 Hz with the EE
target fixed. A 1 N / 0.5 s lateral kick at t = 6 s; the sway mode's damping is
read off the base-position log decrement. Used 2026-10-10 to choose the PD
pair after cadm_06/07 wobbled +-10..40 mm at 0.5 Hz.

    /usr/bin/python3 sway_bench.py            # the grid
"""
import math, sys, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "..", "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_controller import coordinated_admittance as CA

Q_HOME = np.array([0.0, 0.6981317, 0.6981317, 0.0])
I_B = np.array([[0.118, 0.0089, 0.0], [0.0089, 0.119, 0.0], [0.0, 0.0, 0.131]])


def run(kpp, kpd, kpi, keta_p, keta_d, lam=10.0265, T=14.0, kick=1.0, seed=0, verbose=False, int_max=0.5, accff=True, delay_s=0.0,
        keta_i=(0.0, 0.0, 0.0), int_eta_max=0.2, leash_m=0.15, leash_rad=0.2, v0=(0.0, 0.0, 0.0),
        k_tau=0.0, tau_lpf=0.1, tau_kick=0.0):
    """keta_i: addition (j), the attitude integral; leash_*: the planner's
    full-mode leash (2026-10-10, replaces the jump re-sync); v0: an initial
    base velocity (the DIRECT-entry surge of cadm_07/08 was 0.3 m/s)."""
    rng = np.random.default_rng(seed)
    cfg = CA.FlightLawConfig(kpp=kpp, kpd=kpd, kpi=[kpi] * 3, int_max=int_max, keta_p=keta_p, keta_d=keta_d, inertia=I_B,
                             keta_i=list(keta_i), int_eta_max=int_eta_max)
    mcfg = CA.MpcConfig(k_p=4.0, u_max=np.array([0.6] * 3 + [3.0] * 4), du_max=np.array([0.3] * 3 + [3.0] * 4),
                        v_max=np.array([0.4] * 3 + [1.0] * 4), ee_body_min=np.array([0.05, -0.25, -0.35]), ee_body_max=np.array([0.35, 0.25, 0.10]))
    mpc = CA.CoordinatedMpc(mcfg)
    adm = CA.AdmittanceFilter(0.5, [17.3, 17.3, 24.5], [150.0, 150.0, 300.0], 0.15)
    m = cfg.mass
    p = np.array([0.1, -0.07, 1.3]); v = np.array(v0, float); R = np.eye(3); om = np.zeros(3)
    q = Q_HOME.copy(); qd = np.zeros(4)
    f_act = m * CA.G; tau_act = np.zeros(3)
    F_bias = np.array([0.55, -0.50, -1.4]); tau_bias = np.array([0.0, 0.2, -0.095])   # arm CoM pitch moment + yaw bias
    _, e0 = CA.arm_fk_actual(Q_HOME); p_o = p + e0
    dt = 0.004; last = None; up = np.zeros(7); e_int = np.zeros(3); e_eta_int = np.zeros(3); pmeas = p.copy(); vmeas = v.copy()
    n_leash = 0; n_unconv = 0; solve_ms = []
    tau_hat_f = np.zeros(3)
    pd = p.copy(); vd = np.zeros(3); ad = np.zeros(3); qcmd = q.copy()
    est = CA.MomentumEstimator(m, I_B, 20.0, 5.0); est.reset(np.zeros(6)); f_hat = np.zeros(3)
    log_t, log_p, log_tilt = [], [], []
    nd = int(round(delay_s / dt)); hist = []     # feedback transport delay (position, velocity, attitude, rate)
    Rm, omm = R.copy(), om.copy()
    for k in range(int(T / dt)):
        t = k * dt
        hist.append((p.copy(), v.copy(), R.copy(), om.copy()))
        pdl, vdl, Rm, omm = hist[max(0, len(hist) - 1 - nd)]
        if k % 4 == 0:
            pmeas = pdl + rng.normal(0, 5e-4, 3); vmeas = vdl + rng.normal(0, 0.022, 3)
        if k % 12 == 0:
            p_r, v_r, a_r = adm.update(f_hat, p_o, np.zeros(3), np.zeros(3), 0.05)
            w, _ = CA.weight_vector(f_hat, 3.0, 2.0, CA.ARM_M[1:], xy_only=True)
            if last is None:
                x0 = np.concatenate([pmeas, q, vmeas, qd])
            else:
                pc, la = CA.leash(last["h1"][:3], pmeas, leash_m)
                qc = np.clip(last["h1"][3:], q - leash_rad, q + leash_rad)
                n_leash += int(la or np.any(np.abs(qc - last["h1"][3:]) > 0))
                x0 = np.concatenate([pc, qc, last["hdot1"]])
            import time as _t; _t0 = _t.perf_counter()
            out = mpc.solve(x0, Rm, p_r, v_r, 0.0, 0.0, Q_HOME, w, up); solve_ms.append((_t.perf_counter() - _t0) * 1e3)
            if out["info"]["converged"] or last is None:
                up = out["u0"]; last = out
                pd, vd, ad, qcmd = out["h1"][:3], out["hdot1"][:3], (out["u0"][:3] if accff else np.zeros(3)), out["h1"][3:]
            else:
                n_unconv += 1; vd = np.zeros(3); ad = np.zeros(3)
        qd = (qcmd - q) / 0.1; q = q + qd * dt
        law = CA.flight_law(cfg, pmeas, vmeas, Rm, omm, pd, vd, ad, 0.0, e_int, e_eta_int,
                            k_tau * tau_hat_f if k_tau > 0 else None)
        e_int = np.clip(e_int + law["e_p"] * dt, -int_max, int_max)
        e_eta_int = np.clip(e_eta_int + law["e_eta"] * dt, -int_eta_max, int_eta_max)
        # rotor lag on the commanded thrust and torque (exact ZOH)
        a_l = math.exp(-lam * dt)
        f_act = law["f_t"] + (f_act - law["f_t"]) * a_l
        tau_act = law["tau_body"] + (tau_act - law["tau_body"]) * a_l
        eta = law["eta"]; eta_dot = law["eta_dot"]
        u = est.applied_force(f_act, Rm, eta, eta_dot, CA.euler_w(eta).T @ tau_act)
        chi = est.momentum(vmeas, eta, eta_dot)
        r_hat = est.update(u, chi, dt); f_hat = r_hat[:3]
        a_ = math.exp(-dt / tau_lpf) if tau_lpf > 0 else 0.0
        tau_hat_f = a_ * tau_hat_f + (1 - a_) * r_hat[3:]
        Fk = np.array([0.0, kick, 0.0]) if 6.0 <= t < 6.5 else np.zeros(3)
        acc = (f_act * (R @ CA.E3) - m * CA.G * CA.E3 + F_bias + Fk) / m
        Tk = np.array([0.0, tau_kick, 0.0]) if (6.0 <= t < 6.5 or (tau_kick < 0 and t >= 6.0)) else np.zeros(3)
        om_dot = np.linalg.solve(I_B, tau_act - np.cross(om, I_B @ om) + tau_bias + Tk)
        v = v + acc * dt; p = p + v * dt; om = om + om_dot * dt
        R = R @ (np.eye(3) + CA.hat(om * dt) + 0.5 * CA.hat(om * dt) @ CA.hat(om * dt))
        U_, _, Vt_ = np.linalg.svd(R); R = U_ @ Vt_
        log_t.append(t); log_p.append(p.copy()); log_tilt.append(math.degrees(math.acos(min(1.0, R[2, 2]))))
        if k == int(T / dt) - 1:
            e_eta_final = law["e_eta"]
    lt = np.array(log_t); lp = np.array(log_p); tilt = np.array(log_tilt)
    # pre-kick settle (2..6 s) and the kick response (6.5..14 s): peaks of y - mean
    pre = (lt > 3.0) & (lt < 6.0); post = lt > 6.5
    y = lp[post, 1] - lp[post, 1][-200:].mean()
    peaks = []
    for i in range(1, len(y) - 1):
        if y[i] > y[i - 1] and y[i] > y[i + 1] and y[i] > 1e-3:
            peaks.append((lt[post][i], y[i]))
    zeta = float("nan"); f_hz = float("nan")
    if len(peaks) >= 2:
        (t1, a1), (t2, a2) = peaks[0], peaks[1]
        f_hz = 1.0 / (t2 - t1); dlt = math.log(a1 / a2) if a2 > 0 else float("inf")
        zeta = dlt / math.sqrt(4 * math.pi ** 2 + dlt ** 2)
    pre_std = lp[pre, :2].std(0) * 1e3
    return dict(zeta=zeta, f_hz=f_hz, peak_mm=max(abs(y)) * 1e3, pre_std_mm=pre_std, tilt_max=tilt[post].max(), peaks=peaks[:4],
                e_eta_final_deg=np.degrees(e_eta_final), n_leash=n_leash, n_unconv=n_unconv,
                solve_ms=float(np.mean(solve_ms)), p_err_final_mm=float(np.linalg.norm(lp[-1] - lp[0]) * 1e3),
                entry_peak_mm=float(np.max(np.linalg.norm(lp[lt < 3.0] - lp[0], axis=1)) * 1e3))


if __name__ == "__main__":
    print(f"{'kpp':>5s} {'kpd':>5s} {'kpi':>4s} {'keta_p':>7s} {'keta_d':>7s} | hold std xy [mm] | kick peak [mm] f [Hz] zeta  tilt")
    grid = [
        ([2.5, 2.5, 8.0], [2.5, 2.5, 5.0], 1.0, [3.337, 3.337, 1.737], [1.2, 1.2, 0.4105], "cadm_08 as flown (no keta_i)", dict()),
        ([2.5, 2.5, 8.0], [2.5, 2.5, 5.0], 1.0, [3.337, 3.337, 1.737], [1.2, 1.2, 0.4105], "+ keta_i 1/1/0.5", dict(keta_i=(1.0, 1.0, 0.5))),
        ([2.5, 2.5, 8.0], [2.5, 2.5, 5.0], 1.0, [3.337, 3.337, 1.737], [1.2, 1.2, 0.4105], "+ keta_i 2/2/1", dict(keta_i=(2.0, 2.0, 1.0))),
        ([2.5, 2.5, 8.0], [2.5, 2.5, 5.0], 1.0, [3.337, 3.337, 1.737], [1.2, 1.2, 0.4105], "+ keta_i 1/1/0.5, entry surge 0.3", dict(keta_i=(1.0, 1.0, 0.5), v0=(0.3, 0.0, 0.0))),
        ([2.5, 2.5, 8.0], [2.5, 2.5, 5.0], 1.0, [3.337, 3.337, 1.737], [1.2, 1.2, 0.4105], "+ keta_i 1/1/0.5, 40 ms delay", dict(keta_i=(1.0, 1.0, 0.5), delay_s=0.04)),
        ([2.5, 2.5, 8.0], [2.5, 2.5, 5.0], 1.0, [3.337, 3.337, 1.737], [1.2, 1.2, 0.4105], "+ keta_i 2/2/1, 40 ms delay", dict(keta_i=(2.0, 2.0, 1.0), delay_s=0.04)),
    ]
    for i, (kpp, kpd, kpi, kp_, kd_, name, kw) in enumerate(grid):
        r = run(kpp, kpd, kpi, kp_, kd_, **kw)
        print(f"{kpp[0]:5.2f} {kpd[0]:5.2f} {kpi:4.1f} {kp_[0]:7.3f} {kd_[0]:7.3f} | {r['pre_std_mm'][0]:6.1f} {r['pre_std_mm'][1]:6.1f} | {r['peak_mm']:8.1f} {r['f_hz']:5.2f} {r['zeta']:5.2f} {r['tilt_max']:5.1f}"
              f" | e_eta end {np.round(r['e_eta_final_deg'], 2)} deg, entry peak {r['entry_peak_mm']:.0f} mm, leash {r['n_leash']}, unconv {r['n_unconv']}, {r['solve_ms']:.1f} ms  {name}")
