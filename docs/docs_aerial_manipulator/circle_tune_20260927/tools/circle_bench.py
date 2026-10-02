#!/usr/bin/env python3
"""Circle bench for the whole-body 4-D L1 law -- the tuning tool of 2026-09-27.

The exact Python law (wb_entry_sim.Law: controller.py + l1_observer.py, the C++
port's source of truth) flies a recorded/generated planner reference stream
(gen_circle_stream.py) against the MIRROR plant identified from the 0918/0921/
0924 flights, at RTF 1 (the clock hardware has). What is NEW against
sim2real_tuning_20260926/tools/circle_sweep.py, which this file does not modify:

  * FEEDBACK IS WHAT THE LAW GETS ON HARDWARE, not the true state. EKF-fused
    odometry = the state 20 ms ago + band-limited velocity noise (fused hover
    noise 1.2-1.5 cm/s measured) + position noise; attitude 4 ms old + jitter;
    gyro noise; encoder quantisation (4096/rev); the arm velocity observer's
    12 ms lag. Every gain now pays for the noise it amplifies, which a
    perfect-feedback bench never charges -- the reason high gains always looked
    free offline.
  * common random numbers: every candidate sees the SAME noise sequence (seed),
    so differences between candidates are not noise luck.
  * an actuator-effort score (rotor-command and joint-torque tick-to-tick rms),
    the offline stand-in for the motor-wear / ripple cost of a noisy tune.
  * any stream: r = 0.50 m @ 0.13 m/s and r = 0.75 m @ 0.20 m/s (same 24 s lap,
    same arm motion) are both generated from the current planner + yaml.

Scoring is on the TRUE state. Everything here is a screen: fly the result.
"""
import copy
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "docs", "docs_aerial_manipulator", "sim2real_tuning_20260926", "tools"))
sys.path.insert(0, os.path.join(REPO, "application", "robotic_arm", "utils"))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
import circle_sweep as CS                                   # noqa: E402  (constants + helpers only)
import wb_entry_sim as ES                                   # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C       # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

DATA = os.path.join(HERE, "..", "data")
R_MODEL = TP.R_MODEL
N = 4

# ---- the SHIPPED law (hardware 4-D yaml as of 2026-09-27, k_w 0.9) ----------
BASE = dict(k_x=32.0, k_v=20.0, k_R=2.0, k_w=0.9, mrd_s=1.0,
            ky=80.0, dy=24.0, ky_psi=0.3, dy_psi=0.3,
            omega_c_t=2.0, omega_c_r=0.5, omega_c_q=0.5, omega_x=0.25, omega_q=0.2,
            dls=0.3)
MRD0 = np.array([0.130710, 0.135962, 0.134261])

# ---- feedback realism (defaults; 'ideal' switches all of it off) -----------
# vel_noise CALIBRATED (2026-09-27): the shipped law's hover |e_R| is set by the
# fused velocity noise (it feeds f_d's tilt), and 0.020 m/s reproduces the
# flights' hover 0.021 (0.013, the first guess, gave 0.014). gyro_noise only
# moves rotor chatter, 0.03 rad/s.
FB = dict(odo_lag_s=0.020, vel_noise=0.020, vel_bw_hz=5.0, pos_noise=0.001,
          att_lag_s=0.004, att_noise=0.002, gyro_noise=0.03, enc_quant=2 * np.pi / 4096,
          qd_lag_s=0.012, qd_noise=0.01)
FB_IDEAL = dict(odo_lag_s=0.0, vel_noise=0.0, vel_bw_hz=5.0, pos_noise=0.0, att_lag_s=0.0,
                att_noise=0.0, gyro_noise=0.0, enc_quant=0.0, qd_lag_s=0.0, qd_noise=0.0)


def load_stream(path):
    d = np.load(path, allow_pickle=True)
    S = {"t": d["wbref__recv"] - d["wb__recv"][0]}
    for k in ("x_cd", "x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4", "b1_d", "b1_d_dot", "b1_d_ddot",
              "r_ed", "r_ed_dot", "r_ed_ddot", "b1_de", "b1_de_dot", "b1_de_ddot"):
        S[k] = np.column_stack([d[f"wbref__{k}.{a}"] for a in "xyz"])
    S["q_d"] = d["wbref__q_d"]; S["qdot_d"] = d["wbref__qdot_d"]
    sv = [str(x) for x in d["pl_status__data"]]
    ts = d["pl_status__recv"] - d["wb__recv"][0]
    ex = [(ts[i], float(s.split("T=")[1].rstrip("s"))) for i, s in enumerate(sv) if s.startswith("EXECUTING")]
    a, T = max(ex, key=lambda x: x[1])
    S["run"] = (float(a), float(a + T))
    S["name"] = os.path.basename(path)
    return S


STREAMS = {}


def stream(name):
    if name not in STREAMS:
        STREAMS[name] = load_stream(os.path.join(DATA, f"stream_{name}.npz"))
    return STREAMS[name]


def make_cfg(p):
    return CS.hw_gains(k_x=p["k_x"], k_v=p["k_v"], k_R=p["k_R"], k_w=p["k_w"],
                       mrd=tuple(MRD0 * p["mrd_s"]),
                       ky=(p["ky"], p["ky"], p["ky"], p["ky_psi"]), dy=(p["dy"], p["dy"], p["dy"], p["dy_psi"]),
                       dls_lambda=p["dls"])


def make_l1(p):
    return CS.hw_l1(omega_c_t=p["omega_c_t"], omega_c_r=p["omega_c_r"], omega_c_q=p["omega_c_q"],
                    omega_x=p["omega_x"], omega_q=p["omega_q"])


class _AR1:
    """Exact-discretisation band-limited noise, stationary rms sigma at any dt."""
    def __init__(self, rng, sigma, bw_hz, dt, dim):
        self.a = np.exp(-2 * np.pi * bw_hz * dt); self.s = sigma; self.rng = rng
        self.x = rng.standard_normal(dim) * sigma

    def __call__(self):
        self.x = self.a * self.x + np.sqrt(1 - self.a ** 2) * self.s * self.rng.standard_normal(self.x.shape)
        return self.x


def simulate(p, S, profile="mirror", delay_ms=16.0, fb=None, seed=0, t_settle=8.0, gust=None,
             inertia=1.0, kf=1.0, keep=False, rtf=1.0, plant_armature=None, ff_scale=None,
             ff_vel="observer", pv_lag_s=0.048, pv_quant=0.024, arm_delay_ms=0.0, full=False):
    """One run of the law on stream S. p = gain dict (BASE keys). Returns metrics.

    rtf < 1 = ISAAC-CLOCK EMULATION (2026-09-27): the law, its L1 observer, the
    arm velocity observer and the planner run on the WALL clock (tick dt) while
    the plant advances rtf*dt per tick, exactly as Isaac at RTF ~0.48 under
    replay.sh (rotor lag and transport delay are already expressed in wall ticks
    there, so they stay per-tick here). Consequences reproduced: reference
    derivatives arrive scaled by rtf^k; the arm observer's q_dot is rtf x the
    plant rate; and the L1 predictor integrates plant-time momentum changes with
    the wall step, i.e. d_hat = d - (1 - rtf) p_dot -- a positive acceleration
    feedback through C(s) that hardware does not have."""
    fb = dict(FB if fb is None else fb)
    rng = np.random.default_rng(seed)
    model = TP.make_params_t650(armature_diag=CS.J_ARM)
    case = ES.Case(posture=0.0, ee_ref="relative", int_ff=True, int_ff_source="four_d",
                   mass=1.0, inertia=1.0, com=(-0.017854, 0.0, 0.0), kf=1.0, kf_alloc=1.0,
                   observer="l1", delay_ms=delay_ms, gains={}, l1=make_l1(p))
    if profile == "robustness":
        case.mass, case.inertia, case.arm_mass, case.com = CS.ROB_MASS, CS.ROB_INERTIA * inertia, CS.ROB_ARM_MASS, CS.ROB_COM
        plant = ES.plant_params(model, case)
        model["base_com"] = np.zeros(3)
        S = CS.shift_stream_base_com(S, R_MODEL.T @ np.array([-0.017854, 0.0, 0.0]), np.zeros(3),
                                     model["m_i"][0], sum(model["m_i"]))
        kf_alloc, km_ratio, fric_plant = CS.ROB_KF_ALLOC, CS.ROB_KM_ALLOC_RATIO, CS.ROB_FRIC_SCALE
        fb_body, tb_body = np.zeros(3), np.zeros(3)
        kf_plant, km_plant = ES.KF_TRUE * kf, ES.KM_TRUE
        t_settle = max(t_settle, 20.0)
    else:
        case.inertia = inertia
        plant = ES.plant_params(model, case)
        model["base_com"] = R_MODEL.T @ np.array([-0.017854, 0.0, 0.0])
        kf_alloc, km_ratio, fric_plant = CS.KF_ALLOC, CS.KM_ALLOC_RATIO, CS.FRIC_SCALE
        fb_body, tb_body = CS.FORCE_BIAS_BODY, CS.TORQUE_BIAS_BODY
        kf_plant, km_plant = CS.KF_PLANT * kf, CS.KM_PLANT
    if plant_armature is not None:       # the PLANT's servo armature (Isaac 06 default: 0.020 each)
        plant["armature_diag"] = np.asarray(plant_armature, float)
    ffs = CS.FF_SCALE if ff_scale is None else np.asarray(ff_scale, float)
    cfg = make_cfg(p)
    law = ES.Law(model, cfg, case)
    dt = case.dt
    dtp = rtf * dt                       # plant seconds per law tick
    B = np.zeros((4, 4))
    for i, r in enumerate(ES.ROTOR_POS):
        B[0, i] = 1.0; B[1, i] = r[1]; B[2, i] = -r[0]; B[3, i] = ES.ROT_DIR[i] * km_ratio
    Bp = np.linalg.pinv(B)
    rp = ES.ROTOR_POS; rd = ES.ROT_DIR

    def allocate(thrust, tau):
        f = np.maximum(Bp @ np.array([thrust, *tau]), 0.0)
        return np.clip(np.sqrt(f / kf_alloc), 0.0, ES.OMEGA_MAX)

    fbias_m = R_MODEL.T @ fb_body; tbias_m = R_MODEL.T @ tb_body
    t_run0, t_run1 = S["run"]
    i_hold = max(np.searchsorted(S["t"], t_run0) - 1, 0)
    r0 = CS.ref_at(S, 0, i_hold)
    X = np.zeros(12 + 3 * N + 6)
    X[12:12 + N] = r0["q_d"]
    psi = np.arctan2(r0["b1_d"][1], r0["b1_d"][0])
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1.0]])
    X[3:12] = Rz.flatten(order="F")
    X[0:3] = r0["x_cd"] - Rz @ C.dynamics(X, model)["r_0c_0"]
    gp = C.dynamics(X, plant)["g"]
    omega_rot = allocate(gp[2], gp[3:6])
    n_del = int(round(delay_ms * 1e-3 / dt))
    fifo = [omega_rot.copy() for _ in range(n_del)]
    # feedback state
    n_odo = int(round(fb["odo_lag_s"] / dt)); n_att = int(round(fb["att_lag_s"] / dt))
    hist_X = [X.copy() for _ in range(max(n_odo, n_att) + 1)]
    vnoise = _AR1(rng, fb["vel_noise"], fb["vel_bw_hz"], dt, 3)
    a_qd = np.exp(-dt / fb["qd_lag_s"]) if fb["qd_lag_s"] > 0 else 0.0
    qd_obs = X[18 + N:18 + 2 * N].copy()
    # the servo's Present Velocity (joint_states.velocity): first-order lag + quantum, in plant units
    a_pv = np.exp(-dt / pv_lag_s) if pv_lag_s > 0 else 0.0
    qd_pv = X[18 + N:18 + 2 * N].copy()
    n_arm = int(round(arm_delay_ms * 1e-3 / dt))       # arm torque transport (controller -> topic -> physics ZOH)
    tj_fifo = [np.zeros(N) for _ in range(n_arm)]
    t_start = t_run0 - t_settle
    steps = int((t_run1 + 2.0 - t_start) / dtp)
    H = {k: np.zeros(steps) for k in ("t", "ee", "ex", "hd", "eR", "tilt", "tau", "nsat", "dw", "dtau")}
    Hq = np.zeros((steps, N)); Hqd = np.zeros((steps, N))
    F = ({k: np.zeros((steps, 3)) for k in ("xc", "xcd", "eR", "ew", "ee")}
         | {k: np.zeros((steps, N)) for k in ("qe", "qde")}) if full else None
    w_prev = omega_rot.copy(); tj_prev = np.zeros(N)
    verdict = "completed"; k_end = steps
    SC = {"x_cd_dot": 1, "x_cd_ddot": 2, "x_cd_d3": 3, "x_cd_d4": 4, "b1_d_dot": 1, "b1_d_ddot": 2,
          "r_ed_dot": 1, "r_ed_ddot": 2, "b1_de_dot": 1, "b1_de_ddot": 2, "qdot_d": 1}
    for k in range(steps):
        t = t_start + k * dtp            # PLANT time (= the plan's nominal time)
        if t < t_run0:
            ref = dict(CS.ref_at(S, t, i_hold))
            for kk in ("x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4", "b1_d_dot", "b1_d_ddot",
                       "r_ed_dot", "r_ed_ddot", "b1_de_dot", "b1_de_ddot"):
                ref[kk] = np.zeros(3)
            ref["qdot_d"] = np.zeros(N)
        else:
            ref = CS.ref_at(S, t, np.searchsorted(S["t"], t))
            if rtf != 1.0:               # derivatives per WALL second
                ref = dict(ref)
                for kk, pw in SC.items():
                    ref[kk] = ref[kk] * rtf ** pw
        # ---------------- what the law MEASURES ----------------
        Xo = hist_X[-1 - n_odo]; Xa = hist_X[-1 - n_att]
        Ra_true = Xa[3:12].reshape(3, 3, order="F")
        if fb["att_noise"] > 0:
            dv = rng.standard_normal(3) * fb["att_noise"]; nv = np.linalg.norm(dv)
            Ra = Ra_true @ C.joint_rotation(dv / (nv + 1e-12), nv)
        else:
            Ra = Ra_true
        Ro = Xo[3:12].reshape(3, 3, order="F")
        v_world = Ro @ Xo[12 + N:15 + N] + (vnoise() if fb["vel_noise"] > 0 else 0.0)
        Xm = X.copy()
        Xm[0:3] = Xo[0:3] + (rng.standard_normal(3) * fb["pos_noise"] if fb["pos_noise"] > 0 else 0.0)
        Xm[3:12] = Ra.flatten(order="F")
        Xm[12 + N:15 + N] = Ra.T @ v_world
        Xm[15 + N:18 + N] = X[15 + N:18 + N] + (rng.standard_normal(3) * fb["gyro_noise"] if fb["gyro_noise"] > 0 else 0.0)
        qt = X[12:12 + N]
        Xm[12:12 + N] = np.round(qt / fb["enc_quant"]) * fb["enc_quant"] if fb["enc_quant"] > 0 else qt
        qd_obs = a_qd * qd_obs + (1 - a_qd) * rtf * X[18 + N:18 + 2 * N]
        Xm[18 + N:18 + 2 * N] = qd_obs + (rng.standard_normal(N) * fb["qd_noise"] if fb["qd_noise"] > 0 else 0.0)
        dyn = C.dynamics(Xm, model)
        out = law(Xm, dyn, ref, dt)
        u1 = float(out["u1"]); tau_body = np.asarray(out["tau_body"], float)
        tau_joint = np.clip(np.asarray(out["tau_joint"], float), -cfg.tau_max, cfg.tau_max)
        q = X[12:12 + N]; qd = X[18 + N:18 + 2 * N]
        load = np.abs(tau_joint)
        # arm-side friction FF (measured-velocity relay: the observer's q_dot)
        qd_pv = a_pv * qd_pv + (1 - a_pv) * X[18 + N:18 + 2 * N]
        if ff_vel == "present":        # the relay on Present Velocity (what the arm controller reads)
            v_ff = np.round(qd_pv / pv_quant) * pv_quant if pv_quant > 0 else qd_pv
            tau_joint = tau_joint + ffs * (CS.FRIC_FC + CS.FRIC_MU * load) * np.tanh(v_ff / CS.FF_W_MEAS)
        elif ff_vel == "reference":    # the pre-2026-09-26 relay on the reference velocity (width 0.015)
            tau_joint = tau_joint + ffs * (CS.FRIC_FC + CS.FRIC_MU * load) * np.tanh(ref["qdot_d"] / 0.015)
        else:                          # the 12 ms encoder observer
            tau_joint = tau_joint + ffs * (CS.FRIC_FC + CS.FRIC_MU * load) * np.tanh(Xm[18 + N:18 + 2 * N] / CS.FF_W_MEAS)
        w_cmd = allocate(u1, tau_body)
        H["dw"][k] = np.sqrt(np.mean((w_cmd - w_prev) ** 2)); w_prev = w_cmd
        H["dtau"][k] = np.sqrt(np.mean((tau_joint - tj_prev) ** 2)); tj_prev = tau_joint
        if n_del > 0:
            fifo.append(w_cmd); w_cmd = fifo.pop(0)
        if n_arm > 0:
            tj_fifo.append(tau_joint.copy()); tau_joint = tj_fifo.pop(0)
        omega_rot = w_cmd + (omega_rot - w_cmd) * np.exp(-ES.LAMBDA_ROTOR * dt)
        fr_ = kf_plant * omega_rot ** 2
        thrust_a = fr_.sum()
        tau_a = np.array([np.dot(rp[:, 1], fr_), -np.dot(rp[:, 0], fr_), np.dot(rd, km_plant * omega_rot ** 2)])
        fr = fric_plant * (CS.FRIC_FC + CS.FRIC_MU * load) * np.tanh(qd / CS.FRIC_W)
        fr = np.clip(fr, -0.02 * np.abs(qd) / dtp, 0.02 * np.abs(qd) / dtp)
        over = q - ES.Q_MAX; under = ES.Q_MIN - q
        tau_stop = (-np.where(over > 0, ES.K_STOP * over + ES.C_STOP * np.maximum(qd, 0), 0.0)
                    + np.where(under > 0, ES.K_STOP * under + ES.C_STOP * np.maximum(-qd, 0), 0.0))
        dynp = C.dynamics(X, plant)
        R = X[3:12].reshape(3, 3, order="F")
        fb_now = fbias_m
        if gust is not None and t_run0 + gust[0] <= t < t_run0 + gust[0] + gust[1]:
            fb_now = fb_now + R.T @ np.array([gust[2], gust[3], 0.0])
        Q = np.concatenate([fb_now + [0.0, 0.0, thrust_a], tau_a + tbias_m, tau_joint - fr + tau_stop])
        xi = np.concatenate([X[12 + N:15 + N], X[15 + N:18 + N], X[18 + N:18 + 2 * N]])
        xi = xi + np.linalg.solve(dynp["M"], Q - dynp["C"] @ xi - dynp["g"]) * dtp
        v0, w0, qdn = xi[0:3], xi[3:6], xi[6:6 + N]
        # scoring on the TRUE state (plant kinematics = model kinematics)
        r_e = X[0:3] + R @ dynp["r_0e_0"]
        x_c = X[0:3] + R @ dyn["r_0c_0"]
        H["t"][k] = t
        H["ee"][k] = np.linalg.norm(r_e - ref["r_ed"])
        H["ex"][k] = np.linalg.norm(x_c - ref["x_cd"])
        b1e = (R @ dynp["R_e_0"])[:, 0]
        H["hd"][k] = np.degrees(np.arcsin(np.clip(np.cross(b1e, ref["b1_de"])[2], -1, 1)))
        H["eR"][k] = np.linalg.norm(out["e_R"])
        H["tilt"][k] = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        H["tau"][k] = np.abs(tau_joint).max(); H["nsat"][k] = out["n_sat"]
        Hq[k] = q; Hqd[k] = ref["q_d"]
        if full:
            F["xc"][k] = x_c; F["xcd"][k] = ref["x_cd"]; F["eR"][k] = out["e_R"]
            F["ew"][k] = X[15 + N:18 + N] - R.T @ out["R0c"] @ out["omega_0c"]
            F["ee"][k] = r_e - ref["r_ed"]; F["qe"][k] = q - ref["q_d"]; F["qde"][k] = qd - ref["qdot_d"]
        X[0:3] = X[0:3] + R @ v0 * dtp
        nw = np.linalg.norm(w0)
        R = R @ C.joint_rotation(w0 / (nw + 1e-12), nw * dtp)
        uu, _, vt = np.linalg.svd(R); R = uu @ vt
        X[3:12] = R.flatten(order="F")
        X[12:12 + N] = X[12:12 + N] + qdn * dtp
        X[12 + N:18 + N] = np.concatenate([v0, w0]); X[18 + N:18 + 2 * N] = qdn
        hist_X.append(X.copy()); hist_X.pop(0)
        if not np.isfinite(H["ex"][k]) or H["tilt"][k] > 35.0 or H["ex"][k] > 1.5:
            verdict = "ABORT"; k_end = k + 1; break
    H = {kk: v[:k_end] for kk, v in H.items()}; Hq = Hq[:k_end]; Hqd = Hqd[:k_end]
    rms = lambda v: float(np.sqrt(np.mean(v ** 2))) if len(v) else float("nan")  # noqa: E731
    m = (H["t"] >= t_run0) & (H["t"] <= t_run1)
    we = (H["t"] < t_run0) & (H["t"] > t_run0 - 3.0)       # the last 3 s of the pre-run hold
    res = dict(verdict=verdict, t_end=float(H["t"][-1] - t_start) if k_end else 0.0)
    if verdict == "completed":
        res.update(ee_rms=rms(H["ee"][m]), ee_pk=float(H["ee"][m].max()), com_rms=rms(H["ex"][m]),
                   com_pk=float(H["ex"][m].max()), head_rms=rms(H["hd"][m]), eR_mean=float(H["eR"][m].mean()),
                   tilt_pk=float(H["tilt"][m].max()), tau_pk=float(H["tau"][m].max()),
                   sat_pct=float(100 * np.mean(H["nsat"][m] > 0)),
                   q_err_rms=[rms(np.degrees(Hq[m, j] - Hqd[m, j])) for j in range(N)],
                   dw_rms=rms(H["dw"][m]), dtau_rms=rms(H["dtau"][m]),
                   hold_com_rms=rms(H["ex"][we]), hold_eR=float(H["eR"][we].mean()),
                   entry_pk=float(H["ex"][H["t"] < t_run0].max()))
        big = np.where((H["t"] < t_run0) & (H["ex"] > 0.030))[0]
        res["entry_t30"] = float(H["t"][big[-1]] - t_start) if len(big) else 0.0
        if gust is not None:
            ta = t_run0 + gust[0]
            pre = (H["t"] >= ta - 3.0) & (H["t"] < ta); base = rms(H["ex"][pre])
            wg = (H["t"] >= ta) & (H["t"] < ta + gust[1] + 6.0)
            res["gust_rise"] = float(H["ex"][wg].max() - base)
            idx = np.where(wg & (H["ex"] > base + 0.010))[0]
            res["gust_rec"] = float(H["t"][idx[-1]] - (ta + gust[1])) if len(idx) else 0.0
    if full:
        res["F"] = {kk: v[:k_end] for kk, v in F.items()} | {"t": H["t"], "hd": H["hd"], "run": (t_run0, t_run1)}
    if keep:
        res["H"] = H; res["Hq"] = Hq; res["Hqd"] = Hqd
    return res


def fmt(name, r):
    if r["verdict"] != "completed":
        return f"{name:40} ABORT at {r['t_end']:.1f} s"
    q = r["q_err_rms"]
    return (f"{name:40} EEabs {r['ee_rms']*1e3:6.1f} (pk {r['ee_pk']*1e3:4.0f}) | CoM {r['com_rms']*1e3:6.1f} "
            f"| head {r['head_rms']:4.2f}° | |eR| {r['eR_mean']:.4f} | tilt {r['tilt_pk']:4.1f} | q2/q3 {q[1]:.2f}/{q[2]:.2f}° "
            f"| sat {r['sat_pct']:.1f}% | dω {r['dw_rms']:.2f} dτ {r['dtau_rms']*1e3:.1f}m | hold CoM {r['hold_com_rms']*1e3:.1f} |eR| {r['hold_eR']:.4f}")


if __name__ == "__main__":
    import multiprocessing as mp
    import time
    jobs = []
    for nm in ("r050", "r075"):
        jobs += [(f"shipped k_w 0.9 | {nm}", dict(BASE), nm, None),
                 (f"as flown k_w 1.5 | {nm}", dict(BASE, k_w=1.5), nm, None),
                 (f"shipped, ideal feedback | {nm}", dict(BASE), nm, FB_IDEAL)]

    def job(j):
        t0 = time.time(); nm, p, s, fb = j
        r = simulate(p, stream(s), fb=fb)
        return nm, r, time.time() - t0
    with mp.get_context("fork").Pool(len(jobs)) as pool:
        for nm, r, dtw in pool.imap(job, jobs):
            print(fmt(nm, r) + f"  [{dtw:.0f} s]", flush=True)
