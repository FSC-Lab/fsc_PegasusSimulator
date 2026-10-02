#!/usr/bin/env python3
"""Offline gain sweep of the whole-body L1-4D law on the RECORDED 0924 circle.

The reference stream the hardware law consumed on 0924 F2 (WholeBodyReference,
100 Hz: CoM chain through snap, platform heading chain, EE position and
heading chains, q_d/qdot_d) is replayed through the EXACT Python law
(controller.py + l1_observer.py, the C++ port's source of truth, via
wb_entry_sim.Law) against the MIRROR plant identified from the flights, at
one consistent clock (RTF 1 -- what the hardware had). Every case is scored
on the run segment: EE ABSOLUTE error (world r_e vs the streamed r_ed), CoM
error, heading, attitude, tilt, torques, saturation.

It is a screening tool (the plant is the model + the mirror terms, not PhysX);
trust the ORDERING, fly the winners in Isaac, then on hardware.

    /usr/bin/python3 circle_sweep.py --baseline
    /usr/bin/python3 circle_sweep.py --sweep            # one group at a time
    /usr/bin/python3 circle_sweep.py --case k_x=48,k_v=25,omega_c_t=4
"""
import argparse
import copy
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "application", "robotic_arm", "utils"))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
import wb_entry_sim as ES                                            # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

DATA = os.path.join(HERE, "..", "data")
# ---- the mirror plant (params_..._l1_4d_..._sim.yaml section 1, F2 lift-off kf)
KF_BENCH = 4.540431e-05
KF_PLANT = ES.KF_TRUE * 1.030            # F2 lift-off
KF_ALLOC = 4.260431e-05                  # the flown allocator belief
KM_PLANT = 8.247173e-07 * 0.70           # 0.70 x bench
KM_ALLOC_RATIO = 0.018164                # alloc_rotor*_km (bench c/kf)
FORCE_BIAS_BODY = np.array([0.55, -0.50, 0.0])   # actual FLU
TORQUE_BIAS_BODY = np.array([0.0, 0.0, -0.095])
J_ARM = np.array([0.010, 0.0194, 0.0097, 0.0097])
FRIC_FC = np.array([0.01711, 0.03143, 0.05751, 0.05237])
FRIC_MU = np.array([0.0, 0.246, 0.161, 0.0])
FRIC_SCALE = np.array([1.0, 0.70, 0.65, 1.5])   # the mirror PLANT's friction
FF_SCALE = np.array([1.0, 0.70, 0.65, 1.0])     # the arm yaml's corrected feed-forward
FRIC_W = 0.015
FF_W_MEAS = 0.03
DELAY_MS = 16.0
# ---- the ROBUSTNESS plant (params_..._l1_4d_..._sim_robustness.yaml section 1 +
# its allocator stress), used as the pass/fail gate for any tuned candidate
ROB_KF_ALLOC = 4.7544506e-05             # +17.6 % over the plant's 4.041283e-05
ROB_KM_ALLOC_RATIO = 0.0612219           # = plant c/kf (matched)
ROB_MASS, ROB_INERTIA, ROB_ARM_MASS = 1.10, 1.10, 1.05
ROB_COM = (0.010, 0.010, 0.005)          # actual frame
ROB_FRIC_SCALE = np.array([1.05, 1.05, 1.05, 1.05])
R_MODEL = TP.R_MODEL                     # actual = R_MODEL @ model


def hw_gains(**over):
    """The hardware yaml as of 2026-09-26 (K_y 80/24, rescaled M_r_d)."""
    v = dict(k_x=32.0, k_v=20.0, k_R=2.0, k_w=1.5,
             mrd=(0.130710, 0.135962, 0.134261),
             ky=(80.0, 80.0, 80.0, 0.3), dy=(24.0, 24.0, 24.0, 0.3),
             my=(1.0, 1.0, 1.0, 0.05), ko=(0.5, 0.1, 0.1),
             use_gmo=True, dls_lambda=0.3, tau_max=3.0)
    v.update(over)
    return ES.make_gains(**v)


def hw_l1(**over):
    v = dict(a_t=2.0, a_r=2.0, a_q=2.0, omega_c_t=2.0, omega_c_r=0.5, omega_c_q=0.5,
             omega_i=2.0, omega_x=0.25, adapt_period_s=0.0, decompose=True,
             lc_var_f=100.0, lc_var_m=0.25, lc_var_q=0.0025,
             max_force_n=20.0, max_torque_nm=2.0, max_joint_nm=1.5,
             max_wrench_force_n=15.0, max_wrench_torque_nm=3.0,
             four_d=True, omega_q=0.2, contact=False, collision_threshold_n=0.0,
             omega_x_t=0.0, omega_x_r=0.0, omega_x_q=0.0)
    v.update(over)
    return v


def load_stream(nm="a2"):
    d = np.load(os.path.join(DATA, f"{nm}.npz"), allow_pickle=True)
    t0 = d["wb__recv"][0]
    t = d["wbref__recv"] - t0
    V = lambda k: np.column_stack([d[f"wbref__{k}.{a}"] for a in "xyz"])  # noqa: E731
    S = {"t": t}
    for k in ("x_cd", "x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4", "b1_d", "b1_d_dot", "b1_d_ddot",
              "r_ed", "r_ed_dot", "r_ed_ddot", "b1_de", "b1_de_dot", "b1_de_ddot"):
        S[k] = V(k)
    S["q_d"] = d["wbref__q_d"]
    S["qdot_d"] = d["wbref__qdot_d"]
    # the run segment: planner EXECUTING with the longest T
    sv = [str(x) for x in d["pl_status__data"]]
    ts = d["pl_status__recv"] - t0
    ex = [(ts[i], float(s.split("T=")[1].rstrip("s"))) for i, s in enumerate(sv) if s.startswith("EXECUTING")]
    # one entry per leg (the status republishes while executing)
    legs = []
    for t_, T_ in ex:
        if not legs or t_ > legs[-1][0] + legs[-1][1] + 0.5:
            legs.append((t_, T_))
    if nm == "f18":
        # the whole 0918 mission: EE out, home, +0.61 m base step, EE out, home
        S["run"] = (legs[0][0], float(t[-1]) - 0.5)
        S["legs"] = [(a_, a_ + T_) for a_, T_ in legs]
    else:
        a, T = max(legs, key=lambda x: x[1])
        S["run"] = (a, a + T)
    S["hold0"] = (t[0], ex[0][0] if ex else a)
    return S


def shift_stream_base_com(S, b_old_model, b_new_model, m0, m):
    """The planner's x_cd for a model whose base CoM is b_new instead of b_old:
    x_cd += (m0/m) Rz(psi) (b_new - b_old), psi = heading of b1_d (model frame);
    first two derivatives analytic, d3/d4 neglected (mm/s^3 scale)."""
    S2 = dict(S)
    db = (m0 / m) * (np.asarray(b_new_model, float) - np.asarray(b_old_model, float))
    b1, b1d, b1dd = S["b1_d"], S["b1_d_dot"], S["b1_d_ddot"]
    psi = np.arctan2(b1[:, 1], b1[:, 0])
    n2 = b1[:, 0] ** 2 + b1[:, 1] ** 2
    pd = (b1[:, 0] * b1d[:, 1] - b1[:, 1] * b1d[:, 0]) / n2
    pdd = (b1[:, 0] * b1dd[:, 1] - b1[:, 1] * b1dd[:, 0]) / n2
    Sk = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    D, D1, D2 = np.zeros_like(b1), np.zeros_like(b1), np.zeros_like(b1)
    for i, ps in enumerate(psi):
        Rz = np.array([[np.cos(ps), -np.sin(ps), 0], [np.sin(ps), np.cos(ps), 0], [0, 0, 1.0]])
        D[i] = Rz @ db
        D1[i] = pd[i] * Rz @ Sk @ db
        D2[i] = pdd[i] * Rz @ Sk @ db + pd[i] ** 2 * Rz @ Sk @ Sk @ db
    S2["x_cd"] = S["x_cd"] + D
    S2["x_cd_dot"] = S["x_cd_dot"] + D1
    S2["x_cd_ddot"] = S["x_cd_ddot"] + D2
    return S2


def ref_at(S, tq, idx):
    """Nearest-sample reference (the stream is 100 Hz, the law 250 Hz: the C++
    node latches the last sample too)."""
    i = min(max(idx, 0), len(S["t"]) - 1)
    r = {k: S[k][i] for k in ("x_cd", "x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4", "b1_d", "b1_d_dot", "b1_d_ddot",
                              "r_ed", "r_ed_dot", "r_ed_ddot", "b1_de", "b1_de_dot", "b1_de_ddot", "q_d", "qdot_d")}
    return r


def simulate(gains_over, l1_over, S, t_settle=6.0, verbose=False, plant_over=None, delay_ms=DELAY_MS,
             profile="mirror"):
    n = 4
    model = TP.make_params_t650(armature_diag=J_ARM)
    case = ES.Case(dhat_rate_ff=bool((plant_over or {}).get("dhat_rate_ff", False)), posture=0.0, ee_ref="relative", int_ff=True, int_ff_source="four_d",
                   mass=1.0, inertia=1.0, com=(-0.017854, 0.0, 0.0), kf=1.0, kf_alloc=1.0,
                   observer="l1", delay_ms=delay_ms, gains={}, l1=hw_l1(**l1_over))
    po = plant_over or {}
    rob = profile == "robustness"
    if rob:
        case.mass, case.inertia, case.arm_mass, case.com = ROB_MASS, ROB_INERTIA * po.get("inertia", 1.0), ROB_ARM_MASS, ROB_COM
        plant = ES.plant_params(model, case)
        model["base_com"] = np.zeros(3)             # the robustness yaml's wb_base_com
        S = shift_stream_base_com(S, R_MODEL.T @ np.array([-0.017854, 0.0, 0.0]), np.zeros(3),
                                  model["m_i"][0], sum(model["m_i"]))
        kf_alloc, km_ratio = ROB_KF_ALLOC, ROB_KM_ALLOC_RATIO
        fric_plant = ROB_FRIC_SCALE
        fb_body, tb_body = np.zeros(3), np.zeros(3)
        kf_plant_nom, km_plant_nom = ES.KF_TRUE, ES.KM_TRUE
        t_settle = max(t_settle, 20.0)              # a 17.6 % handover needs longer to settle
    else:
        case.inertia = po.get("inertia", 1.0)
        plant = ES.plant_params(model, case)
        # the law's own model carries the measured base CoM too (wb_base_com)
        model["base_com"] = R_MODEL.T @ np.array([-0.017854, 0.0, 0.0])
        kf_alloc, km_ratio = KF_ALLOC, KM_ALLOC_RATIO
        fric_plant = FRIC_SCALE
        fb_body = np.zeros(3) if (po.get("no_fbias") or po.get("fbias_world")) else FORCE_BIAS_BODY
        tb_body = np.zeros(3) if po.get("no_tbias") else TORQUE_BIAS_BODY
        kf_plant_nom, km_plant_nom = KF_PLANT, KM_PLANT
        if po.get("km_match"):
            km_plant_nom = KF_PLANT * KM_ALLOC_RATIO
        if po.get("no_fric"):
            fric_plant = np.zeros(4)
    cfg = hw_gains(**gains_over)
    law = ES.Law(model, cfg, case)
    dt = case.dt
    kf_plant = kf_plant_nom * po.get("kf", 1.0)
    km_plant = km_plant_nom * po.get("km", 1.0)
    # allocator with the hardware's believed km ratio
    B = np.zeros((4, 4))
    for i, r in enumerate(ES.ROTOR_POS):
        B[0, i] = 1.0; B[1, i] = r[1]; B[2, i] = -r[0]; B[3, i] = ES.ROT_DIR[i] * km_ratio
    Bp = np.linalg.pinv(B)

    def allocate(thrust, tau):
        f = np.maximum(Bp @ np.array([thrust, *tau]), 0.0)
        return np.clip(np.sqrt(f / kf_alloc), 0.0, ES.OMEGA_MAX)

    def wrench(om):
        f = kf_plant * om ** 2
        tau = np.zeros(3)
        for i, r in enumerate(ES.ROTOR_POS):
            tau[0] += r[1] * f[i]; tau[1] += -r[0] * f[i]; tau[2] += ES.ROT_DIR[i] * km_plant * om[i] ** 2
        return f.sum(), tau

    fbias_m = R_MODEL.T @ fb_body
    tbias_m = R_MODEL.T @ tb_body
    # initial state: hover at the stream's first sample, arm at q_d(0)
    t_run0, t_run1 = S["run"]
    i_hold = max(np.searchsorted(S["t"], t_run0) - 1, 0)   # the sample the run starts from
    r0 = ref_at(S, 0, i_hold)
    X = np.zeros(12 + 3 * n + 6)
    X[3:12] = np.eye(3).flatten(order="F")
    X[12:12 + n] = r0["q_d"]
    dyn0 = C.dynamics(X, model)
    X[0:3] = r0["x_cd"] - dyn0["r_0c_0"]
    # heading: rotate so body x aligns with b1_d(0)
    psi = np.arctan2(r0["b1_d"][1], r0["b1_d"][0])
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1.0]])
    X[3:12] = Rz.flatten(order="F")
    dyn0 = C.dynamics(X, model)
    X[0:3] = r0["x_cd"] - Rz @ dyn0["r_0c_0"]
    dynp = C.dynamics(X, plant)
    gp = dynp["g"]
    omega_rot = allocate(gp[2], gp[3:6])
    n_del = int(round(delay_ms * 1e-3 / dt))
    fifo = [omega_rot.copy() for _ in range(n_del)]
    t_start = t_run0 - t_settle
    steps = int((t_run1 + 2.0 - t_start) / dt)
    hist = {k: [] for k in ("t", "ee_abs", "ex", "hd", "eR", "tilt", "tau", "nsat", "q", "qd",
                            "exv", "eRv", "dth", "drh", "R", "xc", "dtrue", "dhat", "b3c", "b3", "w0c", "w0")}
    verdict = "completed"
    for k in range(steps):
        t = t_start + k * dt
        if t < t_run0:
            ref = ref_at(S, t, i_hold)         # hold at the run's start sample
            ref = {kk: (v.copy() if hasattr(v, "copy") else v) for kk, v in ref.items()}
            for kk in ("x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4", "b1_d_dot", "b1_d_ddot", "r_ed_dot", "r_ed_ddot", "b1_de_dot", "b1_de_ddot"):
                ref[kk] = np.zeros(3)
            ref["qdot_d"] = np.zeros(n)
        else:
            ref = ref_at(S, t, np.searchsorted(S["t"], t))
        dyn = C.dynamics(X, model)
        out = law(X, dyn, ref, dt)
        u1 = float(out["u1"]); tau_body = np.asarray(out["tau_body"], float)
        tau_joint = np.clip(np.asarray(out["tau_joint"], float), -cfg.tau_max, cfg.tau_max)
        q = X[12:12 + n]; qd = X[18 + n:18 + 2 * n]
        # arm-side friction feed-forward (measured-velocity relay, corrected coefficients)
        load = np.abs(tau_joint)
        tau_joint = tau_joint + FF_SCALE * (FRIC_FC + FRIC_MU * load) * np.tanh(qd / FF_W_MEAS)
        w_cmd = allocate(u1, tau_body)
        if n_del > 0:
            fifo.append(w_cmd); w_cmd = fifo.pop(0)
        omega_rot = w_cmd + (omega_rot - w_cmd) * np.exp(-ES.LAMBDA_ROTOR * dt)
        thrust_a, tau_a = wrench(omega_rot)
        # plant gearbox friction on the joint's own motion (momentum-clamped)
        fr = fric_plant * (FRIC_FC + FRIC_MU * load) * np.tanh(qd / FRIC_W)
        fr = np.clip(fr, -0.02 * np.abs(qd) / dt, 0.02 * np.abs(qd) / dt)
        tau_stop = np.zeros(n)
        over = q - ES.Q_MAX; under = ES.Q_MIN - q
        tau_stop -= np.where(over > 0, ES.K_STOP * over + ES.C_STOP * np.maximum(qd, 0), 0.0)
        tau_stop += np.where(under > 0, ES.K_STOP * under + ES.C_STOP * np.maximum(-qd, 0), 0.0)
        dynp = C.dynamics(X, plant)
        R = X[3:12].reshape(3, 3, order="F")
        fb_now = (R.T @ (R_MODEL.T @ FORCE_BIAS_BODY)) if po.get("fbias_world") else fbias_m
        g_ = po.get("gust")
        if g_ is not None and t_run0 + g_[0] <= t < t_run0 + g_[0] + g_[1]:
            fb_now = fb_now + R.T @ np.array([g_[2], g_[3], 0.0])      # world-frame step force
        Q = np.concatenate([fb_now + [0.0, 0.0, thrust_a], tau_a + tbias_m, tau_joint - fr + tau_stop])
        xi = np.concatenate([X[12 + n:15 + n], X[15 + n:18 + n], X[18 + n:18 + 2 * n]])
        xi_dot = np.linalg.solve(dynp["M"], Q - dynp["C"] @ xi - dynp["g"])
        xi = xi + xi_dot * dt
        v0, w0, qdn = xi[0:3], xi[3:6], xi[6:6 + n]
        X[0:3] = X[0:3] + R @ v0 * dt
        nw = np.linalg.norm(w0)
        R = R @ C.joint_rotation(w0 / (nw + 1e-12), nw * dt)
        uu, _, vt = np.linalg.svd(R); R = uu @ vt
        X[3:12] = R.flatten(order="F")
        X[12:12 + n] = X[12:12 + n] + qdn * dt
        X[12 + n:18 + n] = np.concatenate([v0, w0]); X[18 + n:18 + 2 * n] = qdn
        # scoring: EE absolute error in the world against the streamed r_ed
        r_e = X[0:3] + R @ dyn["r_0e_0"]
        hist["t"].append(t)
        hist["ee_abs"].append(np.linalg.norm(r_e - ref["r_ed"]))
        hist["ex"].append(np.linalg.norm(out["e_x"]))
        b1e = (R @ dyn["R_e_0"])[:, 0]
        hist["hd"].append(np.degrees(np.arcsin(np.clip(np.cross(b1e, ref["b1_de"])[2], -1, 1))))
        hist["eR"].append(np.linalg.norm(out["e_R"]))
        hist["tilt"].append(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
        hist["tau"].append(np.abs(tau_joint).max()); hist["nsat"].append(out["n_sat"])
        hist["q"].append(np.degrees(q).copy()); hist["qd"].append(np.degrees(ref["q_d"]).copy())
        hist["exv"].append(np.asarray(out["e_x"], float).copy()); hist["eRv"].append(np.asarray(out["e_R"], float).copy())
        hist["dth"].append(np.asarray(out["d_t_hat"], float).copy()); hist["drh"].append(np.asarray(out["d_r_hat"], float).copy())
        hist["R"].append(R.copy()); hist["xc"].append(X[0:3] + R @ dyn["r_0c_0"])
        # the TRUE bias in the observer's transformed coordinates, xi = T V:
        # generalized forces map as Q_xi = T^-T Q_V (world translation, body rotation, joints)
        hist["dtrue"].append(np.linalg.solve(dyn["T"].T, np.concatenate([fbias_m, tbias_m, np.zeros(n)])))
        hist["b3c"].append(out["R0c"][:, 2].copy()); hist["b3"].append(R[:, 2].copy())
        hist["w0c"].append(out["omega_0c"].copy()); hist["w0"].append(X[15 + n:18 + n].copy())
        hist["dhat"].append(np.concatenate([out["d_t_hat"], out["d_r_hat"], np.zeros(n)]))
        if not np.isfinite(hist["ex"][-1]) or hist["tilt"][-1] > 35.0 or hist["ex"][-1] > 1.5:
            verdict = "ABORT"; break
    H = {k: np.asarray(v) for k, v in hist.items()}
    m = (H["t"] >= t_run0) & (H["t"] <= t_run1)
    rms = lambda v: float(np.sqrt(np.mean(v ** 2)))  # noqa: E731
    res = dict(verdict=verdict, t_end=float(H["t"][-1] - t_start),
               ee_abs_rms=rms(H["ee_abs"][m]) if m.any() else np.nan, ee_abs_peak=float(H["ee_abs"][m].max()) if m.any() else np.nan,
               com_rms=rms(H["ex"][m]) if m.any() else np.nan, com_peak=float(H["ex"][m].max()) if m.any() else np.nan,
               head_rms=rms(H["hd"][m]) if m.any() else np.nan,
               eR_mean=float(H["eR"][m].mean()) if m.any() else np.nan, tilt_peak=float(H["tilt"][m].max()) if m.any() else np.nan,
               tau_peak=float(H["tau"][m].max()) if m.any() else np.nan, sat_pct=float(100 * np.mean(H["nsat"][m] > 0)) if m.any() else np.nan,
               joint_err_rms=[rms((H["q"][m] - H["qd"][m])[:, j]) for j in range(4)] if m.any() else None)
    # DIRECT-entry handover (the observer starts at zero at t_start)
    we = H["t"] < t_run0
    res["entry_peak_com"] = float(H["ex"][we].max()) if we.any() else np.nan
    res["entry_tilt_pk"] = float(H["tilt"][we].max()) if we.any() else np.nan
    big = np.where(we & (H["ex"] > 0.030))[0]
    res["entry_t30"] = float(H["t"][big[-1]] - t_start) if len(big) else 0.0
    g_ = po.get("gust")
    if g_ is not None and verdict == "completed":
        ta = t_run0 + g_[0]
        pre = (H["t"] >= ta - 3.0) & (H["t"] < ta)
        base = float(np.sqrt(np.mean(H["ex"][pre] ** 2)))
        wg = (H["t"] >= ta) & (H["t"] < ta + g_[1] + 6.0)
        res["gust_peak_com"] = float(H["ex"][wg].max())
        res["gust_rise"] = float(H["ex"][wg].max() - base)
        post = (H["t"] >= ta) & (H["t"] < ta + g_[1] + 6.0) & (H["ex"] > base + 0.010)
        idx = np.where(post)[0]
        res["gust_recover"] = float(H["t"][idx[-1]] - (ta + g_[1])) if len(idx) else 0.0
    if "legs" in S and verdict == "completed":
        # transient metrics per leg: peak CoM / EE error from leg start to the next leg,
        # and the settle time after the leg ends (CoM error back inside 15 mm for good)
        lg = []
        L = S["legs"]
        for i, (a_, b_) in enumerate(L):
            c_ = L[i + 1][0] if i + 1 < len(L) else S["run"][1]
            w = (H["t"] >= a_) & (H["t"] < c_)
            after = (H["t"] >= b_) & (H["t"] < c_)
            big = np.where(after & (H["ex"] > 0.015))[0]
            settle = float(H["t"][big[-1]] - b_) if len(big) else 0.0
            if len(big) and big[-1] >= np.where(after)[0][-1] - 1:
                settle = float("inf")
            lg.append(dict(peak_com=float(H["ex"][w].max()), peak_ee=float(H["ee_abs"][w].max()),
                           settle=settle, tilt_pk=float(H["tilt"][w].max())))
        res["legs"] = lg
    return res, H


def fmt(name, r):
    if r["verdict"] != "completed":
        return f"{name:38} ABORT at {r['t_end']:.1f} s"
    je = r["joint_err_rms"]
    return (f"{name:38} EEabs {r['ee_abs_rms']*1e3:6.1f} mm (pk {r['ee_abs_peak']*1e3:5.0f}) | CoM {r['com_rms']*1e3:6.1f} (pk {r['com_peak']*1e3:5.0f}) "
            f"| head {r['head_rms']:5.2f} deg | |eR| {r['eR_mean']:.4f} | tilt pk {r['tilt_peak']:4.1f} | tau pk {r['tau_peak']:.2f} sat {r['sat_pct']:.1f}% "
            f"| q err {je[1]:.2f}/{je[2]:.2f}")


def run(name, gains, l1, S, results, plant=None, delay_ms=DELAY_MS):
    t0 = time.time()
    r, _ = simulate(gains, l1, S, plant_over=plant, delay_ms=delay_ms)
    r["gains"] = gains; r["l1"] = l1; r["plant"] = plant or {}; r["delay_ms"] = delay_ms
    results[name] = r
    print(fmt(name, r) + f"   [{time.time()-t0:.0f} s]", flush=True)
    return r


GATE_CANDS = {
    "hw (shipped)": ({}, {}),
    "wc 4/0.5/0.5": ({}, dict(omega_c_t=4.0)),
    "wc 2/1/0.5": ({}, dict(omega_c_r=1.0)),
    "wc 4/1/0.5": ({}, dict(omega_c_t=4.0, omega_c_r=1.0)),
    "wc 6/0.5/0.5": ({}, dict(omega_c_t=6.0)),
    "wc 6/1/0.5": ({}, dict(omega_c_t=6.0, omega_c_r=1.0)),
    "wc 6/1/1": ({}, dict(omega_c_t=6.0, omega_c_r=1.0, omega_c_q=1.0)),
    "kR 3 kw 1.84": (dict(k_R=3.0, k_w=1.84), {}),
    "kx 48 kv 24.5": (dict(k_x=48.0, k_v=24.5), {}),
}
GATE_TESTS = [("mirror", 16.0), ("mirror", 20.0), ("mirror", 24.0), ("mirror", 28.0),
              ("robustness", 16.0), ("robustness", 20.0)]


def _gate_job(args):
    nm, prof, dly = args
    g, l = GATE_CANDS[nm]
    S = load_stream("a2")
    r, _ = simulate(g, l, S, delay_ms=dly, profile=prof)
    r.update(gains=g, l1=l, profile=prof, delay_ms=dly)
    return f"{nm} | {prof} {dly:g} ms", r


def gate(jobs):
    import multiprocessing as mp
    tasks = [(nm, pr, d) for nm in GATE_CANDS for pr, d in GATE_TESTS]
    out = {}
    with mp.get_context("fork").Pool(jobs) as pool:
        for key, r in pool.imap_unordered(_gate_job, tasks):
            out[key] = r
            print(fmt(key, r), flush=True)
    return dict(sorted(out.items()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--case", default=None)
    ap.add_argument("--combo", action="store_true", help="combined candidates")
    ap.add_argument("--gate", action="store_true", help="candidates x (mirror delay scan + robustness), parallel")
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--margin", action="store_true", help="delay / kf / inertia margin on the candidates")
    ap.add_argument("--out", default=os.path.join(HERE, "..", "analysis", "circle_sweep.json"))
    a = ap.parse_args()
    S = load_stream("a2")
    print(f"stream: run {S['run'][0]:.1f}..{S['run'][1]:.1f} s ({S['run'][1]-S['run'][0]:.1f} s)")
    results = {}
    if a.case:
        g, l = {}, {}
        for kv in a.case.split(","):
            k, v = kv.split("="); v = float(v)
            (l if k.startswith("omega") or k.startswith("a_") else g)[k] = v
        run(a.case, g, l, S, results)
    if a.baseline or a.sweep:
        run("hw 2026-09-26 (K_y 80/24, mrd new)", {}, {}, S, results)
        run("as flown 0924 (K_y 20/12, mrd old)", dict(ky=(20.0, 20.0, 20.0, 0.3), dy=(12.0, 12.0, 12.0, 0.3), mrd=(0.116522, 0.136107, 0.125102)), {}, S, results)
    if a.sweep:
        # group 1: translational stiffness/damping (zeta kept ~0.9)
        for kx, kv in ((48.0, 24.5), (64.0, 28.3), (96.0, 34.6)):
            run(f"k_x {kx:g} k_v {kv:g}", dict(k_x=kx, k_v=kv), {}, S, results)
        # group 2: attitude pair
        for kr, kw in ((3.0, 1.84), (4.0, 2.12)):
            run(f"k_R {kr:g} k_w {kw:g}", dict(k_R=kr, k_w=kw), {}, S, results)
        # group 3: L1 filter bandwidths
        for ct, cr, cq in ((4.0, 0.5, 0.5), (4.0, 1.0, 0.5), (6.0, 1.0, 1.0)):
            run(f"omega_c {ct:g}/{cr:g}/{cq:g}", {}, dict(omega_c_t=ct, omega_c_r=cr, omega_c_q=cq), S, results)
        # group 4: u3 feedforward filter
        for ox in (0.5, 1.0):
            run(f"omega_x {ox:g}", {}, dict(omega_x=ox), S, results)
        # group 5: EE impedance
        run("K_y 120/36", dict(ky=(120.0, 120.0, 120.0, 0.3), dy=(36.0, 36.0, 36.0, 0.3)), {}, S, results)
        run("K_y psi 0.6", dict(ky=(80.0, 80.0, 80.0, 0.6), dy=(24.0, 24.0, 24.0, 0.6)), {}, S, results)
    G_KR = dict(k_R=3.0, k_w=1.84)
    G_KX = dict(k_x=48.0, k_v=24.5)
    L_4 = dict(omega_c_t=4.0, omega_c_r=1.0, omega_c_q=0.5)
    L_6 = dict(omega_c_t=6.0, omega_c_r=1.0, omega_c_q=1.0)
    CANDS = {"C1 wc4/1/.5 + kR3": (dict(G_KR), dict(L_4)),
             "C2 C1 + kx48": ({**G_KR, **G_KX}, dict(L_4)),
             "C3 wc6/1/1 + kR3": (dict(G_KR), dict(L_6)),
             "C4 wc4/1/.5 + kx48": (dict(G_KX), dict(L_4)),
             "C5 wc6/1/1 + kR3 + kx48": ({**G_KR, **G_KX}, dict(L_6))}
    if a.combo:
        for nm, (g, l) in CANDS.items():
            run(nm, g, l, S, results)
    if a.margin:
        SINGLES = {"wc 6/1/1": ({}, dict(L_6)), "wc 4/1/.5": ({}, dict(L_4)),
                   "wc 4/.5/.5": ({}, dict(omega_c_t=4.0)), "kR 3": (dict(G_KR), {}), "kx 48": (dict(G_KX), {})}
        for nm, (g, l) in [("hw", ({}, {}))] + list(SINGLES.items()):
            for tag, kw in (("delay 32 ms", dict(delay_ms=32.0)), ("plant kf x1.10", dict(plant=dict(kf=1.10))),
                            ("plant I x1.25", dict(plant=dict(inertia=1.25))), ("plant I x0.8", dict(plant=dict(inertia=0.8)))):
                run(f"{nm} | {tag}", g, l, S, results, **kw)
    if a.gate:
        results.update(gate(a.jobs))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump({k: {kk: (v.tolist() if hasattr(v, "tolist") else v) for kk, v in r.items()} for k, r in results.items()},
              open(a.out, "w"), indent=1, default=float)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
