#!/usr/bin/env python3
"""Clamped-box push bench (2026-10-25): the 10-24 evening hardware pushes, offline.

The 10-22 contact-tune bench (box_pushing_contact_stability_tune_20261021/tools: circle_bench.py
-> wb_entry_sim.Law = controller.py + l1_observer.py, the C++ law's source of truth; wh_bench.py's
room hold + free-axis anchor + the box-pushing yaml) with ONE addition, on the plant side only (the
law never learns it, as in flight): the box clamped in the jaws.
  * its mass rides the claw (added to the last link at the EE point);
  * a stiff one-sided table under the claw carries its weight (penalty k_t, c_t);
  * Coulomb friction mu * N on the claw's horizontal velocity, regularised over v_eps;
  * all three enter the plant as a world force F at the claw: Q += (J_y T)^T [F; 0].
The box is a point at the claw: the clamp + table carry the friction's moment (no tipping).

The reference is the planner's push, synthesised: the flight's own hand-over hold (x_cd, r_ed,
b1_d, b1_de, q_d at 48.5 s of 194310), then a septic rest-to-rest slide of `dist` m along the push
heading in `T_push` s (the planner's profile: peak 2.19 x the mean), after a 1.5 s clamped hold.
Feedback: raw mocap (10-21 noise) + the 10-22 hardware-like drift (a slow random horizontal world
force, 0.32 N / axis at 0.12 Hz -- PX4's attitude-estimate error, `hw-wander-attitude-error`).

Usage: box_bench.py            (the candidate table, 3 seeds each)
"""
import copy
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", ".."))
CT = os.path.join(ARCH, "box_pushing_contact_stability_tune_20261021", "tools")
sys.path.insert(0, CT)
import wh_bench as WH          # noqa: E402  (patches CB.ES.Case: room hold + free-axis weight)
CB, CS, ES = WH.CB, WH.CS, WH.ES
C, TP = CB.C, CB.TP
N = 4
NPZ = os.path.join(HERE, "..", "npz", "b194310.npz")
T_HOLD = 48.5                  # the flight's room-held hold at the grasp point (before the push)
# the REAL arm's range (external_torque_controller_hardware_aerial_pwm.yaml min/max_position): q2 [-20, 45]
QMAX = np.array(ES.Q_MAX, float).copy(); QMIN = np.array(ES.Q_MIN, float).copy()
QMAX[1] = np.radians(45.0); QMIN[1] = np.radians(-20.0)
L_BOX, H_GRASP = 0.240, 0.233  # box length along the push, grasp height above the table [m]


def flight_hold():
    d = np.load(NPZ, allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
    t = d["wbref__recv"] - t0
    i = int(np.searchsorted(t, T_HOLD))
    r = {k: np.array([d[f"wbref__{k}.{a}"][i] for a in "xyz"]) for k in ("x_cd", "r_ed", "b1_d", "b1_de")}
    r["q_d"] = np.array(d["wbref__q_d"][i], float)
    return r


def push_stream(dist=0.50, T_push=23.81, settle=1.5, t_after=4.0, heading_deg=-90.0, dt=0.01):
    """The push as a bench stream: hold, `settle` s clamped, septic slide, hold."""
    r0 = flight_hold()
    u = np.array([np.cos(np.radians(heading_deg)), np.sin(np.radians(heading_deg)), 0.0])
    t_run0 = 10.0
    t = np.arange(0.0, t_run0 + settle + T_push + t_after, dt)
    tau = np.clip((t - t_run0 - settle) / T_push, 0.0, 1.0)
    on = ((t - t_run0 - settle) > 0) & ((t - t_run0 - settle) < T_push)
    c = [35.0, -84.0, 70.0, -20.0]                       # s = sum c_k tau^(k+4), rest-to-rest septic
    s = sum(ck * tau ** (k + 4) for k, ck in enumerate(c))
    D = [s]
    for n in range(1, 5):
        dn = np.zeros_like(t)
        for k, ck in enumerate(c):
            p = k + 4
            if p >= n:
                coef = np.prod(np.arange(p, p - n, -1))
                dn += ck * coef * tau ** (p - n)
        D.append(np.where(on, dn / T_push ** n, 0.0))
    D = [dist * x for x in D]
    S = {"t": t}
    z3 = np.zeros((len(t), 3))
    S["x_cd"] = r0["x_cd"] + np.outer(D[0], u)
    S["r_ed"] = r0["r_ed"] + np.outer(D[0], u)
    for k, n in (("x_cd_dot", 1), ("x_cd_ddot", 2), ("x_cd_d3", 3), ("x_cd_d4", 4)):
        S[k] = np.outer(D[n], u)
    S["r_ed_dot"] = np.outer(D[1], u); S["r_ed_ddot"] = np.outer(D[2], u)
    S["b1_d"] = np.tile(r0["b1_d"], (len(t), 1)); S["b1_de"] = np.tile(r0["b1_de"], (len(t), 1))
    for k in ("b1_d_dot", "b1_d_ddot", "b1_de_dot", "b1_de_ddot"):
        S[k] = z3.copy()
    S["q_d"] = np.tile(r0["q_d"], (len(t), 1)); S["qdot_d"] = np.zeros((len(t), N))
    S["run"] = (t_run0, t_run0 + settle + T_push + t_after)
    S["push"] = (t_run0 + settle, t_run0 + settle + T_push)
    S["u"] = u; S["dist"] = dist; S["name"] = f"push_{dist:.2f}m_{T_push:.1f}s"
    return S


def add_box(plant, mb):
    """The box as a point mass at the EE point (the claw), on the last link."""
    p = copy.deepcopy(plant)
    if mb <= 0:
        return p
    m4 = p["m_i"][N]; c4 = np.asarray(p["com_i"][N - 1], float); l4 = np.asarray(p["l_i"][N - 1], float)
    cn = (m4 * c4 + mb * l4) / (m4 + mb)
    d4, db = c4 - cn, l4 - cn
    I = np.asarray(p["I_i_i"][N], float)
    p["I_i_i"][N] = I + m4 * (d4 @ d4 * np.eye(3) - np.outer(d4, d4)) + mb * (db @ db * np.eye(3) - np.outer(db, db))
    p["m_i"][N] = m4 + mb
    p["com_i"][N - 1] = cn
    return p


def simulate(p, S, box=None, fb=None, seed=0, t_settle=15.0, delay_ms=16.0, ee_ref="world", free_w=0.75,
             contact=True, t_hand_lead=0.6, blend_s=1.0, fy_noise=None):
    """The law on the push stream with the clamped box. p = gain dict (WH.PUSH keys).
    box = dict(m, mu, v_eps, k_t, c_t); None = no box (free hover on the same reference).
    Staged as flown: the claw CoM-anchored and no box for t_settle (the observer's entry transient
    settles), the room hold + free-axis weight blended in over blend_s from t_run0 - t_hand_lead
    (the planner's hand-over), the box clamped and the contact flag on at t_run0 (the push step;
    its 1.5 s clamped hold then the slide). ee_ref "relative" keeps the CoM anchor throughout."""
    WH.EE_REF[0] = "relative"
    WH.EE_FREE_W[0] = 0.0
    fb = dict(WH.FB_RAW if fb is None else fb)
    rng = np.random.default_rng(seed)
    model = TP.make_params_t650(armature_diag=CS.J_ARM)
    l1 = dict(omega_c_t=p["omega_c_t"], omega_c_r=p["omega_c_r"], omega_c_q=p["omega_c_q"], omega_x=p["omega_x"],
              omega_q=p["omega_q"], omega_x_t=p["omega_x_blk"], omega_x_r=p["omega_x_blk"],
              omega_x_q=p["omega_x_blk"], contact=False)
    case = ES.Case(posture=0.0, ee_ref="relative", int_ff=True, int_ff_source="four_d",
                   mass=1.0, inertia=1.0, com=(-0.017854, 0.0, 0.0), kf=1.0, kf_alloc=1.0,
                   observer="l1", delay_ms=delay_ms, gains={}, l1=CS.hw_l1(**l1))
    plant = ES.plant_params(model, case)
    model["base_com"] = CB.R_MODEL.T @ np.array([-0.017854, 0.0, 0.0])
    bx = dict(m=0.28, mu=0.25, v_eps=0.004, k_t=5000.0, c_t=60.0, k_c=0.0, c_c=0.05, W=0.160) | (box or {})
    # THE CLAMP MOMENT (k_c > 0): the jaws hold the fin of a box standing on the table, so the claw's
    # TILT (pitch about the push normal, roll about the push axis) is tied to the box's through the
    # pads / fin / bracket compliance k_c [N.m/rad], capped at the box's tipping moment m g L/2
    # (pitch, L = 0.24 along the push) / m g W/2 (roll, W = 0.16) -- beyond it the box rocks on an edge.
    plant_box = add_box(plant, bx["m"]) if box is not None else plant
    kf_alloc, km_ratio, fric_plant = CS.KF_ALLOC, CS.KM_ALLOC_RATIO, CS.FRIC_SCALE
    fb_body, tb_body = CS.FORCE_BIAS_BODY, CS.TORQUE_BIAS_BODY
    kf_plant, km_plant = CS.KF_PLANT, CS.KM_PLANT
    ffs = CS.FF_SCALE
    cfg = CB.make_cfg(p)
    law = ES.Law(model, cfg, case)
    dt = case.dt
    # the hardware's contact reading (fy_noise = (sigma_al, sigma_ac, sigma_z) N, bw Hz): an AR(1) added
    # to the 4-D observer's F_y (task frame: model x = right, y = nose, z = up) while the contact flag is
    # on -- the flight's F_hat_y is ~25x noisier than the bench's; also logs the consumed F_y
    FY = {"last": np.zeros(4)}
    if law.l1 is not None:
        _upd = law.l1.update
        fyn = CB._AR1(np.random.default_rng(seed + 1000), 1.0, fy_noise[3], dt, 3) if fy_noise else None
        def _update(*a, **k):
            est = _upd(*a, **k)
            if fyn is not None and law.l1.g.contact:
                z = fyn() * np.array(fy_noise[:3], float)
                est = dict(est); est["F_y"] = np.asarray(est["F_y"], float) + np.array([z[1], z[0], z[2], 0.0])
            FY["last"] = np.asarray(est["F_y"], float)
            return est
        law.l1.update = _update
    B = np.zeros((4, 4))
    for i, r in enumerate(ES.ROTOR_POS):
        B[0, i] = 1.0; B[1, i] = r[1]; B[2, i] = -r[0]; B[3, i] = ES.ROT_DIR[i] * km_ratio
    Bp = np.linalg.pinv(B)
    rp = ES.ROTOR_POS; rd = ES.ROT_DIR

    def allocate(thrust, tau):
        f = np.maximum(Bp @ np.array([thrust, *tau]), 0.0)
        return np.clip(np.sqrt(f / kf_alloc), 0.0, ES.OMEGA_MAX)

    fbias_m = CB.R_MODEL.T @ fb_body; tbias_m = CB.R_MODEL.T @ tb_body
    t_run0, t_run1 = S["run"]
    i_hold = max(np.searchsorted(S["t"], t_run0) - 1, 0)
    r0 = CS.ref_at(S, 0, i_hold)
    X = np.zeros(12 + 3 * N + 6)
    X[12:12 + N] = r0["q_d"]
    psi = np.arctan2(r0["b1_d"][1], r0["b1_d"][0])
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1.0]])
    X[3:12] = Rz.flatten(order="F")
    X[0:3] = r0["x_cd"] - Rz @ C.dynamics(X, model)["r_0c_0"]
    dyn0 = C.dynamics(X, plant)
    z_rest = None                                             # the table height, set at the clamp
    t_hand, t_clamp = S["run"][0] - t_hand_lead, S["run"][0]
    clamped = False; Re_clamp = None
    gp = dyn0["g"]
    omega_rot = allocate(gp[2], gp[3:6])
    n_del = int(round(delay_ms * 1e-3 / dt))
    fifo = [omega_rot.copy() for _ in range(n_del)]
    n_odo = int(round(fb["odo_lag_s"] / dt)); n_att = int(round(fb["att_lag_s"] / dt))
    hist_X = [X.copy() for _ in range(max(n_odo, n_att) + 1)]
    vnoise = CB._AR1(rng, fb["vel_noise"], fb["vel_bw_hz"], dt, 3)
    fnoise = CB._AR1(rng, fb["force_noise"], fb.get("force_bw_hz", 0.3), dt, 2) if fb.get("force_noise", 0) > 0 else None
    a_qd = np.exp(-dt / fb["qd_lag_s"]) if fb["qd_lag_s"] > 0 else 0.0
    qd_obs = X[18 + N:18 + 2 * N].copy()
    t_start = t_run0 - t_settle
    steps = int((t_run1 - t_start) / dt)
    keys = ("t", "ex_al", "ex_ac", "ee_al", "claw_al", "claw_v", "N", "Ff", "eR_p", "tilt", "tau_pk", "nsat", "a_al", "fy_al")
    H = {k: np.zeros(steps) for k in keys}
    Hq = np.zeros((steps, N)); Href = np.zeros(steps)
    u = S["u"]; nrm = np.array([-u[1], u[0], 0.0])
    verdict = "completed"; k_end = steps
    for k in range(steps):
        t = t_start + k * dt
        if t < t_run0:
            ref = dict(CS.ref_at(S, t, i_hold))
        else:
            ref = CS.ref_at(S, t, int(np.searchsorted(S["t"], t)))
        Xo = hist_X[-1 - n_odo]; Xa = hist_X[-1 - n_att]
        Ra = Xa[3:12].reshape(3, 3, order="F")
        if fb["att_noise"] > 0:
            dv = rng.standard_normal(3) * fb["att_noise"]; nv = np.linalg.norm(dv)
            Ra = Ra @ C.joint_rotation(dv / (nv + 1e-12), nv)
        Ro = Xo[3:12].reshape(3, 3, order="F")
        v_world = Ro @ Xo[12 + N:15 + N] + (vnoise() if fb["vel_noise"] > 0 else 0.0)
        Xm = X.copy()
        Xm[0:3] = Xo[0:3] + (rng.standard_normal(3) * fb["pos_noise"] if fb["pos_noise"] > 0 else 0.0)
        Xm[3:12] = Ra.flatten(order="F")
        Xm[12 + N:15 + N] = Ra.T @ v_world
        Xm[15 + N:18 + N] = X[15 + N:18 + N] + (rng.standard_normal(3) * fb["gyro_noise"] if fb["gyro_noise"] > 0 else 0.0)
        qt = X[12:12 + N]
        Xm[12:12 + N] = np.round(qt / fb["enc_quant"]) * fb["enc_quant"] if fb["enc_quant"] > 0 else qt
        qd_obs = a_qd * qd_obs + (1 - a_qd) * X[18 + N:18 + 2 * N]
        Xm[18 + N:18 + 2 * N] = qd_obs + (rng.standard_normal(N) * fb["qd_noise"] if fb["qd_noise"] > 0 else 0.0)
        dyn = C.dynamics(Xm, model)
        if ee_ref == "world" and t >= t_hand:              # the hand-over: blend CoM anchor -> room hold
            beta = min(1.0, (t - t_hand) / blend_s)
            law.case.ee_ref = "world"; law.case.ee_free_w = beta * float(free_w)
            Rm = Xm[3:12].reshape(3, 3, order="F")
            e_x = Xm[0:3] + Rm @ dyn["r_0c_0"] - ref["x_cd"]
            e_v = v_world - ref["x_cd_dot"]
            ref = dict(ref); ref["r_ed"] = ref["r_ed"] + (1 - beta) * e_x
            ref["r_ed_dot"] = ref["r_ed_dot"] + (1 - beta) * e_v
        if not clamped and t >= t_clamp:                   # the jaws close: box on, contact flag on
            clamped = True
            if law.l1 is not None:
                object.__setattr__(law.l1.g, "contact", bool(contact))
            if box is not None:
                plant = plant_box
                Rt = X[3:12].reshape(3, 3, order="F")
                dq = C.dynamics(X, plant)
                z_rest = (X[0:3] + Rt @ dq["r_0e_0"])[2] + bx["m"] * 9.81 / bx["k_t"]
                Re_clamp = Rt @ dq["R_e_0"]
        out = law(Xm, dyn, ref, dt)
        u1 = float(out["u1"]); tau_body = np.asarray(out["tau_body"], float)
        tau_joint = np.clip(np.asarray(out["tau_joint"], float), -cfg.tau_max, cfg.tau_max)
        q = X[12:12 + N]; qd = X[18 + N:18 + 2 * N]
        load = np.abs(tau_joint)
        tau_joint = tau_joint + ffs * (CS.FRIC_FC + CS.FRIC_MU * load) * np.tanh(Xm[18 + N:18 + 2 * N] / CS.FF_W_MEAS)
        w_cmd = allocate(u1, tau_body)
        if n_del > 0:
            fifo.append(w_cmd); w_cmd = fifo.pop(0)
        omega_rot = w_cmd + (omega_rot - w_cmd) * np.exp(-ES.LAMBDA_ROTOR * dt)
        fr_ = kf_plant * omega_rot ** 2
        thrust_a = fr_.sum()
        tau_a = np.array([np.dot(rp[:, 1], fr_), -np.dot(rp[:, 0], fr_), np.dot(rd, km_plant * omega_rot ** 2)])
        fr = fric_plant * (CS.FRIC_FC + CS.FRIC_MU * load) * np.tanh(qd / CS.FRIC_W)
        fr = np.clip(fr, -0.02 * np.abs(qd) / dt, 0.02 * np.abs(qd) / dt)
        over = q - QMAX; under = QMIN - q
        tau_stop = (-np.where(over > 0, ES.K_STOP * over + ES.C_STOP * np.maximum(qd, 0), 0.0)
                    + np.where(under > 0, ES.K_STOP * under + ES.C_STOP * np.maximum(-qd, 0), 0.0))
        dynp = C.dynamics(X, plant)
        R = X[3:12].reshape(3, 3, order="F")
        fb_now = fbias_m
        if fnoise is not None:
            f2 = fnoise(); fb_now = fb_now + R.T @ np.array([f2[0], f2[1], 0.0])
        Q = np.concatenate([fb_now + [0.0, 0.0, thrust_a], tau_a + tbias_m, tau_joint - fr + tau_stop])
        xi = np.concatenate([X[12 + N:15 + N], X[15 + N:18 + N], X[18 + N:18 + 2 * N]])
        # ---- the clamped box: table + Coulomb friction at the claw ----
        JyT = dynp["J_y"] @ dynp["T"]
        claw = X[0:3] + R @ dynp["r_0e_0"]
        cv = (JyT @ xi)[0:3]
        Nn = Ff_al = 0.0
        if box is not None and clamped:
            pen = z_rest - claw[2]
            Nn = max(0.0, bx["k_t"] * pen - bx["c_t"] * cv[2]) if pen > 0 else 0.0
            vh = np.array([cv[0], cv[1], 0.0])
            Ff = -bx["mu"] * Nn * vh / np.sqrt(vh @ vh + bx["v_eps"] ** 2)
            Fw = Ff + np.array([0.0, 0.0, Nn])
            Q = Q + JyT.T @ np.array([Fw[0], Fw[1], Fw[2], 0.0])
            Ff_al = float(Ff @ u)
            if bx["k_c"] > 0:
                Re = R @ dynp["R_e_0"]
                dR = Re @ Re_clamp.T
                phi = 0.5 * np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
                w_e = R @ (xi[3:6] + dynp["J_q_omega_e"] @ xi[6:6 + N])          # claw angular velocity, world
                Mw = np.zeros(3)
                for ax, lim in ((nrm, bx["m"] * 9.81 * L_BOX / 2), (u, bx["m"] * 9.81 * bx["W"] / 2)):
                    mom = -bx["k_c"] * float(phi @ ax) - bx["c_c"] * float(w_e @ ax)
                    Mw += np.clip(mom, -lim, lim) * ax
                Mb = R.T @ Mw
                Q[3:6] = Q[3:6] + Mb
                Q[6:6 + N] = Q[6:6 + N] + dynp["J_q_omega_e"].T @ Mb
        xi = xi + np.linalg.solve(dynp["M"], Q - dynp["C"] @ xi - dynp["g"]) * dt
        v0, w0, qdn = xi[0:3], xi[3:6], xi[6:6 + N]
        x_c = X[0:3] + R @ dyn["r_0c_0"]
        H["t"][k] = t
        H["ex_al"][k] = (x_c - ref["x_cd"]) @ u
        H["ex_ac"][k] = (x_c - ref["x_cd"]) @ nrm
        H["ee_al"][k] = (claw - ref["r_ed"]) @ u
        H["claw_al"][k] = claw @ u
        H["claw_v"][k] = cv @ u
        H["N"][k] = Nn; H["Ff"][k] = Ff_al; H["fy_al"][k] = FY["last"][1]
        H["a_al"][k] = ((JyT @ xi)[0:3] - cv) @ u / dt if (box is not None and clamped) else 0.0
        H["eR_p"][k] = float(np.asarray(out["e_R"])[0])
        H["tilt"][k] = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        H["tau_pk"][k] = np.abs(tau_joint).max(); H["nsat"][k] = out["n_sat"]
        Hq[k] = q; Href[k] = ref["r_ed"] @ u
        X[0:3] = X[0:3] + R @ v0 * dt
        nw = np.linalg.norm(w0)
        R = R @ C.joint_rotation(w0 / (nw + 1e-12), nw * dt)
        uu, _, vt = np.linalg.svd(R); R = uu @ vt
        X[3:12] = R.flatten(order="F")
        X[12:12 + N] = X[12:12 + N] + qdn * dt
        X[12 + N:18 + N] = np.concatenate([v0, w0]); X[18 + N:18 + 2 * N] = qdn
        hist_X.append(X.copy()); hist_X.pop(0)
        if not np.isfinite(H["ex_al"][k]) or H["tilt"][k] > 25.0 or abs(H["ex_al"][k]) > 1.0:
            verdict = "ABORT"; k_end = k + 1; break
    H = {kk: v[:k_end] for kk, v in H.items()}
    H["q"] = Hq[:k_end]; H["ref_al"] = Href[:k_end]
    return verdict, H


def score(S, verdict, H):
    """Push-window metrics: box (= claw) speed vs plan, reversals, the vehicle's along error,
    pitch error, joint ranges. Same definitions as moment_fit.py on the flight."""
    t = H["t"]
    a, b = S["push"]
    m = (t >= a) & (t <= b)
    out = dict(verdict=verdict)
    if verdict != "completed" or m.sum() < 10:
        out["t_end"] = float(t[-1] - t[0]); return out
    v = H["claw_v"][m] * 1e3
    nz = np.sign(np.where(np.abs(v) < 5, 0, v)); nz = nz[nz != 0]
    e = H["ex_al"][m]
    q = np.degrees(H["q"][m])
    vt = np.diff(H["claw_al"][m]) / np.diff(t[m])
    out.update(v_std=float(np.std(v)), back=float(np.mean(v < -5)), rev=int(np.sum(np.diff(nz) != 0)),
               v_min=float(v.min()), v_max=float(v.max()),
               ex_rms=float(1e3 * np.std(e)), ex_pp=float(1e3 * (e.max() - e.min())),
               eR_rms=float(1e3 * np.std(H["eR_p"][m])), tilt_pk=float(H["tilt"][m].max()),
               q2=(float(q[:, 1].min()), float(q[:, 1].max())), q3=(float(q[:, 2].min()), float(q[:, 2].max())),
               q2_wall=float(np.mean(q[:, 1] > 44.0)),
               lag_rms=float(1e3 * np.std(H["claw_al"][m] - H["ref_al"][m])),
               Ff=float(np.mean(np.abs(H["Ff"][m]))), tau_pk=float(H["tau_pk"][m].max()),
               fy_rms=float(np.std(H["fy_al"][m])), tilt_rms=float(np.std(H["tilt"][m])))
    # the planner's q3 rate guard (q2-aware, 2026-10-24), replayed on the bench joints, whole run
    qa = np.degrees(H["q"]); v3 = np.zeros(len(t)); v2 = np.zeros(len(t)); since = None; trip = None
    for i in range(1, len(t)):
        d = t[i] - t[i - 1]; g = 1 - np.exp(-d / 0.02)
        v3[i] = v3[i - 1] + g * ((qa[i, 2] - qa[i - 1, 2]) / d - v3[i - 1])
        v2[i] = v2[i - 1] + g * ((qa[i, 1] - qa[i - 1, 1]) / d - v2[i - 1])
        pred = qa[i, 2] + 0.2 * v3[i] + min(1.0, max(0.0, qa[i, 1] + 0.2 * v2[i] - 45.0))
        if t[i] > a - 1.5 and ((pred < 10.0 and v3[i] < -5.0) or qa[i, 2] < 10.0):
            since = t[i] if since is None else since
            if t[i] - since >= 0.05 and trip is None:
                trip = t[i]
        else:
            since = None
    out["q3_trip"] = None if trip is None else float(trip - a)
    # the box's tipping: the horizontal claw force needed (m a - friction) vs m g (L/2) / h_grasp
    mb = 0.28 if "m_box" not in S else S["m_box"]
    Fc = np.abs(mb * H["a_al"][m] - H["Ff"][m])
    Fc_lp = np.convolve(Fc, np.ones(25) / 25, mode="same")
    out["tip"] = float(Fc_lp.max() / (mb * 9.81 * (L_BOX / 2) / H_GRASP)) if mb > 0 else 0.0
    return out


def fmt(name, o):
    if o["verdict"] != "completed":
        return f"{name:46s} {o['verdict']} at {o.get('t_end', 0):.1f} s"
    return (f"{name:46s} box v std {o['v_std']:5.1f} mm/s [{o['v_min']:5.0f},{o['v_max']:4.0f}] back {o['back']:4.0%} "
            f"rev {o['rev']:2d} | CoM al rms {o['ex_rms']:4.1f} p-p {o['ex_pp']:5.1f} mm | eR {o['eR_rms']:4.1f} mrad "
            f"tilt {o['tilt_pk']:4.1f} F_y al rms {o['fy_rms']:4.2f} | q2 {o['q2'][0]:4.1f}..{o['q2'][1]:4.1f} ({o['q2_wall']:3.0%} >44) "
            f"q3 {o['q3'][0]:4.1f}..{o['q3'][1]:4.1f} | box-plan {o['lag_rms']:4.1f} mm tip {o['tip']:.2f} | "
            + ("q3 guard --" if o["q3_trip"] is None else f"q3 guard TRIP {o['q3_trip']:+.1f}s"))


def job(args):
    name, p, kw, S_kw, seed = args
    S = push_stream(**S_kw)
    S["m_box"] = (kw.get("box") or {}).get("m", 0.0) if kw.get("box") is not None else 0.0
    v, H = simulate(p, S, seed=seed, **kw)
    return name, seed, score(S, v, H)


HW = dict(WH.FB_RAW, force_noise=0.32, force_bw_hz=0.12)

P = dict(WH.PUSH)
B28, B51 = dict(m=0.28), dict(m=0.51)
CASES = {
    "A0": ("no box (hover on the push ref)", P, dict(box=None, fb=HW), {}),
    "A1": ("280 g, as flown", P, dict(box=B28, fb=HW), {}),
    "A2": ("280 g, as flown, no drift", P, dict(box=B28, fb=WH.FB_RAW), {}),
    "A3": ("510 g, as flown", P, dict(box=B51, fb=HW), {}),
    "B1": ("280 g, free-axis anchor OFF (w 0)", P, dict(box=B28, fb=HW, free_w=0.0), {}),
    "B2": ("280 g, free-axis w 0.4", P, dict(box=B28, fb=HW, free_w=0.4), {}),
    "B3": ("280 g, claw CoM-anchored (all axes)", P, dict(box=B28, fb=HW, ee_ref="relative", free_w=0.0), {}),
    "C1": ("280 g, push 30 mm/s", P, dict(box=B28, fb=HW), dict(T_push=0.5 / 0.030)),
    "C2": ("280 g, push 40 mm/s", P, dict(box=B28, fb=HW), dict(T_push=0.5 / 0.040)),
    "D1": ("280 g, k_R/k_w 2.134/1.567 (H1b)", P | dict(k_R=2.134, k_w=1.567), dict(box=B28, fb=HW), {}),
    "D2": ("280 g, omega_c_r 1.2", P | dict(omega_c_r=1.2), dict(box=B28, fb=HW), {}),
    "E1": ("280 g, contact flag off (chi free)", P, dict(box=B28, fb=HW, contact=False), {}),
    "F1": ("510 g, free-axis anchor OFF", P, dict(box=B51, fb=HW, free_w=0.0), {}),
    # the clamp moment (k_c, N.m/rad), sized from the flight: deadbeat pitch moment / pitch error ~ 3-5
    "K3": ("280 g, clamp k_c 3", P, dict(box=B28 | dict(k_c=3.0), fb=HW), {}),
    "K6": ("280 g, clamp k_c 6", P, dict(box=B28 | dict(k_c=6.0), fb=HW), {}),
    "K12": ("280 g, clamp k_c 12", P, dict(box=B28 | dict(k_c=12.0), fb=HW), {}),
    "K6n": ("280 g, clamp k_c 6, no drift", P, dict(box=B28 | dict(k_c=6.0), fb=WH.FB_RAW), {}),
    "K6h": ("510 g, clamp k_c 6", P, dict(box=B51 | dict(k_c=6.0), fb=HW), {}),
    # the hardware's contact reading noise on the rendered F_y (flight: 0.96-1.50 N rms along, 0.24-0.32 across)
    "N1": ("280 g, F_y noise 1.0/0.3/0.3 N @0.3 Hz", P, dict(box=B28 | dict(k_c=6.0), fb=HW, fy_noise=(1.0, 0.3, 0.3, 0.3)), {}),
    "N1off": ("280 g, F_y noise, contact flag OFF", P, dict(box=B28 | dict(k_c=6.0), fb=HW, fy_noise=(1.0, 0.3, 0.3, 0.3), contact=False), {}),
    "N2": ("280 g, F_y noise 1.5/0.3/0.3 N", P, dict(box=B28 | dict(k_c=6.0), fb=HW, fy_noise=(1.5, 0.3, 0.3, 0.3)), {}),
    "N1h": ("510 g, F_y noise 1.0/0.3/0.3 N", P, dict(box=B51 | dict(k_c=6.0), fb=HW, fy_noise=(1.0, 0.3, 0.3, 0.3)), {}),
    "K6b": ("280 g, clamp k_c 6 (baseline, F_y logged)", P, dict(box=B28 | dict(k_c=6.0), fb=HW), {}),
}


if __name__ == "__main__":
    sel = os.environ.get("CASES", ",".join(CASES)).split(",")
    seeds = [int(s) for s in os.environ.get("SEEDS", "1,2,3").split(",")]
    jobs = [(c, *CASES[c][1:], s) for c in sel for s in seeds]
    names = {c: CASES[c][0] for c in sel}
    with ProcessPoolExecutor(max_workers=int(os.environ.get("JOBS", "8"))) as ex:
        for c, seed, o in ex.map(job, jobs):
            print(fmt(f"{c} {names[c]} [s{seed}]", o), flush=True)
