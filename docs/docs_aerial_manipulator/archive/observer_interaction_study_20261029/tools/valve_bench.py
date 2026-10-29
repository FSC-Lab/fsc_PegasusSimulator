#!/usr/bin/env python3
"""Clamped-valve bench (2026-10-29, the observer / interaction-mode study) -- isolated, offline.

The 10-25 clamped-box bench (archive/wb_box_pushing_flight_20261024_evening/tools/box_bench.py: the exact
Python law wb_entry_sim.Law = controller.py + l1_observer.py, the C++ law's source of truth, on the mirror
plant with hardware-like feedback, the room hold + free-axis anchor staged as flown) with the BOX replaced
by the VALVE the 10-28 hardware flights grasped, on the plant side only (the law never learns it):

  * the fin point p_f(theta) = axis + Rz(theta) (pin - axis) at the clamp, theta the rotor's angle;
  * the claw tied to it by the grip: a stiff 3-D spring-damper (k_c, c_c) -- RADIAL and VERTICAL are
    the bearing's constraint, TANGENTIAL drives the rotor -- and a torsional one on the claw's heading
    (k_g, c_g; the 10-26 sweep measured the grip's twist stiffness at 0.1-0.8 N.m/rad);
  * the rotor: I theta_dd = (r x F_on_fin).z + M_grip - f_c tanh(theta_d / w), momentum-clamped.

What the earlier benches did not carry, each a default-off knob (the HYPOTHESES):
  arm_delay_ms   the arm-torque transport (law -> arm controller -> servos)
  stiction       arm-joint static friction  f_s = stiction x f_c below v_s (Stribeck)
  att_acc        a PX4-like ACCELERATION-CORRELATED attitude error: the law's tilt = the true tilt +
                 att_acc * LP_tau(a_horizontal) / g (world frame; >0 = PX4 over-reads the tilt, the
                 sign measured on 10-28 flight 2), att_tau s
  fy_noise       noise on the contact reading F_y while the contact flag is on (as the box bench)
  contact        the contact flag (chi) on at the clamp, or never
  odo_lag_s      the mocap -> law transport (fb dict)

The reference is the flight's own WholeBodyReference (10-28 flight 1, the claw on the fin before the
turn) held constant: a CLAMPED HOLD, no turn -- the hardware sway was already growing before the turn
moved, so a contact-loop instability must show here. Scoring on the TRUE state: the CoM error across
and along the nose (the hardware's lateral sway = ACROSS), its dominant frequency, and the envelope
growth rate (log of half-cycle peaks).
"""
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ARCH, "box_pushing_contact_stability_tune_20261021", "tools"))
import wh_bench as WH          # noqa: E402  (circle_bench + the room hold + the free-axis weight)
CB, CS, ES = WH.CB, WH.CS, WH.ES
C, TP = CB.C, CB.TP
N = 4
NPZ = os.path.join(ARCH, "wb_valve_turning_flight_20261028", "npz", "b180652.npz")
T_HOLD = 38.45                 # flight 1: the claw on the fin, room-held, the jaws closing
AXIS = np.array([0.996, 0.011])  # the planner's captured axis (flight 1 log)
QMAX = np.array(ES.Q_MAX, float).copy(); QMIN = np.array(ES.Q_MIN, float).copy()
QMAX[1] = np.radians(45.0); QMIN[1] = np.radians(-20.0)
# the VALVE yaml's law (= the box push's + omega_c_t 2.0, make_valve_turning_yaml.py)
VALVE = dict(WH.PUSH, omega_c_t=2.0)
HW = dict(WH.FB_RAW, force_noise=0.32, force_bw_hz=0.12)    # raw mocap + the hardware-like drift
I_ROT = 0.0031 * 204.0 / 243.0                                # the CAD rotor at 204 g
VALVE_DEF = dict(k_c=3000.0, c_c=25.0, k_g=0.5, c_g=0.02, f_c=0.072, w=0.01, I=I_ROT)


def flight_ref():
    d = np.load(NPZ, allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
    t = d["wbref__recv"] - t0
    i = int(np.searchsorted(t, T_HOLD))
    r = {k: np.array([d[f"wbref__{k}.{a}"][i] for a in "xyz"]) for k in ("x_cd", "r_ed", "b1_d", "b1_de")}
    r["q_d"] = np.array(d["wbref__q_d"][i], float)
    return r


def hold_stream(t_total=40.0, t_run0=12.0, dt=0.01):
    r0 = flight_ref()
    t = np.arange(0.0, t_total, dt)
    S = {"t": t}
    z3 = np.zeros((len(t), 3))
    for k in ("x_cd", "r_ed", "b1_d", "b1_de"):
        S[k] = np.tile(r0[k], (len(t), 1))
    for k in ("x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4", "b1_d_dot", "b1_d_ddot",
              "r_ed_dot", "r_ed_ddot", "b1_de_dot", "b1_de_ddot"):
        S[k] = z3.copy()
    S["q_d"] = np.tile(r0["q_d"], (len(t), 1)); S["qdot_d"] = np.zeros((len(t), N))
    S["run"] = (t_run0, t_total - 0.5)
    S["name"] = "valve_clamped_hold"
    return S


def simulate(p, S, valve=None, fb=None, seed=0, t_settle=12.0, delay_ms=16.0, ee_ref="world", free_w=0.75,
             contact=True, t_hand_lead=1.5, blend_s=1.0, fy_noise=None, arm_delay_ms=0.0, stiction=1.0,
             v_s=0.02, att_acc=0.0, att_tau=0.3, wrap_l1=None, wrap_law=None, clamp=True,
             neg_damp=(0.0, 0.0, 0.0, 0.0), relay=(0.0, 0.0, 0.0, 0.0), grip_rot=None):
    WH.EE_REF[0] = "relative"; WH.EE_FREE_W[0] = 0.0
    fb = dict(HW if fb is None else fb)
    vv = dict(VALVE_DEF) | (valve or {})
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
    kf_alloc, km_ratio, fric_plant = CS.KF_ALLOC, CS.KM_ALLOC_RATIO, CS.FRIC_SCALE
    fbias_m = CB.R_MODEL.T @ CS.FORCE_BIAS_BODY; tbias_m = CB.R_MODEL.T @ CS.TORQUE_BIAS_BODY
    kf_plant, km_plant = CS.KF_PLANT, CS.KM_PLANT
    ffs = CS.FF_SCALE
    cfg = CB.make_cfg(p)
    law = ES.Law(model, cfg, case)
    if wrap_law is not None:
        wrap_law(law)
    dt = case.dt
    FY = {"last": np.zeros(4)}
    if law.l1 is not None:
        if wrap_l1 is not None:
            wrap_l1(law.l1)
        _upd = law.l1.update
        fyn = CB._AR1(np.random.default_rng(seed + 1000), 1.0, fy_noise[3], dt, 3) if fy_noise else None

        def _update(*a, **k):
            est = _upd(*a, **k)
            if fyn is not None and law.l1.g.contact:
                z = fyn() * np.array(fy_noise[:3], float)
                est = dict(est); est["F_y"] = np.asarray(est["F_y"], float) + np.array([z[0], z[1], z[2], 0.0])
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

    t_run0, t_run1 = S["run"]
    r0 = CS.ref_at(S, 0, 0)
    X = np.zeros(12 + 3 * N + 6)
    X[12:12 + N] = r0["q_d"]
    psi = np.arctan2(r0["b1_d"][1], r0["b1_d"][0])
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1.0]])
    X[3:12] = Rz.flatten(order="F")
    X[0:3] = r0["x_cd"] - Rz @ C.dynamics(X, model)["r_0c_0"]
    gp = C.dynamics(X, plant)["g"]
    omega_rot = allocate(gp[2], gp[3:6])
    n_del = int(round(delay_ms * 1e-3 / dt)); fifo = [omega_rot.copy() for _ in range(n_del)]
    n_arm = int(round(arm_delay_ms * 1e-3 / dt)); tj_fifo = [np.zeros(N) for _ in range(n_arm)]
    n_odo = int(round(fb["odo_lag_s"] / dt)); n_att = int(round(fb["att_lag_s"] / dt))
    hist_X = [X.copy() for _ in range(max(n_odo, n_att) + 1)]
    vnoise = CB._AR1(rng, fb["vel_noise"], fb["vel_bw_hz"], dt, 3)
    fnoise = CB._AR1(rng, fb["force_noise"], fb.get("force_bw_hz", 0.3), dt, 2) if fb.get("force_noise", 0) > 0 else None
    a_qd = np.exp(-dt / fb["qd_lag_s"]) if fb["qd_lag_s"] > 0 else 0.0
    qd_obs = X[18 + N:18 + 2 * N].copy()
    a_att = np.exp(-dt / att_tau) if att_tau > 0 else 0.0
    acc_f = np.zeros(2); v_prev = np.zeros(3)
    t_hand, t_clamp = t_run0 - t_hand_lead, t_run0
    clamped = False; pin = None; th = thd = 0.0; psi_f0 = 0.0
    t_start = 0.0
    steps = int((t_run1 - t_start) / dt)
    keys = ("t", "ex_al", "ex_ac", "ez", "tilt", "eR_al", "eR_ac", "tau", "nsat", "F_rad", "F_tan", "F_z",
            "th", "fy_ac", "fy_al", "dt_ac", "dt_al", "M_t", "M_v")
    H = {k: np.zeros(steps) for k in keys}
    Hq = np.zeros((steps, N))
    verdict = "completed"; k_end = steps
    nose = nrm = None
    for k in range(steps):
        t = t_start + k * dt
        ref = dict(CS.ref_at(S, t, int(np.searchsorted(S["t"], t))))
        Xo = hist_X[-1 - n_odo]; Xa = hist_X[-1 - n_att]
        Ra = Xa[3:12].reshape(3, 3, order="F")
        if att_acc != 0.0:                                     # the PX4-like acceleration-correlated tilt error
            dv = att_acc * acc_f / 9.81
            tilt_vec = np.array([dv[1], -dv[0], 0.0])          # rotation axis: z x a  (tilt TOWARD a)
            nv = np.linalg.norm(tilt_vec)
            if nv > 1e-12:
                Ra = C.joint_rotation(tilt_vec / nv, nv) @ Ra
        if fb["att_noise"] > 0:
            dvn = rng.standard_normal(3) * fb["att_noise"]; nvn = np.linalg.norm(dvn)
            Ra = Ra @ C.joint_rotation(dvn / (nvn + 1e-12), nvn)
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
        if ee_ref == "world" and t >= t_hand:                  # the hand-over: CoM anchor -> room hold
            beta = min(1.0, (t - t_hand) / blend_s)
            law.case.ee_ref = "world"; law.case.ee_free_w = beta * float(free_w)
            Rm = Xm[3:12].reshape(3, 3, order="F")
            e_x = Xm[0:3] + Rm @ dyn["r_0c_0"] - ref["x_cd"]
            e_v = v_world - ref["x_cd_dot"]
            ref = dict(ref); ref["r_ed"] = ref["r_ed"] + (1 - beta) * e_x
            ref["r_ed_dot"] = ref["r_ed_dot"] + (1 - beta) * e_v
        R = X[3:12].reshape(3, 3, order="F")
        dynp = C.dynamics(X, plant)
        if clamp and not clamped and t >= t_clamp:             # the jaws close: grip on, contact flag on
            clamped = True
            if law.l1 is not None:
                object.__setattr__(law.l1.g, "contact", bool(contact))
            pin = X[0:3] + R @ dynp["r_0e_0"]
            b1e = (R @ dynp["R_e_0"])[:, 0]; psi_f0 = np.arctan2(b1e[1], b1e[0])
            Rc0 = R @ dynp["R_e_0"]                                  # the claw's orientation at the clamp
        if nose is None:
            b1 = R[:, 0]; ang = np.arctan2(b1[1], b1[0]) + np.pi / 2   # the model x is the body's right; nose = model y
            nose = np.array([np.cos(ang), np.sin(ang), 0.0]); nrm = np.array([-nose[1], nose[0], 0.0])
        out = law(Xm, dyn, ref, dt)
        u1 = float(out["u1"]); tau_body = np.asarray(out["tau_body"], float)
        tau_joint = np.clip(np.asarray(out["tau_joint"], float), -cfg.tau_max, cfg.tau_max)
        q = X[12:12 + N]; qd = X[18 + N:18 + 2 * N]
        load = np.abs(tau_joint)
        tau_joint = tau_joint + ffs * (CS.FRIC_FC + CS.FRIC_MU * load) * np.tanh(Xm[18 + N:18 + 2 * N] / CS.FF_W_MEAS)
        w_cmd = allocate(u1, tau_body)
        if n_del > 0:
            fifo.append(w_cmd); w_cmd = fifo.pop(0)
        if n_arm > 0:
            tj_fifo.append(tau_joint.copy()); tau_joint = tj_fifo.pop(0)
        omega_rot = w_cmd + (omega_rot - w_cmd) * np.exp(-ES.LAMBDA_ROTOR * dt)
        fr_ = kf_plant * omega_rot ** 2
        thrust_a = fr_.sum()
        tau_a = np.array([np.dot(rp[:, 1], fr_), -np.dot(rp[:, 0], fr_), np.dot(rd, km_plant * omega_rot ** 2)])
        fc = fric_plant * (CS.FRIC_FC + CS.FRIC_MU * load)
        if stiction != 1.0:                                    # Stribeck: f_s = stiction * f_c below v_s
            fc = fc * (1.0 + (stiction - 1.0) * np.exp(-(qd / v_s) ** 2))
        fr = fc * np.tanh(qd / CS.FRIC_W)
        fr = np.clip(fr, -0.02 * np.abs(qd) / dt, 0.02 * np.abs(qd) / dt)
        over = q - QMAX; under = QMIN - q
        tau_stop = (-np.where(over > 0, ES.K_STOP * over + ES.C_STOP * np.maximum(qd, 0), 0.0)
                    + np.where(under > 0, ES.K_STOP * under + ES.C_STOP * np.maximum(-qd, 0), 0.0))
        fb_now = fbias_m
        if fnoise is not None:
            f2 = fnoise(); fb_now = fb_now + R.T @ np.array([f2[0], f2[1], 0.0])
        # an arm joint that INJECTS energy (the hardware's back-EMF / friction feed-forward over-cancelling):
        # +b q_dot (net negative damping) and +r sign(q_dot) (a relay pushing WITH the motion)
        inj = np.asarray(neg_damp, float) * qd + np.asarray(relay, float) * np.tanh(qd / 0.01)
        Q = np.concatenate([fb_now + [0.0, 0.0, thrust_a], tau_a + tbias_m, tau_joint - fr + tau_stop + inj])
        xi = np.concatenate([X[12 + N:15 + N], X[15 + N:18 + N], X[18 + N:18 + 2 * N]])
        JyT = dynp["J_y"] @ dynp["T"]
        claw = X[0:3] + R @ dynp["r_0e_0"]
        cv = (JyT @ xi)[0:3]
        Fw = np.zeros(3); Mz = 0.0
        if clamped:
            ax3 = np.array([AXIS[0], AXIS[1], pin[2]])
            c_, s_ = np.cos(th), np.sin(th)
            Rth = np.array([[c_, -s_, 0], [s_, c_, 0], [0, 0, 1.0]])
            rf = Rth @ (pin - ax3)
            pf = ax3 + rf
            vf = np.cross(np.array([0, 0, thd]), rf)
            Fw = -vv["k_c"] * (claw - pf) - vv["c_c"] * (cv - vf)              # the grip's force ON the claw
            b1e = (R @ dynp["R_e_0"])[:, 0]; psi_c = np.arctan2(b1e[1], b1e[0])
            wz_c = (JyT @ xi)[3]
            dpsi = np.arctan2(np.sin(psi_c - psi_f0 - th), np.cos(psi_c - psi_f0 - th))
            Mz = -vv["k_g"] * dpsi - vv["c_g"] * (wz_c - thd)                 # the grip's twist ON the claw
            rhat = rf / np.linalg.norm(rf); that = np.cross(np.array([0, 0, 1.0]), rhat)
            if grip_rot is None:
                Q = Q + JyT.T @ np.array([Fw[0], Fw[1], Fw[2], Mz])
                Mw_z = Mz
            else:
                # a ROTATIONALLY STIFF grasp (pads clamping a plate): a full 3-D orientation spring between the
                # claw and the fin, in the fin's axes -- radial (the jaws' closing axis = the fin normal: a pivot,
                # friction only), tangential and vertical (in the fin's plane: the pads' faces resist)
                Rc = R @ dynp["R_e_0"]; Rt = Rth @ Rc0
                Ew = Rc @ Rt.T; phi = 0.5 * np.array([Ew[2, 1] - Ew[1, 2], Ew[0, 2] - Ew[2, 0], Ew[1, 0] - Ew[0, 1]])
                wc = R @ (xi[3:6] + dynp["J_q_omega_e"] @ xi[6:6 + N]) - np.array([0, 0, thd])
                Mw = np.zeros(3)
                for a_, kk in ((rhat, grip_rot["k_n"]), (that, grip_rot["k_t"]), (np.array([0, 0, 1.0]), grip_rot["k_v"])):
                    Mw = Mw - (kk * (phi @ a_) + grip_rot.get("c", 0.02) * (wc @ a_)) * a_
                Q = Q + JyT.T @ np.array([Fw[0], Fw[1], Fw[2], 0.0])
                Mb = R.T @ Mw
                Q[3:6] = Q[3:6] + Mb; Q[6:6 + N] = Q[6:6 + N] + dynp["J_q_omega_e"].T @ Mb
                Mw_z = Mw[2]; H["M_t"][k] = Mw @ that; H["M_v"][k] = Mw[2]
            # the rotor: the reaction of both, the bearing's Coulomb friction (momentum-clamped)
            tq = np.cross(rf, -Fw)[2] - Mw_z
            tf = -vv["f_c"] * np.tanh(thd / vv["w"])
            tf = np.clip(tf, -vv["I"] * abs(thd) / dt - abs(tq), vv["I"] * abs(thd) / dt + abs(tq))
            thd = thd + (tq + tf) / vv["I"] * dt
            th = th + thd * dt
            H["F_rad"][k] = Fw @ rhat; H["F_tan"][k] = Fw @ that; H["F_z"][k] = Fw[2]
        xi = xi + np.linalg.solve(dynp["M"], Q - dynp["C"] @ xi - dynp["g"]) * dt
        v0, w0, qdn = xi[0:3], xi[3:6], xi[6:6 + N]
        # the base's horizontal acceleration (world), for the PX4-like tilt error
        vw = R @ v0
        acc = (vw - v_prev) / dt; v_prev = vw
        acc_f = a_att * acc_f + (1 - a_att) * acc[:2]
        x_c = X[0:3] + R @ dyn["r_0c_0"]
        e = x_c - ref["x_cd"]
        H["t"][k] = t; H["ex_al"][k] = e @ nose; H["ex_ac"][k] = e @ nrm; H["ez"][k] = e[2]
        H["tilt"][k] = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        eR = np.asarray(out["e_R"]); H["eR_al"][k] = eR[0]; H["eR_ac"][k] = eR[1]
        H["tau"][k] = np.abs(tau_joint).max(); H["nsat"][k] = out["n_sat"]; H["th"][k] = th
        fyw = FY["last"][:3]; H["fy_ac"][k] = fyw @ nrm; H["fy_al"][k] = fyw @ nose
        dth = getattr(law, "_dbg_dt", None)
        Hq[k] = q
        X[0:3] = X[0:3] + R @ v0 * dt
        nw = np.linalg.norm(w0)
        R = R @ C.joint_rotation(w0 / (nw + 1e-12), nw * dt)
        uu, _, vt = np.linalg.svd(R); R = uu @ vt
        X[3:12] = R.flatten(order="F")
        X[12:12 + N] = X[12:12 + N] + qdn * dt
        X[12 + N:18 + N] = np.concatenate([v0, w0]); X[18 + N:18 + 2 * N] = qdn
        hist_X.append(X.copy()); hist_X.pop(0)
        if not np.isfinite(H["ex_ac"][k]) or H["tilt"][k] > 25.0 or np.hypot(H["ex_ac"][k], H["ex_al"][k]) > 1.0:
            verdict = "ABORT"; k_end = k + 1; break
    H = {kk: v[:k_end] for kk, v in H.items()}
    H["q"] = Hq[:k_end]
    return verdict, H


def envelope_growth(t, x, thr=0.004):
    s = np.sign(x); pk = []; i = 0
    while i < len(x):
        j = i
        while j < len(x) and s[j] == s[i]:
            j += 1
        kk = i + int(np.argmax(np.abs(x[i:j]))); pk.append((t[kk], abs(x[kk]))); i = j
    a = np.array([p for p in pk if p[1] > thr])
    if len(a) < 4:
        return float("nan")
    return float(np.polyfit(a[:, 0], np.log(a[:, 1]), 1)[0])


def dominant(t, x):
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), t[1] - t[0]); m = (f > 0.08) & (f < 3.0)
    return float(f[m][np.argmax(P[m])])


def score(S, verdict, H, win_pre=(6.0, 10.5)):
    t = H["t"]; a = S["run"][0]
    out = dict(verdict=verdict, t_end=float(t[-1]))
    mc = t >= a + 1.0                                          # clamped (after the first second; the same window unclamped)
    mp = (t >= win_pre[0]) & (t <= win_pre[1])                 # free, before the hand-over
    if mc.sum() < 200:
        return out
    tc = t[mc]
    for nm, key in (("ac", "ex_ac"), ("al", "ex_al")):
        x = H[key][mc]
        out[f"{nm}_rms"] = float(1e3 * np.std(x)); out[f"{nm}_f"] = dominant(tc, x)
        out[f"{nm}_gr"] = envelope_growth(tc, x - np.mean(x))
        third = len(x) // 3
        out[f"{nm}_first"] = float(1e3 * np.std(x[:third])); out[f"{nm}_last"] = float(1e3 * np.std(x[-third:]))
        out[f"{nm}_pre"] = float(1e3 * np.std(H[key][mp]))
    q = np.degrees(H["q"][mc])
    out.update(q1=(float(q[:, 0].min()), float(q[:, 0].max())), q2=(float(q[:, 1].min()), float(q[:, 1].max())),
               q3=(float(q[:, 2].min()), float(q[:, 2].max())), q4=(float(q[:, 3].min()), float(q[:, 3].max())),
               tilt_pk=float(H["tilt"][mc].max()), F_rad=float(np.std(H["F_rad"][mc])),
               F_tan=float(np.mean(np.abs(H["F_tan"][mc]))), th=float(np.degrees(H["th"][mc][-1])),
               fy_ac=float(np.std(H["fy_ac"][mc])))
    return out


def fmt(name, o):
    if "ac_rms" not in o:
        return f"{name:52s} {o['verdict']} at {o['t_end']:.1f} s"
    return (f"{name:52s} {o['verdict'][:5]} | ACROSS rms {o['ac_rms']:5.1f} (pre {o['ac_pre']:4.1f}, first/last third "
            f"{o['ac_first']:4.1f}/{o['ac_last']:5.1f}) mm @ {o['ac_f']:.2f} Hz gr {o['ac_gr']:+.2f} | ALONG {o['al_rms']:5.1f} mm "
            f"@ {o['al_f']:.2f} | q1 {o['q1'][0]:+5.1f}..{o['q1'][1]:+5.1f} q3 {o['q3'][0]:4.1f}..{o['q3'][1]:4.1f} | tilt {o['tilt_pk']:4.1f} "
            f"F_rad sd {o['F_rad']:.2f} N F_y ac sd {o['fy_ac']:.2f} | valve {o['th']:+5.1f} deg")


def job(args):
    name, p, kw, seed = args
    S = hold_stream()
    v, H = simulate(p, S, seed=seed, **kw)
    return name, seed, score(S, v, H)


CASES = {
    "N0f": ("FREE hold (no clamp), j1/j4 b +0.3", VALVE, dict(clamp=False, neg_damp=(0.3, 0, 0, 0.3), fb=WH.FB_RAW)),
    "N1":  ("clamped, j1/j4 b +0.1", VALVE, dict(neg_damp=(0.1, 0, 0, 0.1), fb=WH.FB_RAW)),
    "N2":  ("clamped, j1/j4 b +0.2", VALVE, dict(neg_damp=(0.2, 0, 0, 0.2), fb=WH.FB_RAW)),
    "N3":  ("clamped, j1/j4 b +0.3", VALVE, dict(neg_damp=(0.3, 0, 0, 0.3), fb=WH.FB_RAW)),
    "N4":  ("clamped, j1 only b +0.3", VALVE, dict(neg_damp=(0.3, 0, 0, 0.0), fb=WH.FB_RAW)),
    "N5":  ("clamped, j4 only b +0.3", VALVE, dict(neg_damp=(0.0, 0, 0, 0.3), fb=WH.FB_RAW)),
    "N3c": ("clamped, j1/j4 b +0.3, contact flag OFF", VALVE, dict(neg_damp=(0.3, 0, 0, 0.3), fb=WH.FB_RAW, contact=False)),
    "R1":  ("clamped, j1/j4 relay +0.02 N.m", VALVE, dict(relay=(0.02, 0, 0, 0.02), fb=WH.FB_RAW)),
    "R2":  ("clamped, j1/j4 relay +0.05 N.m", VALVE, dict(relay=(0.05, 0, 0, 0.05), fb=WH.FB_RAW)),
    "V0": ("valve clamped hold, as flown (raw mocap + drift)", VALVE, dict()),
    "V0n": ("  ... no drift force", VALVE, dict(fb=WH.FB_RAW)),
    "V0c": ("  ... contact flag OFF", VALVE, dict(contact=False)),
    "H1a": ("H arm-torque delay 20 ms", VALVE, dict(arm_delay_ms=20.0)),
    "H1b": ("H arm-torque delay 40 ms", VALVE, dict(arm_delay_ms=40.0)),
    "H2a": ("H mocap transport 40 ms", VALVE, dict(fb=dict(HW, odo_lag_s=0.040))),
    "H3a": ("H arm stiction x2", VALVE, dict(stiction=2.0)),
    "H4a": ("H PX4 tilt over-reads accel x0.25 (tau 0.3)", VALVE, dict(att_acc=0.25)),
    "H4b": ("H PX4 tilt UNDER-reads accel x0.25 (tau 0.3)", VALVE, dict(att_acc=-0.25)),
    "H5a": ("H grip soft k_c 600 N/m", VALVE, dict(valve=dict(k_c=600.0, c_c=8.0))),
    "H6a": ("H F_y noise 1.0/0.3/0.3 N @0.3 Hz", VALVE, dict(fy_noise=(0.3, 1.0, 0.3, 0.3))),
}


if __name__ == "__main__":
    sel = os.environ.get("CASES", ",".join(CASES)).split(",")
    seeds = [int(s) for s in os.environ.get("SEEDS", "1,2").split(",")]
    jobs = [(c, CASES[c][1], CASES[c][2], s) for c in sel for s in seeds]
    with ProcessPoolExecutor(max_workers=int(os.environ.get("JOBS", "12"))) as ex:
        for c, seed, o in ex.map(job, jobs):
            print(fmt(f"{c} {CASES[c][0]} [s{seed}]", o), flush=True)
