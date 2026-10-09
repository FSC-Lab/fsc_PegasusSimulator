#!/usr/bin/env python3
"""Can the armature model + task gains make the whole-body arm channel stiff enough? (2026-09-26)

Offline, deterministic, seconds per case. The whole-body law is wb_entry_sim.Law (a verbatim copy of
controller.py's law with the flown hooks), run with the 4-D HARDWARE config: 4-D L1 observer
(omega_x 0.25, omega_q 0.2), wb_ee_anchor_com, wb_u3_internal_ff (4-D source), K_y 20 / D_y 12,
M_y 1/1/1/0.05, k_x 32 / k_v 20, k_R 2 / k_w 1.5, DLS 0.3, rotor lag 10.03 1/s, 16 ms transport.

What makes it a test of the ARM question rather than of hover:
  * plant armature = JOINT-DIAGONAL (the physics; also what Isaac authors), value J_true;
  * plant gearbox friction with STICK-SLIP: static = the arm yaml's fc + mu|tau_g| (what the
    friction feed-forward assumes), kinetic = 0.85x (j2) / 0.65x (j3) / 0.8x (j1, j4) of it --
    the ratios the 0924 flights identified (tools/friction_id.py);
  * the arm controller's friction feed-forward as flown: (fc + mu|tau_cmd|) tanh(qdot_ref / 0.015);
  * the servo caps (max_effort) [0.34, 2.44, 1.42, 0.39] N.m;
  * the law's qdot is the servo's Present Velocity: 48 ms late and quantised at 0.024 rad/s;
  * the reference is flight 3's arm design: fold 55 deg, q2 = 25 + 15 sin(2 pi t / 12 s),
    q3 = 55 - q2, base hovering with the CoM held (a compatible internal motion).

Law-model variants: "hht" = the flown J h h^T on each child link; "diag" = joint-diagonal armature
(controller.dynamics' armature_diag hook); M_r_d is rescaled with M_r so the ATTITUDE loop's gain
M_r M_r_d^-1 k_R is unchanged -- only the arm channel differs between variants.

    /usr/bin/python3 arm_stiffness_sim.py terms          # per-term effect of the armature model
    /usr/bin/python3 arm_stiffness_sim.py run [names]    # closed-loop cases (default: all)
"""
import copy
import os
import sys
import time

import numpy as np

REPO = "/home/shiqi/fsc_PegasusSimulator"
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
sys.path.insert(0, os.path.join(REPO, "application", "robotic_arm", "utils"))
import wb_entry_sim as E  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

N = 4
DEG = np.pi / 180.0
JA0 = 353.5 ** 2 * 1.6e-7                                  # the modelled 0.0200 kg m^2
CAP = np.array([0.34, 2.44, 1.42, 0.39])                   # servo max_effort [N.m]
FC = np.array([0.01711, 0.03143, 0.05751, 0.05237])        # arm yaml friction (N.m)
MU = np.array([0.0, 0.246, 0.161, 0.0])
W_FF = 0.015                                               # FF tanh width [rad/s]
# THE FRICTION FEED-FORWARD's VELOCITY SOURCE (2026-10-08): "reference" = as flown
# 0918-0924 (tanh(qdot_d / W_FF)); "measured" = the arm controller since 2026-09-28
# (friction_velocity_source: measured, tanh(qdot_meas / FF_W_MEAS), the law's own
# velocity estimate as qdot_meas); "blend" = measured + FF_BLEND x reference, the
# sum clipped to +-1. FF_SCALE = the per-joint scaling (hardware 2026-09-28:
# j2 x0.70, j3 x0.65 on fc AND mu).
FF_SOURCE = "reference"
FF_W_MEAS = 0.03
FF_BLEND = 0.3
FF_SCALE = np.ones(4)
KIN = np.array([0.8, 0.85, 0.65, 0.8])                     # kinetic / static (flight ID)
VQ, V_DELAY = 0.023969, 0.048                              # Present Velocity quantum / lag
V_NOISE = 0.0                                              # white velocity noise [rad/s] (observer study)
# The law's qdot source (2026-09-26): "present" = Present Velocity (V_DELAY late,
# VQ quantum, as flown); "observer" = the arm controller's velocity_observer --
# the SAME discrete 2nd-order observer (20 Hz, zeta 1, gains at the nominal
# 250 Hz period) run on encoder positions quantised at 4096 counts/rev.
VEL_SOURCE = "present"
# Arm-channel TRANSPORT (2026-09-26, default 0 = as before): the joint state the
# law reads (q and its qdot) arrives LAW_ARM_DELAY_S late (DDS, joint_states /
# velocity_observer -> flight node) and the joint torque reaches the servo
# ARM_CMD_DELAY_S late (flight node -> ExternalTorqueController -> bus).
LAW_ARM_DELAY_S, ARM_CMD_DELAY_S = 0.0, 0.0
OBS_BW_HZ, OBS_ZETA, ENC_Q = 20.0, 1.0, 2 * np.pi / 4096
FOLD, Q2C, Q2A, PER, T0, TRAMP = 55.0, 25.0, 15.0, 12.0, 3.0, 3.0
T_END, DT = 30.0, 1.0 / 250.0
WIN = (T0 + TRAMP, T_END)
HW_GAINS = dict(k_x=32.0, k_v=20.0, k_R=2.0, k_w=1.5, mrd=(0.116522, 0.136107, 0.125102),
                ky=(20.0, 20.0, 20.0, 0.3), dy=(12.0, 12.0, 12.0, 0.3), my=(1.0, 1.0, 1.0, 0.05),
                use_gmo=True, dls_lambda=0.3, tau_max=3.0)
HW_L1 = dict(four_d=True, omega_x=0.25, omega_q=0.2)


def links_only():
    P = TP.make_params_t650(); L = copy.deepcopy(P)
    for i in range(N):
        h = np.asarray(P["h_i_im1"][i], float)
        L["I_i_i"][i + 1] = np.asarray(P["I_i_i"][i + 1]) - JA0 * np.outer(h, h)
    return L


def model_params(kind, J=JA0):
    L = links_only()
    if kind == "hht":
        for i in range(N):
            h = np.asarray(L["h_i_im1"][i], float)
            L["I_i_i"][i + 1] = L["I_i_i"][i + 1] + J * np.outer(h, h)
    elif kind == "diag":
        L["armature_diag"] = [J] * N
    return L


def q_ref(t):
    """q2 = 25 + a(t) sin(2 pi (t - T0)/PER), a ramps in over TRAMP (C2-smooth), q3 = FOLD - q2."""
    s = np.clip((t - T0) / TRAMP, 0.0, 1.0)
    a = Q2A * (10 * s**3 - 15 * s**4 + 6 * s**5)
    q2 = Q2C + a * np.sin(2 * np.pi * max(t - T0, 0.0) / PER)
    return np.array([0.0, q2, FOLD - q2, 0.0]) * DEG


def build_reference(model, x_cd, R0):
    tg = np.arange(0.0, T_END + 0.1, DT)
    Q = np.array([q_ref(t) for t in tg])
    Qd = np.gradient(Q, DT, axis=0); Qdd = np.gradient(Qd, DT, axis=0)
    X = np.zeros(18 + 2 * N); X[3:12] = R0.reshape(9, order="F")
    off = np.zeros((len(tg), 3))
    for i, q in enumerate(Q):
        X[12:12 + N] = q
        d = C.dynamics(X, model)
        off[i] = R0 @ (d["r_0e_0"] - d["r_0c_0"])
    b1e = d["R_e_0"][:, 0]                          # q1 = q4 = 0, fold constant -> fixed EE heading
    offd = np.gradient(off, DT, axis=0); offdd = np.gradient(offd, DT, axis=0)
    z3 = np.zeros(3)

    def ref(t):
        k = min(int(round(t / DT)), len(tg) - 1)
        return {"x_cd": x_cd, "x_cd_dot": z3, "x_cd_ddot": z3, "x_cd_d3": z3, "x_cd_d4": z3,
                "b1_d": R0[:, 0].copy(), "b1_d_dot": z3, "b1_d_ddot": z3,
                "r_ed": x_cd + off[k], "r_ed_dot": offd[k], "r_ed_ddot": offdd[k],
                "b1_de": R0 @ b1e, "b1_de_dot": z3, "b1_de_ddot": z3,
                "q_d": Q[k].copy(), "qdot_d": Qd[k].copy()}
    return ref


def mrd_for(model, base_mrd):
    """Rescale M_r_d with M_r so M_r M_r_d^-1 (the attitude loop's gain) matches the flown model's."""
    X = np.zeros(18 + 2 * N); X[3:12] = np.eye(3).reshape(9, order="F"); X[12:12 + N] = q_ref(0.0)
    mr_new = np.diag(C.dynamics(X, model)["M_r"]); mr_old = np.diag(C.dynamics(X, model_params("hht"))["M_r"])
    return tuple(np.asarray(base_mrd) * mr_new / mr_old)


def friction(M, rhs, qd, g_joint):
    """Coupled stick-slip (Karnopp): returns joint friction torques and the stuck mask."""
    Fs = FC + MU * np.abs(g_joint)
    Fk = KIN * Fs
    Minv = np.linalg.inv(M)
    fr = np.zeros(N)
    stuck = np.abs(qd) < 1e-4
    fr[~stuck] = -Fk[~stuck] * np.sign(qd[~stuck])
    for _ in range(4):
        S = np.where(stuck)[0]
        full = np.concatenate([np.zeros(6), fr])
        a0 = Minv @ (rhs + full)
        if S.size == 0:
            break
        idx = 6 + S
        fS = np.linalg.solve(Minv[np.ix_(idx, idx)], -a0[idx]) + fr[S]
        ok = np.abs(fS) <= Fs[S]
        if ok.all():
            fr[S] = fS
            break
        for j, f, o in zip(S, fS, ok):
            if not o:
                stuck[j] = False
                fr[j] = -Fk[j] * np.sign(-f)       # slips the way the net drive pushes
            else:
                fr[j] = 0.0
    return fr, stuck, Minv


def simulate(name, model_kind="hht", J_model=JA0, J_true=JA0, gains=None, posture=0.0,
             posture_kp=2.0, posture_kd=0.25, friction_on=True, verbose=False):
    gains = dict(HW_GAINS, **(gains or {}))
    model = model_params(model_kind, J_model)
    if model_kind != "hht":
        gains["mrd"] = mrd_for(model, gains["mrd"])
    plant = model_params("diag", J_true)
    case = E.Case(posture=posture, posture_kp=posture_kp, posture_kd=posture_kd, posture_ki=0.0,
                  posture_imax=0.0, ee_ref="relative", int_ff=True, int_ff_source="four_d",
                  mass=1.0, inertia=1.0, com=(0.0, 0.0, 0.0), kf=1.0, kf_alloc=1.0,
                  delay_ms=16.0, observer="l1", l1=dict(HW_L1))
    cfg = E.make_gains(**gains)
    law = E.Law(model, cfg, case)

    X = np.zeros(12 + 3 * N + 6)
    X[0:3] = [0.0, 0.0, 1.0]; X[3:12] = np.eye(3).reshape(9, order="F"); X[12:12 + N] = q_ref(0.0)
    d0 = C.dynamics(X, model)
    ref = build_reference(model, X[0:3] + d0["r_0c_0"], np.eye(3))
    dynp = C.dynamics(X, plant); gp = dynp["g"]
    kf = E.KF_TRUE
    omega_rot = E.allocate(gp[2], gp[3:6], kf)
    n_del = int(round(16e-3 / DT)); fifo = [omega_rot.copy() for _ in range(n_del)]
    v_del = int(round(V_DELAY / DT)); vbuf = [np.zeros(N) for _ in range(v_del)]
    steps = int(T_END / DT)
    rng = np.random.default_rng(1)
    obs_w = 2 * np.pi * OBS_BW_HZ
    obs_l1, obs_l2 = 2 * OBS_ZETA * obs_w * DT, obs_w * obs_w * DT
    q_hat = X[12:12 + N].copy(); v_hat = np.zeros(N)
    n_la = int(round(LAW_ARM_DELAY_S / DT)); n_ac = int(round(ARM_CMD_DELAY_S / DT))
    labuf = [(X[12:12 + N].copy(), np.zeros(N)) for _ in range(n_la)]
    acbuf = [np.zeros(N) for _ in range(n_ac)]
    H = {k: [] for k in ("t", "q", "qd_true", "qref", "tau", "tilt", "ex", "ey", "stuck", "cap")}
    verdict, tfail = "completed", None
    for k in range(steps):
        t = k * DT
        R = ref(t)
        qd_true = X[18 + N:18 + 2 * N].copy()
        vbuf.append(qd_true); qd_pv = np.round(vbuf.pop(0) / VQ) * VQ + V_NOISE * rng.standard_normal(N)
        if VEL_SOURCE == "observer":
            q_enc = np.round(X[12:12 + N] / ENC_Q) * ENC_Q
            q_pred = q_hat + DT * v_hat; e_o = q_enc - q_pred
            q_hat = q_pred + obs_l1 * e_o; v_hat = v_hat + obs_l2 * e_o
            qd_meas = v_hat.copy()
        else:
            qd_meas = qd_pv
        XL = X.copy(); XL[18 + N:18 + 2 * N] = qd_meas
        if n_la:
            labuf.append((X[12:12 + N].copy(), qd_meas.copy())); q_l, qd_l = labuf.pop(0)
            XL[12:12 + N] = q_l; XL[18 + N:18 + 2 * N] = qd_l
        out = law(XL, C.dynamics(XL, model), R, DT)
        u1 = float(out["u1"]); tau_body = np.asarray(out["tau_body"], float)
        tau_law = np.clip(np.asarray(out["tau_joint"], float), -cfg.tau_max, cfg.tau_max)
        w_cmd = E.allocate(u1, tau_body, kf); fifo.append(w_cmd); w_cmd = fifo.pop(0)
        omega_rot = w_cmd + (omega_rot - w_cmd) * np.exp(-E.LAMBDA_ROTOR * DT)
        thrust_a, tau_a = E.wrench_from_rotors(omega_rot, kf)
        if FF_SOURCE == "reference":
            gate = np.tanh(R["qdot_d"] / W_FF)
        elif FF_SOURCE == "measured":
            gate = np.tanh(qd_meas / FF_W_MEAS)
        else:
            gate = np.clip(np.tanh(qd_meas / FF_W_MEAS) + FF_BLEND * np.tanh(R["qdot_d"] / W_FF), -1.0, 1.0)
        ff = FF_SCALE * (FC + MU * np.abs(tau_law)) * gate if friction_on else np.zeros(N)
        tau_cmd = np.clip(tau_law + ff, -CAP, CAP)
        if n_ac:
            acbuf.append(tau_cmd.copy()); tau_cmd = acbuf.pop(0)
        q = X[12:12 + N]
        over = q - E.Q_MAX; under = E.Q_MIN - q
        tau_stop = (-np.where(over > 0, E.K_STOP * over + E.C_STOP * np.maximum(qd_true, 0), 0.0)
                    + np.where(under > 0, E.K_STOP * under + E.C_STOP * np.maximum(-qd_true, 0), 0.0))
        dynp = C.dynamics(X, plant)
        Q = np.concatenate([[0.0, 0.0, thrust_a], tau_a, tau_cmd + tau_stop])
        xi = np.concatenate([X[12 + N:15 + N], X[15 + N:18 + N], qd_true])
        rhs = Q - dynp["C"] @ xi - dynp["g"]
        if friction_on:
            fr, stuck, Minv = friction(dynp["M"], rhs, qd_true, dynp["g"][6:])
        else:
            fr, stuck, Minv = np.zeros(N), np.zeros(N, bool), np.linalg.inv(dynp["M"])
        xi = xi + (Minv @ (rhs + np.concatenate([np.zeros(6), fr]))) * DT
        qn = xi[6:]
        if friction_on:                              # stuck, or crossed zero this step -> at rest
            qn[stuck] = 0.0
            qn[(np.sign(qn) != np.sign(qd_true)) & (qd_true != 0)] = 0.0
        v0, w0 = xi[0:3], xi[3:6]
        Rb = X[3:12].reshape(3, 3, order="F")
        X[0:3] = X[0:3] + Rb @ v0 * DT
        nw = np.linalg.norm(w0)
        Rb = Rb @ C.joint_rotation(w0 / (nw + 1e-12), nw * DT); uu, _, vt = np.linalg.svd(Rb); Rb = uu @ vt
        X[3:12] = Rb.reshape(9, order="F")
        X[12:12 + N] = X[12:12 + N] + qn * DT
        X[12 + N:18 + N] = np.concatenate([v0, w0]); X[18 + N:18 + 2 * N] = qn
        tilt = np.degrees(np.arccos(np.clip(Rb[2, 2], -1, 1)))
        ex = float(np.linalg.norm(out["e_x"]))
        H["t"].append(t); H["q"].append(X[12:12 + N].copy()); H["qd_true"].append(qn.copy())
        H["qref"].append(R["q_d"]); H["tau"].append(tau_cmd.copy()); H["tilt"].append(tilt)
        H["ex"].append(ex); H["ey"].append(float(np.linalg.norm(out["e_y"][:3])))
        H["stuck"].append(stuck.copy()); H["cap"].append(np.abs(tau_cmd) >= CAP - 1e-9)
        if verbose and k % 500 == 0:
            print(f"   t={t:5.1f} q={np.degrees(X[12:16]).round(1)} ref={np.degrees(R['q_d']).round(1)} "
                  f"tau={tau_cmd.round(2)} tilt={tilt:.1f} ex={ex*1e3:.0f} mm", flush=True)
        if not np.isfinite(ex) or tilt > 35 or ex > 1.5:
            verdict, tfail = "ABORT", t
            break
    for kk in H:
        H[kk] = np.asarray(H[kk])
    return score(name, H, verdict, tfail), H


def score(name, H, verdict, tfail):
    t = H["t"]; m = (t >= WIN[0]) & (t <= WIN[1])
    if verdict != "completed" or m.sum() < 100:
        return dict(name=name, verdict=f"ABORT {tfail:.1f}s")
    q = np.degrees(H["q"][m]); r = np.degrees(H["qref"][m]); v = np.degrees(H["qd_true"][m])
    rv = np.gradient(r, DT, axis=0)
    o = dict(name=name, verdict="ok")
    for j, lab in ((1, "j2"), (2, "j3")):
        e = q[:, j] - r[:, j]
        mv = np.abs(rv[:, j]) > 0.25 * np.abs(rv[:, j]).max()
        o[lab] = dict(rms=np.sqrt((e ** 2).mean()), emax=np.abs(e).max(),
                      span=100 * (q[:, j].max() - q[:, j].min()) / (r[:, j].max() - r[:, j].min()),
                      stuck=100 * np.mean(np.abs(v[mv, j]) < 1.0), vp=np.percentile(np.abs(v[:, j]), 99),
                      over=max(q[:, j].max() - r[:, j].max(), r[:, j].min() - q[:, j].min()))
    o["tilt"] = H["tilt"][m].max(); o["ex"] = 1e3 * H["ex"][m].max(); o["ey"] = 1e3 * np.sqrt((H["ey"][m] ** 2).mean())
    o["taupk"] = np.abs(H["tau"][m]).max(axis=0); o["cap"] = 100 * H["cap"][m].any(axis=1).mean()
    return o


def fmt(o):
    if o["verdict"] != "ok":
        return f"{o['name']:44s} {o['verdict']}"
    s = f"{o['name']:44s}"
    for lab in ("j2", "j3"):
        d = o[lab]
        s += (f" | {lab} rms {d['rms']:4.1f} max {d['emax']:4.1f} span {d['span']:4.0f}% "
              f"stuck {d['stuck']:3.0f}% v99 {d['vp']:4.1f} over {d['over']:+5.1f}")
    s += f" | tilt {o['tilt']:4.1f} CoM {o['ex']:5.0f} mm EE {o['ey']:4.1f} mm | tau j2/j3 {o['taupk'][1]:.2f}/{o['taupk'][2]:.2f} cap {o['cap']:.1f}%"
    return s


CASES = {
    "A_flown":            dict(model_kind="hht"),
    "B_diag":             dict(model_kind="diag"),
    "C_diag_K50":         dict(model_kind="diag", gains=dict(ky=(50.0, 50.0, 50.0, 0.3), dy=(20.0, 20.0, 20.0, 0.3))),
    "D_diag_K100":        dict(model_kind="diag", gains=dict(ky=(100.0, 100.0, 100.0, 0.3), dy=(28.0, 28.0, 28.0, 0.3))),
    "E_diag_K200":        dict(model_kind="diag", gains=dict(ky=(200.0, 200.0, 200.0, 0.3), dy=(40.0, 40.0, 40.0, 0.3))),
    "F_flown_K50":        dict(model_kind="hht", gains=dict(ky=(50.0, 50.0, 50.0, 0.3), dy=(20.0, 20.0, 20.0, 0.3))),
    "G_flown+jointPD2":   dict(model_kind="hht", posture=1.0, posture_kp=2.0, posture_kd=0.25),
    "H_flown+jointPD10":  dict(model_kind="hht", posture=1.0, posture_kp=10.0, posture_kd=0.5),
    "Z_flown_nofriction": dict(model_kind="hht", friction_on=False),
}


def terms():
    np.set_printoptions(precision=4, suppress=True, linewidth=200)
    X = np.zeros(18 + 2 * N); X[3:12] = np.eye(3).reshape(9, order="F")
    rows = [("flown: J h h^T, 0.020", model_params("hht")), ("joint-diagonal, 0.020", model_params("diag")),
            ("joint-diagonal, 0.012", model_params("diag", 0.012)), ("no armature", model_params("none"))]
    for qdeg in ([0, 25, 30, 0], [0, 40, 40, 0]):
        X[12:16] = np.radians(qdeg)
        print(f"\n==== q = {qdeg} deg")
        for lab, P in rows:
            d = C.dynamics(X, P); Mr = d["M_tilde"][6:, 6:]; w = np.linalg.eigvalsh(Mr[1:3, 1:3])
            J3 = d["J_3y"]; Lam = d["Lambda_y"]
            G = J3.T @ np.linalg.solve(J3 @ J3.T + 0.09 * np.eye(4), Lam @ np.linalg.inv(np.diag([1, 1, 1, 0.05])))
            Kq = G @ np.diag([20, 20, 20, 0.3]) @ d["J_y"][:, 6:]; kw = np.linalg.eigvalsh(0.5 * (Kq[1:3, 1:3] + Kq[1:3, 1:3].T))
            print(f"  {lab:24s} M_rho diag {np.diag(Mr)} j2/j3 eig {w} | Kq(K_y 20) diag {np.diag(Kq)} j2/j3 eig {kw} | "
                  f"M_r diag {np.diag(d['M_r'])} | N1 rows(x) {d['N1'][:, 0]} | Lambda_y diag {np.diag(Lam)}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "terms":
        terms(); sys.exit(0)
    names = sys.argv[2:] if len(sys.argv) > 2 else list(CASES)
    for nm in names:
        t0 = time.time()
        o, H = simulate(nm, **CASES[nm])
        np.savez_compressed(os.path.join(os.path.dirname(os.path.abspath(__file__)), f"sim_{nm}.npz"), **H)
        print(fmt(o) + f"   [{time.time() - t0:.0f} s]", flush=True)
