#!/usr/bin/env python3
"""generate_coordinated_truth.py -- parity fixture for the C++ port of the
coordinated-admittance flight law (Liu et al., CEP 2024, eqs 11-14) and its
momentum wrench estimator (eq 25).

The C++ (fsc_autopilot_ros2 .../single_aerial_manipulator_coordinated_admittance_
direct_actuation/client_lib/src/{coordinated_flight_law,momentum_wrench_estimator}.cpp)
is held to the Python reference (extensions/.../utils_controller/
coordinated_admittance.py) by fsc_autopilot_lib/single_vehicle_baseline/tests/
src/test_coordinated_parity.cpp, which replays the ROLLOUTS written here.

Coverage: three gain sets (the shipped one; one with every clamp active --
position/velocity saturation, tilt clamp, thrust floor, torque clamps -- and
one with products of inertia and unequal K1/K2 per channel); states swept
through large attitudes (|roll|, |pitch| up to 40 deg, yaw through the wrap),
non-zero body rates, moving references; the estimator run on a closed-loop
rigid-body plant with an injected constant wrench, non-uniform dt, with its
clamp hit on one set.

    /usr/bin/python3 generate_coordinated_truth.py [out.json]
Default output: the fixture path the gtest reads. Never hand-edit it.
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_controller import coordinated_admittance as CA  # noqa: E402

DEFAULT_OUT = os.path.expanduser(
    "~/ros2_ws/src/fsc_autopilot_ros2/fsc_autopilot_ros2_node/"
    "single_aerial_manipulator_coordinated_admittance_direct_actuation/client_lib/tests/data/"
    "coordinated_truth_t650.json")


def gains_to_json(g: CA.FlightLawConfig):
    return dict(mass=g.mass, kpp=g.kpp.tolist(), kpd=g.kpd.tolist(), kpi=g.kpi.tolist(), int_max=g.int_max,
                keta_p=g.keta_p.tolist(),
                keta_d=g.keta_d.tolist(), keta_i=g.keta_i.tolist(), int_eta_max=g.int_eta_max, inertia=g.inertia.flatten(order="F").tolist(),
                max_pos_err_m=g.max_pos_err_m, max_vel_err_mps=g.max_vel_err_mps,
                max_tilt_deg=g.max_tilt_deg, min_thrust_frac=g.min_thrust_frac,
                max_torque_xy=g.max_torque_xy, max_torque_z=g.max_torque_z)


def law_cases(rng, cfg, n):
    cases = []
    for _ in range(n):
        eta = np.array([rng.uniform(-0.7, 0.7), rng.uniform(-0.7, 0.7), rng.uniform(-math.pi, math.pi)])
        R = CA.euler_to_r(eta)
        p = rng.normal(0.0, 1.0, 3)
        v = rng.normal(0.0, 0.8, 3)
        om = rng.normal(0.0, 1.0, 3)
        p_d = p + rng.normal(0.0, 0.8, 3)
        v_d = rng.normal(0.0, 0.5, 3)
        a_d = rng.normal(0.0, 1.0, 3)
        yaw_d = rng.uniform(-math.pi, math.pi)
        e_int = rng.normal(0.0, 0.3, 3)          # beyond int_max on some axes: exercises the clamp
        e_eta_int = rng.normal(0.0, 0.2, 3)      # same for the attitude integral (addition (j))
        tau_ff = rng.normal(0.0, 0.3, 3)         # addition (n): a cancelled disturbance torque
        out = CA.flight_law(cfg, p, v, R, om, p_d, v_d, a_d, yaw_d, e_int, e_eta_int, tau_ff)
        cases.append(dict(p=p.tolist(), v=v.tolist(), R=R.flatten(order="F").tolist(), omega=om.tolist(),
                          p_d=p_d.tolist(), v_d=v_d.tolist(), a_d=a_d.tolist(), yaw_d=yaw_d, e_int=e_int.tolist(),
                          e_eta_int=e_eta_int.tolist(), e_eta_i=out["e_eta_i"].tolist(), tau_ff=tau_ff.tolist(),
                          f_t=out["f_t"], tau_body=out["tau_body"].tolist(), tau_eta=out["tau_eta"].tolist(),
                          R_des=out["R_des"].flatten(order="F").tolist(), e_p=out["e_p"].tolist(),
                          e_v=out["e_v"].tolist(), eta=out["eta"].tolist(), eta_d=out["eta_d"].tolist(),
                          e_eta=out["e_eta"].tolist(), eta_dot=out["eta_dot"].tolist(),
                          f_des=out["f_des"].tolist()))
    return cases


def estimator_rollout(rng, cfg, k1, k2, clamp_f, clamp_t, n_steps):
    I = cfg.inertia
    est = CA.MomentumEstimator(cfg.mass, I, k1, k2)
    p = np.zeros(3)
    v = rng.normal(0.0, 0.2, 3)
    R = CA.euler_to_r(np.array([0.1, -0.15, 0.4]))
    om = rng.normal(0.0, 0.3, 3)
    F_true = rng.normal(0.0, 1.5, 3)
    tau_true = rng.normal(0.0, 0.08, 3)
    eta = CA.euler_zyx(R)
    eta_dot = np.linalg.solve(CA.euler_w(eta), om)
    chi0 = est.momentum(v, eta, eta_dot)
    est.reset(chi0)
    steps = []
    for k in range(n_steps):
        dt = 0.004 if k % 7 else 0.0105
        law = CA.flight_law(cfg, p, v, R, om, np.zeros(3), np.zeros(3), np.zeros(3), 0.2)
        eta = CA.euler_zyx(R)
        eta_dot = np.linalg.solve(CA.euler_w(eta), om)
        u = est.applied_force(law["f_t"], R, eta, eta_dot, law["tau_eta"])
        chi = est.momentum(v, eta, eta_dot)
        r_raw = est.update(u, chi, dt)
        # the C++ clamps inside update(); mirror it here on the stored state
        r = r_raw.copy()
        if clamp_f > 0.0:
            r[:3] = np.clip(r[:3], -clamp_f, clamp_f)
        if clamp_t > 0.0:
            r[3:] = np.clip(r[3:], -clamp_t, clamp_t)
        est.r_hat = r
        steps.append(dict(dt=dt, f_t=law["f_t"], R=R.flatten(order="F").tolist(), v=v.tolist(),
                          eta=eta.tolist(), eta_dot=eta_dot.tolist(), tau_eta=law["tau_eta"].tolist(),
                          u=u.tolist(), chi=chi.tolist(), r_hat=r.tolist()))
        acc = (law["f_t"] * (R @ CA.E3) - cfg.mass * CA.G * CA.E3 + F_true) / cfg.mass
        om_dot = np.linalg.solve(I, law["tau_body"] - np.cross(om, I @ om) + tau_true)
        v = v + acc * dt
        p = p + v * dt
        om = om + om_dot * dt
        dR = np.eye(3) + CA.hat(om * dt) + 0.5 * CA.hat(om * dt) @ CA.hat(om * dt)
        R = R @ dR
        U_, _, Vt_ = np.linalg.svd(R)
        R = U_ @ Vt_
    return dict(k1=(np.asarray(k1, float) * np.ones(6)).tolist(), k2=(np.asarray(k2, float) * np.ones(6)).tolist(),
                clamp_f=clamp_f, clamp_t=clamp_t, chi0=chi0.tolist(), steps=steps)


def _num(v):
    """JSON has no inf: +-inf become the strings "+inf"/"-inf"."""
    if np.isposinf(v):
        return "+inf"
    if np.isneginf(v):
        return "-inf"
    return float(v)


def planner_cases(rng):
    """The onboard planner (coordinated_planner.{hpp,cpp}): arm model,
    admittance filter, weights, and the assembled MPC QP + the Python solution."""
    base_com = np.array([0.0, -0.017854, 0.0])
    arm = []
    for _ in range(20):
        q = rng.uniform(CA.Q_MIN, CA.Q_MAX)
        c0, Jc, e0, Je = CA.arm_jacobians(q, base_com)
        arm.append(dict(q=q.tolist(), c=c0.tolist(), e=e0.tolist(), Jc=Jc.flatten(order="F").tolist(),
                        Je=Je.flatten(order="F").tolist()))
    adm = CA.AdmittanceFilter([0.5, 0.5, 0.5], [17.3, 17.3, 24.5], [150.0, 150.0, 300.0], 0.02)
    adm_steps = []
    for k in range(60):
        f = rng.normal(0.0, 2.0, 3) if k < 40 else rng.normal(0.0, 12.0, 3)   # the e_max clamp at the end
        p_o, v_o, a_o = rng.normal(0.0, 1.0, 3), rng.normal(0.0, 0.3, 3), rng.normal(0.0, 0.5, 3)
        dt = 0.05 if k % 5 else 0.08
        p_r, v_r, a_r = adm.update(f, p_o, v_o, a_o, dt)
        adm_steps.append(dict(f=f.tolist(), p_o=p_o.tolist(), v_o=v_o.tolist(), a_o=a_o.tolist(), dt=dt,
                              p_r=p_r.tolist(), v_r=v_r.tolist(), a_r=a_r.tolist(), e=adm.e.tolist(),
                              e_dot=adm.e_dot.tolist()))
    weights = []
    for k in range(12):
        f = rng.normal(0.0, 1.5, 3) if k < 10 else np.zeros(3)
        for xy in (False, True):
            w, w_uav = CA.weight_vector(f, 3.0, 2.0, CA.ARM_M[1:], xy_only=xy)
            weights.append(dict(f=f.tolist(), xy_only=xy, w=w.tolist(), w_uav=w_uav))
    mcfg = CA.MpcConfig(k_p=4.0, u_max=np.array([0.6] * 3 + [3.0] * 4), du_max=np.array([0.3] * 3 + [3.0] * 4),
                        v_max=np.array([0.4] * 3 + [1.0] * 4), q_min=np.radians([-33.0, -18.0, 5.0, -118.0]),
                        q_max=np.radians([33.0, 43.0, 48.0, 118.0]), ee_body_min=np.array([0.05, -0.25, -0.35]),
                        ee_body_max=np.array([0.35, 0.25, 0.10]), base_com=base_com)
    # a HIGH-ACCURACY reference solve (the C++ solves the same QP with OSQP)
    mpc = CA.CoordinatedMpc(mcfg, CA.AdmmQp(max_iter=60000, eps_abs=1e-9, eps_rel=1e-9))
    cap = {}
    orig = mpc.qp.solve

    def grab(P, q, A, l, u, x0=None, z0=None, y0=None):
        cap.update(P=P, q=q, A=A, l=l, u=u)
        return orig(P, q, A, l, u, x0, z0, y0)
    mpc.qp.solve = grab
    q_home = np.array([0.0, 0.6981317, 0.6981317, 0.0])
    cases = []
    for k in range(4):
        q0 = np.clip(q_home + rng.normal(0.0, 0.15, 4), mcfg.q_min, mcfg.q_max)
        v0 = rng.normal(0.0, 0.15, 3) if k != 2 else np.array([0.85, -0.5, 0.1])   # case 2: beyond v_max -> clipped
        qd0 = rng.normal(0.0, 0.3, 4)
        x0 = np.concatenate([rng.normal(0.0, 0.5, 2), [1.2 + rng.normal(0.0, 0.1)], q0, v0, qd0])
        R = CA.euler_to_r(np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1), rng.uniform(-math.pi, math.pi)]))
        _, e0 = CA.arm_fk_actual(q0, base_com)
        p_r = x0[:3] + R @ e0 + rng.normal(0.0, 0.05, 3)
        v_r = rng.normal(0.0, 0.05, 3)
        psi_b = math.atan2(R[1, 0], R[0, 0]) - 0.5 * math.pi
        psi_e = psi_b + rng.uniform(-0.3, 0.3)
        q_ref = np.clip(q0 + rng.normal(0.0, 0.1, 4), mcfg.q_min, mcfg.q_max)
        f = rng.normal(0.0, 1.5, 3) if k != 3 else np.array([2.5, 1.0, 0.0])
        w_am, _ = CA.weight_vector(f, 3.0, 2.0, CA.ARM_M[1:], xy_only=True)
        u_prev = rng.normal(0.0, 0.2, 7)
        mpc.x_warm = None
        out = mpc.solve(x0, R, p_r, v_r, psi_b, psi_e, q_ref, w_am, u_prev)
        P, q, A, l, u = cap["P"], cap["q"], cap["A"], cap["l"], cap["u"]
        cases.append(dict(x0=x0.tolist(), R=R.flatten(order="F").tolist(), p_r=p_r.tolist(), v_r=v_r.tolist(),
                          psi_b=psi_b, psi_e=psi_e, q_ref=q_ref.tolist(), w_am=w_am.tolist(), u_prev=u_prev.tolist(),
                          x0_used=out["x0"].tolist(), x0_clamped=bool(out["x0_clamped"]), viol=int(out["viol"]),
                          n=int(P.shape[0]), m=int(A.shape[0]),
                          P=P.flatten(order="F").tolist(), q=q.tolist(), A=A.flatten(order="F").tolist(),
                          l=[_num(v) for v in l], u=[_num(v) for v in u],
                          h1=out["h1"].tolist(), hdot1=out["hdot1"].tolist(), u0=out["u0"].tolist(),
                          cost=out["cost"], slack=out["slack"].tolist(), converged=bool(out["info"]["converged"]),
                          iters=int(out["info"]["iters"]), p_e_now=out["p_e_now"].tolist()))
    cfg_json = dict(dt=mcfg.dt, n_p=mcfg.n_p, n_c=mcfg.n_c, k_p=mcfg.k_p, w_e=mcfg.w_e.tolist(), w_c=mcfg.w_c.tolist(),
                    w_u_base=mcfg.w_u_base.tolist(), w_u_scale=mcfg.w_u_scale, w_du=mcfg.w_du, w_psi=mcfg.w_psi,
                    w_q_post=mcfg.w_q_post.tolist(), u_max=mcfg.u_max.tolist(), du_max=mcfg.du_max.tolist(),
                    v_max=mcfg.v_max.tolist(), p_min=mcfg.p_min.tolist(), p_max=mcfg.p_max.tolist(),
                    q_min=mcfg.q_min.tolist(), q_max=mcfg.q_max.tolist(), ee_body_min=mcfg.ee_body_min.tolist(),
                    ee_body_max=mcfg.ee_body_max.tolist(), l_m=mcfg.l_m, w_slack_lin=mcfg.w_slack_lin,
                    w_slack_quad=mcfg.w_slack_quad, base_com=base_com.tolist())
    return dict(base_com=base_com.tolist(), link_masses=CA.ARM_M[1:].tolist(), arm=arm,
                admittance=dict(m=adm.M.tolist(), d=adm.D.tolist(), k=adm.K.tolist(), e_max=adm.e_max, steps=adm_steps),
                weights=weights, mpc=dict(cfg=cfg_json, cases=cases))


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUT
    rng = np.random.default_rng(20261010)
    sets = []
    shipped = CA.FlightLawConfig(kpp=[2.5, 2.5, 8.0], kpd=[2.5, 2.5, 5.0], kpi=[1.0, 1.0, 1.0], int_max=0.5,
                                 keta_p=[3.337, 3.337, 1.737], keta_d=[1.2, 1.2, 0.4105],
                                 keta_i=[1.0, 1.0, 0.5], int_eta_max=0.2,
                                 inertia=np.array([[0.118, 0.0089, 0.0], [0.0089, 0.119, 0.0], [0.0, 0.0, 0.131]]),
                                 max_torque_z=0.40)
    clamps = CA.FlightLawConfig(kpp=[12.0, 9.0, 6.0], kpd=[4.0, 4.0, 2.0], keta_p=[6.0, 5.0, 3.0],
                                keta_d=[1.2, 1.1, 0.6], max_pos_err_m=0.3, max_vel_err_mps=0.4,
                                max_tilt_deg=12.0, min_thrust_frac=0.9, max_torque_xy=0.5, max_torque_z=0.1)
    odd = CA.FlightLawConfig(mass=4.1, kpp=[3.0, 4.0, 5.0], kpd=[1.5, 2.5, 3.5], kpi=[0.5, 0.0, 2.0], int_max=0.1,
                             keta_p=[2.0, 3.0, 1.0], keta_i=[0.3, 0.0, 0.7], int_eta_max=0.05,
                             keta_d=[0.5, 0.7, 0.3],
                             inertia=np.array([[0.06, 0.004, -0.002], [0.004, 0.07, 0.003], [-0.002, 0.003, 0.09]]),
                             max_pos_err_m=0.0, max_vel_err_mps=0.0, max_tilt_deg=0.0, min_thrust_frac=0.0,
                             max_torque_xy=0.0, max_torque_z=0.0)
    for name, cfg, k1, k2, cf, ct in (("shipped", shipped, 20.0, 5.0, 20.0, 3.0),
                                      ("clamps", clamps, [20.0] * 3 + [10.0] * 3, [5.0] * 3 + [2.5] * 3, 0.8, 0.05),
                                      ("odd", odd, [8.0, 9.0, 10.0, 11.0, 12.0, 13.0], [2.0, 2.2, 2.4, 2.6, 2.8, 3.0], 0.0, 0.0)):
        sets.append(dict(name=name, gains=gains_to_json(cfg), law=law_cases(rng, cfg, 60),
                         estimator=estimator_rollout(rng, cfg, k1, k2, cf, ct, 300)))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    planner = planner_cases(rng)
    with open(out_path, "w") as f:
        json.dump(dict(g=CA.G, sets=sets, planner=planner), f)
    n_law = sum(len(s["law"]) for s in sets)
    n_est = sum(len(s["estimator"]["steps"]) for s in sets)
    print(f"wrote {out_path}: {len(sets)} gain sets, {n_law} law cases, {n_est} estimator steps; planner: "
          f"{len(planner['arm'])} arm poses, {len(planner['admittance']['steps'])} admittance steps, "
          f"{len(planner['weights'])} weight cases, {len(planner['mpc']['cases'])} MPC QPs")


if __name__ == "__main__":
    main()
