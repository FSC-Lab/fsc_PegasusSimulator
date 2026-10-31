#!/usr/bin/env python3
"""generate_arbitrary_time_truth.py -- parity fixture for the C++ port of the
arbitrary-convergence-time architecture (Chen et al., IEEE TIE 72(12) 2025).

The C++ (fsc_autopilot_ros2 .../single_aerial_manipulator_arbitrary_time_direct_
actuation/client_lib/src/arbitrary_time_law.cpp) is held to the Python reference
(extensions/.../utils_controller/arbitrary_time_control.py) by fsc_autopilot_lib/
single_vehicle_baseline/tests/src/test_arbitrary_time_parity.cpp, which replays
what is written here.

Coverage:
  * the arm model: FK (r_0c, r_0e, Jv) with a non-zero base_com, M, C, G at
    random poses / rates / base attitudes;
  * J_E tau_hat (the interaction on the base) and the stream -> base reference;
  * the quadrotor law at states that put the window in its SHRINKING phase and on
    its FLOOR, with every clamp hit on some case (position/velocity/attitude
    error saturations, the tilt clamp, the thrust floor, the torque clamps);
  * the arm law (shrinking / floored), the admittance with its box clamp hit;
  * ACTNDO rolled out on a simulated arm with retriggered windows and the
    estimate clamp hit on one set;
  * TSKFE rolled out over an input path that spawns rules, with a second
    configuration whose rule cap is reached (ephemeral evaluations).

    /usr/bin/python3 generate_arbitrary_time_truth.py [out.json]
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
from fsc_aerial_manipulation.robotic_arm.utils_controller import arbitrary_time_control as A  # noqa: E402

DEFAULT_OUT = os.path.expanduser(
    "~/ros2_ws/src/fsc_autopilot_ros2/fsc_autopilot_ros2_node/"
    "single_aerial_manipulator_arbitrary_time_direct_actuation/client_lib/tests/data/"
    "arbitrary_time_truth_t650.json")

L = lambda a: np.asarray(a, float).flatten(order="F").tolist()  # noqa: E731 (column-major, Eigen's)


def arm_cases(rng, n):
    out = []
    for _ in range(n):
        q = rng.uniform([-0.5, -0.3, 0.1, -1.5], [0.5, 0.8, 0.9, 1.5])
        qd = rng.normal(0.0, 0.6, 4)
        R = A.euler_to_r([rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), rng.uniform(-math.pi, math.pi)])
        g_b = (R @ A.RZ_M90).T @ np.array([0.0, 0.0, -A.G_ACC])
        bc = rng.normal(0.0, 0.02, 3)
        arm = A.ARMATURE * rng.uniform(0.5, 1.5, 4)
        r0c, r0e, Jv = A.arm_fk(q, bc)
        A_save = A.ARMATURE.copy()
        A.ARMATURE[:] = arm
        M, C, G = A.arm_dynamics(q, qd, g_b)
        A.ARMATURE[:] = A_save
        out.append(dict(q=L(q), qd=L(qd), g_b=L(g_b), base_com=L(bc), armature=L(arm),
                        r0c=L(r0c), r0e=L(r0e), Jv=L(Jv), M=L(M), C=L(C), G=L(G)))
    return out


def interaction_cases(rng, n):
    out = []
    for _ in range(n):
        q = rng.uniform([-0.5, -0.3, 0.1, -1.5], [0.5, 0.8, 0.9, 1.5])
        tau = rng.normal(0.0, 0.3, 4)
        R = A.euler_to_r(rng.uniform(-0.5, 0.5, 3))
        lam = float(rng.uniform(0.01, 0.1))
        F, Mb, Fm = A.interaction_on_base(q, tau, R, lam)
        out.append(dict(q=L(q), tau=L(tau), R=L(R), lam=lam, F=L(F), M=L(Mb), Fm=L(Fm)))
    return out


def stream_cases(rng, n):
    out = []
    for _ in range(n):
        x_cd = rng.normal(0.0, 1.0, 3)
        acc = rng.normal(0.0, 0.8, 3)
        ang = rng.uniform(-math.pi, math.pi)
        b1 = np.array([math.cos(ang), math.sin(ang), rng.normal(0.0, 0.05)])
        q = rng.uniform([-0.5, -0.3, 0.1, -1.5], [0.5, 0.8, 0.9, 1.5])
        bc = rng.normal(0.0, 0.02, 3)
        r_ed = rng.normal(0.0, 1.0, 3)
        x_b, yaw, term, res = A.base_reference(x_cd, acc, b1, q, bc, r_ed)
        out.append(dict(x_cd=L(x_cd), x_cd_ddot=L(acc), b1_d=L(b1), q=L(q), base_com=L(bc), r_ed=L(r_ed),
                        x_b=L(x_b), yaw=yaw, arm_term=L(term), residual=res))
    return out


def quad_cfgs():
    c0 = A.QuadConfig()
    c1 = A.QuadConfig(eta1=np.array([1.2, 2.0, 1.1, 5.0, 3.5, 2.5]), eta2=np.array([1.6, 1.0, 2.2, 3.0, 6.0, 4.0]),
                      DT=2.0, floor=0.3, ff_gain=0.7, max_pos_err_m=0.3, max_vel_err_mps=0.4, max_att_err_rad=0.2,
                      max_tilt_deg=12.0, min_thrust_frac=0.8, max_torque_xy=0.5, max_torque_z=0.05)
    c1.inertia = np.array([[0.12, 0.01, -0.004], [0.01, 0.11, 0.003], [-0.004, 0.003, 0.14]])
    return [c0, c1]


def qcfg_json(c):
    return dict(mass=c.mass, inertia=L(c.inertia), eta1=L(c.eta1), eta2=L(c.eta2), DT=c.DT, floor=c.floor,
                ff_gain=c.ff_gain, max_pos_err_m=c.max_pos_err_m, max_vel_err_mps=c.max_vel_err_mps,
                max_att_err_rad=c.max_att_err_rad, max_tilt_deg=c.max_tilt_deg,
                min_thrust_frac=c.min_thrust_frac, max_torque_xy=c.max_torque_xy, max_torque_z=c.max_torque_z)


def quad_cases(rng, n):
    out = []
    for ci, c in enumerate(quad_cfgs()):
        for _ in range(n):
            win = A.Window(c.DT, c.floor, t0=0.0)
            t = float(rng.uniform(0.0, c.DT + 1.0))              # shrinking or floored
            eta = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-math.pi, math.pi)])
            R = A.euler_to_r(eta)
            p = rng.normal(0.0, 1.0, 3)
            v = rng.normal(0.0, 0.6, 3)
            om = rng.normal(0.0, 0.8, 3)
            p_d = p + rng.normal(0.0, 0.6, 3)
            v_d = rng.normal(0.0, 0.4, 3)
            a_d = rng.normal(0.0, 1.0, 3)
            yaw_d = float(rng.uniform(-math.pi, math.pi))
            D = np.concatenate([rng.normal(0.0, 2.0, 3), rng.normal(0.0, 0.1, 3)])
            F = rng.normal(0.0, 1.0, 3)
            Mb = rng.normal(0.0, 0.1, 3)
            r = A.quad_law(c, win, t, p, v, R, om, p_d, v_d, a_d, yaw_d, D, F, Mb)
            out.append(dict(cfg=ci, t=t, p=L(p), v=L(v), R=L(R), omega=L(om), p_d=L(p_d), v_d=L(v_d), a_d=L(a_d),
                            yaw_d=yaw_d, D=L(D), F=L(F), Mb=L(Mb),
                            thrust=r.thrust, torque=L(r.torque), f_des=L(r.f_des), eta=L(r.eta), eta_d=L(r.eta_d),
                            eta_dot=L(r.eta_dot), r1=L(r.r1), r2=L(r.r2), eps=L(r.eps), g=L(r.g), x1=r.x1, x2=r.x2))
    return out


def acfg(c):
    return dict(Hd=L(c.Hd), Dd=L(c.Dd), Kd=L(c.Kd), eta1=L(c.eta1), eta2=L(c.eta2), DT=c.DT, floor=c.floor,
                e_max=c.e_max, q_min=L(c.q_min), q_max=L(c.q_max), tau_max=c.tau_max)


def arm_cfgs():
    c0 = A.ArmConfig()
    c1 = A.ArmConfig(Hd=np.array([0.5, 1.0, 2.0, 0.8]), Dd=np.array([3.0, 6.0, 2.0, 5.0]),
                     Kd=np.array([20.0, 9.0, 4.0, 15.0]), eta1=np.array([3.0, 6.0, 2.0, 4.0]),
                     eta2=np.array([5.0, 2.5, 3.0, 6.0]), DT=2.5, floor=0.4, e_max=0.05, tau_max=0.4)
    return [c0, c1]


def arm_law_cases(rng, n):
    out = []
    for ci, c in enumerate(arm_cfgs()):
        for _ in range(n):
            win = A.Window(c.DT, c.floor, t0=0.0)
            t = float(rng.uniform(0.0, c.DT + 1.0))
            q = rng.uniform([-0.5, -0.3, 0.1, -1.5], [0.5, 0.8, 0.9, 1.5])
            qd = rng.normal(0.0, 0.5, 4)
            g_b = A.euler_to_r(rng.uniform(-0.3, 0.3, 3)).T @ np.array([0.0, 0.0, -A.G_ACC])
            M, C, G = A.arm_dynamics(q, qd, g_b)
            tau_hat = rng.normal(0.0, 0.2, 4)
            qc = q + rng.normal(0.0, 0.2, 4)
            qcd = rng.normal(0.0, 0.3, 4)
            qcdd = rng.normal(0.0, 0.5, 4)
            tau, th1, _ = A.arm_law(c, win, t, q, qd, M, C, G, tau_hat, qc, qcd, qcdd)
            out.append(dict(cfg=ci, t=t, q=L(q), qd=L(qd), M=L(M), C=L(C), G=L(G), tau_hat=L(tau_hat), qc=L(qc),
                            qcd=L(qcd), qcdd=L(qcdd), tau=L(tau), th1=L(th1)))
    return out


def admittance_rollouts(rng):
    out = []
    for ci, c in enumerate(arm_cfgs()):
        adm = A.ArmAdmittance(c)
        q_d = np.array([0.0, 0.55, 0.49, 0.0])
        steps = []
        for k in range(300):
            dt = float(rng.uniform(0.003, 0.005))
            tau = np.array([1.2 * math.sin(0.03 * k), -0.8, 0.6 * math.cos(0.01 * k), 0.4]) * (1.0 if k < 200 else -1.0)
            e, ed, edd = adm.step(dt, tau, q_d)
            steps.append(dict(dt=dt, tau=L(tau), q_d=L(q_d), e=L(e), ed=L(ed), edd=L(edd)))
        out.append(dict(cfg=ci, steps=steps))
    return out


def actndo_rollouts():
    out = []
    cfgs = [A.ActndoConfig(),
            A.ActndoConfig(k=4.0, eta_o1=2.0, eta_o2=3.0, gamma1=np.array([10.0, 30.0, 20.0, 15.0]),
                           gamma2=np.array([40.0, 10.0, 25.0, 5.0]), dt_o1=0.6, dt_o2=0.9, floor=0.15, est_max=0.12)]
    for ci, c in enumerate(cfgs):
        obs = A.Actndo(c)
        q = np.array([0.0, 0.55, 0.5, 0.0])
        qd = np.zeros(4)
        g0 = np.array([0.0, 0.0, -A.G_ACC])
        obs.reset(A.arm_mass_matrix(q) @ qd)
        obs.trigger(0.0)
        steps = []
        dt = 0.004
        t = 0.0
        for k in range(500):
            trig = (k == 150 or k == 380)
            if trig:
                obs.trigger(t)
            te = np.array([0.08, -0.25, 0.15, 0.02]) if k >= 150 else np.zeros(4)
            M, C, G = A.arm_dynamics(q, qd, g0)
            tau_m = G + M @ (-6.0 * qd - 25.0 * (q - np.array([0.0, 0.55, 0.5, 0.0])))
            H = M @ qd
            CTqd = C.T @ qd
            est = obs.update(dt, t, H, CTqd, G, tau_m)
            steps.append(dict(t=t, trig=bool(trig), H=L(H), CTqd=L(CTqd), G=L(G), tau_m=L(tau_m),
                              est=L(est), s1=L(obs.s1_est)))
            qdd = np.linalg.solve(M, tau_m + te - C @ qd - G)
            qd = qd + dt * qdd
            q = q + dt * qd
            t += dt
        out.append(dict(cfg=dict(k=c.k, eta_o1=c.eta_o1, eta_o2=c.eta_o2, gamma1=L(c.gamma1), gamma2=L(c.gamma2),
                                 dt_o1=c.dt_o1, dt_o2=c.dt_o2, floor=c.floor, est_max=c.est_max),
                        H0=L(A.arm_mass_matrix(np.array([0.0, 0.55, 0.5, 0.0])) @ np.zeros(4)), steps=steps))
    return out


def fuzzy_rollouts(rng):
    out = []
    cfgs = [A.FuzzyConfig(),
            A.FuzzyConfig(k_closest=2, l_max=0.05, c=np.array([5.0, 10.0, 15.0, 0.5, 0.2, 0.3]), max_rules=30,
                          a_max=np.array([2.0, 2.0, 2.0, 0.2, 0.2, 0.2]), out_max=np.array([1.5, 1.5, 1.5, 0.1, 0.1, 0.1]))]
    for ci, c in enumerate(cfgs):
        fz = A.TskFuzzyEstimator(c)
        steps = []
        for k in range(400):
            x1 = abs(0.25 * math.sin(0.021 * k) + 0.02 * math.sin(0.37 * k))
            x2 = abs(0.6 * math.cos(0.013 * k) * math.sin(0.05 * k))
            g = np.array([math.sin(0.01 * k), -0.5, 0.3 * math.cos(0.02 * k), 0.1, -0.2, 0.05 * math.sin(0.1 * k)])
            dt = 0.004
            D, ph, X = fz.evaluate(x1, x2)
            fz.adapt(dt, g, ph, X)
            steps.append(dict(x1=x1, x2=x2, g=L(g), dt=dt, D=L(D), n_rules=fz.n_rules, n_gen=fz.n_generated,
                              n_eph=fz.n_ephemeral))
        out.append(dict(cfg=dict(k_closest=c.k_closest, l_max=c.l_max, c=L(c.c), max_rules=c.max_rules,
                                 a_max=L(c.a_max), out_max=L(c.out_max)), steps=steps))
    return out


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUT
    rng = np.random.default_rng(20261024)
    data = dict(
        description="arbitrary_time_control.py parity fixture (generate_arbitrary_time_truth.py) -- never hand-edit",
        total_mass=A.total_mass(),
        arm=arm_cases(rng, 25),
        interaction=interaction_cases(rng, 20),
        stream=stream_cases(rng, 20),
        quad_cfgs=[qcfg_json(c) for c in quad_cfgs()],
        quad=quad_cases(rng, 40),
        arm_cfgs=[acfg(c) for c in arm_cfgs()],
        arm_law=arm_law_cases(rng, 30),
        admittance=admittance_rollouts(rng),
        actndo=actndo_rollouts(),
        fuzzy=fuzzy_rollouts(rng),
    )
    # coverage asserts: clamps and phases actually exercised
    qc = data["quad"]
    assert any(c["cfg"] == 1 and abs(abs(c["torque"][2]) - 0.05) < 1e-12 for c in qc), "yaw clamp not hit"
    assert any(c["t"] < quad_cfgs()[c["cfg"]].DT - quad_cfgs()[c["cfg"]].floor for c in qc), "no shrinking case"
    assert any(c["t"] > quad_cfgs()[c["cfg"]].DT for c in qc), "no floored case"
    assert any(any(s["e"][j] == s["e"][j] and abs(abs(s["e"][j]) - 0.05) < 1e-12 for j in range(4))
               for s in data["admittance"][1]["steps"]), "admittance clamp not hit"
    assert data["fuzzy"][0]["steps"][-1]["n_gen"] > 3, "TSKFE generated no rules"
    assert data["fuzzy"][1]["steps"][-1]["n_eph"] > 0, "rule cap not reached"
    assert any(abs(abs(v) - 0.12) < 1e-12 for s in data["actndo"][1]["steps"] for v in s["est"]), "est clamp not hit"
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(data, f)
    print(f"wrote {out_path}: {len(data['arm'])} arm, {len(data['quad'])} quad, {len(data['arm_law'])} arm-law cases, "
          f"2 admittance / 2 ACTNDO / 2 TSKFE rollouts (rules {data['fuzzy'][0]['steps'][-1]['n_rules']}, "
          f"generated {data['fuzzy'][0]['steps'][-1]['n_gen']}; capped set ephemeral "
          f"{data['fuzzy'][1]['steps'][-1]['n_eph']})")


if __name__ == "__main__":
    main()
