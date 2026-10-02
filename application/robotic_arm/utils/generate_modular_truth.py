#!/usr/bin/env python3
"""generate_modular_truth.py -- parity fixture for the C++ modular adaptive law.

The C++ port (fsc_autopilot_ros2 .../single_aerial_manipulator_modular_adaptive_direct_actuation/
client_lib/src/modular_adaptive_law.cpp) is held to the Python reference
(extensions/.../utils_controller/modular_adaptive.py) by
fsc_autopilot_lib/single_vehicle_baseline/tests/src/test_modular_parity.cpp,
which replays the ROLLOUTS written here. Rollouts, not single steps: every
interesting property of the law is a recursion (the adaptive gains, zeta, the
chi_ddot and alpha_ddot_d differentiators).

Coverage, by construction of the inputs:
  * both boundary-layer branches (|r_j| >= varpi_j and < varpi_j) on every
    module -- the measurement errors are swept from large to small;
  * nu = 0 (pure integration) on one gain of one module;
  * the 100 Hz-sample / 250 Hz-tick reference latching (sample_changed);
  * a non-uniform dt;
  * Table II literal, a tuned-shape config (Q split, small nu) and a third
    with every module's constants different from the other two.

    /usr/bin/python3 generate_modular_truth.py [out.json]
Default output: the fixture path the gtest reads (fsc_autopilot_ros2 workspace).
Never hand-edit the fixture.
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_controller import modular_adaptive as MA  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

DEFAULT_OUT = os.path.expanduser(
    "~/ros2_ws/src/fsc_autopilot_ros2/fsc_autopilot_ros2_node/"
    "single_aerial_manipulator_modular_adaptive_direct_actuation/client_lib/tests/data/modular_truth_t650.json")
BASE_COM = [0.0, -0.017854, 0.0]          # MODEL frame (the hardware value; exercises the m0 term)


def ref_at(t):
    w = 0.6
    x = [np.array([0.5 * math.cos(w * t), 0.5 * math.sin(w * t), 1.2 + 0.05 * math.sin(2 * w * t)])]
    for k in range(1, 5):
        x.append(np.array([0.5 * w ** k * math.cos(w * t + k * math.pi / 2),
                           0.5 * w ** k * math.sin(w * t + k * math.pi / 2),
                           0.05 * (2 * w) ** k * math.sin(2 * w * t + k * math.pi / 2)]))
    psi, pd, pdd = 0.26 * t + 0.1 * math.sin(t), 0.26 + 0.1 * math.cos(t), -0.1 * math.sin(t)
    b1 = np.array([math.cos(psi), math.sin(psi), 0.0])
    t1 = np.array([-math.sin(psi), math.cos(psi), 0.0])
    q = np.array([0.05 * math.sin(t), math.radians(25 + 15 * math.sin(1.047 * t)),
                  math.radians(30 - 15 * math.sin(1.047 * t)), 0.1 * math.sin(0.5 * t)])
    qd = np.array([0.05 * math.cos(t), math.radians(15 * 1.047 * math.cos(1.047 * t)),
                   -math.radians(15 * 1.047 * math.cos(1.047 * t)), 0.05 * math.cos(0.5 * t)])
    return dict(x_cd=x[0], x_cd_dot=x[1], x_cd_ddot=x[2], x_cd_d3=x[3], x_cd_d4=x[4],
                b1_d=b1, b1_d_dot=pd * t1, b1_d_ddot=pdd * t1 - pd ** 2 * b1, q_d=q, qdot_d=qd)


def rot(axis, ang):
    return MA.joint_rotation(np.asarray(axis, float), ang)


def configs():
    c1 = MA.table2_config()
    c2 = MA.table2_config()
    for g, (kp, kd) in ((c2.pos, (4.0, 3.5)), (c2.att, (60.0, 12.0))):
        g.lam1 = np.full(3, kp); g.lam2 = np.full(3, kd); g.Lam = np.ones(3)
        g.q_v = 0.05; g.nu = np.full(4, 0.5)
    c2.pos.mbar = np.full(3, 3.746170); c2.att.mbar = np.full(3, 0.095)
    c2.arm.mbar = np.array([0.022, 0.033, 0.016, 0.010]); c2.arm.lam1 = np.full(4, 600.0)
    c2.arm.lam2 = np.full(4, 40.0); c2.arm.q_v = 0.05; c2.arm.nu = np.array([0.5, 0.5, 0.0, 0.5])
    c2.pos.varpi, c2.att.varpi, c2.arm.varpi = 0.02, 0.004, 0.003
    c3 = MA.table2_config(m_g=3.5)
    c3.pos.mbar = np.array([3.0, 3.2, 3.4]); c3.pos.lam1 = np.array([5.0, 6.0, 9.0]); c3.pos.lam2 = np.array([3.0, 3.3, 5.0])
    c3.pos.Lam = np.array([1.2, 0.9, 1.1]); c3.pos.q_e, c3.pos.q_v = 2.0, 0.3; c3.pos.varpi = 0.02
    c3.att.mbar = np.array([0.09, 0.1, 0.11]); c3.att.lam1 = np.array([40.0, 45.0, 25.0]); c3.att.lam2 = np.array([10.0, 11.0, 8.0])
    c3.att.q_e, c3.att.q_v = 0.7, 0.1; c3.att.varpi = 0.003; c3.att.nu = np.array([2.0, 0.1, 5.0, 1.0])
    c3.arm.lam1 = np.array([300.0, 400.0, 500.0, 200.0]); c3.arm.lam2 = np.array([30.0, 35.0, 40.0, 25.0])
    c3.arm.Lam = np.array([1.0, 1.1, 0.9, 1.0]); c3.arm.varpi = 0.01; c3.arm.K_init = 0.05
    c3.acc_filter_hz = 7.0; c3.tau_max = 1.5
    return [("table2", c1, 10.0), ("tuned_shape", c2, 10.0), ("mixed", c3, 4.0)]


def gains_json(g):
    return dict(mbar=list(g.mbar), lam1=list(g.lam1), lam2=list(g.lam2), Lam=list(g.Lam), q_e=g.q_e, q_v=g.q_v,
                nu=list(g.nu), eps=g.eps, varpi=g.varpi, K_init=g.K_init, zeta_init=g.zeta_init)


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUT
    params = TP.make_params_t650(armature_diag=[0.010, 0.0194, 0.0097, 0.0097])
    params["base_com"] = np.array(BASE_COM)
    rng = np.random.default_rng(7)
    fx = {"generator": os.path.basename(__file__), "base_com": BASE_COM, "rollouts": []}
    for name, cfg, qdd_hz in configs():
        law = MA.ModularAdaptiveLaw(cfg)
        rc = MA.ReferenceConverter(params, qdd_filter_hz=qdd_hz)
        steps = []
        t = 0.0
        n = 90
        for k in range(n):
            dt = 0.004 if k % 17 else 0.0053          # a non-uniform tick now and then
            t += dt
            changed = (k % 5) in (0, 2)               # ~100 Hz samples on a 250 Hz tick
            tr = 0.01 * (k - (k % 5) + (2 if (k % 5) >= 2 else 0))   # the sample's own time
            ref = ref_at(tr)
            # errors decay from large (|r| >= varpi) to small (< varpi): both branches
            s = 1.0 if k < n // 3 else (0.2 if k < 2 * n // 3 else 0.02)
            p = ref["x_cd"] + s * rng.normal(0, 0.15, 3)
            v = ref["x_cd_dot"] + s * rng.normal(0, 0.3, 3)
            psi = math.atan2(ref["b1_d"][1], ref["b1_d"][0]) + math.pi / 2
            R = rot([0, 0, 1], psi + s * rng.normal(0, 0.2)) @ rot([1, 0, 0], s * rng.normal(0, 0.15)) \
                @ rot([0, 1, 0], s * rng.normal(0, 0.15))
            Om = s * rng.normal(0, 0.8, 3)
            q = ref["q_d"] + s * rng.normal(0, 0.2, 4)
            qd = ref["qdot_d"] + s * rng.normal(0, 0.5, 4)
            meas = dict(p=p, v=v, R=R, Omega=Om, q=q, qdot=qd)
            c = rc.convert(ref, dt, changed)
            o = law.step(meas, c, dt)
            L = lambda a: [float(x) for x in np.asarray(a).ravel(order="F")]  # noqa: E731
            steps.append(dict(
                dt=dt, changed=bool(changed),
                ref={k2: L(v2) for k2, v2 in ref.items()},
                meas=dict(p=L(p), v=L(v), R=L(R), Omega=L(Om), q=L(q), qdot=L(qd)),
                rc=dict(p_d=L(c["p_d"]), v_d=L(c["v_d"]), a_d=L(c["a_d"]), R_ff=L(c["R_ff"]),
                        w_d=L(c["w_d"]), wdot_d=L(c["wdot_d"]), qddot_d=L(c["qddot_d"])),
                out=dict(tau_p=L(o["tau_p"]), u1=float(o["u1"]), Rd=L(o["Rd"]), tau_q=L(o["tau_q"]),
                         tau_a=L(o["tau_a"]), n_sat=int(o["n_sat"]), xi_n=float(o["xi_n"]),
                         xi_n0=float(o["xi_n0"]), chi_n=float(o["chi_n"]), rho=L(o["rho"]), nr=L(o["nr"]),
                         K_p=L(o["K_p"]), K_q=L(o["K_q"]), K_a=L(o["K_a"]), zeta=L(o["zeta"]))))
        nr_all = np.array([s_["out"]["nr"] for s_ in steps])
        vp = np.array([cfg.pos.varpi, cfg.att.varpi, cfg.arm.varpi])
        cov = [(bool((nr_all[:, j] >= vp[j]).any()), bool((nr_all[:, j] < vp[j]).any())) for j in range(3)]
        print(f"  {name:12s}: {len(steps)} steps, boundary-layer branches hit (outside, inside) per module {cov}")
        fx["rollouts"].append(dict(name=name, qdd_filter_hz=qdd_hz, m_g=cfg.m_g, tau_max=cfg.tau_max,
                                   acc_filter_hz=cfg.acc_filter_hz, min_thrust_frac=cfg.min_thrust_frac,
                                   pos=gains_json(cfg.pos), att=gains_json(cfg.att), arm=gains_json(cfg.arm),
                                   steps=steps))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(fx, f)
    print(f"wrote {out_path} ({os.path.getsize(out_path) / 1024:.0f} kB)")


if __name__ == "__main__":
    main()
