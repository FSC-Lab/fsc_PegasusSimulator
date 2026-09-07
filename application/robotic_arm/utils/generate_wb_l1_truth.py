#!/usr/bin/env python3
"""
generate_wb_l1_truth.py — parity truth for the C++ L1 adaptive observer.

The sibling of generate_wb_truth.py, for the OTHER estimator. It evaluates
the Python reference (utils_controller/l1_observer.py) on the same T650
whole-body model over a deterministic batch of rollouts and dumps every input
and output to JSON; the C++ port (wb_l1_observer.cpp in fsc_autopilot_ros2's
single_aerial_manipulator_whole_body_direct_actuation fork) must reproduce it
before it is allowed near a rotor.  Gate: 1e-8 relative, the same as the law's.

A SEPARATE file and a SEPARATE fixture on purpose. wb_truth_t650.json locks
the GMO law and has been regenerated for several model changes already;
mixing a second estimator into it would mean one file whose regeneration
touches two independent things.

WHY ROLLOUTS AND NOT SINGLE STEPS. Every interesting property of this
estimator is a recursion: the deadbeat inversion of the predictor error, the
piecewise-constant hold across an adaptation interval, three first-order
filters, and the null-channel observer that integrates from w_hat(0) = 0. A
single step would check none of them. Each rollout therefore drives a drifting
state for 40 steps and records the full internal state at every one.

Run (system python3, no Isaac, no ROS):
    python3 application/robotic_arm/utils/generate_wb_l1_truth.py [out.json]
    # default: wb_l1_truth_t650.json in the fork's tests/data/

Deterministic: fixed seed, no wall-clock in the data.
"""

import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "extensions", "fsc_aerial_manipulation"))

from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C        # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import l1_observer as L1      # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

SEED = 20260906
DT = 0.004                # the 250 Hz control step
ROLLOUT_STEPS = 40

#: One rollout per gain set. They deliberately cover every branch the
#: implementation has: N = 1 vs N > 1 adaptation, equal vs unequal A_s across
#: the three channel groups, minimum-norm vs prior-variance L_c, and the
#: decompose on/off switch that selects between the attributed and the lumped
#: task force.
GAIN_SETS = [
    dict(tag="baseline", a_t=2.0, a_r=2.0, a_q=2.0, adapt_period_s=0.0,
         omega_c_t=2.0, omega_c_r=0.5, omega_c_q=0.5, omega_i=1.0,
         omega_x=20.0, lc_var_f=1.0, lc_var_m=1.0, lc_var_q=1.0,
         decompose=True),
    dict(tag="weighted-fast", a_t=8.0, a_r=4.0, a_q=2.0, adapt_period_s=0.0,
         omega_c_t=6.0, omega_c_r=3.0, omega_c_q=1.0, omega_i=5.0,
         omega_x=30.0, lc_var_f=100.0, lc_var_m=0.25, lc_var_q=0.0025,
         decompose=True),
    dict(tag="held-Ts-N5", a_t=20.0, a_r=15.0, a_q=10.0, adapt_period_s=0.020,
         omega_c_t=4.0, omega_c_r=2.0, omega_c_q=1.0, omega_i=2.0,
         omega_x=25.0, lc_var_f=100.0, lc_var_m=0.25, lc_var_q=0.0025,
         decompose=True),
    dict(tag="lumped-Fy", a_t=3.0, a_r=3.0, a_q=3.0, adapt_period_s=0.0,
         omega_c_t=1.0, omega_c_r=1.0, omega_c_q=1.0, omega_i=0.5,
         omega_x=15.0, lc_var_f=25.0, lc_var_m=1.0, lc_var_q=0.01,
         decompose=False),
    dict(tag="clamped", a_t=5.0, a_r=5.0, a_q=5.0, adapt_period_s=0.008,
         omega_c_t=8.0, omega_c_r=8.0, omega_c_q=8.0, omega_i=3.0,
         omega_x=40.0, lc_var_f=100.0, lc_var_m=0.25, lc_var_q=0.0025,
         decompose=True, max_force_n=1.5, max_torque_nm=0.2,
         max_joint_nm=0.15, max_wrench_force_n=2.0,
         max_wrench_torque_nm=0.3),
]


def _rand_rotation(rng, max_tilt_rad):
    ax = rng.normal(size=3)
    ax /= np.linalg.norm(ax)
    ang = rng.uniform(-max_tilt_rad, max_tilt_rad)
    K = C.hat(ax)
    return np.eye(3) + np.sin(ang) * K + (1.0 - np.cos(ang)) * K @ K


def _rand_state(rng, n):
    R0 = _rand_rotation(rng, 0.45)
    q = np.concatenate([rng.uniform(-0.5, 0.5, 1), rng.uniform(-0.4, 0.8, 2),
                        rng.uniform(-1.5, 1.5, 1)])
    return np.concatenate([
        rng.uniform(-2.0, 2.0, 3), R0.flatten(order="F"), q,
        rng.uniform(-0.6, 0.6, 3), rng.uniform(-0.8, 0.8, 3),
        rng.uniform(-0.7, 0.7, n)])


def _xi_of(X, dyn, n):
    V = np.concatenate([X[12 + n:15 + n], X[15 + n:18 + n],
                        X[18 + n:18 + 2 * n]])
    return dyn["T"] @ V


def _j(v):
    return np.asarray(v, dtype=float).tolist()


def main():
    default_out = os.path.abspath(os.path.join(
        _REPO, "..", "ros2_ws", "src", "fsc_autopilot_ros2",
        "fsc_autopilot_ros2_node",
        "single_aerial_manipulator_whole_body_direct_actuation",
        "client_lib", "tests", "data", "wb_l1_truth_t650.json"))
    out_path = sys.argv[1] if len(sys.argv) > 1 else default_out
    rng = np.random.default_rng(SEED)
    params = TP.make_params_t650()
    n = params["n"]

    data = {
        "meta": {
            "seed": SEED, "dt": DT, "rollout_steps": ROLLOUT_STEPS,
            "description":
                "L1 adaptive observer parity truth: l1_observer.py evaluated "
                "on transition_planner.make_params_t650(). Each rollout "
                "drives a drifting state and records the estimator's full "
                "internal state at every step. The commanded wrench u fed to "
                "propagate() is a deterministic function of the step index, "
                "not a control law -- the estimator does not care where u "
                "comes from, and this keeps the fixture independent of the "
                "law's own parity file.",
        },
        # The model is embedded so the C++ test is self-contained (and so a
        # model change makes THIS fixture stale too, loudly).
        "model": {
            "n": n,
            "m_i": _j(params["m_i"]),
            "l_i": _j(params["l_i"]),
            "com_i": _j(params["com_i"]),
            "h_i_im1": _j(params["h_i_im1"]),
            "I_i_i": _j(params["I_i_i"]),
            "g": params["g"],
        },
        "rollouts": [],
    }

    worst_geom = 0.0
    for gs in GAIN_SETS:
        tag = gs["tag"]
        kw = {k: v for k, v in gs.items() if k != "tag"}
        gains = L1.L1Gains(**kw)
        obs = L1.L1DisturbanceObserver(gains)

        X = _rand_state(rng, n)
        dX = rng.uniform(-0.03, 0.03, size=3)
        dq = rng.uniform(-0.012, 0.012, size=n)
        steps = []
        for s in range(ROLLOUT_STEPS):
            dyn = C.dynamics(X, params)
            R0 = X[3:12].reshape(3, 3, order="F")
            xi = _xi_of(X, dyn, n)
            est = obs.update(dyn, R0, xi, DT)
            # A deterministic pseudo-command with a hover-sized collective and
            # small moments/joint torques, plus a slow sweep so the predictor
            # is exercised rather than sitting at one operating point.
            u = np.concatenate([
                R0 @ np.array([0.0, 0.0, 36.75 + 2.0 * np.sin(0.7 * s)]),
                np.array([0.08 * np.sin(0.3 * s), -0.05 * np.cos(0.4 * s),
                          0.02 * np.sin(0.9 * s)]),
                np.array([0.01, 0.45 + 0.05 * np.sin(0.5 * s),
                          0.35 - 0.04 * np.cos(0.6 * s), -0.01])])
            steps.append({
                "X": X.tolist(),
                "u": u.tolist(),
                "d_sigma_c": _j(est["d_sigma_c"]),
                "d_f": _j(est["d_f"]),
                "F_y": _j(est["F_y"]),
                "w_hat": _j(est["w_hat"]),
                "w_e": _j(est["w_e"]),
                "adapted": bool(est["adapted"]),
            })
            obs.propagate(dyn, xi, u, DT)

            # geometry cross-check, recorded on the first step of each rollout
            if s == 0:
                Je = L1.end_effector_jacobian(dyn)
                Z0 = L1.null_motion_basis(dyn)
                worst_geom = max(worst_geom, float(np.abs(Je @ Z0).max()))
                steps[-1]["J_e"] = _j(Je)
                steps[-1]["Z_0"] = _j(Z0)

            X = X.copy()
            X[0:3] += dX
            R0n = X[3:12].reshape(3, 3, order="F") @ C.joint_rotation(
                np.array([0.0, 0.0, 1.0]), 0.0025)
            X[3:12] = R0n.flatten(order="F")
            X[12:12 + n] += dq

        data["rollouts"].append({
            "tag": tag,
            "gains": {k: (bool(v) if isinstance(v, bool) else float(v))
                      for k, v in kw.items()},
            "dt": DT,
            "steps": steps,
        })

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(data, f)
    size_mb = os.path.getsize(out_path) / 1e6
    print(f"Wrote {out_path}  ({size_mb:.2f} MB)")
    print(f"  rollouts: {len(GAIN_SETS)} x {ROLLOUT_STEPS} steps  "
          f"({', '.join(g['tag'] for g in GAIN_SETS)})")
    print(f"  worst |J_e Z_0| over the recorded configurations: {worst_geom:.2e}")
    # A quick look at what the fixture actually contains, so a regeneration
    # that silently produced zeros is obvious.
    for r in data["rollouts"]:
        last = r["steps"][-1]
        print(f"  {r['tag']:<14s} |d^c|={np.abs(last['d_sigma_c']).max():9.4f}"
              f"  |d_f|={np.abs(last['d_f']).max():8.4f}"
              f"  |F_y|={np.abs(last['F_y']).max():8.4f}"
              f"  |w_e|={np.abs(last['w_e']).max():8.4f}"
              f"  adapted={last['adapted']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
