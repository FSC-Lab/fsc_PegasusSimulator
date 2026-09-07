#!/usr/bin/env python3
"""
generate_wb_truth.py — parity truth fixtures for the C++ whole-body port.

Stage 0 of the whole-body fsc_autopilot_ros2 fork (plan: glowing-toasting-
phoenix). Evaluates THE law — utils_controller/controller.py, unmodified —
on the T650 aerial-manipulator model variant over a deterministic batch of
random states/references, and dumps every input and output to JSON. The C++
port (`wb_model.cpp` / `wb_controller.cpp` in fsc_autopilot_ros2's
single_aerial_manipulator_whole_body_direct_actuation fork) must reproduce
every number to <=1e-8 relative before it is allowed near a rotor.

T650 MODEL VARIANT (the load-bearing part — do NOT copy 05's `_body_dI`):
the model lives in AM_realign's y-forward body frame while the flying asset
(AM_xfwd) and t650_params are x-forward. The T650 body override must therefore
be ROTATED into the model frame before it touches I_i_i[0]:

    dI_model = R_MODELᵀ · (T650_INERTIA_TENSOR − diag(AM_xfwd authored)) · R_MODEL
    I0_model(T650) = I0_model(AM_realign) + dI_model        (swaps xx↔yy, flips Ixy)
    m0(T650)       = m0(AM_realign) + (2.95 − 2.4760795)  = 3.1095500 kg

05 adds the un-rotated delta — harmless there (only gravity g[6:] is consumed,
and g carries no inertia), fatal for a whole-body law that consumes I0 through
M_r/Λ_y/T/GMO. This script computes the correct numbers from t650_params and
prints them for the fork's YAML; the JSON embeds the full model + gains so the
C++ parity test is self-contained.

Run (system python3, no Isaac):
    python3 application/robotic_arm/utils/generate_wb_truth.py \
        [out.json]      # default: wb_truth_t650.json next to this script

Deterministic: fixed seed, no wall-clock in the data.
"""

import json
import os
import sys

import numpy as np

# --- import the extension package (controller.py + control_params.py) --------
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "extensions", "fsc_aerial_manipulation"))

from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C          # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import control_params as CP    # noqa: E402

# t650_params is pure python, but its package __init__ drags in Isaac (pxr) —
# load the module straight from its file instead.
import importlib.util as _ilu                                                             # noqa: E402
_t650_spec = _ilu.spec_from_file_location(
    "t650_params",
    os.path.join(_REPO, "extensions", "fsc_aerial_manipulation",
                 "fsc_aerial_manipulation", "rotorcraft", "t650_params.py"))
t650_params = _ilu.module_from_spec(_t650_spec)
_t650_spec.loader.exec_module(t650_params)

SEED = 20260822
DT = 0.004                       # the 250 Hz physics/control step
N_CASES = 220
N_ROLLOUTS = 4
ROLLOUT_STEPS = 10

# Model frame adapter (AM_realign y-forward <- AM_xfwd x-forward), columns.
R_MODEL = np.array([[0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0]])

# AM_xfwd.usda authored /body values (asset ~line 3570; verified 2026-08-22).
AM_XFWD_BODY_MASS = 2.4760795
AM_XFWD_BODY_DIAG_INERTIA = np.diag([0.06334175, 0.06301228, 0.09868092])


# The T650 whole-body model has ONE definition, in the extension, so this
# fixture and the whole-body planner's planner can never describe different robots (they
# were briefly duplicated, 2026-08-23). It also carries the EE = GRIPPER offset
# that the C++ t650Defaults mirrors — the thing this fixture exists to lock.
from fsc_aerial_manipulation.robotic_arm.utils_planner.transition_planner import (  # noqa: E402
    make_params_t650,
)


def _rand_unit(rng, horizontal=False):
    v = rng.normal(size=3)
    if horizontal:
        v[2] = 0.0
    n = np.linalg.norm(v)
    if n < 1e-6:
        return _rand_unit(rng, horizontal)
    return v / n


def _rand_rotation(rng, max_tilt_rad):
    """Random rotation: yaw uniform, tilt axis horizontal, tilt <= max_tilt."""
    yaw = rng.uniform(-np.pi, np.pi)
    Rz = C.joint_rotation(np.array([0.0, 0.0, 1.0]), yaw)
    axis = _rand_unit(rng, horizontal=True)
    tilt = rng.uniform(0.0, max_tilt_rad)
    return Rz @ C.joint_rotation(axis, tilt)


def _rand_state(rng, n, fast_omega=False):
    r0 = rng.uniform(-2.0, 2.0, size=3) + np.array([0.0, 0.0, 1.5])
    R0 = _rand_rotation(rng, np.radians(35.0 if fast_omega else 20.0))
    q = np.array([rng.uniform(-0.55, 0.55),
                  rng.uniform(-1.1, 0.8),
                  rng.uniform(-0.6, 0.8),
                  rng.uniform(-1.2, 1.2)])
    v0 = rng.uniform(-0.8, 0.8, size=3)
    if fast_omega:
        w0 = _rand_unit(rng) * rng.uniform(1.0, 1.6)
    else:
        w0 = rng.uniform(-0.4, 0.4, size=3)
    qdot = rng.uniform(-0.8, 0.8, size=n)
    return np.concatenate([r0, R0.flatten(order="F"), q, v0, w0, qdot])


def _rand_ref(rng, X, dyn, big_error=False):
    """Reference near the current state (or deliberately far, to saturate)."""
    n = 4
    r0 = X[0:3]
    R0 = X[3:12].reshape(3, 3, order="F")
    x_c = r0 + R0 @ dyn["r_0c_0"]
    r_e = r0 + R0 @ dyn["r_0e_0"]
    err = 2.5 if big_error else 0.35
    ref = {
        "x_cd": x_c + rng.uniform(-err, err, size=3),
        "x_cd_dot": rng.uniform(-0.5, 0.5, size=3),
        "x_cd_ddot": rng.uniform(-0.4, 0.4, size=3),
        "x_cd_d3": rng.uniform(-0.3, 0.3, size=3),
        "x_cd_d4": rng.uniform(-0.3, 0.3, size=3),
        "b1_d": _rand_unit(rng, horizontal=True),
        "b1_d_dot": rng.uniform(-0.2, 0.2, size=3),
        "b1_d_ddot": rng.uniform(-0.2, 0.2, size=3),
        "r_ed": r_e + rng.uniform(-err, err, size=3),
        "r_ed_dot": rng.uniform(-0.4, 0.4, size=3),
        "r_ed_ddot": rng.uniform(-0.3, 0.3, size=3),
        "b1_de": _rand_unit(rng, horizontal=True),
        "b1_de_dot": rng.uniform(-0.2, 0.2, size=3),
        "b1_de_ddot": rng.uniform(-0.2, 0.2, size=3),
    }
    return ref


def _jsonify(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    if isinstance(v, dict):
        return {k: _jsonify(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonify(x) for x in v]
    return v


_DYN_KEYS = ("M_r", "C_r", "C_rp", "C_p", "M_tilde", "C_tilde", "g_tilde", "T",
             "M", "C", "g", "A", "N1", "r_0c_0", "r_0e_0", "R_e_0",
             "J_y", "J_y_dot", "Lambda_y", "J_1y", "J_2y", "J_3y",
             "J_q_omega_e", "J_q_dot_omega_e", "omega_0e_0")
_OUT_KEYS = ("u1", "u2", "u3", "thrust", "tau_body", "tau_joint", "tau_body_arm",
             "R0c", "e_R", "e_y", "t_thrust", "t_imp", "t_dist", "t_cpl",
             "d_t_hat", "d_r_hat", "d_rho_hat", "d_e_hat", "gmo_active", "n_sat")


def mass_identity_residual(dyn):
    return float(np.linalg.norm(dyn["M"] - dyn["T"].T @ dyn["M_tilde"] @ dyn["T"])
                 / np.linalg.norm(dyn["M"]))


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_HERE, "wb_truth_t650.json")
    rng = np.random.default_rng(SEED)

    params = make_params_t650()
    n = params["n"]
    cfg = CP.load("px4_direct_free")

    # ---- print the numbers the fork's YAML must carry ----------------------
    I0 = params["I_i_i"][0]
    print("T650 whole-body model (MODEL frame — for the fork YAML):")
    print(f"  base mass  m_i[0] = {params['m_i'][0]:.7f} kg   (total {sum(params['m_i']):.6f})")
    print(f"  I0_xx = {I0[0, 0]:.8f}   I0_yy = {I0[1, 1]:.8f}   I0_zz = {I0[2, 2]:.8f}")
    print(f"  I0_xy = {I0[0, 1]:.8f}   (I0_xz = {I0[0, 2]:.3g}, I0_yz = {I0[1, 2]:.3g})")
    print(f"  gains: {cfg.summary()}")

    data = {
        "meta": {
            "seed": SEED, "dt": DT, "n_cases": N_CASES,
            "n_rollouts": N_ROLLOUTS, "rollout_steps": ROLLOUT_STEPS,
            "gains_source": cfg.source,
            "description": "Whole-body T650 parity truth: controller.py evaluated "
                           "on make_params_t650(); GMO fed the COMMANDED wrench "
                           "(the law's own u), matching the Python law exactly.",
        },
        "model": {
            "n": n,
            "m_i": _jsonify(params["m_i"]),
            "l_i": _jsonify(params["l_i"]),
            "com_i": _jsonify(params["com_i"]),
            "h_i_im1": _jsonify(params["h_i_im1"]),
            "I_i_i": _jsonify(params["I_i_i"]),
            "g": params["g"],
        },
        "gains": {
            "k_x": cfg.k_x, "k_v": cfg.k_v, "k_R": cfg.k_R, "k_w": cfg.k_w,
            "M_r_d_diag": np.diag(cfg.M_r_d).tolist(),
            "K_y_diag": np.diag(cfg.K_y).tolist(),
            "D_y_diag": np.diag(cfg.D_y).tolist(),
            "K_o_diag": np.diag(cfg.K_o).tolist(),
            "use_gmo": bool(cfg.use_gmo),
            "impedance_mode": cfg.impedance_mode,
            "M_Y_diag": None if cfg.M_Y is None else np.diag(cfg.M_Y).tolist(),
            "dls_lambda": cfg.dls_lambda,
            "tau_max": cfg.tau_max,
            "omega_e_ff": bool(cfg.omega_e_ff),
        },
        "cases": [],
        "rollouts": [],
    }

    # ---- single-step cases --------------------------------------------------
    worst_identity = 0.0
    n_saturated = 0
    for k in range(N_CASES):
        fast_omega = (k % 5 == 0)              # 20%: ||omega|| >= 1 rad/s
        big_error = (k % 11 == 3)              # ~9%: force tau saturation
        hold = (k % 7 == 1)                    # ~14%: takeoff-hold branch
        gmo_inhibit = (k % 13 == 2)            # ~8%: inhibit branch
        X = _rand_state(rng, n, fast_omega=fast_omega)
        dyn = C.dynamics(X, params)
        ref = _rand_ref(rng, X, dyn, big_error=big_error)

        ctrl = C.MatlabController(params, cfg)
        ctrl.hold = hold
        ctrl.gmo_inhibit = gmo_inhibit
        # Pre-seed the observer momentum away from p = M~*xi so d_e_hat != 0
        # on active cases (a fresh init would always give d_e_hat = 0).
        p_hat_init = None
        if not hold and not gmo_inhibit:
            pmom = dyn["M_tilde"] @ (dyn["T"] @ np.concatenate(
                [X[12 + n:15 + n], X[15 + n:18 + n], X[18 + n:18 + 2 * n]]))
            p_hat_init = pmom + rng.uniform(-0.5, 0.5, size=6 + n)
            ctrl._p_hat = p_hat_init.copy()

        res = ctrl(X, dyn, ref, DT)

        resid = mass_identity_residual(dyn)
        worst_identity = max(worst_identity, resid)
        n_saturated += int(res["n_sat"] > 0)

        data["cases"].append({
            "X": X.tolist(), "dt": DT,
            "hold": hold, "gmo_inhibit": gmo_inhibit,
            "p_hat_init": None if p_hat_init is None else p_hat_init.tolist(),
            "ref": _jsonify(ref),
            "dyn": {kk: _jsonify(dyn[kk]) for kk in _DYN_KEYS},
            "out": {kk: _jsonify(res[kk]) for kk in _OUT_KEYS},
            "p_hat_after": ctrl._p_hat.tolist(),
            "mass_identity_residual": resid,
        })

    # ---- GMO rollouts (10 forward-Euler steps on a drifting state) ----------
    for r in range(N_ROLLOUTS):
        X = _rand_state(rng, n, fast_omega=(r % 2 == 0))
        dX = np.zeros_like(X)
        dX[0:3] = rng.uniform(-0.02, 0.02, size=3)          # slow positional drift
        dq = rng.uniform(-0.01, 0.01, size=n)
        dyn0 = C.dynamics(X, params)
        ref = _rand_ref(rng, X, dyn0, big_error=False)
        ctrl = C.MatlabController(params, cfg)              # p_hat inits to pmom
        steps = []
        for s in range(ROLLOUT_STEPS):
            dyn = C.dynamics(X, params)
            res = ctrl(X, dyn, ref, DT)
            steps.append({
                "X": X.tolist(),
                "d_e_hat": res["d_e_hat"].tolist(),
                "tau_joint": res["tau_joint"].tolist(),
                "u1": res["u1"],
                "tau_body": res["tau_body"].tolist(),
                "p_hat_after": ctrl._p_hat.tolist(),
            })
            # drift the state smoothly (recompute R0 via a small rotation)
            X = X.copy()
            X[0:3] += dX[0:3]
            R0 = X[3:12].reshape(3, 3, order="F")
            R0 = R0 @ C.joint_rotation(np.array([0.0, 0.0, 1.0]), 0.002)
            X[3:12] = R0.flatten(order="F")
            X[12:12 + n] += dq
        data["rollouts"].append({"ref": _jsonify(ref), "dt": DT, "steps": steps})

    with open(out_path, "w") as f:
        json.dump(data, f)

    size_mb = os.path.getsize(out_path) / 1e6
    print(f"\nWrote {out_path}  ({size_mb:.1f} MB)")
    print(f"  cases: {N_CASES}  (saturated: {n_saturated}, "
          f"fast-omega: {sum(1 for k in range(N_CASES) if k % 5 == 0)}, "
          f"hold: {sum(1 for k in range(N_CASES) if k % 7 == 1)}, "
          f"inhibit: {sum(1 for k in range(N_CASES) if k % 13 == 2)})")
    print(f"  rollouts: {N_ROLLOUTS} x {ROLLOUT_STEPS} steps")
    print(f"  worst |M - T'*Mt*T|/|M| = {worst_identity:.2e}  (G0 gate: ~1e-15)")
    if worst_identity > 1e-12:
        print("G0 FAIL: mass identity residual too large", file=sys.stderr)
        return 1
    if n_saturated == 0:
        print("G0 FAIL: no case exercised tau saturation", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
