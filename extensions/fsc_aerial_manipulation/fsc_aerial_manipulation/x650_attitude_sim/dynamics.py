#!/usr/bin/env python
"""
| File: dynamics.py
| License: BSD-3-Clause
| Description: Standalone (no Isaac Sim) X650 attitude-only rigid-body
|   dynamics, using the SAME rotor calibration, rotor spin-up lag, and
|   thrust-allocation convention the real Isaac Sim path
|   (Multirotor.update/force_and_torques_to_velocities +
|   LaggedQuadraticThrustCurve) uses, plus the new propeller parasitic-drag
|   and rotor-gyroscopic-torque models (aerodynamics/propeller_drag.py).

Attitude-only, matching Automatica2026Simulation/quad_attitude_sim's scope
(same "set the translational dynamics aside first" instruction this session
started with, applied here to a second, unrelated project): state is just
(R, omega) -- the vehicle body attitude and body-frame angular rate.
Commanded total thrust is held at hover (m*g) throughout (no altitude loop);
only the commanded MOMENT (from controller.py's geometric law) varies. This
mirrors real flight where the attitude/rate loop runs regardless of what the
altitude loop is doing, and isolates exactly the piece being validated.

Per-step pipeline (all four real-sim mechanisms, reproduced faithfully):
    1. controller computes tau_q (moment command) from (R, omega, reference)
    2. [f_cmd=m*g, tau_q] -> per-rotor omega_cmd via the SAME allocation-
       matrix pseudo-inverse Multirotor.force_and_torques_to_velocities uses
    3. omega_cmd -> actual rotor omega via the exact discrete first-order
       lag LaggedQuadraticThrustCurve implements (rotor_lambda from the
       MN4014+15x5" bench test, x650_params.ROTOR_LAMBDA)
    4. actual rotor omega -> actual [f, tau_x, tau_y, tau_z] via the
       allocation matrix's FORWARD direction (not the commanded values --
       the lag means these differ from the commanded [f_cmd, tau_q] until
       the rotors catch up), PLUS propeller parasitic drag torque and rotor
       gyroscopic torque from the same actual rotor omega
    5. Sigma_j: J @ dot(omega) = tau_actual_total - omega x (J @ omega)
"""
import numpy as np

from .x650_params_loader import load_x650_params
from ..aerodynamics.propeller_drag import (
    PropellerParasiticDrag, RotorGyroscopicTorque, K_H_PLACEHOLDER, J_ROTOR_PLACEHOLDER,
)

X650 = load_x650_params()


def skew(v):
    v = np.asarray(v).flatten()
    return np.array([[0., -v[2], v[1]],
                      [v[2], 0., -v[0]],
                      [-v[1], v[0], 0.]])


def project_to_SO3(R):
    U, _, Vt = np.linalg.svd(R)
    R_proj = U @ Vt
    if np.linalg.det(R_proj) < 0:
        U[:, -1] *= -1
        R_proj = U @ Vt
    return R_proj


# ── Allocation matrix (identical construction to Multirotor.force_and_torques_to_velocities) ──
def _build_allocation_matrix():
    A = np.zeros((4, X650.NUM_ROTORS))
    A[0, :] = X650.ROTOR_CONSTANT
    A[1, :] = X650.ROTOR_POSITIONS[:, 1] * X650.ROTOR_CONSTANT
    A[2, :] = -X650.ROTOR_POSITIONS[:, 0] * X650.ROTOR_CONSTANT
    A[3, :] = X650.ROLLING_MOMENT_COEFFICIENT * X650.ROT_DIR
    return A


ALLOC_MATRIX = _build_allocation_matrix()
ALLOC_MATRIX_PINV = np.linalg.pinv(ALLOC_MATRIX)
J = np.diag(X650.INERTIA_DIAG)
J_INV = np.linalg.inv(J)
HOVER_THRUST = X650.MASS * 9.81
HOVER_ROTOR_OMEGA = np.sqrt(HOVER_THRUST / (X650.NUM_ROTORS * X650.ROTOR_CONSTANT))

PARASITIC_DRAG = PropellerParasiticDrag(X650.ROTOR_POSITIONS, K_H_PLACEHOLDER)
GYRO_TORQUE = RotorGyroscopicTorque(X650.ROT_DIR, J_ROTOR_PLACEHOLDER)


def force_moment_to_rotor_omega_cmd(f, M):
    """Same normalize-then-saturate pseudo-inverse allocation
    Multirotor.force_and_torques_to_velocities uses."""
    squared = ALLOC_MATRIX_PINV @ np.array([f, M[0], M[1], M[2]])
    squared = np.maximum(squared, 0.0)
    max_sq = X650.MAX_ROTOR_VEL ** 2
    max_val = np.max(squared)
    if max_val >= max_sq:
        squared = squared / max(max_val / max_sq, 1.0)
    return np.sqrt(squared)


def initial_state():
    return {
        'R': np.eye(3),
        'omega': np.zeros(3),
        'rotor_omega': np.full(X650.NUM_ROTORS, HOVER_ROTOR_OMEGA),
    }


def rotor_lag_step(rotor_omega, rotor_omega_cmd, dt, rotor_lambda=X650.ROTOR_LAMBDA):
    """Exact discrete zero-order-hold solution -- identical formula to
    LaggedQuadraticThrustCurve.update(), reproduced here in plain numpy."""
    decay = np.exp(-rotor_lambda * dt)
    omega_next = rotor_omega_cmd + (rotor_omega - rotor_omega_cmd) * decay
    return np.clip(omega_next, X650.MIN_ROTOR_VEL, X650.MAX_ROTOR_VEL)


def actual_wrench_and_extra_torques(rotor_omega, omega_body):
    """From the ACTUAL (lagged) rotor speeds: [f,taux,tauy,tauz] via the
    allocation matrix's forward direction, plus the parasitic-drag and
    gyroscopic reaction torques from the same rotor speeds."""
    wrench = ALLOC_MATRIX @ (rotor_omega ** 2)
    _, _, tau_drag = PARASITIC_DRAG.compute(rotor_omega, v_body=np.zeros(3), omega_body=omega_body)
    tau_gyro = GYRO_TORQUE.compute(rotor_omega, omega_body)
    return wrench, tau_drag, tau_gyro


def rk4_step(state, tau_q_fixed, dt):
    """RK4 over (R, omega) only -- rotor_omega is propagated with its own
    EXACT lag update (not part of the RK4 quadrature; same zero-order-hold
    treatment LaggedQuadraticThrustCurve gives it in the real sim, where the
    thruster and the rigid-body physics step are both once-per-physics-step,
    not sub-stepped against each other). tau_q_fixed is the moment command
    computed once at the top of the outer step (zero-order-hold across
    whatever sub-steps this function takes), matching the zero-order-hold
    convention used throughout the companion Automatica2026Simulation repo.
    """
    omega_cmd = force_moment_to_rotor_omega_cmd(HOVER_THRUST, tau_q_fixed)
    rotor_omega_next = rotor_lag_step(state['rotor_omega'], omega_cmd, dt)

    def deriv(R, omega):
        wrench, tau_drag, tau_gyro = actual_wrench_and_extra_torques(rotor_omega_next, omega)
        tau_total = np.array(wrench[1:4]) + tau_drag + tau_gyro
        dot_omega = J_INV @ (tau_total - skew(omega) @ (J @ omega))
        dot_R = R @ skew(omega)
        return dot_R, dot_omega

    R, omega = state['R'], state['omega']
    k1R, k1w = deriv(R, omega)
    k2R, k2w = deriv(R + dt / 2 * k1R, omega + dt / 2 * k1w)
    k3R, k3w = deriv(R + dt / 2 * k2R, omega + dt / 2 * k2w)
    k4R, k4w = deriv(R + dt * k3R, omega + dt * k3w)

    R_next = R + (dt / 6) * (k1R + 2 * k2R + 2 * k3R + k4R)
    omega_next = omega + (dt / 6) * (k1w + 2 * k2w + 2 * k3w + k4w)

    return {'R': project_to_SO3(R_next), 'omega': omega_next, 'rotor_omega': rotor_omega_next}
