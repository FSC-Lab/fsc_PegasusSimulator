#!/usr/bin/env python
"""
| File: controller.py
| License: BSD-3-Clause
| Description: AGAS (Almost Global Asymptotic Stability) geometric SO(3)
|   attitude tracker, ported verbatim from
|   Automatica2026Simulation/quad_attitude_sim/controller.py (a separate,
|   unrelated project's validated implementation of the same control law --
|   support_document.tex's "Quadrotor Attitude Tracking" section, eq.
|   attitude_control). That project already found and fixed two notation
|   bugs in the source equations (independently confirmed both numerically
|   and, separately, by the paper's own author from the closed-loop error
|   dynamics) -- this is the corrected form, not the literal original.

    tau_q = -b_omega*omega_tilde - b_r*e_r
            - omega_tilde^x @ J @ omega_tilde + omega^x @ J @ omega
            - J @ (omega_tilde^x @ Rtilde^T @ omega_d - Rtilde^T @ dot_omega_d)
            + mu_R
    e_A   = omega_tilde + c2 * J^-1 @ e_r
    mu_R  = -delta_R^2 * e_A / (delta_R*||e_A|| + epsilon_R)
    Rtilde = R_d^T @ R,  omega_tilde = omega - Rtilde^T @ omega_d
    e_r   = sum_i skew(e_i) @ Rtilde @ e_i

Only the moment tau_q is computed here -- no thrust-magnitude/position loop
(see x650_attitude_sim/__init__.py: attitude-only by design, matching the
Automatica2026Simulation source's own scoping).
"""
import numpy as np

E = np.eye(3)


def skew(v):
    v = np.asarray(v).flatten()
    return np.array([[0., -v[2], v[1]],
                      [v[2], 0., -v[0]],
                      [-v[1], v[0], 0.]])


def e_r_from_Rtilde(Rtilde):
    return sum(skew(E[:, i]) @ Rtilde @ E[:, i] for i in range(3))


def compute_control(R, omega, ref, J, J_inv, gains):
    """
    R, omega : current X650 attitude/body-rate.
    ref      : {'R_d','omega_d','dot_omega_d'} from reference.py.
    gains    : dict(b_omega, b_r, delta_R, epsilon_R, c2).
    Returns (tau_q, info).
    """
    R_d, omega_d, dot_omega_d = ref['R_d'], ref['omega_d'], ref['dot_omega_d']
    Rtilde = R_d.T @ R
    e_r = e_r_from_Rtilde(Rtilde)
    omega_tilde = omega - Rtilde.T @ omega_d

    e_A = omega_tilde + gains['c2'] * (J_inv @ e_r)
    ea_norm = np.linalg.norm(e_A)
    mu_R = -gains['delta_R'] ** 2 * e_A / (gains['delta_R'] * ea_norm + gains['epsilon_R'])

    omega_tilde_x = skew(omega_tilde)
    omega_x = skew(omega)
    tau_q = (
        - gains['b_omega'] * omega_tilde
        - gains['b_r'] * e_r
        - omega_tilde_x @ (J @ omega_tilde)
        + omega_x @ (J @ omega)
        - J @ (omega_tilde_x @ (Rtilde.T @ omega_d) - Rtilde.T @ dot_omega_d)
        + mu_R
    )

    Psi_q = 0.5 * np.trace(np.eye(3) - Rtilde)
    info = {'Rtilde': Rtilde, 'e_r': e_r, 'omega_tilde': omega_tilde, 'Psi_q': Psi_q}
    return tau_q, info
