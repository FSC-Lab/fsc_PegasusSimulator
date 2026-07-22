#!/usr/bin/env python3
"""
controller.py — faithful Python port of the working MATLAB controller.

Direct, function-structured port of the MATLAB pipeline so the code mirrors the
reference exactly (which validated stable in simulation):

    dynamics(X, params)          <- dynamics.m   (M,C,g, M̃,C̃,g̃, T, J_y, ...)
    controller(X, dyn, ref, p)   <- controller.m (u1, u2, u3)
    tau = Tᵀ [u1·R0·e3 ; u2 ; u3]                <- closed_loop_dynamics.m

State vector (MATLAB layout, length 18+2n):
    X = [ r0(3); vec(R0)(9, col-major); q(n); v0(3); omega0(3); qdot(n) ]

The MATLAB ran as a single-process, zero-latency closed loop (controller and
plant share M,C,g). Here `dynamics`+`controller` are the controller; Isaac is
the plant, driven over ROS2 — so the arm-impedance loop still faces the loop
latency, but the control law itself is the validated MATLAB law verbatim.
"""

import os

import numpy as np
from dataclasses import dataclass, field

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    _HAS_ROS2 = True
except ImportError:
    _HAS_ROS2 = False


# ===========================================================================
# Math helpers
# ===========================================================================


E3 = np.array([0.0, 0.0, 1.0]).T


def hat(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def joint_rotation(h, q):
    """Elementary rotation about unit axis h by angle q (Rodrigues)."""
    a = h / (np.linalg.norm(h) + 1e-15)
    K = hat(a)
    return np.eye(3) + np.sin(q) * K + (1.0 - np.cos(q)) * (K @ K)


def quat_to_rot(w, x, y, z):
    n = np.sqrt(w*w + x*x + y*y + z*z) + 1e-15
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y+z*z),   2*(x*y-w*z),     2*(x*z+w*y)],
        [2*(x*y+w*z),   1 - 2*(x*x+z*z),     2*(y*z-w*x)],
        [2*(x*z-w*y),       2*(y*z+w*x), 1 - 2*(x*x+y*y)],
    ])


# ===========================================================================
# Parameters  (mirrors main_sim.m's params struct; gripper_bat values)
# ===========================================================================

def make_params():
    n = 4
    # Inertial parameters read from the realigned USD (2-AM mod/AM_realign.usda) MassAPI —
    # the convex-decomposition "approximation" mass/diagonalInertia PhysX baked per body.
    # Base link 0 = quadrotor body + 4 rotors (rigidly attached, so their mass adds to the
    # base); I0 keeps the body-only inertia (rotor spin-inertia omitted, as before).
    #   body : mass 2.4760795, diagInertia (0.06301228, 0.06334175, 0.09868092)
    #   rotor: 4 × 0.0398865
    m0 = 2.4760795 + 4 * 0.0398865                       # = 2.6356 kg (body + rotors)
    I0 = np.diag([0.06301228, 0.06334175, 0.09868092])   # body-only diagonalInertia

    # link2, link3, link4, manip_base. manip_base lumps the whole gripper mass:
    #   hub 0.0096343 + right_grip 0.02895701 + left_grip 0.02895701
    #   + left_link 0.01048464 + right_link 0.01048464 = 0.08852 kg.
    # Masses + diagonalInertia are straight from the USD (link CoM offset still approximated
    # at l_i/2 in the dynamics; refine there if the arm needs it).
    link_masses = [0.1048326, 0.1423463, 0.13992727, 0.161 + 0.08852]
    link_inertias = [
        np.diag([3.72000e-05, 3.74900e-05, 2.16100e-05]),   # link2
        np.diag([3.59370e-04, 3.48590e-04, 5.50600e-05]),   # link3
        np.diag([3.13020e-04, 3.25300e-05, 3.18470e-04]),   # link4
        np.diag([6.11100e-05, 9.35500e-05, 1.07070e-04]),   # manip_base
    ]
    # h_i^{i-1} : joint axes (body frame at q=0), l_i : O_{i-1}->O_i offsets.
    # MEASURED from the realigned USD (2-AM mod/AM_realign.usda), matching the method in
    # aerial_manipulator_ee_impedance.py _print_arm_joint_axes. At q=0 every link Xform is
    # identity-oriented w.r.t. the body frame and every joint localRot1 is identity, so the
    # body-frame axis is exactly the joint's physics:axis and O_i(body) = localPos0 mapped
    # into the body frame (O_i = localPos0 + t_child - t_body). The realign rotated the mount
    # 180° about z (x,y flip vs the old flatten model) and cleaned the joints to identity
    # localRot, so the axes are exact unit vectors: +z, +x, +x, +z.
    #
    # SIGN: these are +physics:axis. That MUST match how q flows — DC reports q about
    # +physics:axis and the plant publishes q/qd unflipped (JointStatePublisher) and applies
    # effort with ARM_TAU_SIGN=+1. Keep all three consistent; a sign mismatch here builds the
    # dynamics backwards and runs away the instant torque mode engages. Verify against the
    # [ARM-AXES] print on the next run: it should read [0,0,1] [1,0,0] [1,0,0] [0,0,1].
    h_i_im1 = [
        np.array([0.0, 0.0, 1.0]),   # joint 1: about +z
        np.array([1.0, 0.0, 0.0]),   # joint 2: about +x
        np.array([1.0, 0.0, 0.0]),   # joint 3: about +x
        np.array([0.0, 0.0, 1.0]),   # joint 4: about +z
    ]
    l_i = [
        np.array([ 0.0005,  0.002, -0.042]),   # body  -> joint1  (= O1 - O0)
        np.array([-0.019,   0.000, -0.041]),   # joint1-> joint2  (= O2 - O1)
        np.array([ 0.000,   0.024, -0.128]),   # joint2-> joint3  (= O3 - O2)
        np.array([ 0.020,   0.129, -0.022]),   # joint3-> joint4/EE (= O4 - O3)
    ]
    # reflected rotor (armature) inertia along each joint axis (rank-1)
    J_arm = 353.5 ** 2 * 1.6e-7
    I_i_i = [I0.copy()]
    for i in range(n):
        I_i_i.append(link_inertias[i] + J_arm * np.outer(h_i_im1[i], h_i_im1[i]))

    return {
        "n": n,
        "m_i": [m0] + link_masses,     # len n+1
        "l_i": l_i,                     # len n
        "h_i_im1": h_i_im1,             # len n
        "I_i_i": I_i_i,                 # len n+1
        "g": 9.81,
    }


# ===========================================================================
# dynamics(X, params)  — faithful port of dynamics.m  (sections A–K)
# ===========================================================================

def dynamics(X, params):
    n = params["n"]
    mi = params["m_i"]
    m_total = sum(mi)
    I_i_i = params["I_i_i"]
    l_i = params["l_i"]
    h_i_im1 = params["h_i_im1"]
    g_const = params["g"]

    r_0 = X[0:3]
    R_0 = X[3:12].reshape(3, 3, order="F")
    q = X[12:12 + n]
    omega_0 = X[15 + n:18 + n]
    qdot = X[18 + n:18 + 2 * n]

    # ---- A. rotation kinematics ----
    R_i_0 = [np.eye(3)]
    for i in range(n):
        R_i_0.append(R_i_0[i] @ joint_rotation(h_i_im1[i], q[i]))   # R_i^0 = R_{i-1}^0 R_i^{i-1}
    R_e_0 = R_i_0[n]
    I_i_0 = [R_i_0[i] @ I_i_i[i] @ R_i_0[i].T for i in range(n + 1)]
    h_i_0 = [np.zeros(3)]
    for i in range(1, n + 1):
        h_i_0.append(R_i_0[i - 1] @ h_i_im1[i - 1])

    # ---- B. translation kinematics ----
    O = [np.zeros(3)]                                  # r_{0,O_i}^0
    for i in range(1, n + 1):
        O.append(O[i - 1] + R_i_0[i] @ l_i[i - 1])
    r_0e_0 = O[n]
    r0i = [np.zeros(3)]                                # CoM r_{0i}^0
    for i in range(1, n + 1):
        r0i.append(O[i - 1] + R_i_0[i] @ (l_i[i - 1] / 2.0))
    r_0c_0 = sum(mi[i] * r0i[i] for i in range(n + 1)) / m_total

    A_i = [np.zeros((3, n)) for _ in range(n + 1)]     # dr_{0i}/dq
    for i in range(1, n + 1):
        for k in range(1, i + 1):
            A_i[i][:, k - 1] = np.cross(h_i_0[k], r0i[i] - O[k - 1])
    A_e = np.zeros((3, n))
    for k in range(1, n + 1):
        A_e[:, k - 1] = np.cross(h_i_0[k], r_0e_0 - O[k - 1])
    A = sum(mi[i] * A_i[i] for i in range(n + 1)) / m_total
    A_til = [A_i[i] - A for i in range(n + 1)]
    A_til_e = A_e - A
    d = [r0i[i] - r_0c_0 for i in range(n + 1)]
    d_e = r_0e_0 - r_0c_0

    # ---- C. rotation velocity-level ----
    h_dot = [np.zeros(3) for _ in range(n + 1)]
    for i in range(2, n + 1):
        for j in range(1, i):
            h_dot[i] = h_dot[i] + np.cross(h_i_0[j], h_i_0[i]) * qdot[j - 1]
    Jq = [np.zeros((3, n)) for _ in range(n + 1)]      # J_omega_i^q
    for i in range(1, n + 1):
        for k in range(1, i + 1):
            Jq[i][:, k - 1] = h_i_0[k]
    Jq_e = Jq[n]
    Jqd = [np.zeros((3, n)) for _ in range(n + 1)]     # d/dt J_omega_i^q
    for i in range(1, n + 1):
        for k in range(1, i + 1):
            Jqd[i][:, k - 1] = h_dot[k]
    Jqd_e = Jqd[n]
    w0i = [np.zeros(3)]                                # omega_{0i}^0
    for i in range(1, n + 1):
        w0i.append(Jq[i] @ qdot)
    w0e = w0i[n]
    wi = [omega_0] + [omega_0 + w0i[i] for i in range(1, n + 1)]   # omega_i^0
    rdotO = [np.zeros(3) for _ in range(n + 1)]        # r_dot_{0,O_i}^0
    for i in range(1, n + 1):
        for j in range(1, i + 1):
            rdotO[i] = rdotO[i] + np.cross(w0i[j], R_i_0[j] @ l_i[j - 1])

    # ---- D. simplified-notation derivatives ----
    rdot0i = [np.zeros(3)] + [A_i[i] @ qdot for i in range(1, n + 1)]
    A_dot_i = [np.zeros((3, n)) for _ in range(n + 1)]
    for i in range(1, n + 1):
        for k in range(1, i + 1):
            A_dot_i[i][:, k - 1] = (np.cross(h_dot[k], r0i[i] - O[k - 1])
                                    + np.cross(h_i_0[k], rdot0i[i] - rdotO[k - 1]))
    rdot0e = A_e @ qdot
    A_dot_e = np.zeros((3, n))
    for k in range(1, n + 1):
        A_dot_e[:, k - 1] = (np.cross(h_dot[k], r_0e_0 - O[k - 1])
                             + np.cross(h_i_0[k], rdot0e - rdotO[k - 1]))
    A_dot = sum(mi[i] * A_dot_i[i] for i in range(n + 1)) / m_total
    A_til_dot = [A_dot_i[i] - A_dot for i in range(n + 1)]
    A_til_dot_e = A_dot_e - A_dot
    d_dot = [A_til[i] @ qdot for i in range(n + 1)]
    d_dot_e = A_til_e @ qdot
    W0 = hat(omega_0)
    Xi = [hat(wi[i]) @ I_i_0[i] + I_i_0[i] @ W0 for i in range(n + 1)]
    I_dot_0 = [Xi[i] + Xi[i].T for i in range(n + 1)]

    # ---- E. transformed preliminaries: M_rho, B, N1, L, W (+ derivatives) ----
    M_rho = np.zeros((n, n)); B = np.zeros((n, 3))
    M_rho_dot = np.zeros((n, n)); B_dot = np.zeros((n, 3))
    for i in range(n + 1):
        M_rho += mi[i] * A_til[i].T @ A_til[i] + Jq[i].T @ I_i_0[i] @ Jq[i]
        B += -mi[i] * A_i[i].T @ hat(d[i]) + Jq[i].T @ I_i_0[i]
        M_rho_dot += (mi[i] * (A_til_dot[i].T @ A_til[i] + A_til[i].T @ A_til_dot[i])
                      + Jqd[i].T @ I_i_0[i] @ Jq[i] + Jq[i].T @ I_dot_0[i] @ Jq[i]
                      + Jq[i].T @ I_i_0[i] @ Jqd[i])
        B_dot += (-mi[i] * (A_til_dot[i].T @ hat(d[i]) + A_til[i].T @ hat(d_dot[i]))
                  + Jqd[i].T @ I_i_0[i] + Jq[i].T @ I_dot_0[i])
    N1 = np.linalg.solve(M_rho, B)
    N1_dot = np.linalg.solve(M_rho, B_dot - M_rho_dot @ N1)

    L = [-hat(d[i]) - A_til[i] @ N1 for i in range(n + 1)]
    L_e = -hat(d_e) - A_til_e @ N1
    L_dot = [-hat(d_dot[i]) - A_til_dot[i] @ N1 - A_til[i] @ N1_dot for i in range(n + 1)]
    L_dot_e = -hat(d_dot_e) - A_til_dot_e @ N1 - A_til_e @ N1_dot
    W = [np.eye(3) - Jq[i] @ N1 for i in range(n + 1)]
    W_e = np.eye(3) - Jq_e @ N1
    W_dot = [-Jqd[i] @ N1 - Jq[i] @ N1_dot for i in range(n + 1)]
    W_dot_e = -Jqd_e @ N1 - Jq_e @ N1_dot

    # ---- F. transformed whole-body: M̃ blocks, g̃, C̃ blocks ----
    M_t = m_total * np.eye(3)
    M_r = np.zeros((3, 3)); C_r = np.zeros((3, 3))
    C_rp = np.zeros((3, n)); C_p = np.zeros((n, n))
    for i in range(n + 1):
        M_r += mi[i] * L[i].T @ L[i] + W[i].T @ I_i_0[i] @ W[i]
        C_r += (mi[i] * L[i].T @ (W0 @ L[i] + L_dot[i])
                + W[i].T @ I_i_0[i] @ W_dot[i] + W[i].T @ Xi[i] @ W[i])
        C_rp += (mi[i] * L[i].T @ (W0 @ A_til[i] + A_til_dot[i])
                 + W[i].T @ I_i_0[i] @ Jqd[i] + W[i].T @ Xi[i] @ Jq[i])
        C_p += (mi[i] * A_til[i].T @ (W0 @ A_til[i] + A_til_dot[i])
                + Jq[i].T @ I_i_0[i] @ Jqd[i] + Jq[i].T @ Xi[i] @ Jq[i])
    g_tilde = np.concatenate([m_total * g_const * E3, np.zeros(3), np.zeros(n)])

    # ---- G. end-effector task Jacobian J_y, J̇_y, Λ_y, partition ----
    R_0e = R_e_0.T
    e3R0e = (E3 @ R_0e).reshape(1, 3)
    J_y = np.zeros((4, n + 6))
    J_y[0:3, 0:3] = np.eye(3); J_y[0:3, 3:6] = R_0 @ L_e; J_y[0:3, 6:] = R_0 @ A_til_e
    J_y[3:4, 3:6] = e3R0e @ W_e; J_y[3:4, 6:] = e3R0e @ Jq_e
    J_y_dot = np.zeros((4, n + 6))
    J_y_dot[0:3, 3:6] = R_0 @ (W0 @ L_e + L_dot_e)
    J_y_dot[0:3, 6:] = R_0 @ (W0 @ A_til_e + A_til_dot_e)
    W0e = hat(w0e)
    J_y_dot[3:4, 3:6] = e3R0e @ (W_dot_e - W0e @ W_e)
    J_y_dot[3:4, 6:] = e3R0e @ (Jqd_e - W0e @ Jq_e)

    M_tilde = np.zeros((n + 6, n + 6))
    M_tilde[0:3, 0:3] = M_t; M_tilde[3:6, 3:6] = M_r; M_tilde[6:, 6:] = M_rho
    Lambda_y = np.linalg.inv(J_y @ np.linalg.solve(M_tilde, J_y.T))
    J_y_pinv = np.linalg.solve(M_tilde, J_y.T) @ Lambda_y
    J_y_pinv_T = J_y_pinv.T
    J_1y = J_y_pinv_T[:, 0:3]; J_2y = J_y_pinv_T[:, 3:6]; J_3y = J_y_pinv_T[:, 6:]

    # ---- H. transformed-state map T ----
    T = np.zeros((n + 6, n + 6))
    T[0:3, 0:3] = R_0; T[0:3, 3:6] = -R_0 @ hat(r_0c_0); T[0:3, 6:] = R_0 @ A
    T[3:6, 3:6] = np.eye(3); T[6:, 3:6] = N1; T[6:, 6:] = np.eye(n)

    # ---- I. original (untransformed) dynamics M, C, g  (the plant) ----
    M = np.zeros((n + 6, n + 6)); C = np.zeros((n + 6, n + 6)); g = np.zeros(n + 6)
    for i in range(n + 1):
        Jvi_p = np.zeros((3, n + 6))
        Jvi_p[:, 0:3] = np.eye(3); Jvi_p[:, 3:6] = -hat(r0i[i]); Jvi_p[:, 6:] = A_i[i]
        Jvi = R_0 @ Jvi_p
        Jvi_p_dot = np.zeros((3, n + 6))
        Jvi_p_dot[:, 0:3] = W0
        Jvi_p_dot[:, 3:6] = -(W0 @ hat(r0i[i]) + hat(rdot0i[i]))
        Jvi_p_dot[:, 6:] = W0 @ A_i[i] + A_dot_i[i]
        Jwi = np.zeros((3, n + 6)); Jwi[:, 3:6] = np.eye(3); Jwi[:, 6:] = Jq[i]
        Jwi_dot = np.zeros((3, n + 6)); Jwi_dot[:, 6:] = Jqd[i]
        M += mi[i] * (Jvi_p.T @ Jvi_p) + Jwi.T @ I_i_0[i] @ Jwi
        C += mi[i] * (Jvi_p.T @ Jvi_p_dot) + Jwi.T @ I_i_0[i] @ Jwi_dot + Jwi.T @ Xi[i] @ Jwi
        g += Jvi.T @ (mi[i] * g_const * E3)

    return {
        "M_r": M_r, "C_r": C_r, "C_rp": C_rp, "C_p": C_p,
        "M_tilde": M_tilde, "g_tilde": g_tilde, "T": T,
        "M": M, "C": C, "g": g,
        "A": A, "N1": N1, "r_0c_0": r_0c_0, "r_0e_0": r_0e_0, "R_e_0": R_e_0,
        "J_y": J_y, "J_y_dot": J_y_dot, "Lambda_y": Lambda_y,
        "J_1y": J_1y, "J_2y": J_2y, "J_3y": J_3y,
        "J_q_omega_e": Jq_e, "J_q_dot_omega_e": Jqd_e, "omega_0e_0": w0e,
    }


# ===========================================================================
# generate_reference  — port of generate_reference.m (trajectory library)
# ===========================================================================
# Choose the trajectory here (mirrors traj_config() in generate_reference.m).
# The drone first climbs to TAKEOFF_ALTITUDE and settles, then the selected
# trajectory runs, anchored at [spawn_xy, TAKEOFF_ALTITUDE].
TRAJ_TYPE = "hover"     # "hover" | "line" | "sine" | "circle" | "poly"
TRAJ_CONFIG = {
    "hover":  {"T": 8.0},
    "line":   {"axis": "xy", "D": 5.0, "T": 10.0},
    "sine":   {"axis": "xy", "A": 0.5, "Ncyc": 2, "T": 12.0},
    "circle": {"r": 3.0, "T": 20.0},
    "poly":   {"D": 2.0, "A": 0.2, "T": 12.0},
}
TAKEOFF_ALTITUDE = 1.5     # [m] climb to this before the trajectory
TAKEOFF_TIME     = 6.0     # [s] climb + settle window before trajectory starts
# The arm is ALWAYS driven by the full EE-impedance torque (tau_joint = tau[6:]).
# There is no arm-lock switch: whether the arm holds still or moves is decided by
# the commanded EE reference (the trajectory), not by a mode flag. The plant
# applies these torques as direct joint efforts.
#
# Whole-body coupling is kept (tau_body carries the arm reaction N1ᵀ·u3), but made
# SATURATION-CONSISTENT: the joint torque is clamped to the real servo limit before
# it feeds the base moment, so the attitude channel never commands a reaction the
# arm cannot actually produce. TAU_MAX MUST match ARM_MAX_FORCE in the Isaac plant
# (aerial_manipulator_ee_impedance.py) — the XM430-W350 stall torque.
TAU_MAX = 4.1              # [N·m] per-joint servo saturation (XM430-W350 stall @12V)
# PLATFORM_TUNE: isolate the quadrotor for gain tuning. When True, the arm is a
# rigid fixed payload — command ZERO joint torque and drop the arm coupling from the
# base moment (u3 = 0), so the base is a clean quadrotor carrying the arm's fixed
# mass. Tune k_x/k_v/k_R/k_w against this, THEN set False for full whole-body.
# MUST match PLATFORM_TUNE in the Isaac plant (it position-holds the arm when True).
PLATFORM_TUNE = True
# Startup sequencing (when PLATFORM_TUNE=False): the plant holds the arm RIGID
# through takeoff+settle, then releases it to torque control. The controller keeps
# the arm channel OFF (u3=0, zero joint torque) until ARM_ACTIVATE_TIME so it hands
# over only once the drone hovers stably and the arm is at its reference (small e_y)
# — this avoids the initial saturation slam. No change to the impedance law; only
# WHEN it engages. Keep ≥ the plant's ARM_RELEASE_DELAY so torque is ready on release.
ARM_ACTIVATE_TIME = TAKEOFF_TIME + 3.0    # [s after control start] hold, then whole-body
# Attitude feedforward (analytic ω̇0,c) — off for ROS2 (it amplifies the laggy
# body rate into an attitude limit cycle; the lock mode that flew had it off).
ATTITUDE_FEEDFORWARD = False


def _axis_dir(s):
    table = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1],
             "xy": [1, 1, 0], "xz": [1, 0, 1], "yz": [0, 1, 1], "xyz": [1, 1, 1]}
    e = np.array(table[s.lower()], float)
    return e / np.linalg.norm(e)


def _minjerk(tau):
    """Rest-to-rest min-jerk profile s and derivatives ds..d4s (wrt tau)."""
    if tau <= 0.0:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    if tau >= 1.0:
        return 1.0, 0.0, 0.0, 0.0, 0.0
    s = 10*tau**3 - 15*tau**4 + 6*tau**5
    ds = 30*tau**2 - 60*tau**3 + 30*tau**4
    d2s = 60*tau - 180*tau**2 + 120*tau**3
    d3s = 60 - 360*tau + 360*tau**2
    d4s = -360 + 720*tau
    return s, ds, d2s, d3s, d4s


def build_traj(x_c0, d0, b1_0, traj_type=None):
    """Combine the selected config with the initial anchors (port of build_traj)."""
    ttype = traj_type or TRAJ_TYPE
    cfg = TRAJ_CONFIG[ttype]
    tr = {"type": ttype, "x_c0": np.array(x_c0, float),
          "d0": np.array(d0, float), "b1": np.array(b1_0, float), "T": cfg["T"]}
    if ttype == "line":
        tr["dir"] = _axis_dir(cfg["axis"]); tr["D"] = cfg["D"]
    elif ttype == "sine":
        tr["e"] = _axis_dir(cfg["axis"]); tr["A"] = cfg["A"]; tr["Ncyc"] = cfg["Ncyc"]
    elif ttype == "circle":
        tr["r"] = cfg["r"]
        hxy = np.array([b1_0[0], b1_0[1], 0.0])
        if np.linalg.norm(hxy) < 1e-9:
            tr["theta0"] = 0.0
        else:
            hxy /= np.linalg.norm(hxy)
            tr["theta0"] = np.arctan2(-hxy[0], hxy[1])
        c = tr["x_c0"] - tr["r"] * np.array([np.cos(tr["theta0"]), np.sin(tr["theta0"]), 0.0])
        c[2] = tr["x_c0"][2]
        tr["c"] = c
    elif ttype == "poly":
        tr["D"] = cfg["D"]; tr["A"] = cfg["A"]
        ef = np.array([b1_0[0], b1_0[1], 0.0])
        ef = ef / np.linalg.norm(ef) if np.linalg.norm(ef) > 1e-9 else np.array([1.0, 0, 0])
        tr["e_fwd"] = ef; tr["e_lat"] = np.array([-ef[1], ef[0], 0.0])
    elif ttype != "hover":
        raise ValueError(f"unsupported trajectory type {ttype}")
    return tr


def _fixed_ee(ref, tr):
    ref["r_ed"] = ref["x_cd"] + tr["d0"]
    ref["r_ed_dot"] = ref["x_cd_dot"]; ref["r_ed_ddot"] = ref["x_cd_ddot"]
    z3 = np.zeros(3)
    ref["b1_d"] = tr["b1"]; ref["b1_d_dot"] = z3; ref["b1_d_ddot"] = z3
    ref["b1_de"] = tr["b1"]; ref["b1_de_dot"] = z3; ref["b1_de_ddot"] = z3
    return ref


def generate_reference(t, tr):
    """Reference at time t for the built trajectory tr (port of generate_reference)."""
    z3 = np.zeros(3)
    ttype = tr["type"]
    if ttype == "hover":
        ref = {"x_cd": tr["x_c0"].copy(), "x_cd_dot": z3, "x_cd_ddot": z3,
               "x_cd_d3": z3, "x_cd_d4": z3}
        return _fixed_ee(ref, tr)

    if ttype == "line":
        T, D, e = tr["T"], tr["D"], tr["dir"]
        s, ds, d2s, d3s, d4s = _minjerk(t / T)
        ref = {"x_cd": tr["x_c0"] + D*s*e, "x_cd_dot": D*ds/T*e,
               "x_cd_ddot": D*d2s/T**2*e, "x_cd_d3": D*d3s/T**3*e, "x_cd_d4": D*d4s/T**4*e}
        return _fixed_ee(ref, tr)

    if ttype == "sine":
        T, A, e, K = tr["T"], tr["A"], tr["e"], 2*np.pi*tr["Ncyc"]
        s, ds, d2s, d3s, d4s = _minjerk(t / T)
        p = K*s; p1 = K*ds/T; p2 = K*d2s/T**2; p3 = K*d3s/T**3; p4 = K*d4s/T**4
        sp, cp = np.sin(p), np.cos(p)
        f0 = sp; f1 = cp*p1; f2 = -sp*p1**2 + cp*p2
        f3 = -cp*p1**3 - 3*sp*p1*p2 + cp*p3
        f4 = sp*p1**4 - 6*cp*p1**2*p2 - 3*sp*p2**2 - 4*sp*p1*p3 + cp*p4
        ref = {"x_cd": tr["x_c0"] + A*f0*e, "x_cd_dot": A*f1*e, "x_cd_ddot": A*f2*e,
               "x_cd_d3": A*f3*e, "x_cd_d4": A*f4*e}
        return _fixed_ee(ref, tr)

    if ttype == "circle":
        r, c, T, th0 = tr["r"], tr["c"], tr["T"], tr["theta0"]
        s, ds, d2s, d3s, d4s = _minjerk(t / T)
        th = th0 + 2*np.pi*s
        a = 2*np.pi*ds/T; b = 2*np.pi*d2s/T**2; cc = 2*np.pi*d3s/T**3; dd = 2*np.pi*d4s/T**4
        ct, st = np.cos(th), np.sin(th)
        pth1 = r*np.array([-st, ct, 0.0]); pth2 = r*np.array([-ct, -st, 0.0])
        pth3 = r*np.array([st, -ct, 0.0]); pth4 = r*np.array([ct, st, 0.0])
        ref = {"x_cd": c + r*np.array([ct, st, 0.0]),
               "x_cd_dot": pth1*a, "x_cd_ddot": pth2*a**2 + pth1*b,
               "x_cd_d3": pth3*a**3 + 3*pth2*a*b + pth1*cc,
               "x_cd_d4": pth4*a**4 + 6*pth3*a**2*b + pth2*(3*b**2 + 4*a*cc) + pth1*dd}
        # EE offset carried with the base yaw; headings follow the tangent
        phi = th - th0; cph, sph = np.cos(phi), np.sin(phi)
        Sz = np.array([[0, -1, 0.], [1, 0, 0], [0, 0, 0]])
        Rz = np.array([[cph, -sph, 0.], [sph, cph, 0], [0, 0, 1]])
        d_off = Rz @ tr["d0"]
        ref["r_ed"] = ref["x_cd"] + d_off
        ref["r_ed_dot"] = ref["x_cd_dot"] + a*(Sz @ d_off)
        ref["r_ed_ddot"] = ref["x_cd_ddot"] + b*(Sz @ d_off) + a**2*(Sz @ Sz @ d_off)
        b1 = np.array([-st, ct, 0.0]); db1 = np.array([-ct, -st, 0.0]); d2b1 = np.array([st, -ct, 0.0])
        ref["b1_d"] = b1; ref["b1_d_dot"] = db1*a; ref["b1_d_ddot"] = d2b1*a**2 + db1*b
        ref["b1_de"] = b1; ref["b1_de_dot"] = db1*a; ref["b1_de_ddot"] = d2b1*a**2 + db1*b
        return ref

    if ttype == "poly":
        T, D, A = tr["T"], tr["D"], tr["A"]
        ef, el = tr["e_fwd"], tr["e_lat"]
        s, ds, d2s, d3s, d4s = _minjerk(t / T)
        sig = s; sd = ds/T; sdd = d2s/T**2; s3 = d3s/T**3; s4 = d4s/T**4
        X = D*sig; Xp = D
        Y = A*(3*sig**2 - 2*sig**3); Yp = A*(6*sig - 6*sig**2); Ypp = A*(6 - 12*sig); Yppp = -12*A
        P = tr["x_c0"] + X*ef + Y*el
        P1 = Xp*ef + Yp*el; P2 = Ypp*el; P3 = Yppp*el
        ref = {"x_cd": P, "x_cd_dot": P1*sd, "x_cd_ddot": P2*sd**2 + P1*sdd,
               "x_cd_d3": P3*sd**3 + 3*P2*sd*sdd + P1*s3,
               "x_cd_d4": 6*P3*sd**2*sdd + P2*(3*sdd**2 + 4*sd*s3) + P1*s4}
        return _fixed_ee(ref, tr)

    raise ValueError(f"unknown trajectory type {ttype}")


# ===========================================================================
# controller(X, dyn, ref, params)  — faithful port of controller.m
# ===========================================================================

@dataclass
class Gains:
    k_x: float = 10.0   # position stiffness — MATLAB-faithful; k_x≫10 poisons attitude
    k_v: float = 5.0    # (outer loop too fast for the inner attitude loop → large e_R)
    k_R: float = 8.0    # attitude stiffness (MATLAB value)
    k_w: float = 2.0    # angular-rate gain (MATLAB value; k_w=4 amplified laggy body rate → worse)
    # ISAAC ADAPTATION: the MATLAB uses M_r_d = eye(3), which gives a weak
    # attitude loop (u2 ≈ M̃_r·kR·e_R ≈ 0.09 N·m) that holds only with the
    # MATLAB's zero latency + perfect model. Over the ROS2 loop it tips over.
    # I0_body restores the strong attitude authority that flew in lock mode.
    M_r_d: np.ndarray = field(default_factory=lambda: np.diag([0.065194, 0.064352, 0.100439]))
    # Task-space (EE) impedance. Soft (K_y = 4) is the LATENCY-STABLE value: it
    # hovers, but the arm sags under its own weight (near-zero holding torque). A
    # stiff K_y (tried 50) destabilizes the EE-impedance loop over the ROS2 DDS
    # latency → flip. So stiffness is capped by the loop latency, NOT by the law;
    # holding the arm firmly needs arm gravity-feedforward at soft gains, or the
    # zero-latency in-process path — not brute stiffness. Kept soft here.
    K_y: np.ndarray = field(default_factory=lambda: 4.0 * np.eye(4))
    D_y: np.ndarray = field(default_factory=lambda: 4.0 * np.eye(4))  # (D_y=10 amplified laggy task-vel → worse)


class MatlabController:
    """Holds the omega_e_dot finite-difference memory (MATLAB `persistent`)."""

    def __init__(self, params, gains=None, attitude_feedforward=True):
        self.p = params
        self.g = gains or Gains()
        self.attitude_feedforward = attitude_feedforward
        # Arm-channel gate. False → arm decoupled + zero joint torque (base is a clean
        # quadrotor). Set by the node: False during platform-tune / takeoff hold, True
        # for full whole-body once the arm is released. The impedance law is unchanged;
        # this only gates WHEN it engages.
        self.arm_active = False
        self._qdot_prev = None
        self._omega0_prev = None

    def __call__(self, X, dyn, ref, dt):
        g = self.g; p = self.p; n = p["n"]; m = sum(p["m_i"]); gg = p["g"]
        r_0 = X[0:3]; R_0 = X[3:12].reshape(3, 3, order="F")
        omega_0 = X[15 + n:18 + n]; qdot = X[18 + n:18 + 2 * n]
        v_0 = X[12 + n:15 + n]
        V = np.concatenate([v_0, omega_0, qdot])
        om_hat = hat(omega_0); e3 = E3

        A = dyn["A"]; N1 = dyn["N1"]
        r_0c_0 = dyn["r_0c_0"]; r_0e_0 = dyn["r_0e_0"]; R_e_0 = dyn["R_e_0"]
        M_r = dyn["M_r"]; C_r = dyn["C_r"]; C_rp = dyn["C_rp"]; C_p = dyn["C_p"]
        J_y = dyn["J_y"]; J_y_dot = dyn["J_y_dot"]; Lambda_y = dyn["Lambda_y"]
        J_1y = dyn["J_1y"]; J_2y = dyn["J_2y"]; J_3y = dyn["J_3y"]
        omega_0e_0 = dyn["omega_0e_0"]; J_we = dyn["J_q_omega_e"]; J_we_dot = dyn["J_q_dot_omega_e"]
        T = dyn["T"]

        xi = T @ V
        xc_dot = xi[0:3]; rho = xi[6:6 + n]

        # ---- translation ----
        x_c = r_0 + R_0 @ r_0c_0
        e_x = x_c - ref["x_cd"]
        e_vx = xc_dot - ref["x_cd_dot"]
        f_d = -g.k_x * e_x - g.k_v * e_vx + m * ref["x_cd_ddot"] + m * gg * e3
        u1 = float(f_d.T @ (R_0 @ e3))
        xc_ddot = (u1 / m) * (R_0 @ e3) - gg * e3
        e_ax = xc_ddot - ref["x_cd_ddot"]
        f_d_dot = -g.k_x * e_vx - g.k_v * e_ax + m * ref["x_cd_d3"]
        u1_dot = float(f_d_dot.T @ (R_0 @ e3) + f_d.T @ (R_0 @ om_hat @ e3))
        xc_d3 = (u1_dot / m) * (R_0 @ e3) + (u1 / m) * (R_0 @ om_hat @ e3)
        e_jx = xc_d3 - ref["x_cd_d3"]
        f_d_ddot = -g.k_x * e_ax - g.k_v * e_jx + m * ref["x_cd_d4"]

        # ---- rotation (geometric, 2-level feedforward) ----
        nf = np.linalg.norm(f_d)
        b3c = f_d / nf
        P1 = np.eye(3) - np.outer(b3c, b3c)
        s = P1 @ ref["b1_d"]
        b3c_dot = (1.0 / nf) * P1 @ f_d_dot
        ns = np.linalg.norm(s)
        b1c = s / ns
        P1_dot = -(np.outer(b3c_dot, b3c) + np.outer(b3c, b3c_dot))
        P2 = np.eye(3) - np.outer(b1c, b1c)
        s_dot = P1_dot @ ref["b1_d"] + P1 @ ref["b1_d_dot"]
        b1c_dot = (1.0 / ns) * P2 @ s_dot
        P2_dot = -(np.outer(b1c_dot, b1c) + np.outer(b1c, b1c_dot))
        b2c = np.cross(b3c, b1c)
        R0c = np.column_stack([b1c, b2c, b3c])
        b2c_dot = np.cross(b3c_dot, b1c) + np.cross(b3c, b1c_dot)
        R0c_dot = np.column_stack([b1c_dot, b2c_dot, b3c_dot])
        omega_0c = vee(R0c.T @ R0c_dot)

        b3c_ddot = ((1.0 / nf) * (P1_dot @ f_d_dot + P1 @ f_d_ddot)
                    - (b3c.T @ f_d_dot) / nf**2 * (P1 @ f_d_dot))
        P1_ddot = -(np.outer(b3c_ddot, b3c) + 2 * np.outer(b3c_dot, b3c_dot) + np.outer(b3c, b3c_ddot))
        s_ddot = P1_ddot @ ref["b1_d"] + 2 * P1_dot @ ref["b1_d_dot"] + P1 @ ref["b1_d_ddot"]
        b1c_ddot = (1.0 / ns) * (P2_dot @ s_dot + P2 @ s_ddot) - (b1c.T @ s_dot) / ns * b1c_dot
        b2c_ddot = (np.cross(b3c_ddot, b1c) + 2 * np.cross(b3c_dot, b1c_dot) + np.cross(b3c, b1c_ddot))
        R0c_ddot = np.column_stack([b1c_ddot, b2c_ddot, b3c_ddot])
        omega_0c_dot = vee(R0c_dot.T @ R0c_dot + R0c.T @ R0c_ddot)

        # Attitude feedforward (ω0c, ω̇0c) amplifies the noisy/laggy body rate
        # over the ROS2 loop (f_d_ddot → ω̇0c is driven by ω0) → a fast attitude
        # limit cycle. Disable it for ROS2 (matches the lock mode that flew);
        # keep it only on the zero-latency path.
        # if not self.attitude_feedforward:
        #     omega_0c = np.zeros(3); omega_0c_dot = np.zeros(3)
        e_R = 0.5 * vee(R0c.T @ R_0 - R_0.T @ R0c)
        e_w = omega_0 - R_0.T @ R0c @ omega_0c
        u2 = (M_r @ (R_0.T @ R0c @ omega_0c_dot - om_hat @ R_0.T @ R0c @ omega_0c
                     - np.linalg.solve(g.M_r_d, g.k_R * e_R + g.k_w * e_w))
              + C_r @ omega_0 + C_rp @ rho)

        # ---- arm ----
        R_e = R_0 @ R_e_0
        R_0e = R_e_0.T
        r_e = r_0 + R_0 @ r_0e_0
        omega_e = R_0e @ (omega_0 + omega_0e_0)
        omega_e_hat = hat(omega_e)

        # omega_e_dot feedforward — prefer the node's filtered, data-rate derivative
        # estimate (ext_qddot / ext_omega0_dot). Fall back to a fixed-dt finite
        # difference for standalone calls that don't set them. Law unchanged.
        qddot = getattr(self, "ext_qddot", None)
        omega_0_dot = getattr(self, "ext_omega0_dot", None)
        if qddot is None or omega_0_dot is None:
            if self._qdot_prev is None or dt <= 0:
                qddot = np.zeros(n); omega_0_dot = np.zeros(3)
            else:
                qddot = (qdot - self._qdot_prev) / dt
                omega_0_dot = (omega_0 - self._omega0_prev) / dt
            self._qdot_prev = qdot.copy(); self._omega0_prev = omega_0.copy()

        omega_e_dot = R_0e @ (omega_0_dot + J_we @ qddot + J_we_dot @ qdot + om_hat @ omega_0e_0)
        omega_e_dot_hat = hat(omega_e_dot)

        b2e = R_e @ np.array([0.0, 1, 0]); b3e = R_e @ e3
        ydot = J_y @ xi
        r_e_dot = ydot[0:3]
        omega_3e = float(omega_e.T @ e3)

        P1e = np.eye(3) - np.outer(b3e, b3e)
        s_e = P1e @ ref["b1_de"]
        b3e_dot = R_e @ omega_e_hat @ e3
        nse = np.linalg.norm(s_e)
        b1ec = s_e / nse
        P1e_dot = -(np.outer(b3e_dot, b3e) + np.outer(b3e, b3e_dot))
        P2e = np.eye(3) - np.outer(b1ec, b1ec)
        s_e_dot = P1e_dot @ ref["b1_de"] + P1e @ ref["b1_de_dot"]
        b1ec_dot = (1.0 / nse) * P2e @ s_e_dot
        P2e_dot = -(np.outer(b1ec_dot, b1ec) + np.outer(b1ec, b1ec_dot))

        b3e_ddot = R_e @ (omega_e_hat @ omega_e_hat + omega_e_dot_hat) @ e3
        P1e_ddot = -(np.outer(b3e_ddot, b3e) + 2 * np.outer(b3e_dot, b3e_dot) + np.outer(b3e, b3e_ddot))
        s_e_ddot = P1e_ddot @ ref["b1_de"] + 2 * P1e_dot @ ref["b1_de_dot"] + P1e @ ref["b1_de_ddot"]
        b1ec_ddot = (1.0 / nse) * (P2e_dot @ s_e_dot + P2e @ s_e_ddot) - (b1ec.T @ s_e_dot) / nse * b1ec_dot

        omega_3ec = float(b3e.T @ np.cross(b1ec, b1ec_dot))
        omega_3ec_dot = float(b3e_dot.T @ np.cross(b1ec, b1ec_dot)
                              + np.cross(b3e, b1ec).T @ b1ec_ddot)

        e_xE = r_e - ref["r_ed"]
        e_RE3 = float(-(b1ec.T @ b2e))
        e_y = np.array([e_xE[0], e_xE[1], e_xE[2], e_RE3])
        e_vE = r_e_dot - ref["r_ed_dot"]
        e_wE3 = omega_3e - omega_3ec
        e_vy = np.array([e_vE[0], e_vE[1], e_vE[2], e_wE3])
        yddot_d = np.array([ref["r_ed_ddot"][0], ref["r_ed_ddot"][1],
                            ref["r_ed_ddot"][2], omega_3ec_dot])

        F_trans = u1 * (R_0 @ e3) - m * gg * e3
        tau_rot = u2 - C_r @ omega_0 - C_rp @ rho
        # stacked = np.concatenate([F_trans, tau_rot], axis=0)
        inner = (J_1y @ F_trans + J_2y @ tau_rot
                 + Lambda_y @ (J_y_dot @ xi - yddot_d)
                 + g.D_y @ e_vy + g.K_y @ e_y)
        u3 = -np.linalg.solve(J_3y, inner) - C_rp.T @ omega_0 + C_p @ rho

        # Whole-body COUPLED wrench, made servo-saturation consistent.
        #
        # Physical wrench  τ = Tᵀ·[u1·R0·e3 ; u2 ; u3]  (as in MATLAB
        # closed_loop_dynamics). Its blocks are:
        #     τ_joint = τ[6:]  = u3 + u1·Aᵀ·e3            (what the servos apply)
        #     τ_body  = τ[3:6] = u2 + u1·hat(r0c)·e3 + N1ᵀ·u3   (base moment; the
        #               N1ᵀ·u3 term is the arm's reaction on the airframe = coupling)
        #
        # In MATLAB τ_joint is applied exactly, so N1ᵀ·u3 in the base moment is always
        # consistent. On real hardware the XM430-W350 saturates at ±TAU_MAX: the demanded
        # u3 is NOT what the arm actually produces. Feeding the base moment the demanded
        # u3 while the servo clamps the joint torque commands an attitude reaction the arm
        # never delivers → inconsistency → flip (what we saw).
        #
        # Fix: clamp τ_joint to the servo limit, back out the REALIZED u3 from
        # τ_joint = u3 + u1·Aᵀ·e3, and rebuild the base moment with that realized u3.
        # Attitude and arm then stay physically consistent while remaining fully coupled.
        if not self.arm_active:
            # Arm channel OFF (platform-tune, or takeoff hold before release): the arm
            # is a rigid fixed payload (plant position-holds it). Command ZERO joint
            # torque and drop the arm coupling from the base moment (u3 = 0) → the base
            # is a clean quadrotor carrying the arm's fixed mass.
            u_body = np.concatenate([u1 * (R_0 @ e3), u2, np.zeros(n)])
            tau_body = (T.T @ u_body)[3:6]
            tau_joint = np.zeros(n)
        else:
            u = np.concatenate([u1 * (R_0 @ e3), u2, u3])
            # Arm GRAVITY COMPENSATION feedforward (additive; impedance law untouched).
            # The arm channel is pure impedance and would otherwise hold the arm only by
            # SAGGING until K_y·e_y == gravity torque. dyn["g"][6:] is the model joint
            # gravity torque (EOM: M·V̇ + C·V + g = τ, so τ_static = g). Adding it lets the
            # arm hold at ~zero error at soft K_y → small u3 → small coupling → the base
            # isn't dragged. Only the KNOWN self-weight is cancelled; an external payload
            # is still absorbed by K_y/D_y (the intended impedance behaviour is preserved).
            tau_joint = (T.T @ u)[6:] + dyn["g"][6:]
            tau_joint = np.clip(tau_joint, -TAU_MAX, TAU_MAX)          # servo saturation
            u3_real = tau_joint - u1 * (A.T @ e3)                       # realized u3
            u_real = np.concatenate([u1 * (R_0 @ e3), u2, u3_real])
            tau_body = (T.T @ u_real)[3:6]                              # coupled + consistent
        return {
            "u1": u1, "u2": u2, "u3": u3,
            "thrust": u1, "tau_body": tau_body, "tau_joint": tau_joint,
            "R0c": R0c, "e_R": e_R, "e_y": e_y, "e_x": e_x,
            # compatibility diagnostics (EE/CoM actual vs reference)
            "r_e": r_e, "r_ed": ref["r_ed"], "x_c": x_c, "x_cd": ref["x_cd"],
        }


# ===========================================================================
# Rotor mixer (Isaac-specific; not in the MATLAB plant)
# ===========================================================================

@dataclass
class RotorMixerParams:
    # Body-frame rotor positions, channel i → USD /rotor{i}. MUST match the live
    # USD (aerial_manipulator_ee_impedance.py prints them as [ROTORS] at startup).
    # The 2-AM "realign" model rotated the body 180° about z, so every rotor's
    # (x,y) is the NEGATIVE of the pre-realign layout — using the old values
    # inverts the roll/pitch allocation and flips the drone on takeoff.
    rotor_positions: np.ndarray = field(default_factory=lambda: np.array([
        [ 0.229907,  0.229907, 0.182191],   # ch0 → rotor0
        [-0.229907, -0.229907, 0.182191],   # ch1 → rotor1
        [ 0.229907, -0.229907, 0.182191],   # ch2 → rotor2
        [-0.229907,  0.229907, 0.182191]])) # ch3 → rotor3
    rot_dir: np.ndarray = field(default_factory=lambda: np.array([-1., -1., 1., 1.]))
    k_thrust: float = 1.03e-5
    k_torque: float = 1.0e-6


class RotorMixer:
    def __init__(self, p):
        self.p = p
        B = np.zeros((4, len(p.rotor_positions)))
        for i in range(len(p.rotor_positions)):
            r = p.rotor_positions[i]
            B[0, i] = 1.0; B[1, i] = r[1]; B[2, i] = -r[0]
            B[3, i] = p.rot_dir[i] * p.k_torque / p.k_thrust
        self._B_inv = np.linalg.pinv(B)

    def mix(self, thrust, tau_body):
        F = np.maximum(self._B_inv @ np.array([thrust, *tau_body]), 0.0)
        return np.sqrt(F / (self.p.k_thrust + 1e-12))


# Must match PX4MavlinkBackendConfig defaults used by
# x650_rotorcraft_utils.spawn_rotorcraft_with_mavlink(). PX4's normalized
# ActuatorMotors value u becomes omega = u * scaling + armed_idle in Pegasus.
PX4_ROTOR_INPUT_SCALING = 1000.0
PX4_ROTOR_ARMED_IDLE = 100.0
PX4_OFFBOARD_WARMUP_SETPOINTS = 250  # 1 s at the controller's 250 Hz timer
PX4_COMMAND_RETRY_INTERVAL = 100


# ===========================================================================
# ROS 2 node
# ===========================================================================

if _HAS_ROS2:
    class MatlabControllerNode(Node):
        CLIMB_RATE = 0.4     # [m/s] takeoff altitude ramp

        def __init__(self):
            super().__init__("matlab_aerial_manipulator_controller")
            self._control_mode = os.environ.get("AERIAL_MANIPULATOR_CONTROL_MODE", "direct")
            if self._control_mode not in {"direct", "px4_offboard"}:
                raise ValueError(
                    "AERIAL_MANIPULATOR_CONTROL_MODE must be 'direct' or 'px4_offboard', "
                    f"got {self._control_mode!r}"
                )
            self._px4_offboard = self._control_mode == "px4_offboard"
            self._px4_auto_arm = os.environ.get("AERIAL_MANIPULATOR_AUTO_ARM", "0") == "1"
            self.p = make_params()
            self.ctrl = MatlabController(self.p, attitude_feedforward=ATTITUDE_FEEDFORWARD)
            self.mixer = RotorMixer(RotorMixerParams())
            n = self.p["n"]
            self._R0 = np.eye(3); 
            self._p0 = np.zeros(3)
            self._omega0 = np.zeros(3); 
            self._v0 = np.zeros(3)
            self._q = np.zeros(n); 
            self._qd = np.zeros(n)
            self._tr = None; 
            self._t = 0.0; 
            self._climb_z = None
            self._dt = 1/250.0; # was 0.001 
            self._ready = False; 
            self._first = True; self._log = 0
            self._arm_active_prev = False   # to detect the takeoff-hold → torque switchover
            self._reanchored = False        # re-anchor the reference once, at arm activation
            self._traj_t0 = 0.0             # trajectory time origin (set at re-anchor)

            # Acceleration-level feedforward estimate (qddot, omega_0_dot) for the
            # omega_e_dot term. Computed DATA-DRIVEN in the state callbacks using the
            # true elapsed time between messages, then low-pass filtered — this
            # replaces the old fixed-1kHz-dt finite difference, which produced
            # sample-and-hold spikes on the slower-arriving data. Only the feedforward
            # estimate changes; the impedance control law is untouched.
            self._qddot_est = np.zeros(n)
            self._omega0_dot_est = np.zeros(3)
            self._qd_prev_fd = None; self._t_qd_prev = None
            self._omega0_prev_fd = None; self._t_omega0_prev = None
            self._accel_lpf_alpha = 0.3   # EMA coeff (~15 Hz cutoff @250 Hz data); TUNE

            self.create_subscription(PoseStamped,  "state/pose",           self._pose_cb, qos_profile_sensor_data)
            self.create_subscription(TwistStamped, "state/twist_inertial", self._tlin_cb, qos_profile_sensor_data)
            self.create_subscription(TwistStamped, "state/twist",          self._tang_cb, qos_profile_sensor_data)
            self.create_subscription(JointState,   "joint_states",         self._joint_cb, 10)
            self._pub_joints = self.create_publisher(Float64MultiArray, "joint_torque_cmd", 10)
            self._pub_rotors = None
            self._px4_ocm_pub = None
            self._px4_motors_pub = None
            self._px4_command_pub = None
            self._px4_status_sub = None
            self._px4_status = None
            self._px4_setpoint_count = 0
            self._setup_rotor_output()
            self.create_timer(self._dt, self._loop)
            self.get_logger().info(
                f"ctrl_matlab node ready: rotor output={self._control_mode}, "
                f"PX4 auto-arm={self._px4_auto_arm}"
            )

        def _setup_rotor_output(self):
            """Create either the original Isaac rotor publisher or PX4 DDS I/O."""
            if not self._px4_offboard:
                self._pub_rotors = self.create_publisher(Float64MultiArray, "rotor_velocity_command", 10)
                return

            try:
                from px4_msgs.msg import ActuatorMotors, OffboardControlMode, VehicleCommand, VehicleStatus
                from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            except ImportError as exc:
                raise RuntimeError(
                    "PX4 Offboard mode requires px4_msgs. Source the PX4 ROS 2 workspace "
                    "before starting controller.py."
                ) from exc

            self._ActuatorMotors = ActuatorMotors
            self._OffboardControlMode = OffboardControlMode
            self._VehicleCommand = VehicleCommand
            self._VehicleStatus = VehicleStatus
            px4_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            # Relative names intentionally inherit /uav_<id> from the node namespace,
            # matching PX4_UXRCE_DDS_NS in the SITL launcher.
            self._px4_ocm_pub = self.create_publisher(
                OffboardControlMode, "fmu/in/offboard_control_mode", px4_qos
            )
            self._px4_motors_pub = self.create_publisher(
                ActuatorMotors, "fmu/in/actuator_motors", px4_qos
            )
            self._px4_command_pub = self.create_publisher(
                VehicleCommand, "fmu/in/vehicle_command", px4_qos
            )
            self._px4_status_sub = self.create_subscription(
                VehicleStatus, "fmu/out/vehicle_status", self._px4_status_cb, px4_qos
            )

        def _px4_status_cb(self, msg):
            old = self._px4_status
            self._px4_status = msg
            if old is None or old.arming_state != msg.arming_state or old.nav_state != msg.nav_state:
                self.get_logger().info(
                    f"PX4 status: arming_state={msg.arming_state}, nav_state={msg.nav_state}"
                )

        def _stamp_us(self):
            return self.get_clock().now().nanoseconds // 1000

        def _publish_px4_command(self, command, param1=0.0, param2=0.0):
            msg = self._VehicleCommand()
            msg.timestamp = self._stamp_us()
            msg.command = command
            msg.param1 = float(param1)
            msg.param2 = float(param2)
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            self._px4_command_pub.publish(msg)

        def _maybe_engage_px4(self):
            if not self._px4_auto_arm:
                return
            status = self._px4_status
            offboard = status is not None and status.nav_state == self._VehicleStatus.NAVIGATION_STATE_OFFBOARD
            armed = status is not None and status.arming_state == self._VehicleStatus.ARMING_STATE_ARMED
            count = self._px4_setpoint_count
            if (
                count >= PX4_OFFBOARD_WARMUP_SETPOINTS
                and count % PX4_COMMAND_RETRY_INTERVAL == 0
                and not (offboard and armed)
            ):
                if not offboard:
                    self._publish_px4_command(
                        self._VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
                    )
                    self.get_logger().info("Requested PX4 Offboard mode")
                elif not armed:
                    self._publish_px4_command(
                        self._VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0
                    )
                    self.get_logger().info("Requested PX4 arming in Offboard mode")

        def _publish_rotors(self, omega):
            """Publish desired rotor speed through the selected command path.

            Channel order is identity across the controller, PX4 none_iris, and
            the USD: 0=front-left, 1=rear-right, 2=front-right, 3=rear-left.
            PX4 performs only Offboard/arming/output gating in direct-actuator
            mode; allocation remains in :class:`RotorMixer`.
            """
            if not self._px4_offboard:
                msg = Float64MultiArray()
                msg.data = omega.tolist()
                self._pub_rotors.publish(msg)
                return

            stamp = self._stamp_us()
            mode = self._OffboardControlMode()
            mode.timestamp = stamp
            mode.direct_actuator = True
            self._px4_ocm_pub.publish(mode)

            controls = np.clip(
                (np.asarray(omega) - PX4_ROTOR_ARMED_IDLE) / PX4_ROTOR_INPUT_SCALING,
                0.0,
                1.0,
            )
            motors = self._ActuatorMotors()
            motors.timestamp = stamp
            motors.timestamp_sample = stamp
            motors.control = [float("nan")] * self._ActuatorMotors.NUM_CONTROLS
            for index, value in enumerate(controls):
                motors.control[index] = float(value)
            self._px4_motors_pub.publish(motors)
            self._px4_setpoint_count += 1
            self._maybe_engage_px4()

        def _build_X(self):
            n = self.p["n"]
            return np.concatenate([self._p0, self._R0.flatten(order="F"),
                                   self._q, self._v0, self._omega0, self._qd])

        def _pose_cb(self, msg):
            o = msg.pose.orientation; pos = msg.pose.position
            self._p0 = np.array([pos.x, pos.y, pos.z])
            self._R0 = quat_to_rot(o.w, o.x, o.y, o.z)
            if not self._ready:
                X0 = self._build_X()
                dyn0 = dynamics(X0, self.p)
                x_c = self._p0 + self._R0 @ dyn0["r_0c_0"]       # initial system CoM
                # anchor the trajectory at [spawn_xy, TAKEOFF_ALTITUDE]
                x_c0 = np.array([x_c[0], x_c[1], TAKEOFF_ALTITUDE])
                d0 = self._R0 @ (dyn0["r_0e_0"] - dyn0["r_0c_0"])  # EE-to-CoM offset {I}
                b1_0 = self._R0 @ np.array([1.0, 0, 0])
                self._tr = build_traj(x_c0, d0, b1_0)
                self._climb_z = x_c[2]                            # start of climb ramp
                self._ready = True
                self.get_logger().info(
                    f"first pose p0={self._p0} | trajectory='{self._tr['type']}' "
                    f"climb {x_c[2]:.2f}->{TAKEOFF_ALTITUDE:.2f} m over {TAKEOFF_TIME:.0f}s")

        def _tlin_cb(self, msg):
            l = msg.twist.linear
            self._v0 = self._R0.T @ np.array([l.x, l.y, l.z])

        def _tang_cb(self, msg):
            a = msg.twist.angular
            omega0 = np.array([a.x, a.y, a.z])
            self._omega0_dot_est = self._filtered_deriv(
                omega0, "_omega0_prev_fd", "_t_omega0_prev", self._omega0_dot_est)
            self._omega0 = omega0

        def _joint_cb(self, msg):
            n = self.p["n"]
            self._q = np.array(msg.position[:n])
            qd = np.array(msg.velocity[:n])
            self._qddot_est = self._filtered_deriv(
                qd, "_qd_prev_fd", "_t_qd_prev", self._qddot_est)
            self._qd = qd

        def _now_sec(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _filtered_deriv(self, x, prev_attr, t_attr, est):
            """Low-pass-filtered finite difference of x at the DATA rate. Uses the
            true elapsed time between messages (not the fixed loop dt), so it has no
            sample-and-hold spikes, and EMA-smooths the result for the feedforward."""
            now = self._now_sec()
            x_prev = getattr(self, prev_attr); t_prev = getattr(self, t_attr)
            if x_prev is not None and t_prev is not None:
                dt = now - t_prev
                if dt > 1e-6:
                    raw = (x - x_prev) / dt
                    a = self._accel_lpf_alpha
                    est = a * raw + (1.0 - a) * est
            setattr(self, prev_attr, x.copy()); setattr(self, t_attr, now)
            return est

        def _loop(self):
            if not self._ready:
                return
            self._t += self._dt

            # Arm gating (decide first — the reference construction depends on it).
            arm_active = (not PLATFORM_TUNE) and (self._t >= ARM_ACTIVATE_TIME)

            # RE-ANCHOR at the hold→active transition: rebuild the trajectory anchor from
            # the CURRENT actual pose so e_x = e_y ≈ 0 at handoff, no matter how far the
            # base drifted during the (possibly long, laggy) hold. The arm then holds its
            # current pose with ~zero initial error, and any trajectory starts from here.
            if arm_active and not self._reanchored:
                X0 = self._build_X()
                dyn0 = dynamics(X0, self.p)
                x_c = self._p0 + self._R0 @ dyn0["r_0c_0"]
                d0  = self._R0 @ (dyn0["r_0e_0"] - dyn0["r_0c_0"])
                b1_0 = self._R0 @ np.array([1.0, 0, 0])
                self._tr = build_traj(x_c.copy(), d0, b1_0)
                self._traj_t0 = self._t
                self._reanchored = True
                print(f"[CTRL] Re-anchored EE/CoM reference at arm activation (t={self._t:.2f}s)", flush=True)

            # Reference: takeoff climb / hover-hold (fixed EE offset) until the arm is
            # active, then run the trajectory from the re-anchored pose.
            if not arm_active:
                if self._t < TAKEOFF_TIME:
                    dz = TAKEOFF_ALTITUDE - self._climb_z
                    self._climb_z += max(-self.CLIMB_RATE * self._dt, min(self.CLIMB_RATE * self._dt, dz))
                z3 = np.zeros(3)
                ref = {"x_cd": np.array([self._tr["x_c0"][0], self._tr["x_c0"][1], self._climb_z]),
                       "x_cd_dot": z3, "x_cd_ddot": z3, "x_cd_d3": z3, "x_cd_d4": z3}
                ref = _fixed_ee(ref, self._tr)
            else:
                ref = generate_reference(self._t - self._traj_t0, self._tr)
            try:
                X = self._build_X()
                dyn = dynamics(X, self.p)
                # Hand the filtered, data-rate acceleration estimates to the arm
                # feedforward (matches the MATLAB `acc` interface). Impedance law unchanged.
                self.ctrl.ext_qddot = self._qddot_est
                self.ctrl.ext_omega0_dot = self._omega0_dot_est
                # Gate the arm channel (computed above, drives the re-anchor + reference).
                self.ctrl.arm_active = arm_active
                res = self.ctrl(X, dyn, ref, self._dt)
                # EE/CoM compatibility report at the takeoff-hold → torque switch.
                # If e_y is small here, the reference IS compatible and any later arm swing
                # is post-switch drift; if e_y is large, the reference itself is off.
                # Repeat for ~200 cycles (~0.2 s) so it can't scroll past unseen.
                if self.ctrl.arm_active and not self._arm_active_prev:
                    self._switch_log = 200
                if getattr(self, "_switch_log", 0) > 0:
                    self._switch_log -= 1
                    print("======== [SWITCH] arm→torque compatibility ========\n"
                          f"  EE  actual r_e ={res['r_e'].round(3)}  ref r_ed={res['r_ed'].round(3)}\n"
                          f"  EE  error  e_y ={res['e_y'].round(3)}   (small ⇒ compatible)\n"
                          f"  CoM actual x_c ={res['x_c'].round(3)}  ref x_cd={res['x_cd'].round(3)}\n"
                          f"  CoM error  e_x ={res['e_x'].round(3)}  |  e_R={res['e_R'].round(3)}\n"
                          "===================================================", flush=True)
                self._arm_active_prev = self.ctrl.arm_active
                omega = self.mixer.mix(res["thrust"], res["tau_body"])
                self._publish_rotors(omega)
                # Append the arm-active flag (5th element) so the plant releases the arm
                # EXACTLY when the controller starts commanding — no clock drift between
                # the controller's accumulated-dt timer and the plant's sim-time timer.
                m = Float64MultiArray()
                m.data = res["tau_joint"].tolist() + [1.0 if self.ctrl.arm_active else 0.0]
                self._pub_joints.publish(m)
                if self._first:
                    self._first = False
                    self.get_logger().info(
                        f"first ctrl: thrust={res['thrust']:.2f} omega={omega.round(1)}")
                self._log += 1
                if self._log >= 100:
                    self._log = 0
                    self.get_logger().info(
                        f"z={self._p0[2]:.3f} thrust={res['thrust']:.1f}N "
                        f"e_x={res['e_x'].round(3)} e_R={res['e_R'].round(3)} "
                        f"e_y={res['e_y'].round(3)} "
                        f"|u3|={np.linalg.norm(res['u3']):.3f}")
            except Exception as exc:
                self.get_logger().error(f"loop: {exc}")

    def main(args=None):
        rclpy.init(args=args)
        rclpy.spin(MatlabControllerNode())
        rclpy.shutdown()

    if __name__ == "__main__":
        main()


# ===========================================================================
# Standalone sanity / cross-validation against ctrl.py
# ===========================================================================

if __name__ == "__main__" and not _HAS_ROS2:
    params = make_params()
    n = params["n"]
    R0 = np.eye(3); q = np.array([0.0, 0.3, -0.6, 0.0])
    X = np.concatenate([np.zeros(3), R0.flatten(order="F"), q,
                        np.zeros(3), np.zeros(3), np.zeros(n)])
    dyn = dynamics(X, params)
    Mt = np.linalg.inv(dyn["T"]).T  # not used; quick checks below
    print("m_tot         =", sum(params["m_i"]))
    print("r_0c_0        =", dyn["r_0c_0"].round(4))
    sv = np.linalg.svd(dyn["J_3y"], compute_uv=False)
    print("J_3y cond     = %.1f" % (sv[0] / sv[-1]))

    r0 = X[0:3]; R0_ = X[3:12].reshape(3, 3, order="F")
    x_c = r0 + R0_ @ dyn["r_0c_0"]
    tr = build_traj(x_c, R0_ @ (dyn["r_0e_0"] - dyn["r_0c_0"]),
                    R0_ @ np.array([1.0, 0, 0]), "hover")
    ref = generate_reference(0.0, tr)
    ctrl = MatlabController(params)
    res = ctrl(X, dyn, ref, 0.01)
    print(f"hover thrust  = {res['thrust']:.3f} N  (m·g = {sum(params['m_i'])*9.81:.3f})")
    print(f"tau_body      = {res['tau_body'].round(4)}")
    print(f"tau_joint     = {res['tau_joint'].round(4)}")
    print(f"e_y           = {res['e_y'].round(4)}")

    # cross-check dynamics against the validated ctrl.py
    try:
        import ctrl as _c
        d2 = _c.AerialManipulatorDynamics(_c.RobotParams())
        d2.update(R0, q, np.zeros(n), np.zeros(3))
        print("\ncross-check vs ctrl.py:")
        print("  |M_r diff|   = %.2e" % np.linalg.norm(dyn["M_r"] - d2.M_tilde_r))
        print("  |J_y diff|   = %.2e" % np.linalg.norm(dyn["J_y"] - d2.J_y))
        print("  |T diff|     = %.2e" % np.linalg.norm(dyn["T"] - d2.T))
    except Exception as exc:
        print("cross-check skipped:", exc)
