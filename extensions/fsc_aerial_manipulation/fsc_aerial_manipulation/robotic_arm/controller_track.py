#!/usr/bin/env python3
"""
controller_track.py — faithful Python port of the working MATLAB controller.

Author: Ben Natra (send2ben123@gmail.com)
Author: Shiqi Gao (shiqi.gao907@gmail.com)

Direct, function-structured port of the MATLAB pipeline so the code mirrors the
reference exactly (which validated stable in simulation):

    dynamics(X, params)          <- dynamics.m   (M,C,g, M̃,C̃,g̃, T, J_y, ...)
    controller(X, dyn, ref, p)   <- controller.m (u1, u2, u3)
    tau = Tᵀ [u1·R0·e3 ; u2 ; u3]                <- closed_loop_dynamics.m

State vector (MATLAB layout, length 18+2n):
    X = [ r0(3); vec(R0)(9, col-major); q(n); v0(3); omega0(3); qdot(n) ]

The MATLAB ran as a single-process, zero-latency closed loop (controller and
plant share M,C,g). Here `dynamics`+`controller` are the controller and Isaac
is the plant. Two ways to close the loop:

  in-process — 02_aerial_manipulator_track.py imports this module and runs it
      inside a physics-step callback (250 Hz, dt = the physics step). The
      preferred rig on Windows, where the ROS2 round-trip latency is fatal.
  ROS2 node — `python3 controller_track.py` where rclpy is available (Linux /
      low-latency setups): subscribes to the vehicle state topics and
      publishes one synchronized [omega(4), tau_joint(n)] actuator packet.

Either way the arm runs in true effort mode from the start, with a software
PD+gravity hold during takeoff (`MatlabController.hold`).
"""

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
    # ALL values below re-extracted from AM_realign.usda (2026-07-14, scratch
    # extract_realign_params.py). That asset re-authored the model: every joint
    # is wired PARENT-FIRST (body0 = parent link) with IDENTITY joint frames and
    # identity prim orients, and the body frame is rotated 180° about z vs the
    # old gripper_drone_zz.usda — so l_i/com_i are the x,y-mirror of the old
    # values, the authored diagonalInertia values are already in the body frame
    # (no permutation), and the old child-first axis negation is GONE: the
    # authored axes ARE the model axes, and Isaac's raw q/qdot/tau are the model
    # coordinates directly. (The MATLAB keeps its own original axes — its plant
    # is the model itself.)
    n = 4
    # m0 = body (2.4761) + 4 rotors (4 × 0.039887); rotors are rigid on the base, not
    # arm links. Omitting them left total mass 0.23 kg light → hover feedforward short.
    m0: float = 2.4760795 + 0.15955
    # I0 = body inertia + the 4 rotors (own inertia + parallel-axis about the
    # body origin — the model places the base CoM at the body prim origin).
    I0 = np.diag([0.07368128, 0.07250296, 0.11707737])

    # link_masses[3] = manip_base (0.1610) + hub + 2×grip + 2×finger link, lumped.
    link_masses = [0.1048326, 0.1423463, 0.13992727, 0.2495176]

    # Inertia diagonals in the LINK frame (= body frame at q=0), as authored in
    # AM_realign.usda (link2/link3 were re-derived by the asset realign — do not
    # compare against the old permuted values). link_inertias[3] is the lumped
    # gripper assembly about the lump CoM (parts' own diagonals + parallel-axis;
    # off-diagonals ≤1.7e-5 dropped). The pre-realign model used only the bare
    # manip_base inertia here — revert to diag([6.111e-05, 9.355e-05, 1.0707e-04])
    # if behaviour must be A/B'd against the old flying config.
    link_inertias = [
        np.diag([3.720000e-05, 3.749000e-05, 2.161000e-05]),
        np.diag([3.593700e-04, 3.485900e-04, 5.506000e-05]),
        np.diag([3.130200e-04, 3.253000e-05, 3.184700e-04]),
        np.diag([1.908700e-04, 2.637700e-04, 1.951000e-04]),
    ]
    # h_i^{i-1} : joint axes (body frame at q=0), l_i : O_{i-1}->O_i offsets.
    # Parent-first wiring + identity joint frames: PhysX measures child-rel-parent
    # about the authored +axis, which is exactly the model convention.
    h_i_im1 = [
        np.array([+0.000000, +0.000000, +1.000000]),   # manip_joint1 ~ +z
        np.array([+1.000000, +0.000000, +0.000000]),   # manip_joint2 ~ +x
        np.array([+1.000000, +0.000000, +0.000000]),   # manip_joint3 ~ +x
        np.array([+0.000000, +0.000000, +1.000000]),   # manip_joint4 ~ +z
    ]
    # ── RAW geometry as extracted from AM_realign.usda ──────────────────────────
    # The offset chain straight from the asset, with the RAW chain point list
    #     O_raw = [ base, manip_joint1, manip_joint2, manip_joint3, manip_joint4 ].
    # raw_l[k]   = O_raw[k+1] - O_raw[k]  (link-offset vectors down the chain)
    # raw_com[k] = O_raw[k]   -> CoM of link k+1  (link's CoM from its proximal point)
    # raw_com[3] is the manip_base + hub + grips + 2×finger-link lump (mass 0.2495176
    # = link_masses[3]), at the gripper's authored rest pose (jaws open).
    # The catch motivating the re-registration below: raw_l[0] is a base->manip_joint1
    # offset, so O_raw[k] = manip_joint_k — but dynamics() pivots joint k about O[k-1].
    # Feeding raw_l straight in mis-registers every joint one link too PROXIMAL and
    # over-estimates manip_joint3's gravity by ~0.092 N·m. (The posture anchor below
    # masked that; the corrected geometry is strictly better and also fixes J_y/M/C
    # for arm-moving trajectories — reach/pickplace — not just the gravity bias.)
    raw_l = [
        np.array([+0.000500, +0.002000, -0.042000]),   # base         -> manip_joint1
        np.array([-0.019000, +0.000070, -0.040500]),   # manip_joint1 -> manip_joint2
        np.array([+0.000089, +0.024000, -0.128000]),   # manip_joint2 -> manip_joint3
        np.array([+0.019976, +0.128927, -0.021750]),   # manip_joint3 -> manip_joint4
    ]
    raw_com = [
        np.array([+0.000960, +0.001520, -0.073027]),   # base         -> link1 (USD link2) CoM
        np.array([+0.000320, +0.010163, -0.137795]),   # manip_joint1 -> link2 (USD link3) CoM
        np.array([+0.019480, +0.107164, -0.128866]),   # manip_joint2 -> link3 (USD link4) CoM
        np.array([+0.018278, +0.123460, -0.061369]),   # manip_joint3 -> link4 (gripper)   CoM
    ]

    # ── Step 1: accumulate to ABSOLUTE positions in the body frame (base at 0) ──
    # O_raw[k] = sum(raw_l[:k]); joint k anchor = O_raw[k]; link k CoM = O_raw[k-1]+raw_com[k-1].
    O_raw = [np.zeros(3)]
    for d in raw_l:
        O_raw.append(O_raw[-1] + d)
    joint_pos = O_raw[1:]                                       # manip_joint1..4 (absolute)
    link_com  = [O_raw[k] + raw_com[k] for k in range(n)]       # link k+1 CoM   (absolute)

    # ── Step 2: RE-REGISTER into the convention dynamics() expects ──────────────
    # dynamics() needs joint k at O[k-1] and the EE at O[n], with O_0 at the base.
    # Shift the chain up by one to O[k-1]=manip_joint_k; manip_joint1 (a vertical +z
    # axis -> zero gravity moment) is IDEALIZED onto O_0, its base->joint1 offset
    # folding harmlessly into l_i[0] (mirroring main_sim.m). Result:
    #     O = [ base(0), manip_joint2, manip_joint3, manip_joint4, EE ].
    # This moves joint 3's pivot from O_raw[2]=manip_joint2 to manip_joint3, removing
    # the l_i[2].y·(mass below)·g ≈ 0.092 N·m gravity over-estimate. Every body CoM
    # and the EE point stay put; only the joint pivots change.
    ee_pos  = joint_pos[3]                                      # EE = the wrist (manip_joint4)
    O_chain = [np.zeros(3)] + joint_pos[1:] + [ee_pos]          # O_0..O_n  (len n+1)
    l_i   = [O_chain[k]   - O_chain[k - 1] for k in range(1, n + 1)]     # O_{k-1} -> O_k
    com_i = [link_com[i - 1] - O_chain[i - 1] for i in range(1, n + 1)]  # O_{i-1} -> CoM_i
    # reflected rotor (armature) inertia along each joint axis (rank-1)
    J_arm = 353.5 ** 2 * 1.6e-7
    I_i_i = [I0.copy()]
    for i in range(n):
        I_i_i.append(link_inertias[i] + J_arm * np.outer(h_i_im1[i], h_i_im1[i]))

    return {
        "n": n,
        "m_i": [m0] + link_masses,     # len n+1
        "l_i": l_i,                     # len n
        "com_i": com_i,                 # len n  (O_{i-1}→CoM_i, link-i frame)
        "h_i_im1": h_i_im1,             # len n
        "I_i_i": I_i_i,                 # len n+1
        "g": 9.81,
    }


# ---------------------------------------------------------------------------
# POSTURE_KP/KD — joint-space posture anchor added to u3 (impedance mode),
# tracking the trajectory's q_d (home q=0 when none). Two jobs: (1) stabilize
# the arm's internal coordinate against the gravity-map residual slope
# (~0.5-1 N·m/rad of leftover CoM error — without it the arm folds slowly to a
# joint limit while e_y stays small); (2) pin the q1/q4 pair, which is
# gravity-free (vertical axes) and forms the near-cancelling internal direction
# the heading task can excite. Stiffer on 1/4 for that reason. Added to u3
# BEFORE τ = Tᵀu so tau_body pre-compensates its reaction consistently.
# Values from the 2026-07-07 gain sweep (utils/gain_sweep.py, validated on
# reach + circle + pickplace): 2x the original anchor — best EE tracking of
# the sweep at the lowest mean torque, and smoother (2.5x less high-frequency
# EE error than the original gains).
POSTURE_KP = np.array([8.0, 4.0, 4.0, 8.0])     # [N·m/rad]
POSTURE_KD = np.array([0.6, 0.3, 0.3, 0.6])     # [N·m·s/rad]
# USE_POSTURE_ANCHOR — selects the END-EFFECTOR control law on u3:
#   True  (default): the FULL law — the paper's coupled EE impedance PLUS the
#          joint-space posture-anchor PD above (the extra term on u3).
#   False : STRICTLY the paper's law — u3 is the coupled EE impedance alone, with
#          NO posture anchor. With the geometry-correct model the arm's gravity
#          bias is gone, so this is the clean A/B of the paper's law vs the +PD
#          form (and matches controller_gmo minus the disturbance observer).
# Overridable per-run via AM_SWEEP {"USE_POSTURE_ANCHOR": false}.
USE_POSTURE_ANCHOR = False
# DLS_LAMBDA — damped-least-squares regularization of the J_3y solve in u3.
# 0.0 = the paper's EXACT inverse (current setting, 2026-07-14): with the
# AM_realign asset the model matches the plant well enough that the only
# non-paper term in the law is the posture anchor. If the q1/q4 heading
# chatter ever returns, raise to 0.3: J_3y still passes through an
# ill-conditioned region mid-reach (smallest singular value ~0.35 around
# q2=q3≈−0.1 vs ~0.85 at home) where the exact inverse commands a large
# near-cancelling q1/q4 torque pair for mrad-level heading errors — on the
# OLD mis-modeled asset that drove the 2026-07-02/03 reach runaways, and
# λ=0.3 caps that direction's gain (s → s/(s²+λ²)) while leaving
# well-conditioned directions (s ≳ 1) intact.
DLS_LAMBDA = 0.3

OMEGA_E_FF = False


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
    com_i = params.get("com_i")                        # O_{i-1}→CoM_i, link-i frame
    if com_i is None:
        com_i = [l_i[k] / 2.0 for k in range(n)]       # fallback: midpoint approx
    r0i = [np.zeros(3)]                                # CoM r_{0i}^0
    for i in range(1, n + 1):
        r0i.append(O[i - 1] + R_i_0[i] @ com_i[i - 1])
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
    # B/B_dot follow the paper/MATLAB transcription. This sign is forced by the
    # decoupling identity M == Tᵀ·M̃·T (expanding the ω/q̇ cross block gives
    # exactly B = Σ(−mᵢAᵢᵀ·hat(dᵢ) + Jωᵢᵀ·Iᵢ)).
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
TRAJ_TYPE = "hover"  # "hover" | "line" | "sine" | "circle" | "circle_bent" | "poly" | "reach" | "pickplace"
TRAJ_CONFIG = {
    "hover":  {"T": 8.0},
    "line":   {"axis": "xy", "D": 2.0, "T": 10.0},
    "sine":   {"axis": "xy", "A": 0.5, "Ncyc": 2, "T": 12.0},
    "circle": {"r": 1.0, "T": 20.0},
    # circle_bent: the SAME circle, but the arm is HELD at a better-conditioned
    # pose q_hold instead of q=0. Diagnostic to isolate whether the posture anchor
    # is only needed because of J_3y conditioning: run it with the POSTURE_KP/KD
    # term in u3 commented out and see if the arm still folds. MEASURED cond(J_3y)
    # on this asset (joints 2/3 rotate about x): ~96 at q=0, best ~35 at q2=q3~+0.8
    # (POSITIVE; negative q2/q3 is MUCH worse -- ~150 at -0.5, ~880 at -0.75, which
    # is why the reach target [0,-0.5,-0.5,0] is the wrong pose for this test).
    # Keep q1=q4=0: they barely change conditioning and zeroing them keeps
    # b1_de = tangent consistent (e_RE3 = 0). NOTE: +q2/q3 swings the arm
    # outward-and-up (reach q_target sign note) -- reduce if it hits the body/rotors.
    "circle_bent": {"r": 1.0, "T": 20.0, "q_hold": [0.0, 0.8, 0.8, 0.0]},
    "poly":   {"D": 2.0, "A": 0.2, "T": 12.0},
    # "reach": body hovers in place; the ARM moves relative to it. Joint-space
    # min-jerk 0 → q_target over T_move (after T_hold of settled hover), the
    # gripper closes during T_grip at the target pose, then min-jerk back home
    # over T_return with the gripper kept closed. The EE reference r_ed is
    # generated CONSISTENTLY from the arm FK (as a delta from the anchored
    # offset, so it is continuous at the maneuver start), and ref carries
    # q_d/q_d_dot so the posture anchor TRACKS the maneuver instead of pulling
    # the arm back to q=0. ref["grip"] (0 open / 1 closed) tells the demo when
    # to actuate gripper_joint. Keep q_target[0]=q_target[3]=0 — those joints
    # yaw the EE heading while the b1_de reference stays fixed.
    # q_target SIGN (2026-07-14): AM_realign's re-authored axes invert the
    # physical direction of q2/q3 vs the old asset — [0,−0.5,−0.5,0] reproduces
    # the maneuver every validated run flew (arm sweeps across under the body,
    # EE delta ≈ [0, −0.19, −0.05] m from its hover-home offset in the new
    # frame). [+0.5,+0.5] would instead swing outward-and-up (untested; better
    # J_3y conditioning, but the measured EE_SAG / grasp geometry don't apply).
    "reach":  {"q_target": [0.0, -0.5, -0.5, 0.0],
               "T_hold": 2.0, "T_move": 5.0, "T_grip": 3.0, "T_return": 5.0},
    # "pickplace": full mission — fly to the pick point, reach + close the
    # gripper, retract, fly to the place point, reach + open, retract, return
    # home. pick_wp/place_wp are CoM waypoint OFFSETS from the hover anchor [m]
    # (keep z=0 for constant altitude). Base flights and arm moves are min-jerk;
    # the same FK-consistent EE reference + q_d posture tracking as "reach".
    # At the arm target the EE sits ≈[0, −0.19, −0.05] m from its hover-home
    # offset (joints 2/3 swing in the body y-z plane; see the reach q_target
    # sign note above) — place objects there.
    # dz_approach/T_vert: horizontal flights happen dz ABOVE the waypoints and
    # the vehicle descends/climbs vertically at each one — the landing legs
    # reach ~0.6 m below the body, so cruising at grasp altitude sweeps them
    # through the pick stand (observed: feet knocked the cube off). A vertical
    # final approach lets the stand rise between the legs instead.
    "pickplace": {"pick_wp":  [1.0, 0.0, 0.0],
                  "place_wp": [1.0, -1.5, 0.0],
                  "q_target": [0.0, -0.5, -0.5, 0.0],
                  "T_hold": 2.0, "T_fly": 4.0, "T_arm": 4.0, "T_grip": 2.0,
                  "dz_approach": 0.5, "T_vert": 2.5},
}
TAKEOFF_ALTITUDE = 1.5     # [m] climb to this before the trajectory
TAKEOFF_TIME     = 4.0     # [s] climb + settle window before trajectory starts
# EE_INTEGRAL — optional integral term K_i·∫e_y on u3: rejects any residual
# constant feedback-linearization offset the impedance leaves (e_y_ss = δ/K_y).
# Off by default now that the model matches the plant; useful later for payload.
EE_INTEGRAL = False   # enable/disable the integral term on e_y
EI_CLAMP    = 5.0    # anti-windup clamp on ∫e_y (per component)
EI_LEAK     = 0.03   # leak rate [1/s]: ei -= EI_LEAK·ei·dt. Bleeds windup, bounds the
                     # low-freq gain (e_y_ss≈δ/(K_y+K_i/EI_LEAK)). 0 = pure integrator.


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


_FK_PARAMS = None


def _arm_fk(q):
    """Light FK for reference generation (level body, R0=I): returns
    (r_0e_0, r_0c_0, A_e, A) at joint config q — the subset of dynamics()
    needed to turn a joint-space arm reference into a consistent EE reference."""
    global _FK_PARAMS
    if _FK_PARAMS is None:
        _FK_PARAMS = make_params()
    p = _FK_PARAMS
    n = p["n"]; mi = p["m_i"]; m_total = sum(mi)
    R = [np.eye(3)]
    for i in range(n):
        R.append(R[i] @ joint_rotation(p["h_i_im1"][i], q[i]))
    h0 = [np.zeros(3)]
    for i in range(1, n + 1):
        h0.append(R[i - 1] @ p["h_i_im1"][i - 1])
    O = [np.zeros(3)]
    for i in range(1, n + 1):
        O.append(O[i - 1] + R[i] @ p["l_i"][i - 1])
    r_e = O[n]
    r0i = [np.zeros(3)]
    for i in range(1, n + 1):
        r0i.append(O[i - 1] + R[i] @ p["com_i"][i - 1])
    r_c = sum(mi[i] * r0i[i] for i in range(n + 1)) / m_total
    A_i = [np.zeros((3, n)) for _ in range(n + 1)]
    for i in range(1, n + 1):
        for k in range(1, i + 1):
            A_i[i][:, k - 1] = np.cross(h0[k], r0i[i] - O[k - 1])
    A_e = np.zeros((3, n))
    for k in range(1, n + 1):
        A_e[:, k - 1] = np.cross(h0[k], r_e - O[k - 1])
    A = sum(mi[i] * A_i[i] for i in range(n + 1)) / m_total
    return r_e, r_c, A_e, A


def build_traj(x_c0, d0, b1_0, traj_type=None):
    """Combine the selected config with the initial anchors (port of build_traj)."""
    ttype = traj_type or TRAJ_TYPE
    cfg = TRAJ_CONFIG[ttype]
    tr = {"type": ttype, "x_c0": np.array(x_c0, float),
          "d0": np.array(d0, float), "b1": np.array(b1_0, float),
          "T": cfg.get("T", 0.0)}
    if ttype == "line":
        tr["dir"] = _axis_dir(cfg["axis"]); tr["D"] = cfg["D"]
    elif ttype == "sine":
        tr["e"] = _axis_dir(cfg["axis"]); tr["A"] = cfg["A"]; tr["Ncyc"] = cfg["Ncyc"]
    elif ttype in ("circle", "circle_bent"):
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
        if ttype == "circle_bent":
            # arm held at a fixed better-conditioned pose; the demo's takeoff hold
            # drives the arm to q_hold BEFORE the circle starts, so the re-anchored
            # d0 is already the bent EE offset (no FK delta needed -- d0 carries it).
            tr["q_hold"] = np.array(cfg["q_hold"], float)
    elif ttype == "poly":
        tr["D"] = cfg["D"]; tr["A"] = cfg["A"]
        ef = np.array([b1_0[0], b1_0[1], 0.0])
        ef = ef / np.linalg.norm(ef) if np.linalg.norm(ef) > 1e-9 else np.array([1.0, 0, 0])
        tr["e_fwd"] = ef; tr["e_lat"] = np.array([-ef[1], ef[0], 0.0])
    elif ttype == "reach":
        tr["q_target"] = np.array(cfg["q_target"], float)
        tr["T_hold"] = cfg["T_hold"]; tr["T_move"] = cfg["T_move"]
        tr["T_grip"] = cfg["T_grip"]; tr["T_return"] = cfg["T_return"]
        tr["T"] = cfg["T_hold"] + cfg["T_move"] + cfg["T_grip"] + cfg["T_return"]
        # FK offset at home, so the reach EE reference is generated as a DELTA
        # from the anchored d0 (continuous at the maneuver start even though the
        # actual hover offset differs slightly from the level-body FK).
        r_e0, r_c0, _, _ = _arm_fk(np.zeros(4))
        tr["d_fk0"] = r_e0 - r_c0
    elif ttype == "pickplace":
        qT = np.array(cfg["q_target"], float)
        z4 = np.zeros(4)
        home = np.zeros(3)
        pick = np.array(cfg["pick_wp"], float)
        place = np.array(cfg["place_wp"], float)
        Th, Tf, Ta, Tg = cfg["T_hold"], cfg["T_fly"], cfg["T_arm"], cfg["T_grip"]
        Tv = cfg.get("T_vert", 2.5)
        up = np.array([0.0, 0.0, cfg.get("dz_approach", 0.5)])
        # (duration, base_from, base_to, q_from, q_to, grip) — min-jerk within
        # each segment; only one of base/arm moves per segment. Horizontal
        # flights cruise dz above the waypoints; each waypoint is entered and
        # left VERTICALLY so the stands pass between the landing legs.
        segs = [
            (Th, home,       home,       z4, z4, 0.0),   # settle at the anchor
            (Tf, home,       pick + up,  z4, z4, 0.0),   # cruise above the pick stand
            (Tv, pick + up,  pick,       z4, z4, 0.0),   # vertical descent
            (Ta, pick,       pick,       z4, qT, 0.0),   # reach toward the object
            (Tg, pick,       pick,       qT, qT, 1.0),   # close the gripper
            (Ta, pick,       pick,       qT, z4, 1.0),   # retract with the object
            (Tv, pick,       pick + up,  z4, z4, 1.0),   # vertical climb-out
            (Tf, pick + up,  place + up, z4, z4, 1.0),   # cruise to the place point
            (Tv, place + up, place,      z4, z4, 1.0),   # vertical descent
            (Ta, place,      place,      z4, qT, 1.0),   # reach to set it down
            (Tg, place,      place,      qT, qT, 0.0),   # open the gripper
            (Ta, place,      place,      qT, z4, 0.0),   # retract empty
            (Tv, place,      place + up, z4, z4, 0.0),   # vertical climb-out
            (Tf, place + up, home,       z4, z4, 0.0),   # return to the anchor
        ]
        t0 = 0.0
        sched = []
        for (Ts, b0, b1, q0, q1, gr) in segs:
            sched.append({"t0": t0, "t1": t0 + Ts,
                          "b0": np.array(b0, float), "b1": np.array(b1, float),
                          "q0": np.array(q0, float), "q1": np.array(q1, float),
                          "grip": gr})
            t0 += Ts
        tr["sched"] = sched
        tr["T"] = t0
        r_e0, r_c0, _, _ = _arm_fk(np.zeros(4))
        tr["d_fk0"] = r_e0 - r_c0
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

    if ttype == "reach":
        qT = tr["q_target"]
        Th, Tm, Tg, Tr = tr["T_hold"], tr["T_move"], tr["T_grip"], tr["T_return"]
        n4 = len(qT)
        if t < Th:                              # settled hover, arm home, gripper open
            q_d, q_dd, q_ddd, grip = np.zeros(n4), np.zeros(n4), np.zeros(n4), 0.0
        elif t < Th + Tm:                       # min-jerk out to the target pose
            s, ds, d2s, _, _ = _minjerk((t - Th) / Tm)
            q_d, q_dd, q_ddd, grip = s * qT, ds / Tm * qT, d2s / Tm**2 * qT, 0.0
        elif t < Th + Tm + Tg:                  # hold at target, close the gripper
            q_d, q_dd, q_ddd, grip = qT.copy(), np.zeros(n4), np.zeros(n4), 1.0
        elif t < Th + Tm + Tg + Tr:             # min-jerk back home, gripper stays closed
            s, ds, d2s, _, _ = _minjerk((t - Th - Tm - Tg) / Tr)
            q_d, q_dd, q_ddd = (1 - s) * qT, -ds / Tr * qT, -d2s / Tr**2 * qT
            grip = 1.0
        else:                                   # hold home with the object
            q_d, q_dd, q_ddd, grip = np.zeros(n4), np.zeros(n4), np.zeros(n4), 1.0

        # EE reference from FK, as a delta from the anchored offset (continuous
        # at the maneuver start). r_ed_dot/ddot via the offset Jacobian
        # J = A_e − A (Ȧ terms dropped — the maneuver is slow).
        r_e, r_c, A_e, A = _arm_fk(q_d)
        J = A_e - A
        ref = {"x_cd": tr["x_c0"].copy(), "x_cd_dot": z3, "x_cd_ddot": z3,
               "x_cd_d3": z3, "x_cd_d4": z3,
               "r_ed": tr["x_c0"] + tr["d0"] + ((r_e - r_c) - tr["d_fk0"]),
               "r_ed_dot": J @ q_dd, "r_ed_ddot": J @ q_ddd,
               "b1_d": tr["b1"], "b1_d_dot": z3, "b1_d_ddot": z3,
               "b1_de": tr["b1"], "b1_de_dot": z3, "b1_de_ddot": z3,
               "q_d": q_d, "q_d_dot": q_dd, "grip": grip}
        return ref

    if ttype == "pickplace":
        seg = None
        for s_ in tr["sched"]:
            if t < s_["t1"]:
                seg = s_
                break
        if seg is None:                       # past the end: hold the final state
            seg = tr["sched"][-1]
            t = seg["t1"]
        Ts = seg["t1"] - seg["t0"]
        s, ds, d2s, d3s, d4s = _minjerk((t - seg["t0"]) / Ts)
        db = seg["b1"] - seg["b0"]
        x_cd = tr["x_c0"] + seg["b0"] + s * db
        ref = {"x_cd": x_cd, "x_cd_dot": ds / Ts * db, "x_cd_ddot": d2s / Ts**2 * db,
               "x_cd_d3": d3s / Ts**3 * db, "x_cd_d4": d4s / Ts**4 * db}
        dq_ = seg["q1"] - seg["q0"]
        q_d = seg["q0"] + s * dq_
        q_dd = ds / Ts * dq_
        q_ddd = d2s / Ts**2 * dq_
        # FK-consistent EE reference carried with the moving base (same delta
        # construction as "reach")
        r_e, r_c, A_e, A = _arm_fk(q_d)
        J = A_e - A
        ref.update({"r_ed": x_cd + tr["d0"] + ((r_e - r_c) - tr["d_fk0"]),
                    "r_ed_dot": ref["x_cd_dot"] + J @ q_dd,
                    "r_ed_ddot": ref["x_cd_ddot"] + J @ q_ddd,
                    "b1_d": tr["b1"], "b1_d_dot": z3, "b1_d_ddot": z3,
                    "b1_de": tr["b1"], "b1_de_dot": z3, "b1_de_ddot": z3,
                    "q_d": q_d, "q_d_dot": q_dd, "grip": seg["grip"]})
        return ref

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

    if ttype in ("circle", "circle_bent"):
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
        # EE offset carried with the base yaw; headings follow the tangent. For
        # circle_bent the arm sits at q_hold, so tr["d0"] (anchored at handover,
        # after the takeoff hold moved the arm to q_hold) is ALREADY the bent EE
        # offset — the same carry-with-yaw applies, arm stays fixed to the platform.
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
        if ttype == "circle_bent":
            # tell the posture anchor to track the bent pose (not q=0)
            ref["q_d"] = tr["q_hold"].copy()
            ref["q_d_dot"] = np.zeros_like(tr["q_hold"])
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
    # k_x/k_v from the 2026-07-07 gain sweep (utils/gain_sweep.py, validated on
    # reach + circle + pickplace): 16/8 halved the CoM error vs the MATLAB's
    # 10/5 with no cost in attitude, EE error, or torque. k_R/k_w unchanged.
    k_x: float = 16.0
    k_v: float = 8.0
    k_R: float = 8.0
    k_w: float = 2.0
    # M_r_d: the paper/MATLAB use eye(3) → attitude gain M_r·M_r_d⁻¹·k_R with
    # M_r ~ 0.1 gives only ~0.09 N·m of authority. IN-PROCESS TEST (user,
    # 2026-07-03): eye(3) still tips over even at 250 Hz zero-latency;
    # 2·eye(3) flies. I0_body kept as the default. (Sweepable via AM_SWEEP
    # "M_r_d" in gain_sweep.py.)
    M_r_d = np.diag([0.065194, 0.064352, 0.100439])
    # EE impedance gains. Position channels 8/6 from the 2026-07-07 sweep
    # (MATLAB base was 4/4; 16 improved EE further but doubled attitude error —
    # skipped). The 4th (EE-heading) channel stays DETUNED to 1: J_3y⁻¹'s
    # heading column is high-gain (it drives the near-massless q1/q4 wrist
    # pair — mrad-level e_y[3]/e_vy[3] noise produced N·m-level torque and
    # clamp chatter, 2026-07-02 log). K_y does NOT remove steady gravity
    # droop; the integral does.
    D_y: np.ndarray = field(default_factory=lambda: np.diag([6.0, 6.0, 6.0, 1.0]))
    K_y: np.ndarray = field(default_factory=lambda: np.diag([8.0, 8.0, 8.0, 1.0]))
    # Integral gain on the EE error (EE_INTEGRAL) — the ROBUST droop fix (rejects the
    # feedback-linearization residual δ; e_y_ss=δ/K_y without it). K_i=3 destabilized:
    # it put integral action near the coupled loop's marginal frequency → ringing.
    # Small K_i pushes that action to LOW frequency where the loop has phase margin, so
    # it nulls the droop slowly and stably. 0.6+leak converged cleanly (no ringing, big
    # margin) but slow (~0.003/s) to a ~0.08 residual; raised to 1.0 (with lower leak)
    # for ~3x low-freq authority → faster convergence, ~0.04 residual, still in margin.
    K_i: np.ndarray = field(default_factory=lambda: 1.0 * np.eye(4))


class MatlabController:
    """The coupled feedback-linearizing law (controller.m port).

    `hold` — takeoff hold: while True the demo holds the arm with a PD +
    gravity-comp torque through the same effort path (the coupled law is only
    valid AIRBORNE — ground contact is not in the model, and running it on the
    ground flailed the arm and tumbled the vehicle, 2026-07-02 log). The law
    then zeros u3 before forming τ so tau_body carries no reaction N1ᵀ·u3 for
    an arm torque that is not being applied.
    """

    def __init__(self, params, gains=None):
        self.p = params
        self.g = gains or Gains()
        self.hold = False
        self._qdot_prev = None
        self._omega0_prev = None
        self._ei_y = np.zeros(4)   # EE-error integral accumulator (EE_INTEGRAL)

    def __call__(self, X, dyn, ref, dt):
        g = self.g; p = self.p; n = p["n"]; m = sum(p["m_i"]); gg = p["g"]
        r_0 = X[0:3]; R_0 = X[3:12].reshape(3, 3, order="F")
        omega_0 = X[15 + n:18 + n]; qdot = X[18 + n:18 + 2 * n]
        v_0 = X[12 + n:15 + n]
        V = np.concatenate([v_0, omega_0, qdot])
        om_hat = hat(omega_0); e3 = E3

        A = dyn["A"]
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


        if not OMEGA_E_FF or self._qdot_prev is None or dt <= 0:
            qddot = np.zeros(n); omega_0_dot = np.zeros(3)
        else:
            qddot = (qdot - self._qdot_prev) / dt
            omega_0_dot = (omega_0 - self._omega0_prev) / dt
        self._qdot_prev = qdot.copy(); self._omega0_prev = omega_0.copy()

        # omega_e_dot: the q̈ / ω̇0 feedforward terms are ZEROED — MATLAB closes
        # them with exact closed-loop accelerations; a finite-difference stand-in
        # fed noise into the high-gain EE-heading channel whenever the arm moved
        # (2026-07-02 reach chatter). Only the velocity-level transport terms
        # remain; the impedance absorbs the small feedforward error.
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

        # EE-error integral (toggle). Accumulate only once the arm is released, so it
        # doesn't wind up during the takeoff hold. Leaky (bleeds windup, bounds the
        # low-freq gain) + anti-windup clamp per axis.
        if EE_INTEGRAL and not self.hold and dt > 0:
            self._ei_y = np.clip((1.0 - EI_LEAK * dt) * self._ei_y + e_y * dt,
                                 -EI_CLAMP, EI_CLAMP)
        else:
            self._ei_y = np.zeros(4)
        ei_term = (g.K_i @ self._ei_y) if EE_INTEGRAL else np.zeros(4)

        F_trans = u1 * (R_0 @ e3) - m * gg * e3
        tau_rot = u2 - C_r @ omega_0 - C_rp @ rho
        # Full coupled EE law. tau_body carries the arm reaction N1ᵀ·u3 so the
        # rotors pre-compensate it. K_i·∫e_y removes any residual steady droop.
        coupling_ff = J_1y @ F_trans + J_2y @ tau_rot
        inner = (coupling_ff
                 + Lambda_y @ (J_y_dot @ xi - yddot_d)
                 + g.D_y @ e_vy + g.K_y @ e_y + ei_term)
        # Damped least-squares inverse of J_3y (see DLS_LAMBDA): caps the torque
        # gain of near-singular task directions (the q1/q4 heading pair).
        _JJt = J_3y @ J_3y.T + DLS_LAMBDA**2 * np.eye(4)
        _sol = lambda v: J_3y.T @ np.linalg.solve(_JJt, v)
        u3 = -_sol(inner) - C_rp.T @ omega_0 + C_p @ rho
        # Joint-space posture anchor (USE_POSTURE_ANCHOR): an EXTRA PD on u3, NOT in
        # the paper, tracking the trajectory's q_d (home q=0 when none). It was
        # INDISPENSABLE on the OLD mis-registered geometry, which over-recruited arm
        # gravity (~0.092 N·m on joint 3) and drifted the arm to a joint limit; with
        # the geometry-correct model that bias is gone, so the paper's law (anchor
        # OFF) is now viable. Flip USE_POSTURE_ANCHOR to choose which law runs.
        if USE_POSTURE_ANCHOR:
            q_arm = X[12:12 + n]
            q_d = ref.get("q_d", np.zeros(n))
            q_d_dot = ref.get("q_d_dot", np.zeros(n))
            u3 = u3 - POSTURE_KP * (q_arm - q_d) - POSTURE_KD * (qdot - q_d_dot)

        # TAKEOFF HOLD: the demo holds the arm itself (PD + gravity comp through
        # the effort path), so the task law's u3 is NOT applied. Zero it *before*
        # forming τ, so the body torque τ[3:6] does not inherit the coupling
        # N1ᵀ·u3 of an arm torque that isn't there (that phantom reaction was a
        # roll-runaway path). The body torque still carries the arm mass via the
        # CoM-offset term u1·hat(r0c)·e3 inside τ = Tᵀu.
        if self.hold:
            u3 = np.zeros(n)

        # physical wrench  τ = Tᵀ [u1·R0·e3 ; u2 ; u3]
        u = np.concatenate([u1 * (R_0 @ e3), u2, u3])
        tau = T.T @ u
        tau_joint = np.zeros(n) if self.hold else tau[6:]

        # --- arm-torque term breakdown (diagnostic) ---------------------------
        # tau_joint = u1·Aᵀe3 + u3. Split by source so the logs show which term
        # drives the arm: thrust-recruitment vs EE impedance vs integral vs the
        # coupling FF. Uses the same DLS solve as u3.
        t_thrust = u1 * (A.T @ e3)                         # body-loop recruitment
        t_imp    = -_sol(g.D_y @ e_vy + g.K_y @ e_y)       # EE impedance (hold)
        t_int    = -_sol(ei_term)                          # integral (droop null)
        t_cpl    = -_sol(coupling_ff)                      # J_1y·F_trans + J_2y·tau_rot
        return {
            "u1": u1, "u2": u2, "u3": u3,
            "thrust": u1, "tau_body": tau[3:6], "tau_joint": tau_joint,
            # arm-reaction pre-compensation inside tau_body. If the plant's arm
            # doesn't accelerate as modeled, this term goes uncancelled into the
            # attitude channel — log it against the observed body rates.
            "tau_body_arm": dyn["N1"].T @ u3,
            "R0c": R0c, "e_R": e_R, "e_y": e_y,
            "t_thrust": t_thrust, "t_imp": t_imp, "t_int": t_int, "t_cpl": t_cpl,
        }


# ===========================================================================
# Rotor mixer (Isaac-specific; not in the MATLAB plant)
# ===========================================================================

@dataclass
class RotorMixerParams:
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


# ===========================================================================
# ROS 2 node — for Linux / low-latency setups where the ROS2 round-trip is
# acceptable (on Windows/WSL2 the latency is fatal; use the in-process rig).
# Publishes ONE synchronized actuator packet [omega(4), tau_joint(n)] so the
# rotor wrench (which carries the arm reaction N1ᵀ·u3 in tau_body) and the arm
# torque land on the same plant step. The arm is torque-driven from the start:
# during takeoff the node publishes the software PD+gravity hold torque through
# the same packet (the plant keeps its arm dofs in effort mode throughout).
# ===========================================================================

if _HAS_ROS2:
    class MatlabControllerNode(Node):
        CLIMB_RATE  = 0.4    # [m/s] takeoff altitude ramp
        ARM_HOLD_KP = 3.0    # [N·m/rad]   takeoff hold (matches the in-process rig)
        ARM_HOLD_KD = 0.25   # [N·m·s/rad]

        def __init__(self):
            super().__init__("matlab_aerial_manipulator_controller")
            self.p = make_params()
            self.ctrl = MatlabController(self.p)
            self.ctrl.hold = True            # takeoff hold until airborne
            self.mixer = RotorMixer(RotorMixerParams())
            n = self.p["n"]
            self._R0 = np.eye(3); self._p0 = np.zeros(3)
            self._omega0 = np.zeros(3); self._v0 = np.zeros(3)
            self._q = np.zeros(n); self._qd = np.zeros(n)
            self._tr = None; self._t = 0.0; self._climb_z = 0.0   # set to x_c[2] at first pose
            self._dt = 0.01; self._ready = False; self._first = True; self._log = 0

            self.create_subscription(PoseStamped,  "state/pose",           self._pose_cb, qos_profile_sensor_data)
            self.create_subscription(TwistStamped, "state/twist_inertial", self._tlin_cb, qos_profile_sensor_data)
            self.create_subscription(TwistStamped, "state/twist",          self._tang_cb, qos_profile_sensor_data)
            self.create_subscription(JointState,   "joint_states",         self._joint_cb, 10)
            self._pub_act = self.create_publisher(Float64MultiArray, "actuator_command", 10)
            self.create_timer(self._dt, self._loop)
            self.get_logger().info("matlab controller node ready")

        def _build_X(self):
            return np.concatenate([self._p0, self._R0.flatten(order="F"),
                                   self._q, self._v0, self._omega0, self._qd])

        def _anchor(self, tag):
            """(Re)anchor the trajectory at the CURRENT CoM/EE pose."""
            dyn0 = dynamics(self._build_X(), self.p)
            x_c = self._p0 + self._R0 @ dyn0["r_0c_0"]
            x_c0 = np.array([x_c[0], x_c[1], TAKEOFF_ALTITUDE])
            d0 = self._R0 @ (dyn0["r_0e_0"] - dyn0["r_0c_0"])
            b1_0 = self._R0 @ np.array([1.0, 0, 0])
            self._tr = build_traj(x_c0, d0, b1_0)
            self.get_logger().info(f"{tag}: anchored '{self._tr['type']}' at "
                                   f"x_c={x_c.round(3)}")
            return x_c

        def _pose_cb(self, msg):
            o = msg.pose.orientation; pos = msg.pose.position
            self._p0 = np.array([pos.x, pos.y, pos.z])
            self._R0 = quat_to_rot(o.w, o.x, o.y, o.z)
            if not self._ready:
                x_c = self._anchor("first pose")
                self._climb_z = x_c[2]                            # start of climb ramp
                self._ready = True

        def _tlin_cb(self, msg):
            l = msg.twist.linear
            self._v0 = self._R0.T @ np.array([l.x, l.y, l.z])

        def _tang_cb(self, msg):
            a = msg.twist.angular
            self._omega0 = np.array([a.x, a.y, a.z])

        def _joint_cb(self, msg):
            n = self.p["n"]
            self._q = np.array(msg.position[:n]); self._qd = np.array(msg.velocity[:n])

        def _loop(self):
            if not self._ready or self._tr is None:
                return
            self._t += self._dt
            # phase 1: climb to TAKEOFF_ALTITUDE with the arm on the software
            # hold; phase 2: run the selected trajectory anchored at altitude.
            hold = self._t < TAKEOFF_TIME
            if hold != self.ctrl.hold:
                self.ctrl.hold = hold
                self.get_logger().info(f"arm hold -> {hold} (t={self._t:.1f}s)")
                if not hold:
                    # RE-ANCHOR at the hold→impedance handover so e_x=e_y=0 at
                    # the actual hover equilibrium (see the in-process rig).
                    self._anchor("handover")
            if hold:
                dz = TAKEOFF_ALTITUDE - self._climb_z
                step = max(-self.CLIMB_RATE * self._dt, min(self.CLIMB_RATE * self._dt, dz))
                self._climb_z += step
                # Velocity feedforward: with x_cd_dot=0 the translation damping
                # (k_v) and EE damping (D_y) both fight the climb → e_y ramps.
                vz = step / self._dt if self._dt > 0 else 0.0
                z3 = np.zeros(3)
                ref = {"x_cd": np.array([self._tr["x_c0"][0], self._tr["x_c0"][1], self._climb_z]),
                       "x_cd_dot": np.array([0.0, 0.0, vz]), "x_cd_ddot": z3,
                       "x_cd_d3": z3, "x_cd_d4": z3}
                ref = _fixed_ee(ref, self._tr)
            else:
                ref = generate_reference(self._t - TAKEOFF_TIME, self._tr)
            try:
                X = self._build_X()
                dyn = dynamics(X, self.p)
                res = self.ctrl(X, dyn, ref, self._dt)
                omega = self.mixer.mix(res["thrust"], res["tau_body"])
                if hold:
                    # software hold torque through the SAME effort path the
                    # impedance uses — the plant never switches drive modes
                    tau_j = (-self.ARM_HOLD_KP * self._q
                             - self.ARM_HOLD_KD * self._qd + dyn["g"][6:])
                else:
                    tau_j = res["tau_joint"]
                m = Float64MultiArray()
                m.data = omega.tolist() + np.asarray(tau_j, float).tolist()
                self._pub_act.publish(m)
                if self._first:
                    self._first = False
                    self.get_logger().info(
                        f"first ctrl: thrust={res['thrust']:.2f} omega={omega.round(1)}")
                self._log += 1
                if self._log >= 100:
                    self._log = 0
                    self.get_logger().info(
                        f"z={self._p0[2]:.3f} thrust={res['thrust']:.1f}N "
                        f"e_R={res['e_R'].round(3)} e_y={res['e_y'].round(3)}")
            except Exception as exc:
                self.get_logger().error(f"loop: {exc}")

    def main(args=None):
        rclpy.init(args=args)
        rclpy.spin(MatlabControllerNode())
        rclpy.shutdown()

    if __name__ == "__main__":
        main()


# ===========================================================================
# Standalone sanity checks (plain python; no Isaac/ROS2 needed)
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

    # transform identity — must hold to machine precision (validates N1/T/M̃)
    err = (np.linalg.norm(dyn["M"] - dyn["T"].T @ dyn["M_tilde"] @ dyn["T"])
           / np.linalg.norm(dyn["M"]))
    print(f"|M - Tᵀ·M̃·T|/|M| = {err:.2e}  (expect ~1e-17)")
