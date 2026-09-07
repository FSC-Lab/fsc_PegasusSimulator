#!/usr/bin/env python3
"""
controller.py — THE controller. One law, every scenario.

Author: Ben Natra (send2ben123@gmail.com)
Author: Shiqi Gao (shiqi.gao907@gmail.com)

Merged 2026-08-09 from controller_free.py / controller_pick.py /
controller_push.py, which were three copies of this same law. Measured before
the merge, ignoring comments: make_params, dynamics, the Gains dataclass and
the rotor mixer were byte-identical across all three, and MatlabController
differed only in that `free` had two things the others never received — the
`gmo_inhibit` flag and the saturation-consistent coupling (clamp tau, back out
the REALIZED u3, rebuild the base moment from it). The copies had also drifted
apart on a real control parameter without anyone deciding to: DLS_LAMBDA read
0.3 in one and `0#.3` — zero, a digit commented out mid-token — in the other
two. That is the failure mode this file exists to end.

WHAT VARIES BETWEEN SCENARIOS IS NOW ONLY NUMBERS, and the numbers live in
robotic_arm/config/<scenario>.yaml:

    from ...utils_controller import controller as C, control_params as CP
    cfg  = CP.load("pick")                       # or "free" / "push" / …
    ctrl = C.MatlabController(C.make_params(), cfg)

There are no gain defaults in this module — a scenario file that is missing a
key fails at startup rather than flying something nobody chose. The free-flight
demo additionally passes its trajectory type as a variant, so one mode can
carry its own gains (CP.load("free", variant="poly_whole")).

WHAT DOES NOT BELONG HERE: flight phases. Takeoff, the task handover, landing,
the arm hold and the rotor ramp-off are the DEMO's business (they are about a
particular vehicle on a particular ground); this module only answers "given the
state and the reference right now, what are thrust, base moment and joint
torques?". The trajectories themselves live in utils_planner/.

Its sibling controller_track.py stays separate on purpose — it is a different
law (joint-space posture anchor, no observer), the A/B baseline, not a
parameterization of this one.

ARM GEOMETRY NOTE: make_params registers the joint chain so joint k pivots about
manip_joint_k (i.e. O[k-1] = manip_joint_k), deriving l_i/com_i from the raw
AM_realign extraction. That raw extraction lists an extra base->manip_joint1
offset, so feeding it in naively would pivot every arm joint one link too
PROXIMAL — over-estimating manip_joint3's gravity by ~0.092 N·m and, with no
posture anchor, folding the arm to its +50° joint limit. The re-registration in
make_params avoids that while holding every body CoM and the EE point fixed
(verified: g[6:]=[0,0.522,0.416,0], r_0e_0/r_0c_0 unchanged). See make_params.

This is the sibling of `controller_track.py`, updated to the LATEST MATLAB
design (`refs_matlab/utils/utils_control/controller.m` + `main_sim.m`, see
`doc/Control flowchart.pdf`).  Two deliberate differences from the track port:

  1. The joint-space posture-anchor PD (POSTURE_KP/KD) that `controller_track.py`
     added on top of the task law is REMOVED.  The MATLAB law has no such term;
     the momentum observer below is meant to replace the job it was doing
     (fighting the gravity-map residual).
  2. A GENERALIZED-MOMENTUM DISTURBANCE OBSERVER (GMO) is added.  It estimates
     the transformed end-effector disturbance
          d_e_hat = K_o (p − p_hat),   p = M̃·ξ   (transformed momentum)
     by integrating the observer copy
          p_hat_dot = C̃ᵀ·ξ − g̃ + u + d_e_hat
     and the controller cancels d_t_hat in the thrust law (u1), d_r_hat in the
     base-moment law (u2), and — only in the SHAPED impedance mode — injects
     F_hat_y = (J_y^#)ᵀ d_e_hat into the arm law (u3).  (PDF "Control —
     Disturbance Observer (GMO)".)

Direct, function-structured port of the MATLAB pipeline:

    dynamics(X, params)          <- dynamics.m   (M,C,g, M̃,C̃,g̃, T, J_y, ...)
    controller(X, dyn, ref, dt)  <- controller.m (u1, u2, u3, GMO)
    tau = Tᵀ [u1·R0·e3 ; u2 ; u3]                <- closed_loop_dynamics.m

State vector (MATLAB layout, length 18+2n):
    X = [ r0(3); vec(R0)(9, col-major); q(n); v0(3); omega0(3); qdot(n) ]

The MATLAB ran as a single-process, zero-latency closed loop (controller and
plant share M,C,g), and integrated the GMO state p_hat as part of the ODE.
Here `dynamics`+`controller` are the controller and Isaac is the plant, so the
observer momentum p_hat is carried ON THE CONTROLLER INSTANCE and advanced by a
forward-Euler step (dt = the physics step) each call.

This module is a pure LIBRARY — it owns the MODEL (make_params, dynamics) and
the CONTROL LAW (MatlabController, RotorMixer), and nothing else. The desired
trajectories moved out to the utils_planner/ package next to this file, so the
reference the controller tracks has exactly one source. It is imported and
stepped IN-PROCESS from a physics-step callback (250 Hz, dt = the step) by:

  02_aerial_manipulator_free.py                     rotors driven directly
  03_px4_direct_aerial_manipulator_free.py          rotors gated through PX4

(The ROS2 `MatlabControllerNode` this file used to carry was dropped: nothing
launched it, it duplicated the demos' takeoff/handover phase logic, and it
existed for a Windows/WSL2 setup that is no longer in use. `python3
controller_free.py` now always runs the standalone sanity checks at the bottom.)

The arm runs in true effort mode from the start, with a software
PD+gravity hold during takeoff (`MatlabController.hold`).  The GMO is held reset
(p_hat = p, d_e_hat = 0) during that hold and only runs once airborne, so the
software-hold torque — which is NOT the law's u3 — cannot corrupt the estimate.
"""

import numpy as np
from dataclasses import dataclass, field


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
    # These are the ORIGINAL values (identical to controller_track.py's l_i/com_i)
    # — the offset chain straight from the asset, with the RAW chain point list
    #     O_raw = [ base, manip_joint1, manip_joint2, manip_joint3, manip_joint4 ].
    # raw_l[k]   = O_raw[k+1] - O_raw[k]  (link-offset vectors down the chain)
    # raw_com[k] = O_raw[k]   -> CoM of link k+1  (link's CoM from its proximal point)
    # The catch that motivates everything below: raw_l[0] is a base->manip_joint1
    # offset, so O_raw[k] = manip_joint_k, i.e. joint k sits at O_raw[k] — but
    # dynamics() pivots joint k about O[k-1]. Feeding raw_l straight in therefore
    # mis-registers every joint one link too proximal (the joint-3 gravity bug).
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
    # O_raw[k] = sum(raw_l[:k]); joint k anchor = O_raw[k] = joint_pos[k-1];
    # link k CoM = O_raw[k-1] + raw_com[k-1] = link_com[k-1].
    O_raw = [np.zeros(3)]
    for d in raw_l:
        O_raw.append(O_raw[-1] + d)
    joint_pos = O_raw[1:]                                       # manip_joint1..4 (absolute)
    link_com  = [O_raw[k] + raw_com[k] for k in range(n)]       # link k+1 CoM   (absolute)

    # ── Step 2: RE-REGISTER into the convention dynamics() expects ──────────────
    # dynamics() needs joint k at O[k-1] and the EE at O[n], with O_0 at the base.
    # So shift the chain up by one to O[k-1]=manip_joint_k. manip_joint1 (a vertical
    # +z axis -> zero gravity moment) is IDEALIZED onto O_0, its base->joint1 offset
    # folding harmlessly into l_i[0] — mirroring main_sim.m, where joint 1 sits at
    # the base. Result:  O = [ base(0), manip_joint2, manip_joint3, manip_joint4, EE ].
    # This is the whole fix: it moves joint 3's pivot from O_raw[2]=manip_joint2 to
    # manip_joint3, removing the l_i[2].y·(mass below)·g ≈ 0.092 N·m gravity
    # over-estimate that (with no posture anchor) lifted the arm to its +50° limit.
    ee_pos  = joint_pos[3]                                      # EE = the wrist (manip_joint4)
    O_chain = [np.zeros(3)] + joint_pos[1:] + [ee_pos]          # O_0..O_n  (len n+1)
    l_i   = [O_chain[k] - O_chain[k - 1] for k in range(1, n + 1)]     # O_{k-1} -> O_k
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
# GMO / impedance-mode configuration  (mirrors main_sim.m Section 1b/1c).
#
# NOTE (posture anchor removed): controller_track.py added a joint-space
# posture-anchor PD to u3.  It is GONE here — the MATLAB law has no such term.
# Its job (rejecting the gravity-map residual so the arm doesn't fold to a
# joint limit) is meant to be taken over by the GMO.  In NATURAL impedance mode
# (M_Y = None) the GMO does NOT inject into the arm channel (see the F_hat_y
# note in the Arm section), so the arm relies on the task impedance (D_y,K_y)
# alone to hold — if it drifts on this plant, switch to SHAPED mode below (which
# activates the arm GMO compensation), matching main_sim.m's latest default.
#
# USE_GMO — master on/off for the generalized-momentum observer.  True: estimate
#   the transformed EE disturbance and cancel d_t_hat in u1, d_r_hat in u2 (and
#   the shaped-mode arm term).  False: d_e_hat = 0 (nominal controller, no
#   disturbance compensation) — the GMO-vs-no-GMO A/B baseline.
# ---------------------------------------------------------------------------
# CONTROL PARAMETERS LIVE IN YAML, NOT HERE.
#
# Everything that used to be a module global in this block — USE_GMO,
# IMPEDANCE_MODE, M_Y, DLS_LAMBDA, TAU_MAX, OMEGA_E_FF — plus every gain that
# used to be a `Gains` dataclass default is now one key in
# robotic_arm/config/<scenario>.yaml, loaded into a ControlParams and handed to
# MatlabController. See control_params.py for the schema and the reason there
# are no defaults in code (three copies of this file had already drifted apart
# on DLS_LAMBDA without anyone deciding to).
#
# The law below reads them as self.cfg.<key> — nothing else in this module
# holds a tunable number.
# ---------------------------------------------------------------------------


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
    # r0i[0] is the BASE link's CoM. Zero (i.e. the base CoM sits on the body
    # origin, which is the geometric rotor centre) unless params carries a
    # measured "base_com" — see transition_planner.make_params_t650 and the
    # C++ wb_base_com_*. Absent key => the flight-validated behaviour.
    r0i = [np.asarray(params.get("base_com", np.zeros(3)), dtype=float)]
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
    # Full transformed Coriolis C̃ (block layout from dynamics.m): the
    # translation row/col is zero, and the arm↔rotation coupling is
    # antisymmetric (−C_rpᵀ). The GMO momentum observer needs C̃ᵀ·ξ.
    C_tilde = np.zeros((n + 6, n + 6))
    C_tilde[3:6, 3:6] = C_r
    C_tilde[3:6, 6:]  = C_rp
    C_tilde[6:, 3:6]  = -C_rp.T
    C_tilde[6:, 6:]   = C_p

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
        "M_tilde": M_tilde, "C_tilde": C_tilde, "g_tilde": g_tilde, "T": T,
        "M": M, "C": C, "g": g,
        "A": A, "N1": N1, "r_0c_0": r_0c_0, "r_0e_0": r_0e_0, "R_e_0": R_e_0,
        # A_e: arm-only EE Jacobian in the BASE frame, rdot_0e^0 = A_e qdot.
        # Additive (2026-09-06) — the L1 augmentation's contact decomposition
        # needs the full end-effector Jacobian J_e, whose arm block this is.
        # No existing consumer reads it; _DYN_KEYS in generate_wb_truth.py is
        # an explicit list, so the parity fixture is unaffected.
        "A_e": A_e,
        "J_y": J_y, "J_y_dot": J_y_dot, "Lambda_y": Lambda_y,
        "J_1y": J_1y, "J_2y": J_2y, "J_3y": J_3y,
        "J_q_omega_e": Jq_e, "J_q_dot_omega_e": Jqd_e, "omega_0e_0": w0e,
    }


# ===========================================================================
# Trajectories — MOVED to utils_planner/  (2026-08-08)
# ===========================================================================
# TRAJ_CONFIG, build_traj, generate_reference, the takeoff ramp and the
# compatible-plan hook used to live here. They are now the single-source
# trajectory package next to this file:
#
#     from fsc_aerial_manipulation.robotic_arm import utils_planner as P
#     P.set_traj_type("poly_whole")
#     tr  = P.build_traj(x_c0, d0, b1_0, params=params)
#     ref = P.generate_reference(t, tr)            # P.takeoff_reference() for the climb
#
# This module is now the MODEL and the CONTROL LAW only: make_params, dynamics,
# MatlabController and RotorMixer. It does not know what trajectory is flying,
# and nothing here should import utils_planner — the dependency runs one way,
# planner -> model (build_traj needs make_params for the compatible solve).

# ===========================================================================
# controller(X, dyn, ref, params)  — faithful port of controller.m
# ===========================================================================

# `Gains` used to live here as a dataclass of defaults. It is now
# ControlParams in control_params.py, built from the scenario YAML — same
# attribute names (k_x, k_v, k_R, k_w, M_r_d, K_y, D_y, K_o), so the law body
# below is unchanged.


class MatlabController:
    """The coupled feedback-linearizing law + GMO observer (controller.m port).

    `hold` — takeoff hold: while True the demo holds the arm with a PD +
    gravity-comp torque through the same effort path (the coupled law is only
    valid AIRBORNE — ground contact is not in the model, and running it on the
    ground flailed the arm and tumbled the vehicle, 2026-07-02 log). The law
    then zeros u3 before forming τ so tau_body carries no reaction N1ᵀ·u3 for
    an arm torque that is not being applied.  The GMO is also held reset while
    `hold` is True (p_hat = p, d_e_hat = 0) so the software-hold torque — which
    is not the law's u3 — cannot corrupt the momentum estimate.

    The hold↔law switch needs no torque blend, PROVIDED the caller switches at a
    CONSTANT reference and only after a settled hover (the demo's sequencing
    rule).  Measured at both of 02_aerial_manipulator_free.py's switch states:
    with e_y = 0, everything at rest and F_trans = 0, the law's u3 is ~0 and its
    arm torque reduces to the same gravity term the PD hold applies — the step
    is 0.005 N·m at takeoff and 0.000 at landing, i.e. 0.1% of TAU_MAX.  What
    genuinely hurts is stepping the REFERENCE while the law still owns the arm:
    a 1.17 m setpoint drop cuts thrust, F_trans ≈ −mg floods u3 through the
    coupling feedforward, and the demanded arm torque hits 90 N·m (22× the
    clamp).  Sequence the two, and no blend is needed.

    `gmo_inhibit` — hold the observer reset without zeroing u3.  Lets a caller
    freeze the estimate on its own schedule (the demo asserts it exactly while a
    hold phase owns the arm) independently of `hold`.

    GMO state: `_p_hat` is the observer momentum p_hat ∈ R^{6+n}, integrated by
    forward Euler each call.  It is (re)initialized to p = M̃·ξ on the first
    airborne call so d_e_hat starts at zero (matching main_sim's p_hat(0)=p(0)).
    """

    def __init__(self, params, cfg):
        """params: make_params() model.  cfg: ControlParams from the scenario
        YAML (control_params.load) — REQUIRED, there is no default gain set."""
        self.p = params
        self.cfg = cfg
        self.g = cfg          # the law reads gains as g.<name>; ControlParams
                              # carries exactly those attribute names
        self.hold = False
        self.gmo_inhibit = False   # hold the observer reset without zeroing u3
        self._qdot_prev = None
        self._omega0_prev = None
        self._p_hat = None         # GMO observer momentum p_hat (6+n); None until init

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

        # ---- GMO disturbance observer: estimate (state derivative formed later) --
        # p = M̃·ξ (transformed momentum); d_e_hat = K_o (p − p_hat) is a
        # first-order low-pass of the true transformed disturbance. Split into the
        # translational / rotational / arm parts the three control laws consume.
        # The observer is HELD RESET while `hold` is True (or USE_GMO off): p_hat
        # tracks p so d_e_hat = 0 and it is primed to run cleanly at the handover.
        # `gmo_inhibit` does the same without zeroing u3 — the demo uses it to keep
        # the observer reset for a beat AFTER the handover, so the switching
        # transient is not mistaken for a disturbance and fought.
        M_tilde = dyn["M_tilde"]; C_tilde = dyn["C_tilde"]; g_tilde = dyn["g_tilde"]
        pmom = M_tilde @ xi
        gmo_active = self.cfg.use_gmo and not self.hold and not self.gmo_inhibit
        if not gmo_active:
            self._p_hat = pmom.copy()
            d_e_hat = np.zeros(6 + n)
        else:
            if self._p_hat is None:
                self._p_hat = pmom.copy()
            d_e_hat = g.K_o @ (pmom - self._p_hat)
        d_t_hat = d_e_hat[0:3]        # translational -> cancelled in u1
        d_r_hat = d_e_hat[3:6]        # rotational    -> cancelled in u2
        d_rho_hat = d_e_hat[6:6 + n]  # arm           -> used in u3 via F_hat_y (shaped)

        # ---- translation ----
        # d_t_hat cancelled in f_d so the realized thrust carries the estimated
        # load once the GMO has converged (only the zeroth-order force is
        # compensated; f_d_dot / f_d_ddot keep their reference-only form because
        # a near-constant inertial load has d_t_hat_dot ≈ 0).
        x_c = r_0 + R_0 @ r_0c_0
        e_x = x_c - ref["x_cd"]
        e_vx = xc_dot - ref["x_cd_dot"]
        f_d = -g.k_x * e_x - g.k_v * e_vx + m * ref["x_cd_ddot"] + m * gg * e3 - d_t_hat
        u1 = float(f_d.T @ (R_0 @ e3))
        # plant CoM accel carries the disturbance: m·xc_ddot = u1·R0·e3 − m·g·e3 + d_t
        xc_ddot = (u1 / m) * (R_0 @ e3) - gg * e3 + (1.0 / m) * d_t_hat
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
        # base-moment law, with the estimated rotational disturbance d_r_hat cancelled
        u2 = (M_r @ (R_0.T @ R0c @ omega_0c_dot - om_hat @ R_0.T @ R0c @ omega_0c
                     - np.linalg.solve(g.M_r_d, g.k_R * e_R + g.k_w * e_w))
              + C_r @ omega_0 + C_rp @ rho - d_r_hat)

        # ---- arm ----
        R_e = R_0 @ R_e_0
        R_0e = R_e_0.T
        r_e = r_0 + R_0 @ r_0e_0
        omega_e = R_0e @ (omega_0 + omega_0e_0)
        omega_e_hat = hat(omega_e)


        if not self.cfg.omega_e_ff or self._qdot_prev is None or dt <= 0:
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

        # ---- desired task inertia M_y ----
        # natural (M_Y = None): M_y = Lambda_y -> Lam_Myinv = I, so the impedance
        #   is unscaled and the GMO arm term below vanishes (arm disturbance left
        #   to D_y,K_y).  shaped (M_Y != Lambda_y): imposes a design task inertia,
        #   which the −(Lam_Myinv−I)·F_hat_y term realizes using the GMO estimate.
        M_y = Lambda_y if self.cfg.M_Y is None else self.cfg.M_Y
        Lam_Myinv = Lambda_y @ np.linalg.solve(M_y, np.eye(4))   # Lambda_y·M_y⁻¹
        # estimated task-space disturbance force  F_hat_y = (J_y^#)ᵀ d_e_hat
        F_hat_y = J_1y @ d_t_hat + J_2y @ d_r_hat + J_3y @ d_rho_hat

        F_trans = u1 * (R_0 @ e3) - m * gg * e3
        tau_rot = u2 - C_r @ omega_0 - C_rp @ rho
        # Full coupled EE law (controller.m). tau_body carries the arm reaction
        # N1ᵀ·u3 so the rotors pre-compensate it.
        coupling_ff = J_1y @ F_trans + J_2y @ tau_rot
        inner = (coupling_ff
                 + Lambda_y @ (J_y_dot @ xi - yddot_d)
                 + Lam_Myinv @ (g.D_y @ e_vy + g.K_y @ e_y)
                 - (Lam_Myinv - np.eye(4)) @ F_hat_y)
        # Damped least-squares inverse of J_3y (see DLS_LAMBDA): λ=0 recovers the
        # MATLAB plain solve (J_3y \ inner); λ>0 caps the torque gain of
        # near-singular task directions (the q1/q4 heading pair) — kept as a
        # margin now that the posture anchor that used to pin them is gone.
        _JJt = J_3y @ J_3y.T + self.cfg.dls_lambda**2 * np.eye(4)
        _sol = lambda v: J_3y.T @ np.linalg.solve(_JJt, v)
        u3 = -_sol(inner) - C_rp.T @ omega_0 + C_p @ rho
        # NOTE: controller_track.py's joint-space posture-anchor PD is REMOVED —
        # the MATLAB law has none, and the GMO is meant to reject the gravity-map
        # residual that anchor was fighting.

        # TAKEOFF HOLD: the demo holds the arm itself (PD + gravity comp through
        # the effort path), so the task law's u3 is NOT applied. Zero it *before*
        # forming τ, so the body torque τ[3:6] does not inherit the coupling
        # N1ᵀ·u3 of an arm torque that isn't there (that phantom reaction was a
        # roll-runaway path). The body torque still carries the arm mass via the
        # CoM-offset term u1·hat(r0c)·e3 inside τ = Tᵀu.
        # HOLD: the caller drives the arm itself (PD + gravity comp through the
        # same effort path), so the task law's u3 is NOT applied. Zero it *before*
        # forming τ, so the body torque τ[3:6] does not inherit the coupling
        # N1ᵀ·u3 of an arm torque that isn't there (that phantom reaction was a
        # roll-runaway path). The body torque still carries the arm mass via the
        # CoM-offset term u1·hat(r0c)·e3 inside τ = Tᵀu.
        if self.hold:
            u3 = np.zeros(n)

        # physical wrench  τ = Tᵀ [u1·R0·e3 ; u2 ; u3]   (u is the transformed wrench)
        # blocks:  τ_joint = τ[6:]  = u3 + u1·Aᵀe3           (what the servos apply)
        #          τ_body  = τ[3:6] = u2 + u1·hat(r0c)e3 + N1ᵀ·u3   (base moment;
        #                    the N1ᵀ·u3 term is the arm's reaction = the coupling)
        u = np.concatenate([u1 * (R_0 @ e3), u2, u3])
        tau = T.T @ u
        tau_joint = np.zeros(n) if self.hold else tau[6:]

        # ---- SERVO SATURATION, made coupling-consistent ----------------------
        # In the MATLAB the joint torque is applied exactly, so the N1ᵀ·u3 the
        # base moment carries is always what the arm really produces. Real
        # servos clamp at ±TAU_MAX, and feeding the base moment the DEMANDED u3
        # while the arm delivers less commands an attitude reaction that never
        # arrives — the documented flip path on this rig (a stepped reference
        # once demanded 90 N·m, 22x the clamp, on two joints at once).
        # So: clamp τ_joint, back the REALIZED u3 out of τ_joint = u3 + u1·Aᵀe3,
        # and rebuild the base moment from it. Attitude and arm stay consistent
        # while the law stays fully coupled. No-op when nothing saturates.
        n_sat = 0
        if not self.hold and self.cfg.tau_max is not None:
            _tm = self.cfg.tau_max
            tau_clamped = np.clip(tau_joint, -_tm, _tm)
            n_sat = int(np.count_nonzero(tau_clamped != tau_joint))
            if n_sat:
                tau_joint = tau_clamped
                u3 = tau_joint - u1 * (A.T @ e3)          # realized u3
                u = np.concatenate([u1 * (R_0 @ e3), u2, u3])
                tau = T.T @ u                             # coupled + consistent

        # ---- GMO observer: advance p_hat one forward-Euler step (formed AFTER u,
        # it depends on u).  p_hat_dot = C̃ᵀ·ξ − g̃ + u + d_e_hat.  Only while
        # airborne + enabled — during the hold gmo_active is False and p_hat was
        # already synced to p above (d_e_hat = 0), so no windup on the hold torque.
        if gmo_active and dt > 0:
            p_hat_dot = C_tilde.T @ xi - g_tilde + u + d_e_hat
            self._p_hat = self._p_hat + p_hat_dot * dt

        # --- arm-torque term breakdown (diagnostic) ---------------------------
        # tau_joint = u1·Aᵀe3 + u3. Split by source so the logs show which term
        # drives the arm: thrust-recruitment vs EE impedance vs GMO comp vs the
        # coupling FF. Uses the same DLS solve as u3.
        t_thrust = u1 * (A.T @ e3)                                   # body-loop recruitment
        t_imp    = -_sol(Lam_Myinv @ (g.D_y @ e_vy + g.K_y @ e_y))   # EE impedance
        t_dist   = -_sol(-(Lam_Myinv - np.eye(4)) @ F_hat_y)         # GMO arm comp (shaped)
        t_cpl    = -_sol(coupling_ff)                                # J_1y·F_trans + J_2y·tau_rot
        return {
            "u1": u1, "u2": u2, "u3": u3,
            "thrust": u1, "tau_body": tau[3:6], "tau_joint": tau_joint,
            # arm-reaction pre-compensation inside tau_body. If the plant's arm
            # doesn't accelerate as modeled, this term goes uncancelled into the
            # attitude channel — log it against the observed body rates.
            "tau_body_arm": dyn["N1"].T @ u3,
            "R0c": R0c, "e_R": e_R, "e_y": e_y,
            "t_thrust": t_thrust, "t_imp": t_imp, "t_dist": t_dist, "t_cpl": t_cpl,
            # GMO disturbance estimates (transformed frame): translation -> u1,
            # rotation -> u2, arm -> u3 (shaped mode only). gmo_active is False
            # during the takeoff hold / when USE_GMO is off.
            "d_t_hat": d_t_hat, "d_r_hat": d_r_hat, "d_rho_hat": d_rho_hat,
            "d_e_hat": d_e_hat, "gmo_active": gmo_active,
            # how many joints hit ±TAU_MAX this step. Non-zero means the law is
            # asking for more than the servos can give: the arm tracks worse
            # than commanded and the caller should treat it as a fault signal,
            # not a routine event (tau_body IS consistent with it either way).
            "n_sat": n_sat,
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
# Standalone sanity checks (plain python; no Isaac/ROS2 needed)
# ===========================================================================

if __name__ == "__main__":
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

    # local import: the planner depends on this module, never the reverse.
    # Run as a bare script sys.path[0] is utils_controller/, so walk up to the
    # extension root (utils_controller -> robotic_arm -> package -> root).
    try:
        from fsc_aerial_manipulation.robotic_arm import utils_planner as P
    except ModuleNotFoundError:
        import os, sys
        _here = os.path.abspath(__file__)
        for _ in range(4):
            _here = os.path.dirname(_here)
        sys.path.insert(0, _here)
        from fsc_aerial_manipulation.robotic_arm import utils_planner as P
    r0 = X[0:3]; R0_ = X[3:12].reshape(3, 3, order="F")
    x_c = r0 + R0_ @ dyn["r_0c_0"]
    tr = P.build_traj(x_c, R0_ @ (dyn["r_0e_0"] - dyn["r_0c_0"]),
                      R0_ @ np.array([1.0, 0, 0]), "hover_drone", params=params)
    ref = P.generate_reference(0.0, tr)
    # Parameters come from a scenario file like any other run — no hidden
    # defaults, not even in the self-test. Name one on the command line to
    # check any scenario:  python controller.py pick
    import sys as _sys
    from fsc_aerial_manipulation.robotic_arm.utils_controller import (
        control_params as CP)
    scenario = _sys.argv[1] if len(_sys.argv) > 1 else "free"
    cfg = CP.load(scenario)
    ctrl = MatlabController(params, cfg)
    res = ctrl(X, dyn, ref, 0.01)
    print(f"hover thrust  = {res['thrust']:.3f} N  (m·g = {sum(params['m_i'])*9.81:.3f})")
    print(f"tau_body      = {res['tau_body'].round(4)}")
    print(f"tau_joint     = {res['tau_joint'].round(4)}")
    print(f"e_y           = {res['e_y'].round(4)}")

    # transform identity — must hold to machine precision (validates N1/T/M̃)
    err = (np.linalg.norm(dyn["M"] - dyn["T"].T @ dyn["M_tilde"] @ dyn["T"])
           / np.linalg.norm(dyn["M"]))
    print(f"|M - Tᵀ·M̃·T|/|M| = {err:.2e}  (expect ~1e-17)")

    # ---- GMO sanity ----
    # C_tilde must be exported and block-structured (translation row/col zero).
    Ct = dyn["C_tilde"]
    print(f"C_tilde shape = {Ct.shape}  (expect ({n+6},{n+6}))")
    print(f"C_tilde trans-row/col zero = "
          f"{np.allclose(Ct[0:3, :], 0) and np.allclose(Ct[:, 0:3], 0)}")
    # p_hat initializes to p on the first airborne call -> d_e_hat starts at 0.
    print(f"use_gmo={cfg.use_gmo}  impedance={cfg.impedance_mode}  "
          f"M_Y={'None' if cfg.M_Y is None else 'shaped'}  "
          f"tau_max={cfg.tau_max}")
    print(f"d_e_hat(0)    = {res['d_e_hat'].round(6)}  (expect ~0 at init)")
    print(f"gmo_active    = {res['gmo_active']}  (True: airborne + use_gmo)")
