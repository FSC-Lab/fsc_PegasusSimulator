#!/usr/bin/env python3
"""make_arbitrary_time_yaml.py -- generate the arbitrary-convergence-time (Chen
et al., IEEE TIE 72(12) 2025) rig's SIMULATION yaml from the 4-D whole-body
box-pushing sim yaml (2026-10-24).

Everything the two rigs SHARE is copied verbatim -- the plant (section 1, the
sim_ keys the Pegasus launcher reads), the SAFETY loop + UDE, the guards, the
wrench allocator and the trajectory-planner sections -- so the only differences
between the rigs are the DIRECT law and its interface keys:

  * sections 2.4-2.7 (the whole-body law, its observer, the 4-D attribution and
    the arm interface/gates) are REPLACED by the paper's four modules (atc_*)
    and this node's arm/planner interface (atc_*);
  * every other wb_ key (section 0's arm topics, section 3's hold stream) is
    dropped -- the atc_ interface block carries the same topics;
  * the raw-mocap guard's two keys are dropped (GUARD_KEYS, the coordinated-
    admittance generator's reason: only the whole-body box-pushing stacks start
    the estimator with mocap_guard:=true, and a planner given
    push_pull_mocap_guard_topic with no guard publishing refuses every step).

The planner is pointed at THIS rig by the stack script on its command line
(mode_topic, reference_topic, push_pull_contact_service, push_pull_force_topic),
not here, so the planner section stays byte-identical to the whole-body one.

Checks: the sim_ plant keys and the planner sections are identical to the
source's (bar the guard key), no wb_ key survives, every required atc_ key is
present, the arm model's base_com / armature equal the planner's.

    /usr/bin/python3 make_arbitrary_time_yaml.py [--check]
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
SRC = "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_box_pushing.yaml"
DST = "params_single_aerial_manipulator_arbitrary_time_direct_actuation_t650_sim_box_pushing.yaml"
VEHICLE = "AM-T650-ATC-PUSH"

# ---- the paper's four modules (the flight node) ------------------------------
# GAINS. The paper's experiment lists eta_u 2.2/2.75 (one pair for all six
# channels), eta_m 5.3/4.8, k 8.2, eta_o 2.11/2.63, c [20.9, 31.1, 33.3, 3.1,
# 4.22, 3.75], H/D/K_d 1/4/9, DT_u = DT_m = 6 s, DT_o = 4 s -- for a 4.1 kg
# vehicle whose attitude was most likely PX4's (MAVLink offboard). Here the
# quadrotor's attitude rows drive the rotors directly, so (README section 2):
#   * the post-window gains are eta/T_floor (Python addition (a)); the per-channel
#     etas below put the floored loop at the bandwidths the other decoupled rigs
#     fly on this plant: position wn 2.0 rad/s zeta 0.87 (x, y), 2.7 / 0.93 (z),
#     attitude wn 7.1 rad/s zeta 0.99 (roll, pitch), 5.1 / 0.98 (yaw) -- the
#     coordinated-admittance rig's 4/3.5, 8/5, 5/1.6 and 4/0.8 in these units;
#   * the windows are short (DT_u 1.2 s, DT_m 1.5 s, DT_o 0.8 s) so the gains
#     start no lower than eta/DT at DIRECT entry (the prescribed-time gain RISES
#     toward the deadline; with the paper's 6 s the attitude loop would start at
#     wn ~1 rad/s);
#   * c for the attitude rows 0.1 (the paper's 3.1-4.2 on this inertia adds
#     ~26 N.m/rad of stiffness through the fuzzy integral -- 4x the law's own);
#   * Gamma_o1/Gamma_o2 (not given in the paper) 25 / 25 rad^2/s^2.
LAW = [
    ("atc_quad_eta1_x", 1.04), ("atc_quad_eta1_y", 1.04), ("atc_quad_eta1_z", 1.5),
    ("atc_quad_eta1_roll", 4.2), ("atc_quad_eta1_pitch", 4.2), ("atc_quad_eta1_yaw", 3.0),
    ("atc_quad_eta2_x", 1.04), ("atc_quad_eta2_y", 1.04), ("atc_quad_eta2_z", 1.5),
    ("atc_quad_eta2_roll", 4.2), ("atc_quad_eta2_pitch", 4.2), ("atc_quad_eta2_yaw", 3.0),
    ("atc_quad_window_s", 1.2), ("atc_quad_floor_s", 0.6), ("atc_quad_ff_gain", 1.0),
    ("atc_inertia_xx", 0.118), ("atc_inertia_yy", 0.119), ("atc_inertia_zz", 0.131),
    ("atc_inertia_xy", 0.0089), ("atc_inertia_xz", 0.0), ("atc_inertia_yz", 0.0),
    ("atc_max_pos_err_m", 1.0), ("atc_max_vel_err_mps", 2.0), ("atc_max_att_err_rad", 0.6),
    ("atc_max_tilt_deg", 30.0), ("atc_min_thrust_frac", 0.3), ("atc_max_torque_xy", 8.0),
    ("atc_max_torque_z", 0.55),
    # TSKFE
    ("atc_fuzzy_enable", True),
    ("atc_fuzzy_c_x", 20.9), ("atc_fuzzy_c_y", 31.1), ("atc_fuzzy_c_z", 33.3),
    ("atc_fuzzy_c_roll", 0.1), ("atc_fuzzy_c_pitch", 0.1), ("atc_fuzzy_c_yaw", 0.1),
    ("atc_fuzzy_k_closest", 3.0), ("atc_fuzzy_l_max", 0.0), ("atc_fuzzy_max_rules", 400.0),
    ("atc_fuzzy_a_max_force", 10.0), ("atc_fuzzy_a_max_torque", 1.0),
    ("atc_fuzzy_out_max_force", 10.0), ("atc_fuzzy_out_max_torque", 1.0),
    # ACTNDO
    # obs_k 8.2 -> 4.0 and the arm window/floor 1.5/0.6 -> 2.0/1.0 (README section 4, flight atc_01):
    # the paper's values put a 3 Hz joint-1/yaw oscillation on the arm's ~36 ms torque path
    ("atc_obs_enable", True), ("atc_obs_k", 4.0), ("atc_obs_eta1", 5.0), ("atc_obs_eta2", 5.0),
    ("atc_obs_gamma1", 25.0), ("atc_obs_gamma2", 25.0), ("atc_obs_window1_s", 0.4),
    ("atc_obs_window2_s", 0.4), ("atc_obs_floor_s", 0.25), ("atc_obs_est_max", 3.0),
    # the arm
    # admittance H/D/K 1/4/9 -> 0.3333/1.34/3 (2026-10-25 tuning, README section 5): the paper's
    # admittance DYNAMICS kept (omega = sqrt(K/H) = 3 rad/s, zeta = D/(2 sqrt(KH)) = 0.67), one third of
    # its stiffness. At K 9 the push failed 7 of 8 attempts (box-lag guard) and the tuning flight tipped
    # the box past the 2 deg guard; K 3 completed 4 of 4; K 1.5 and K 1.0 aborted (the estimate the base
    # feed-forward carries collapses as the arm yields), D 2.0 (zeta 1) one completed flight, not adopted.
    ("atc_adm_h_j1", 0.3333), ("atc_adm_h_j2", 0.3333), ("atc_adm_h_j3", 0.3333), ("atc_adm_h_j4", 0.3333),
    ("atc_adm_d_j1", 1.34), ("atc_adm_d_j2", 1.34), ("atc_adm_d_j3", 1.34), ("atc_adm_d_j4", 1.34),
    ("atc_adm_k_j1", 3.0), ("atc_adm_k_j2", 3.0), ("atc_adm_k_j3", 3.0), ("atc_adm_k_j4", 3.0),
    ("atc_adm_e_max", 0.35),
    ("atc_arm_eta1_j1", 5.3), ("atc_arm_eta1_j2", 5.3), ("atc_arm_eta1_j3", 5.3), ("atc_arm_eta1_j4", 5.3),
    ("atc_arm_eta2_j1", 4.8), ("atc_arm_eta2_j2", 4.8), ("atc_arm_eta2_j3", 4.8), ("atc_arm_eta2_j4", 4.8),
    ("atc_arm_window_s", 2.0), ("atc_arm_floor_s", 1.0),
    ("atc_arm_q_min_deg_j1", -33.0), ("atc_arm_q_min_deg_j2", -18.0), ("atc_arm_q_min_deg_j3", 5.0),
    ("atc_arm_q_min_deg_j4", -118.0),
    ("atc_arm_q_max_deg_j1", 33.0), ("atc_arm_q_max_deg_j2", 43.0), ("atc_arm_q_max_deg_j3", 48.0),
    ("atc_arm_q_max_deg_j4", 118.0),
    ("atc_tau_max", 3.0),
    # the arm model's base CoM / armature: filled from the planner section (checked)
    ("atc_base_com_x", None), ("atc_base_com_y", None), ("atc_base_com_z", None),
    ("atc_armature_j1", None), ("atc_armature_j2", None), ("atc_armature_j3", None), ("atc_armature_j4", None),
    ("atc_controller_window_on_contact", False), ("atc_ie_lambda", 0.05),
    # addition (q), the bumpless contact transfer (README section 4, flights atc_02..06): at the
    # jaws' stall the base is a few mm off its reference and returning to it drags the clamped box
    ("atc_contact_reseat", True), ("atc_contact_reseat_blend_s", 1.0),
]

INTERFACE = [
    ("atc_arm_joint_topic", "fsc_open_manipulator/joint_states"),
    ("atc_arm_velocity_topic", "fsc_open_manipulator/external_torque_controller/velocity_observer"),
    ("atc_arm_velocity_timeout_s", 0.05),
    ("atc_arm_reference_topic", "fsc_open_manipulator/external_torque_controller/reference_joint_trajectory"),
    ("atc_arm_torque_topic", "fsc_open_manipulator/external_torque_controller/joint_torque_command"),
    ("atc_streamed_ref_topic", "fsc_autopilot_ros2/arbitrary_time_direct_actuation/reference"),
    ("atc_streamed_ref_timeout_s", 0.25),
    ("atc_arm_state_timeout_s", 0.5),
    ("atc_arm_reference_timeout_s", 0.2),
    ("atc_arm_hold_kp", 3.0), ("atc_arm_hold_kd", 0.25), ("atc_arm_hold_stream", True),
    ("atc_gate_pos_m", 0.15), ("atc_gate_vel_mps", 0.20), ("atc_gate_arm_rad", 0.05),
]

GUARD_KEYS = ("push_pull_mocap_guard_topic", "system_mocap_guard_topic")
GUARD_RE = re.compile(r"^\s+(" + "|".join(map(re.escape, GUARD_KEYS)) + r"):")

# THIS RIG'S OWN SAFETY GUARDS (2026-10-25, user request: tune the guards for this paper, the
# whole-body rig's untouched). Same rule as the whole-body guards' margin over that rig's own
# nominal readings (box lag 40 mm vs 15-22 mm peaks, force 5 N vs ~2.6 N filtered, ~1.8-2x), read
# off this rig's four completed tuning pushes at the tuned admittance (tools/guard_readings.py):
#   box lag: the soft base (~22 N/m position stiffness) carries the box's friction transitions, so
#     the box swings ahead of and behind its plan in a 3-4 s stick-slip cycle with the vehicle calm
#     and the box upright: peaks +25/-14, +59/-69, +63/-49, +42/-32 mm. 100 mm = 1.45x the worst;
#   force: the guard reads THIS node's estimate (atc_control_debug [97..99]), which under-reads the
#     true contact force along the push (damped J_m^-T in the arm's weakest direction + the ACTNDO's
#     attribution); its planner-filtered nominal peak is 1.10-1.46 N, so 3.5 N = 2.4x;
#   box tilt 2 deg, q3, vehicle tilt: the shared values (nominal box tilt <= 0.31 deg, q3 >= 25 deg).
# The jam tests with these values are in the README (section 5).
ATC_GUARDS = {"push_pull_guard_box_lag": "0.100", "push_pull_guard_force_n": "3.5"}
# A guard ADDED for this rig (the planner key exists since 2026-10-25, default 0 = off, absent from
# every whole-body yaml): push_pull_guard_box_cross, the box dragged sideways across the push. The
# base-relative arm admittance drags the clamped box with the soft base's sideways swings: nominal up
# to 35 mm in 21 recorded pushes, and twice (K 9 tuning flight t0, results attempt r3_a1) far enough
# that the box went over the table's side edge (its CoM after 75 mm) and fell off once released.
# 45 mm = 1.3x the worst nominal; replayed (tools/cross_guard_replay.py) it trips in both of those
# pushes with the box 8-14 mm inside the edge, which the base's swing can still carry past while the
# jaws open -- it reduces that risk, it does not remove it. Inserted after push_pull_guard_box_lag.
ATC_GUARDS_ADDED = {"push_pull_guard_box_cross": "0.045"}

HEADER = """# ============================================================================
# !! SIMULATION CONFIG -- GENERATED, DO NOT HAND-EDIT.                        !!
# ============================================================================
# AM-T650 ARBITRARY CONVERGENCE TIME CONTROL (Chen, Liu, Cai, Shang, Chang,
# Chao, Wang, Shen, "Arbitrary Convergence Time Control of Unmanned Aerial
# Manipulator for Active Interaction and Dynamic Tracking", IEEE Trans.
# Industrial Electronics 72(12), 2025, 13693-13702) -- the SIMULATION comparison
# rig of 2026-10-24 for the box-pushing task.
#   flight node: autopilot_arbitrary_time_direct_actuation_node
#     ACTNDO (eqs 6-11) + TSKFE (eqs 19-24, 29) + the quadrotor's and the
#     manipulator's arbitrary-convergence-time controllers (eqs 25-28, 34-38)
#   arm: the fsc_open_manipulator TORQUE-mode stack (the node streams the
#     joint torques), arm_planner following this node's mode topic
#     (torque_controller_isaac_aerial_atc.yaml)
#   reference: fsc_pegasus .../utils_controller/arbitrary_time_control.py
#
# GENERATED by fsc_PegasusSimulator
#   docs/docs_aerial_manipulator/archive/box_pushing_arbitrary_time_20261024/tools/make_arbitrary_time_yaml.py
# from {src}. Everything the rigs share is that file VERBATIM -- section 1 (the
# plant; checked key by key), the SAFETY loop + UDE, the guards, the allocator
# and the trajectory-planner sections. Only the DIRECT law (2.4 below) differs.
# To change anything shared, change the whole-body yaml and regenerate.
#
#   WB_SIM_PROFILE=box_pushing start_arbitrary_time_direct_actuation_t650_aerial_manipulator_stack.sh <cfg>
#   fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_arbitrary_time_box_pushing_sitl.sh <cfg>
# ============================================================================

"""


def fmt(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return repr(float(v))
    if isinstance(v, str):
        return f'"{v}"'
    return str(v)


def law_block(law):
    L = []
    a = L.append
    a("    # ── 2.4 THE DIRECT LAW: ARBITRARY CONVERGENCE TIME (Chen et al., TIE 2025) ──")
    a("    # Four modules (arbitrary_time_control.py is the reference; where the paper")
    a("    # leaves a choice its header lists what is done):")
    a("    #  C. quadrotor (eqs 25-28): per channel r1 = chi_d - chi, r2 = chi_d' - chi',")
    a("    #     delta = -eta1 phi(r1)/t_go, eps = r2 - delta, N = eta1 [phi'(r1) r2/t_go")
    a("    #     + phi(r1)/t_go^2] + eta2 phi(eps)/t_go, tau_u = C chi' + G + M(chi_d'' +")
    a("    #     r1 + N) - J_E tau_hat + D_hat; the position rows' force through the")
    a("    #     decoupling module (eq 3) gives the thrust and the roll/pitch references.")
    a("    #  THE ONE STRUCTURAL ADDITION: t_go = max(t_e - t, floor). The paper's gain")
    a("    #  is unbounded at t_e and ZERO after it (an undamped unit oscillator); after")
    a("    #  the floor the law is a constant-gain backstepping, small-signal")
    a("    #  r1'' + (a+b) r1' + (1+ab) r1 = 0 with a = eta1/floor, b = eta2/floor.")
    a("    #  eta per channel [x y z roll pitch yaw] (the paper: one pair for all six).")
    for k, v in law[:15]:
        a(f"    {k}: {fmt(v)}")
    a("    # the UAM's inertia about the body origin, FLU (the coordinated-admittance")
    a("    # rig's numbers: the arm at home)")
    for k, v in law[15:21]:
        a(f"    {k}: {fmt(v)}")
    a("    # safety shaping (not the paper's law): error saturations, the desired-force")
    a("    # tilt/floor clamps, per-axis torque clamps (yaw inside the allocator's budget)")
    for k, v in law[21:28]:
        a(f"    {k}: {fmt(v)}")
    a("    #  B. TSKFE (eqs 19-24, 29): x1 = max|chi_d - chi|, x2 = max|chi_d' - chi'|;")
    a("    #     the paper's 5 x 5 triangles; k-FRI (k = 3) spawns a rule for an input")
    a("    #     that fires none; omega_e' = c_e (M^-T eps)_e X phi^T. c: the paper's")
    a("    #     position values, the attitude ones scaled to this inertia.")
    for k, v in law[28:42]:
        a(f"    {k}: {fmt(v)}")
    a("    #  A. ACTNDO (eqs 6-11): stage 1 (generalized momentum, Gamma1, eta_o1) and")
    a("    #     stage 2 (tau_hat = rho + k H, Gamma2, eta_o2) on the arm's fixed-base")
    a("    #     dynamics; windows restart at DIRECT entry and every interaction edge")
    a("    #     (the planner's contact flag). The proof-consistent sign of w_o2.")
    for k, v in law[42:52]:
        a(f"    {k}: {fmt(v)}")
    a("    #  D. the arm (eqs 34-38): admittance H_d/D_d/K_d (the paper's experiment")
    a("    #     1/4/9) on tau_hat makes q_c; the backstepping law tracks it with the")
    a("    #     computed-torque model, minus tau_hat. q_c is kept in the joint box.")
    for k, v in law[52:84]:
        a(f"    {k}: {fmt(v)}")
    a("    # the arm model (= the planner's: base_com and the joint-diagonal armature")
    a("    # copied from section 4 by the generator), the windows, the J_m^-T damping")
    for k, v in law[84:]:
        a(f"    {k}: {fmt(v)}")
    a("    # ── 2.5 THE INTERFACE: the arm stack and the planner's stream ──────────")
    a("    # The ONE arm reference (arm_planner in SAFETY, the trajectory planner in")
    a("    # DIRECT); joint TORQUES out; the task as WholeBodyReference (MODEL frame),")
    a("    # converted to the base trajectory x_b = x_cd - R0 r_0c(q_d) and the joint")
    a("    # trajectory q_d. SAFETY streams the PD + model-gravity arm hold (the Isaac")
    a("    # arm stack has no gravity model). DIRECT entry is gated on a settled hover.")
    for k, v in INTERFACE:
        a(f"    {k}: {fmt(v)}")
    a("")
    return L


def build():
    text = open(os.path.join(CFG, SRC)).read()
    lines = text.split("\n")
    i_node = lines.index("/**/fsc_autopilot_ros2:")
    i_plan = next(i for i, l in enumerate(lines)
                  if l.startswith("# ===") and "4. WHOLE-BODY TRAJECTORY PLANNER" in "\n".join(lines[i:i + 3]))
    pre = lines[:i_node]
    body = lines[i_node:i_plan]
    tail = lines[i_plan:]
    # drop the guard keys (and the comment run directly above each)
    def drop_guard(ls):
        out = []
        for l in ls:
            if GUARD_RE.match(l):
                while out and out[-1].startswith("    #"):
                    out.pop()
                continue
            out.append(l)
        return out
    body = drop_guard(body)
    tail = drop_guard(tail)
    # this rig's own guard values (ATC_GUARDS), in the planner section only
    i_tp = tail.index("/**/whole_body_trajectory_planner:")
    for k, v in ATC_GUARDS.items():
        hits = [i for i in range(i_tp, len(tail)) if re.match(rf"^\s+{k}:", tail[i])]
        assert len(hits) == 1, f"{k}: {len(hits)} lines in the planner section"
        i = hits[0]
        old = re.match(rf"^(\s+{k}:)\s*([^#\n]*)", tail[i]).group(2).strip()
        tail[i] = re.sub(rf"^(\s+{k}:)\s*[^#\n]*", rf"\g<1> {v}", tail[i]).rstrip() + \
            f"   # ARBITRARY-TIME RIG (2026-10-25): {old} on the whole-body rig, see ATC_GUARDS"
    for k, v in ATC_GUARDS_ADDED.items():
        assert not any(re.match(rf"^\s+{k}:", l) for l in tail[i_tp:]), f"{k} already in the planner section"
        i = next(i for i in range(i_tp, len(tail)) if re.match(r"^\s+push_pull_guard_box_lag:", tail[i]))
        ind = re.match(r"^(\s+)", tail[i]).group(1)
        tail.insert(i + 1, f"{ind}{k}: {v}   # ARBITRARY-TIME RIG (2026-10-25): absent (off) on the whole-body rig, see ATC_GUARDS_ADDED")
    # the planner's base_com / armature -> the arm model
    psec = "\n".join(tail)
    bc = [float(x) for x in re.search(r"^\s+base_com:\s*\[([^\]]*)\]", psec, re.M).group(1).split(",")]
    arm = [float(x) for x in re.search(r"^\s+armature:\s*\[([^\]]*)\]", psec, re.M).group(1).split(",")]
    law = []
    for k, v in LAW:
        if k.startswith("atc_base_com_"):
            v = bc["xyz".index(k[-1])]
        elif k.startswith("atc_armature_j"):
            v = arm[int(k[-1]) - 1]
        law.append((k, v))
    i24 = next(i for i, l in enumerate(body) if l.startswith("    # ── 2.4 "))
    i28 = next(i for i, l in enumerate(body) if l.startswith("    # ── 2.8 "))
    body = body[:i24] + law_block(law) + body[i28:]
    out, pend = [], []
    for l in body:
        if l.strip().startswith("#") and l.startswith("    "):
            pend.append(l)
            continue
        if re.match(r"^\s+wb_(\w+):", l):
            pend = []
            continue
        out += pend
        pend = []
        out.append(l)
    out += pend
    out = [re.sub(r'vehicle_name: "[^"]*"', f'vehicle_name: "{VEHICLE}"', l) for l in out]
    out_text = HEADER.format(src=SRC) + "\n".join(pre[:0]) + "\n".join(out).rstrip("\n") + "\n\n" + "\n".join(tail)
    # ---- checks ---------------------------------------------------------------
    keys = dict(re.findall(r"^\s+(\w+):\s*([^#\n]*)", out_text, re.M))
    left = [k for k in keys if k.startswith("wb_")]
    assert not left, f"wb_ keys left: {left}"
    need = [k for k, _ in law] + [k for k, _ in INTERFACE] + ["vehicle_mass", "alloc_thrust_coeff", "ude_gain"]
    missing = [k for k in need if k not in keys]
    assert not missing, missing
    src_plant = dict(re.findall(r"^\s+(sim_\w+):\s*([^#\n]*)", text, re.M))
    dst_plant = {k: v for k, v in keys.items() if k.startswith("sim_")}
    assert src_plant == dst_plant, "plant (sim_) keys differ from the whole-body yaml"
    for sect in ("/**/whole_body_trajectory_planner:", "/**/whole_body_planner:"):
        s0 = text[text.index(sect):].split("\n/**/")[0]
        s1 = out_text[out_text.index(sect):].split("\n/**/")[0]
        if s0 != s1:
            k0 = dict(re.findall(r"^[ \t]+(\w+):[ \t]*([^#\n]*)", s0, re.M))
            k1 = dict(re.findall(r"^[ \t]+(\w+):[ \t]*([^#\n]*)", s1, re.M))
            diff = {k for k in set(k0) | set(k1) if k0.get(k, "").strip() != k1.get(k, "").strip()}
            allowed = set(GUARD_KEYS) | (set(ATC_GUARDS) | set(ATC_GUARDS_ADDED)
                                         if sect == "/**/whole_body_trajectory_planner:" else set())
            assert diff <= allowed, f"{sect} differs beyond the guard keys: {diff - allowed}"
    assert not any(GUARD_RE.match(l) for l in out_text.split("\n")), "a raw-mocap guard key survived"
    for k in ("alloc_thrust_coeff", "alloc_rotor0_km", "vehicle_mass", "posctl_k_vel_x", "ude_gain",
              "system_wd_max_drift_m", "system_wd_max_tilt_deg", "vehicle_thrust_scaling"):
        sv = re.search(rf"^\s+{k}:\s*([^#\n]*)", text, re.M).group(1).strip()
        assert keys[k].strip() == sv, k
    return out_text, len(dst_plant), bc, arm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="compare with the file on disk, write nothing")
    ap.add_argument("--out-dir", default=CFG)
    a = ap.parse_args()
    out_text, n_plant, bc, arm = build()
    dst = os.path.join(a.out_dir, DST)
    if a.check:
        cur = open(dst).read() if os.path.exists(dst) else ""
        print(f"{DST}: {'UP TO DATE' if cur == out_text else 'STALE'}")
        return int(cur != out_text)
    open(dst, "w").write(out_text)
    print(f"wrote {dst}: {n_plant} plant keys identical to {SRC}, planner sections identical (mocap-guard key dropped; this rig's guards {ATC_GUARDS} + {ATC_GUARDS_ADDED}), "
          f"{len(LAW)} law + {len(INTERFACE)} interface atc_ keys; arm model base_com {bc}, armature {arm}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
