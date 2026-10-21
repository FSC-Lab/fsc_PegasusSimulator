#!/usr/bin/env python3
"""make_coordinated_yaml.py -- generate the coordinated-admittance (Liu et al.,
CEP 2024) rig's SIMULATION yamls from the 4-D whole-body sim yamls of the same
profile (2026-10-10).

Everything the two rigs SHARE is copied verbatim -- the plant (section 1, the
sim_ keys the Pegasus launcher reads), the SAFETY loop + UDE, the guards, the
wrench allocator and the trajectory-planner sections -- so the only differences
between the rigs are the DIRECT law and the arm transport:

  * sections 2.4-2.7 (the whole-body law, observers, arm interface/gates) are
    REPLACED by the paper's UAV controller + estimator keys (cadm_*);
  * every other wb_ key (section 0's arm topics, section 3's hold stream) is
    dropped -- this rig's arm is the position-mode stack, driven by the
    coordinated planner node, and the flight node never touches it;
  * a `/**/coordinated_admittance_planner:` section (the paper's admittance
    filter, adaptive weights and kinematic MPC) is inserted before the
    trajectory-planner sections.

Checks: the sim_ plant keys and the planner sections are identical to the
source's, no wb_ key survives, every cadm_ key the node requires is present.

THE RAW-MOCAP GUARD's two keys (2026-10-21, GUARD_KEYS below) are DROPPED, each
with the comment run directly above it: they belong to the WHOLE-BODY box-pushing
stacks, which launch the estimator with mocap_guard:=true. This rig's stack does
not, and a planner given push_pull_mocap_guard_topic with no guard publishing
refuses every step ("no mocap-guard status").

    /usr/bin/python3 make_coordinated_yaml.py [--profile mirror|box_pushing] [--check]

(box_pushing was named push_pull until 2026-10-16; --profile push_pull still works.)
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
SRC = "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650{suffix}"
DST = "params_single_aerial_manipulator_coordinated_admittance_direct_actuation_t650{suffix}"
SUFFIX = {"mirror": "_sim.yaml", "box_pushing": "_sim_box_pushing.yaml",
          "valve_turning": "_sim_valve_turning.yaml"}            # 2026-10-17: the valve (turn form of the mode)
VEHICLE = {"mirror": "AM-T650-CADM", "box_pushing": "AM-T650-CADM-PUSH", "valve_turning": "AM-T650-CADM-VALVE"}
ALIAS = {"push_pull": "box_pushing"}   # the profile's name until 2026-10-16

# ---- the paper's UAV controller + estimator (the flight node) -----------------
# Position pair = the geometric rig's 2026-10-01 tune in ACCELERATION units
# (Kp/m, Kv/m with m = 3.746170): the paper gives no UAV gains. Attitude pair
# = that rig's Euler-equivalent PD (sim-validated against the MN4010 rotor-lag
# pole). Estimator K1/K2 = the paper's 20/5 (omega_n 10 rad/s, zeta 1).
LAW = [
    # cadm_19: at 2.5 / 2.5 (9 N/m of lateral stiffness against a 1.1 N friction box) the base's
    # +-30 mm lateral motion rotated the held box and the lateral loop let go 11 s into the push;
    # under the 5.0 / 1.6 attitude loop the bench keeps 4.0 / 3.5 at zeta 0.21-0.24 (kick 44 vs 60 mm)
    ("cadm_kpp_x", 4.0), ("cadm_kpp_y", 4.0), ("cadm_kpp_z", 8.0),
    ("cadm_kpd_x", 3.5), ("cadm_kpd_y", 3.5), ("cadm_kpd_z", 5.0),
    ("cadm_kpi_x", 1.0), ("cadm_kpi_y", 1.0), ("cadm_kpi_z", 3.0), ("cadm_int_max_m_s", 1.0),   # z 3.0: the mirror's thrust sag ramps the deficit, the claw sat 10-18 mm low at the fin (cadm_17..20)
    # cadm_16: a finger touching the fin (~1 N, 0.3 m below the CoM) tilted the nose toward the box
    # and the thrust drove the vehicle INTO it (0.3 N.m / 3.337 = 5 deg); 5.0 / 1.6 (bench-safe at
    # 40 ms delay, zeta kept) makes that 3.4 deg, and K_eta_i 2 unwinds it twice as fast.
    # yaw (cadm_18): holding the box 0.25 m ahead, a lateral base sway puts ~0.3 N.m of yaw through
    # the arm; at K 1.737 that is 9 deg, which swings the claw sideways and feeds back. 4.0 / 0.8
    # (wn 5.5 rad/s, zeta 0.55) inside the 0.4 N.m clamp, integral 2.
    ("cadm_keta_p_x", 5.0), ("cadm_keta_p_y", 5.0), ("cadm_keta_p_z", 4.0),
    ("cadm_keta_d_x", 1.6), ("cadm_keta_d_y", 1.6), ("cadm_keta_d_z", 0.8),
    # addition (j): attitude integral (0 = the paper). Measured without it:
    # 3 deg pitch / 4 deg yaw standing Euler error in every hover (the arm's
    # ~0.2 N.m moment + the plant's -0.1 N.m yaw bias on a pure PD).
    ("cadm_keta_i_x", 2.0), ("cadm_keta_i_y", 2.0), ("cadm_keta_i_z", 2.0), ("cadm_int_eta_max_rad_s", 0.5),
    # addition (n): the paper's own torque estimate (eq 25) cancelled in the attitude law (bench:
    # a sustained 0.3 N.m contact torque tilts 7.6 -> 4.1 deg, excursion 14 -> 9 mm)
    # OFF (cadm_15): at gain 1 the fed-back estimate (which carries the law's inertia mismatch
    # with the arm) formed a growing ~1 Hz attitude oscillation against the rotor-lag pole, flip
    # in 8 s. Kept as a key, 0 = the paper.
    ("cadm_tau_ff_gain", 0.0), ("cadm_tau_ff_lpf_s", 0.1),
    ("cadm_inertia_xx", 0.118), ("cadm_inertia_yy", 0.119), ("cadm_inertia_zz", 0.131),
    ("cadm_inertia_xy", 0.0089), ("cadm_inertia_xz", 0.0), ("cadm_inertia_yz", 0.0),
    ("cadm_max_pos_err_m", 1.0), ("cadm_max_vel_err_mps", 2.0), ("cadm_max_tilt_deg", 30.0),
    ("cadm_min_thrust_frac", 0.3), ("cadm_max_torque_xy", 8.0), ("cadm_max_torque_z", 0.55),   # under the allocator's 0.654 N.m free budget; the mirror delivers 0.70 of it
    ("cadm_est_enable", True), ("cadm_est_k1_force", 20.0), ("cadm_est_k2_force", 5.0),
    ("cadm_est_k1_torque", 20.0), ("cadm_est_k2_torque", 5.0), ("cadm_est_max_dt_s", 0.05),
    ("cadm_est_max_force_n", 20.0), ("cadm_est_max_torque_nm", 3.0),
]

# ---- the coordinated planner node (admittance + weights + kinematic MPC) ------
PLANNER = {
    "mirror": [
        ("rate_hz", 20.0),
        ("admittance_m", [0.5, 0.5, 0.5]),
        ("admittance_d", [17.3, 17.3, 24.5]),
        ("admittance_k", [150.0, 150.0, 300.0]),
        ("admittance_e_max", 0.15),
        # addition (p): the estimate is tared in free flight (frozen in the contact phase), and in
        # the contact phase the admittance is compliant across the push and vertically -- gains in
        # the CLAW frame [along, lateral, vertical]
        ("force_tare_enable", True), ("force_tare_tau_s", 2.0),
        ("admittance_k_contact", [150.0, 20.0, 20.0]),
        ("admittance_d_contact", [17.3, 20.0, 20.0]),
        ("weight_f_max", 1.5),      # on the TARED force: the 1.14 N slide force is "large" (w_uav 0.17)
        ("weight_alpha", 2.0),
        ("weight_force_xy_only", True),
        ("mpc_dt", 0.1), ("mpc_n_p", 10), ("mpc_n_c", 5),
        ("mpc_k_p", 4.0),
        ("mpc_w_e", [10.0, 10.0, 10.0]),
        ("mpc_w_c", [0.1, 0.1]),
        ("mpc_w_u_base", [0.05, 0.05, 0.05, 0.02, 0.02, 0.02, 0.2]),   # q4: see MpcConfig
        ("mpc_w_u_scale", 3.0),
        ("mpc_w_du", 0.0),
        ("mpc_w_psi", 5.0),
        ("mpc_w_q_post", [0.0, 1.0, 1.0, 1.0]),
        ("mpc_u_max", [0.6, 0.6, 0.6, 3.0, 3.0, 3.0, 3.0]),
        ("mpc_du_max", [0.3, 0.3, 0.3, 3.0, 3.0, 3.0, 3.0]),
        ("mpc_v_max", [0.4, 0.4, 0.4, 1.0, 1.0, 1.0, 1.0]),
        ("mpc_p_min", [-3.0, -3.0, 0.3]),
        ("mpc_p_max", [3.0, 3.0, 2.2]),
        ("mpc_q_min_deg", [-33.0, -18.0, 5.0, -118.0]),
        ("mpc_q_max_deg", [33.0, 43.0, 48.0, 118.0]),
        ("mpc_ee_body_min", [0.05, -0.25, -0.35]),
        ("mpc_ee_body_max", [0.35, 0.25, 0.10]),
        ("mpc_l_m", 0.06),
        # soft state/EE/CoM boxes, QUADRATIC penalty only (a linear term is a
        # kink OSQP crawls on: 2750 vs 50 iterations per hover tick)
        ("mpc_w_slack_lin", 0.0), ("mpc_w_slack_quad", 1.0e6),
        ("mpc_qp_eps_abs", 1.0e-6), ("mpc_qp_eps_rel", 1.0e-6), ("mpc_qp_max_iter", 4000), ("mpc_qp_polish", True),
        ("mpc_x0_from_command", "full"),
        ("mpc_resync_m", 0.15),
        ("mpc_resync_rad", 0.20),
        ("mpc_cmd_accel_ff", True),
        ("mpc_hold_on_unconverged", True),
        # addition (m): the arm absorbs the base's tracking error (the paper's measured-state
        # coordination, kept with the reference-state base command; cadm_11)
        # (cadm_13: at 0.1 s the arm chased the 0.33 Hz sway to its joint limits and its reaction
        # sustained it -- the term absorbs the STANDING offset, the sway must die on its own)
        # OFF (cadm_13/14): even at a 1 s low-pass (45 % gain, 90 deg lag at the 0.33 Hz sway) the
        # arm's reaction sustained a +-180 mm limit cycle; the claw rides the base on this rig
        ("mpc_arm_compensates_base_error", False), ("mpc_arm_compensation_lpf_s", 1.0),
        # addition (o): in CONTACT (w_uav below the threshold) the arm is held at its pose and the
        # heading term dropped -- with the claw clamped to the box a joint motion moves the BASE
        # (cadm_19/20: the heading term's q1 counter-rotation yawed the base -19 deg in 3 s)
        ("mpc_lock_arm_in_contact", True), ("mpc_lock_w_q", 10.0),
        ("attitude_source", "px4"),
    ],
}
PLANNER["box_pushing"] = PLANNER["mirror"]
PLANNER["valve_turning"] = PLANNER["mirror"]      # the same onboard planner tune; the task block is the whole-body valve yaml's

# Trajectory-planner keys this rig sets DIFFERENTLY from the whole-body yaml
# (applied to the copied planner section; the identity check allows exactly
# these). The descent trim is the decoupled rigs' sanctioned remedy for a
# standing claw offset (pick-and-place, 2026-10-03): the planner measures the
# claw error over a window at the hover above the handle and shifts the descent
# goal by it. cadm_05: the claw arrived 17 mm short / 15 mm high and the near
# jaw tip landed ON the fin.
# cadm_09 (2026-10-10): the 3 s trim window sampled the base's slow sway (+-34 mm std at the hover)
# and shifted the descent goal 27.6 mm BESIDE the fin, where the claw then sat to 2 mm -- the
# 10-09 hardware lesson again. With the attitude + position integrals the standing claw error at
# the hover is ~3 mm, so the trim is OFF (= the whole-body yaml's planner block, verbatim).
# cadm_10: the go-to-start at the whole-body bounds (0.30 m/s, 0.15 m/s^2) arrives with this rig's
# soft base loop 70/120 mm behind its command and swinging (the ~0.4 Hz sway, zeta ~0.3; 17 mm
# after a 6 s dwell), and the 3 s descent then ran the lagging claw onto the fin from the side.
# Gentler transitions for the decoupled PD base (its tracking lag is the paper's structure, not
# a tune); the push itself is unchanged (push_pull_push_time_s).
# The contact phase on this rig runs from the jaws' stall (the Push press) to the Exit press:
# the claw must be stiff and precise until the fin is gripped (contact_at_ready false), and the
# arm stays held while the jaws open (contact_off_at_push_end false).
PLANNER_TAIL_OVERRIDES = {"box_pushing": {"v_max": "0.20", "a_max": "0.08", "w_max": "0.20",
                                          "push_pull_contact_at_ready": "false",
                                          "push_pull_contact_off_at_push_end": "false"}}
# the valve (2026-10-17): the same gentler transitions and the same contact-phase span as the box
# (the arm held while the jaws open); the TURN itself (push_pull_turn_deg, the circle's 36 s) is
# the whole-body valve yaml's, verbatim
PLANNER_TAIL_OVERRIDES["valve_turning"] = dict(PLANNER_TAIL_OVERRIDES["box_pushing"])

# THE RAW-MOCAP GUARD (2026-10-21, fsc_PegasusSimulator archive/mocap_safety_guard_20261021):
# keys the whole-body box-pushing yaml carries (make_push_pull_yaml.py's planner block and
# ADDITIONS) that this rig must NOT inherit -- the planner key makes the planner refuse every
# step unless the estimator runs its guard (mocap_guard:=true), which only the whole-body
# box-pushing stacks launch. Dropped from the copied node and planner sections, each with
# the comment run directly above it; the planner identity check allows exactly these.
GUARD_KEYS = ("push_pull_mocap_guard_topic", "system_mocap_guard_topic")
GUARD_RE = re.compile(r"^\s+(" + "|".join(map(re.escape, GUARD_KEYS)) + r"):")


def drop_guard_keys(lines):
    """`lines` without the GUARD_KEYS lines and the comment run directly above each."""
    out = []
    for l in lines:
        if GUARD_RE.match(l):
            while out and out[-1].startswith("    #"):
                out.pop()
            continue
        out.append(l)
    return out

HEADER = """# ============================================================================
# !! SIMULATION CONFIG -- GENERATED, DO NOT HAND-EDIT.                        !!
# ============================================================================
# AM-T650 COORDINATED ADMITTANCE (Liu, Lyu, Guo, Wang, Yu, Guo, "A coordinated
# framework of aerial manipulator for safe and compliant physical interaction",
# Control Engineering Practice 146 (2024) 105898) -- the SIMULATION comparison
# rig of 2026-10-10 for the physical-interaction task of the {profile} profile
# (box_pushing: the box push; valve_turning: the valve's TURN, 2026-10-17).
#   flight node: autopilot_coordinated_admittance_direct_actuation_node
#                (eqs 11-14 PD/flatness UAV law + eq 25 momentum estimator)
#   planner node: scripts/tools/coordinated_admittance_planner.py
#                (eq 27 admittance + eqs 31-32 weights + eqs 28-39 kinematic MPC)
#   arm: the fsc_open_manipulator POSITION-mode stack (the paper's servo PD)
#
# GENERATED by fsc_PegasusSimulator
#   docs/docs_aerial_manipulator/archive/push_pull_coordinated_admittance_20261010/tools/make_coordinated_yaml.py
# from {src} ({profile} profile). Everything the two rigs share is that file
# VERBATIM -- section 1 (the plant; checked key by key), the SAFETY loop + UDE,
# the guards, the allocator and the trajectory-planner sections. Only the
# DIRECT law (2.4 below) and the planner-node section differ.
# To change anything shared, change the whole-body yaml and regenerate.
#
#   WB_SIM_PROFILE={profile} start_coordinated_admittance_direct_actuation_t650_aerial_manipulator_stack.sh <cfg>
#   fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_coordinated_admittance_direct_actuation_sitl.sh <cfg>
# ============================================================================

"""


def fmt(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, list):
        return "[" + ", ".join(fmt(x) for x in v) + "]"
    if isinstance(v, float):
        return repr(float(v))
    if isinstance(v, str):
        return f'"{v}"'
    return str(v)


def law_block():
    L = []
    a = L.append
    a("    # ── 2.4 THE DIRECT LAW: COORDINATED ADMITTANCE (Liu et al., CEP 2024) ──")
    a("    # The UAV side only (the arm is the position-mode stack, commanded by the")
    a("    # coordinated planner node; the flight node has NO arm model -- paper")
    a("    # Remark 3). Eq 11: nu = -K_Pp e_p - K_Pd e_v + a_d, f_t R e3 = m nu + m g e3;")
    a("    # eqs 12-13: flatness attitude from F_des and the yaw reference; eq 14:")
    a("    # tau_eta = -K_eta_p e_eta - K_eta_d eta_dot in Euler ZYX, body torque =")
    a("    # W^-T tau_eta. No integral, no adaptation (Section 3.2 passivity).")
    a("    # GAINS: the paper lists none for the UAV. Chosen on tools/sway_bench.py")
    a("    # (rigid body + rotor lag + the real law and planner, mocap noise, 1 N kick,")
    a("    # 0/24/40 ms feedback delay): the geometric rig's 20/11 N/m pair (5.37/2.95")
    a("    # per kg, cadm_05) has zeta ~0 at 0.5 Hz here -- the PD-attitude cascade's")
    a("    # lateral sway -- and 15/10 (cadm_07) zeta 0.2; stiffer attitude pairs")
    a("    # (keta_p 5-6.5) damp it but DIVERGE at 40 ms delay. 2.5 / 2.5 per kg with")
    a("    # keta_d 1.2 keeps zeta 0.45-0.55 and a 6-9 mm hold at every delay tested.")
    a("    # Vertical pair stiffer (no attitude lag on that axis): the 3 s descent")
    a("    # lagged 9 mm at 3.6 / 2.89 and the jaw tip caught the fin top (cadm_05).")
    for k, v in LAW[:6]:
        a(f"    {k}: {fmt(v)}")
    a("    # ADDITION (e), the one structural change: integral action on the UAV")
    a("    # position error (0 = the paper's PD). The mirror plant's ~1.4 N thrust-")
    a("    # model error parks the paper's PD ~8 cm low and the kinematic planner")
    a("    # cannot absorb it (the arm's upward reach at the push pose is ~3 cm).")
    a("    # Clamped at int_max [m.s] (1.0: K_Pi int_max = 1.5 m/s^2 = 5.6 N of")
    a("    # authority -- 0.3 saturated against the ~2 N vertical deficit, cadm_02),")
    a("    # reset at DIRECT entry, frozen while the tilt/thrust clamps act.")
    a("    # Routh: K_Pd K_Pp = 15.8 >> K_Pi.")
    for k, v in LAW[6:10]:
        a(f"    {k}: {fmt(v)}")
    for k, v in LAW[10:16]:
        a(f"    {k}: {fmt(v)}")
    a("    # inertia about the body origin, FLU: the estimator's M_eta = W' I W")
    for k, v in LAW[16:22]:
        a(f"    {k}: {fmt(v)}")
    a("    # safety shaping (not the paper's law): error saturations, the thrust")
    a("    # tilt/floor clamps, per-axis torque clamps (yaw sized to the allocator)")
    for k, v in LAW[22:28]:
        a(f"    {k}: {fmt(v)}")
    a("    # ── 2.5 THE ESTIMATOR: momentum-based external generalized force (eq 25)")
    a("    # r_hat = K1 K2 / (s^2 + K1 s + K1 K2) r per channel: the paper's 20 / 5")
    a("    # (omega_n 10 rad/s, zeta 1). Published on external_wrench_estimate for")
    a("    # the planner node (force WORLD frame, torque Euler-generalized).")
    for k, v in LAW[28:]:
        a(f"    {k}: {fmt(v)}")
    a("")
    return L


def planner_section(profile):
    L = []
    a = L.append
    a("# ============================================================================")
    a("# 2b. THE COORDINATED PLANNER NODE (coordinated_admittance_planner_node, C++/OSQP;")
    a("#     scripts/tools/coordinated_admittance_planner.py is the reference) -- the")
    a("#     paper's onboard planner: eq 27 admittance filter on the EE reference,")
    a("#     eqs 31-32 force-adaptive weights, eqs 28-39 kinematic MPC on h = [P_b; q].")
    a("#     Reads this node's section with --params-file; keys documented in")
    a("#     fsc_PegasusSimulator/.../utils_controller/coordinated_admittance.py.")
    a("#     Task-driven settings (stated in the README): admittance K_d 60 N/m (the")
    a("#     paper's push-pull 1.0 N/m would let a 1.2 N box reaction move the EE")
    a("#     reference 1.2 m -- no push), F_max 3 N / alpha 2 sized to the measured")
    a("#     0.7 N standing bias + 1.2 N slide force, the EE-heading and posture")
    a("#     additions (a)/(d), and the pre-processed joint box (Remark 2).")
    a("# ============================================================================")
    a("/**/coordinated_admittance_planner:")
    a("  ros__parameters:")
    for k, v in PLANNER[profile]:
        a(f"    {k}: {fmt(v)}")
    a("")
    return L


def build(profile):
    src_name = SRC.format(suffix=SUFFIX[profile])
    text = open(os.path.join(CFG, src_name)).read()
    lines = text.split("\n")
    i_node = lines.index("/**/fsc_autopilot_ros2:")
    i_plan = next(i for i, l in enumerate(lines) if l.startswith("# ===") and "4. WHOLE-BODY TRAJECTORY PLANNER" in "\n".join(lines[i:i + 3]))
    body = drop_guard_keys(lines[i_node:i_plan])
    tail = drop_guard_keys(lines[i_plan:])
    tail_over = PLANNER_TAIL_OVERRIDES.get(profile, {})
    for k, v in tail_over.items():
        hit = [i for i, l in enumerate(tail) if re.match(rf"^\s+{k}:", l)]
        if hit:
            tail[hit[0]] = re.sub(r"^(\s+" + k + r":\s*)[^#\n]*", rf"\g<1>{v}  ", tail[hit[0]]) + "# COORDINATED-ADMITTANCE OVERRIDE -- make_coordinated_yaml.py"
        else:
            j = next(i for i, l in enumerate(tail) if l.startswith("/**/whole_body_trajectory_planner:")) + 2
            tail.insert(j, f"    {k}: {v}  # COORDINATED-ADMITTANCE ADDITION -- make_coordinated_yaml.py")
    i24 = next(i for i, l in enumerate(body) if l.startswith("    # ── 2.4 "))
    i28 = next(i for i, l in enumerate(body) if l.startswith("    # ── 2.8 "))
    body = body[:i24] + law_block() + body[i28:]
    # drop the remaining wb_ keys with the comment run directly above each
    out, pend = [], []
    for l in body:
        m = re.match(r"^\s+wb_(\w+):", l)
        if l.strip().startswith("#") and l.startswith("    "):
            pend.append(l)
            continue
        if m:
            pend = []
            continue
        out += pend
        pend = []
        out.append(l)
    out += pend
    res = []
    for l in out:
        l = re.sub(r'vehicle_name: "[^"]*"', f'vehicle_name: "{VEHICLE[profile]}"', l)
        res.append(l)
    out_text = (HEADER.format(src=src_name, profile=profile) + "\n".join(res).rstrip("\n") + "\n\n"
                + "\n".join(planner_section(profile)) + "\n" + "\n".join(tail))
    # ---- checks -------------------------------------------------------------
    keys = dict(re.findall(r"^\s+(\w+):\s*([^#\n]*)", out_text, re.M))
    left = [k for k in keys if k.startswith("wb_")]
    assert not left, f"wb_ keys left: {left}"
    need = [k for k, _ in LAW] + ["vehicle_mass", "alloc_thrust_coeff", "posctl_k_vel_x", "ude_gain"]
    missing = [k for k in need if k not in keys]
    assert not missing, missing
    src_keys = dict(re.findall(r"^\s+(sim_\w+):\s*([^#\n]*)", text, re.M))
    dst_keys = {k: v for k, v in keys.items() if k.startswith("sim_")}
    assert src_keys == dst_keys, "plant (sim_) keys differ from the whole-body yaml"
    for sect in ("/**/whole_body_trajectory_planner:", "/**/whole_body_planner:"):
        s0 = text[text.index(sect):].split("\n/**/")[0]
        s1 = out_text[out_text.index(sect):].split("\n/**/")[0]
        if s0 != s1:
            k0 = dict(re.findall(r"^[ \t]+(\w+):[ \t]*([^#\n]*)", s0, re.M))
            k1 = dict(re.findall(r"^[ \t]+(\w+):[ \t]*([^#\n]*)", s1, re.M))
            diff = {k for k in set(k0) | set(k1) if k0.get(k, "").strip() != k1.get(k, "").strip()}
            allowed = set(tail_over) | set(GUARD_KEYS)
            assert diff <= allowed, f"{sect} section differs beyond the declared overrides: {diff - allowed}"
    assert not any(GUARD_RE.match(l) for l in out_text.split("\n")), f"a raw-mocap guard key survived: {GUARD_KEYS}"
    for k in ("alloc_thrust_coeff", "alloc_rotor0_km", "vehicle_mass", "posctl_k_vel_x", "ude_gain",
              "system_wd_max_drift_m", "system_wd_max_tilt_deg", "vehicle_thrust_scaling"):
        sv = re.search(rf"^\s+{k}:\s*([^#\n]*)", text, re.M).group(1).strip()
        assert keys[k].strip() == sv, k
    return out_text, src_name, len(dst_keys)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="all", choices=["all"] + sorted(SUFFIX) + sorted(ALIAS))
    ap.add_argument("--check", action="store_true", help="compare with the file on disk, write nothing")
    ap.add_argument("--out-dir", default=CFG)
    a = ap.parse_args()
    rc = 0
    for profile in (sorted(SUFFIX) if a.profile == "all" else [ALIAS.get(a.profile, a.profile)]):
        out_text, src_name, n_plant = build(profile)
        dst = os.path.join(a.out_dir, DST.format(suffix=SUFFIX[profile]))
        if a.check:
            cur = open(dst).read() if os.path.exists(dst) else ""
            state = "UP TO DATE" if cur == out_text else "STALE"
            rc |= cur != out_text
            print(f"{os.path.basename(dst)}: {state}")
            continue
        open(dst, "w").write(out_text)
        print(f"wrote {dst}: {n_plant} plant keys identical to {src_name}, planner sections identical "
              f"except {sorted(PLANNER_TAIL_OVERRIDES.get(profile, {}))}, {sum(1 for k, _ in LAW)} cadm_ keys")
    return rc


if __name__ == "__main__":
    sys.exit(main())
