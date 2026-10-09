#!/usr/bin/env python3
"""make_modular_yaml.py -- generate the modular adaptive node's SIMULATION yaml
from the 4-D whole-body sim yaml of the same profile (2026-09-30).

Everything the two rigs SHARE is copied verbatim -- the plant (section 1, the
sim_ keys the Pegasus launcher reads), the SAFETY loop + UDE, the guards, the
wrench allocator, the arm interface/gates, and the trajectory-planner sections
-- so the ONLY difference between the rigs is the DIRECT law:

  * every wb_ law / L1-observer key is removed;
  * the arm-interface keys keep their values under the mod_ prefix
    (wb_arm_joint_topic -> mod_arm_joint_topic, ...);
  * the paper's law keys (mod_pos_* / mod_att_* / mod_arm_* ...) are inserted
    from a gains json (the CMA result: tune_modular.py's "best" dict).

The script CHECKS what it promises: section 1's text is byte-identical to the
source's, no wb_ key survives, and every mod_ key the node requires is present.

    /usr/bin/python3 make_modular_yaml.py --gains ../analysis/modular_final_gains.json [--profile mirror|robustness]
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
SRC = "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650{suffix}"
DST = "params_single_aerial_manipulator_modular_adaptive_direct_actuation_t650{suffix}"
SUFFIX = {"mirror": "_sim.yaml", "robustness": "_sim_robustness.yaml",
          "pick_and_place": "_sim_pick_and_place.yaml"}   # 2026-10-08: the mirror + the pick-and-place task block
IFACE = ["tau_max", "arm_state_timeout_s", "arm_reference_timeout_s", "arm_hold_kp", "arm_hold_kd",
         "arm_hold_stream", "gate_pos_m", "gate_vel_mps", "gate_arm_rad", "arm_joint_topic",
         "arm_reference_topic", "arm_torque_topic", "arm_sign_j1", "arm_sign_j2", "arm_sign_j3",
         "arm_sign_j4", "arm_velocity_topic", "arm_velocity_timeout_s", "streamed_ref_timeout_s",
         "streamed_ref_topic", "base_com_x", "base_com_y", "base_com_z", "armature_joint_diag",
         "armature_j1", "armature_j2", "armature_j3", "armature_j4"]
M_NOM = 3.746170
MBAR_ATT = 0.095
MBAR_ARM = [0.022, 0.033, 0.016, 0.010]


def law_block(p, src_note):
    nu = lambda m, i: p.get(f"{m}_nu0", p.get(f"{m}_nu")) if i == 0 else p.get(f"{m}_nu123", p.get(f"{m}_nu"))  # noqa: E731
    L = []
    a = L.append
    a("    # ── 2.4 THE DIRECT LAW: MODULAR ADAPTIVE (Yadav et al., TMECH 2025) ──────")
    a("    # tau_j = Mbar_j(-Lambda_j(lambda1 e + lambda2 edot) - dtau_j + ff_j),")
    a("    # dtau_j = rho_j r_j / max(|r_j|, varpi_j), r_j = B^T P_j xi_j with")
    a("    # A^T P + P A = -diag(q_e, q_v) per axis, rho_j = K0 + K1|xi| + K2|xi|^2")
    a("    # + K3|chi_ddot| + zeta_j, K_i' = |r||xi|^i - nu_i K_i. Symbols = Table II.")
    a("    # Mbar = the nominal inertias (Remark 3): vehicle mass, 0.095 kg m^2 body,")
    a("    # the arm's joint-space diagonal at home [0.022 0.033 0.016 0.010].")
    a("    # Lambda (capital) = 1: Table II's Lambda*lambda products are the tuned")
    a("    # lambda1/lambda2 below (choice 1 in modular_adaptive.py).")
    a(f"    # GAINS: {src_note}")
    a("    # Table II itself is NOT flyable on this plant (bench: abort at 1.7 s --")
    a("    # Mbar_alpha K1 = 0.3 N.m/rad lets the 0.7 N.m arm load sag 26 deg, and")
    a("    # Mbar_qq = 0.015 is ~1/6 of this vehicle's inertia).")
    a(f"    mod_pos_mbar_xy: {M_NOM}")
    a(f"    mod_pos_mbar_z: {M_NOM}")
    a(f"    mod_pos_lambda1_xy: {p['p_kp_xy']:.6g}")
    a(f"    mod_pos_lambda1_z: {p['p_kp_z']:.6g}")
    a(f"    mod_pos_lambda2_xy: {p['p_kd_xy']:.6g}")
    a(f"    mod_pos_lambda2_z: {p['p_kd_z']:.6g}")
    a("    mod_pos_cap_lambda_xy: 1.0")
    a("    mod_pos_cap_lambda_z: 1.0")
    a(f"    mod_pos_q_e: {p['p_q']:.6g}")
    a(f"    mod_pos_q_v: {p['p_q'] * p.get('p_qv', 1.0):.6g}")
    for i in range(4):
        a(f"    mod_pos_nu{i}: {float(nu('p', i)):.6g}")
    a("    mod_pos_eps: 0.0001")
    a(f"    mod_pos_varpi: {p['p_varpi']:.6g}")
    a(f"    mod_pos_k_init: {p.get('p_k0', 0.01):.6g}")
    a("    mod_pos_zeta_init: 0.1")
    a("    # The ONE structural addition (choice 3): the nominal weight, so the")
    a("    # adaptive term does not have to carry 36.7 N through rho r / varpi.")
    a(f"    mod_pos_gravity_mass: {M_NOM}")
    a(f"    mod_att_mbar_rp: {MBAR_ATT * p.get('q_mbar_s', 1.0):.6g}")
    a(f"    mod_att_mbar_yaw: {MBAR_ATT * p.get('q_mbar_s', 1.0):.6g}")
    a(f"    mod_att_lambda1_rp: {p['q_kp_rp']:.6g}")
    a(f"    mod_att_lambda1_yaw: {p['q_kp_y']:.6g}")
    a(f"    mod_att_lambda2_rp: {p['q_kd_rp']:.6g}")
    a(f"    mod_att_lambda2_yaw: {p['q_kd_y']:.6g}")
    a("    mod_att_cap_lambda_rp: 1.0")
    a("    mod_att_cap_lambda_yaw: 1.0")
    a(f"    mod_att_q_e: {p['q_q']:.6g}")
    a(f"    mod_att_q_v: {p['q_q'] * p.get('q_qv', 1.0):.6g}")
    for i in range(4):
        a(f"    mod_att_nu{i}: {float(nu('q', i)):.6g}")
    a("    mod_att_eps: 0.0001")
    a(f"    mod_att_varpi: {p['q_varpi']:.6g}")
    a(f"    mod_att_k_init: {p.get('q_k0', 0.001):.6g}")
    a("    mod_att_zeta_init: 0.01")
    for j in range(4):
        a(f"    mod_arm_mbar_j{j + 1}: {MBAR_ARM[j] * p.get('a_mbar_s', 1.0):.6g}")
    a(f"    mod_arm_lambda1: {p['a_kp']:.6g}")
    a(f"    mod_arm_lambda2: {p['a_kd']:.6g}")
    a("    mod_arm_cap_lambda: 1.0")
    a(f"    mod_arm_q_e: {p['a_q']:.6g}")
    a(f"    mod_arm_q_v: {p['a_q'] * p.get('a_qv', 1.0):.6g}")
    for i in range(4):
        a(f"    mod_arm_nu{i}: {float(nu('a', i)):.6g}")
    a("    mod_arm_eps: 0.0001")
    a(f"    mod_arm_varpi: {p['a_varpi']:.6g}")
    a(f"    mod_arm_k_init: {p.get('a_k0', 1e-4):.6g}")
    a("    mod_arm_zeta_init: 0.01")
    a(f"    mod_acc_filter_hz: {p.get('acc_hz', 10.0):.6g}")
    a("    mod_qdd_filter_hz: 10.0")
    a("    mod_min_thrust_frac: 0.2")
    a("")
    return L


HEADER = """# ============================================================================
# !! SIMULATION CONFIG -- GENERATED, DO NOT HAND-EDIT.                        !!
# ============================================================================
# AM-T650 MODULAR ADAPTIVE (Yadav, Dantu, Pan, Sun, Roy, Baldi, "Modular
# Adaptive Aerial Manipulation Under Unknown Dynamic Coupling Forces",
# IEEE/ASME Trans. Mechatronics 30(4), 2025) -- the SIMULATION comparison rig
# of 2026-09-30.   node: autopilot_modular_adaptive_direct_actuation_node
#
# GENERATED by fsc_PegasusSimulator
#   docs/docs_aerial_manipulator/modular_adaptive_20260930/tools/make_modular_yaml.py
# from {src}
# ({profile} profile). Everything the two rigs share is that file VERBATIM --
# section 1 (the plant; the generator checks it byte for byte), the SAFETY
# loop + UDE, the guards, the allocator, the arm interface (renamed wb_ -> mod_)
# and the trajectory-planner sections. Only the DIRECT law differs (2.4 below).
# To change anything shared, change the whole-body yaml and regenerate.
#
#   start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh <cfg>
#   fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_modular_adaptive_direct_actuation_sitl.sh <cfg>
# ============================================================================

"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gains", required=True, help="json with a 'best' (or flat) bench param dict")
    ap.add_argument("--profile", default="mirror", choices=sorted(SUFFIX))
    ap.add_argument("--note", default="", help="provenance line for the gains")
    ap.add_argument("--out-dir", default=CFG)
    a = ap.parse_args()
    g = json.load(open(a.gains))
    p = g.get("best", g)
    note = a.note or f"tuned on the circle bench, {os.path.basename(a.gains)}"
    src_name = SRC.format(suffix=SUFFIX[a.profile])
    text = open(os.path.join(CFG, src_name)).read()
    lines = text.split("\n")
    i_node = lines.index("/**/fsc_autopilot_ros2:")
    body = lines[i_node:]

    if a.profile in ("mirror", "pick_and_place"):
        i24 = next(i for i, l in enumerate(body) if l.startswith("    # ── 2.4 "))
        i27 = next(i for i, l in enumerate(body) if l.startswith("    # ── 2.7 "))
        keep = [l for l in body[i24:i27] if re.match(r"^\s+wb_(\w+):", l)
                and re.match(r"^\s+wb_(\w+):", l).group(1) in IFACE]
        shared = (["    # shared with the whole-body rig (moved out of its law section):"] + keep + [""]) if keep else []
        body = body[:i24] + law_block(p, note) + shared + body[i27:]
    else:
        # keyed transform: drop wb_ law keys (and the comment block directly above
        # each dropped run), insert the law block before the allocator section.
        out, pend = [], []
        for l in body:
            m = re.match(r"^\s+wb_(\w+):", l)
            if l.strip().startswith("#") and l.startswith("    "):
                pend.append(l); continue
            if m and m.group(1) not in IFACE:
                pend = []; continue
            out += pend; pend = []; out.append(l)
        out += pend
        i_alloc = next(i for i, l in enumerate(out) if re.match(r"^\s+alloc_rotor0_px:", l))
        while i_alloc > 0 and out[i_alloc - 1].strip().startswith("#"):
            i_alloc -= 1
        body = out[:i_alloc] + law_block(p, note) + out[i_alloc:]

    res = []
    for l in body:
        m = re.match(r"^(\s+)wb_(\w+):(.*)$", l)
        if m:
            assert m.group(2) in IFACE, f"unexpected wb_ key survived: {l}"
            l = f"{m.group(1)}mod_{m.group(2)}:{m.group(3)}"
        l = re.sub(r'vehicle_name: "[^"]*"', 'vehicle_name: "AM-T650-MODULAR"', l)
        l = l.replace("MUST equal wb_tau_max above", "MUST equal mod_tau_max above")
        res.append(l)
    out_text = HEADER.format(src=src_name, profile=a.profile) + "\n".join(res)

    # ---- checks -------------------------------------------------------------
    keys = dict(re.findall(r"^\s+(\w+):\s*([^#\n]*)", out_text, re.M))
    assert not [k for k in keys if k.startswith("wb_")], "wb_ keys left"
    need = [f"mod_{k}" for k in ("pos_lambda1_xy", "att_lambda1_rp", "arm_lambda1", "pos_gravity_mass",
                                 "arm_joint_topic", "tau_max", "base_com_y")]
    missing = [k for k in need if k not in keys]
    assert not missing, missing
    src_keys = dict(re.findall(r"^\s+(sim_\w+):\s*([^#\n]*)", text, re.M))
    dst_keys = {k: v for k, v in keys.items() if k.startswith("sim_")}
    assert src_keys == dst_keys, "plant (sim_) keys differ from the whole-body yaml"
    for sect in ("/**/whole_body_trajectory_planner:", "/**/whole_body_planner:"):
        s0 = text[text.index(sect):]; s1 = out_text[out_text.index(sect):]
        s0 = s0.split("\n/**/")[0]; s1 = s1.split("\n/**/")[0]
        assert s0.replace("wb_tau_max", "mod_tau_max") == s1, f"{sect} section differs"
    for k in ("alloc_thrust_coeff", "vehicle_mass", "posctl_k_vel_x", "ude_gain", "system_wd_max_drift_m"):
        sv = re.search(rf"^\s+{k}:\s*([^#\n]*)", text, re.M).group(1).strip()
        assert keys[k].strip() == sv, k
    dst = os.path.join(a.out_dir, DST.format(suffix=SUFFIX[a.profile]))
    open(dst, "w").write(out_text)
    print(f"wrote {dst}: {sum(k.startswith('mod_') for k in keys)} mod_ keys, {len(dst_keys)} plant keys "
          f"(identical to {src_name}), planner sections identical")


if __name__ == "__main__":
    sys.exit(main())
