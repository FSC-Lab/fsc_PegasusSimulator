#!/usr/bin/env python3
"""Build the pick-and-place comparison report page (artifact "Pick-and-Place Controller
Comparison", 2026-10-04) from ../runs/report_data.json (report_data.py). Every number in
section 1 is computed here from that file; section 2 reads the two controller yamls and
the planner block they share.

    PYTHONNOUSERSITE=1 /usr/bin/python3 report_data.py
    PYTHONNOUSERSITE=1 /usr/bin/python3 build_report.py [out.html]

Writes ../report.html by default.
"""
import html
import json
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
WB_YAML = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_place.yaml")
GEO_YAML = os.path.join(CFG, "params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim_pick_place.yaml")
ARM = os.path.expanduser("~/ros2_ws/src/fsc_open_manipulator/open_manipulator_x_isaac_bridge/config")
ARM_T = os.path.join(ARM, "torque_controller_isaac_aerial.yaml")
PHASES = ["approach", "grab", "carry", "place", "return"]
RIGS = ("wb", "geo")


def node(path, suffix):
    for k, v in yaml.safe_load(open(path)).items():
        if str(k).endswith(suffix):
            return v["ros__parameters"]
    raise KeyError(f"{suffix} not in {path}")


def f(x, n=1):
    return f"{x:.{n}f}"


def vec(v, n=3):
    return "[" + ", ".join(f"{float(x):g}" if n is None else f"{float(x):.{n}f}".rstrip("0").rstrip(".")
                           for x in v) + "]"


def esc(s):
    return html.escape(str(s), quote=False)


def table(head, rows, cls=""):
    th = "".join(f"<th>{h}</th>" for h in head)
    body = []
    for r in rows:
        if isinstance(r, str):                      # a group row
            body.append(f'<tr class="grp"><th colspan="{len(head)}">{r}</th></tr>')
            continue
        body.append("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>")
    return f'<div class="tscroll"><table class="{cls}"><thead><tr>{th}</tr></thead><tbody>{"".join(body)}</tbody></table></div>'


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "report.html")
    D = json.load(open(os.path.join(HERE, "..", "runs", "report_data.json")))
    L = D["L"]
    R = {rig: [r for r in D["runs"] if r["rig"] == rig] for rig in RIGS}

    def arr(rig, fn):
        return np.array([fn(r) for r in R[rig]], float)

    # ---------- section 1 numbers ----------
    S = {}
    for rig in RIGS:
        s = S[rig] = {}
        s["n"] = len(R[rig]); s["ok"] = int(sum(r["success"] for r in R[rig]))
        for ph in PHASES:
            mx = arr(rig, lambda r: r["phase_stats"][ph]["max_mm"])
            rm = arr(rig, lambda r: r["phase_stats"][ph]["rms_mm"])
            T = arr(rig, lambda r: r["phase_stats"][ph]["T"])
            s[ph] = dict(mx=mx, rm=rm, T=T)
        for k in ("lift", "release"):
            s["t10_" + k] = arr(rig, lambda r: np.nan if r["t10"][k] is None else r["t10"][k])
        wp = np.array([v for r in R[rig] for v in r["wp_mm"].values()])
        s["wp_n"], s["wp_bad"], s["wp_max"] = len(wp), int(np.sum(wp >= 0.25 * L * 1e3)), float(wp.max())
        s["wp_bad_names"] = sorted({k for r in R[rig] for k, v in r["wp_mm"].items() if v >= 0.25 * L * 1e3})
        for k in ("place_off_mm", "tilt_max", "ee_close_mm", "ee_open_mm", "hover_pick_mm", "hover_place_mm",
                  "slip_mm", "mission_s"):
            s[k] = arr(rig, lambda r: r["score"][k])
        for k in ("roll", "pitch", "ee_rms"):
            s["rip_" + k] = arr(rig, lambda r: r["ripple"][k])
        s["lift_pk"] = arr(rig, lambda r: max(r["lift"]["rho"])) * L * 1e3
        s["rel_pk"] = arr(rig, lambda r: max(r["release"]["rho"])) * L * 1e3
    W, G = S["wb"], S["geo"]

    def mxcell(s, ph):
        m = s[ph]["mx"]
        return (f'<span class="n">{f(m.mean())} ± {f(m.std())}</span> <span class="rho">ρ {m.mean() / L / 1e3:.3f}</span>')

    def rmcell(s, ph):
        m = s[ph]["rm"]
        return f'<span class="n">{f(m.mean())}</span> <span class="rho">ρ {m.mean() / L / 1e3:.3f}</span>'

    def rng(v, n=1, unit=""):
        return f'{f(np.nanmin(v), n)}–{f(np.nanmax(v), n)}{unit}'

    def mean_rng(v, n=1):
        return f'<span class="n">{f(np.nanmean(v), n)}</span> <span class="dim">({f(np.nanmin(v), n)}–{f(np.nanmax(v), n)})</span>'

    def t10cell(v):
        ok = np.isfinite(v)
        if not ok.any():
            return "never"
        txt = f'<span class="n">{f(np.nanmean(v))} s</span> <span class="dim">({int(ok.sum())}/{len(v)} runs'
        return txt + (")" if ok.all() else f"; {int((~ok).sum())} never inside 0.1 L)") + "</span>"

    rows1 = []
    rows1.append(["Success (payload set upright on the place cap)",
                  f'<span class="n">{W["ok"]} / {W["n"]}</span>', f'<span class="n">{G["ok"]} / {G["n"]}</span>', ""])
    rows1.append("Max ε<sub>UAV</sub> per phase, mm (mean ± sd over the runs) and ρ<sub>UAV</sub>")
    for ph in PHASES:
        lab = "<b>grab</b> (the paper's ε<sub>UAV</sub><sup>grab</sup>)" if ph == "grab" else ph
        ratio = G[ph]["mx"].mean() / W[ph]["mx"].mean()
        rows1.append([lab, mxcell(W, ph), mxcell(G, ph), f'<span class="n">{ratio:.1f}×</span>'])
    rows1.append("RMS ε<sub>UAV</sub> per phase, mm")
    for ph in PHASES:
        ratio = G[ph]["rm"].mean() / W[ph]["rm"].mean()
        rows1.append([ph, rmcell(W, ph), rmcell(G, ph), f'<span class="n">{ratio:.1f}×</span>'])
    rows1.append("Recovery and way-points")
    rows1.append(["t<sub>10%</sub> after the lift-off", t10cell(W["t10_lift"]), t10cell(G["t10_lift"]), ""])
    rows1.append(["t<sub>10%</sub> after the release", t10cell(W["t10_release"]), t10cell(G["t10_release"]), ""])
    rows1.append(["Way-points with ε ≥ 0.25 L (93 mm)",
                  f'<span class="n">{W["wp_bad"]} / {W["wp_n"]}</span> <span class="dim">worst {f(W["wp_max"], 0)} mm, ρ {W["wp_max"] / L / 1e3:.2f}</span>',
                  f'<span class="n">{G["wp_bad"]} / {G["wp_n"]}</span> <span class="dim">all at Exit To Pick; worst {f(G["wp_max"], 0)} mm, ρ {G["wp_max"] / L / 1e3:.2f}</span>', ""])
    rows1.append("Phase times, s (mean)")
    rows1.append(["grab / place",
                  f'<span class="n">{f(W["grab"]["T"].mean())} / {f(W["place"]["T"].mean())}</span>',
                  f'<span class="n">{f(G["grab"]["T"].mean())} / {f(G["place"]["T"].mean())}</span>', ""])
    rows1.append(["whole mission in DIRECT", mean_rng(W["mission_s"]), mean_rng(G["mission_s"]), ""])
    t1 = table(["Metric", '<span class="key wb">Whole-body 4-D L1</span>', '<span class="key geo">Decoupled geometric + L1</span>',
                "Decoupled ÷ whole-body"], rows1, "cmp")

    rows2 = [
        "At the gripper (the claw's FK from the measured state against the planner's target)",
        ["Claw from target when the jaws close", mean_rng(W["ee_close_mm"]), mean_rng(G["ee_close_mm"])],
        ["Claw from target when the jaws open", mean_rng(W["ee_open_mm"]), mean_rng(G["ee_open_mm"])],
        ["Claw hover offset above the pick target", mean_rng(W["hover_pick_mm"]), mean_rng(G["hover_pick_mm"])],
        ["Claw hover offset above the place target", mean_rng(W["hover_place_mm"]), mean_rng(G["hover_place_mm"])],
        ["Claw error RMS, whole mission", mean_rng(W["rip_ee_rms"]), mean_rng(G["rip_ee_rms"])],
        "Payload",
        ["Set down, off the place pillar's axis", mean_rng(W["place_off_mm"]), mean_rng(G["place_off_mm"])],
        ["Slip in the jaws while carried", mean_rng(W["slip_mm"]), mean_rng(G["slip_mm"])],
        "Airframe attitude (whole mission in DIRECT)",
        ["Peak tilt, deg", mean_rng(W["tilt_max"]), mean_rng(G["tilt_max"])],
        ["Roll ripple RMS (detrended over 2 s), deg", mean_rng(W["rip_roll"], 2), mean_rng(G["rip_roll"], 2)],
        ["Pitch ripple RMS (detrended over 2 s), deg", mean_rng(W["rip_pitch"], 2), mean_rng(G["rip_pitch"], 2)],
    ]
    t2 = table(["Metric (mm unless stated), mean (range over 5 runs)", '<span class="key wb">Whole-body 4-D L1</span>',
                '<span class="key geo">Decoupled geometric + L1</span>'], rows2, "cmp")

    kpis = [
        ("Missions completed", f'{W["ok"]} / {W["n"]}', f'{G["ok"]} / {G["n"]}', ""),
        ("Max ρ<sub>UAV</sub> during the grab", f'{W["grab"]["mx"].mean() / L / 1e3:.3f}',
         f'{G["grab"]["mx"].mean() / L / 1e3:.3f}', f'{f(W["grab"]["mx"].mean(), 0)} vs {f(G["grab"]["mx"].mean(), 0)} mm'),
        ("t<sub>10%</sub> after the lift-off", f'{f(np.nanmean(W["t10_lift"]))} s', f'{f(np.nanmean(G["t10_lift"]))} s', "back inside 0.1 L"),
        ("Way-points above 0.25 L", f'{W["wp_bad"]} / {W["wp_n"]}', f'{G["wp_bad"]} / {G["wp_n"]}', "the paper's way-point condition"),
    ]
    kpi_html = "".join(
        f'<div class="kpi"><div class="kl">{a}</div><div class="kv"><span class="wbv">{b}</span>'
        f'<span class="sep">vs</span><span class="geov">{c}</span></div><div class="kn">{d}</div></div>'
        for a, b, c, d in kpis)

    # ---------- section 2 parameters ----------
    wp = node(WB_YAML, "fsc_autopilot_ros2")
    gp = node(GEO_YAML, "fsc_autopilot_ros2")
    pl = node(WB_YAML, "whole_body_trajectory_planner")
    at = node(ARM_T, "external_torque_controller")

    def k(key):
        return f'<code>{esc(key)}</code>'

    def ppchip(prev):
        return f' <span class="chip" title="pick-and-place value; the free-flight tune is {esc(prev)}">pick-and-place · free flight {esc(prev)}</span>'

    for key in [k_ for k_ in wp if k_.startswith(("sim_", "alloc_", "vehicle_", "posctl_", "ude_", "system_wd", "system_fb"))
                if k_ in gp and k_ != "vehicle_name"]:
        assert wp[key] == gp[key], f"{key} differs between the two yamls: {wp[key]} vs {gp[key]}"

    shared = [
        "Simulation",
        ["Simulator", "Isaac Sim, headless, real-time pacer; RTF 0.995–1.000 in every 10 s window of every run", "machine conf <code>SIM_RTF=1.0</code>, <code>SIM_REALTIME=1</code>"],
        ["Physics step", "4 ms (250 Hz); law and arm controllers at 250 Hz", ""],
        ["State feedback", f'raw motion capture at {f(wp["sim_feedback_mocap_rate_hz"], 0)} Hz, position noise {wp["sim_feedback_pos_noise_m"] * 1e3:g} mm, velocity noise {wp["sim_feedback_vel_noise_mps"] * 1e2:g} cm/s', k("sim_feedback_*")],
        "Scene (07 pick-and-place)",
        ["Payload", "200 g: 110 × 110 × 65 mm box + 20 × 60 × 200 mm handle plate; not in either controller's model", "<code>PEGASUS_PNP_PAYLOAD_MASS</code>"],
        ["Pillars", "1.0 m to the cap top; pick at (1, 1) m, place at (−1, −1) m; 160 × 10 mm caps", "<code>PEGASUS_PNP_CAP_DIAMETER</code>"],
        ["Friction", "jaw pad on handle 1.2 static / 1.0 dynamic (combine max); payload on cap 0.8 / 0.7", ""],
        ["Gripper drive cap", "0.3 N·m", "<code>PEGASUS_PNP_GRIP_TORQUE</code>"],
        ["Vehicle start", "(0.10, −0.07) m, heading +x", "<code>PEGASUS_PNP_SPAWN_XY</code>"],
        "Plant (the hardware-identified mirror plant)",
        ["Total mass", f'{wp["vehicle_mass"]:g} kg (airframe + arm; the controllers carry the same number)', k("vehicle_mass")],
        ["Thrust coefficient", f'×{wp["sim_plant_kf_scale"]:g} of the T650 plant k<sub>f</sub>, falling {wp["sim_plant_kf_sag_per_min"] * 100:g} %/min from lift-off', k("sim_plant_kf_scale") + ", " + k("sim_plant_kf_sag_per_min")],
        ["Yaw torque coefficient", f'×{wp["sim_plant_km_scale"]:g} of the plant c (0.70 × the bench value)', k("sim_plant_km_scale")],
        ["Rotor lag", f'λ = {wp["sim_plant_rotor_lambda"]:g} s⁻¹ (τ = {1e3 / wp["sim_plant_rotor_lambda"]:.1f} ms)', k("sim_plant_rotor_lambda")],
        ["CoM offset", f'{wp["sim_plant_com_shift_x"] * 1e3:g} mm along body x', k("sim_plant_com_shift_x")],
        ["Body-fixed disturbance", f'force ({wp["sim_plant_force_bias_x"]:g}, {wp["sim_plant_force_bias_y"]:g}, {wp["sim_plant_force_bias_z"]:g}) N, yaw torque {wp["sim_plant_torque_bias_z"]:g} N·m', k("sim_plant_force_bias_*") + ", " + k("sim_plant_torque_bias_z")],
        ["Arm joint friction", f'×({wp["sim_arm_friction_scale_j1"]:g}, {wp["sim_arm_friction_scale_j2"]:g}, {wp["sim_arm_friction_scale_j3"]:g}, {wp["sim_arm_friction_scale_j4"]:g}) of the calibration model, tanh width {wp["sim_arm_friction_width"]:g} rad/s', k("sim_arm_friction_scale_j*")],
        ["Arm current noise", f'({wp["sim_arm_current_noise_a_j1"] * 1e3:g}, {wp["sim_arm_current_noise_a_j2"] * 1e3:g}, {wp["sim_arm_current_noise_a_j3"] * 1e3:g}, {wp["sim_arm_current_noise_a_j4"] * 1e3:g}) mA RMS, {wp["sim_arm_current_noise_bw_hz"]:g} Hz band', k("sim_arm_current_noise_*")],
        ["Reported joint velocity", f'lagged {wp["sim_arm_vel_lag_s"] * 1e3:g} ms, quantised {wp["sim_arm_vel_quant_rad_s"]:g} rad/s (Present Velocity emulation)', k("sim_arm_vel_lag_s") + ", " + k("sim_arm_vel_quant_rad_s")],
        "Rotor allocation (both laws)",
        ["Thrust coefficient believed", f'{wp["alloc_thrust_coeff"]!r} N/(rad/s)²', k("alloc_thrust_coeff")],
        ["Yaw moment per newton", f'±{wp["alloc_rotor0_km"]:g} m', k("alloc_rotor*_km")],
        ["Rotor positions", f'(±{wp["alloc_rotor0_px"]:g}, ±{wp["alloc_rotor0_py"]:g}) m, Quad-X', k("alloc_rotor*_px/py")],
        ["Rotor speed range", f'{wp["alloc_omega_idle"]:g} – {wp["alloc_omega_max"]:g} rad/s', k("alloc_omega_idle/max")],
        "SAFETY mode (takeoff, landing, abort; both rigs)",
        ["Position loop", f'nested PID, k<sub>pos</sub> ({wp["posctl_k_pos_x"]:g}, {wp["posctl_k_pos_y"]:g}, {wp["posctl_k_pos_z"]:g}), k<sub>vel</sub> ({wp["posctl_k_vel_x"]:g}, {wp["posctl_k_vel_y"]:g}, {wp["posctl_k_vel_z"]:g})', k("posctl_*")],
        ["Disturbance estimator", f'UDE gain {wp["ude_gain"]:g}, active above {wp["ude_height_threshold"]:g} m, bound ±{wp["ude_disturbance_ubx"]:g} N', k("ude_*")],
        ["Thrust map", f'scaling {wp["vehicle_thrust_scaling"]:g}, idle {wp["vehicle_idle_thrust"]:g}', k("vehicle_thrust_scaling/idle_thrust")],
        ["Watchdog", f'tilt {wp["system_wd_max_tilt_deg"]:g}°, rate {wp["system_wd_max_rate_dps"]:g} °/s, horizontal drift {wp["system_wd_max_drift_m"]:g} m; feedback timeout {wp["system_fb_timeout_s"]:g} s', k("system_wd_*")],
        "Trajectory planner (one block, both rigs)",
        ["Backend", f'{pl["planner"]}, streamed at {pl["stream_rate"]:g} Hz', k("planner")],
        ["Bounds", f'v {pl["v_max"]:g} m/s, a {pl["a_max"]:g} m/s², ω {pl["w_max"]:g} rad/s, ω̇ {pl["dw_max"]:g} rad/s², leg time {pl["t_min"]:g}–{pl["t_max"]:g} s, joint torque {pl["tau_joint_max"]:g} N·m, rotor bounds {"on" if pl["rotor_bounds"] else "off"}', k("v_max") + " …"],
        ["Drone way-points [x, y, z, yaw°]", f'start {vec(pl["pick_place_start"], None)}, place start {vec(pl["pick_place_place_start"], None)}, land start {vec(pl["pick_place_land_start"], None)}, land {vec(pl["pick_place_land"], None)}', k("pick_place_start") + " …"],
        ["Arm poses [deg]", f'pick {vec(pl["pick_place_pick_pose_deg"], None)}, place {vec(pl["pick_place_place_pose_deg"], None)}, carry {vec(pl["pick_place_carry_pose_deg"], None)}', k("pick_place_*_pose_deg")],
        ["Grasp geometry", f'claw target {pl["pick_place_ee_offset"][2]:g} m above the box centre; place point {vec(pl["pick_place_place_point"], None)}; approach from {pl["pick_place_approach_dz"]:g} m above', k("pick_place_ee_offset") + ", " + k("pick_place_approach_dz")],
        ["Descent trim", f'cap {pl["pick_place_descent_trim_max"]:g} m, flown sideways first then {pl["pick_place_descent_trim_settle_s"]:g} s settle', k("pick_place_descent_trim_*")],
        ["Exit (lift-off) time", f'{pl["pick_place_exit_time_s"]:g} s', k("pick_place_exit_time_s")],
        ["Claw anchor", f'world-held for the pick descent ({"on" if pl["pick_place_world_anchor_pick"] else "off"}), CoM-held at the place ({"world" if pl["pick_place_world_anchor_place"] else "CoM"}); the decoupled law ignores it', k("pick_place_world_anchor_*")],
        ["Tilt guard", f'abort above {pl["pick_place_guard_tilt_deg"]:g}° held {pl["pick_place_guard_persist_s"]:g} s; altitude fence {pl["pick_place_fence_max_z"]:g} m', k("pick_place_guard_*")],
        "Gripper and ground station",
        ["Close / open condition", "claw within 20 mm of the target and within 3 mm across the jaws' closing axis", "<code>pick_place_grip_tol</code>, <code>pick_place_grip_axis_tol</code>"],
        ["Automatic Exit To Pick", "on the gripper action's grasp result (jaws stalled: < 1° over 0.3 s)", "<code>pick_place_auto_exit_pick</code>"],
    ]
    t_shared = table(["Item", "Value", "Key"], shared, "par")

    wbrows = [
        "Translational and attitude loops",
        ["k<sub>x</sub>, k<sub>v</sub>", f'{wp["wb_k_x"]:g}, {wp["wb_k_v"]:g}', k("wb_k_x") + ", " + k("wb_k_v")],
        ["k<sub>R</sub>, k<sub>ω</sub>", f'{wp["wb_k_r"]:g}, {wp["wb_k_w"]:g}' + ppchip("2.134, 1.567"), k("wb_k_r") + ", " + k("wb_k_w")],
        ["M<sub>r,d</sub> (desired rotational inertia)", f'diag({wp["wb_mrd_x"]:g}, {wp["wb_mrd_y"]:g}, {wp["wb_mrd_z"]:g}) kg·m²', k("wb_mrd_*")],
        "End-effector impedance (task y = [r<sub>e</sub>; heading])",
        ["K<sub>y</sub> position, D<sub>y</sub> position", f'{wp["wb_ky_x"]:g}, {wp["wb_dy_x"]:g} (x, y, z alike)', k("wb_ky_*") + ", " + k("wb_dy_*")],
        ["K<sub>ψ</sub>, D<sub>ψ</sub> heading", f'{wp["wb_ky_psi"]:g}, {wp["wb_dy_psi"]:g}', k("wb_ky_psi") + ", " + k("wb_dy_psi")],
        ["M<sub>y</sub>", f'diag({wp["wb_my_x"]:g}, {wp["wb_my_y"]:g}, {wp["wb_my_z"]:g}, {wp["wb_my_psi"]:g}), shaped impedance {"on" if wp["wb_impedance_shaped"] else "off"}', k("wb_my_*")],
        ["DLS damping, joint torque limit", f'λ = {wp["wb_dls_lambda"]:g}, τ<sub>max</sub> = {wp["wb_tau_max"]:g} N·m', k("wb_dls_lambda") + ", " + k("wb_tau_max")],
        ["Claw reference anchored to the CoM", f'{"on" if wp["wb_ee_anchor_com"] else "off"} (blend {wp["wb_ee_anchor_blend_s"]:g} s when the planner switches anchor)', k("wb_ee_anchor_com")],
        ["Internal-disturbance feed-forward in u<sub>3</sub>", f'{"on" if wp["wb_u3_internal_ff"] else "off"}, from ŵ ({"on" if wp["wb_u3_internal_ff_use_w_hat"] else "off"}); j4 estimate clamp {wp["wb_u3_estimate_max_j4"]:g} N·m', k("wb_u3_internal_ff*")],
        "Model",
        ["Base CoM offset", f'{vec([wp["wb_base_com_x"], wp["wb_base_com_y"], wp["wb_base_com_z"]], None)} m (model frame)', k("wb_base_com_*")],
        ["Arm armature, joint-diagonal", f'{vec([wp["wb_armature_j1"], wp["wb_armature_j2"], wp["wb_armature_j3"], wp["wb_armature_j4"]], None)} kg·m²', k("wb_armature_*")],
        "L1 disturbance observer, 4-D attribution",
        ["Predictor poles A<sub>s</sub> (t, r, q)", f'{wp["wb_l1_a_t"]:g}, {wp["wb_l1_a_r"]:g}, {wp["wb_l1_a_q"]:g}; adaptation every tick', k("wb_l1_a_*")],
        ["Filter bandwidth ω<sub>c</sub> (t, r, q)", f'{wp["wb_l1_omega_c_t"]:g}, {wp["wb_l1_omega_c_r"]:g}, {wp["wb_l1_omega_c_q"]:g} rad/s', k("wb_l1_omega_c_*")],
        ["ω<sub>i</sub>, ω<sub>x</sub>, ω<sub>q</sub>", f'{wp["wb_l1_omega_i"]:g}, {wp["wb_l1_omega_x"]:g}, {wp["wb_l1_omega_q"]:g} rad/s', k("wb_l1_omega_i/x/q")],
        ["Attribution prior variances (f, m, q)", f'{wp["wb_l1_lc_var_f"]:g}, {wp["wb_l1_lc_var_m"]:g}, {wp["wb_l1_lc_var_q"]:g}', k("wb_l1_lc_var_*")],
        ["Mode", f'4-D attribution {"on" if wp["wb_l1_four_d"] else "off"}, contact flag {"on" if wp["wb_l1_contact"] else "off"} (free flight; the grasp is not flagged)', k("wb_l1_four_d") + ", " + k("wb_l1_contact")],
        ["Estimate bounds", f'force {wp["wb_l1_max_force_n"]:g} N, torque {wp["wb_l1_max_torque_nm"]:g} N·m, joint {wp["wb_l1_max_joint_nm"]:g} N·m', k("wb_l1_max_*")],
        "Arm interface (torque mode)",
        ["Joint velocity used by the law", f'the arm controller\'s position observer ({at["velocity_observer_bandwidth_hz"]:g} Hz, ζ {at["velocity_observer_damping"]:g}); stale after {wp["wb_arm_velocity_timeout_s"]:g} s', k("wb_arm_velocity_topic")],
        ["DIRECT entry gate", f'position {wp["wb_gate_pos_m"]:g} m, velocity {wp["wb_gate_vel_mps"]:g} m/s, arm {wp["wb_gate_arm_rad"]:g} rad', k("wb_gate_*")],
        ["SAFETY arm hold", f'PD k<sub>p</sub> {wp["wb_arm_hold_kp"]:g}, k<sub>d</sub> {wp["wb_arm_hold_kd"]:g} + gravity', k("wb_arm_hold_kp/kd")],
        ["Arm controller", f'ExternalTorqueController, passes u<sub>3</sub> through at {at.get("state_publish_rate", 250):g} Hz, clamp {at["max_effort"][0]:g} N·m', "<code>torque_controller_isaac_aerial.yaml</code>"],
        ["Friction feed-forward", f'f<sub>c</sub> {vec(at["friction_ff"], 4)} N·m, load coefficient {vec(at["friction_load_coeff"], None)}, on the measured velocity (width {at["friction_measured_width"]:g} rad/s); gravity correction and integral off', "<code>friction_*</code>"],
    ]
    t_wb = table(["Item", "Value", "Key"], wbrows, "par")

    georows = [
        "Position loop (pure P/D, no integral)",
        ["K<sub>p</sub> (x, y, z)", f'{gp["l1geo_kp_x"]:g}, {gp["l1geo_kp_y"]:g}, {gp["l1geo_kp_z"]:g}' + ppchip("20.11, 20.11, 13.5"), k("l1geo_kp_*")],
        ["K<sub>v</sub> (x, y, z)", f'{gp["l1geo_kv_x"]:g}, {gp["l1geo_kv_y"]:g}, {gp["l1geo_kv_z"]:g}' + ppchip("11.05, 11.05, 10.82"), k("l1geo_kv_*")],
        ["Saturations", f'position error {gp["l1geo_max_pos_err_m"]:g} m, velocity error {gp["l1geo_max_vel_err_mps"]:g} m/s, tilt {gp["l1geo_max_tilt_deg"]:g}°', k("l1geo_max_*")],
        "Attitude loop (geometric SO(3), constant yaw reference)",
        ["K<sub>R</sub> (x, y, z)", f'{gp["l1geo_kr_x"]:g}, {gp["l1geo_kr_y"]:g}, {gp["l1geo_kr_z"]:g}', k("l1geo_kr_*")],
        ["K<sub>Ω</sub> (x, y, z)", f'{gp["l1geo_komega_x"]:g}, {gp["l1geo_komega_y"]:g}, {gp["l1geo_komega_z"]:g}', k("l1geo_komega_*")],
        ["Inertia model", f'J = ({gp["l1geo_inertia_xx"]:g}, {gp["l1geo_inertia_yy"]:g}, {gp["l1geo_inertia_zz"]:g}) kg·m², J<sub>xy</sub> {gp["l1geo_inertia_xy"]:g}', k("l1geo_inertia_*")],
        ["Torque limits", f'xy {gp["l1geo_max_torque_xy"]:g} N·m, z {gp["l1geo_max_torque_z"]:g} N·m', k("l1geo_max_torque_*")],
        "L1 adaptive augmentation (matched channel)",
        ["Predictor poles A<sub>s</sub> (v, ω)", f'{gp["l1adapt_as_v"]:g}, {gp["l1adapt_as_omega"]:g}', k("l1adapt_as_*")],
        ["Filter bandwidth ω<sub>c</sub>", f'{gp["l1adapt_omega_c"]:g} rad/s; sample time from the feedback stamps', k("l1adapt_omega_c")],
        ["Injection bounds", f'thrust {gp["l1adapt_max_thrust_n"]:g} N, torque xy {gp["l1adapt_max_torque_xy"]:g} / z {gp["l1adapt_max_torque_z"]:g} N·m, unmatched {gp["l1adapt_max_unmatched_n"]:g} N', k("l1adapt_max_*")],
        "Arm moment feed-forward (live r<sub>os</sub> from the arm encoders)",
        ["Model", f'base {gp["armff_base_mass"]:g} kg; links {vec([gp["armff_link1_mass"], gp["armff_link2_mass"], gp["armff_link3_mass"], gp["armff_link4_mass"]], 4)} kg; CoM trim x {gp["armff_com_trim_x"] * 1e3:g} mm', k("armff_*")],
        "Arm (position mode) and reference",
        ["Arm controller", "PositionController, pure tracking of the planner's q<sub>d</sub>", "<code>position_controller_isaac_aerial.yaml</code>"],
        ["Servo emulation (06)", "PD k<sub>p</sub> 3.0 N·m/rad, k<sub>d</sub> 0.25 N·m·s/rad + integrator k<sub>i</sub> 2.0 (clamp 0.35 N·m) + gravity", "<code>ARM_HOLD_KP/KD</code>, <code>ARM_POS_KI</code>"],
        ["Reference", "the planner's whole-body stream, split by <code>decoupled_reference_bridge.py</code> into an airframe position / yaw reference and a joint reference (base CoM [0, −0.017854, 0] m)", "<code>--base-com</code>"],
    ]
    t_geo = table(["Item", "Value", "Key"], georows, "par")

    page = open(os.path.join(HERE, "report_template.html")).read()
    sub = {
        "%%KPIS%%": kpi_html, "%%T1%%": t1, "%%T2%%": t2, "%%TSHARED%%": t_shared, "%%TWB%%": t_wb, "%%TGEO%%": t_geo,
        "%%DATA%%": json.dumps(D, separators=(",", ":"), allow_nan=False),
        "%%L_MM%%": f(L * 1e3, 0),
        "%%GRAB_W%%": f(W["grab"]["mx"].mean()), "%%GRAB_G%%": f(G["grab"]["mx"].mean()),
        "%%PLACE_W%%": f(W["place"]["mx"].mean()), "%%PLACE_G%%": f(G["place"]["mx"].mean()),
        "%%GRAB_X%%": f(G["grab"]["mx"].mean() / W["grab"]["mx"].mean()),
        "%%PLACE_X%%": f(G["place"]["mx"].mean() / W["place"]["mx"].mean()),
        "%%LIFT_W%%": rng(W["lift_pk"], 0, " mm"), "%%LIFT_G%%": rng(G["lift_pk"], 0, " mm"),
        "%%REL_W%%": rng(W["rel_pk"], 0, " mm"), "%%REL_G%%": rng(G["rel_pk"], 0, " mm"),
        "%%APP_G%%": f(G["approach"]["rm"].mean()), "%%APP_W%%": f(W["approach"]["rm"].mean()),
        "%%HOVER_G%%": rng(np.r_[G["hover_pick_mm"], G["hover_place_mm"]], 0, " mm"),
        "%%CLOSE_W%%": rng(W["ee_close_mm"], 1, " mm"), "%%CLOSE_G%%": rng(G["ee_close_mm"], 1, " mm"),
        "%%OFF_W%%": rng(W["place_off_mm"], 1, " mm"), "%%OFF_G%%": rng(G["place_off_mm"], 1, " mm"),
        "%%MIS_W%%": f(W["mission_s"].mean()), "%%MIS_G%%": f(G["mission_s"].mean()),
        "%%TILT_G1%%": f(G["tilt_max"].max()), "%%LIFTPK_G1%%": f(G["lift_pk"].max(), 0),
        "%%TILT_W%%": rng(W["tilt_max"], 1, "°"), "%%TILT_G%%": rng(G["tilt_max"], 1, "°"),
    }
    for a, b in sub.items():
        page = page.replace(a, b)
    assert "%%" not in page.replace("100%%", ""), [x for x in page.split("%%")[1::2]][:5]
    open(out_path, "w").write(page)
    print(f"wrote {out_path} ({os.path.getsize(out_path) / 1e3:.0f} kB)")


if __name__ == "__main__":
    main()
