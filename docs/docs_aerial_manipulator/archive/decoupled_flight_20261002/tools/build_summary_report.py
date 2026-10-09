#!/usr/bin/env python3
"""The free-flight comparison report: section 1 (2026-10-03, user request) = the six circle runs of 09-28 and 10-02 --
flight tags (dates, controllers, gains and parameters as flown), the runs in 3D, one full-state error table; section 2
(2026-10-05) = the six figure-8 runs of 10-05 in the same layout, data from
../../wb_vs_decoupled_figure8_flight_20261005 (tools/summary_data.py -> analysis/fig8_{metrics,ee3d}.json).
Supersedes the multi-section page build_report.py composed (published as version 3 of the artifact; that builder now
writes ../report_v3_sections.html).

    AM_NPZ=... PYTHONNOUSERSITE=1 /usr/bin/python3 summary_data.py      # -> ../analysis/summary_{metrics,ee3d}.json
    (section 2: the same command in ../../wb_vs_decoupled_figure8_flight_20261005/tools)
    python3 make_templates.py && python3 build_summary_report.py        # -> ../report.html
    python3 build_artifact_page.py <out.html>                          # refreshes the outline, publishable page
Gains are the values in the flown yamls (fsc_autopilot_ros2): whole-body 2704a13 (= the 272c0f0 tune + the 09-28
plan), decoupled afbeb35^ (old) and afbeb35 (tuned); arm controller fsc_open_manipulator b3cc576. The figure-8 flights
flew d85be8a: the same gains, tilt watchdog 15 deg on both controllers, and the figure-8 planner keys.
"""
import json, os
HERE = os.path.dirname(os.path.abspath(__file__)); AN = os.path.join(HERE, "..", "analysis")
OLD = os.path.join(HERE, "..", "..", "wb_vs_decoupled_flight_20260928")
MET = json.load(open(os.path.join(AN, "summary_metrics.json")))
AN8 = os.path.join(HERE, "..", "..", "wb_vs_decoupled_figure8_flight_20261005", "analysis")
MET8 = json.load(open(os.path.join(AN8, "fig8_metrics.json")))
KEYS = ["w1", "w2", "d1", "d2", "r1", "r2"]
GRP = {"wb": ["w1", "w2"], "old": ["d1", "d2"], "new": ["r1", "r2"]}
BAG = {"w1": "flight_wb_l1_4d_circle_20260928_171154", "w2": "flight_wb_l1_4d_circle_20260928_172242",
       "d1": "flight_decoupled_l1_circle_20260928_182835", "d2": "flight_decoupled_l1_circle_20260928_183608",
       "r1": "flight_decoupled_l1_circle_20261002_134247", "r2": "flight_decoupled_l1_circle_20261002_134247"}
CTL = {"wb": "whole-body 4-D L1", "old": "decoupled: geometric + L1 airframe", "new": "decoupled: geometric + L1 airframe"}
ARM = {"wb": "torque (joint torques from the law)", "old": "position (tracks the planner's q<sub>d</sub>)",
       "new": "position (tracks the planner's q<sub>d</sub>)"}
GAINS = {"wb": "2026-09-27 tune (H1b) + arm compensation", "old": "previous set (flown since 2026-08-21)",
         "new": "2026-10-01 tune"}
NOTE = {"r1": "run 1 of the 13:42 flight", "r2": "run 2 of the 13:42 flight; ended 0.8 s before the plan's end (SAFETY)"}


def td(v, cls=""):
    return f'<td class="{"n" + (" " + cls if cls else "")}">{v}</td>'


def grp(text, n):
    return f'<tr class="grp"><td colspan="{n}">{text}</td></tr>'


def tpl(name, data, an=AN):
    t = open(os.path.join(HERE, name), encoding="utf-8").read(); assert t.count("__DATA__") == 1
    return t.replace("__DATA__", open(os.path.join(an, data), encoding="utf-8").read().strip())


# ---- Table 1: flight tags --------------------------------------------------------------------------------------
T1 = ['<div class="tscroll"><table>', "<tr><th>tag</th><th>date, time</th><th>controller</th><th>arm control</th><th>gain set</th>"
      "<th>bag</th><th>scored [s since bag start]</th><th>pack voltage</th></tr>"]
for k in KEYS:
    m = MET[k]; g = m["grp"]
    bag = f'<code>{BAG[k]}</code>' + (f'<br><span class="note">{NOTE[k]}</span>' if k in NOTE else "")
    T1.append(f'<tr><td class="nb">{m["tag"]}</td><td class="nb">2026-{m["when"].split(" ")[0]} {m["when"].split(" ")[1]}</td>'
              f'<td>{CTL[g]}</td><td>{ARM[g]}</td><td>{GAINS[g]}</td><td>{bag}</td>'
              + td("{:.1f} – {:.1f}".format(*m["window"])) + td("{:.2f} V".format(m["metrics"]["pack_v"])) + "</tr>")
T1.append("</table></div>")

# ---- Table 2: shared by all six -------------------------------------------------------------------------------
T2 = ['<div class="tscroll"><table class="wk">', "<tr><th>item</th><th>value (all six flights)</th></tr>",
      "<tr><td>vehicle</td><td>T650 aerial manipulator with the 4-DOF OM-X arm, total mass 3.746 kg (<code>vehicle_mass</code> 3.746170)</td></tr>",
      "<tr><td>plan (<code>fsc_trajectory_planner</code>, EE Trajectory tab)</td><td>end-effector circle, radius 0.50 m about the origin, "
      "counter-clockwise, 0.13 m/s; one lap of 24.17 s inside a 28.17 s run that includes the speed-up and slow-down; gripper "
      "heading along the tangent (360° per lap), gripper level at the height it had when the circle was selected</td></tr>",
      "<tr><td>arm sweep in the plan</td><td>q2 = 25° ± 15° with a 6.04 s period (four cycles per lap), q3 = 55° − q2, "
      "q1 = q4 = 0</td></tr>",
      "<tr><td>feedback</td><td>EKF2-fused odometry (OptiTrack into PX4 EKF2), ~100 Hz; joint encoders</td></tr>",
      "<tr><td>allocator thrust coefficient</td><td>4.260431 × 10<sup>−5</sup> N/(rad/s)<sup>2</sup> (<code>alloc_thrust_coeff</code>)</td></tr>",
      "</table></div>"]

# ---- Table 3: whole-body law ----------------------------------------------------------------------------------
WB = [("position loop k<sub>x</sub> / k<sub>v</sub>", "50.03 / 12.58"),
      ("attitude loop k<sub>R</sub> / k<sub>ω</sub>", "2.134 / 1.567"),
      ("imposed inertia M<sub>r,d</sub> x / y / z [kg·m²]", "0.1400 / 0.1456 / 0.1438"),
      ("EE task stiffness / damping K<sub>y</sub> / D<sub>y</sub> (x, y, z)", "211.9 / 26.82"),
      ("EE heading K<sub>ψ</sub> / D<sub>ψ</sub>", "0.2484 / 0.2903"),
      ("L1 observer bandwidth ω<sub>c</sub>: translation / rotation / arm [rad/s]", "2.927 / 0.7428 / 0.8479"),
      ("L1 predictor poles A<sub>s</sub>: translation / rotation / arm [1/s]", "2.0 / 2.0 / 2.0"),
      ("4-D attribution: feed-forward filter ω<sub>x</sub> / joint trim ω<sub>q</sub> [rad/s]", "0.2072 / 0.2"),
      ("EE reference anchored to the measured CoM; arm-channel internal feed-forward", "on / on"),
      ("arm model armature, joint diagonal [kg·m²]", "0.010 / 0.0194 / 0.0097 / 0.0097"),
      ("joint velocity the law closes on", "the arm controller's velocity observer"),
      ("joint torque limit", "3.0 N·m"),
      ("arm controller (pass-through)", "friction feed-forward j2 ×0.70, j3 ×0.65 of the bench model, switched on the measured "
       "joint velocity; gravity-scale correction on; no integral"),
      ("watchdog: tilt / body rate / drift", "20° / 360°/s / 0.75 m")]
T3 = ['<div class="tscroll"><table class="wk">', '<tr><th>parameter</th><th class="rig-wb">WB-1, WB-2</th></tr>']
T3 += [f"<tr><td>{a}</td><td>{b}</td></tr>" for a, b in WB]
T3.append("</table></div>")

# ---- Table 4: decoupled law, old vs tuned -----------------------------------------------------------------------
DEC = [("position gain K<sub>p</sub> x, y / z [N/m]", "4.0 / 8.0", "20.11 / 13.5"),
       ("velocity gain K<sub>v</sub> x, y / z [N·s/m]", "6.0 / 10.0", "11.05 / 10.82"),
       ("attitude gain k<sub>R</sub> x, y / z", "1.0 / 0.49", "3.337 / 1.737"),
       ("rate gain k<sub>ω</sub> x, y / z", "0.55 / 0.35", "0.9505 / 0.4105"),
       ("L1 predictor poles A<sub>s</sub>: velocity / body rate [1/s]", "2.0 / 2.0", "4.334 / 2.791"),
       ("L1 low-pass bandwidth ω<sub>c</sub> [rad/s]", "6.0", "1.0"),
       ("L1 injection bounds: thrust / torque x, y / torque z", "18 N / 1.5 N·m / 0.25 N·m", "same"),
       ("model inertia xx / yy / zz / xy [kg·m²]", "0.118 / 0.119 / 0.131 / 0.0089", "same"),
       ("arm CoM offset r<sub>os</sub>", "live from the arm encoders, CoM trim x −14.8 mm", "same"),
       ("desired body rate", "ω<sub>d</sub> = 0 (the paper's constant-yaw specialization)", "same"),
       ("watchdog: tilt / body rate / drift", "40° / 360°/s / off", "20° / 360°/s / 0.75 m")]
T4 = ['<div class="tscroll"><table class="wk">', '<tr><th>parameter</th><th class="rig-dec">DEC-1, DEC-2 (old)</th>'
      '<th class="rig-new">DEC-3, DEC-4 (tuned)</th></tr>']
T4 += [f"<tr><td>{a}</td><td>{b}</td><td>{c}</td></tr>" for a, b, c in DEC]
T4.append("</table></div>")

# ---- Table 5: full-state RMS tracking error ---------------------------------------------------------------------
NC = 8
M = {k: MET[k]["metrics"] for k in KEYS}


def row(label, unit, get, fmt="{:.1f}", score=lambda v: v):
    v = {k: get(M[k]) for k in KEYS}
    best = None
    if score is not None:
        means = {g: sum(v[k] for k in ks) / 2 for g, ks in GRP.items()}
        best = min(means, key=lambda g: score(means[g]))
        if len({round(score(x), 9) for x in means.values()}) == 1:
            best = None
    cells = "".join(td(fmt.format(v[k]), "win" if best and k in GRP[best] else "") for k in KEYS)
    return f'<tr><td>{label}</td><td class="unit">{unit}</td>{cells}</tr>'


T5 = ['<div class="tscroll"><table class="rmse">',
      '<tr><th>state</th><th>unit</th>' + "".join(f'<th class="{c}">{MET[k]["tag"]}</th>' for k, c in
      zip(KEYS, ["rig-wb", "rig-wb", "rig-dec", "rig-dec", "rig-new", "rig-new"])) + "</tr>"]
T5.append(grp("Airframe position (body origin) vs the plan's airframe path", NC))
for i, ax in enumerate("xyz"):
    T5.append(row(ax, "mm", lambda m, i=i: m["base_pos"]["rms_xyz"][i]))
T5.append(row("‖·‖", "mm", lambda m: m["base_pos"]["rms_norm"]))
T5.append(grp("System centre of mass vs the planned CoM", NC))
T5.append(row("‖·‖", "mm", lambda m: m["com_pos"]["rms_norm"]))
T5.append(grp("Airframe velocity", NC))
for i, ax in enumerate("xyz"):
    T5.append(row(ax, "mm/s", lambda m, i=i: m["base_vel"]["rms_xyz"][i]))
T5.append(row("‖·‖", "mm/s", lambda m: m["base_vel"]["rms_norm"]))
T5.append(grp("Attitude vs the plan's attitude (body axes)", NC))
for i, ax in enumerate(["roll", "pitch", "yaw"]):
    T5.append(row(ax, "deg", lambda m, i=i: m["att"]["rms"][i], "{:.2f}"))
T5.append(grp("Body rate vs the plan's rate", NC))
for i, ax in enumerate(["roll", "pitch", "yaw"]):
    T5.append(row(ax, "deg/s", lambda m, i=i: m["rate"]["rms"][i]))
T5.append(grp("Arm joint angle vs q<sub>d</sub>", NC))
for i in range(4):
    T5.append(row(f"q{i+1}", "deg", lambda m, i=i: m["joint"]["rms"][i], "{:.2f}"))
T5.append(grp("Arm joint rate vs q̇<sub>d</sub>", NC))
for i in range(4):
    T5.append(row(f"q̇{i+1}", "deg/s", lambda m, i=i: m["joint_vel"]["rms"][i]))
T5.append(grp("End-effector (the task)", NC))
for i, ax in enumerate("xyz"):
    T5.append(row(f"position {ax}", "mm", lambda m, i=i: m["ee_pos"]["rms_xyz"][i]))
T5.append(row("position ‖·‖", "mm", lambda m: m["ee_pos"]["rms_norm"]))
T5.append(row("heading", "deg", lambda m: m["ee_head"]["rms"], "{:.2f}"))
T5.append(grp("Other", NC))
T5.append(row("EE position, worst sample", "mm", lambda m: m["ee_pos"]["max_norm"], "{:.0f}"))
T5.append(row("EE circle radius flown (plan 0.500)", "m", lambda m: m["ee_radius_meas_mean_m"], "{:.3f}", score=lambda v: abs(v - 0.5)))
T5.append(row("tilt, worst sample", "deg", lambda m: m["tilt_max_deg"], "{:.2f}"))
T5.append(row("law's own attitude error ‖e<sub>R</sub>‖", "—", lambda m: m["law_eR_rms"], "{:.3f}"))
T5.append(row("motor-saturated ticks", "count", lambda m: m["sat"], "{:.0f}", score=None))
T5.append("</table></div>")

# =================================================================================================================
# Section 2 (2026-10-05): the six figure-8 runs
# =================================================================================================================
K8 = ["w3", "w4", "r5", "r6", "w5", "w6"]                 # flight order (Table 6, Fig. 2)
K8T = ["w3", "w4", "r5", "w5", "w6", "r6"]                # grouped by speed (Table 9)
SPD8 = {0.10: ["w3", "w4", "r5"], 0.13: ["w5", "w6", "r6"]}
NOTE8 = {"w3": "run 1 of the 16:24 flight", "w4": "run 2 of the 16:24 flight", "w5": "run 1 of the 17:01 flight",
         "w6": "run 2 of the 17:01 flight",
         "r5": "the flight's second DIRECT engagement; the operator returned to SAFETY from the first before any trajectory"}

T6 = ['<div class="tscroll"><table>', "<tr><th>tag</th><th>date, time</th><th>controller</th><th>arm control</th><th>gain set</th>"
      "<th>mean speed</th><th>bag</th><th>scored [s since bag start]</th><th>pack voltage</th></tr>"]
for k in K8:
    m = MET8[k]; g = m["grp"]
    bag = f'<code>{m["bag"]}</code>' + (f'<br><span class="note">{NOTE8[k]}</span>' if k in NOTE8 else "")
    T6.append(f'<tr><td class="nb">{m["tag"]}</td><td class="nb">2026-{m["when"].split(" ")[0]} {m["when"].split(" ")[1]}</td>'
              f'<td>{CTL[g]}</td><td>{ARM[g]}</td><td>{GAINS[g]}</td>' + td("{:.2f} m/s".format(m["speed"])) + f'<td>{bag}</td>'
              + td("{:.1f} – {:.1f}".format(*m["window"])) + td("{:.2f} V".format(m["metrics"]["pack_v"])) + "</tr>")
T6.append("</table></div>")

T7 = ['<div class="tscroll"><table class="wk">', "<tr><th>item</th><th>value (all six runs)</th></tr>",
      "<tr><td>vehicle</td><td>T650 aerial manipulator with the 4-DOF OM-X arm, total mass 3.746 kg (<code>vehicle_mass</code> 3.746170)</td></tr>",
      "<tr><td>plan (<code>fsc_trajectory_planner</code>, EE Trajectory tab)</td><td>end-effector figure-8 (Gerono lemniscate), half-length "
      "A 0.70 m and half-width B 0.35 m, so 1.40 × 0.70 m, long axis on world x, crossing at the gripper's position when the figure-8 was "
      "selected; path 4.268 m per lap; one lap inside a run that adds a 4 s speed-up and slow-down; gripper heading along the tangent "
      "(net 0° per lap, the turn reverses between the two lobes), gripper level at the height it had when the figure-8 was selected</td></tr>",
      "<tr><td>speed</td><td>0.10 m/s mean: lap 42.68 s, run 46.68 s, peak gripper speed 0.15 m/s, peak heading rate 27°/s<br>"
      "0.13 m/s mean: lap 32.83 s, run 36.83 s, peak gripper speed 0.19 m/s, peak heading rate 35°/s</td></tr>",
      "<tr><td>arm sweep in the plan</td><td>q2 = 25° ± 15°, four cycles per lap (10.67 s period at 0.10 m/s, 8.21 s at 0.13 m/s), "
      "q3 = 55° − q2, q1 = 0, |q4| ≤ 0.9°</td></tr>",
      "<tr><td>planner bounds for the EE run</td><td>CoM acceleration 0.40 m/s², yaw rate 1.0 rad/s, CoM speed 0.30 m/s "
      "(<code>ee_traj_a_max</code>, <code>ee_traj_w_max</code>, shared <code>v_max</code>)</td></tr>",
      "<tr><td>feedback</td><td>EKF2-fused odometry (OptiTrack into PX4 EKF2), ~100 Hz; joint encoders</td></tr>",
      "<tr><td>allocator thrust coefficient</td><td>4.260431 × 10<sup>−5</sup> N/(rad/s)<sup>2</sup> (<code>alloc_thrust_coeff</code>)</td></tr>",
      "</table></div>"]

T8 = ['<div class="tscroll"><table class="wk">', '<tr><th>item</th><th class="rig-wb">WB-3 to WB-6</th><th class="rig-new">DEC-5, DEC-6</th></tr>',
      "<tr><td>controller gains and parameters</td><td>as Table 3</td><td>as Table 4, tuned column</td></tr>",
      "<tr><td>watchdog: tilt / body rate / drift</td><td>15° / 360°/s / 0.75 m</td><td>15° / 360°/s / 0.75 m</td></tr>",
      "<tr><td>arm controller</td><td>as Table 3 (fsc_open_manipulator <code>b3cc576</code>)</td><td>position mode, pure tracking of the planner's q<sub>d</sub> (<code>f5dee7d</code>)</td></tr>",
      "</table></div>"]

M8 = {k: MET8[k]["metrics"] for k in K8}
NC8 = 8


def row8(label, unit, get, fmt="{:.1f}", score=lambda v: v):
    v = {k: get(M8[k]) for k in K8T}
    win = set()
    if score is not None:
        for ks in SPD8.values():
            means = {}
            for k in ks:
                means.setdefault(MET8[k]["grp"], []).append(v[k])
            means = {g: sum(x) / len(x) for g, x in means.items()}
            if len({round(score(x), 9) for x in means.values()}) > 1:
                best = min(means, key=lambda g: score(means[g]))
                win |= {k for k in ks if MET8[k]["grp"] == best}
    cells = "".join(td(fmt.format(v[k]), "win" if k in win else "") for k in K8T)
    return f'<tr><td>{label}</td><td class="unit">{unit}</td>{cells}</tr>'


T9 = ['<div class="tscroll"><table class="rmse">',
      '<tr><th></th><th></th><th colspan="3">0.10 m/s</th><th colspan="3">0.13 m/s</th></tr>',
      '<tr><th>state</th><th>unit</th>' + "".join(f'<th class="{"rig-wb" if MET8[k]["grp"] == "wb" else "rig-new"}">{MET8[k]["tag"]}</th>'
                                                  for k in K8T) + "</tr>"]
T9.append(grp("Airframe position (body origin) vs the plan's airframe path", NC8))
for i, ax in enumerate("xyz"):
    T9.append(row8(ax, "mm", lambda m, i=i: m["base_pos"]["rms_xyz"][i]))
T9.append(row8("‖·‖", "mm", lambda m: m["base_pos"]["rms_norm"]))
T9.append(grp("System centre of mass vs the planned CoM", NC8))
T9.append(row8("‖·‖", "mm", lambda m: m["com_pos"]["rms_norm"]))
T9.append(grp("Airframe velocity", NC8))
for i, ax in enumerate("xyz"):
    T9.append(row8(ax, "mm/s", lambda m, i=i: m["base_vel"]["rms_xyz"][i]))
T9.append(row8("‖·‖", "mm/s", lambda m: m["base_vel"]["rms_norm"]))
T9.append(grp("Attitude vs the plan's attitude (body axes)", NC8))
for i, ax in enumerate(["roll", "pitch", "yaw"]):
    T9.append(row8(ax, "deg", lambda m, i=i: m["att"]["rms"][i], "{:.2f}"))
T9.append(grp("Body rate vs the plan's rate", NC8))
for i, ax in enumerate(["roll", "pitch", "yaw"]):
    T9.append(row8(ax, "deg/s", lambda m, i=i: m["rate"]["rms"][i]))
T9.append(grp("Arm joint angle vs q<sub>d</sub>", NC8))
for i in range(4):
    T9.append(row8(f"q{i+1}", "deg", lambda m, i=i: m["joint"]["rms"][i], "{:.2f}"))
T9.append(grp("Arm joint rate vs q̇<sub>d</sub>", NC8))
for i in range(4):
    T9.append(row8(f"q̇{i+1}", "deg/s", lambda m, i=i: m["joint_vel"]["rms"][i]))
T9.append(grp("End-effector (the task)", NC8))
for i, ax in enumerate("xyz"):
    T9.append(row8(f"position {ax}", "mm", lambda m, i=i: m["ee_pos"]["rms_xyz"][i]))
T9.append(row8("position ‖·‖", "mm", lambda m: m["ee_pos"]["rms_norm"]))
T9.append(row8("heading", "deg", lambda m: m["ee_head"]["rms"], "{:.2f}"))
T9.append(grp("Other", NC8))
T9.append(row8("EE position, worst sample", "mm", lambda m: m["ee_pos"]["max_norm"], "{:.0f}"))
T9.append(row8("EE figure-8 extent along x (plan 1.400)", "m", lambda m: m["ee_extent_m"][0], "{:.3f}", score=lambda v: abs(v - 1.4)))
T9.append(row8("EE figure-8 extent along y (plan 0.700)", "m", lambda m: m["ee_extent_m"][1], "{:.3f}", score=lambda v: abs(v - 0.7)))
T9.append(row8("tilt, worst sample", "deg", lambda m: m["tilt_max_deg"], "{:.2f}"))
T9.append(row8("law's own attitude error ‖e<sub>R</sub>‖", "—", lambda m: m["law_eR_rms"], "{:.3f}"))
T9.append(row8("motor-saturated ticks", "count", lambda m: m["sat"], "{:.0f}", score=None))
T9.append("</table></div>")
assert all(abs(M8[k]["ee_extent_ref_m"][0] - 1.4) < 2e-3 and abs(M8[k]["ee_extent_ref_m"][1] - 0.7) < 2e-3 for k in K8)

BODY = f"""
<h1>Whole-body vs decoupled on the end-effector circle and figure-8</h1>
<p class="sub">T650 aerial manipulator, hardware · six runs of the same planned circle: the whole-body 4-D L1 controller (2026-09-28) and the decoupled geometric + L1 controller on its old gains (2026-09-28) and its tuned gains (2026-10-02) · six runs of the same planned figure-8 at 0.10 and 0.13 m/s: both controllers, the decoupled one on its tuned gains (2026-10-05)</p>

<h2>1. Tracking performance on the end-effector circle</h2>

<h3>1.1 Flights, controllers and parameters</h3>
<p><strong>Table 1.</strong> The six circle runs. Every flight hovered in SAFETY, switched to DIRECT, selected the circle in the arm ground station's EE Trajectory tab, went to its start and ran it.</p>
{"".join(T1)}
<p><strong>Table 2.</strong> Settings shared by all six flights.</p>
{"".join(T2)}
<p><strong>Table 3.</strong> Whole-body controller, as flown (<code>params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml</code>, fsc_autopilot_ros2 <code>2704a13</code>; arm controller fsc_open_manipulator <code>b3cc576</code>).</p>
{"".join(T3)}
<p><strong>Table 4.</strong> Decoupled controller, as flown (<code>params_single_aerial_manipulator_geometric_l1_direct_actuation_t650.yaml</code>: old set before fsc_autopilot_ros2 <code>afbeb35</code>, tuned set from <code>afbeb35</code>). Its arm runs in position mode; the planner's airframe path reaches the law through the reference bridge.</p>
{"".join(T4)}

<h3>1.2 3D trajectories</h3>
<!--ee3ds:start-->
{tpl("summary_ee3d_section.html", "summary_ee3d.json")}<!--ee3ds:end-->

<h3>1.3 Full-state tracking errors</h3>
<p>One definition for all six runs, computed only from signals every flight records. Measured: the EKF2-fused odometry (airframe position, world velocity, attitude, body rates) and the joint encoders; the centre of mass, the gripper position and the gripper heading follow from the planner's own kinematic model. Reference: the planner's stream, the same message on both controllers. The airframe reference is the decoupled bridge's conversion of that stream, the reference attitude is the plan's compatible attitude, and joint rates are a 0.1 s central difference of the encoder angle. Every run is scored over the first 27.2 s of its circle run (0.1 s trimmed at the start) on a 100 Hz grid. The law's own attitude error is each law's internal ‖e<sub>R</sub>‖; on the decoupled controller it is mostly yaw. The bold pair is the controller setting with the lowest mean of its two runs.</p>
<p><strong>Table 5.</strong> RMS tracking error over the circle run.</p>
{"".join(T5)}

<h2>2. Tracking performance on the end-effector figure-8</h2>

<h3>2.1 Flights, controllers and parameters</h3>
<p><strong>Table 6.</strong> The six figure-8 runs. Every flight hovered in SAFETY, switched to DIRECT, selected the figure-8 in the arm ground station's EE Trajectory tab, went to its start and ran it; the whole-body flights ran it twice from one start.</p>
{"".join(T6)}
<p><strong>Table 7.</strong> Settings shared by all six runs.</p>
{"".join(T7)}
<p><strong>Table 8.</strong> Controllers, as flown (fsc_autopilot_ros2 <code>d85be8a</code>, the yamls of Tables 3 and 4). The gains are those of the circle flights; the tilt watchdog is 15° on both controllers.</p>
{"".join(T8)}

<h3>2.2 3D trajectories</h3>
<!--ee3d8:start-->
{tpl("fig8_ee3d_section.html", "fig8_ee3d.json", AN8)}<!--ee3d8:end-->

<h3>2.3 Full-state tracking errors</h3>
<p>The definitions of section 1.3, on the same signals and the same 100 Hz grid. Every run is scored over its whole figure-8 run, 0.1 s trimmed at each end: 46.5 s at 0.10 m/s and 36.6 s at 0.13 m/s, speed-up and slow-down included. Bold marks, at each speed, the controller with the lower mean over its runs at that speed (two whole-body runs, one decoupled run).</p>
<p><strong>Table 9.</strong> RMS tracking error over the figure-8 run.</p>
{"".join(T9)}
"""

style = open(os.path.join(OLD, "tools", "report_style.css"), encoding="utf-8").read()
assert style.count("--panel:#141413}") == 1 and style.count("table.rmse th.rig-old{") == 1
style = style.replace("--panel:#141413}", "--panel:#141413;--violet:#9085e9}").replace(
    "table.rmse th.rig-old{", "table.rmse th.rig-new{border-bottom:2px solid var(--violet)}table.rmse th.rig-old{")
style = style.replace("</style>", "table.wk th.rig-wb{border-bottom:2px solid var(--blue)}table.wk th.rig-dec{border-bottom:2px solid var(--orange)}"
                      "table.wk th.rig-new{border-bottom:2px solid var(--violet)}\n</style>")
doc = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Experiment: Free-flight Comparison 0928 + 1002 + 1005</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
{style}
</head>
<body>
{BODY}
</body>
</html>
"""
import sys
sys.path.insert(0, HERE)
import add_outline
doc, _ = add_outline.process(doc)
open(os.path.join(HERE, "..", "report.html"), "w", encoding="utf-8").write(doc)
print("wrote report.html", len(doc))
