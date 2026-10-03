#!/usr/bin/env python3
"""Build ../report.html from the analysis JSON (every table number comes from the scripts) and the three figure
templates. Then build_artifact_page.py <out.html> makes the publishable page (it refreshes the outline first).

    PYTHONNOUSERSITE=1 /usr/bin/python3 build_report.py
Inputs (../analysis/): metrics.json, arm_stats.json, ee_budget.json, mechanism.json, holds.json, ff_check.json,
ee3d.json, err_ts.json, arm_trk.json.
"""
import json, os, html

HERE = os.path.dirname(os.path.abspath(__file__))
AN = os.path.join(HERE, "..", "analysis")
J = lambda f: json.load(open(os.path.join(AN, f)))
M = J("metrics.json"); A = J("arm_stats.json"); B = J("ee_budget.json"); MECH = J("mechanism.json"); HOLD = J("holds.json"); FF = J("ff_check.json")
ARM = {(r["nm"], r["joint"]): r for r in A}


def tpl(name, data):
    t = open(os.path.join(HERE, name), encoding="utf-8").read()
    assert t.count("__DATA__") == 1
    return t.replace("__DATA__", open(os.path.join(AN, data), encoding="utf-8").read().strip())


def td(v, cls=""):
    c = "n" + (" " + cls if cls else "")
    return f'<td class="{c}">{v}</td>'


# ---------------------------------------------------------------- Table 1: full-state RMSE, today -------------
TODAY = ["w1", "w2", "d1", "d2"]


def row1(label, unit, get, fmt="{:.1f}", lower_better=True, ratio=True):
    vals = [get(M[n]["metrics"]) for n in TODAY]
    wb = (vals[0] + vals[1]) / 2; dec = (vals[2] + vals[3]) / 2
    cw, cd = ("win", "lose") if (wb <= dec) == lower_better else ("lose", "win")
    if abs(wb - dec) < 1e-9 * max(1, abs(wb)):
        cw = cd = ""
    cells = [td(fmt.format(v), cw if i < 2 else cd) for i, v in enumerate(vals)]
    rat = (f"{dec / wb:.1f}×" if wb > 1e-9 else "—") if ratio else "—"
    return f'<tr><td>{label}</td><td class="unit">{unit}</td>{"".join(cells)}{td(rat)}</tr>'


def grp(text, n=7):
    return f'<tr class="grp"><td colspan="{n}">{text}</td></tr>'


T1 = ['<div class="tscroll"><table class="rmse">',
      '<tr><th>state</th><th>unit</th><th class="rig-wb">WB-1</th><th class="rig-wb">WB-2</th><th class="rig-dec">DEC-1</th><th class="rig-dec">DEC-2</th><th>DEC ÷ WB</th></tr>']
T1.append(grp("Airframe position (body origin) vs the plan's airframe path"))
for k, ax in enumerate("xyz"):
    T1.append(row1(ax, "mm", lambda m, k=k: m["base_pos"]["rms_xyz"][k]))
T1.append(row1("‖·‖", "mm", lambda m: m["base_pos"]["rms_norm"]))
T1.append(grp("System centre of mass vs the planned CoM"))
T1.append(row1("‖·‖", "mm", lambda m: m["com_pos"]["rms_norm"]))
T1.append(grp("Airframe velocity"))
for k, ax in enumerate("xyz"):
    T1.append(row1(ax, "mm/s", lambda m, k=k: m["base_vel"]["rms_xyz"][k]))
T1.append(row1("‖·‖", "mm/s", lambda m: m["base_vel"]["rms_norm"]))
T1.append(grp("Attitude vs the plan's attitude (body axes)"))
for k, ax in enumerate(["roll", "pitch", "yaw"]):
    T1.append(row1(ax, "deg", lambda m, k=k: m["att"]["rms"][k], "{:.2f}"))
T1.append(grp("Body rate vs the plan's rate"))
for k, ax in enumerate(["roll", "pitch", "yaw"]):
    T1.append(row1(ax, "deg/s", lambda m, k=k: m["rate"]["rms"][k]))
T1.append(grp("Arm joint angle vs q_d"))
for k in range(4):
    T1.append(row1(f"q{k+1}", "deg", lambda m, k=k: m["joint"]["rms"][k], "{:.2f}"))
T1.append(grp("Arm joint rate vs q̇_d"))
for k in range(4):
    T1.append(row1(f"q̇{k+1}", "deg/s", lambda m, k=k: m["joint_vel"]["rms"][k]))
T1.append(grp("End-effector (the task)"))
for k, ax in enumerate("xyz"):
    T1.append(row1(f"position {ax}", "mm", lambda m, k=k: m["ee_pos"]["rms_xyz"][k]))
T1.append(row1("position ‖·‖", "mm", lambda m: m["ee_pos"]["rms_norm"]))
T1.append(row1("heading", "deg", lambda m: m["ee_head"]["rms"], "{:.2f}"))
T1.append(grp("Other"))
T1.append(row1("EE position, worst sample", "mm", lambda m: m["ee_pos"]["max_norm"], "{:.0f}"))
T1.append(row1("EE circle radius flown (plan 0.500)", "m", lambda m: m["ee_radius_meas_mean_m"], "{:.3f}", ratio=False))
T1.append(row1("tilt, worst sample", "deg", lambda m: m["tilt_max_deg"], "{:.2f}"))
T1.append(row1("law's own attitude error ‖e_R‖", "—", lambda m: m["law_eR_rms"], "{:.3f}"))
T1.append(row1("motor-saturated ticks", "count", lambda m: m["sat"], "{:d}", ratio=False))
T1.append("</table></div>")

# ---------------------------------------------------------------- Table 2: EE budget ---------------------------
T2 = ['<div class="tscroll"><table class="rmse">',
      '<tr><th>flight</th><th>EE position error ‖·‖ rms [mm]</th><th>from the airframe position [mm]</th><th>from the airframe attitude [mm]</th><th>from the arm joints [mm]</th></tr>']
for n in TODAY:
    b = B[n]
    cells = "".join(td("{:.1f}".format(b[k])) for k in ("total", "base", "attitude", "joints"))
    T2.append('<tr><td class="nb">' + M[n]["label"] + '</td>' + cells + '</tr>')
T2.append("</table></div>")

# ---------------------------------------------------------------- Table 3: the lateral force ---------------------
f2 = lambda v: f"{v[0]:+.2f}, {v[1]:+.2f}"
T3 = ['<div class="tscroll"><table class="rmse">',
      '<tr><th>flight</th><th>lateral force estimate, body x, y [N]</th><th>what the law does with it</th><th>K<sub>p</sub>e<sub>p</sub> + K<sub>v</sub>e<sub>v</sub>, body x, y [N]</th>'
      '<th>airframe error, body x, y, mean [mm]</th><th>airframe offset at the start hold [mm]</th><th>heading error, mean [deg]</th></tr>']
for n in TODAY:
    r = MECH[n]; wb = n[0] == "w"
    eab = r["airframe_err_body_mm"]
    what = "subtracts it in the thrust vector (−d̂<sub>t</sub>)" if wb else "none: an unmatched force, outside the L1 channel"
    T3.append('<tr><td class="nb">' + M[n]["label"] + '</td>' + td(f2(r["force_body_N"])) + '<td>' + what + '</td>'
              + td("—" if wb else f2(r["kp_ep_kv_ev_body_N"])) + td("{:+.0f}, {:+.0f}".format(eab[0], eab[1]))
              + td("{:.0f}".format(r["hold_offset_mm"])) + td("{:+.1f}".format(r["yaw_err_mean_deg"])) + '</tr>')
T3.append("</table></div>")

# ---------------------------------------------------------------- Table 1b: flights ------------------------------
BATT = {"w1": 24.02, "w2": 23.16, "d1": 22.87, "d2": 24.12}
END = {"w1": "planner descent to 0.35 m, pilot STAB, disarm", "w2": "planner descent to 0.35 m, go-home, pilot STAB, disarm",
       "d1": "SAFETY in the air, arm folds home", "d2": "SAFETY in the air, arm folds home"}
TF = ['<div class="tscroll"><table>', '<tr><th>flight</th><th>controller</th><th>DIRECT [s since bag start]</th><th>circle run [s]</th><th>pack voltage in the run</th><th>how it ended</th></tr>']
for n in TODAY:
    m = M[n]; wb = n[0] == "w"
    ctl = "whole-body 4-D L1, torque-mode arm" if wb else "decoupled: geometric + L1 airframe, position-mode arm"
    TF.append('<tr><td class="nb">' + m["label"] + '</td><td>' + ctl + '</td>'
              + td("{:.1f} – {:.1f}".format(*m["direct"])) + td("{:.1f} – {:.1f}".format(*m["window"])) + td("{:.2f} V".format(BATT[n]))
              + '<td>' + END[n] + '</td></tr>')
TF.append("</table></div>")

# ---------------------------------------------------------------- Table 4: what changed -----------------------------
FFd = lambda n, j, k: FF[n][j][k]
T4 = ['<div class="tscroll"><table>', '<tr><th>what</th><th>0924 flights</th><th>today</th><th>seen in today\'s flight data</th></tr>',
      grp("Whole-body law gains (hardware yaml, fsc_autopilot_ros2 7cfc709 → 272c0f0; the tune called H1b)", 4),
      '<tr><td>position loop k<sub>x</sub> / k<sub>v</sub></td><td class="n">32.0 / 20.0</td><td class="n">50.03 / 12.58</td><td rowspan="6">The bag does not record parameters. The stack script refuses to start on a node build without the new keys, and the debug array is the new build\'s (115 elements, observer slots [106..114] live).</td></tr>',
      '<tr><td>attitude loop k<sub>R</sub> / k<sub>ω</sub></td><td class="n">2.0 / 1.5</td><td class="n">2.134 / 1.567</td></tr>',
      '<tr><td>imposed inertia M<sub>r,d</sub> x / y / z [kg·m²]</td><td class="n">0.1165 / 0.1361 / 0.1251</td><td class="n">0.1400 / 0.1456 / 0.1438</td></tr>',
      '<tr><td>EE task K<sub>y</sub> / D<sub>y</sub></td><td class="n">20 / 12</td><td class="n">211.9 / 26.82</td></tr>',
      '<tr><td>EE heading K<sub>ψ</sub> / D<sub>ψ</sub></td><td class="n">0.30 / 0.30</td><td class="n">0.2484 / 0.2903</td></tr>',
      '<tr><td>observer ω<sub>c</sub> translation / rotation / arm, ω<sub>x</sub> [rad/s]</td><td class="n">2 / 0.5 / 0.5, 0.25</td><td class="n">2.927 / 0.7428 / 0.8479, 0.2072</td></tr>',
      grp("Arm model inside the law", 4),
      '<tr><td>servo armature</td><td>0.020 kg·m² folded into each link (J·hhᵀ)</td><td>joint diagonal [0.010, 0.0194, 0.0097, 0.0097] kg·m², bench-calibrated</td><td>new build (see above)</td></tr>',
      f'<tr><td>joint velocity the law closes on</td><td>Present Velocity (~50 ms lag)</td><td>the arm\'s velocity observer (~12 ms)</td><td class="n">observer used on {FF["w1"]["observer_pct"]:.0f} % / {FF["w2"]["observer_pct"]:.0f} % of ticks</td></tr>',
      grp("Arm controller friction feed-forward (fsc_open_manipulator b3cc576)", 4),
      f'<tr><td>j2: f<sub>c</sub> + µ|τ| [duty counts]</td><td class="n">4.705 + 0.246|τ|</td><td class="n">3.294 + 0.172|τ| (×0.70)</td><td class="n">p99 {FFd("a4","j2","ff_p99"):.3f} → {FFd("w1","j2","ff_p99"):.3f} / {FFd("w2","j2","ff_p99"):.3f} N·m</td></tr>',
      f'<tr><td>j3: f<sub>c</sub> + µ|τ| [duty counts]</td><td class="n">7.7776 + 0.161|τ|</td><td class="n">5.055 + 0.105|τ| (×0.65)</td><td class="n">p99 {FFd("a4","j3","ff_p99"):.3f} → {FFd("w1","j3","ff_p99"):.3f} / {FFd("w2","j3","ff_p99"):.3f} N·m</td></tr>',
      f'<tr><td>velocity the relay switches on</td><td>the reference q̇<sub>d</sub></td><td>the measured q̇, width 0.03 rad/s</td><td>when measured and reference velocity disagree, the FF follows the measured one {FFd("w1","j2","follows_measured_pct"):.0f} / {FFd("w1","j3","follows_measured_pct"):.0f} / {FFd("w2","j2","follows_measured_pct"):.0f} / {FFd("w2","j3","follows_measured_pct"):.0f} % (0924 F4: {FFd("a4","j2","follows_measured_pct"):.0f} %). While a joint is stuck and the reference moves, the FF averages {FFd("w1","j2","ff_when_stuck"):.3f}–{FFd("w2","j2","ff_when_stuck"):.3f} N·m on j2 and {FFd("w2","j3","ff_when_stuck"):.3f}–{FFd("w1","j3","ff_when_stuck"):.3f} on j3, against {FFd("a4","j2","ff_when_stuck"):.2f} and {FFd("a4","j3","ff_when_stuck"):.2f} on 0924 F4.</td></tr>',
      grp("Unchanged", 4),
      '<tr><td>plan</td><td colspan="2">same planner and EE-circle design: r 0.50 m, one lap, fold 55°, q2 25° ± 15° at a 6 s period (0924 F4 and today; F1/F2 flew 30° ± 10° at 48 s, F3 25° ± 15° at 12 s)</td><td class="n">lap 23.98 s (F4) vs 24.17 s</td></tr>',
      '<tr><td>feedback, vehicle</td><td colspan="2">EKF2-fused odometry, the same T650 aerial manipulator; batteries differ per flight</td><td></td></tr>',
      "</table></div>"]

# ---------------------------------------------------------------- Table 5: week over week ---------------------------
WK = ["a1", "a2", "a3", "a4", "w1", "w2"]
HEAD5 = ("<tr><th>metric</th><th>unit</th>" + "".join(f'<th class="rig-old">{M[n]["label"].replace("0924 ", "")}</th>' for n in WK[:4])
         + "".join(f'<th class="rig-wb">{M[n]["label"].split(" ")[0]}</th>' for n in WK[4:]) + "</tr>")
DESIGN = {"a1": "30 ± 10°, 48 s", "a2": "30 ± 10°, 48 s", "a3": "25 ± 15°, 12 s", "a4": "25 ± 15°, 6 s", "w1": "25 ± 15°, 6 s", "w2": "25 ± 15°, 6 s"}


def row5(label, unit, get, fmt="{:.1f}", score=lambda v: v):
    """score(v): lower is better; today's cells are bold when better than 0924 F4, grey when worse; None = no comparison."""
    vals = [get(n) for n in WK]
    cells = []
    for i, v in enumerate(vals):
        cls = ""
        if score is not None and i >= 4 and not isinstance(v, str):
            cls = "win" if score(v) < score(vals[3]) else "lose"
        cells.append(td(v, "txt") if isinstance(v, str) else td(fmt.format(v), cls))
    return f'<tr><td>{label}</td><td class="unit">{unit}</td>{"".join(cells)}</tr>'


mm = lambda n: M[n]["metrics"]
SB = J("sweep_band.json")
over = lambda n, j: max(ARM[(n, j)]["over_hi"], ARM[(n, j)]["over_lo"])
T5 = ['<div class="tscroll"><table class="rmse wk">', HEAD5,
      row5("arm sweep, q2", "", lambda n: DESIGN[n], score=None),
      grp("Task and airframe, circle run (rms)", 8),
      row5("EE position ‖·‖", "mm", lambda n: mm(n)["ee_pos"]["rms_norm"]),
      row5("… from the airframe position", "mm", lambda n: B[n]["base"]),
      row5("… from the arm joints", "mm", lambda n: B[n]["joints"]),
      row5("EE heading", "deg", lambda n: mm(n)["ee_head"]["rms"], "{:.2f}"),
      row5("centre of mass ‖·‖", "mm", lambda n: mm(n)["com_pos"]["rms_norm"]),
      row5("airframe velocity ‖·‖", "mm/s", lambda n: mm(n)["base_vel"]["rms_norm"]),
      row5("roll vs plan", "deg", lambda n: mm(n)["att"]["rms"][0], "{:.2f}"),
      row5("pitch vs plan", "deg", lambda n: mm(n)["att"]["rms"][1], "{:.2f}"),
      row5("law's own ‖e_R‖", "", lambda n: mm(n)["law_eR_rms"], "{:.3f}"),
      row5("tilt, worst sample", "deg", lambda n: mm(n)["tilt_max_deg"], "{:.2f}"),
      grp("Hover before the circle (DIRECT-entry hold)", 8),
      row5("CoM standing offset", "mm", lambda n: HOLD[n]["offset_mm"]),
      row5("CoM wobble (std)", "mm", lambda n: HOLD[n]["wobble_mm"]),
      grp("Arm joints, circle run", 8),
      row5("q2 error rms", "deg", lambda n: ARM[(n, "j2")]["e_rms"], "{:.2f}"),
      row5("q3 error rms", "deg", lambda n: ARM[(n, "j3")]["e_rms"], "{:.2f}"),
      row5("q2 span realised", "%", lambda n: ARM[(n, "j2")]["span"], "{:.0f}", score=lambda v: abs(v - 100)),
      row5("q3 span realised", "%", lambda n: ARM[(n, "j3")]["span"], "{:.0f}", score=lambda v: abs(v - 100)),
      row5("q2 worst overshoot past the reference", "deg", lambda n: over(n, "j2"), "{:+.1f}", score=lambda v: max(v, 0)),
      row5("q3 worst overshoot past the reference", "deg", lambda n: over(n, "j3"), "{:+.1f}", score=lambda v: max(v, 0)),
      row5("q3 on the +50° stop", "% of run", lambda n: ARM[(n, "j3")]["at_stop"], "{:.1f}"),
      row5("q2 stuck while the reference moves", "%", lambda n: ARM[(n, "j2")]["stuck"], "{:.0f}"),
      row5("q3 stuck while the reference moves", "%", lambda n: ARM[(n, "j3")]["stuck"], "{:.0f}"),
      row5("q2 peak speed (reference 16 on F3/F4 and today)", "deg/s", lambda n: ARM[(n, "j2")]["pk_meas"], "{:.0f}"),
      row5("q3 peak speed", "deg/s", lambda n: ARM[(n, "j3")]["pk_meas"], "{:.0f}"),
      row5("q2 offset at the start hold", "deg", lambda n: ARM[(n, "j2")]["hold_mean"], "{:+.1f}", score=abs),
      row5("q3 offset at the start hold", "deg", lambda n: ARM[(n, "j3")]["hold_mean"], "{:+.1f}", score=abs),
      grp("Arm torque, circle run (0.1 s average)", 8),
      row5("applied − intended, j2 rms", "N·m", lambda n: ARM[(n, "j2")]["app_int_rms"], "{:.3f}"),
      row5("applied − intended, j3 rms", "N·m", lambda n: ARM[(n, "j3")]["app_int_rms"], "{:.3f}"),
      row5("peak applied j2, of the 2.44 N·m cap", "%", lambda n: ARM[(n, "j2")]["cap_pct"], "{:.0f}", score=None),
      row5("peak applied j3, of the 1.42 N·m cap", "%", lambda n: ARM[(n, "j3")]["cap_pct"], "{:.0f}", score=None),
      "</table></div>"]

a4, w1, w2 = mm("a4"), mm("w1"), mm("w2")
d1, d2 = mm("d1"), mm("d2")
AJ = lambda n, j, k: ARM[(n, j)][k]


def rng(a, b, f="{:.1f}"):
    x, y = sorted((f.format(a), f.format(b)), key=float)
    return x if x == y else f"{x}–{y}"

BODY = f"""
<h1 id="top">Whole-body vs decoupled on the end-effector circle</h1>
<p class="sub">T650 aerial manipulator, hardware, 2026-09-28 · four flights of the same planned circle: two with the whole-body 4-D L1 controller, two with the decoupled controller (geometric + L1 airframe law, position-mode arm) · compared with last week's whole-body flights (2026-09-24)</p>

<div class="verdict good"><strong>Overall.</strong> The whole-body controller tracked the end-effector circle about eight times more closely than the decoupled one: <strong>{w1["ee_pos"]["rms_norm"]:.0f} and {w2["ee_pos"]["rms_norm"]:.0f} mm rms</strong> against {d1["ee_pos"]["rms_norm"]:.0f} and {d2["ee_pos"]["rms_norm"]:.0f} mm, and <strong>{w1["ee_head"]["rms"]:.2f}° / {w2["ee_head"]["rms"]:.2f}°</strong> of EE heading error against {d1["ee_head"]["rms"]:.1f}°. Same planner, same plan, same day, same fused feedback. The decoupled error comes from two parts of its law, both measured: a body-fixed lateral force of about 0.6–0.7 N that its L1 cannot cancel parks the airframe {MECH["d1"]["hold_offset_mm"]:.0f}–{MECH["d2"]["hold_offset_mm"]:.0f} mm off, and the law flies no yaw-rate feed-forward, so the heading lags 11°. Its position-mode arm tracks its joints best of all four flights, which cannot help an airframe error. Against last week's flight with the same arm sweep (0924 F4), the new gains and arm compensation <strong>halved</strong> the whole-body EE error ({a4["ee_pos"]["rms_norm"]:.0f} → {w2["ee_pos"]["rms_norm"]:.0f}–{w1["ee_pos"]["rms_norm"]:.0f} mm) and cut the q2/q3 joint error from {AJ("a4","j2","e_rms"):.1f}/{AJ("a4","j3","e_rms"):.1f}° to {rng(AJ("w1","j2","e_rms"),AJ("w2","j2","e_rms"))}/{rng(AJ("w1","j3","e_rms"),AJ("w2","j3","e_rms"))}°. The stick-slip overshoot that drove q3 onto its stop last week is gone.</div>

<div class="kv">
<div><b>{(d1["ee_pos"]["rms_norm"]+d2["ee_pos"]["rms_norm"])/(w1["ee_pos"]["rms_norm"]+w2["ee_pos"]["rms_norm"]):.1f}×</b><span>decoupled ÷ whole-body, EE position error, today</span></div>
<div><b>{(d1["ee_head"]["rms"]+d2["ee_head"]["rms"])/(w1["ee_head"]["rms"]+w2["ee_head"]["rms"]):.0f}×</b><span>decoupled ÷ whole-body, EE heading error, today</span></div>
<div><b>−{100*(1-(w1["ee_pos"]["rms_norm"]+w2["ee_pos"]["rms_norm"])/2/a4["ee_pos"]["rms_norm"]):.0f} %</b><span>whole-body EE position error, today vs 0924 F4</span></div>
<div><b>−{100*(1-(AJ("w1","j3","e_rms")+AJ("w2","j3","e_rms"))/2/AJ("a4","j3","e_rms")):.0f} %</b><span>whole-body q3 tracking error, today vs 0924 F4</span></div>
</div>

<h2>1. Tracking performance: whole-body vs decoupled</h2>

<h3>1.1 What was flown</h3>
<p>All four flights ran the same sequence from the arm ground station's EE Trajectory tab: hover in SAFETY, switch to DIRECT, select the circle, Go To Start, Start Trajectory. The planner produced the same plan every time: an end-effector circle of radius 0.50 m about the origin, counter-clockwise, one lap of 24.17 s inside a 28.17 s run that includes 2 s of speed-up and slow-down. The gripper heading follows the tangent (14.6°/s) and the gripper stays level at the height it had when the circle was selected. The arm sweeps q2 = 25° ± 15° with a 6.04 s period while q3 = 55° − q2 (15–45°) and q1 = q4 = 0, so it folds and unfolds four times per lap. The gripper moves at 0.13 m/s along the circle.</p>
<p>Both rigs take that one plan from the same planner node. The whole-body law flies it directly. On the decoupled rig a bridge converts it into an airframe path (x<sub>b</sub> = x<sub>cd</sub> − R<sub>0</sub> r<sub>0c</sub>(q<sub>d</sub>), yaw from the planned heading) for the geometric + L1 law, and the joint half goes to the position-mode arm. The bridge checks on every sample that the converted airframe path puts the planned EE exactly on the plan; its logged residual stayed at or below 0.001 mm in both decoupled flights. What differs between flights is only where the circle starts: it is anchored at the gripper's bearing and height at the moment of selection.</p>
{"".join(TF)}
<p class="note">Every number below is scored over the circle run (the planner's EXECUTING T = 28.2 s window), trimmed 0.1 s at each end, on a 100 Hz grid.</p>

<h3>1.2 3D trajectories</h3>
<!--ee3d:start-->
{tpl("ee3d_section.html", "ee3d.json")}<!--ee3d:end-->

<h3>1.3 Full-state tracking error</h3>
<p>One definition for both rigs, computed only from signals both record, so neither law's own error bookkeeping is involved. Measured: the EKF2-fused odometry (airframe position, world velocity, attitude, body rates) and the joint encoders; the centre of mass, the gripper position and the gripper heading follow from the planner's own kinematic model. Reference: the planner's stream (the same message on both rigs). The airframe reference is the bridge's conversion of it (reproduced here to 0.4 mm), the reference attitude is the plan's compatible attitude (thrust along x<sub>cd</sub>'' + g, heading from the plan), and joint rates are a 0.1 s central difference of the encoder angle. The better controller's pair of cells is in bold.</p>
<p><strong>Table 1.</strong> RMS tracking error over the circle run.</p>
{"".join(T1)}
<p>The decoupled rig loses on every airframe-position, velocity and heading channel, by {rng(d1["base_pos"]["rms_norm"]/w1["base_pos"]["rms_norm"], d2["base_pos"]["rms_norm"]/w2["base_pos"]["rms_norm"], "{:.0f}")}× on airframe position and {rng(d1["att"]["rms"][2]/w1["att"]["rms"][2], d2["att"]["rms"][2]/w2["att"]["rms"][2], "{:.0f}")}× on yaw. Roll and pitch agree between the rigs within 0.4°, and the body rates within 1.6°/s. The one group the decoupled rig wins is the arm: its position servo holds q2 and q3 to {AJ("d1","j2","e_rms"):.1f} and {AJ("d1","j3","e_rms"):.1f}° rms and follows the reference rate closely, while the whole-body law, which moves the joints by torque, leaves {AJ("w2","j2","e_rms"):.1f}–{AJ("w1","j2","e_rms"):.1f}° and {AJ("w1","j3","e_rms"):.1f}°. The law's own attitude error reads 0.18 on the decoupled rig because it is almost all yaw: sin 10.4° = 0.18.</p>

<h3>1.4 Why the decoupled rig is off</h3>
<!--errts:start-->
{tpl("err_section.html", "err_ts.json")}<!--errts:end-->
<p>The EE error splits exactly into three sources: the airframe position error, the airframe attitude error carried out to the gripper, and the arm joint error (the three terms add to the measured error to within 0.002 mm on every flight). Their rms values do not add, because the terms are not orthogonal.</p>
<p><strong>Table 2.</strong> Where the EE error comes from, circle run.</p>
{"".join(T2)}
<h4>The airframe offset is a lateral force the decoupled law cannot cancel</h4>
<p>Both controllers see the same force: +0.46 to +0.63 N along the body x axis and −0.42 to −0.52 N along body y, fixed to the body. It is the same body-fixed force identified from the 0918–0924 flights, (+0.55, −0.50) N. The whole-body law estimates it with its translational observer and subtracts it from the commanded thrust vector, so it leaves only a few millimetres of mean offset. The decoupled law's L1 augmentation acts on the matched channel only (collective thrust and the three torques). A force in the body plane is unmatched, and the only thing left to oppose it is the pure-P/D position loop. Its K<sub>p</sub>e<sub>p</sub> + K<sub>v</sub>e<sub>v</sub> balances the L1's own unmatched estimate to within 0.05 N. With K<sub>p</sub> = 4 N/m the airframe settles 0.14–0.17 m from its reference, forward and to the right.</p>
<p><strong>Table 3.</strong> The lateral force and what each law does with it. Force and airframe error are circle-run means with 2 s trimmed from each end; the start-hold offset is the static hover just before the run.</p>
{"".join(T3)}
<p>Because the force is fixed to the body and the body turns with the circle, the offset turns too. In the circle's frame it stays constant, outward and ahead of the reference (Fig. 2): the decoupled EE flew a circle of radius {d1["ee_radius_meas_mean_m"]:.2f}–{d2["ee_radius_meas_mean_m"]:.2f} m instead of 0.50 m. The offset is already there before the circle starts. At the static start hold the decoupled airframe sat {MECH["d1"]["hold_offset_mm"]:.0f} and {MECH["d2"]["hold_offset_mm"]:.0f} mm from its reference, the whole-body airframe {MECH["w2"]["hold_offset_mm"]:.0f} and {MECH["w1"]["hold_offset_mm"]:.0f} mm.</p>
<h4>Both rigs swing with the arm sweep</h4>
<p>The part of the error that moves is paced by the arm. Its along-track component peaks at 0.16–0.19 Hz on all four flights, the 6 s fold-and-unfold period (0.166 Hz). As the arm folds, the centre of mass shifts and the airframe has to counter-move to keep it on the path, and neither law follows that motion exactly. The swing is {SB["w1"]["along"]["pp_mm"]:.0f}–{SB["w2"]["along"]["pp_mm"]:.0f} mm peak to peak along-track on the whole-body flights and {SB["d1"]["along"]["pp_mm"]:.0f}–{SB["d2"]["along"]["pp_mm"]:.0f} mm on the decoupled ones. The decoupled law models the vehicle as one rigid body, so the arm's horizontal reaction on the airframe is one more lateral force its L1 cannot cancel. On the decoupled rig it rides on top of the 90–125 mm mean offset.</p>
<h4>The heading lag is the missing yaw-rate feed-forward</h4>
<p>The decoupled law runs the paper's constant-yaw specialization: the desired body rate is zero (ω<sub>d</sub> = 0), so while the heading turns at a steady rate the attitude loop settles where k<sub>R</sub>e<sub>R</sub> balances k<sub>ω</sub>ω. The predicted lag is (k<sub>ω,z</sub> / k<sub>R,z</sub>)·ψ̇ = (0.35 / 0.49) × 14.6°/s = 10.4°; measured {MECH["d1"]["yaw_err_mean_deg"]:.1f}° on both flights. The position-mode arm holds q1 = q4 = 0, so the gripper inherits the whole airframe heading error, and with the gripper about 0.28 m from the airframe origin that is the 49–50 mm attitude term in Table 2. The whole-body law feeds the planned heading rate forward and keeps the airframe yaw within {abs(MECH["w1"]["yaw_err_mean_deg"]):.1f}° on average.</p>
<h4>Why the Isaac comparison did not show this</h4>
<p>The 2026-09-26 Isaac run of the same two rigs on this circle put them within 7 % of each other on EE position (77.5 vs 82.8 mm mean). That run flew the robustness plant, which carries no body-fixed lateral force. The heading lag did show there (4.7° vs 10.2°). The mirror plant, identified from the hardware flights, now carries the force.</p>

<h2>2. Gain tuning and the arm compensation: today vs last week</h2>

<h3>2.1 What changed since the 0924 flights</h3>
<p>Two sets of changes flew for the first time today, together: the circle gain tune (H1b, all law gain groups retuned on the bench) and the arm-channel corrections (the bench-calibrated joint-diagonal armature, the law closing on the arm's velocity observer, and a smaller friction feed-forward switched on the measured joint velocity). The right-hand column says how each is visible in today's data.</p>
<p><strong>Table 4.</strong> Configuration, 0924 vs today.</p>
{"".join(T4)}

<h3>2.2 Comparison with last week</h3>
<p>0924 F4 flew the same arm sweep as today and is the like-for-like comparison. F1 and F2 had a slower sweep (30° ± 10° over 48 s) and F3 a 12 s sweep; they are shown for context. For today's two flights a bold cell is better than F4, a grey one worse. F1 is scored only up to the moment its mocap feed froze (75.5 s).</p>
<p><strong>Table 5.</strong> Whole-body flights, 0924 vs today.</p>
{"".join(T5)}

<h3>2.3 Arm tracking curves</h3>
<!--armtrk:start-->
{tpl("arm_trk_section.html", "arm_trk.json")}<!--armtrk:end-->

<h3>2.4 Did they work?</h3>
<div class="verdict good">Yes. Every task and airframe number in Table 5 improved on 0924 F4, the arm reached its commanded fold without overshoot for the first time on this sweep, and the arm changes are visibly live in the data. Two flights each, on different days and batteries; the gains and the arm terms changed together, so the flight data cannot split the credit between them.</div>
<h4>The airframe: gain tune</h4>
<p>Centre-of-mass error fell from {a4["com_pos"]["rms_norm"]:.0f} to {w2["com_pos"]["rms_norm"]:.0f}–{w1["com_pos"]["rms_norm"]:.0f} mm rms, airframe velocity error from {a4["base_vel"]["rms_norm"]:.0f} to {w1["base_vel"]["rms_norm"]:.0f}–{w2["base_vel"]["rms_norm"]:.0f} mm/s, the law's attitude error from {a4["law_eR_rms"]:.3f} to {w2["law_eR_rms"]:.3f}–{w1["law_eR_rms"]:.3f}, and the worst tilt from {a4["tilt_max_deg"]:.1f}° to {w2["tilt_max_deg"]:.1f}–{w1["tilt_max_deg"]:.1f}°. In the hover before the circle the standing CoM offset dropped from {HOLD["a4"]["offset_mm"]:.0f} to {HOLD["w1"]["offset_mm"]:.0f}–{HOLD["w2"]["offset_mm"]:.0f} mm with a slightly smaller wobble. The bench check of this tune on the 6 s sweep predicted a 31 % smaller EE error (20.0 → 13.8 mm on the mirror plant); the flight gave {100*(1-(w1["ee_pos"]["rms_norm"]+w2["ee_pos"]["rms_norm"])/2/a4["ee_pos"]["rms_norm"]):.0f} %. The bench is trusted for ranking, not for absolute numbers, and the ranking held.</p>
<p>Lowering the position loop's damping was part of the tune: with the total mass 3.746 kg, k<sub>x</sub>/k<sub>v</sub> = 50.03/12.58 gives ω<sub>n</sub> 3.65 rad/s and ζ 0.46, against 2.92 rad/s and 0.91 before. No new lightly damped mode shows in the flights: the CoM error spectrum of the circle run has the same peaks as F4 (around 0.04, 0.12 and 0.21 Hz), just smaller, and the hover wobble is 9–10 mm against 10–15 mm last week.</p>
<h4>The arm: compensation terms and the stiffer task</h4>
<p>Last week the joints stick-slipped: F4's q2 and q3 swept {AJ("a4","j2","span"):.0f} % and {AJ("a4","j3","span"):.0f} % of the commanded span, overshot the reference extremes by up to {max(AJ("a4","j2","over_hi"),AJ("a4","j3","over_lo")):.0f}°, broke away at {AJ("a4","j2","pk_meas"):.0f}–{AJ("a4","j3","pk_meas"):.0f}°/s against a {AJ("a4","j2","pk_ref"):.0f}°/s reference, and q3 sat on its +50° stop for {AJ("a4","j3","at_stop"):.1f} % of the run. Today both joints swept {AJ("w1","j2","span"):.0f}–{AJ("w1","j3","span"):.0f} % of the span, overshot by at most {max(AJ(n,j,k) for n in ("w1","w2") for j in ("j2","j3") for k in ("over_hi","over_lo")):.1f}°, peaked at {AJ("w1","j2","pk_meas"):.0f}–{AJ("w1","j3","pk_meas"):.0f}°/s and never reached the stop. The q2/q3 error fell from {AJ("a4","j2","e_rms"):.1f}/{AJ("a4","j3","e_rms"):.1f}° to {rng(AJ("w1","j2","e_rms"),AJ("w2","j2","e_rms"))}/{rng(AJ("w1","j3","e_rms"),AJ("w2","j3","e_rms"))}° rms, with the joints within 0.07 s of the reference. The arm's share of the EE error fell from {B["a4"]["joints"]:.1f} to {B["w2"]["joints"]:.1f}–{B["w1"]["joints"]:.1f} mm.</p>
<p>The friction change removed what drove the stick-slip. On F4 the feed-forward switched on the reference velocity, so it kept pushing a stuck joint with {FF["a4"]["j2"]["ff_when_stuck"]:.2f} N·m (j2) and {FF["a4"]["j3"]["ff_when_stuck"]:.2f} N·m (j3) until it broke away and overshot. Today it switches on the measured velocity and averages {FF["w2"]["j2"]["ff_when_stuck"]:.3f} N·m or less while a joint is stuck. The stiffer task (K<sub>y</sub> 20 → 212 N/m), closed on the observer's 12 ms velocity instead of the 50 ms Present Velocity, then pulls the joint onto the reference. The arm study's offline model predicted 0.3/0.4° at K<sub>y</sub> 200 on the observer velocity; the flight's q2/q3 error is 4–5 times that, so the model is still optimistic about the joints, as the 0924 simulation check also found.</p>
<p>Torque delivery was already faithful and stays so: applied minus intended torque is {AJ("w1","j2","app_int_rms"):.3f} / {AJ("w1","j3","app_int_rms"):.3f} N·m rms on j2 / j3, against {AJ("a4","j2","app_int_rms"):.3f} / {AJ("a4","j3","app_int_rms"):.3f} on F4. Peak applied torque used {AJ("w1","j2","cap_pct"):.0f} % and {AJ("w1","j3","cap_pct"):.0f} % of the servo caps.</p>
<h4>What is still left</h4>
<ul>
<li><strong>Residual stick-slip on q3.</strong> q3 is still stuck for {AJ("w2","j3","stuck"):.0f}–{AJ("w1","j3","stuck"):.0f} % of the time its reference moves (q2 {AJ("w2","j2","stuck"):.0f}–{AJ("w1","j2","stuck"):.0f} %), and the joints still peak at {min(AJ(n,j,"pk_meas")/AJ(n,j,"pk_ref") for n in ("w1","w2") for j in ("j2","j3")):.1f}–{max(AJ(n,j,"pk_meas")/AJ(n,j,"pk_ref") for n in ("w1","w2") for j in ("j2","j3")):.1f} times the reference speed around the sweep reversals (Fig. 3).</li>
<li><strong>A standing joint offset at the start hold.</strong> q2 parks {AJ("w2","j2","hold_mean"):.1f} to {AJ("w1","j2","hold_mean"):.1f}° and q3 +{AJ("w2","j3","hold_mean"):.1f} to +{AJ("w1","j3","hold_mean"):.1f}° from the reference, with a ripple of 0.04° or less, i.e. held by friction. The two offsets have opposite signs, which is the direction that barely moves the gripper (3–5 mm by the model), so the EE task does not see it.</li>
<li><strong>The airframe error is the new floor, and the arm sweep paces it.</strong> It is {B["w2"]["base"]:.0f}–{B["w1"]["base"]:.0f} of the {w2["ee_pos"]["rms_norm"]:.0f}–{w1["ee_pos"]["rms_norm"]:.0f} mm EE error. {SB["w2"]["along"]["share_pct"]:.0f}–{SB["w1"]["along"]["share_pct"]:.0f} % of its along-track variance sits at the 6 s sweep frequency, {SB["w2"]["along"]["pp_mm"]:.0f}–{SB["w1"]["along"]["pp_mm"]:.0f} mm peak to peak against {SB["a4"]["along"]["pp_mm"]:.0f} mm on F4: smaller, but the same mechanism, the airframe not quite following the counter-motion the arm's fold asks of it. The slower radial part is the body-fixed force turning with the heading (the observer still reads {MECH["w2"]["force_body_N"][0]:+.2f} to {MECH["w1"]["force_body_N"][0]:+.2f}, {MECH["w1"]["force_body_N"][1]:+.2f} N), which the circle study traced to the law's thrust-rate term treating that estimate as constant; its fix is a disturbance-rate term, a law change rather than a gain.</li>
</ul>

<h2>3. Data notes</h2>
<ul>
<li>Scoring checked against the whole-body law's own debug array: encoder joints match its q to 0.05°, the model CoM matches its x<sub>c</sub> to 0.6 mm, and the EE error matches its task + CoM error to 0.3 mm. The model EE matches the planner's published current EE to under 1 mm. On the decoupled rig the airframe reference matches the bridge's output to 0.4 mm and the airframe error matches the geometric law's e<sub>p</sub> (correlation 0.99999). Last week's numbers reproduce the 0924 report's table.</li>
<li>Spectral shares use a Hann periodogram of the circle run minus 3 s at each end (<code>sweep_band.py</code>, <code>com_spectrum.py</code>).</li>
<li>The recorder could not subscribe to the planner's joint reference for the position-mode arm on DEC-1 (a QoS durability mismatch; only arm_planner's SAFETY-mode messages were recorded). The joint reference used for both decoupled flights is q<sub>d</sub> from the planner's whole-body stream, the same message the bridge converts.</li>
<li>Feedback gaps: WB-2 has two fused-odometry gaps of 78 and 90 ms inside the circle run. The mocap stream dropped for about 0.1 s once on WB-1 (after the run) and once on DEC-1 (before DIRECT). None of them shows in the errors.</li>
<li>joint_states is published in the order [j2, j3, j1, j4]; the hardware to model sign is [−1, 1, 1, −1]. Log lines received when the recorder started carry the recorder's receive time, not the time they were emitted.</li>
<li>Tools: <code>docs/docs_aerial_manipulator/wb_vs_decoupled_flight_20260928/tools/</code> (README.md lists the run order); raw outputs in <code>analysis/</code>.</li>
</ul>
"""

style = open(os.path.join(HERE, "report_style.css"), encoding="utf-8").read()
doc = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>0928 Circle Comparison</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
{style}
</head>
<body>
{BODY}
</body>
</html>
"""
open(os.path.join(HERE, "..", "report.html"), "w", encoding="utf-8").write(doc)
print("wrote report.html", len(doc))
