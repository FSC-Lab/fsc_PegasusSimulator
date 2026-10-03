#!/usr/bin/env python3
"""Compose ../report.html = the 0928 report (sections 1-2, unchanged, built by ../../wb_vs_decoupled_flight_20260928/tools/
build_report.py) + the 2026-10-02 decoupled section (new section 3; data notes become section 4). Every number comes from
../analysis/*.json (this campaign) and the 0928 analysis JSON. Then `python3 build_artifact_page.py <out.html>`.

    python3 make_templates.py && python3 build_report.py
"""
import json, math, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__)); AN = os.path.join(HERE, "..", "analysis")
OLD = os.path.join(HERE, "..", "..", "wb_vs_decoupled_flight_20260928")
J = lambda p: json.load(open(p))
N = J(f"{AN}/metrics.json"); NB = J(f"{AN}/ee_budget.json"); NM = J(f"{AN}/mechanism.json")
O = J(f"{OLD}/analysis/metrics.json"); OB = J(f"{OLD}/analysis/ee_budget.json"); OM = J(f"{OLD}/analysis/mechanism.json")
MM = {**{k: O[k]["metrics"] for k in ("d1", "d2", "w1", "w2")}, **{k: N[k]["metrics"] for k in ("r1", "r2", "tp")}}
BUD = {**{k: OB[k] for k in ("d1", "d2")}, **{k: NB[k] for k in ("r1", "r2", "tp")}}
KP_OLD, KP_NEW = 4.0, 20.11
LAG_OLD = 0.35 / 0.49 * 14.6; LAG_NEW = NM["r1"]["yaw_lag_pred_deg"]


def td(v, cls=""):
    return f'<td class="{"n" + (" " + cls if cls else "")}">{v}</td>'


def grp(text, n):
    return f'<tr class="grp"><td colspan="{n}">{text}</td></tr>'


def tpl(name, data):
    t = open(os.path.join(HERE, name), encoding="utf-8").read(); assert t.count("__DATA__") == 1
    return t.replace("__DATA__", open(os.path.join(AN, data), encoding="utf-8").read().strip())


# ---------------- Table 6: parameters -------------------------------------------------------------------------
fo = [math.hypot(*OM[k]["force_body_N"]) for k in ("d1", "d2")]; fn = [math.hypot(*NM[k]["force_body_N"]) for k in ("r1", "r2")]
T6 = ['<div class="tscroll"><table>', "<tr><th>parameter</th><th>09-28 (old)</th><th>10-02 (new)</th><th>what it predicts on this circle</th></tr>",
      grp("Geometric SE(3) baseline law (forces in N, torques in N·m)", 4),
      f'<tr><td>position gain K<sub>p</sub> x, y / z [N/m]</td>{td("4.0 / 8.0")}{td("20.11 / 13.5")}<td>airframe offset |F|/K<sub>p</sub> from the unmatched body-fixed force (|F| ≈ {min(fo + fn):.2f}–{max(fo + fn):.2f} N): {min(fo) / KP_OLD * 1e3:.0f}–{max(fo) / KP_OLD * 1e3:.0f} mm → {min(fn) / KP_NEW * 1e3:.0f}–{max(fn) / KP_NEW * 1e3:.0f} mm</td></tr>',
      f'<tr><td>velocity gain K<sub>v</sub> x, y / z [N·s/m]</td>{td("6.0 / 10.0")}{td("11.05 / 10.82")}<td></td></tr>',
      f'<tr><td>attitude gain k<sub>R</sub> x, y / z</td>{td("1.0 / 0.49")}{td("3.337 / 1.737")}<td rowspan="2">heading lag (k<sub>ω,z</sub>/k<sub>R,z</sub>)·ψ̇ at ψ̇ = 14.7°/s: {LAG_OLD:.1f}° → {LAG_NEW:.1f}°</td></tr>',
      f'<tr><td>rate gain k<sub>ω</sub> x, y / z</td>{td("0.55 / 0.35")}{td("0.9505 / 0.4105")}</tr>',
      grp("L1 augmentation (matched channel: collective thrust and the three torques)", 4),
      f'<tr><td>predictor poles A<sub>s</sub>, velocity / body rate [1/s]</td>{td("−2.0 / −2.0")}{td("−4.334 / −2.791")}<td></td></tr>',
      f'<tr><td>low-pass bandwidth ω<sub>c</sub> [rad/s]</td>{td("6.0")}{td("1.0")}<td>the change that bought the tune its delay margin on the bench (44 → 52 ms)</td></tr>',
      grp("Watchdog (reverts to SAFETY)", 4),
      f'<tr><td>tilt limit / drift limit</td>{td("40° / off")}{td("20° / 0.75 m")}<td>never tripped on 10-02</td></tr>',
      "</table></div>"]

# ---------------- Table 7: flights ------------------------------------------------------------------------------
fl = [("Run 1", "r1", "13:42", "circle, first run", "continued: hold, go to start, run 2"),
      ("Run 2", "r2", "13:42", "circle, second run (same flight)", "operator SAFETY at 96.9 s, 0.8 s before the end of the slow-down; arm folds home"),
      ("PS4", "tp", "13:48", "PS4 teleoperation", "operator SAFETY at 96.9 s; arm folds home")]
T7 = ['<div class="tscroll"><table>', "<tr><th>name</th><th>flight</th><th>what</th><th>DIRECT [s since bag start]</th><th>scored [s]</th><th>pack voltage in it</th><th>how it ended</th></tr>"]
for lab, k, clock, what, end in fl:
    o = N[k]
    T7.append(f'<tr><td class="nb">{lab}</td><td class="nb">10-02 {clock}</td><td>{what}</td>{td("{:.1f} – {:.1f}".format(*o["direct"]))}'
              f'{td("{:.1f} – {:.1f}".format(*o["window"]))}{td("{:.2f} V".format(o["metrics"]["pack_v"]))}<td>{end}</td></tr>')
T7.append("</table></div>")

# ---------------- Table 8: RMSE, old vs new gains ---------------------------------------------------------------
COLS = ["d1", "d2", "r1", "r2"]


def row8(label, unit, get, fmt="{:.1f}", score=lambda v: v, ratio=True):
    v = [get(MM[k]) for k in COLS]; wb = (get(MM["w1"]) + get(MM["w2"])) / 2
    old, new = (v[0] + v[1]) / 2, (v[2] + v[3]) / 2
    co, cn = ("", "")
    if score is not None and abs(score(old) - score(new)) > 1e-9 * max(1, abs(score(old))):
        co, cn = ("lose", "win") if score(new) < score(old) else ("win", "lose")
    cells = [td(fmt.format(x), co if i < 2 else cn) for i, x in enumerate(v)]
    rat = f"{new / old:.2f}" if (ratio and old > 1e-9) else "—"
    return f'<tr><td>{label}</td><td class="unit">{unit}</td>{"".join(cells)}{td(rat)}{td(fmt.format(wb))}</tr>'


T8 = ['<div class="tscroll"><table class="rmse">',
      '<tr><th>state</th><th>unit</th><th class="rig-dec">DEC-1 09-28</th><th class="rig-dec">DEC-2 09-28</th><th class="rig-new">Run 1 10-02</th>'
      '<th class="rig-new">Run 2 10-02</th><th>new ÷ old</th><th class="rig-wb">whole-body 09-28, mean</th></tr>']
NC = 8
T8.append(grp("Airframe position (body origin) vs the plan's airframe path", NC))
for k, ax in enumerate("xyz"):
    T8.append(row8(ax, "mm", lambda m, k=k: m["base_pos"]["rms_xyz"][k]))
T8.append(row8("‖·‖", "mm", lambda m: m["base_pos"]["rms_norm"]))
T8.append(grp("System centre of mass vs the planned CoM", NC)); T8.append(row8("‖·‖", "mm", lambda m: m["com_pos"]["rms_norm"]))
T8.append(grp("Airframe velocity", NC))
for k, ax in enumerate("xyz"):
    T8.append(row8(ax, "mm/s", lambda m, k=k: m["base_vel"]["rms_xyz"][k]))
T8.append(row8("‖·‖", "mm/s", lambda m: m["base_vel"]["rms_norm"]))
T8.append(grp("Attitude vs the plan's attitude (body axes)", NC))
for k, ax in enumerate(["roll", "pitch", "yaw"]):
    T8.append(row8(ax, "deg", lambda m, k=k: m["att"]["rms"][k], "{:.2f}"))
T8.append(grp("Body rate vs the plan's rate", NC))
for k, ax in enumerate(["roll", "pitch", "yaw"]):
    T8.append(row8(ax, "deg/s", lambda m, k=k: m["rate"]["rms"][k]))
T8.append(grp("Arm joint angle vs q_d (position-mode servo, unchanged)", NC))
for k in range(4):
    T8.append(row8(f"q{k+1}", "deg", lambda m, k=k: m["joint"]["rms"][k], "{:.2f}"))
T8.append(grp("Arm joint rate vs q̇_d", NC))
for k in range(4):
    T8.append(row8(f"q̇{k+1}", "deg/s", lambda m, k=k: m["joint_vel"]["rms"][k]))
T8.append(grp("End-effector (the task)", NC))
for k, ax in enumerate("xyz"):
    T8.append(row8(f"position {ax}", "mm", lambda m, k=k: m["ee_pos"]["rms_xyz"][k]))
T8.append(row8("position ‖·‖", "mm", lambda m: m["ee_pos"]["rms_norm"]))
T8.append(row8("heading", "deg", lambda m: m["ee_head"]["rms"], "{:.2f}"))
T8.append(grp("Other", NC))
T8.append(row8("EE position, worst sample", "mm", lambda m: m["ee_pos"]["max_norm"], "{:.0f}"))
T8.append(row8("EE circle radius flown (plan 0.500)", "m", lambda m: m["ee_radius_meas_mean_m"], "{:.3f}", score=lambda v: abs(v - 0.5), ratio=False))
T8.append(row8("tilt, worst sample", "deg", lambda m: m["tilt_max_deg"], "{:.2f}"))
T8.append(row8("law's own attitude error ‖e_R‖", "—", lambda m: m["law_eR_rms"], "{:.3f}"))
T8.append(row8("motor-saturated ticks", "count", lambda m: m["sat"], "{:.0f}", score=None, ratio=False))
T8.append("</table></div>")

# ---------------- Table 9: EE budget; Table 10: the two structural errors vs prediction ---------------------------
LBL = {"d1": "DEC-1 09-28", "d2": "DEC-2 09-28", "r1": "Run 1 10-02", "r2": "Run 2 10-02", "tp": "PS4 10-02"}
T9 = ['<div class="tscroll"><table class="rmse">', "<tr><th>run</th><th>gains</th><th>EE position error ‖·‖ rms [mm]</th><th>from the airframe position [mm]</th>"
      "<th>from the airframe attitude [mm]</th><th>from the arm joints [mm]</th></tr>"]
for k in COLS:
    b = BUD[k]
    T9.append(f'<tr><td class="nb">{LBL[k]}</td><td>{"old" if k[0] == "d" else "new"}</td>' + "".join(td(f"{b[c]:.1f}") for c in ("total", "base", "attitude", "joints")) + "</tr>")
T9.append("</table></div>")
MECH = {**{k: OM[k] for k in ("d1", "d2")}, **{k: NM[k] for k in ("r1", "r2")}}
T10 = ['<div class="tscroll"><table class="rmse">', "<tr><th>run</th><th>lateral force estimate |F| [N]</th><th>predicted offset |F|/K<sub>p</sub> [mm]</th>"
       "<th>measured airframe offset, body-frame mean [mm]</th><th>predicted heading lag [deg]</th><th>measured heading error, mean [deg]</th></tr>"]
for k in COLS:
    r = MECH[k]; F = math.hypot(*r["force_body_N"]); kp = KP_OLD if k[0] == "d" else KP_NEW
    T10.append(f'<tr><td class="nb">{LBL[k]}</td>{td(f"{F:.2f}")}{td(f"{F / kp * 1e3:.0f}")}{td("{:.0f}".format(math.hypot(*r["airframe_err_body_mm"])))}'
               f'{td(f"{(LAG_OLD if k[0] == chr(100) else LAG_NEW):.1f}")}{td("{:.1f}".format(-r["yaw_err_mean_deg"]))}</tr>')
T10.append("</table></div>")

# ---------------- Table 11: teleoperation -----------------------------------------------------------------------
tp = MM["tp"]; cm = lambda f: (f(MM["r1"]) + f(MM["r2"])) / 2
mean = tp["ee_pos"]["mean_xyz"]; off = math.hypot(mean[0], mean[1]); resid = math.sqrt(max(tp["ee_pos"]["rms_norm"] ** 2 - sum(x * x for x in mean), 0))


def row11(label, unit, get, fmt="{:.1f}"):
    return f'<tr><td>{label}</td><td class="unit">{unit}</td>{td(fmt.format(get(tp)))}{td(fmt.format(cm(get)))}</tr>'


T11 = ['<div class="tscroll"><table class="rmse">', '<tr><th>state</th><th>unit</th><th class="rig-new">PS4 teleoperation 10-02</th><th class="rig-new">circle runs 10-02, mean (Table 8)</th></tr>',
       grp("Airframe and centre of mass (rms)", 4)]
for k, ax in enumerate("xyz"):
    T11.append(row11(f"airframe position {ax}", "mm", lambda m, k=k: m["base_pos"]["rms_xyz"][k]))
T11 += [row11("airframe position ‖·‖", "mm", lambda m: m["base_pos"]["rms_norm"]), row11("centre of mass ‖·‖", "mm", lambda m: m["com_pos"]["rms_norm"]),
        row11("airframe velocity ‖·‖", "mm/s", lambda m: m["base_vel"]["rms_norm"])]
for k, ax in enumerate(["roll", "pitch", "yaw"]):
    T11.append(row11(ax, "deg", lambda m, k=k: m["att"]["rms"][k], "{:.2f}"))
T11.append(grp("Arm and end-effector (rms)", 4))
T11 += [row11("q2", "deg", lambda m: m["joint"]["rms"][1], "{:.2f}"), row11("q3", "deg", lambda m: m["joint"]["rms"][2], "{:.2f}")]
for k, ax in enumerate("xyz"):
    T11.append(row11(f"EE position {ax}", "mm", lambda m, k=k: m["ee_pos"]["rms_xyz"][k]))
T11 += [row11("EE position ‖·‖", "mm", lambda m: m["ee_pos"]["rms_norm"]), row11("EE heading", "deg", lambda m: m["ee_head"]["rms"], "{:.2f}"),
        grp("Other", 4), row11("EE position, worst sample", "mm", lambda m: m["ee_pos"]["max_norm"], "{:.0f}"),
        row11("tilt, worst sample", "deg", lambda m: m["tilt_max_deg"], "{:.2f}"), row11("law's own attitude error ‖e_R‖", "—", lambda m: m["law_eR_rms"], "{:.3f}"),
        row11("motor-saturated ticks", "count", lambda m: m["sat"], "{:.0f}"), "</table></div>"]

# ---------------- prose numbers ---------------------------------------------------------------------------------
r1, r2, d1, d2 = MM["r1"], MM["r2"], MM["d1"], MM["d2"]
ee_o = (d1["ee_pos"]["rms_norm"] + d2["ee_pos"]["rms_norm"]) / 2; ee_n = (r1["ee_pos"]["rms_norm"] + r2["ee_pos"]["rms_norm"]) / 2
hd_o = (d1["ee_head"]["rms"] + d2["ee_head"]["rms"]) / 2; hd_n = (r1["ee_head"]["rms"] + r2["ee_head"]["rms"]) / 2
ee_wb = (MM["w1"]["ee_pos"]["rms_norm"] + MM["w2"]["ee_pos"]["rms_norm"]) / 2; hd_wb = (MM["w1"]["ee_head"]["rms"] + MM["w2"]["ee_head"]["rms"]) / 2
offo = [math.hypot(*MECH[k]["airframe_err_body_mm"]) for k in ("d1", "d2")]; offn = [math.hypot(*MECH[k]["airframe_err_body_mm"]) for k in ("r1", "r2")]
lagn = [-MECH[k]["yaw_err_mean_deg"] for k in ("r1", "r2")]
f1 = lambda a, b, f="{:.0f}": f"{f.format(a)} and {f.format(b)}"

SEC = f"""
<h2>3. The decoupled controller after its gain tune (2026-10-02)</h2>
<p>On 2026-10-02 the decoupled controller flew again with the gain set tuned on 2026-10-01. That set was found on the bench against the flight-identified mirror plant and then flown three times in Isaac at real time, where it cut the circle's EE error from 206 to 49 mm. On hardware it flew one flight with two runs of the same circle as in section 1, and one PS4 teleoperation flight. The planner, the plan, the reference bridge, the position-mode arm and the EKF2-fused feedback were the same as on 09-28. Only the airframe law's gains and its watchdog changed.</p>

<h3>3.1 What changed and what was flown</h3>
<p><strong>Table 6.</strong> Decoupled-law parameters, 09-28 vs 10-02 (<code>params_single_aerial_manipulator_geometric_l1_direct_actuation_t650.yaml</code>, fsc_autopilot_ros2 <code>afbeb35</code>). The right-hand column is what each change predicts for the two structural errors of section 1.4.</p>
{"".join(T6)}
<p><strong>Table 7.</strong> The 10-02 flights.</p>
{"".join(T7)}
<p class="note">Both runs fly the identical plan of section 1: the planner's trajectory summary matches 09-28 to every digit (r 0.50 m, counter-clockwise, 24.17 s lap, q2 25° ± 15° at a 6.04 s period). Run 2 ended 0.8 s before the end of its slow-down, so both 10-02 runs are scored on their first 27.2 s, trimmed 0.1 s at each end. Scoring the 09-28 runs the same way changes their numbers by under 0.5 %, so they are quoted from Table 1.</p>

<h3>3.2 3D trajectories</h3>
<!--ee3dd:start-->
{tpl("ee3d_dec_section.html", "ee3d_dec.json")}<!--ee3dd:end-->

<h3>3.3 Full-state tracking error</h3>
<p>Same definitions as Table 1: odometry and joint encoders through the planner's model, against the planner's stream. The ratio column is the mean of the two new runs over the mean of the two old ones, so below 1 means the new gains track better; the bold pair is the better gain set. The last column is the whole-body controller on 09-28, for reference.</p>
<p><strong>Table 8.</strong> RMS tracking error over the circle run, decoupled controller on the old and the new gains.</p>
{"".join(T8)}
<p>The tune cut the EE position error by {100 * (1 - ee_n / ee_o):.0f} %, from {f1(d1["ee_pos"]["rms_norm"], d2["ee_pos"]["rms_norm"])} mm to {f1(r1["ee_pos"]["rms_norm"], r2["ee_pos"]["rms_norm"])} mm rms, and the EE heading error by {100 * (1 - hd_n / hd_o):.0f} %, from {hd_o:.1f}° to {f1(r1["ee_head"]["rms"], r2["ee_head"]["rms"], "{:.1f}")}°. The airframe position error fell by the same factor ({f1(d1["base_pos"]["rms_norm"], d2["base_pos"]["rms_norm"])} → {f1(r1["base_pos"]["rms_norm"], r2["base_pos"]["rms_norm"])} mm), and the gripper now flies a circle of radius {r1["ee_radius_meas_mean_m"]:.2f} m instead of {d1["ee_radius_meas_mean_m"]:.2f}–{d2["ee_radius_meas_mean_m"]:.2f} m. The arm channel did not change: the position-mode servo still holds q2 and q3 to {r1["joint"]["rms"][1]:.1f}° and {r1["joint"]["rms"][2]:.1f}° rms. The vertical and attitude channels got worse, most of it in run 2: vertical EE error {f1(r1["ee_pos"]["rms_xyz"][2], r2["ee_pos"]["rms_xyz"][2])} mm against {f1(d1["ee_pos"]["rms_xyz"][2], d2["ee_pos"]["rms_xyz"][2])} mm, roll and pitch {min(r1["att"]["rms"][0], r1["att"]["rms"][1]):.1f}–{max(r2["att"]["rms"][0], r2["att"]["rms"][1]):.1f}° against {min(d1["att"]["rms"][:2]):.1f}–{max(d2["att"]["rms"][:2]):.1f}°, and the worst tilt {f1(r1["tilt_max_deg"], r2["tilt_max_deg"], "{:.1f}")}° against {max(d1["tilt_max_deg"], d2["tilt_max_deg"]):.1f}°. The whole-body controller of 09-28 is still about {ee_n / ee_wb:.0f} times closer on EE position ({ee_wb:.0f} mm) and {hd_n / hd_wb:.0f} times closer on heading ({hd_wb:.2f}°).</p>

<h3>3.4 Where the remaining error comes from</h3>
<!--errtsd:start-->
{tpl("err_dec_section.html", "err_ts_dec.json")}<!--errtsd:end-->
<p><strong>Table 9.</strong> The EE error split into its three sources (the section 1.4 definition; the terms add exactly, their rms values do not).</p>
{"".join(T9)}
<p>The airframe position is still most of the EE error ({NB["r1"]["base"]:.0f} of {NB["r1"]["total"]:.0f} mm and {NB["r2"]["base"]:.0f} of {NB["r2"]["total"]:.0f} mm). The attitude term fell from about {OB["d1"]["attitude"]:.0f} to {f1(NB["r1"]["attitude"], NB["r2"]["attitude"])} mm with the smaller heading lag, and the arm joints contribute {NB["r1"]["joints"]:.0f} mm, as before.</p>
<p><strong>Table 10.</strong> The two structural errors of section 1.4 against what the gains predict. Force and offsets are run means with 2 s trimmed from each end.</p>
{"".join(T10)}
<p>Both structural errors shrank by the factor the gains predict. The body-fixed lateral force is unchanged at about {min(fn):.2f}–{max(fn):.2f} N. With K<sub>p</sub> raised from 4 to 20.11 N/m it should hold the airframe |F|/K<sub>p</sub> ≈ {min(fn) / KP_NEW * 1e3:.0f}–{max(fn) / KP_NEW * 1e3:.0f} mm from its reference, and it sat {min(offn):.0f}–{max(offn):.0f} mm off (it was {min(offo):.0f}–{max(offo):.0f} mm on 09-28). The new yaw pair predicts a heading lag of {LAG_NEW:.1f}° on this circle against {LAG_OLD:.1f}° before; the measured mean was {min(lagn):.1f}–{max(lagn):.1f}°. Both errors remain structural: the L1 cannot cancel an unmatched force and the law still flies ω<sub>d</sub> = 0, so higher gains shrink them but do not remove them.</p>

<h3>3.5 PS4 teleoperation flight</h3>
<p>The second 10-02 flight was flown from the PS4 pad through the planner's TELEOP state for {N["tp"]["window"][1] - N["tp"]["window"][0]:.1f} s. The pad moved the reference for 26 % of that time: the airframe through a 14 × 19 cm patch of the horizontal plane at up to 7 cm/s, and the gripper 7 cm down through a 10° fold of q2 and q3. Altitude and heading were not commanded. The scoring is the same as on the circle, against the reference the planner streamed from the pad.</p>
<!--tpts:start-->
{tpl("teleop_section.html", "teleop_ts.json")}<!--tpts:end-->
<p><strong>Table 11.</strong> RMS tracking error over the teleoperation, next to the circle runs on the same gains.</p>
{"".join(T11)}
<p>The EE error is {tp["ee_pos"]["rms_norm"]:.0f} mm rms, and most of it is the same standing offset as on the circle: {off:.0f} mm on average ({mean[0]:+.0f}, {mean[1]:+.0f} mm in world x and y), which leaves {resid:.0f} mm rms around it. The heading held to {tp["ee_head"]["rms"]:.2f}° rms because the pad did not turn the vehicle, and the worst tilt was {tp["tilt_max_deg"]:.1f}°.</p>
"""

NOTES = ("<li>10-02: the position-mode arm's reference was recorded this time; it equals the planner's q<sub>d</sub> to 0.08° "
         "(receive-time alignment), so the joint errors of the 10-02 runs use q<sub>d</sub>, as on 09-28.</li>\n"
         "<li>10-02 tools: <code>docs/docs_aerial_manipulator/decoupled_flight_20261002/tools/</code> (README.md lists the run order); "
         "raw outputs in <code>analysis/</code>. Sections 1 and 2 are the 09-28 report, unchanged.</li>\n")

src = open(os.path.join(OLD, "report.html"), encoding="utf-8").read()


def sub(t, a, b, regex=False):
    if regex:
        t2, n = re.subn(a, lambda _m: b, t, count=1, flags=re.S)
    else:
        n = t.count(a); t2 = t.replace(a, b, 1)
    assert n >= 1, a[:80]
    return t2


src = sub(src, "<title>0928 Circle Comparison</title>", "<title>Experiment: Free-flight Comparison 0928 + 1002</title>")
src = sub(src, "--panel:#141413}", "--panel:#141413;--violet:#9085e9}")
src = sub(src, "table.rmse th.rig-old{", "table.rmse th.rig-new{border-bottom:2px solid var(--violet)}table.rmse th.rig-old{")
src = sub(src, r'<p class="sub">.*?</p>', '<p class="sub">T650 aerial manipulator, hardware · <strong>2026-09-28</strong>: four flights of the same planned circle, two with the whole-body 4-D L1 controller and two with the decoupled controller (geometric + L1 airframe law, position-mode arm), compared with the 2026-09-24 whole-body flights (sections 1 and 2, where "today" means 09-28) · <strong>2026-10-02</strong>: the decoupled controller on its tuned gains, two runs of the same circle and a PS4 teleoperation flight (section 3)</p>', regex=True)
for a, b in (("EE position error, today</span>", "EE position error, 09-28</span>"), ("EE heading error, today</span>", "EE heading error, 09-28</span>"),
             ("whole-body EE position error, today vs 0924 F4", "whole-body EE position error, 09-28 vs 0924 F4"), ("whole-body q3 tracking error, today vs 0924 F4", "whole-body q3 tracking error, 09-28 vs 0924 F4")):
    src = sub(src, a, b)
UPD = (f'<div class="verdict good"><strong>Update, 2026-10-02.</strong> With the gains tuned on 2026-10-01, the decoupled controller flew the same circle twice at '
       f'<strong>{f1(r1["ee_pos"]["rms_norm"], r2["ee_pos"]["rms_norm"])} mm rms</strong> EE error ({f1(d1["ee_pos"]["rms_norm"], d2["ee_pos"]["rms_norm"])} mm on 09-28) and '
       f'<strong>{r1["ee_head"]["rms"]:.1f}° / {r2["ee_head"]["rms"]:.1f}°</strong> heading error ({hd_o:.1f}°). Both structural errors of its law shrank by the factor the new gains predict: '
       f'the airframe offset from the unmatched lateral force fell from {min(offo):.0f}–{max(offo):.0f} to {min(offn):.0f}–{max(offn):.0f} mm, and the heading lag from about 11° to about 4°. '
       f'The whole-body controller is still about {ee_n / ee_wb:.0f} times closer on EE position ({ee_wb:.0f} mm) and {hd_n / hd_wb:.0f} times closer on heading. Roll, pitch and the worst tilt grew on the new gains, most in run 2 '
       f'(worst tilt {f1(r1["tilt_max_deg"], r2["tilt_max_deg"], "{:.1f}")}° against {max(d1["tilt_max_deg"], d2["tilt_max_deg"]):.1f}°). '
       f'A PS4 teleoperation flight on the new gains held the gripper to {tp["ee_pos"]["rms_norm"]:.0f} mm rms, most of it the same standing offset (section 3).</div>\n\n'
       f'<div class="kv">\n<div><b>−{100 * (1 - ee_n / ee_o):.0f} %</b><span>decoupled EE position error, 10-02 vs 09-28</span></div>\n'
       f'<div><b>−{100 * (1 - hd_n / hd_o):.0f} %</b><span>decoupled EE heading error, 10-02 vs 09-28</span></div>\n'
       f'<div><b>{ee_n / ee_wb:.1f}×</b><span>decoupled (10-02) ÷ whole-body (09-28), EE position error</span></div>\n'
       f'<div><b>{tp["ee_pos"]["rms_norm"]:.0f} mm</b><span>decoupled EE error under PS4 teleoperation, rms</span></div>\n</div>\n')
m_kv = re.search(r'<div class="kv">\n.*?\n</div>\n', src, flags=re.S)          # the 09-28 tile grid, whole
assert m_kv and m_kv.group(0).count('<div>') == 4
src = src[:m_kv.end()] + "\n" + UPD + src[m_kv.end():]
src = sub(src, r'<h2[^>]*>3\. Data notes</h2>', SEC + '\n<h2>4. Data notes</h2>', regex=True)
k = src.index("<h2>4. Data notes</h2>"); e = src.index("</ul>", k)
src = src[:e] + NOTES + src[e:]
sys.path.insert(0, HERE)
import add_outline
src, _ = add_outline.process(src)
open(os.path.join(HERE, "..", "report_v3_sections.html"), "w", encoding="utf-8").write(src)
print("wrote report.html", len(src))
