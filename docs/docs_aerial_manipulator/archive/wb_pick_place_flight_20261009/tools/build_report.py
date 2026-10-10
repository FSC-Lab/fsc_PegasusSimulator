#!/usr/bin/env python3
"""Build ../report.html for the 2026-10-09 whole-body pick-and-place hardware flights.

    python3 build_report.py && python3 build_artifact_page.py <out.html>

Tables come from ../analysis/{tracking,grasp,hover,payload_budget}.json, the figures' data from ../analysis/fig_*.json (fig_data.py),
the chart code from figures.js, the style from report_style.css (the 0928 report's) plus the few rules below.
The outline is added by add_outline.py (run by build_artifact_page.py).
"""
import json, os

HERE = os.path.dirname(os.path.abspath(__file__))
AN = os.path.join(HERE, "..", "analysis")
J = lambda n: json.load(open(os.path.join(AN, n)))
T, GR, HOV, PB = J("tracking.json"), J("grasp.json"), J("hover.json"), J("payload_budget.json")
raw = lambda n: open(os.path.join(AN, n)).read().replace("</", "<\\/")

EXTRA_CSS = """
.plot{width:100%}.plot.h3d{height:clamp(320px,34vw,400px)}.plot.herr{height:900px}.plot.hpick{height:560px}.plot.hplace{height:560px}
.plot.hhover{height:clamp(300px,42vw,520px)}.plot.hload{height:640px}.plot.hview{height:500px}
.vleg{display:flex;flex-wrap:wrap;gap:4px 16px;align-items:center}.vleg span{white-space:nowrap}
.plot.hmap{height:420px}.plot.hts{height:420px}.plot.hland{height:520px}
.pgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,420px),1fr));gap:10px}
.ee3d-grid.three{grid-template-columns:repeat(auto-fit,minmax(min(100%,300px),1fr))}
.pgrid>div{min-width:0}
.ee3d .tog{display:inline-flex;flex-wrap:wrap;gap:6px}.ee3d .tog button{border:1px solid var(--line);border-radius:6px}
.sw.big{border-top-width:5px}
table td.ok{color:var(--aqua);font-weight:600}table td.no{color:var(--red);font-weight:600}
table.cmp th:first-child,table.cmp td:first-child{min-width:170px}
.verdict.warn{border-left-color:#c98500}
.steps li{margin:6px 0}
figure{margin:18px 0}figcaption b{color:var(--ink)}
table.tl td:first-child,table.tl th:first-child{white-space:nowrap;width:1%}table.tl td.n:first-child{text-align:left}
"""

SHORT = [("FLYING go_to_start", "to start"), ("FLYING execute_pick (approach)", "to the pick"), ("WAITING execute_pick", "hover behind the stem"),
         ("FLYING execute_pick (descent)", "slide in"), ("DONE execute_pick", "on the handle"), ("FLYING exit_pick", "lift"),
         ("DONE exit_pick", "hover after the lift"), ("FLYING go_to_place_start", "to place start"), ("DONE go_to_place_start", "hover at place start"),
         ("FLYING execute_place (approach)", "to the place"), ("WAITING execute_place", "hover above the place"),
         ("FLYING execute_place (descent)", "descend"), ("FLYING exit_place", "back out and climb"), ("DONE exit_place", "hover after the place"),
         ("FLYING go_to_land_start", "to land start"), ("FLYING execute_land", "to the landing hover"), ("COMPLETE", "landing hover")]


def leg_rows(nm):
    out = []
    for r in T[nm]["phases"]:
        lab = next((s for k, s in SHORT if r["label"].startswith(k)), None)
        if lab is None or r["t1"] - r["t0"] < 0.6:
            continue
        out.append(f'<tr><td>{lab}</td><td class="n">{r["t0"]:.1f}–{r["t1"]:.1f}</td><td class="n">{r["com"]["rms"]:.0f}</td><td class="n">{r["com"]["max"]:.0f}</td>'
                   f'<td class="n">{r["task"]["rms"]:.1f}</td><td class="n">{r["e_head"]["rms"]:.2f}</td><td class="n">{r["tilt_max"]:.1f}</td></tr>')
    return "\n".join(out)


def leg_table(nm, cap):
    return (f'<div class="tscroll"><table><thead><tr><th>{cap}</th><th>time [s]</th><th>airframe rms [mm]</th><th>airframe max [mm]</th>'
            f'<th>claw vs airframe rms [mm]</th><th>claw heading rms [°]</th><th>tilt max [°]</th></tr></thead><tbody>\n{leg_rows(nm)}\n</tbody></table></div>')


def n(v, d=0):
    return f"{v:.{d}f}"


tot = {k: T[k]["total"] for k in T}
HP = HOV["pooled"]


def hov_row(run, t_near, label, note=""):
    w = min(HOV["windows"][run], key=lambda r: abs(r["t0"] - t_near))
    c = w.get("settled", w["all"])["ee"]; ln = w["t1"] - w["t0"]
    pay = "basket" if w["state"] == "payload" else "none"
    return (f'<tr><td>{label}{note}</td><td>{pay}</td><td class="n">{ln:.0f} s</td><td class="n">{c["bias"]:.0f}</td>'
            f'<td class="n">{c["scatter"]:.0f}</td><td class="n">{c["p95"]:.0f}</td><td class="n">{c["max"]:.0f}</td></tr>')


def hp(st, key, d=0):
    return f'{HP[st]["ee"][key]:.{d}f}'


def pct(st, key):
    return f'{100 * HP[st]["ee"][key]:.0f}'


HPP = J("hover_pick_place.json")
TW = J("trim_window.json")
TC = J("trim_choice.json")


def tc_row(key, label):
    def c(nm):
        r = TC[nm]["res"].get(key)
        return f'<td class="n">{r["rms"]:.0f} / {r["p95"]:.0f}</td>' if r else '<td class="n">too little data</td>'
    return f'<tr><td>{label}</td>{c("p2")}{c("p1")}{c("p3")}</tr>'


def tw_row(W, label):
    c = lambda nm: (f'<td class="n">{TW[nm]["windows"][W]["rms"]:.0f} / {TW[nm]["windows"][W]["p95"]:.0f}</td>' if W in TW[nm]["windows"] else '<td class="n">—</td>')
    return f'<tr><td>{label}</td>{c("p1")}{c("p3")}{c("p2")}</tr>'


def pp_pool(sel):
    """Time-weighted rmse over the chosen hover windows (whole windows, arrival included)."""
    import math
    rows = [r for r in HPP["windows"] if r["tag"] == "all" and sel(r)]
    n = sum(r["ee"]["n"] for r in rows)
    q = lambda who, k: math.sqrt(sum(r[who]["n"] * r[who][k] ** 2 for r in rows) / n)
    return dict(s=n * 0.02, k=len(rows), h=q("ee", "h"), x=q("ee", "x"), y=q("ee", "y"), z=q("ee", "z"), ch=q("com", "h"), cz=q("com", "z"))


def pp_row(label, sel):
    r = pp_pool(sel)
    return (f'<tr><td>{label}</td><td class="n">{r["s"]:.1f} s, {r["k"]}</td><td class="n">{r["h"]:.1f}</td><td class="n">{r["x"]:.1f}</td><td class="n">{r["y"]:.1f}</td>'
            f'<td class="n">{r["z"]:.1f}</td><td class="n">{r["ch"]:.1f}</td><td class="n">{r["cz"]:.1f}</td></tr>')


PW = PB["windows"]; PG = PB["gauges"]; PM = PB["moment"]; PA = PB["arm"]; PU = PB["unloaded"]
TP_ = PB["to_place"]
p2, p3 = GR["p2"], GR["p3"]
v3 = lambda a, d=0: ", ".join(n(x, d) for x in a)
pm = lambda m, s: ", ".join(f"{a:.0f} ± {b:.0f}" for a, b in zip(m, s))

BODY = f"""
<h1>First Hardware Pick-and-Place Flights</h1>
<p class="sub">T650 aerial manipulator, whole-body 4-D L1 controller, EKF2-fused feedback, 9 October 2026. Three flights from the bag
<code>1009 - T650-AM whole-body Pick&amp;Place</code>: PP-1 at 17:25, PP-2 at 18:02, PP-3 at 18:18. Times are seconds from the start of each bag.</p>

<div class="verdict warn"><strong>Overall.</strong> The controller and the pick are in good shape, the place is not, and two events need fixing before the next flight.
<ul>
<li><b>Controller:</b> PP-1 flew all six legs to COMPLETE without a payload. Airframe error 33 mm rms and 105 mm at worst, claw against airframe 2.2 mm rms, tilt at most 2.5°, no motor saturation, no torque clamp.</li>
<li><b>Pick:</b> PP-2 hooked the basket and lifted it. The targets were right. Get matched the basket's resting pose to 3 mm and the gripper closed 21 mm behind and 1 mm beside the basket centre, for a target of 20 and 0.</li>
<li><b>Place:</b> PP-2 set the basket down 166 mm short of the place point, on the rim of the hat. It slid off and stayed hooked on the claw.</li>
<li><b>Link freeze:</b> PP-2 ended when the data link between the Pixhawk and the Orin stopped in hover. The vehicle travelled 2.6 m and came down outside the field.</li>
<li><b>Landings:</b> both manual landings ended tipped over, PP-1 at 45° nose down and PP-3 at 57° nose up.</li>
</ul></div>

<div class="kv">
<div><b>33 mm</b><span>airframe tracking error, rms over the whole mission without payload (PP-1)</span></div>
<div><b>{hp("payload","rms")} mm</b><span>claw hover error with the basket on the claw, rms. Without it: {hp("none","rms")} mm</span></div>
<div><b>166 mm</b><span>short of the place point when the basket touched down (PP-2)</span></div>
<div><b>2.6 m</b><span>travelled after the Pixhawk link froze in hover (PP-2)</span></div>
</div>

<h2>1. The three flights</h2>
<div class="tscroll"><table class="cmp"><thead><tr><th></th><th>PP-1, 17:25</th><th>PP-2, 18:02</th><th>PP-3, 18:18</th></tr></thead><tbody>
<tr><td>Flown</td><td>all six legs, status COMPLETE</td><td>pick, carry, place, to land start</td><td>pick only, then Reset and flown home</td></tr>
<tr><td>Time in DIRECT</td><td class="n">159 s</td><td class="n">130 s</td><td class="n">68 s</td></tr>
<tr><td>Payload on the claw</td><td>none. No step in thrust or arm torque at the lift</td><td>the basket, 190 g on the scale, from 47 s to 118 s</td><td>none. The gripper closed above the arch</td></tr>
<tr><td>Pick</td><td>closed 12 mm from the target, nothing lifted</td><td class="ok">hooked and lifted</td><td class="no">missed, EE Offset z was 0.20</td></tr>
<tr><td>Place</td><td>opened 65 mm from the target</td><td class="no">opened 140 mm from the target, basket not released</td><td>not flown</td></tr>
<tr><td>End of the flight</td><td>descent to 0.35 m in DIRECT, pilot took STAB at 0.40 m, tipped 45° nose down</td><td class="no">link freeze in hover at 1.0 m, 2.6 m uncontrolled, pilot landed it</td><td>descent to 0.35 m in DIRECT, pilot took STAB at 0.44 m, tipped 57° nose up</td></tr>
<tr><td>Basket stream (<code>obj_0</code>)</td><td class="no">dead for the whole flight</td><td>live, orientation flips near the claw</td><td>live, orientation flips near the claw</td></tr>
</tbody></table></div>
<p>In PP-1 the thrust and the arm torques show no load at any time, and the basket's last streamed pose is 4.0 m from the pick hat. The flight is used here as the unloaded reference for the controller.</p>

<h3>1.1 Settings each flight used</h3>
<p>Recovered from the planner's <code>pick_place/info</code> topic and its log lines. Positions in metres, mocap frame.</p>
<div class="tscroll"><table class="cmp"><thead><tr><th></th><th>PP-1</th><th>PP-2</th><th>PP-3</th></tr></thead><tbody>
<tr><td>Pick, from Get (x, y, z, yaw)</td><td class="n">0.977, 1.034, 0.871, 0.6°</td><td class="n">1.007, 1.018, 0.864, 1.0°</td><td class="n">1.014, 1.022, 0.865, 1.3°</td></tr>
<tr><td>EE Offset (x, y, z)</td><td class="n">−0.02, 0, 0.16</td><td class="n">−0.02, 0, 0.16</td><td class="n">−0.02, 0, 0.20</td></tr>
<tr><td>Place (x, y, z, yaw)</td><td class="n">−1.06, −1.02, 0.86, −180°</td><td class="n">−1.03, −1.03, 0.86, −180°</td><td class="n">−1.03, −1.08, 0.86, −180°</td></tr>
<tr><td>Vertical Margin / Side Margin</td><td class="n">0.20 / 0.10</td><td class="n">0.20 / 0.10</td><td class="n">0.20 / 0.10</td></tr>
<tr><td>Start point after Adjust (x, y)</td><td class="n">0.018, 0.017</td><td class="n">0.056, 0.020</td><td class="n">0.032, 0.013</td></tr>
<tr><td>Wait before Pick was pressed</td><td class="n">39.0 s</td><td class="n">4.2 s</td><td class="n">1.6 s</td></tr>
<tr><td>Wait before Place was pressed</td><td class="n">1.6 s</td><td class="n">0.8 s</td><td class="n">not flown</td></tr>
<tr><td>Planner's trim at the pick (x, y, z)</td><td class="n">+17, +6, +3 mm</td><td class="n">+9, +2, −1 mm</td><td class="n">−14, −5, −4 mm</td></tr>
<tr><td>Planner's trim at the place (x, y, z)</td><td class="n">+36, +22, −2 mm</td><td class="n">+55, −8, −13 mm</td><td class="n">not flown</td></tr>
</tbody></table></div>

<h2>2. Does the whole-body controller work well?</h2>
<div class="verdict good"><strong>Yes without a payload. With this payload it flies stably but not accurately enough to place.</strong>
Unloaded, the claw hovers within {hp("none","rms")} mm rms of its target and the airframe tracks the moving legs to 39 mm rms.
With the 190 g basket, which is not in the controller's model, the hover error doubles to {hp("payload","rms")} mm rms and the lift costs a 25 cm excursion.</div>

<h3>2.1 Flight paths in 3-D</h3>
<figure>
<div class="ee3d">
 <div class="ee3d-bar">
  <span class="tog" id="f3d-tog">
   <button type="button" aria-pressed="true" data-k="ref"><span class="sw dash" style="--c:#8f8e84"></span>planned claw path</button>
   <button type="button" aria-pressed="true" data-k="base"><span class="sw" style="--c:#3987e5"></span>airframe</button>
   <button type="button" aria-pressed="true" data-k="claw"><span class="sw" style="--c:#d95926"></span>claw</button>
   <button type="button" aria-pressed="true" data-k="basket"><span class="sw big" style="--c:#199e70"></span>basket</button>
   <button type="button" aria-pressed="true" data-k="fly"><span class="sw big" style="--c:#e66767"></span>after the link froze</button>
  </span>
  <button type="button" class="plain" id="f3d-top">Top view</button>
  <button type="button" class="plain" id="f3d-reset">Reset view</button>
 </div>
 <div class="ee3d-grid three">
  <div class="ee3d-cell"><p class="ee3d-title">PP-1, full mission without payload</p><div class="plot h3d" id="f3d-p1"></div></div>
  <div class="ee3d-cell"><p class="ee3d-title">PP-2, pick, carry and place attempt</p><div class="plot h3d" id="f3d-p2"></div></div>
  <div class="ee3d-cell"><p class="ee3d-title">PP-3, pick attempt, then flown home</p><div class="plot h3d" id="f3d-p3"></div></div>
 </div>
 <p class="note plot-missing" hidden>The interactive charts need the Plotly library, which did not load. The tables below carry the same numbers.</p>
</div>
<figcaption><b>Fig. 1.</b> Each flight from DIRECT entry to the end of the controller's flight. Airframe, claw and basket are mocap positions, the claw by the planner's kinematics on the mocap pose and the measured joints. The dashed line is the claw path the planner streamed. The grey posts are the two pillars. Drag to rotate, scroll to zoom, hover for time and position.</figcaption>
</figure>

<h3>2.2 Tracking errors over time</h3>
<figure>
<div class="ee3d">
 <div class="ee3d-bar"><span class="seg" id="ferr-pick">
  <button type="button" aria-pressed="true" data-k="p1">PP-1</button><button type="button" aria-pressed="false" data-k="p2">PP-2</button><button type="button" aria-pressed="false" data-k="p3">PP-3</button></span>
  <span class="note">Shaded bands are the moving legs. Hover for all values at one time.</span></div>
 <div class="plot herr" id="ferr"></div>
</div>
<figcaption><b>Fig. 2.</b> Airframe error is the system centre of mass minus its reference. Claw vs airframe is the controller's own task error, the claw's offset from the centre of mass against the planned offset. Angles: claw heading error, airframe yaw error and tilt. Thrust and the joint 2 and 3 torques are the controller's commands. In PP-2 the thrust command steps up by 3.7 N at 47 s when the basket lifts and back down at 118 s when it is removed. Section 2.5 explains why that is more than the basket's weight.</figcaption>
</figure>

<h3>2.3 Tracking statistics</h3>
<div class="tscroll"><table class="cmp"><thead><tr><th></th><th>PP-1, no payload</th><th>PP-2, before the lift</th><th>PP-2, with the basket</th><th>PP-3, no payload</th></tr></thead><tbody>
<tr><td>Airframe error in hover, rms / max [mm]</td><td class="n">28 / 69</td><td class="n">15 / 32</td><td class="n">63 / 164</td><td class="n">15 / 30</td></tr>
<tr><td>Airframe error on moving legs, rms / max [mm]</td><td class="n">39 / 105</td><td class="n">16 / 25</td><td class="n">55 / 116</td><td class="n">18 / 30</td></tr>
<tr><td>Airframe error at the lift, max [mm]</td><td class="n">42</td><td class="n">—</td><td class="n">281</td><td class="n">23</td></tr>
<tr><td>Claw vs airframe, rms / max [mm]</td><td class="n">2.2 / 7.6</td><td class="n">2.3 / 6.5</td><td class="n">14 / 46</td><td class="n">2.9 / 8.6</td></tr>
<tr><td>Claw heading error, rms / max [°]</td><td class="n">0.24 / 1.2</td><td class="n">0.23 / 0.9</td><td class="n">8.9 / 23.6</td><td class="n">0.29 / 1.0</td></tr>
<tr><td>Airframe yaw error, rms / max [°]</td><td class="n">0.45 / 3.8</td><td class="n" colspan="2">0.61 / 3.8</td><td class="n">0.62 / 3.5</td></tr>
<tr><td>Tilt, max [°]</td><td class="n">2.5</td><td class="n">2.1</td><td class="n">6.6</td><td class="n">2.1</td></tr>
<tr><td>Motor commands, min – max</td><td class="n">0.42 – 0.78</td><td class="n" colspan="2">0.38 – 0.86</td><td class="n">0.40 – 0.74</td></tr>
<tr><td>Control ticks with a saturated motor / a clamped joint</td><td class="n">0 / 0 of 39 766</td><td class="n" colspan="2">0 / 0 of 32 498</td><td class="n">0 / 0 of 17 068</td></tr>
<tr><td>Joint 2 / joint 3 commanded torque, max [N·m]</td><td class="n">1.05 / 0.46</td><td class="n">0.84 / 0.31</td><td class="n">2.11 / 1.22</td><td class="n">0.94 / 0.49</td></tr>
<tr><td>Joint tracking error j1–j4, rms [°]</td><td class="n">0.5, 2.2, 1.3, 0.1</td><td class="n" colspan="2">0.6, 2.9, 2.0, 16.9</td><td class="n">0.7, 1.8, 1.3, 0.1</td></tr>
</tbody></table></div>
<p>The yaw error's maximum is the first three seconds after DIRECT entry in every flight. The 2° offsets on joints 2 and 3 are the standing hold offset seen in earlier flights. The 16.9° on joint 4 in PP-2 is the hooked basket turning the wrist (Section 2.6). The hover rows count every sample between two legs, including the seconds after each arrival and the lift. Section 2.4 gives the settled hover alone.</p>
<p>For scale: the 09-28 circle flights measured 24–26 mm rms at the claw, and the 10-05 figure-8 flights 37–58 mm.</p>

<h4>PP-1 leg by leg</h4>
{leg_table("p1", "PP-1")}
<h4>PP-2 leg by leg</h4>
{leg_table("p2", "PP-2")}

<h3>2.4 Hover error without and with the payload</h3>
<div class="verdict"><strong>The basket doubles the hover error.</strong> Without a payload the claw hovers {hp("none","rms")} mm rms from its target and stays inside {hp("none","p95")} mm for 95 % of the time. With the basket it is {hp("payload","rms")} mm rms and {hp("payload","p95")} mm. Both the standing offset and the wander grow. The height holds to a few millimetres in both cases.</div>
<figure>
<div class="ee3d"><div class="plot hhover" id="fhover"></div></div>
<figcaption><b>Fig. 3.</b> Horizontal error of the claw in every settled hover of the three flights: measured claw minus the planner's claw reference, in the frame the planner and the controller work in, one dot every 0.1 s. A hover is a span in which the reference stands still. The first 3 s after each arrival are left out. The dotted rings are 20 and 50 mm. The solid ring holds 95 % of the dots.</figcaption>
</figure>
<div class="tscroll"><table class="cmp"><thead><tr><th>Claw, settled hover</th><th>Without payload</th><th>With the basket</th></tr></thead><tbody>
<tr><td>Data</td><td>{hp("none","seconds")} s in {HP["none"]["ee"]["windows"]} hovers, three flights</td><td>{hp("payload","seconds")} s in {HP["payload"]["ee"]["windows"]} hovers, PP-2</td></tr>
<tr><td>Offset: a hover's mean position against its target, rms / largest [mm]</td><td class="n">{hp("none","offset_rms")} / {hp("none","offset_max")}</td><td class="n">{hp("payload","offset_rms")} / {hp("payload","offset_max")}</td></tr>
<tr><td>Wander about that mean, rms [mm]</td><td class="n">{hp("none","wander")}</td><td class="n">{hp("payload","wander")}</td></tr>
<tr><td>Total horizontal error, rms [mm]</td><td class="n">{hp("none","rms")}</td><td class="n">{hp("payload","rms")}</td></tr>
<tr><td>95 % of the time inside [mm]</td><td class="n">{hp("none","p95")}</td><td class="n">{hp("payload","p95")}</td></tr>
<tr><td>Largest [mm]</td><td class="n">{hp("none","max")}</td><td class="n">{hp("payload","max")}</td></tr>
<tr><td>Share of the time within 20 / 50 / 70 mm [%]</td><td class="n">{pct("none","within20")} / {pct("none","within50")} / {pct("none","within70")}</td><td class="n">{pct("payload","within20")} / {pct("payload","within50")} / {pct("payload","within70")}</td></tr>
<tr><td>Height error, mean ± std [mm]</td><td class="n">{HP["none"]["ee"]["z_mean"]:+.0f} ± {hp("none","z_std")}</td><td class="n">{HP["payload"]["ee"]["z_mean"]:+.0f} ± {hp("payload","z_std")}</td></tr>
</tbody></table></div>
<p>The airframe's own numbers are the same to within 1 mm, because the claw follows the airframe to 2–3 mm. The hovers that matter for the pick and the place, one by one:</p>
<div class="tscroll"><table><thead><tr><th>Hover</th><th>Payload</th><th>Length</th><th>Offset [mm]</th><th>Wander [mm]</th><th>95 % inside [mm]</th><th>Largest [mm]</th></tr></thead><tbody>
{hov_row("p1", 51.1, "PP-1, behind the stem before Pick")}
{hov_row("p2", 32.6, "PP-2, behind the stem before Pick")}
{hov_row("p2", 40.1, "PP-2, on the handle before the close")}
{hov_row("p3", 40.6, "PP-3, after the missed pick")}
{hov_row("p2", 49.2, "PP-2, after the lift")}
{hov_row("p2", 68.3, "PP-2, at the place start")}
{hov_row("p2", 80.8, "PP-2, above the place when Place was pressed", '<br><span class="note">first 2 s after arrival, not settled</span>')}
{hov_row("p2", 96.8, "PP-2, after the place, basket still on the claw")}
{hov_row("p1", 125.5, "PP-1, above the place when Place was pressed", '<br><span class="note">first 3 s after arrival, not settled</span>')}
</tbody></table></div>
<h4>Hover RMSE at the pick and at the place</h4>
<p>Whole hovers at the two sites, arrival seconds included, because that is when the buttons were pressed. Along and across refer to the approach direction, which is world x at both sites.</p>
<div class="tscroll"><table><thead><tr><th>Hover</th><th>Data, hovers</th><th>Claw horizontal [mm]</th><th>along [mm]</th><th>across [mm]</th><th>Claw height [mm]</th><th>Airframe horizontal [mm]</th><th>Airframe height [mm]</th></tr></thead><tbody>
{pp_row("Pick, no payload, three flights<br><span class='note'>behind the stem and on the handle, until the lift starts</span>", lambda r: r["grp"] == "pick")}
{pp_row("Place, with the basket, PP-2<br><span class='note'>above the place before Place, and after Exit To Place with the basket still hooked</span>", lambda r: r["grp"] == "place" and r["state"] == "payload")}
{pp_row("Place, no payload, PP-1<br><span class='note'>above the place before Place, and after Exit To Place</span>", lambda r: r["grp"] == "place" and r["run"] == "PP-1")}
</tbody></table></div>
<p>RMSE is the root mean square of measured minus reference, so it holds both the standing offset and the wander. The claw's height error with the basket is mostly its 7 mm sag under the load.</p>
<h4>What this means for the pick and the place</h4>
<ul>
<li><b>Pick, no payload.</b> The gripper closes when the claw is within 20 mm of its target and within 3 mm across the stem. The unloaded claw is inside 20 mm for {pct("none","within20")} % of the time, so the close comes after a wait of a few seconds. All three picks closed 3 to 12 mm from the target. The hover is good enough for the pick.</li>
<li><b>Place, with the basket.</b> The basket is 110 × 115 mm and the hat is 200 mm across. All four corners are on the hat only when the basket's centre is within 20 mm of the hat's centre. Beyond roughly 70 mm the basket tips off. The loaded claw is within 50 mm for {pct("payload","within50")} % of the time and within 70 mm for {pct("payload","within70")} %. On top of that the basket hangs 20 to 30 mm away from where the planner assumes (Section 3.2). So the place is the harder half, as you expected, and the loaded hover alone uses most of the hat.</li>
<li><b>Arrival is worse than the settled hover.</b> In the first 2 to 3 s above the place the offset was 36 mm without the basket and 47 mm with it. Both Place commands were given in that time.</li>
<li><b>Height is not the limit.</b> It holds to ±5 mm without and ±6 mm with the basket. The claw sags 7 mm under the load.</li>
<li><b>What the loaded error looks like.</b> The basket's weight acts 0.25 m ahead of the centre of mass, which is 0.45 N·m nose down (Section 2.5), and the attitude error rose from 0.02 rad rms before the lift to 0.06–0.10 rad with the basket. The standing offset was the same in all three loaded hovers: about 25 mm toward world +x and 10 mm toward +y, at headings of 0° and 180°. It is fixed to the room, not to the vehicle. I do not know its cause. An earlier version of this page tied it to the standing attitude error. The sizes match but the signs do not, so that explanation is withdrawn.</li>
<li><b>How much data this is.</b> The loaded numbers come from one flight and 23 s of settled hover. Treat them as a first measurement.</li>
</ul>

<h3>2.5 The vertical load: why the controller reads more than 190 g</h3>
<div class="verdict"><strong>The basket is 190 g. The 0.31 kg in the first version of this report was my conversion, and it was wrong.</strong>
I divided the step in the controller's thrust command by g. That command is in the controller's own thrust units, which were 7 % larger than delivered newtons at the pick and 13 % larger at the end of the flight, and the step also held battery sag.
The pitch moment the basket put on the vehicle corresponds to {min(v["F"] for v in PM.values()):.1f} to {max(v["F"] for v in PM.values()):.1f} N at the claw, which is 175 to 195 g.</div>
<p>The controller holds no payload mass. Its observer estimates one vertical force: the gap between the thrust it commands and the weight of the vehicle in its model. Everything that changes the thrust command needed to hover lands in that one number. After the grasp that is the basket's weight and four other things.</p>
<figure>
<div class="ee3d">
 <div class="ee3d-bar"><span class="seg" id="fload-pick">
  <button type="button" aria-pressed="true" data-k="p2">PP-2, with the basket</button><button type="button" aria-pressed="false" data-k="p1">PP-1, no payload</button></span>
  <span class="note">One-second means. Hover for the values.</span></div>
 <div class="plot hload" id="fload"></div>
</div>
<figcaption><b>Fig. 4.</b> Top: the collective thrust the controller commands, in its own units, and the thrust delivered, computed from the same motor commands and the battery voltage with the 20 August stepped-payload calibration of these motors. Middle: battery power. Bottom: battery voltage. In PP-1, with nothing on the claw, the command climbs by 2.8 N as the battery falls, while delivered thrust and power stay flat.</figcaption>
</figure>
<h4>Without a payload</h4>
<ul>
<li>Over the three flights the thrust command for the same hover ranged from {min(v["u1"][0] for v in PU.values()):.1f} to {max(v["u1"][1] for v in PU.values()):.1f} N as the battery went from {max(v["V"][0] for v in PU.values()):.1f} to {min(v["V"][1] for v in PU.values()):.1f} V. The observer's vertical estimate went from +{max(v["dz"][0] for v in PU.values()):.1f} to −{-min(v["dz"][1] for v in PU.values()):.1f} N with nothing on the claw.</li>
<li>Delivered thrust stayed at {PU["p1"]["T_fit"]:.1f} ± 0.3 N and battery power at {PU["p1"]["P"]:.0f} ± 5 W in all three flights. These two do not depend on the controller's units, so they are the gauges used below.</li>
<li>The delivered-thrust level, 34.5 N, is 6 % under the 36.7 N weight in the controller's model. The calibration was made on the bare airframe, and this vehicle's mass has not been confirmed on a scale. Only changes of the gauge are used here.</li>
</ul>
<h4>With the basket</h4>
<div class="tscroll"><table><thead><tr><th>Gauge</th><th>Reading</th><th>Load it implies [N]</th></tr></thead><tbody>
<tr><td>Scale</td><td>190 g</td><td class="n">1.86</td></tr>
<tr><td>Pitch moment on the vehicle<br><span class="note">rotational observer in delivered units, claw 0.24–0.25 m ahead of the centre of mass</span></td><td>{min(-v["dM_delivered"] for v in PM.values()):.2f}–{max(-v["dM_delivered"] for v in PM.values()):.2f} N·m nose down</td><td class="n">{min(v["F"] for v in PM.values()):.1f}–{max(v["F"] for v in PM.values()):.1f}</td></tr>
<tr><td>Battery power<br><span class="note">hover power goes as thrust to the power 1.5</span></td><td>{PW["pre"]["P"]:.0f} → {PW["post"]["P"]:.0f} W, +{100 * (PW["post"]["P"] / PW["pre"]["P"] - 1):.0f} %</td><td class="n">{min(v["power_N"][0] for v in PG.values()):.1f}–{max(v["power_N"][1] for v in PG.values()):.1f}</td></tr>
<tr><td>Delivered thrust from the motor commands</td><td>{PW["pre"]["T_fit"]:.1f} → {PW["post"]["T_fit"]:.1f} N</td><td class="n">{min(v["fit_N"] for v in PG.values()):.1f}–{max(v["fit_N"] for v in PG.values()):.1f}</td></tr>
<tr><td>Joint 2 torque<br><span class="note">midpoint between moving up and moving down, lever {PA["j2"]["lever"]:.2f} m</span></td><td>+{PA["j2"]["mid"]:.2f} N·m</td><td class="n">{PA["j2"]["F"]:.1f}</td></tr>
<tr><td>Joint 3 torque<br><span class="note">same, lever {PA["j3"]["lever"]:.2f} m</span></td><td>+{PA["j3"]["mid"]:.2f} N·m</td><td class="n">{PA["j3"]["F"]:.1f}</td></tr>
<tr><td>The controller's thrust command, in its own units</td><td>{PW["pre"]["u1"]:.1f} → {PW["post"]["u1"]:.1f} N</td><td class="n">{PB["lift"]["d_u1"]:.1f}</td></tr>
</tbody></table></div>
<ul>
<li><b>The moment agrees with the scale.</b> It is the one gauge that only a load on the claw can move.</li>
<li><b>The two thrust gauges read 0.4 to 0.9 N more than the weight.</b> That extra has no matching pitch moment, so it is not weight on the claw. It is on the thrust side. The likely cause is that thrust per motor command falls faster with throttle than the 20 August calibration says. That calibration was made at lower throttle and higher voltage. Rotor wash on the open basket would also add vertical load, but it would add nose-down moment with it.</li>
<li><b>The joint torques cannot settle it.</b> Gearbox friction holds 0.15 to 0.25 N·m either way, which is about ±1 N at these levers. Joint 3 reads higher than every other gauge and I have no explanation for it.</li>
</ul>
<h4>What was in the vertical estimate just before the place</h4>
<p>At 79–84 s the observer read −{-PW["ready_place"]["dz"]:.1f} N. The table accounts for all but 0.1 N of it.</p>
<div class="tscroll"><table><thead><tr><th>Contribution</th><th>[N]</th></tr></thead><tbody>
<tr><td>Already there before the pick: battery sag since take-off</td><td class="n">{-PW["pre"]["dz"]:.1f}</td></tr>
<tr><td>The basket's weight, 190 g</td><td class="n">1.9</td></tr>
<tr><td>Thrust beyond the weight, by the delivered-thrust gauge. The power gauge says 0.4</td><td class="n">{TP_["d_T_fit"] - 1.86:.1f}</td></tr>
<tr><td>Battery sag in the 40 s between the pick and the place, {PW["pre"]["V"]:.2f} → {PW["ready_place"]["V"]:.2f} V</td><td class="n">{TP_["battery"]:.1f}</td></tr>
<tr><td>Thrust per motor command falling at the higher throttle, {PW["pre"]["mot"]:.2f} → {PW["ready_place"]["mot"]:.2f}, by the calibration</td><td class="n">{TP_["throttle"]:.1f}</td></tr>
<tr><td>Units: the controller's newtons were 7 % larger than delivered newtons</td><td class="n">{TP_["unit"]:.1f}</td></tr>
<tr><td><b>Total</b></td><td class="n"><b>{-PW["pre"]["dz"] + 1.86 + (TP_["d_T_fit"] - 1.86) + TP_["battery"] + TP_["throttle"] + TP_["unit"]:.1f}</b></td></tr>
</tbody></table></div>
<ul>
<li><b>At the lift itself</b> the command stepped by {PB["lift"]["d_u1"]:.1f} N: {PB["lift"]["d_T_fit"]:.1f} N of load by the delivered-thrust gauge, {PB["lift"]["battery"]:.1f} N of battery sag in the 10 s between the two windows, {PB["lift"]["throttle"]:.1f} N from the throttle and {PB["lift"]["unit"]:.1f} N of units.</li>
<li><b>The vertical channel copes with all of it.</b> The airframe's height error with the basket is 0 ± 4 mm. What disturbs the hover is the moment (Section 2.4).</li>
<li><b>One hover would close the 0.4 to 0.9 N question.</b> Hook a compact 190 g weight on the claw in place of the basket and read the battery power. About 502 W means the basket's shape matters, which points to rotor wash. About 512 W again means the thrust side.</li>
</ul>

<h3>2.6 Arm torque and the torque caps</h3>
<div class="verdict"><strong>Raise joint 3's cap by 20 %. Leave joint 2.</strong> No command was clipped in these flights, but joints 2 and 3 came within 3 % of their caps for a few hundredths of a second. Joint 3 has room in the servo. Joint 2's limit is heat, which a higher cap does not help.</div>
<div class="tscroll"><table class="cmp"><thead><tr><th>PP-2</th><th>Joint 2</th><th>Joint 3</th></tr></thead><tbody>
<tr><td>Arm controller's cap (<code>max_effort</code>)</td><td class="n">2.44 N·m, 365 duty counts</td><td class="n">1.42 N·m, 192 duty counts</td></tr>
<tr><td>Stall current the cap allows, share of the servo's 2.3 A</td><td class="n">1.01 A, 44 %</td><td class="n">0.57 A, 25 %</td></tr>
<tr><td>Applied torque holding the arm alone [N·m]</td><td class="n">0.6</td><td class="n">0.2</td></tr>
<tr><td>Applied torque carrying the basket, mean [N·m]</td><td class="n">1.4, 57 % of the cap</td><td class="n">0.9, 65 % of the cap</td></tr>
<tr><td>Largest torque the whole-body controller commanded [N·m]</td><td class="n">2.12, 87 %</td><td class="n">1.23, 87 %</td></tr>
<tr><td>Largest total the arm controller asked for, with its friction and gravity terms</td><td class="n">97 % of the cap</td><td class="n">98 % of the cap</td></tr>
<tr><td>Largest applied torque [N·m]</td><td class="n">2.18, 89 %, at 67.7 s</td><td class="n">1.36, 96 %, at 56.5 s</td></tr>
<tr><td>Time with the applied torque above 80 % / 90 % of the cap</td><td class="n">0.36 s / 0 s</td><td class="n">1.59 s / 0.02 s</td></tr>
<tr><td>Control ticks clipped at the cap</td><td class="n">0</td><td class="n">0</td></tr>
<tr><td>Servo current while carrying, mean / peak [counts]</td><td class="n">216 / 336</td><td class="n">140 / 204</td></tr>
</tbody></table></div>
<p>Applied torque is the servo's measured current times its torque constant. Both peaks came while the arm moved to the carry pose with the basket on it. Without a payload neither joint's applied torque passed 48 % of its cap.</p>
<div class="tscroll"><table class="wk"><thead><tr><th>Joint</th><th>Cap now</th><th>Recommended</th><th>Why</th></tr></thead><tbody>
<tr><td>3</td><td class="n">192 counts, 1.42 N·m</td><td class="n">230 counts, 1.70 N·m</td><td>It has the least headroom, and its cap allows only 25 % of the stall current, against 44 % on joint 2. At 230 counts it is 0.69 A, 30 %. Raise its servo PWM Limit from 330 to 380 in the same change, by the arm repository's own sizing rule. Otherwise the firmware cuts the back-EMF compensation earlier.</td></tr>
<tr><td>2</td><td class="n">365 counts, 2.44 N·m</td><td class="n">unchanged</td><td>Its peaks above 80 % lasted 0.36 s in total, and its cap is already 1.0 A. Its real limit is heat. It carried the basket at 180 to 224 current counts for 70 s. The arm repository records an overload shutdown of this joint's servo near 250 counts sustained, on 26 August with a 343 g load. A servo that shuts down in flight leaves the arm limp. A higher cap does not change the sustained current.</td></tr>
<tr><td>1 and 4</td><td class="n">57.6 counts, 0.34 and 0.39 N·m</td><td class="n">unchanged</td><td>Peaks 0.25 and 0.26 N·m. The basket turned the wrist by 52° because the controller's heading stiffness is soft: it never commanded more than 0.11 N·m on joint 4.</td></tr>
</tbody></table></div>
<ul>
<li><b>A cap protects the servo and whatever the arm pushes against.</b> After PP-1's touchdown the arm pushed into the floor at exactly these caps for 2.4 s (Section 4.2). With joint 3 at 230 counts it would push 20 % harder there. That is the cost of the raise.</li>
<li><b>For joint 2, reduce the heat instead.</b> Keep the loaded part of a flight short, leave two minutes between loaded flights as the arm repository's rule says, and record the servos' temperature. The temperature is read by the arm's hardware interface but is not in these bags.</li>
<li><b>If you still want more on joint 2,</b> 400 counts (2.67 N·m, +10 %) with its PWM Limit at 550 is the most I would add, and only with the temperature recorded.</li>
<li><b>The controller's own limit does not match.</b> <code>wb_tau_max</code> is 3.0 N·m for every joint, above all four arm caps. If the arm ever clips, the controller is not told. It did not happen here. Per-joint limits equal to the arm's caps would close this, and that needs a code change.</li>
</ul>

<h2>3. Are the pick-and-place targets precise?</h2>
<div class="verdict"><strong>The pick targets are precise. The place failed for other reasons than the typed point.</strong>
Get, EE Offset x and EE Offset y are correct to a few millimetres. EE Offset z = 0.16 works with 7 mm to spare and 0.20 is above the arch.
At the place the claw was 13 cm short when the basket touched down, from the planner's trim, the loaded tracking error and the way the basket hangs.</div>

<h3>3.1 The pick, from above and from the side</h3>
<figure>
<div class="ee3d">
 <div class="ee3d-bar"><span class="seg" id="fv-pick-run"><button type="button" aria-pressed="true" data-k="p2">PP-2, basket picked</button><button type="button" aria-pressed="false" data-k="p1">PP-1, no basket</button><button type="button" aria-pressed="false" data-k="p3">PP-3, missed</button></span>
  <span class="note vleg"><span><span class="sw" style="--c:#3987e5"></span>Ready To Pick</span><span><span class="sw" style="--c:#d95926"></span>Pick</span><span><span class="sw" style="--c:#199e70"></span>Exit To Pick</span><span>thick line: claw</span><span><span class="sw dot" style="--c:#b9b8ae"></span>basket centre</span><span><span class="sw dash thin" style="--c:#8f8e84"></span>planned claw path</span><span>✕ claw target</span></span></div>
 <div class="pgrid"><div><p class="ee3d-title">Top view</p><div class="plot hview" id="fv-pick-top"></div></div><div><p class="ee3d-title">Side view</p><div class="plot hview" id="fv-pick-side"></div></div></div>
</div>
<figcaption><b>Fig. 5.</b> The three stages of the pick. The origin is the centre of the pick hat, on its top surface. The horizontal axis is the direction the claw slides in along, which is world +x here. Left: seen from above, with the hat's 100 mm radius, the basket where it rested and its hanger. Right: seen from the side, with heights above the hat. The claw is its grasp point, from the vehicle's mocap pose and the measured joints, so it is in the same frame as the basket. The thick lines are the claw and the dotted lines of the same colour are the basket's centre, which hangs 0.17 m under the claw once it is lifted. The grey dashes are the claw path the planner asked for. Filled markers are events on the claw's track and open markers the same instants on the basket's. The dotted amber line in the side view is the claw height at which the fingers meet the arch. In PP-2 the claw waited 0.10 m behind the stem, slid in at grasp height, closed, and lifted the basket once the fingers met the arch. The planned exit is straight up by 0.20 m. The claw and the basket instead moved 25 cm forward while they rose, because the basket's weight pushed the vehicle forward, so the basket left the hat on a slope: its underside was 26 mm above the hat at 45 mm from the centre and 60 mm above it at the rim. PP-1 flew the same path with no basket there, and its hat is drawn at the captured pick point. In PP-3, with EE Offset z = 0.20, the claw came in 4 cm higher and closed above the arch.</figcaption>
</figure>
<div class="tscroll"><table class="cmp"><thead><tr><th>Claw in the basket frame (x, y, z) [mm]</th><th>PP-2, planner</th><th>PP-2, mocap</th><th>PP-3, planner</th><th>PP-3, mocap</th></tr></thead><tbody>
<tr><td>Hover behind the stem, mean ± std.<br><span class="note">Target −120, 0, 160 (PP-3: 200)</span></td><td class="n">{pm(p2["pick"]["ready_hold_planner"]["mean"], p2["pick"]["ready_hold_planner"]["std"])}</td><td class="n">{pm(p2["pick"]["ready_hold_mocap"]["mean"], p2["pick"]["ready_hold_mocap"]["std"])}</td><td class="n">{pm(p3["pick"]["ready_hold_planner"]["mean"], p3["pick"]["ready_hold_planner"]["std"])}</td><td class="n">{pm(p3["pick"]["ready_hold_mocap"]["mean"], p3["pick"]["ready_hold_mocap"]["std"])}</td></tr>
<tr><td>On the handle, mean ± std.<br><span class="note">Target −20, 0, 160 (PP-3: 200)</span></td><td class="n">{pm(p2["pick"]["on_handle_planner"]["mean"], p2["pick"]["on_handle_planner"]["std"])}</td><td class="n">{pm(p2["pick"]["on_handle_mocap"]["mean"], p2["pick"]["on_handle_mocap"]["std"])}</td><td class="n">{pm(p3["pick"]["on_handle_planner"]["mean"], p3["pick"]["on_handle_planner"]["std"])}</td><td class="n">{pm(p3["pick"]["on_handle_mocap"]["mean"], p3["pick"]["on_handle_mocap"]["std"])}</td></tr>
<tr><td>When the gripper was told to close</td><td class="n">{v3(p2["pick"]["at_close_planner"])}</td><td class="n">{v3(p2["pick"]["at_close_mocap"])}</td><td class="n">{v3(p3["pick"]["at_close_planner"])}</td><td class="n">{v3(p3["pick"]["at_close_mocap"])}</td></tr>
<tr><td>Claw height when the basket left the hat</td><td class="n">{n(p2["pick"]["claw_z_at_liftoff_planner"])}</td><td class="n">{n(p2["pick"]["claw_z_at_liftoff_mocap"])}</td><td class="n" colspan="2">basket not lifted</td></tr>
<tr><td>Time on the handle before the lift</td><td class="n" colspan="2">{n(p2["pick"]["on_handle_s"], 1)} s</td><td class="n" colspan="2">{n(p3["pick"]["on_handle_s"], 1)} s</td></tr>
</tbody></table></div>
<p>The frame is the basket's: x is the direction the claw slides in along, y is along the arch, z is up. The planner columns use the fused odometry and PX4's attitude, which is what the planner and the arm ground station gate on. The two columns differ by 4 to 10 mm in height because PX4's attitude and the mocap attitude differ by 2 to 3° (Section 4.7).</p>
<ul>
<li><b>Get is accurate.</b> The capture differed from the basket's resting pose by (−2.3, −1.9, −0.4) mm in PP-2 and (−2.7, −2.6, −0.6) mm in PP-3, and by 0.03° in yaw.</li>
<li><b>EE Offset x and y are right.</b> At the close in PP-2 the claw sat 21 mm behind and 1 mm beside the basket centre in the planner's frame, 27 and 4 mm by mocap. The hanger plane is 27.5 mm behind the basket centre in the CAD file, and the hanging basket put the claw 29 mm behind it, so the fingers sat on the plane.</li>
<li><b>EE Offset z = 0.16 is close to the limit.</b> The basket lifted off when the claw reached 167 mm in the planner's frame. The fingers therefore passed 7 mm under the arch. While the claw waited on the handle its height ranged from 152 to 167 mm, so it touched the arch at the top of that range.</li>
<li><b>EE Offset z = 0.20 is above the arch.</b> In PP-3 the gripper closed at 192 mm, 25 mm above the height where the fingers meet the arch, and the basket never moved.</li>
<li><b>The close dragged the basket.</b> In PP-2 the gripper was told to close with the claw centred to 1 mm. Closing took 0.6 s, the claw drifted 18 mm sideways in that time, and the jaws moved the basket 30 mm across the hat before the lift.</li>
</ul>
<p>What limits the pick is the hover, not the setpoint. In PP-1's 39 s wait behind the stem the claw's error had a mean of 4 mm, a standard deviation of 15, 16 and 3 mm in x, y and z, a peak-to-peak of 67, 79 and 14 mm and a period of 6 to 7 s.</p>

<h3>3.2 How the basket hangs on the claw</h3>
<ul>
<li>The basket hangs tilted by 10.8°, about the arch. The hanger plane is 27.5 mm off the basket centre, so this is how the basket is built. The tilt reached 19° during the carry.</li>
<li>Its centre therefore sits 20 mm closer to the vehicle than the planner assumes. In the hover after the lift the claw was (+0.5, +17.6, 171.5) mm from the basket centre. The planner assumes (−20, 0, 160).</li>
<li>Above the place, with the claw within 8 mm of its goal, the basket was 29 mm short of the place point and 17 mm to the side.</li>
<li>The 17 mm sideways offset comes from where the jaws caught the stem in this flight. It is not a constant.</li>
</ul>

<h3>3.3 The place, from above and from the side</h3>
<figure>
<div class="ee3d">
 <div class="ee3d-bar"><span class="seg" id="fv-place-run"><button type="button" aria-pressed="true" data-k="p2">PP-2, with the basket</button><button type="button" aria-pressed="false" data-k="p1">PP-1, no basket</button></span>
  <span class="note vleg"><span><span class="sw" style="--c:#3987e5"></span>Ready To Place</span><span><span class="sw" style="--c:#d95926"></span>Place</span><span><span class="sw" style="--c:#199e70"></span>Exit To Place</span><span>thick line: claw</span><span><span class="sw dot" style="--c:#b9b8ae"></span>basket centre</span><span><span class="sw dash thin" style="--c:#8f8e84"></span>planned claw path</span><span>✕ claw target</span></span></div>
 <div class="pgrid"><div><p class="ee3d-title">Top view</p><div class="plot hview" id="fv-place-top"></div></div><div><p class="ee3d-title">Side view</p><div class="plot hview" id="fv-place-side"></div></div></div>
</div>
<figcaption><b>Fig. 6.</b> The three stages of the place, drawn like Fig. 5. The origin is the typed place point, the approach runs along world −x, and the dashed outline is the basket if it stood on that point. The planned claw path in the side view is a loop of three parts: the descent, the back-out by the Side Margin along the bottom, and the exit climb. The right-hand dashed line is the descent as commanded, already shifted by the planner's trim. The dotted white line above the cross is where the descent would run without the trim. The left-hand dashed line is the exit climb. In PP-2 the claw was 56 mm beyond the place point when Place was pressed, swung back to 150 mm before it and came down there. The basket touched down 164 mm before the point, at resting height. The gripper opened, the basket dropped 4 cm off the rim onto the open fingers, and the exit carried it away again. The hat is drawn centred on the typed point and at the pick hat's height. Its real position was not measured, and the three flights used three place points up to 6 cm apart (Section 1.1). The basket came to rest at hat height 164 mm from the typed point, so the real hat reaches further toward the vehicle than drawn.</figcaption>
</figure>
<div class="tscroll"><table class="tl"><thead><tr><th>Time [s]</th><th>What happened in PP-2</th><th>Claw short of its target [mm]</th></tr></thead><tbody>
<tr><td class="n">81.6</td><td>The claw arrives above the target, still swinging past it.</td><td class="n">−28</td></tr>
<tr><td class="n">82.4</td><td>Place is pressed 0.8 s after arrival. The planner averages the claw's error over the last second and shifts the goal by it: 55 mm toward the vehicle.</td><td class="n">−77</td></tr>
<tr><td class="n">83–85</td><td>The reference moves 55 mm. The vehicle swings back on its own at the same time.</td><td class="n">−72 to +41</td></tr>
<tr><td class="n">86.0</td><td>Peak on the near side.</td><td class="n">+148</td></tr>
<tr><td class="n">87–88.5</td><td>Hover before the descent: 55 mm of trim plus 60–70 mm of tracking error with the basket.</td><td class="n">+115 to +125</td></tr>
<tr><td class="n">89.5</td><td>The basket touches down 166 mm short of the place point and 25 mm to the side. The hat's radius is 100 mm.</td><td class="n">+130</td></tr>
<tr><td class="n">90.8</td><td>Descent complete. The ground station logs "EE 140 mm from the target" and opens the gripper. The hook place opens without checking the position.</td><td class="n">+134</td></tr>
<tr><td class="n">91.0–91.5</td><td>The basket slides off the rim, drops from 0.860 to 0.823 m and hangs on the open fingers.</td><td class="n"></td></tr>
<tr><td class="n">92–97</td><td>The exit backs out 10 cm, dragging the basket, then climbs 20 cm and lifts it again.</td><td class="n"></td></tr>
<tr><td class="n">105–110</td><td>The operator opens, closes and opens the gripper. The basket stays on the claw.</td><td class="n"></td></tr>
<tr><td class="n">119</td><td>The basket leaves the claw. From 122 s it rests on the floor at (−1.56, 2.95), 4 m away, so it did not fall from the claw.</td><td class="n"></td></tr>
</tbody></table></div>
<p>PP-1 flew the same sequence without a payload. Place was pressed 1.6 s after arrival, the trim was (+36, +22) mm, and the gripper opened 65 mm from the target: 61 mm short and 20 mm to the side.</p>
<h4>Why the basket landed short</h4>
<p>At the touchdown in PP-2 the 164 mm splits into three parts along the approach, all toward the vehicle:</p>
<div class="tscroll"><table><thead><tr><th>Part</th><th>PP-2, with the basket [mm]</th><th>PP-1, no basket, at the release [mm]</th></tr></thead><tbody>
<tr><td>The planned claw path behind the claw's target: the planner's descent trim</td><td class="n">55</td><td class="n">36</td></tr>
<tr><td>The claw behind its planned path: tracking error</td><td class="n">74</td><td class="n">33</td></tr>
<tr><td>The basket's centre behind where the planner assumes it under the claw</td><td class="n">35</td><td class="n">—</td></tr>
<tr><td><b>Basket short of the place point</b> (PP-1: claw short of its target)</td><td class="n"><b>164</b></td><td class="n"><b>68</b></td></tr>
</tbody></table></div>
<p>The basket, which the controller's model does not contain, is in the second and third rows. Against PP-1 it added about 40 mm of tracking error, and the 35 mm is how it hangs. The first row is the planner's own shift of the path and has nothing to do with the model. In Fig. 6 it is the dashed descent line standing 55 mm to the left of the target cross.</p>
<ol>
<li><b>The trim sampled a swing.</b> The planner's descent trim averages the claw's error over 1 s. The claw's hover error swings with a 6 to 7 s period, and 2 s averages of it ranged from −21 to +23 mm in x during PP-1's long wait, around a mean of 4 mm. There is no steady offset for the trim to remove on this controller. Both places were pressed during the arrival overshoot, so both trims pushed the goal toward the vehicle, by 36 and 55 mm.</li>
<li><b>The loaded airframe sat behind its reference.</b> 58 to 86 mm during the descent in PP-2 and 74 mm at the touchdown. The trim's own 55 mm step and the swing back from the arrival overshoot went the same way, and the claw overshot both. The loaded hover error is 45 mm rms and reaches 84 mm for 5 % of the time (Section 2.4).</li>
<li><b>The basket hangs 20 to 35 mm short of the planner's assumption</b> (Section 3.2). The planner expects its centre 20 mm ahead of the claw. At the touchdown it was 15 mm behind.</li>
<li><b>Nothing stopped the release.</b> With the hook grasp the ground station opens at the bottom of the descent whatever the claw's error.</li>
</ol>

<h3>3.4 Recommended values</h3>
<p>Revised on 10 October after your proposal (EE Offset z 0.17, Pick x + 0.01, Place x forward by 0.01 to 0.02).</p>
<div class="tscroll"><table class="wk"><thead><tr><th>Setting</th><th>Flown</th><th>Recommended</th><th>Evidence</th></tr></thead><tbody>
<tr><td>EE Offset x, y</td><td class="n">−0.02, 0</td><td class="n">−0.02, 0</td><td>Unchanged. The forward shift goes on the pick point instead.</td></tr>
<tr><td>EE Offset z</td><td class="n">0.16 (PP-3: 0.20)</td><td class="n">0.16, not 0.17</td><td>The fingers meet the arch when the claw is at 0.167. See the table below.</td></tr>
<tr><td>Pick point</td><td>Get</td><td>Get x + 0.01, rest as Get</td><td>At the close the stem was only 0.5 mm inside the jaws by mocap, and PP-3's claw stopped 10 to 18 mm short of the stem. Forward is +x because the pick yaw is 1°.</td></tr>
<tr><td>Place x</td><td>Get x</td><td>Get x − 0.03</td><td>At the place the basket's centre hung 10 to 22 mm behind the claw. The planner assumes 20 mm ahead. At yaw −180° forward is −x.</td></tr>
<tr><td>Place y</td><td>Get y</td><td>Get y</td><td>The basket hung 10 to 20 mm to one side in PP-2, from where the jaws caught the thin stem. A stem that fills the jaws should centre it.</td></tr>
<tr><td>Place z</td><td class="n">0.86 typed</td><td>Get z − 0.02, not higher</td><td>The claw must end below the arch or the open fingers still carry the basket. See below.</td></tr>
<tr><td>Place yaw</td><td class="n">−180</td><td class="n">−180</td><td></td></tr>
<tr><td>Vertical / Side Margin</td><td class="n">0.20 / 0.10</td><td class="n">0.20 / 0.10</td><td>The basket's underside is 17 cm above the hat at the hover. The claw waited at least 60 mm from the stem.</td></tr>
</tbody></table></div>

<h4>EE Offset z: why not 0.17</h4>
<p>Yesterday's claw heights at the pick, shifted to each setting. Heights are above the basket's centre. The fingers meet the arch at 167 mm in the planner's frame and 163 mm by mocap.</p>
<div class="tscroll"><table><thead><tr><th>EE Offset z</th><th>Gap under the arch while waiting to close, mean (smallest) [mm]</th><th>Claw above the contact height during the slide-in</th><th>Claw above the contact height while waiting</th></tr></thead><tbody>
<tr><td class="n">0.15</td><td class="n">22 (11)</td><td class="n">never</td><td class="n">never</td></tr>
<tr><td class="n">0.16, as flown</td><td class="n">12 (1)</td><td class="n">never, 6 mm to spare</td><td class="n">never, within 1 mm once</td></tr>
<tr><td class="n">0.17</td><td class="n">2 (−9)</td><td class="n">67 % of the time</td><td class="n">30 % of the time</td></tr>
</tbody></table></div>
<ul>
<li><b>You saw the gap correctly.</b> With 0.16 the fingers sat 12 mm under the arch on average, by mocap.</li>
<li><b>The gap costs nothing.</b> The climb takes up 12 mm in 0.8 s and meets the arch at about 5 cm/s.</li>
<li><b>Closing the gap costs the slide-in.</b> At 0.17 the fingers would be at arch height for two thirds of the slide-in and would push the arch instead of passing under it. About 0.5 N at that height tips or slides a 190 g basket.</li>
<li><b>PP-3 is the same mistake further on:</b> at 0.20 the fingers closed above the arch.</li>
<li>These heights belong to yesterday's handle. If the new print changed the arch's height, the rule is: EE Offset z = the height where the fingers meet the arch, less 7 to 10 mm.</li>
</ul>

<h4>Place x: how the basket hung under the claw</h4>
<div class="tscroll"><table><thead><tr><th>PP-2, basket centre relative to the claw, along the vehicle's heading [mm]</th><th>Planner's frame</th><th>Mocap</th></tr></thead><tbody>
<tr><td>What the planner assumes</td><td class="n">+20</td><td class="n">+20</td></tr>
<tr><td>Hover after the lift</td><td class="n">−5</td><td class="n">−1</td></tr>
<tr><td>Above the place</td><td class="n">−10</td><td class="n">−10</td></tr>
<tr><td>Descent, first part</td><td class="n">−19</td><td class="n">−13</td></tr>
<tr><td>Descent, until the touchdown</td><td class="n">−22</td><td class="n">−17</td></tr>
</tbody></table></div>
<ul>
<li>Positive means the basket's centre is ahead of the claw. It was behind, by 10 to 22 mm, so with the claw exactly on its target the basket lands 30 to 42 mm short of the place point. A shift of 0.01 to 0.02 is not enough. 0.03 puts it within about 10 mm.</li>
<li>The arch slid along the fingers during the carry, from 7 mm ahead to 17 mm behind, so this number is not fixed.</li>
<li><b>The new, wider stem may change it.</b> If the jaws now clamp the stem and the basket no longer hangs freely, it will sit closer to what the planner assumes, and 0.03 would overshoot. Check it on the ground with the new handle: grip the basket with the arm in the place pose and compare the basket's position with the claw's. The shift to type is 20 mm minus the distance by which the basket's centre is ahead of the claw.</li>
</ul>

<h4>Place z: keep Get z − 0.02</h4>
<ul>
<li>At the bottom the claw has to be below the height where the fingers meet the arch. Otherwise the basket's weight is still on the fingers when they open, an open gripper still carries the arch, and the back-out drags the basket off the hat.</li>
<li>With EE Offset z = 0.16 and Place z = Get z − 0.02 the claw's target is 140 mm above the basket's centre, 27 mm under the arch contact. Higher gives that margin away.</li>
<li>Yesterday's typed 0.86 was about 5 mm under the basket's resting height on the hat, not 20 mm: the basket first touched the place hat with its centre at 0.860 to 0.863 m. The claw still ended 15 to 23 mm under the arch contact, because by mocap it flies 11 to 16 mm lower at the place than the planner reads.</li>
<li>If you raise EE Offset z, lower Place z by the same amount. The two belong together.</li>
</ul>

<h4>The wider stem and the slide-in</h4>
<p>Between the start of the slide-in and the close, the claw's sideways error reached 17 mm in PP-2 and 25 mm in PP-3, by mocap. The open jaws must clear the stem by more than that on each side, so the open gap less the stem's width should be at least 50 mm. If it is less, the fingers will meet the stem during the slide-in on some attempts. Please measure the new stem against the open gripper.</p>

<h4>What removes most of the 13 cm</h4>
<p>These are not setpoints. The offsets above add margin, and the first three below remove the error.</p>
<ol class="steps">
<li><b>Wait before pressing Place.</b> At least 8 s after the status reads WAITING, with the 5 s trim window now set. PP-2 pressed after 0.8 s and PP-1 after 1.6 s.</li>
<li><b>Change the descent trim for this controller.</b> It was built for the decoupled controller's steady 5 cm hover offset. Taken over one second it adds error here in every case (Section 3.5). Its window is now 5 s in the hardware pick-and-place yaml, decided on 10 October (Section 3.5).</li>
<li><b>Gate the release.</b> Do not open the gripper when the claw is more than about 40 mm from its target sideways. That is a ground-station change and is not made yet.</li>
<li><b>The loaded hover sets the floor.</b> With the basket the claw is within 50 mm of its target two thirds of the time (Section 2.4). The three steps above remove the 13 cm. They do not remove this.</li>
</ol>

<h3>3.5 What a Pick or Place press does</h3>
<div class="verdict warn"><strong>The planner does not wait for the error to converge.</strong> It accepts the press as soon as the approach leg has ended, takes the claw's average error over the last second as "the hover error", and shifts the descent by it. A press during the arrival swing therefore builds that swing into the descent. For the pick the ground station catches it afterwards. For the place nothing does.</div>
<h4>At the press: the planner</h4>
<ol>
<li><b>Refused</b> only in three cases: the approach leg is still flying, the claw's latest error is more than 50 mm away from its own one-second average, or that average is larger than 150 mm.</li>
<li><b>The 50 mm is not a distance to the target.</b> With the descent trim on, which is the default and what flew, the check compares the claw with its own recent average. It measures how fast the claw is moving. A claw 76 mm from its target and drifting slowly passes.</li>
<li><b>The shift.</b> The one-second average becomes the trim. The planner moves the descent by it so that a claw with a steady offset lands on the target. If the claw sits behind its target, the descent moves forward by the same amount. If the claw is past its target, the descent moves back. A shift over 5 mm is flown sideways first, held 2 s, then the claw goes straight down.</li>
</ol>
<h4>After the descent: the ground station</h4>
<ul>
<li><b>Pick.</b> The gripper closes only when the claw is within 20 mm of its target and within 3 mm across the stem. Until then the claw waits on the handle. A hurried Pick costs a few seconds, not the grasp.</li>
<li><b>Place, hook grasp.</b> The gripper opens at the bottom of the descent whatever the error, and Exit To Place starts at once. The Place press is the last decision.</li>
</ul>
<h4>The five presses in the flights</h4>
<div class="tscroll"><table><thead><tr><th>Press</th><th>Wait after arrival</th><th>Claw error at the press [mm]</th><th>One-second average = the trim [mm]</th><th>Claw vs that average [mm]</th><th>Result</th></tr></thead><tbody>
<tr><td>PP-1 Pick</td><td class="n">39.0 s</td><td class="n">7</td><td class="n">18</td><td class="n">14</td><td>accepted, descent shifted 18 mm</td></tr>
<tr><td>PP-2 Pick</td><td class="n">4.2 s</td><td class="n">6</td><td class="n">9</td><td class="n">4</td><td>accepted, shifted 9 mm</td></tr>
<tr><td>PP-3 Pick</td><td class="n">1.6 s</td><td class="n">7</td><td class="n">15</td><td class="n">9</td><td>accepted, shifted 15 mm</td></tr>
<tr><td>PP-1 Place</td><td class="n">1.6 s</td><td class="n">31</td><td class="n">42</td><td class="n">15</td><td>accepted, shifted 42 mm</td></tr>
<tr><td>PP-2 Place</td><td class="n">0.8 s</td><td class="n">76</td><td class="n">57</td><td class="n">30</td><td>accepted, shifted 57 mm, of which 55 mm toward the vehicle</td></tr>
</tbody></table></div>
<ul>
<li>No press was refused. At PP-2's Place the claw was 76 mm from its target, outside the 50 mm the status line shows, and the planner still logged "claw within tolerance above the target, descending".</li>
<li>The status line reads "claw N mm from the point above the target (tolerance 50 mm), press Place to descend" from the moment the leg ends. It invites the press and does not say whether the claw has settled.</li>
<li>Even PP-1's Pick, after a 39 s wait and with the claw 7 mm from its target, got an 18 mm shift. The hover wanders with a 5 to 7 s period, so a one-second average is a sample of the wander, not a steady offset.</li>
</ul>
<h4>Trim off, or trim on and wait longer?</h4>
<p>Waiting removes the arrival swing. It does not make a one-second trim right. Replaying the planner's rule on the three long hovers of these flights: if Place or Pick had been pressed at any instant of the hover, the claw would reach the bottom this far from its target (rms / 95 % inside, mm).</p>
<div class="tscroll"><table><thead><tr><th>Setting</th><th>PP-2 at the place, basket on the claw, 21 s</th><th>PP-1 behind the stem, no payload, 39 s</th><th>PP-3 after the missed pick, no payload, 22 s</th></tr></thead><tbody>
{tc_row("off_any", "Trim off")}
{tc_row("on_1", "Trim on, 1 s window, as flown")}
{tc_row("on_5", "Trim on, 5 s window")}
{tc_row("on_10", "Trim on, 10 s window")}
</tbody></table></div>
<p>With the trim off the claw is at the bottom 3.2 s after the press. With it on, the shift is flown first and held 2 s, and the bottom comes 8.4 s after the press, as in both flown places.</p>
<ul>
<li><b>The one-second trim is the worst setting in every hover,</b> settled or not. Waiting longer with the window left at one second does not help.</li>
<li><b>Off removes the harm.</b> With the basket it leaves 48 mm rms, because the loaded hover has a steady offset that nothing then removes. In all three loaded hovers of PP-2 the claw sat about 25 mm toward world +x and 10 mm toward +y from its reference, at both headings.</li>
<li><b>A long window removes that offset</b> and leaves the wander, which is 34 mm rms with the basket. The 22 mm in the table comes from only 3 s of possible press times, so read it as "about the wander", not as 22.</li>
<li><b>Without a payload a long window costs nothing:</b> 23 against 22 mm and 7 against 11 mm.</li>
<li><b>Waiting for a small reading before pressing does not help with the trim off.</b> Pressing only when the error reads under 30 mm still lands at 46 mm rms, because the hover moves on during the descent.</li>
</ul>
<h4>Decided on 10 October: a 5 s window</h4>
<p><code>pick_place_descent_trim_window_s</code> is now 5 s in the hardware pick-and-place yaml, which both hardware controllers' planner reads. The simulation files keep 1 s. The setting applies to Pick as well, where it costs nothing.</p>
<div class="tscroll"><table><thead><tr><th>Place pressed this long after arrival</th><th>What the 5 s average holds</th><th>Trim error on the loaded hover of PP-2 [mm]</th></tr></thead><tbody>
<tr><td>under 2.5 s</td><td>less than half a window: the planner applies no shift</td><td class="n">no trim</td></tr>
<tr><td>5 s</td><td>the whole arrival swing</td><td class="n">49</td></tr>
<tr><td>6 s</td><td>most of the swing has left the window</td><td class="n">20</td></tr>
<tr><td>8 s</td><td>settled hover only</td><td class="n">20</td></tr>
<tr><td>10 s</td><td>settled hover only</td><td class="n">18</td></tr>
</tbody></table></div>
<ul>
<li><b>Press Place no earlier than 8 s after the status reads WAITING.</b> At exactly 5 s the average still holds the first two seconds of the arrival, when the loaded claw is 80 to 140 mm off.</li>
<li><b>Which way the descent moves.</b> In the settled loaded hover at the place the claw sat about 29 mm behind its target, toward the vehicle. A correct trim therefore moves the descent about 29 mm forward. On 9 October the one-second trim moved it 55 mm backward, because it sampled the claw while it was past the target.</li>
<li><b>What to expect.</b> About 38 mm rms at the bottom of the descent with the basket, against 48 mm with the trim off. A 10 s window with a 15 s wait would remove a little more.</li>
<li><b>The decoupled controller</b> reads the same planner block. It relies on the trim for its steady 5 cm offset, so there too the press must come at least 5 s after arrival, or the descent is not trimmed.</li>
</ul>

<h2>4. Problems in the flight test</h2>
<p>In order of how much they matter for the next flight.</p>

<h3>4.1 The Pixhawk data link froze in hover (PP-2)</h3>
<figure>
<div class="ee3d"><div class="pgrid"><div><div class="plot hmap" id="ffreeze-map"></div></div><div><div class="plot hts" id="ffreeze-ts"></div></div></div></div>
<figcaption><b>Fig. 7.</b> PP-2 after the stream from the Pixhawk stopped at 144.67 s. Left: the airframe's track from above, with a dot every 0.5 s. Right: attitude, speed and height from mocap. The vehicle was hovering at (−0.94, 0.01, 1.00) m with the mission's last leg still to fly.</figcaption>
</figure>
<div class="tscroll"><table class="tl"><thead><tr><th>Time [s]</th><th>Evidence</th></tr></thead><tbody>
<tr><td class="n">144.67</td><td>The Pixhawk's 400 Hz sensor and attitude topics and its 100 Hz odometry stop within the same millisecond, and none of its slower topics publishes again. This was 18:04:53. Mocap and every node on the Orin keep running.</td></tr>
<tr><td class="n">144.67–145.67</td><td>The flight node keeps computing on frozen feedback. Its attitude and position errors do not change for a full second.</td></tr>
<tr><td class="n">145.60</td><td>Roll 11.6°, yaw −14°, speed 0.62 m/s.</td></tr>
<tr><td class="n">145.67</td><td>The node logs POSITION FEEDBACK LOST after its 1.0 s gate, reverts to SAFETY and prints "NO feedback of any kind … TAKE MANUAL CONTROL".</td></tr>
<tr><td class="n">145.8–146.0</td><td>Roll goes from 14.8° back to 1.5°. Speed 1.45 m/s. The vehicle then coasts level.</td></tr>
<tr><td class="n">146.7–147.35</td><td>Descent at 1.2 to 1.3 m/s and touchdown at (−2.41, −1.51), still moving at 1.1 m/s.</td></tr>
<tr><td class="n">148.2</td><td>At rest at (−2.70, −2.00), upright, 2.6 m from the hover point and 0.45 m beyond the edge of the 4.5 × 4.2 m field the mission is laid out in.</td></tr>
<tr><td class="n">149.90</td><td>One sample of each Pixhawk topic arrives, then nothing until the bag ends at 166 s. It is fresh, so the Pixhawk never rebooted. It shows STAB, disarmed by the RC switch, and EKF2 no longer fusing mocap, so the Orin's data was not reaching the Pixhawk either.</td></tr>
</tbody></table></div>
<ul>
<li><b>What it was.</b> The uXRCE-DDS session between the Pixhawk and the Orin stalled in both directions. The Pixhawk held its last motor command for about a second and then levelled the vehicle, which fits PX4's offboard-loss timeout. The pilot landed it in STAB.</li>
<li><b>Ruled out by the data.</b> A Pixhawk reboot, since its clock is continuous. The battery, at 23.0 V and 20 A as before. The WiFi, since the Orin's own log reports the loss. The mocap, whose largest gap was 24 ms. The time-sync round trip gave no warning: 0.4 to 1.1 ms in the last seconds, as in the rest of the flight.</li>
<li><b>It may have happened twice.</b> In PP-3 the same stream stopped at 89.7 s, 7 s after disarm, while the flight node and the estimator kept publishing for 30 s more. If nobody powered the Pixhawk down at that moment, that is a second occurrence.</li>
</ul>
<h4>What to do</h4>
<ol class="steps">
<li>On the Orin, read the kernel log and the MicroXRCEAgent pane for 18:04:53. Look for the Ethernet link going down and for agent errors.</li>
<li>Pull the PX4 log of that flight and look at the <code>uxrce_dds_client</code> messages and the offboard-loss failsafe.</li>
<li>Reseat and strain-relieve the Ethernet cable between the Pixhawk and the Orin.</li>
<li>Shorten the blind second. Set PX4's <code>COM_OF_LOSS_T</code> to 0.2–0.3 s for direct-actuation flights. In the flight node, add a short timeout on attitude and rate, which today wait behind the 1.0 s odometry gate.</li>
<li>Do not fly a payload near the pillars until the cause is known.</li>
</ol>

<h3>4.2 Both manual landings tipped over, and the arm then pushed into the floor</h3>
<figure>
<div class="ee3d"><div class="pgrid">
<div><p class="ee3d-title">PP-1</p><div class="plot hland" id="fland-p1"></div></div>
<div><p class="ee3d-title">PP-3</p><div class="plot hland" id="fland-p3"></div></div></div></div>
<figcaption><b>Fig. 8.</b> The last seconds of PP-1 and PP-3. Height and attitude from mocap. PX4's own attitude agrees within 2°. Arm torque from the servo currents.</figcaption>
</figure>
<ul>
<li><b>PP-1.</b> The vehicle came down to 0.35 m in DIRECT through a drone ground-station target. The pilot took STAB at 0.40 m. It dropped at 1.2 m/s from 0.43 m, pitched from 15° to 49° in one second and stayed at 45° nose down. Disarm came 1.5 s after STAB.</li>
<li><b>PP-3.</b> STAB at 0.44 m. The vehicle climbed to 0.56 m, pitched 28° nose up in the air, hit the floor at 1.75 m/s and came to rest at 57° nose up.</li>
<li><b>The arm in PP-1.</b> After the pilot took over, the flight node still reported DIRECT. The planner's tilt guard saw 18.4° and flew its abort: gripper open, arm to the release pose. On the floor the arm followed that reference until joint 2 stopped at 32° against a target of 2°. Joints 2 and 3 then held 2.7 and 1.4 N·m, their caps, from 190.0 s until the bag ended 2.4 s later.</li>
</ul>
<h4>What to do</h4>
<ol class="steps">
<li>Land as the workflow says: back to SAFETY, then land and disarm from the drone ground station. Revert at 0.8 m or higher, since earlier flights measured a dip of about 20 cm at every revert on this vehicle.</li>
<li>If the pilot lands in STAB, hand over at 0.6 m or higher with the throttle stick at hover.</li>
<li>Make the flight node report SAFETY as soon as PX4 leaves OFFBOARD or disarms, and make the planner's guard do nothing when the vehicle is not in OFFBOARD. Neither change is made yet.</li>
</ol>

<h3>4.3 The place released the basket 13 cm short</h3>
<p>Section 3.3. The typed point was not the cause.</p>

<h3>4.4 The basket disturbs the hover more than in simulation</h3>
<p>Sections 2.4 to 2.6. The basket weighs 190 g, close to the 200 g the simulations used. In flight the lift cost 25 cm and 6.6° of tilt against 4.2° in simulation, the loaded hover error was twice the unloaded one, and joints 2 and 3 peaked at 89 % and 96 % of their torque caps.</p>

<h3>4.5 The basket's mocap body</h3>
<ul>
<li><b>Dead in PP-1.</b> <code>/vrpn_mocap/obj_0/pose</code> carried no message in 192 s. The processor kept republishing one frozen pose at 60 Hz, (−2.30, 3.25, 0.56), 11 423 identical samples, and <code>mocap_status</code> read normal throughout. A Get in that state stores a stale pose with zero scatter.</li>
<li><b>Flips near the claw.</b> In PP-2 and PP-3 the solved orientation jumped by 180° about the vertical, with 5° of tilt, whenever the claw was close. From 35 to 40 s in PP-2 it read a yaw of −179°. Before the approach it was right 99.8 to 100 % of the time.</li>
<li><b>Why it matters.</b> The drone's heading at the pick follows the captured yaw. A Get taken during a flip would send the drone to the far side of the pillar and turn EE Offset x around.</li>
<li><b>Fix.</b> Move one marker on the basket so the layout is not symmetric, and check in Motive that the body does not flip with the gripper next to it.</li>
<li><b>Guard added.</b> In the new planner build both Get buttons refuse a pose that is frozen and a yaw that changes by more than 10° inside the half-second window, and say why on the status line.</li>
</ul>

<h3>4.6 PP-3 missed the pick</h3>
<p>EE Offset z was raised from 0.16 to 0.20 after PP-2. That put the claw above the arch (Section 3.1).</p>

<h3>4.7 PX4's attitude and the mocap attitude disagree by 2 to 3°</h3>
<ul>
<li>Fixed to the body: 1.9 to 2.2° in roll and 2.2 to 2.9° in pitch in PP-1 and PP-2, and 1.2° and 1.4° in PP-3. A further 0.4 to 0.6° is fixed to the room.</li>
<li>The claw is 0.26 m ahead of the body origin, so this moves the planner's idea of the claw height by 5 to 10 mm against mocap.</li>
<li>The recommended EE Offset z of 0.16 is measured in the planner's own frame and already carries this. A level calibration of the Pixhawk and a re-zero of the vehicle's mocap body would remove it.</li>
</ul>

<h3>4.8 Smaller items</h3>
<ul>
<li>The Orin's topics show one 74 to 105 ms gap at 125.9 s in PP-2, a WiFi burst on the recording side.</li>
<li>Battery: 24.4 V at rest to 23.0 V under 20 A at the end of each flight. The thrust command for the same hover rose by 2.8 N over PP-1's 150 s for that reason (Section 2.5).</li>
<li>PP-1 hovered 39 s behind the stem before Pick. Nothing went wrong, and it is the best hover record in the bag.</li>
</ul>

<h2>5. Before the next flight</h2>
<ol class="steps">
<li><b>Find the link fault</b> (Section 4.1). This comes first.</li>
<li><b>Change the landing</b> (Section 4.2): SAFETY from 0.8 m, or STAB from 0.6 m.</li>
<li><b>Fix the basket's mocap body</b> so it cannot flip, and confirm <code>obj_0</code> is streaming before every Get.</li>
<li><b>Arm torque caps</b> (Section 2.6): joint 3 from 192 to 230 duty counts with its PWM Limit from 330 to 380, joint 2 unchanged. Record the servo temperatures.</li>
<li><b>Settings:</b> EE Offset (−0.02, 0, 0.16). Pick = Get x + 0.01. Place = Get x − 0.03, Get y, Get z − 0.02, yaw −180, after a ground check of how the basket hangs on the new handle (Section 3.4). Margins unchanged. The arm ground station now has a Get on the Place row and editable boxes on the Pick row. It needs the planner rebuilt on the Orin and the ground station rebuilt on the laptop.</li>
<li><b>Planner:</b> the descent trim's window is set to 5 s in the hardware pick-and-place yaml (Section 3.5). Pull it on the Orin.</li>
<li><b>Operator:</b> wait at least 8 s in the hover above the place before pressing Place.</li>
<li><b>First flight back:</b> repeat PP-2 over open floor, with the place hat on a low stand, before going back to the pillars.</li>
</ol>

<h2>6. Method and data</h2>
<ul>
<li><b>Bags.</b> <code>flight_wb_l1_4d_pick_and_place_20261009_172557</code>, <code>_180228</code>, <code>_181817</code>, recorded on the ground-station computer.</li>
<li><b>Measured states.</b> Airframe from the fused odometry the controller flies on, and separately from the vehicle's mocap body. Claw by the planner's kinematic model on either pose with the measured joints. The model reproduces the planner's own claw estimate to 0.05 mm.</li>
<li><b>References.</b> The planner's whole-body reference stream.</li>
<li><b>Basket.</b> Its mocap body. At the pick the basket's resting pose before the approach is the frame, because the solved orientation flips when the claw is near.</li>
<li><b>Scoring windows.</b> DIRECT entry to the controller's last tick with live feedback: 28.0–187.2 s, 14.6–144.6 s and 12.4–80.7 s.</li>
<li><b>Hover windows.</b> Spans in which the planner's centre-of-mass and claw references both move slower than 5 mm/s for at least 1.5 s. Settled means without the first 3 s. With the basket: from the lift-off at 47.0 s to its removal at 118.3 s, leaving out 89.5–96.5 s when it rested on the hat's rim.</li>
<li><b>Load gauges.</b> Delivered thrust is the sum of the squared rotor speeds the motor commands ask for, times a thrust constant that follows the battery voltage and the throttle as fitted on the seven stepped-payload hovers of 20 August. Battery power is voltage times current from PX4's battery status. The moment gauge is the controller's rotational disturbance estimate about the pitch axis, converted with the same thrust ratio.</li>
<li><b>Applied arm torque.</b> The servos' measured current divided by the calibrated counts per newton-metre. The arm's own weight is taken out with the arm controller's calibrated gravity model.</li>
<li><b>Not in the bag.</b> The gripper's commands, the PX4 log and the Orin's system log.</li>
<li><b>Files.</b> Tools, numbers and this page's source are in <code>docs/docs_aerial_manipulator/archive/wb_pick_place_flight_20261009/</code>.</li>
</ul>
"""

HEAD = ('<!doctype html>\n<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">\n'
        '<title>1009 Pick-and-Place Flights</title>\n'
        '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">\n')
css = open(os.path.join(HERE, "report_style.css")).read().replace("</style>", EXTRA_CSS + "</style>")
data = "\n".join(f'<script type="application/json" id="data-{k}">{raw("fig_" + f + ".json")}</script>'
                 for k, f in (("3d", "3d"), ("err", "err"), ("hover", "hover"), ("load", "load"), ("views", "views"), ("freeze", "freeze"), ("land", "land")))
js = ('<script src="https://cdn.jsdelivr.net/npm/plotly.js-gl3d-dist-min@2.35.2/plotly-gl3d.min.js"></script>\n'
      "<script>\n" + open(os.path.join(HERE, "figures.js")).read() + "\n</script>")
html = HEAD + css + "\n</head>\n<body>\n" + BODY + "\n" + data + "\n" + js + "\n</body></html>\n"
out = os.path.join(HERE, "..", "report.html")
open(out, "w").write(html)
print("wrote", os.path.abspath(out), len(html) // 1024, "kB")
