#!/usr/bin/env python3
"""Build ../report.html for the 2026-10-09 and 2026-10-10 whole-body pick-and-place hardware flights.

    python3 build_report.py && python3 build_artifact_page.py <out.html>

Tables come from ../analysis/{tracking,grasp,hover,payload_budget}.json, the figures' data from ../analysis/fig_*.json (fig_data.py),
the chart code from figures.js, the style from report_style.css (the 0928 report's) plus the few rules below.
The outline is added by add_outline.py (run by build_artifact_page.py).
PP-4 (2026-10-10): numbers from ../../wb_pick_place_flight_20261010/analysis/{pp4,load_p4,tracking_p4,extra_p4,fig_p4}.json
(that folder's tools), its extra charts from ../../wb_pick_place_flight_20261010/tools/figures10.js.
"""
import base64, json, math, os

HERE = os.path.dirname(os.path.abspath(__file__))
AN = os.path.join(HERE, "..", "analysis")
J = lambda n: json.load(open(os.path.join(AN, n)))
T, GR, HOV, PB = J("tracking.json"), J("grasp.json"), J("hover.json"), J("payload_budget.json")
raw = lambda n: open(os.path.join(AN, n)).read().replace("</", "<\\/")
D10 = os.path.join(HERE, "..", "..", "wb_pick_place_flight_20261010")
J10 = lambda n: json.load(open(os.path.join(D10, "analysis", n)))
A4, L4, T4, X4, F4 = J10("pp4.json"), J10("load_p4.json"), J10("tracking_p4.json"), J10("extra_p4.json"), J10("fig_p4.json")
P4, PL4, SD4, TK4, J24, BR4, ST4 = A4["pick"], A4["place"], A4["setdown"], A4["takeover"], A4["joint2"], A4["basket_rest"], A4["settings"]

EXTRA_CSS = """
.plot{width:100%}.plot.h3d{height:clamp(320px,34vw,400px)}.plot.herr{height:900px}.plot.hpick{height:560px}.plot.hplace{height:560px}
.plot.hhover{height:clamp(300px,42vw,520px)}.plot.hload{height:640px}.plot.hview{height:500px}
.vleg{display:flex;flex-wrap:wrap;gap:4px 16px;align-items:center}.vleg span{white-space:nowrap}
.plot.hmap{height:420px}.plot.hts{height:420px}.plot.hland{height:520px}
.plot.htrim{height:560px}.plot.hsd{height:760px}.plot.hcarry{height:660px}.plot.hhang{height:480px}h5{font-size:.9rem;margin:16px 0 4px;font-weight:600;text-wrap:balance}.plot.hfollow{height:340px}
figure img.photo{filter:none;background:#000;border-color:var(--line)}
.pgrid.three{grid-template-columns:repeat(auto-fit,minmax(min(100%,300px),1fr))}
.ee3d-grid.four{grid-template-columns:repeat(auto-fit,minmax(min(100%,440px),1fr))}
table td.part{color:var(--ink2)}
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
    for r in (T4 if nm == "p4" else T[nm])["phases"]:
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
    return f"{v:.{d}f}".replace("-", "−")


def sg(v, d=0):
    """Signed, with a typographic minus."""
    return f"{v:+.{d}f}".replace("-", "−")


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
HG2, HG4 = J("setpoint_tuning.json")["hang_mocap"], A4["hang"]["mocap"]


def pool4(labels, key="com"):
    """PP-4: duration-weighted rms and the largest value over the phases whose label starts with one of labels."""
    rows = [r for r in T4["phases"] if any(r["label"].startswith(l) for l in labels)]
    w = sum(r["t1"] - r["t0"] for r in rows)
    return math.sqrt(sum((r["t1"] - r["t0"]) * r[key]["rms"] ** 2 for r in rows) / w), max(r[key]["max"] for r in rows)


HOV_U = ["READY", "DONE go_to_start", "WAITING execute_pick", "DONE execute_pick"]
MOV_U = ["FLYING go_to_start", "FLYING execute_pick (approach)", "FLYING execute_pick (descent)"]
HOV_L = ["DONE exit_pick", "DONE go_to_place_start", "WAITING execute_place"]
MOV_L = ["FLYING go_to_place_start", "FLYING execute_place (approach)"]
rm = lambda labels, key="com", d=0: "{:.{d}f} / {:.{d}f}".format(*pool4(labels, key), d=d)


def hang_row(label, k2, k4):
    """Basket centre relative to the claw (mocap): along the heading (+ = ahead of the claw), to the right, tilt."""
    a, b = HG2.get(k2), HG4.get(k4)
    c2 = f'<td class="n">{sg(a["along"][0])}</td><td class="n">{n(-a["left"][0])}</td>' if a else '<td class="n">—</td><td class="n">—</td>'
    c4 = (f'<td class="n">{sg(b["along"][0])}</td><td class="n">{n(-b["left"][0])}</td><td class="n">{b["tilt"][0]:.0f}°</td>' if b
          else '<td class="n">—</td><td class="n">—</td><td class="n">—</td>')
    return f"<tr><td>{label}</td>{c2}{c4}</tr>"


HANG = J10("fig_hang.json")
FW = J10("hang_follow.json")                     # the basket's tilt against the fingers' pitch while it hangs
FL2, FL4 = FW["p2"], FW["p4"]
SH = J10("sim_hang.json")                        # the same measurement on the recorded Isaac missions
SHV = lambda key: [r["stages"][st][key] for r in SH.values() for st in ("carry", "place_start", "above_place", "descent")]
HANG["follow"] = {k: {kk: v[kk] for kk in ("t", "pitch_s", "side_s", "free_hang", "ev")} for k, v in FW.items()}
H2, H4 = HANG["runs"]["p2"]["stages"], HANG["runs"]["p4"]["stages"]


def hang_row2(key, label):
    """Basket centre relative to the claw (mocap), both days: along the heading (+ ahead), to the right, tilt top forward."""
    c = lambda s: f'<td class="n">{sg(s["along"][0])}</td><td class="n">{n(-s["left"][0])}</td><td class="n">{s["tilt_side"]:.0f}°</td>'
    return f"<tr><td>{label}</td>{c(H2[key])}{c(H4[key])}</tr>"


def hang_note(key):
    a, b = H2[key], H4[key]
    w = lambda v: f"{abs(v):.0f} mm {'behind' if v < 0 else 'ahead of'} the claw"
    return (f'<p class="note">Centre {w(a["along"][0])} on 9 October and {w(b["along"][0])} on 10 October. '
            f'Tilt {a["tilt_side"]:.0f}° and {b["tilt_side"]:.0f}°.</p>')


def hang_part(s, h):
    """The three parts of the centre's position along the heading [mm]: arch on the fingers, hanger plane, tilt."""
    th = math.radians(s["tilt_side"])
    return s["arch"][0], -HANG["stem_x"] * math.cos(th), -h * math.sin(th)


def handle_case(lean_deg):
    """A stem of today's length leaning by lean_deg (+ = top toward the vehicle), same base. Returns the arch's
    position behind the centre [mm], the free-hang tilt [deg], and for the PP-2-like and PP-4-like hang: (tilt at the
    touchdown [deg], basket short of the place point with the claw on target [mm]). Assumes the basket settles at the
    free-hang angle, is then turned by the claw as far as in that flight, and the arch sits on the fingers as it did."""
    ell = 167.0 - 32.5; a = math.radians(lean_deg)
    xa = -27.5 - ell * math.sin(a); h = 32.5 + ell * math.cos(a); L = math.hypot(xa, h); th0 = math.degrees(math.atan2(-xa, h))
    res = []
    for s, f in ((H2["descent"], FL2), (H4["descent"], FL4)):
        d = s["tilt_side"] - f["free_hang"]
        res.append((th0 + d, -xa + L * math.sin(math.radians(d)) - (s["arch"][0] + 7.5)))
    return -xa, th0, res


HC0, HCB, HCF = handle_case(0.0), handle_case(15.0), handle_case(-math.degrees(math.asin(27.5 / (167.0 - 32.5))))
HD2, HD4 = H2["descent"]["tilt_side"] - FL2["free_hang"], H4["descent"]["tilt_side"] - FL4["free_hang"]
HP2, HP4 = hang_part(H2["descent"], HANG["runs"]["p2"]["contact"]), hang_part(H4["descent"], HANG["runs"]["p4"]["contact"])
WI = P4["ee_z_what_if"]
wi = lambda z, k, f="mocap": WI[z][f][k]
G4, M4, CP4 = L4["gauges"], L4["moment"], L4["caps"]["with the basket"]
STRIP = base64.b64encode(open(os.path.join(D10, "figures", "setdown_strip.jpg"), "rb").read()).decode()
v3 = lambda a, d=0: ", ".join(n(x, d) for x in a)
pm = lambda m, s: ", ".join(f"{a:.0f} ± {b:.0f}".replace("-", "−") for a, b in zip(m, s))

BODY = f"""
<h1>Hardware Pick-and-Place Flights, 9 and 10 October</h1>
<p class="sub">T650 aerial manipulator, whole-body 4-D L1 controller, EKF2-fused feedback. Four flights: PP-1 at 17:25, PP-2 at 18:02 and PP-3 at 18:18 on 9 October 2026, from the bag
<code>1009 - T650-AM whole-body Pick&amp;Place</code>, and PP-4 at 19:07 on 10 October, from <code>1010 - T650-AM whole-body Pick&amp;Place</code>. Times are seconds from the start of each bag. Updated on 12 October with PP-4.</p>

<div class="verdict warn"><strong>Overall, after the 10 October flight.</strong> The pick works. The place reached the hat for the first time and was lost in the second after the touchdown. The link freeze of 9 October has a known cause and did not come back.
<ul>
<li><b>Pick, PP-4:</b> hooked and lifted, as in PP-2. The gripper closed {abs(P4["claw_vs_planner_target_at_close_mm"]["planner"][0]):.0f} mm from its target, with the fingers {wi("0.16", "slide_gap_min"):.0f} to {wi("0.16", "slide_gap_mean"):.0f} mm under the arch.</li>
<li><b>Place, PP-4:</b> the basket touched down on the hat, {PL4["basket_from_hat_centre_mm"]:.0f} mm from its centre. {SD4["t_vehicle_starts"] - SD4["t_touchdown"]:.1f} s later the vehicle began to move back, {SD4["vehicle_back_max_mm"] / 10:.0f} cm in one second, with the gripper still closed on the stem. It pulled the basket over the rim {SD4["t_tip"] - SD4["t_touchdown"]:.1f} s after the touchdown. The gripper was told to open {SD4["touchdown_to_open_cmd_s"]:.1f} s after the touchdown.</li>
<li><b>Your question 1, the 5 s trim window:</b> it did not help in this flight. Place was pressed {ST4["wait_place"]:.1f} s after arrival, so the window held the whole arrival swing and the descent was moved {X4["place"]["avg_mm"]:.0f} mm the wrong way. Without the trim the claw would have come down {PL4["trim_what_if"]["trim off"]["dist_xy"]:.0f} mm from its target instead of {PL4["trim_what_if"]["as flown, 5 s window pressed at 5.0 s"]["dist_xy"]:.0f} mm (Section 3.2).</li>
<li><b>Your question 2, the offsets:</b> Pick x + 0.01 and EE Offset z = 0.16 worked. EE Offset z is the tightest of them, not a conservative one. Place x − 0.03 went the right way and is about 3 cm too little for the way the basket hung. An EE Offset y of +0.02 is missing (Sections 3.1 and 3.2).</li>
<li><b>Your question 3, what to improve:</b> release the basket at the touchdown, not at the end of the descent. That comes before the platform, the offsets and the payload weight, which all help (Section 5).</li>
<li><b>Link freeze:</b> the MicroXRCEAgent on the Orin had locked itself up under real-time priority. The fix was committed on the Orin 27 minutes before PP-4, which flew 69 s in DIRECT with no gap over {max(L["max_gap_ms"] for k, L in A4["link"].items() if k in ("vatt", "sc")):.0f} ms on the Pixhawk's 400 Hz topics (Section 4.2).</li>
<li><b>End of PP-4:</b> the pilot took STAB at {TK4["z_at_stab"]:.2f} m. The motors stopped at {TK4["z_top"]:.2f} m, the vehicle fell and came to rest tipped at {TK4["rest_tilt"]:.0f}°. It is the third takeover that ended on the floor at an angle (Section 4.1).</li>
</ul></div>

<div class="kv">
<div><b>{PL4["basket_from_hat_centre_mm"]:.0f} mm</b><span>from the hat's centre when the basket touched down in PP-4. On 9 October it came down on the rim</span></div>
<div><b>{SD4["vehicle_back_max_mm"] / 10:.0f} cm</b><span>the vehicle moved back in the second after the touchdown, gripper still closed</span></div>
<div><b>{SD4["touchdown_to_open_cmd_s"]:.1f} s</b><span>from the touchdown to the command that opens the gripper</span></div>
<div><b>{X4["place"]["avg_mm"]:.0f} mm</b><span>the 5 s trim moved the descent, away from the target</span></div>
</div>

<h2>1. The four flights</h2>
<div class="tscroll"><table class="cmp"><thead><tr><th></th><th>PP-1, 9 Oct 17:25</th><th>PP-2, 9 Oct 18:02</th><th>PP-3, 9 Oct 18:18</th><th>PP-4, 10 Oct 19:07</th></tr></thead><tbody>
<tr><td>Flown</td><td>all six legs, status COMPLETE</td><td>pick, carry, place, to land start</td><td>pick only, then Reset and flown home</td><td>pick, carry, place, then the pilot took over</td></tr>
<tr><td>Time in DIRECT</td><td class="n">159 s</td><td class="n">130 s</td><td class="n">68 s</td><td class="n">{T4["extra"]["direct"][1] - T4["extra"]["direct"][0]:.0f} s</td></tr>
<tr><td>Payload on the claw</td><td>none. No step in thrust or arm torque at the lift</td><td>the basket, 190 g on the scale, from 47 s to 118 s</td><td>none. The gripper closed above the arch</td><td>the basket with the new handle, from {P4["t_liftoff"]:.1f} s to {SD4["t_tip"]:.1f} s. Not weighed since (Section 4.1)</td></tr>
<tr><td>Pick</td><td>closed 12 mm from the target, nothing lifted</td><td class="ok">hooked and lifted</td><td class="no">missed, EE Offset z was 0.20</td><td class="ok">hooked and lifted</td></tr>
<tr><td>Place</td><td>opened 65 mm from the target</td><td class="no">opened 140 mm from the target, basket not released</td><td>not flown</td><td class="no">touched down on the hat, pulled off it {SD4["t_tip"] - SD4["t_touchdown"]:.1f} s later</td></tr>
<tr><td>End of the flight</td><td>descent to 0.35 m in DIRECT, pilot took STAB at 0.40 m, tipped 45° nose down</td><td class="no">link freeze in hover at 1.0 m, 2.6 m uncontrolled, pilot landed it</td><td>descent to 0.35 m in DIRECT, pilot took STAB at 0.44 m, tipped 57° nose up</td><td class="no">pilot took STAB at {TK4["z_at_stab"]:.2f} m, motors stopped at {TK4["z_top"]:.2f} m, fell, tipped {TK4["rest_tilt"]:.0f}°</td></tr>
<tr><td>Basket stream (<code>obj_0</code>)</td><td class="no">dead for the whole flight</td><td>live, orientation flips near the claw</td><td>live, orientation flips near the claw</td><td>live, no flip at the pick</td></tr>
</tbody></table></div>
<p>In PP-1 the thrust and the arm torques show no load at any time, and the basket's last streamed pose is 4.0 m from the pick hat. The flight is used here as the unloaded reference for the controller. PP-2 and PP-4 are the two flights with the basket on the claw.</p>

<h3>1.1 Settings each flight used</h3>
<p>Recovered from the planner's <code>pick_place/info</code> topic and its log lines. Positions in metres, mocap frame.</p>
<div class="tscroll"><table class="cmp"><thead><tr><th></th><th>PP-1</th><th>PP-2</th><th>PP-3</th><th>PP-4</th></tr></thead><tbody>
<tr><td>Pick point (x, y, z, yaw)</td><td class="n">0.977, 1.034, 0.871, 0.6°</td><td class="n">1.007, 1.018, 0.864, 1.0°</td><td class="n">1.014, 1.022, 0.865, 1.3°</td><td class="n">{ST4["pick_point"][0]:.3f}, {ST4["pick_point"][1]:.3f}, {ST4["pick_point"][2]:.3f}, {ST4["pick_yaw"]:.1f}°</td></tr>
<tr><td>How the pick point was set</td><td>Get</td><td>Get</td><td>Get</td><td>Get, then x + 0.01</td></tr>
<tr><td>EE Offset (x, y, z)</td><td class="n">−0.02, 0, 0.16</td><td class="n">−0.02, 0, 0.16</td><td class="n">−0.02, 0, 0.20</td><td class="n">−0.02, 0, 0.16</td></tr>
<tr><td>Place point (x, y, z, yaw)</td><td class="n">−1.06, −1.02, 0.86, −180°</td><td class="n">−1.03, −1.03, 0.86, −180°</td><td class="n">−1.03, −1.08, 0.86, −180°</td><td class="n">{n(ST4["place_point"][0], 2)}, {n(ST4["place_point"][1], 3)}, {ST4["place_point"][2]:.2f}, −180°</td></tr>
<tr><td>How the place point was set</td><td>typed</td><td>typed</td><td>typed</td><td>Get, then x − 0.03 and z − 0.01</td></tr>
<tr><td>Vertical Margin / Side Margin</td><td class="n">0.20 / 0.10</td><td class="n">0.20 / 0.10</td><td class="n">0.20 / 0.10</td><td class="n">0.20 / 0.10</td></tr>
<tr><td>Start point after Adjust (x, y)</td><td class="n">0.018, 0.017</td><td class="n">0.056, 0.020</td><td class="n">0.032, 0.013</td><td class="n">{n(ST4["start_xy"][0], 3)}, {n(ST4["start_xy"][1], 3)}</td></tr>
<tr><td>Descent trim window</td><td class="n">1 s</td><td class="n">1 s</td><td class="n">1 s</td><td class="n">5 s</td></tr>
<tr><td>Wait before Pick was pressed</td><td class="n">39.0 s</td><td class="n">4.2 s</td><td class="n">1.6 s</td><td class="n">{ST4["wait_pick"]:.1f} s</td></tr>
<tr><td>Wait before Place was pressed</td><td class="n">1.6 s</td><td class="n">0.8 s</td><td class="n">not flown</td><td class="n">{ST4["wait_place"]:.1f} s</td></tr>
<tr><td>Planner's trim at the pick (x, y, z)</td><td class="n">+17, +6, +3 mm</td><td class="n">+9, +2, −1 mm</td><td class="n">−14, −5, −4 mm</td><td>none, the press came before half the window had filled</td></tr>
<tr><td>Planner's trim at the place (x, y, z)</td><td class="n">+36, +22, −2 mm</td><td class="n">+55, −8, −13 mm</td><td class="n">not flown</td><td class="n">{sg(ST4["trim_mm"]["place"][0])}, {sg(ST4["trim_mm"]["place"][1])}, {n(ST4["trim_mm"]["place"][2])} mm</td></tr>
<tr><td>MicroXRCEAgent on the Orin</td><td>real-time priority</td><td>real-time priority</td><td>real-time priority</td><td>ordinary priority, since 18:40</td></tr>
</tbody></table></div>
<ul>
<li><b>How the 10 October points were read.</b> On the Pick row x has two decimals and y and z have four. On the Place row x and z have two and y has four. The two-decimal numbers were typed and the others came from Get. With what you described, that is Pick = Get x + 0.01 and Place = Get x − 0.03, Get z − 0.01. The pick point is 6 mm ahead of where the basket stood, which fits a Get good to 4 mm plus the 0.01.</li>
<li><b>Where the place hat stood.</b> The Get on the Place row was therefore at ({n(PL4["hat_centre_assumed"][0], 2)}, {n(PL4["hat_centre_assumed"][1], 3)}) with the basket's centre at 0.88 m. This page takes that as the centre of the place hat. In the flight the basket lay flat on that hat with its centre at {PL4["basket_flat_on_hat"]["z"]:.3f} m, which agrees, and the video shows it touching down between the hat's centre and its near rim.</li>
<li><b>The pillars had moved since 9 October.</b> The basket rested {math.hypot(BR4["early"][0] - 1.0091, BR4["early"][1] - 1.0197) * 100:.0f} cm from its 9 October position on the pick hat and {(BR4["early"][2] - 0.8647) * 1e3:.0f} mm higher.</li>
</ul>

<h2>2. Whole-body controller overall performance</h2>
<div class="verdict good"><strong>Accurate without a payload. With this payload it flies stably but not accurately enough to place.</strong>
Unloaded, the claw hovers within {hp("none","rms")} mm rms of its target and the airframe tracks the moving legs to 39 mm rms.
With the 190 g basket, which is not in the controller's model, the hover error doubles to {hp("payload","rms")} mm rms and the lift costs a 25 cm excursion.
PP-4 on 10 October confirms this and adds two things: the arm's commands reached its torque caps (Section 4.2), and joint 2 stuck and slipped under the load (Section 4.1).</div>

<h3>2.1 The flight path in 3-D</h3>
<figure>
<div class="ee3d">
 <div class="ee3d-bar">
  <span class="tog" id="f3d-tog">
   <button type="button" aria-pressed="true" data-k="ref"><span class="sw dash" style="--c:#8f8e84"></span>planned claw path</button>
   <button type="button" aria-pressed="true" data-k="base"><span class="sw" style="--c:#3987e5"></span>airframe</button>
   <button type="button" aria-pressed="true" data-k="claw"><span class="sw" style="--c:#d95926"></span>claw</button>
   <button type="button" aria-pressed="true" data-k="basket"><span class="sw big" style="--c:#199e70"></span>basket</button>
   <button type="button" aria-pressed="true" data-k="fly"><span class="sw big" style="--c:#e66767"></span>after the controller's flight ended</button>
  </span>
  <button type="button" class="plain" id="f3d-top">Top view</button>
  <button type="button" class="plain" id="f3d-reset">Reset view</button>
 </div>
 <div class="ee3d-grid four">
  <div class="ee3d-cell"><p class="ee3d-title">PP-1, full mission without payload</p><div class="plot h3d" id="f3d-p1"></div></div>
  <div class="ee3d-cell"><p class="ee3d-title">PP-2, pick, carry and place attempt</p><div class="plot h3d" id="f3d-p2"></div></div>
  <div class="ee3d-cell"><p class="ee3d-title">PP-3, pick attempt, then flown home</p><div class="plot h3d" id="f3d-p3"></div></div>
  <div class="ee3d-cell"><p class="ee3d-title">PP-4, 10 October: pick, carry, place, takeover</p><div class="plot h3d" id="f3d-p4"></div></div>
 </div>
 <p class="note plot-missing" hidden>The interactive charts need the Plotly library, which did not load. The tables below carry the same numbers.</p>
</div>
<figcaption><b>Fig. 1.</b> Each flight from DIRECT entry to the end of the controller's flight. Airframe, claw and basket are mocap positions, the claw by the planner's kinematics on the mocap pose and the measured joints. The dashed line is the claw path the planner streamed. The grey posts are the two pillars. The red track is the vehicle after the controller's flight ended: after the link froze in PP-2, and after the pilot took over in PP-4. Drag to rotate, scroll to zoom, hover for time and position.</figcaption>
</figure>

<h3>2.2 Tracking errors: curves and tables for all four flights</h3>
<figure>
<div class="ee3d">
 <div class="ee3d-bar"><span class="seg" id="ferr-pick">
  <button type="button" aria-pressed="true" data-k="p1">PP-1</button><button type="button" aria-pressed="false" data-k="p2">PP-2</button><button type="button" aria-pressed="false" data-k="p3">PP-3</button><button type="button" aria-pressed="false" data-k="p4">PP-4</button></span>
  <span class="note">Shaded bands are the moving legs. Hover for all values at one time.</span></div>
 <div class="plot herr" id="ferr"></div>
</div>
<figcaption><b>Fig. 2.</b> Airframe error is the system centre of mass minus its reference. Claw vs airframe is the controller's own task error, the claw's offset from the centre of mass against the planned offset. Angles: claw heading error, airframe yaw error and tilt. Thrust and the joint 2 and 3 torques are the controller's commands. In PP-2 the thrust command steps up by 3.7 N at 47 s when the basket lifts and back down at 118 s when it is removed. Section 4.1 explains why that is more than the basket's weight. In PP-4 the step is {L4["gauges"]["above_place"]["d_u1"]:.1f} N at {P4["t_liftoff"]:.1f} s, and the airframe error swings by ±0.15 m at 48 to 53 s while joint 2 sticks and slips (Section 4.1).</figcaption>
</figure>

<h4>All four flights</h4>
<div class="tscroll"><table class="cmp"><thead><tr><th></th><th>PP-1<br>no payload</th><th>PP-2<br>before the lift</th><th>PP-2<br>with the basket</th><th>PP-3<br>no payload</th><th>PP-4<br>before the lift</th><th>PP-4<br>with the basket</th></tr></thead><tbody>
<tr><td>Airframe error in hover, rms / max [mm]</td><td class="n">28 / 69</td><td class="n">15 / 32</td><td class="n">63 / 164</td><td class="n">15 / 30</td><td class="n">{rm(HOV_U)}</td><td class="n">{rm(HOV_L)}</td></tr>
<tr><td>Airframe error on moving legs, rms / max [mm]</td><td class="n">39 / 105</td><td class="n">16 / 25</td><td class="n">55 / 116</td><td class="n">18 / 30</td><td class="n">{rm(MOV_U)}</td><td class="n">{rm(MOV_L)}</td></tr>
<tr><td>Airframe error at the lift, max [mm]</td><td class="n">42</td><td class="n">—</td><td class="n">281</td><td class="n">23</td><td class="n">—</td><td class="n">{A4["lift"]["com_err_max_mm"]:.0f}</td></tr>
<tr><td>Claw vs airframe, rms / max [mm]</td><td class="n">2.2 / 7.6</td><td class="n">2.3 / 6.5</td><td class="n">14 / 46</td><td class="n">2.9 / 8.6</td><td class="n">{rm(HOV_U + MOV_U, "task", 1)}</td><td class="n">{rm(HOV_L + MOV_L, "task", 0)}</td></tr>
<tr><td>Claw heading error, rms / max [°]</td><td class="n">0.24 / 1.2</td><td class="n">0.23 / 0.9</td><td class="n">8.9 / 23.6</td><td class="n">0.29 / 1.0</td><td class="n">{rm(HOV_U + MOV_U, "e_head", 2)}</td><td class="n">{rm(HOV_L + MOV_L, "e_head", 1)}</td></tr>
<tr><td>Airframe yaw error, rms / max [°]</td><td class="n">0.45 / 3.8</td><td class="n" colspan="2">0.61 / 3.8</td><td class="n">0.62 / 3.5</td><td class="n">{T4["totals"]["before the lift (no payload)"]["e_yaw"]["rms"]:.2f} / {T4["totals"]["before the lift (no payload)"]["e_yaw"]["max"]:.1f}</td><td class="n">{T4["totals"]["carry and place (payload on the claw)"]["e_yaw"]["rms"]:.2f} / {T4["totals"]["carry and place (payload on the claw)"]["e_yaw"]["max"]:.1f}</td></tr>
<tr><td>Tilt, max [°]</td><td class="n">2.5</td><td class="n">2.1</td><td class="n">6.6</td><td class="n">2.1</td><td class="n">{max(r["tilt_max"] for r in T4["phases"] if any(r["label"].startswith(l) for l in HOV_U + MOV_U)):.1f}</td><td class="n">{max(A4["lift"]["tilt_max"], T4["totals"]["carry and place (payload on the claw)"]["tilt_max"]):.1f}</td></tr>
<tr><td>Motor commands, min – max</td><td class="n">0.42 – 0.78</td><td class="n" colspan="2">0.38 – 0.86</td><td class="n">0.40 – 0.74</td><td class="n">{T4["totals"]["before the lift (no payload)"]["motor_min"]:.2f} – {T4["totals"]["before the lift (no payload)"]["motor_max"]:.2f}</td><td class="n">{T4["totals"]["carry and place (payload on the claw)"]["motor_min"]:.2f} – {T4["totals"]["carry and place (payload on the claw)"]["motor_max"]:.2f}</td></tr>
<tr><td>Control ticks with a saturated motor / a joint at the controller's own 3 N·m limit</td><td class="n">0 / 0 of 39 766</td><td class="n" colspan="2">0 / 0 of 32 498</td><td class="n">0 / 0 of 17 068</td><td class="n" colspan="2">0 / 0 of {f"{T4['extra']['ticks']:,}".replace(",", " ")}</td></tr>
<tr><td>Joint 2 / joint 3 commanded torque, max [N·m]</td><td class="n">1.05 / 0.46</td><td class="n">0.84 / 0.31</td><td class="n">2.11 / 1.22</td><td class="n">0.94 / 0.49</td><td class="n">{T4["totals"]["before the lift (no payload)"]["tau_max"][1]:.2f} / {T4["totals"]["before the lift (no payload)"]["tau_max"][2]:.2f}</td><td class="n">{J24["law_tau_max"][1]:.2f} / {J24["law_tau_max"][2]:.2f}</td></tr>
<tr><td>Arm commands cut at the arm controller's caps, joint 2 / joint 3 [s]</td><td class="n">0 / 0</td><td class="n">0 / 0</td><td class="n">0 / 0</td><td class="n">0 / 0</td><td class="n">0 / 0</td><td class="n">{CP4["s_clipped"][1]:.2f} / {CP4["s_clipped"][2]:.2f}</td></tr>
<tr><td>Joint tracking error j1–j4, rms [°]</td><td class="n">0.5, 2.2, 1.3, 0.1</td><td class="n" colspan="2">0.6, 2.9, 2.0, 16.9</td><td class="n">0.7, 1.8, 1.3, 0.1</td><td class="n">{", ".join(f"{v:.1f}" for v in T4["totals"]["before the lift (no payload)"]["e_q"]["rms"])}</td><td class="n">{", ".join(f"{v:.1f}" for v in T4["totals"]["carry and place (payload on the claw)"]["e_q"]["rms"])}</td></tr>
</tbody></table></div>
<ul>
<li><b>Without a payload the four flights agree.</b> Hover 15 to 28 mm rms, moving legs 16 to 39 mm, the claw within 2 to 3 mm of its place on the airframe, no motor saturated.</li>
<li><b>With the basket the two flights agree too.</b> Hover {rm(HOV_L).split(" / ")[0]} mm rms in PP-4 against 63 in PP-2, {A4["lift"]["com_err_max_mm"]:.0f} against 281 mm at the lift, and the arm's torque commands twice as large.</li>
<li><b>The yaw error's maximum</b> is the first three seconds after DIRECT entry in every flight.</li>
<li><b>The 2° on joints 2 and 3</b> is the standing hold offset seen in earlier flights. The 16.9° on joint 4 in PP-2 is the hooked basket turning the wrist (Section 4.2).</li>
<li><b>What the hover rows count.</b> Every sample between two legs, including the seconds after each arrival and the lift. The settled hover alone is further down. PP-4's hover and moving rows leave out the first four seconds after DIRECT entry, when the airframe was up to 119 mm and the yaw 4.8° from their references. Its bag starts with the vehicle already in the air.</li>
<li><b>For scale:</b> the 09-28 circle flights measured 24–26 mm rms at the claw, and the 10-05 figure-8 flights 37–58 mm.</li>
</ul>

<h4>Leg by leg</h4>
<h5>PP-1</h5>
{leg_table("p1", "PP-1")}
<h5>PP-2</h5>
{leg_table("p2", "PP-2")}
<h5>PP-4</h5>
{leg_table("p4", "PP-4")}
<p>The descent row runs to the end of the leg and so includes the second after the touchdown (Section 3.2). Up to the touchdown its airframe error was {X4["descent_before_td"]["com_max"]:.0f} mm at most.</p>

<h4>Settled hover, without and with the payload</h4>
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
<tr><td>Height error, mean ± std [mm]</td><td class="n">{sg(HP["none"]["ee"]["z_mean"])} ± {hp("none","z_std")}</td><td class="n">{sg(HP["payload"]["ee"]["z_mean"])} ± {hp("payload","z_std")}</td></tr>
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
<h5>Hover RMSE at the pick and at the place</h5>
<p>Whole hovers at the two sites, arrival seconds included, because that is when the buttons were pressed. Along and across refer to the approach direction, which is world x at both sites.</p>
<div class="tscroll"><table><thead><tr><th>Hover</th><th>Data, hovers</th><th>Claw horizontal [mm]</th><th>along [mm]</th><th>across [mm]</th><th>Claw height [mm]</th><th>Airframe horizontal [mm]</th><th>Airframe height [mm]</th></tr></thead><tbody>
{pp_row("Pick, no payload, three flights<br><span class='note'>behind the stem and on the handle, until the lift starts</span>", lambda r: r["grp"] == "pick")}
{pp_row("Place, with the basket, PP-2<br><span class='note'>above the place before Place, and after Exit To Place with the basket still hooked</span>", lambda r: r["grp"] == "place" and r["state"] == "payload")}
{pp_row("Place, no payload, PP-1<br><span class='note'>above the place before Place, and after Exit To Place</span>", lambda r: r["grp"] == "place" and r["run"] == "PP-1")}
</tbody></table></div>
<p>RMSE is the root mean square of measured minus reference, so it holds both the standing offset and the wander. The claw's height error with the basket is mostly its 7 mm sag under the load.</p>
<h5>What this means for the pick and the place</h5>
<ul>
<li><b>Pick, no payload.</b> The gripper closes when the claw is within 20 mm of its target and within 3 mm across the stem. The unloaded claw is inside 20 mm for {pct("none","within20")} % of the time, so the close comes after a wait of a few seconds. All three picks closed 3 to 12 mm from the target. The hover is good enough for the pick.</li>
<li><b>Place, with the basket.</b> The basket is 110 × 115 mm and the hat is 200 mm across. All four corners are on the hat only when the basket's centre is within 20 mm of the hat's centre. Beyond roughly 70 mm the basket tips off. The loaded claw is within 50 mm for {pct("payload","within50")} % of the time and within 70 mm for {pct("payload","within70")} %. On top of that the basket hangs 20 to 30 mm away from where the planner assumes (Section 3.2). So the place is the harder half, as you expected, and the loaded hover alone uses most of the hat.</li>
<li><b>Arrival is worse than the settled hover.</b> In the first 2 to 3 s above the place the offset was 36 mm without the basket and 47 mm with it. Both Place commands were given in that time.</li>
<li><b>Height is not the limit.</b> It holds to ±5 mm without and ±6 mm with the basket. The claw sags 7 mm under the load.</li>
<li><b>What the loaded error looks like.</b> The basket's weight acts 0.25 m ahead of the centre of mass, which is 0.45 N·m nose down (Section 4.1), and the attitude error rose from 0.02 rad rms before the lift to 0.06–0.10 rad with the basket. The standing offset was the same in all three loaded hovers: about 25 mm toward world +x and 10 mm toward +y, at headings of 0° and 180°. It is fixed to the room, not to the vehicle. I do not know its cause. An earlier version of this page tied it to the standing attitude error. The sizes match but the signs do not, so that explanation is withdrawn. On 10 October the settled loaded hover above the place sat ({sg(A4["hover"]["pooled"]["payload"]["mean_xy"][0])}, {sg(A4["hover"]["pooled"]["payload"]["mean_xy"][1])}) mm from its target, the other way, so the offset is not fixed to the room either. It looks like the slow part of the wander.</li>
<li><b>How much data this is.</b> The loaded numbers come from one flight and 23 s of settled hover. Treat them as a first measurement.</li>
</ul>
<h5>PP-4, 10 October</h5>
<ul>
<li><b>Settled, it was better than on 9 October.</b> In the last two seconds above the place the claw was {A4["hover"]["pooled"]["payload"]["rms"]:.0f} mm rms from its target: an offset of {A4["hover"]["pooled"]["payload"]["offset_rms"]:.0f} mm and {A4["hover"]["pooled"]["payload"]["wander"]:.0f} mm of wander, inside {A4["hover"]["pooled"]["payload"]["p95"]:.0f} mm for 95 % of the time. PP-2's loaded hovers gave 45 mm rms. It is {A4["hover"]["pooled"]["payload"]["seconds"]:.0f} s of data.</li>
<li><b>Arrivals were worse.</b> At the place start the claw was 100 mm rms from its target for three seconds and 186 mm at worst. Above the place it overshot by {PL4["overshoot_max_mm"]:.0f} mm one second after arriving.</li>
</ul>

<h2>3. Pick-and-place precision</h2>
<div class="verdict warn"><strong>On 10 October the targets put the basket on the hat. What lost it was the second after the touchdown, not the typed points.</strong>
The pick worked on both days with the same EE Offset. The place came down {PL4["basket_from_hat_centre_mm"]:.0f} mm from the hat's centre, where on 9 October it came down on the rim. The vehicle then moved back {SD4["vehicle_back_max_mm"] / 10:.0f} cm with the gripper still closed on the stem. Section 3.1 covers the pick and its offsets. Section 3.2 covers the place: how the basket hangs, the touchdown, the trim window and the place offsets.</div>
<div class="tscroll"><table class="cmp"><thead><tr><th>The two flights with the basket</th><th>PP-2, 9 October</th><th>PP-4, 10 October</th></tr></thead><tbody>
<tr><td>Pick point</td><td>Get</td><td>Get x + 0.01</td></tr>
<tr><td>Claw from its target when the gripper was told to close [mm]</td><td class="n">1</td><td class="n">{abs(P4["claw_vs_planner_target_at_close_mm"]["planner"][0]):.0f}</td></tr>
<tr><td>Smallest gap between the fingers and the arch in the slide-in [mm]</td><td class="n">6</td><td class="n">{wi("0.16", "slide_gap_min"):.0f}</td></tr>
<tr><td>Airframe pushed forward at the lift [mm]</td><td class="n">267</td><td class="n">231</td></tr>
<tr><td>Wait before Place was pressed</td><td class="n">0.8 s</td><td class="n">{ST4["wait_place"]:.1f} s</td></tr>
<tr><td>Trim window, and the shift it gave toward the vehicle</td><td class="n">1 s, 55 mm</td><td class="n">5 s, {PL4["short_parts_mm"]["trim"]:.0f} mm</td></tr>
<tr><td>Claw behind its typed target at the touchdown [mm]</td><td class="n">129</td><td class="n">{PL4["short_parts_mm"]["trim"] + PL4["short_parts_mm"]["tracking"]:.0f}</td></tr>
<tr><td>Basket short of the typed place point / to the side [mm]</td><td class="n">164 / 25</td><td class="n">{PL4["short_total_mm"]:.0f} / {abs(PL4["side_mm"]):.0f}</td></tr>
<tr><td>Basket's centre from the hat's centre at the touchdown</td><td>not measured. It came down on the rim</td><td>{PL4["basket_from_hat_centre_mm"]:.0f} mm, which is {F4["setdown"]["rim"]:.0f} mm inside the rim on the vehicle's side</td></tr>
<tr><td>Did the hat take the basket's weight?</td><td class="no">no. It stayed tilted at 15 to 18°, half on the hook</td><td class="ok">yes. It lay flat, tilt {PL4["basket_flat_on_hat"]["tilt"]:.0f}°</td></tr>
<tr><td>What happened next</td><td>The gripper opened 140 mm from its target. The basket slid off the rim and stayed on the open fingers.</td><td>The vehicle moved back {SD4["vehicle_back_max_mm"] / 10:.0f} cm. The basket was pulled over the rim {SD4["t_tip"] - SD4["t_touchdown"]:.1f} s after the touchdown and fell when the gripper opened.</td></tr>
</tbody></table></div>

<h3>3.1 Pick</h3>
<h4>The pick, from above and from the side</h4>
<figure>
<div class="ee3d">
 <div class="ee3d-bar"><span class="seg" id="fv-pick-run"><button type="button" aria-pressed="true" data-k="p4">PP-4, basket picked</button><button type="button" aria-pressed="false" data-k="p2">PP-2, basket picked</button><button type="button" aria-pressed="false" data-k="p1">PP-1, no basket</button><button type="button" aria-pressed="false" data-k="p3">PP-3, missed</button></span>
  <span class="note vleg"><span><span class="sw" style="--c:#3987e5"></span>Ready To Pick</span><span><span class="sw" style="--c:#d95926"></span>Pick</span><span><span class="sw" style="--c:#199e70"></span>Exit To Pick</span><span>thick line: claw</span><span><span class="sw dot" style="--c:#b9b8ae"></span>basket centre</span><span><span class="sw dash thin" style="--c:#8f8e84"></span>planned claw path</span><span>✕ claw target</span></span></div>
 <div class="pgrid"><div><p class="ee3d-title">Top view</p><div class="plot hview" id="fv-pick-top"></div></div><div><p class="ee3d-title">Side view</p><div class="plot hview" id="fv-pick-side"></div></div></div>
</div>
<figcaption><b>Fig. 4.</b> The three stages of the pick. The origin is the centre of the pick hat, on its top surface, taken as the place where the basket rested when the claw slid in. The horizontal axis is the direction the claw slides in along, which is world +x. Left: seen from above, with the hat's 100 mm radius, the basket and its hanger. Right: seen from the side, with heights above the hat. The claw is its grasp point, from the vehicle's mocap pose and the measured joints, so it is in the same frame as the basket. The thick lines are the claw and the dotted lines of the same colour are the basket's centre, which hangs 0.17 m under the claw once it is lifted. The grey dashes are the claw path the planner asked for. Filled markers are events on the claw's track and open markers the same instants on the basket's. The dotted amber line in the side view is the claw height at which the fingers meet the arch. <b>PP-4:</b> the dotted amber outline in the top view is where the basket stood until the vehicle arrived. The rotor wash moved it 21 mm and turned it 7.5°, so the claw target, typed for the earlier position, sits 18 mm to the side of the basket's centre. The claw waited 0.10 m behind the stem, slid in under the arch, closed and lifted. The basket again left the hat on a slope, pushed forward with the vehicle. <b>PP-2:</b> the same sequence on 9 October. <b>PP-1</b> flew the path with no basket there, and its hat is drawn at the captured pick point. <b>PP-3:</b> with EE Offset z = 0.20 the claw came in 4 cm higher and closed above the arch.</figcaption>
</figure>
<div class="tscroll"><table class="cmp"><thead><tr><th>Claw in the basket frame (x, y, z) [mm]</th><th>PP-4, planner</th><th>PP-4, mocap</th><th>PP-2, planner</th><th>PP-2, mocap</th></tr></thead><tbody>
<tr><td>Hover behind the stem, mean ± std.<br><span class="note">Target −120, 0, 160. In PP-4 it was {n(P4["target_in_frame"][0] - 100)}, {sg(P4["target_in_frame"][1])}, {n(P4["target_in_frame"][2])}, because the basket had moved</span></td><td class="n">{pm(P4["wait"]["planner"]["mean"], P4["wait"]["planner"]["std"])}</td><td class="n">{pm(P4["wait"]["mocap"]["mean"], P4["wait"]["mocap"]["std"])}</td><td class="n">{pm(p2["pick"]["ready_hold_planner"]["mean"], p2["pick"]["ready_hold_planner"]["std"])}</td><td class="n">{pm(p2["pick"]["ready_hold_mocap"]["mean"], p2["pick"]["ready_hold_mocap"]["std"])}</td></tr>
<tr><td>On the handle, mean ± std.<br><span class="note">Target −20, 0, 160. In PP-4: {n(P4["target_in_frame"][0])}, {sg(P4["target_in_frame"][1])}, {n(P4["target_in_frame"][2])}</span></td><td class="n">{pm(P4["on_handle"]["planner"]["mean"], P4["on_handle"]["planner"]["std"])}</td><td class="n">{pm(P4["on_handle"]["mocap"]["mean"], P4["on_handle"]["mocap"]["std"])}</td><td class="n">{pm(p2["pick"]["on_handle_planner"]["mean"], p2["pick"]["on_handle_planner"]["std"])}</td><td class="n">{pm(p2["pick"]["on_handle_mocap"]["mean"], p2["pick"]["on_handle_mocap"]["std"])}</td></tr>
<tr><td>When the gripper was told to close</td><td class="n">{v3(P4["at_close"]["planner"])}</td><td class="n">{v3(P4["at_close"]["mocap"])}</td><td class="n">{v3(p2["pick"]["at_close_planner"])}</td><td class="n">{v3(p2["pick"]["at_close_mocap"])}</td></tr>
<tr><td>Claw height when the basket left the hat</td><td class="n">{n(P4["contact_height"]["planner"])}</td><td class="n">{n(P4["contact_height"]["mocap"])}</td><td class="n">{n(p2["pick"]["claw_z_at_liftoff_planner"])}</td><td class="n">{n(p2["pick"]["claw_z_at_liftoff_mocap"])}</td></tr>
<tr><td>Time on the handle before the lift</td><td class="n" colspan="2">{n(P4["on_handle_s"], 1)} s</td><td class="n" colspan="2">{n(p2["pick"]["on_handle_s"], 1)} s</td></tr>
</tbody></table></div>
<p>The frame is the basket's: x is the direction the claw slides in along, y is along the arch, z is up. The planner columns use the fused odometry and PX4's attitude, which is what the planner and the arm ground station gate on. On 9 October the two columns differed by 4 to 10 mm in height. On 10 October they agree to {abs(A4["attitude_offset"]["claw_z_planner_minus_mocap_mm"]["pick"]):.0f} mm at the pick (Section 4.1).</p>
<h5>PP-4, 10 October</h5>
<ul>
<li><b>The rotor wash moved the basket before the grasp.</b> Between {BR4["t_move"][0]:.1f} and {BR4["t_move"][1]:.1f} s, as the vehicle arrived {BR4["horiz_dist_vehicle"]:.2f} m away and {BR4["height_above_basket"]:.2f} m higher, the basket slid {BR4["moved_xy_mm"]:.0f} mm across the hat and turned by {abs(BR4["yaw_change"]):.1f}°. In the hover behind the stem it sits under the edge of the front rotor discs. Before that it had shifted by up to 4 mm while the vehicle was still more than a metre away. The pick point had been captured earlier, so it was {abs(BR4["pick_point_minus_at_grasp_mm"][1]):.0f} mm to the side of the basket when the claw slid in.</li>
<li><b>Get and the +0.01 shift did their job.</b> The pick point was ({sg(BR4["pick_point_minus_early_mm"][0], 1)}, {sg(BR4["pick_point_minus_early_mm"][1], 1)}, {sg(BR4["pick_point_minus_early_mm"][2], 1)}) mm from the basket's pose in the first seconds of the bag: Get to 4 mm, plus the 10 mm typed. The claw stopped {abs(P4["claw_vs_planner_target_at_close_mm"]["planner"][0]):.0f} mm short of its target, as it tends to, and the gripper closed with the fingers {abs(P4["claw_vs_stem_at_close_mm"]["mocap"][0]):.0f} mm from the hanger plane.</li>
<li><b>EE Offset z = 0.16 left 4 to 8 mm under the arch.</b> The basket left the hat when the claw reached {P4["contact_height"]["mocap"]:.0f} mm above the basket's centre. During the slide-in the fingers passed {wi("0.16", "slide_gap_mean"):.0f} mm under that height on average and {wi("0.16", "slide_gap_min"):.0f} mm at the least. On the handle the gap was {wi("0.16", "wait_gap_mean"):.0f} mm.</li>
<li><b>The jaws centre on a point 20 mm to the left of the basket body's origin.</b> This is new. At the close the claw sat {P4["claw_vs_stem_at_close_mm"]["mocap"][1]:.0f} mm to the left of the basket's origin, by mocap, and the closing jaws moved the basket by only {abs(P4["basket_dragged_by_close_mm"][0]):.0f} mm. While the basket hung, its origin stayed 15 to 24 mm to the right of the claw (Section 3.2). On 9 October the claw closed centred on the origin, the jaws pushed the basket 30 mm sideways, and it then hung 11 to 20 mm to the right. Both days agree: the stem is about 20 mm to the left of where <code>obj_0</code> puts the basket's centre, seen along the approach. On 10 October the push from the rotor wash happened to cancel it.</li>
<li><b>The lift pushed the vehicle forward by 23 cm,</b> with {A4["lift"]["tilt_max"]:.1f}° of tilt, and joint 2 gave way by {abs(A4["lift"]["q_err_min"][1]):.0f}°. The basket had moved {abs(A4["lift"]["basket_at_14mm_up"][0]):.0f} mm forward and {abs(A4["lift"]["basket_at_14mm_up"][1]):.0f} mm sideways by the time it was 14 mm up. It left the hat on a slope, as on 9 October.</li>
</ul>
<h5>PP-2 and PP-3, 9 October</h5>
<ul>
<li><b>Get is accurate.</b> The capture differed from the basket's resting pose by (−2.3, −1.9, −0.4) mm in PP-2 and (−2.7, −2.6, −0.6) mm in PP-3, and by 0.03° in yaw.</li>
<li><b>EE Offset x is right.</b> At the close in PP-2 the claw sat 21 mm behind the basket centre in the planner's frame and 27 mm by mocap. The hanger plane is 27.5 mm behind the basket centre in the CAD file.</li>
<li><b>EE Offset z = 0.16 is close to the limit.</b> The basket lifted off when the claw reached 167 mm in the planner's frame, so the fingers passed 7 mm under the arch. While the claw waited on the handle its height ranged from 152 to 167 mm.</li>
<li><b>EE Offset z = 0.20 is above the arch.</b> In PP-3 the claw waited on the handle at ({pm(p3["pick"]["on_handle_planner"]["mean"], p3["pick"]["on_handle_planner"]["std"])}) mm in the planner's frame, for a target of (−20, 0, 200), and the gripper closed at ({v3(p3["pick"]["at_close_planner"])}) mm. That is 25 mm above the height where the fingers meet the arch, and the basket never moved.</li>
<li><b>The close pushed the basket.</b> In PP-2 the gripper was told to close with the claw within 1 mm of the basket's origin across the stem, and the jaws then moved the basket 30 mm across the hat. An earlier version of this page put that down to the claw drifting during the close. With PP-4 the simpler reading is the 20 mm between the stem and the body's origin.</li>
</ul>
<p>What limits the pick is the hover, not the setpoint. In PP-1's 39 s wait behind the stem the claw's error had a mean of 4 mm, a standard deviation of 15, 16 and 3 mm in x, y and z, a peak-to-peak of 67, 79 and 14 mm and a period of 6 to 7 s.</p>

<h4>Were the pick offsets right?</h4>
<div class="verdict"><strong>Yes, and one offset is missing.</strong> EE Offset (−0.02, 0, 0.16) picked the basket on both days, and Pick x + 0.01 put the fingers 2 mm from the hanger plane on 10 October. None of them is too conservative in the sense of holding margin you could give back: EE Offset z has the least margin of all. The missing one is EE Offset y.</div>
<div class="tscroll"><table class="wk"><thead><tr><th>Setting</th><th>Flown on 10 October</th><th>What the flight showed</th><th>For the next flight</th></tr></thead><tbody>
<tr><td>Pick x</td><td class="n">Get x + 0.01</td><td>The gripper closed with the fingers {abs(P4["claw_vs_stem_at_close_mm"]["mocap"][0]):.0f} mm from the hanger plane. The claw stops about 10 mm short of its target at the end of the slide-in, and the 0.01 makes up for that.</td><td>Keep.</td></tr>
<tr><td>EE Offset x</td><td class="n">−0.02</td><td>Right for the pick. It is also used at the place, where the basket hangs {-HG4["descent, until the touchdown"]["along"][0]:.0f} mm behind the claw. The place point has to carry that difference.</td><td>Keep.</td></tr>
<tr><td>EE Offset y</td><td class="n">0</td><td>The jaws centre about 20 mm to the left of the basket body's origin on both days. With 0 the closing jaws push the basket sideways at the pick, and the basket lands 11 to 24 mm to the side at the place.</td><td><b>+0.02.</b> Or move the body's pivot onto the stem in Motive and keep 0.</td></tr>
<tr><td>EE Offset z</td><td class="n">0.16</td><td>Fingers {wi("0.16", "slide_gap_mean"):.0f} mm under the arch on average and {wi("0.16", "slide_gap_min"):.0f} mm at the least. Two picks out of two. A claw flying 5 mm higher would have met the arch.</td><td><b>0.15</b>, which gives {wi("0.15", "slide_gap_min"):.0f} to {wi("0.15", "slide_gap_mean"):.0f} mm. Never above 0.16.</td></tr>
<tr><td>Vertical / Side Margin</td><td class="n">0.20 / 0.10</td><td>The claw waited {-(P4["wait"]["mocap"]["max"][0] - P4["stem_in_frame"][0]):.0f} to {-(P4["wait"]["mocap"]["min"][0] - P4["stem_in_frame"][0]):.0f} mm behind the hanger plane and slid in without touching it. Its sideways error during the slide-in reached {X4["pick_slide_across_mm"]["max"]:.0f} mm.</td><td>Keep.</td></tr>
</tbody></table></div>
<h5>EE Offset z: the same table as on 9 October, with PP-4's heights</h5>
<p>PP-4's claw heights at the pick, shifted to each setting, by mocap. On 10 October the fingers met the arch at {P4["contact_height"]["mocap"]:.0f} mm above the basket's centre.</p>
<div class="tscroll"><table><thead><tr><th>EE Offset z</th><th>Gap under the arch in the slide-in, mean (smallest) [mm]</th><th>Claw above the contact height during the slide-in</th><th>Gap on the handle, mean (smallest) [mm]</th></tr></thead><tbody>
<tr><td class="n">0.15</td><td class="n">{wi("0.15", "slide_gap_mean"):.0f} ({wi("0.15", "slide_gap_min"):.0f})</td><td class="n">never</td><td class="n">{wi("0.15", "wait_gap_mean"):.0f} ({wi("0.15", "wait_gap_min"):.0f})</td></tr>
<tr><td class="n">0.16, as flown</td><td class="n">{wi("0.16", "slide_gap_mean"):.0f} ({wi("0.16", "slide_gap_min"):.0f})</td><td class="n">never, {wi("0.16", "slide_gap_min"):.0f} mm to spare</td><td class="n">{wi("0.16", "wait_gap_mean"):.0f} ({wi("0.16", "wait_gap_min"):.0f})</td></tr>
<tr><td class="n">0.17</td><td class="n">{n(wi("0.17", "slide_gap_mean"))} ({n(wi("0.17", "slide_gap_min"))})</td><td class="n">{100 * wi("0.17", "slide_above"):.0f} % of the time</td><td class="n">{n(wi("0.17", "wait_gap_mean"))} ({n(wi("0.17", "wait_gap_min"))})</td></tr>
</tbody></table></div>
<ul>
<li>On 9 October the same table read 6 mm to spare at 0.16 and 67 % above the contact height at 0.17. Two flights, the same answer: 0.16 passes under the arch by a few millimetres and 0.17 does not.</li>
<li>The few millimetres are smaller than the day-to-day change in the claw's height. PX4's attitude and the mocap attitude moved the planner's claw height by 4 to 10 mm against mocap on 9 October and by 1 to 4 mm on 10 October.</li>
<li>A lower claw costs nothing at the pick. With 0.15 the climb takes up 10 mm more before it carries the basket. If you change EE Offset z, change Place z the other way by the same amount. The two belong together.</li>
</ul>
<h5>The stem and the open jaws</h5>
<p>Between the start of the slide-in and the close the claw's sideways error reached {X4["pick_slide_across_mm"]["max"]:.0f} mm in PP-4, 17 mm in PP-2 and 25 mm in PP-3. The open jaws must clear the stem by more than that on each side, so the open gap less the stem's width should be at least 60 mm. Please measure the new stem against the open gripper.</p>

<h3>3.2 Place</h3>
<h4>How the basket hangs on the claw</h4>
<div class="verdict"><strong>The planner assumes the basket sits under the claw at the place as it does at the pick: centre 20 mm ahead of the claw and level. Hanging, it does not.</strong>
Once lifted, the basket tilts with its top forward and its centre swings back toward the vehicle: {abs(H2["descent"]["along"][0]):.0f} mm behind the claw in PP-2 and {abs(H4["descent"]["along"][0]):.0f} mm in PP-4, where the planner assumes 20 mm ahead. So even with the claw exactly on its target the basket comes down {20 - H2["descent"]["along"][0]:.0f} and {20 - H4["descent"]["along"][0]:.0f} mm before the place point. That is what "short" means on this page.</div>

<h5>What the planner assumes about the grasped basket</h5>
<ul>
<li><b>It plans the claw, not the basket.</b> It holds two points and one vector. The points are where the basket's centre is at the pick, from Get, and where it should be at the place. Both mean the origin of the mocap body <code>obj_0</code>. The vector is the EE Offset, from that centre to the claw's grasp point.</li>
<li><b>Claw target = point + EE Offset, at both ends.</b> The offset is turned to the yaw of each point. With (−0.02, 0, 0.16) the claw's target is 20 mm behind the centre and 160 mm above it at the pick, and the same at the place.</li>
<li><b>Nothing else.</b> There is no payload in the planner: no mass, no swing, no tilt, and nothing that follows the basket after the grasp. Using one offset at both ends is the assumption that the basket is carried rigidly and level, where it was when the gripper closed, with its centre 20 mm ahead of the claw and 160 mm below it.</li>
<li><b>At the pick that holds by construction,</b> because the basket stands on the hat and the claw is flown to it. The 0.01 typed onto Pick x only moves the claw 10 mm further in. At the place the basket hangs from the arch. In Fig. 5 the dashed outline is the assumption and the coloured outlines are where the basket was.</li>
</ul>

<figure>
<div class="ee3d">
 <div class="ee3d-bar"><span class="note vleg"><span><span class="sw dash thin" style="--c:#8f8e84"></span>what the planner assumes</span><span><span class="sw" style="--c:#3987e5"></span>PP-2, 9 October</span><span><span class="sw" style="--c:#d95926"></span>PP-4, 10 October</span><span><span class="sw big" style="--c:#d8d7cf"></span>fingers</span><span>● basket centre</span></span></div>
 <div class="pgrid three">
  <div><p class="ee3d-title">Hover after the lift</p>{hang_note("after_lift")}<div class="plot hhang" id="fhang-after_lift"></div></div>
  <div><p class="ee3d-title">Above the place</p>{hang_note("above_place")}<div class="plot hhang" id="fhang-above_place"></div></div>
  <div><p class="ee3d-title">Descent, until the touchdown</p>{hang_note("descent")}<div class="plot hhang" id="fhang-descent"></div></div>
 </div>
</div>
<figcaption><b>Fig. 5.</b> The basket under the claw, seen from the side with the vehicle on the left, at the three stages flown with the arm straight ahead. The origin is the claw's grasp point and the thick light line is the fingers at their measured pitch. Dashed grey: the basket where the planner takes it to be, level, with its centre 20 mm ahead of the claw and 160 mm below it. Blue: PP-2 on 9 October. Orange: PP-4 on 10 October. Each outline is the basket's 110 × 65 mm body at its measured position and tilt, with the stem and the arch of its hanger. The dotted lines drop from each centre to its distance along the heading in millimetres, negative behind the claw. Positions are the mocap body's origin against the claw, which comes from the vehicle's mocap pose and the measured joints. The tilt is the body's, less its reading while the basket stood on the pick hat. Averages over each stage.</figcaption>
</figure>
<div class="tscroll"><table><thead><tr><th>Basket centre relative to the claw</th><th>PP-2, ahead (+) or behind (−) [mm]</th><th>PP-2, to the right [mm]</th><th>PP-2, tilt</th><th>PP-4, ahead (+) or behind (−) [mm]</th><th>PP-4, to the right [mm]</th><th>PP-4, tilt</th></tr></thead><tbody>
<tr><td>What the planner assumes</td><td class="n">+20</td><td class="n">0</td><td class="n">0°</td><td class="n">+20</td><td class="n">0</td><td class="n">0°</td></tr>
{hang_row2("after_lift", "Hover after the lift")}
{hang_row2("place_start", "Hover at the place start<br><span class='note'>carry pose, arm yawed 12°: the basket also leans " + f"{H2['place_start']['tilt_roll']:.0f}° and {H4['place_start']['tilt_roll']:.0f}°" + " sideways</span>")}
{hang_row2("above_place", "Above the place")}
{hang_row2("descent", "Descent, until the touchdown")}
</tbody></table></div>
<p>Tilt is with the top forward, away from the vehicle. In the three stages of Fig. 5 the sideways lean is {min(abs(s["tilt_roll"]) for k, s in list(H2.items()) + list(H4.items()) if k != "place_start"):.0f} to {max(abs(s["tilt_roll"]) for k, s in list(H2.items()) + list(H4.items()) if k != "place_start"):.0f}°.</p>

<h5>Where the {20 - H4["descent"]["along"][0]:.0f} mm comes from</h5>
<p>The basket hangs from the arch, which rests on the fingers. Just before the touchdown:</p>
<div class="tscroll"><table><thead><tr><th>Along the heading, + is ahead [mm]</th><th>PP-2</th><th>PP-4</th></tr></thead><tbody>
<tr><td>The arch sits on the fingers this far toward their tips from the grasp point</td><td class="n">{sg(HP2[0])}</td><td class="n">{sg(HP4[0])}</td></tr>
<tr><td>The hanger plane is 27.5 mm behind the centre, so the centre is ahead of the arch</td><td class="n">{sg(HP2[1])}</td><td class="n">{sg(HP4[1])}</td></tr>
<tr><td>The tilt swings the centre back under the arch: arch height {HANG["runs"]["p2"]["contact"]:.0f} and {HANG["runs"]["p4"]["contact"]:.0f} mm, tilt {H2["descent"]["tilt_side"]:.0f}° and {H4["descent"]["tilt_side"]:.0f}°</td><td class="n">{sg(HP2[2])}</td><td class="n">{sg(HP4[2])}</td></tr>
<tr><td><b>Centre relative to the claw, measured</b></td><td class="n"><b>{sg(H2["descent"]["along"][0])}</b></td><td class="n"><b>{sg(H4["descent"]["along"][0])}</b></td></tr>
<tr><td>What the planner assumes</td><td class="n">+20</td><td class="n">+20</td></tr>
<tr><td><b>Basket short of the place point with the claw exactly on its target</b></td><td class="n"><b>{20 - H2["descent"]["along"][0]:.0f}</b></td><td class="n"><b>{20 - H4["descent"]["along"][0]:.0f}</b></td></tr>
</tbody></table></div>
<ul>
<li><b>The tilt is the large part.</b> A level basket hanging from the same arch would have its centre {HP2[0] + 27.5:.0f} and {HP4[0] + 27.5:.0f} mm ahead of the claw. The tilt moves it back by {HP2[0] + 27.5 - H2["descent"]["along"][0]:.0f} and {HP4[0] + 27.5 - H4["descent"]["along"][0]:.0f} mm.</li>
<li><b>A free hang would be {math.degrees(math.atan2(27.5, 167)):.0f}°.</b> The hanger is 27.5 mm behind the centre and the arch 167 mm above it, so a basket pivoting freely on its arch tilts by that much and no more. The rest comes from the grip, under the next heading.</li>
<li><b>The tilt grows through the flight:</b> {H2["after_lift"]["tilt_side"]:.0f}, {H2["above_place"]["tilt_side"]:.0f} and {H2["descent"]["tilt_side"]:.0f}° in PP-2, and {H4["after_lift"]["tilt_side"]:.0f}, {H4["above_place"]["tilt_side"]:.0f} and {H4["descent"]["tilt_side"]:.0f}° in PP-4. So the offset is not one fixed number. The value just before the touchdown is the one that sets where the basket lands. At the touchdown itself PP-2 read 35 mm.</li>
<li><b>To the side it is the same on both days:</b> the basket's origin is 11 to 24 mm to the right of the claw, and more with the arm yawed in the carry pose. Section 3.1 gives the reason.</li>
<li><b>The tilt changes the touchdown.</b> A basket tilted by {H4["descent"]["tilt_side"]:.0f}° meets the hat with its far edge first, about 20 mm before it would lie flat, and has to rotate down while the claw keeps descending.</li>
</ul>

<h5>What sets the angle: the closed gripper holds the stem</h5>
<p>A basket pivoting freely on its arch would hang at one angle whatever the claw did. It does not. Fig. 6 puts the basket's tilt beside the pitch of the fingers for the whole time it hangs.</p>
<figure>
<div class="ee3d">
 <div class="pgrid">
  <div><p class="ee3d-title">PP-2, 9 October</p><div class="plot hfollow" id="ffollow-p2"></div></div>
  <div><p class="ee3d-title">PP-4, 10 October</p><div class="plot hfollow" id="ffollow-p4"></div></div>
 </div>
</div>
<figcaption><b>Fig. 6.</b> The hanging basket's tilt against the pitch of the fingers, from just after the lift to the touchdown. Both are measured in the claw's own vertical plane, so the arm's 12° yaw in the carry pose does not mix the axes. Light line: how far the fingers point below the horizontal. Coloured line: how far the basket's top leans toward the fingertips, less the body's reading at rest. Dotted line: the angle at which the basket would hang freely from its arch. Orientation samples that flipped are left out. Marks: 1, the arm leaves the pick pose. 2, joint 2 jumps, by {FL2["ev"]["jump_deg"]:.0f}° in PP-2 and {FL4["ev"]["jump_deg"]:.0f}° in PP-4. 3, the arm turns to the place pose.</figcaption>
</figure>
<ul>
<li><b>Just after the lift it hangs freely.</b> The lift is a jolt and the stem slips in the jaws. The basket settles at {FL2["lift"][1]:.0f}° in PP-2 and {FL4["lift"][1]:.0f}° in PP-4, near the free-hang angle, while the fingers point {FL2["lift"][0]:.0f} and {FL4["lift"][0]:.0f}° down.</li>
<li><b>After that it turns with the claw.</b> Changes of the fingers' pitch faster than 2 s appear in the basket's tilt at {100 * FL2["fast"]["slope"]:.0f} % in PP-2 and {100 * FL4["fast"]["slope"]:.0f} % in PP-4. In PP-4's carry joint 2 jumps by up to {FL4["ev"]["jump_deg"]:.0f}°, and the basket jumps with it.</li>
<li><b>From the carry to the touchdown the fingers pitch down further,</b> from {FL2["carry"][0]:.0f} to {FL2["place"][0]:.0f}° in PP-2 and from {FL4["carry"][0]:.0f} to {FL4["place"][0]:.0f}° in PP-4. The place pose points the claw 10° lower than the carry pose, and the arm gives a few degrees more in the descent. The basket follows {100 * FL2["follow"]:.0f} and {100 * FL4["follow"]:.0f} % of that: from {FL2["carry"][1]:.0f} to {FL2["place"][1]:.0f}° and from {FL4["carry"][1]:.0f} to {FL4["place"][1]:.0f}°.</li>
<li><b>In simulation the basket only hung freely.</b> Up to 12 October the simulation carried the first handle, with its 3 mm stem, and the simulated claw cannot close below about 19 mm, so it never gripped the stem. In {len(SH)} missions recorded on 8 and 9 October the basket hangs at {min(SHV("side")):.0f} to {max(SHV("side")):.0f}° from the carry to the touchdown, while the fingers' pitch goes from {min(SHV("pitch")):.0f} to {max(SHV("pitch")):.0f}°, and its centre stays within {max(abs(v) for v in SHV("along")):.0f} mm of the point under the claw. The simulation had the free-hang part and not the grip part, so it never showed the {20 - H2["descent"]["along"][0]:.0f} to {20 - H4["descent"]["along"][0]:.0f} mm. Since 13 October the simulation carries this holder, with a 20 mm pad on the fin for the claw to close on. In one flight the clamped basket turned with the claw one to one, where the real stem slips. So it still does not show the hang offset.</li>
<li><b>So the tilt at the touchdown has two parts.</b> About 9° is the free hang, which the handle's shape sets. The other {HD2:.0f} and {HD4:.0f}° is the claw turning the gripped basket. Section 5.3 uses this for the handle.</li>
</ul>

<h5>Is it the new handle?</h5>
<ul>
<li><b>The basket hangs back on both days.</b> On 9 October, with the old handle, its centre was already {abs(H2["descent"]["along"][0]):.0f} mm behind the claw before the touchdown.</li>
<li><b>On 10 October it reads {abs(H4["above_place"]["along"][0] - H2["above_place"]["along"][0]):.0f} to {abs(H4["after_lift"]["along"][0] - H2["after_lift"]["along"][0]):.0f} mm further back and {H4["descent"]["tilt_side"] - H2["descent"]["tilt_side"]:.0f}° more tilted, at each of the three stages.</b> A difference that constant comes from something that changed between the two days, not from one flight's swing. The extra {H4["descent"]["tilt_side"] - H2["descent"]["tilt_side"]:.0f}° at an arch height of 171 mm is 12 mm, so the two readings are one change.</li>
<li><b>Two things changed, and the bag cannot tell them apart.</b> One is the handle. Its arch is 8 mm higher above the basket's origin, 171 against 163 mm at the lift, and a different weight on top changes the angle the basket hangs at. The other is the mocap body. On 10 October it reads {HANG["runs"]["p4"]["rest_tilt"]:.1f}° of tilt while the basket stands on the pick hat, against {HANG["runs"]["p2"]["rest_tilt"]:.1f}° on 9 October, and the 180° flips of 9 October are gone (Section 4.1). So the body was probably defined again in Motive, and its origin and axes may sit slightly differently in the basket. The tilts here have that resting reading taken out, which is right only if the pick hat is level.</li>
<li><b>For the place it does not matter which.</b> Get on the Place row captures the same origin with the basket standing on the hat. What Place x has to carry is how far that origin hangs from where the planner assumes it, {20 - H4["descent"]["along"][0]:.0f} mm on 10 October. A ground check with the basket hooked gives that number for any handle and any body (Section 5.2).</li>
</ul>

<h4>The place, from above and from the side</h4>
<figure>
<div class="ee3d">
 <div class="ee3d-bar"><span class="seg" id="fv-place-run"><button type="button" aria-pressed="true" data-k="p4">PP-4, with the basket</button><button type="button" aria-pressed="false" data-k="p2">PP-2, with the basket</button><button type="button" aria-pressed="false" data-k="p1">PP-1, no basket</button></span>
  <span class="note vleg"><span><span class="sw" style="--c:#3987e5"></span>Ready To Place</span><span><span class="sw" style="--c:#d95926"></span>Place</span><span><span class="sw" style="--c:#199e70"></span>Exit To Place</span><span>thick line: claw</span><span><span class="sw dot" style="--c:#b9b8ae"></span>basket centre</span><span><span class="sw dash thin" style="--c:#8f8e84"></span>planned claw path</span><span>✕ claw target</span></span></div>
 <div class="pgrid"><div><p class="ee3d-title">Top view</p><div class="plot hview" id="fv-place-top"></div></div><div><p class="ee3d-title">Side view</p><div class="plot hview" id="fv-place-side"></div></div></div>
</div>
<figcaption><b>Fig. 7.</b> The three stages of the place, drawn like Fig. 4. The origin is the typed place point, the approach runs along world −x, and the dashed outline is the basket if it stood on that point. The planned claw path in the side view is a loop of three parts: the descent, the back-out by the Side Margin along the bottom, and the exit climb. The right-hand dashed line is the descent as commanded, already shifted by the planner's trim. The dotted white line above the cross is where the descent would run without the trim. <b>PP-4:</b> the hat is drawn where the Get on the Place row put it, 30 mm before the typed point. The dotted amber line is the claw height at which the fingers meet the arch of a basket standing on the hat. The claw overshot its target by 75 mm on arrival, Place was pressed 5.0 s later, and the descent was shifted {PL4["short_parts_mm"]["trim"]:.0f} mm toward the vehicle. The basket touched down {PL4["short_total_mm"]:.0f} mm before the typed point and {PL4["basket_from_hat_centre_mm"]:.0f} mm from the hat's centre. The claw then moved {SD4["vehicle_back_max_mm"] / 10:.0f} cm back and took the basket with it over the rim. <b>PP-2:</b> the claw was 56 mm beyond the place point when Place was pressed, swung back to 150 mm before it and came down there. The basket touched down 164 mm before the point, the gripper opened, the basket dropped 4 cm off the rim onto the open fingers, and the exit carried it away again. For PP-2 and PP-1 the hat is drawn centred on the typed point and at the pick hat's height. Its real position was not measured on 9 October.</figcaption>
</figure>
<div class="tscroll"><table class="tl"><thead><tr><th>Time [s]</th><th>What happened in PP-4</th><th>Claw against its typed target, along the approach [mm]</th></tr></thead><tbody>
<tr><td class="n">{PL4["t_arrive"]:.1f}</td><td>The claw arrives above the hat and is still moving forward.</td><td class="n">{-PL4["claw_err_at_arrival_mm"][0]:.0f} beyond</td></tr>
<tr><td class="n">{PL4["t_arrive"] + PL4["t_overshoot_after_arrival"]:.1f}</td><td>Largest overshoot, {PL4["overshoot_max_mm"]:.0f} mm from the target with the sideways part.</td><td class="n">{-PL4["overshoot_vec_mm"][0]:.0f} beyond</td></tr>
<tr><td class="n">{PL4["settled_window"][0]:.1f}–{PL4["settled_window"][1]:.1f}</td><td>The claw has settled.</td><td class="n">{PL4["mean_settled_last2s_mm"][0]:.0f} beyond, on average</td></tr>
<tr><td class="n">{PL4["t_press"]:.1f}</td><td>Place is pressed {ST4["wait_place"]:.1f} s after arrival. The planner averages the claw's error over the last 5 s, which is the whole wait, and shifts the goal by it: {ST4["trim_mm"]["place"][0]:.0f} mm toward the vehicle, {ST4["trim_mm"]["place"][1]:.0f} mm sideways and {abs(ST4["trim_mm"]["place"][2]):.0f} mm down.</td><td class="n">{-PL4["claw_err_at_press_mm"][0]:.0f} beyond</td></tr>
<tr><td class="n">65.7–71.2</td><td>The shift is flown sideways and held for 2 s.</td><td class="n"></td></tr>
<tr><td class="n">71.2–{PL4["t_touchdown"]:.1f}</td><td>Descent. The airframe is within {X4["descent_before_td"]["com_max"]:.0f} mm of its reference and the tilt under {X4["descent_before_td"]["tilt_max"]:.0f}°.</td><td class="n"></td></tr>
<tr><td class="n">{PL4["t_touchdown"]:.2f}</td><td>The basket's far edge meets the hat. Its centre is {PL4["short_total_mm"]:.0f} mm before the typed point and {abs(PL4["side_mm"]):.0f} mm to the side, {PL4["basket_from_hat_centre_mm"]:.0f} mm from the hat's centre.</td><td class="n">{PL4["short_parts_mm"]["trim"] + PL4["short_parts_mm"]["tracking"]:.0f} short</td></tr>
<tr><td class="n">{SD4["t_vehicle_starts"]:.2f}</td><td>The airframe starts to move back.</td><td class="n"></td></tr>
<tr><td class="n">73.3–73.45</td><td>The basket lies flat on the hat, centre at {PL4["basket_flat_on_hat"]["z"]:.3f} m, and is already being pulled along.</td><td class="n"></td></tr>
<tr><td class="n">{SD4["t_tip"]:.2f}</td><td>The basket goes over the rim, {SD4["basket_dragged_mm"] / 10:.0f} cm from where it touched down. It hangs on the claw again and pulls the arm down by 7 cm.</td><td class="n"></td></tr>
<tr><td class="n">{SD4["t_back_max"]:.2f}</td><td>The airframe has moved back {SD4["vehicle_back_max_mm"]:.0f} mm. Its pitch swings from {abs(SD4["pitch_change"]):.0f}° nose up to {SD4["pitch_max_after"] - SD4["pitch_pre"]:.0f}° nose down as the basket's weight comes back.</td><td class="n"></td></tr>
<tr><td class="n">{SD4["t_leg_end"]:.2f}–{SD4["t_open_cmd"]:.2f}</td><td>The descent leg ends. The ground station logs "EE 268 mm from the target" and opens the gripper.</td><td class="n"></td></tr>
<tr><td class="n">{SD4["t_fingers_open"]:.2f}</td><td>The fingers are open. The basket falls and is on the floor at {SD4["t_floor"]:.1f} s, at ({n(SD4["basket_final"][0], 2)}, {n(SD4["basket_final"][1], 2)}).</td><td class="n"></td></tr>
<tr><td class="n">{TK4["t_stab"]:.2f}</td><td>The pilot takes STAB (Section 4.1).</td><td class="n"></td></tr>
</tbody></table></div>
<h5>The second after the touchdown</h5>
<figure>
<img class="photo" alt="Four video frames of PP-4: the basket touching the place hat, lying on it, being pulled over the rim and falling" src="data:image/jpeg;base64,{STRIP}">
<figcaption><b>Fig. 8.</b> PP-4 from the flight video, cropped to the place pillar. The vehicle faces right. The basket is the clear box with white markers under the claw, and the orange blocks are the foam on the landing gear. Times are bag seconds. The video runs 14.0 s ahead of the bag.</figcaption>
</figure>
<figure>
<div class="ee3d"><div class="plot hsd" id="fsetdown"></div></div>
<figcaption><b>Fig. 9.</b> PP-4, 1.3 s before the touchdown to 2.2 s after it. Top: heights above the hat's top surface. The amber line is the claw height at which the fingers carry the arch. It follows the basket. After the touchdown the claw stays on it and does not follow its planned path below it. Middle: how far the airframe and the basket have moved toward the vehicle since the touchdown. Bottom: the airframe's pitch against its hover before the touchdown. The dotted vertical lines are, in order: the touchdown, the airframe starting to move back, the basket going over the rim, the command that opens the gripper, and the fingers open.</figcaption>
</figure>
<ul>
<li><b>What moved the vehicle.</b> The hanging basket had been pulling the nose down with {-M4["above_place"]["dM_delivered"]:.1f} N·m, and the controller's observer had learned to hold against it. When the hat took the weight, that moment was gone within about 0.3 s, faster than the observer lets go. The airframe pitched {abs(SD4["pitch_change"]):.1f}° nose up and moved back at up to {SD4["speed_back_max"]:.2f} m/s. The arm sprang up for the same reason.</li>
<li><b>It is the lift in reverse.</b> At the lift the vehicle was pushed 23 cm forward on 10 October and 27 cm on 9 October. PP-2 showed no push at its set-down because the hat never took the basket's weight there.</li>
<li><b>Why the basket came along.</b> Two things held it to the claw. The gripper was still closed on the stem. And the fingers never came clear of the arch: the claw should have gone on down to about 3 cm below the arch contact, but after the touchdown it stayed {SD4["claw_above_ref_mm"][0]:.0f} mm above its planned height, within a few millimetres of the arch (Fig. 9, top). In the middle panel the basket moves with the airframe from 73.3 s on.</li>
<li><b>The simulation.</b> Up to 12 October the simulated claw never gripped the stem, and the simulated vehicle moved back 11 to 15 cm at the set-down. In the flight the pads close fully and the vehicle moved {SD4["vehicle_back_max_mm"] / 10:.0f} cm. With the holder and the grip pad in the simulation (13 October, one flight) the vehicle moves back 22 cm and drags the clamped basket 6 cm across the hat before the release.</li>
<li><b>How little time there was.</b> From the touchdown: the airframe was moving after {SD4["t_vehicle_starts"] - SD4["t_touchdown"]:.2f} s, the basket was over the rim after {SD4["t_tip"] - SD4["t_touchdown"]:.2f} s, the leg ended after {SD4["touchdown_to_leg_end_s"]:.2f} s, the open command came after {SD4["touchdown_to_open_cmd_s"]:.2f} s and the fingers were open after {SD4["t_fingers_open"] - SD4["t_touchdown"]:.2f} s.</li>
<li><b>Why the leg ran on for 1.3 s.</b> The planned bottom was {(PL4["claw_minus_goal_at_td_mm"][2] + abs(ST4["trim_mm"]["place"][2])):.0f} mm below the claw's height at the touchdown, and a planned leg slows to rest at its end, so those last millimetres are its slowest part. Three things add up to that distance: the tilted basket touches about 20 mm early, the claw's target is about 20 mm under the arch contact so that the fingers come free, and the trim lowered it by another {abs(ST4["trim_mm"]["place"][2]):.0f} mm.</li>
<li><b>A basket set down on the hat's centre would have been pulled off too.</b> It would have had 100 mm to the rim, not {F4["setdown"]["rim"]:.0f}. The airframe had moved 100 mm by 73.45 s, still 0.5 s before the leg ended.</li>
</ul>
<h5>Why the basket landed short of the typed point</h5>
<p>At the touchdown the distance splits into three parts along the approach, all toward the vehicle:</p>
<div class="tscroll"><table><thead><tr><th>Part</th><th>PP-4, with the basket [mm]</th><th>PP-2, with the basket [mm]</th><th>PP-1, no basket, at the release [mm]</th></tr></thead><tbody>
<tr><td>The planned claw path behind the claw's target: the planner's descent trim</td><td class="n">{PL4["short_parts_mm"]["trim"]:.0f}</td><td class="n">55</td><td class="n">36</td></tr>
<tr><td>The claw behind its planned path: tracking error</td><td class="n">{PL4["short_parts_mm"]["tracking"]:.0f}</td><td class="n">74</td><td class="n">33</td></tr>
<tr><td>The basket's centre behind where the planner assumes it under the claw</td><td class="n">{PL4["short_parts_mm"]["hang"]:.0f}</td><td class="n">35</td><td class="n">—</td></tr>
<tr><td><b>Basket short of the place point</b> (PP-1: claw short of its target)</td><td class="n"><b>{PL4["short_total_mm"]:.0f}</b></td><td class="n"><b>164</b></td><td class="n"><b>68</b></td></tr>
</tbody></table></div>
<ol>
<li><b>The trim sampled a swing, three times out of three.</b> All three places were pressed inside the arrival, with the claw past its target, so all three trims moved the goal toward the vehicle: by 36, 55 and {PL4["short_parts_mm"]["trim"]:.0f} mm. The trim window is further down.</li>
<li><b>The tracking error was a third of PP-2's.</b> The claw was {PL4["short_parts_mm"]["tracking"]:.0f} mm behind its path at the touchdown, against 74 mm. The loaded hover wanders by about that much, so this part will come out differently every time.</li>
<li><b>The hang was {PL4["short_parts_mm"]["hang"]:.0f} mm,</b> against 35 mm on 9 October (the hang, above). The 0.03 added to Place x covered two thirds of it.</li>
<li><b>The typed 0.03 is why the basket reached the hat.</b> Measured from the hat's centre the basket was {PL4["short_total_mm"] - 30:.0f} mm short and {abs(PL4["side_mm"]):.0f} mm to the side.</li>
</ol>
<h5>PP-2 and PP-1, 9 October</h5>
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
<p>In PP-2 nothing stopped the release either: with the hook grasp the ground station opens at the bottom of the descent whatever the claw's error.</p>

<h4>Did the longer trim window help?</h4>
<div class="verdict warn"><strong>Not in this flight.</strong> The window was 5 s and Place was pressed {ST4["wait_place"]:.1f} s after the claw arrived, so the average covered the whole arrival swing. The descent was moved {X4["place"]["avg_mm"]:.0f} mm, {PL4["short_parts_mm"]["trim"]:.0f} of them toward the vehicle. Without that shift the claw would have reached the touchdown {PL4["trim_what_if"]["trim off"]["dist_xy"]:.0f} mm from its typed target. With it, it was {PL4["trim_what_if"]["as flown, 5 s window pressed at 5.0 s"]["dist_xy"]:.0f} mm away.</div>
<figure>
<div class="ee3d"><div class="plot htrim" id="ftrim"></div></div>
<figcaption><b>Fig. 10.</b> PP-4, the claw above the place hat. Top: the claw's position along the approach against its typed target. Positive is beyond the target, negative is short of it, toward the vehicle. Bottom: the same across the approach. The claw is in the planner's frame. The grey dashes are the claw path the planner commanded. They step by the trim after Place is pressed. The shaded band is the 5 s the planner averaged. The dotted vertical lines are the claw's arrival, the Place press and the touchdown.</figcaption>
</figure>
<div class="tscroll"><table><thead><tr><th>PP-4, same flight, same claw motion</th><th>Shift of the descent (x, y) [mm]</th><th>Claw from its typed target at the touchdown [mm]</th></tr></thead><tbody>
<tr><td>As flown: 5 s window, pressed 5.0 s after arrival</td><td class="n">{PL4["trim_what_if"]["as flown, 5 s window pressed at 5.0 s"]["trim_mm"][0]:+.0f}, {PL4["trim_what_if"]["as flown, 5 s window pressed at 5.0 s"]["trim_mm"][1]:+.0f}</td><td class="n">{PL4["trim_what_if"]["as flown, 5 s window pressed at 5.0 s"]["dist_xy"]:.0f}</td></tr>
<tr><td>Average of the settled part only, from 3 s after arrival</td><td class="n">{PL4["trim_what_if"]["average of the settled part only"]["trim_mm"][0]:+.0f}, {PL4["trim_what_if"]["average of the settled part only"]["trim_mm"][1]:+.0f}</td><td class="n">{PL4["trim_what_if"]["average of the settled part only"]["dist_xy"]:.0f}</td></tr>
<tr><td>Trim off</td><td class="n">0, 0</td><td class="n">{PL4["trim_what_if"]["trim off"]["dist_xy"]:.0f}</td></tr>
</tbody></table></div>
<ul>
<li><b>The planner did what it was set to do.</b> It logged a trim of ({sg(ST4["trim_mm"]["place"][0], 1)}, {sg(ST4["trim_mm"]["place"][1], 1)}, {n(ST4["trim_mm"]["place"][2], 1)}) mm. The average of target minus claw over the 5.0 s of the wait, computed from the bag, is ({sg(PL4["mean_5s_mm"][0], 1)}, {sg(PL4["mean_5s_mm"][1], 1)}, {n(PL4["mean_5s_mm"][2], 1)}) mm.</li>
<li><b>Two thirds of it was the arrival.</b> From 3 s after arrival the same average was ({PL4["mean_settled_last2s_mm"][0]:+.0f}, {PL4["mean_settled_last2s_mm"][1]:+.0f}) mm. The claw overshot by {PL4["overshoot_max_mm"]:.0f} mm one second after arriving and needed about three seconds to come back.</li>
<li><b>This is what the 9 October replay predicted.</b> For a press at exactly 5 s it gave a trim error of 49 mm (the last part of this section). The flight gave {X4["place"]["avg_mm"]:.0f} mm of trim, of which about 30 mm was error. The rule that came with the 5 s window was to wait at least 8 s. It was not kept, and nothing in the planner or on the ground station asks for it.</li>
<li><b>Even the settled average did not predict the landing.</b> The settled claw sat {PL4["mean_settled_last2s_mm"][0]:.0f} mm beyond its target. At the touchdown, 7 s later, it was {PL4["short_parts_mm"]["tracking"]:.0f} mm behind its path. The loaded hover wanders by 25 to 30 mm with a period of 5 to 7 s, and with the trim on the bottom of the descent comes 7 s after the press, against 3 s with it off.</li>
<li><b>The trim also moved the bottom down by {abs(ST4["trim_mm"]["place"][2]):.0f} mm,</b> because the claw read that much high during the wait. That lengthened the slow end of the descent (the place, above).</li>
<li><b>At the pick the window did nothing, and that was fine.</b> Pick was pressed {ST4["wait_pick"]:.1f} s after arrival. Less than half the window had filled, so the planner applied no shift. The claw was {X4["pick"]["err_at_press_mm"]:.0f} mm from its target at the press, and the ground station held the close until it was within {abs(P4["claw_vs_planner_target_at_close_mm"]["planner"][0]):.0f} mm.</li>
<li><b>Against 9 October.</b> PP-2 pressed after 0.8 s with a 1 s window and got 57 mm. PP-4 pressed after 5.0 s with a 5 s window and got {X4["place"]["avg_mm"]:.0f} mm. The longer window is not worse, and it cannot help while the press comes inside the arrival. Section 5.5 has what I would change.</li>
</ul>

<h4>Were the place offsets right?</h4>
<div class="verdict"><strong>The place offset went the right way and needs to go further.</strong> Place x − 0.03 covered two thirds of the hang. EE Offset x and y are shared with the pick (Section 3.1), so the place point has to carry what the hang adds.</div>
<div class="tscroll"><table class="wk"><thead><tr><th>Setting</th><th>Flown on 10 October</th><th>What the flight showed</th><th>For the next flight</th></tr></thead><tbody>
<tr><td>Place x</td><td class="n">Get x − 0.03</td><td>The basket touched down {PL4["short_total_mm"] - 30:.0f} mm short of the Get position. The hang explains {PL4["short_parts_mm"]["hang"]:.0f} mm of the distance to the typed point, the tracking error {PL4["short_parts_mm"]["tracking"]:.0f} and the trim {PL4["short_parts_mm"]["trim"]:.0f}.</td><td><b>Get x − 0.06</b> with the trim off: the hang plus about 10 mm for the claw arriving short.</td></tr>
<tr><td>Place y</td><td class="n">Get y</td><td>The basket landed {abs(PL4["side_mm"]):.0f} mm to the side: trim {abs(ST4["trim_mm"]["place"][1]):.0f}, tracking {abs(PL4["claw_minus_ref_at_td_mm"][1]):.0f}, hang 15.</td><td>Get y. EE Offset y = +0.02 takes care of the hang.</td></tr>
<tr><td>Place z</td><td class="n">Get z − 0.01</td><td>The planned bottom was about 20 mm under the arch contact, and {20 + abs(ST4["trim_mm"]["place"][2]):.0f} mm with the trim. The fingers do have to end below the arch. Lower only lengthens the time between the touchdown and the release.</td><td>Get z with EE Offset z = 0.15, which is the same claw height. Not lower.</td></tr>
<tr><td>Place yaw</td><td class="n">−180</td><td></td><td>Keep.</td></tr>
<tr><td>Descent trim window</td><td class="n">5 s</td><td>It did not help in this flight. See the trim window above.</td><td>Trim off for this controller, or a planner that waits (Section 5.5).</td></tr>
</tbody></table></div>

<h4>What a Pick or Place press does</h4>
<div class="verdict warn"><strong>The planner does not wait for the error to converge.</strong> It accepts the press as soon as the approach leg has ended, takes the claw's average error over the trim window as "the hover error", and shifts the descent by it. A press during the arrival swing therefore builds that swing into the descent. For the pick the ground station catches it afterwards. For the place nothing does.</div>
<h5>At the press: the planner</h5>
<ol>
<li><b>Refused</b> only in three cases: the approach leg is still flying, the claw's latest error is more than 50 mm away from its own average over the trim window, or that average is larger than 150 mm.</li>
<li><b>The 50 mm is not a distance to the target.</b> With the descent trim on, which is the default and what flew, the check compares the claw with its own recent average. It measures how fast the claw is moving. A claw 76 mm from its target and drifting slowly passes.</li>
<li><b>The shift.</b> That average becomes the trim. The planner moves the descent by it so that a claw with a steady offset lands on the target. If the claw sits behind its target, the descent moves forward by the same amount. If the claw is past its target, the descent moves back. A shift over 5 mm is flown sideways first, held 2 s, then the claw goes straight down.</li>
<li><b>No shift before half a window.</b> If the claw has been above the target for less than half the trim window, the planner applies no shift at all. That is what happened at PP-4's Pick.</li>
</ol>
<h5>After the descent: the ground station</h5>
<ul>
<li><b>Pick.</b> The gripper closes only when the claw is within 20 mm of its target and within 3 mm across the stem. Until then the claw waits on the handle. A hurried Pick costs a few seconds, not the grasp.</li>
<li><b>Place, hook grasp.</b> The gripper opens at the bottom of the descent whatever the error, and Exit To Place starts at once. The Place press is the last decision.</li>
</ul>
<h5>The seven presses in the flights</h5>
<div class="tscroll"><table><thead><tr><th>Press</th><th>Wait after arrival</th><th>Trim window</th><th>Claw error at the press [mm]</th><th>Window average = the trim [mm]</th><th>Claw vs that average [mm]</th><th>Result</th></tr></thead><tbody>
<tr><td>PP-1 Pick</td><td class="n">39.0 s</td><td class="n">1 s</td><td class="n">7</td><td class="n">18</td><td class="n">14</td><td>accepted, descent shifted 18 mm</td></tr>
<tr><td>PP-2 Pick</td><td class="n">4.2 s</td><td class="n">1 s</td><td class="n">6</td><td class="n">9</td><td class="n">4</td><td>accepted, shifted 9 mm</td></tr>
<tr><td>PP-3 Pick</td><td class="n">1.6 s</td><td class="n">1 s</td><td class="n">7</td><td class="n">15</td><td class="n">9</td><td>accepted, shifted 15 mm</td></tr>
<tr><td>PP-4 Pick</td><td class="n">{ST4["wait_pick"]:.1f} s</td><td class="n">5 s</td><td class="n">{X4["pick"]["err_at_press_mm"]:.0f}</td><td class="n">{X4["pick"]["avg_mm"]:.0f}</td><td class="n">{X4["pick"]["claw_vs_avg_mm"]:.0f}</td><td>accepted, no shift: less than half the window had filled</td></tr>
<tr><td>PP-1 Place</td><td class="n">1.6 s</td><td class="n">1 s</td><td class="n">31</td><td class="n">42</td><td class="n">15</td><td>accepted, shifted 42 mm</td></tr>
<tr><td>PP-2 Place</td><td class="n">0.8 s</td><td class="n">1 s</td><td class="n">76</td><td class="n">57</td><td class="n">30</td><td>accepted, shifted 57 mm, of which 55 mm toward the vehicle</td></tr>
<tr><td>PP-4 Place</td><td class="n">{ST4["wait_place"]:.1f} s</td><td class="n">5 s</td><td class="n">{X4["place"]["err_at_press_mm"]:.0f}</td><td class="n">{X4["place"]["avg_mm"]:.0f}</td><td class="n">{X4["place"]["claw_vs_avg_mm"]:.0f}</td><td>accepted, shifted {X4["place"]["avg_mm"]:.0f} mm, of which {PL4["short_parts_mm"]["trim"]:.0f} mm toward the vehicle</td></tr>
</tbody></table></div>
<ul>
<li>No press was refused. At PP-2's Place the claw was 76 mm from its target, outside the 50 mm the status line shows, and the planner still logged "claw within tolerance above the target, descending". In PP-4 the status line read "claw 59 mm from the point above the target (tolerance 50 mm), press Place to descend" at arrival.</li>
<li>The status line invites the press from the moment the leg ends. It does not say whether the claw has settled, or how full the trim window is.</li>
<li>Even PP-1's Pick, after a 39 s wait and with the claw 7 mm from its target, got an 18 mm shift. The hover wanders with a 5 to 7 s period, so a one-second average is a sample of the wander, not a steady offset.</li>
</ul>
<h5>The replay on the 9 October hovers: trim off, or trim on and wait longer?</h5>
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
<h5>The 5 s window, decided on 10 October</h5>
<p><code>pick_place_descent_trim_window_s</code> has been 5 s in the hardware pick-and-place yaml since 10 October, which both hardware controllers' planner reads. The simulation files keep 1 s. The table is the 9 October replay that came with the decision. PP-4 pressed at 5.0 s and its trim was about 30 mm wrong, close to the 49 mm in the second row (the trim window, above).</p>
<div class="tscroll"><table><thead><tr><th>Place pressed this long after arrival</th><th>What the 5 s average holds</th><th>Trim error on the loaded hover of PP-2 [mm]</th></tr></thead><tbody>
<tr><td>under 2.5 s</td><td>less than half a window: the planner applies no shift</td><td class="n">no trim</td></tr>
<tr><td>5 s</td><td>the whole arrival swing</td><td class="n">49</td></tr>
<tr><td>6 s</td><td>most of the swing has left the window</td><td class="n">20</td></tr>
<tr><td>8 s</td><td>settled hover only</td><td class="n">20</td></tr>
<tr><td>10 s</td><td>settled hover only</td><td class="n">18</td></tr>
</tbody></table></div>
<ul>
<li><b>The window only works with the wait.</b> At exactly 5 s the average still holds the first seconds of the arrival, when the loaded claw is 60 to 140 mm off.</li>
<li><b>The decoupled controller</b> reads the same planner block. It relies on the trim for its steady 5 cm offset, so there too the press must come at least 5 s after arrival, or the descent is not trimmed.</li>
</ul>

<h2>4. Problems in the flight test</h2>
<p>Open problems first, in order of how much they matter for the next flight. Then the ones that are closed.</p>

<h3>4.1 Open problems</h3>

<h4>The set-down pulls the basket off the hat (PP-4)</h4>
<p>Section 3.2. When the hat takes the basket's weight the vehicle moves back {SD4["vehicle_back_max_mm"] / 10:.0f} cm within a second, and the gripper, still closed on the stem, takes the basket with it. The ground station opens the gripper only when the descent leg has ended, {SD4["touchdown_to_open_cmd_s"]:.1f} s after the touchdown. PP-2 never reached this point, because its basket came down on the rim and the hat never took its weight. Section 5.4 has the fix.</p>

<h4>The descent trim moves the place the wrong way (PP-1, PP-2, PP-4)</h4>
<p>Section 3.2. Three places, three shifts toward the vehicle: 36, 55 and {PL4["short_parts_mm"]["trim"]:.0f} mm. The 5 s window did not change that, because the press still came inside the arrival. Section 5.5.</p>

<h4>Every pilot takeover has ended on the floor at an angle (PP-1, PP-3, PP-4)</h4>
<figure>
<div class="ee3d"><div class="pgrid three">
<div><p class="ee3d-title">PP-1</p><div class="plot hland" id="fland-p1"></div></div>
<div><p class="ee3d-title">PP-3</p><div class="plot hland" id="fland-p3"></div></div>
<div><p class="ee3d-title">PP-4</p><div class="plot hland" id="fland-p4"></div></div></div></div>
<figcaption><b>Fig. 11.</b> The last seconds of PP-1, PP-3 and PP-4. Height and attitude from mocap. Arm torque from the servo currents. In PP-4 "motors stop" is the instant the battery current fell from {TK4["current_before"]:.0f} A to under 1 A.</figcaption>
</figure>
<ul>
<li><b>PP-1.</b> The vehicle came down to 0.35 m in DIRECT through a drone ground-station target. The pilot took STAB at 0.40 m. It dropped at 1.2 m/s from 0.43 m, pitched from 15° to 49° in one second and stayed at 45° nose down. Disarm came 1.5 s after STAB.</li>
<li><b>PP-3.</b> STAB at 0.44 m. The vehicle climbed to 0.56 m, pitched 28° nose up in the air, hit the floor at 1.75 m/s and came to rest at 57° nose up.</li>
<li><b>PP-4.</b> The pilot took STAB at {TK4["t_stab"]:.1f} s, {TK4["t_stab"] - SD4["t_open_cmd"]:.1f} s after the gripper opened, at {TK4["z_at_stab"]:.2f} m and while the vehicle was still swinging from the set-down. It climbed to {TK4["z_top"]:.2f} m, travelling at 0.6 m/s. At {TK4["t_current_drop"]:.1f} s the motors stopped. The vehicle fell, hit the floor at {abs(TK4["vz_impact"]):.1f} m/s at {TK4["t_impact"]:.1f} s and came to rest {TK4["travel_m"]:.1f} m from where the pilot took over, tipped by {TK4["rest_tilt"]:.0f}°. PX4 reported disarmed at {TK4["t_disarm"]:.1f} s, after the impact.</li>
<li><b>The flight node and the planner after the takeover.</b> In all three flights the flight node went on reporting DIRECT after the pilot had the vehicle. In PP-1 the planner's tilt guard saw 18.4° and flew its abort: gripper open, arm to the release pose. On the floor the arm followed that reference until joint 2 stopped at 32° against a target of 2°, and joints 2 and 3 held 2.7 and 1.4 N·m, their caps, for the 2.4 s until the bag ended. In PP-4 the guard saw 20.5° during the fall and flew the same abort on the floor for 10 s. The arm met nothing this time and its torque stayed under {math.ceil(TK4["arm_torque_on_ground_max"][1] * 10) / 10:.1f} N·m.</li>
</ul>
<p>Section 5.7 has what to change.</p>

<h4>Joint 2 sticks and slips under the load (PP-4)</h4>
<figure>
<div class="ee3d"><div class="plot hcarry" id="fcarry"></div></div>
<figcaption><b>Fig. 12.</b> PP-4, the end of the carry and the hover at the place start, with the basket on the claw. Top: joint 2 and its reference. Middle: the torque the whole-body controller commands on joint 2, with the arm controller's cap. Bottom: the airframe's horizontal error. Joint 2 stays at 36° while the commanded torque climbs, jumps 9° in 0.2 s, stays near 45° while the torque falls, and drops back. The airframe swings with it. The dotted vertical lines are the arrival at the place start and the Ready To Place press. The red dotted lines are the arm controller's 45° position guard and its torque cap.</figcaption>
</figure>
<ul>
<li><b>Joint 2 moves in jumps.</b> Its reference in the carry pose is 38°. From 46 to 53 s it sat at {J24["q2_low_high"][0]:.0f}° or at {J24["q2_low_high"][1]:.0f}° and crossed between them every {J24["stick_slip_period"]:.1f} s. The controller's torque command went from 0.6 to 2.3 N·m and back in each cycle. Joint 3 moved between {J24["q3_low_high"][0]:.0f}° and {J24["q3_low_high"][1]:.0f}° and the wrist between {n(J24["q4_low_high"][0])}° and {n(J24["q4_low_high"][1])}°.</li>
<li><b>The airframe follows.</b> Its error swung by ±0.15 m at the same period, 193 mm at worst. This is the 100 mm rms of the hover at the place start.</li>
<li><b>The torque is at the caps.</b> The controller commanded up to {J24["law_tau_max"][1]:.2f} N·m on joint 2, for a cap of 2.44, and {J24["law_tau_max"][2]:.2f} on joint 3, for a cap of 1.42. With the arm controller's own friction and gravity terms the totals passed the caps and were cut for {CP4["s_clipped"][1]:.2f} s and {CP4["s_clipped"][2]:.2f} s, all on the carry leg. They were above 90 % of the caps for {CP4["s_above_90"][1]:.1f} s and {CP4["s_above_90"][2]:.1f} s. The whole-body controller is not told when this happens. The caps themselves are in Section 4.2.</li>
<li><b>Servo current while carrying:</b> {L4["current_counts_loaded"]["mean"][1]:.0f} counts on joint 2 and {L4["current_counts_loaded"]["mean"][2]:.0f} on joint 3 on average, {L4["current_counts_loaded"]["peak"][1]:.0f} and {L4["current_counts_loaded"]["peak"][2]:.0f} at the peaks, over {L4["current_counts_loaded"]["seconds"]:.0f} s. On 9 October: 216 and 140, 336 and 204.</li>
<li><b>Joint 2 reached its position guard.</b> The arm controller replaces the commanded torque with its own pull-back above 45°, the real arm's limit. Joint 2 read {J24["q2_max"]:.1f}° at the top of a jump, and also at 7 to 8 s while holding the home pose, where its reference is 40° and it sits 5° high. It was above 44° for {J24["s_over_44"]:.1f} s of the flight.</li>
<li><b>Why it sticks.</b> The gearbox friction of these servos grows with the load on the joint. Carrying the basket, joint 2 holds 1.3 N·m, and the torque has to change by about ±0.8 N·m before the joint moves. The controller holds the claw with a soft spring, so the torque builds slowly, the joint breaks loose, overshoots and sticks again.</li>
</ul>

<h4>The rotor wash moves the basket on the pick hat (PP-4)</h4>
<ul>
<li>The basket slid {BR4["moved_xy_mm"]:.0f} mm and turned {abs(BR4["yaw_change"]):.1f}° as the vehicle arrived behind the stem (Section 3.1). On 9 October the same arrival moved it by 4 to 6 mm.</li>
<li>The pick point is captured before the flight, so the claw then aims at where the basket was. On 10 October the push happened to bring the stem onto the claw's line. A push the other way would have put it 4 cm off.</li>
<li>The same wash blows on a basket released at the place while the vehicle backs out. A rim or a rubber mat on both hats would hold it. A lighter basket moves more (Section 5.3).</li>
</ul>

<h4>The basket's mocap body</h4>
<ul>
<li><b>Its origin is not on the stem.</b> The jaws centre about 20 mm to one side of it on both days (Section 3.1). Either EE Offset y carries that, or the body's pivot is moved onto the stem in Motive.</li>
<li><b>Orientation flips.</b> On 9 October the solved orientation jumped by 180° about the vertical whenever the claw was close, and read −179° for five seconds at PP-2's pick. On 10 October it did not flip at the pick. It jumped four times, for single samples, during the turn of the carry. If the markers were moved with the new handle, that worked.</li>
<li><b>Why it matters.</b> The drone's heading at the pick follows the captured yaw. A Get taken during a flip would send the drone to the far side of the pillar and turn EE Offset x around. Since 10 October both Get buttons refuse a yaw that changes by more than 10° inside their half-second window.</li>
</ul>

<h4>PX4's attitude and the mocap attitude disagree in roll</h4>
<ul>
<li>On 9 October the two differed by 1.9 to 2.2° in roll and 2.2 to 2.9° in pitch, fixed to the body. On 10 October: {abs(A4["attitude_offset"]["roll"]):.1f}° in roll and {abs(A4["attitude_offset"]["pitch"]):.1f}° in pitch before the lift, {abs(A4["attitude_offset"]["roll_loaded"]):.1f}° and {abs(A4["attitude_offset"]["pitch_loaded"]):.1f}° with the basket. The pitch part is gone and the roll part is not.</li>
<li>The claw is 0.26 m ahead of the body origin, so pitch is what moves the planner's idea of the claw height. That difference was 4 to 10 mm on 9 October and is {abs(A4["attitude_offset"]["claw_z_planner_minus_mocap_mm"]["pick"]):.0f} mm at the pick and {abs(A4["attitude_offset"]["claw_z_planner_minus_mocap_mm"]["place"]):.0f} mm at the place on 10 October.</li>
<li>EE Offset z is measured in the planner's own frame and already carries whatever is left. It is one more reason not to run it with 4 mm of margin (Section 3.1).</li>
</ul>

<h4>The vertical load: why the controller reads more than 190 g (PP-2, PP-4)</h4>
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
<figcaption><b>Fig. 13.</b> Top: the collective thrust the controller commands, in its own units, and the thrust delivered, computed from the same motor commands and the battery voltage with the 20 August stepped-payload calibration of these motors. Middle: battery power. Bottom: battery voltage. In PP-1, with nothing on the claw, the command climbs by 2.8 N as the battery falls, while delivered thrust and power stay flat.</figcaption>
</figure>
<h5>Without a payload</h5>
<ul>
<li>Over the three flights the thrust command for the same hover ranged from {min(v["u1"][0] for v in PU.values()):.1f} to {max(v["u1"][1] for v in PU.values()):.1f} N as the battery went from {max(v["V"][0] for v in PU.values()):.1f} to {min(v["V"][1] for v in PU.values()):.1f} V. The observer's vertical estimate went from +{max(v["dz"][0] for v in PU.values()):.1f} to −{-min(v["dz"][1] for v in PU.values()):.1f} N with nothing on the claw.</li>
<li>Delivered thrust stayed at {PU["p1"]["T_fit"]:.1f} ± 0.3 N and battery power at {PU["p1"]["P"]:.0f} ± 5 W in all three flights. These two do not depend on the controller's units, so they are the gauges used below.</li>
<li>The delivered-thrust level, 34.5 N, is 6 % under the 36.7 N weight in the controller's model. The calibration was made on the bare airframe, and this vehicle's mass has not been confirmed on a scale. Only changes of the gauge are used here.</li>
</ul>
<h5>With the basket</h5>
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
<h5>What was in the vertical estimate just before the place</h5>
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
<li><b>The vertical channel copes with all of it.</b> The airframe's height error with the basket is 0 ± 4 mm. What disturbs the hover is the moment (Section 2.2).</li>
<li><b>One hover would close the 0.4 to 0.9 N question.</b> Hook a compact 190 g weight on the claw in place of the basket and read the battery power. About 502 W means the basket's shape matters, which points to rotor wash. About 512 W again means the thrust side.</li>
</ul>
<h5>PP-4, 10 October: what the load gauges read</h5>
<p>The same gauges, taken the same way: the hover behind the stem before the lift against the hover above the place.</p>
<div class="tscroll"><table><thead><tr><th>Gauge</th><th>PP-2, 9 October</th><th>PP-4, 10 October</th></tr></thead><tbody>
<tr><td>Pitch moment on the vehicle</td><td class="n">0.44–0.46 N·m, 1.7–1.9 N at the claw</td><td class="n">{-M4["above_place"]["dM_delivered"]:.2f}–{-M4["trim_hold"]["dM_delivered"]:.2f} N·m, {M4["above_place"]["F"]:.1f}–{M4["trim_hold"]["F"]:.1f} N at the claw</td></tr>
<tr><td>Battery power</td><td class="n">466 → 512 W, 2.2–2.4 N</td><td class="n">{L4["windows"]["pre"]["P"]:.0f} → {L4["windows"]["above_place"]["P"]:.0f} W, {G4["above_place"]["power_N"][0]:.1f}–{G4["above_place"]["power_N"][1]:.1f} N</td></tr>
<tr><td>Delivered thrust from the motor commands</td><td class="n">+2.7–2.8 N</td><td class="n">+{G4["trim_hold"]["fit_N"]:.1f}–{G4["above_place"]["fit_N"]:.1f} N</td></tr>
<tr><td>The controller's thrust command, in its own units</td><td class="n">+3.7 N</td><td class="n">+{G4["above_place"]["d_u1"]:.1f} N</td></tr>
<tr><td>Joint 2 torque carrying the basket, mean</td><td class="n">1.4 N·m</td><td class="n">{J24["applied_mean_loaded"][1]:.1f} N·m</td></tr>
</tbody></table></div>
<ul>
<li><b>Three of the four vehicle gauges read 0.3 to 0.7 N more than on 9 October,</b> and the pitch moment, which agreed with the scale then, reads a third more. The joint torque does not. The likely reason is that the basket with the new handle is heavier than 190 g. Please weigh it. Section 5.3 depends on that number.</li>
<li><b>The vertical channel again copes.</b> The observer's vertical estimate went from {sg(A4["lift"]["dz_before"], 1)} to {n(A4["lift"]["dz_after"], 1)} N and the height held.</li>
</ul>

<h4>Smaller items</h4>
<ul>
<li>The controller's own torque limit, <code>wb_tau_max</code>, is 3.0 N·m for every joint, above all four arm caps. When the arm controller cuts a command the whole-body controller is not told. On 10 October that happened for a tenth of a second.</li>
<li>DIRECT entry in PP-4 left the airframe 119 mm and the yaw 4.8° from their references for the first four seconds.</li>
<li>Battery in PP-4: {A4["battery"]["start"][0]:.1f} V and {A4["battery"]["start"][1]:.0f} A at DIRECT entry, {A4["battery"]["carry"][0]:.1f} V and {A4["battery"]["carry"][1]:.0f} A with the basket. On 9 October the thrust command for the same hover rose by 2.8 N over PP-1's 150 s as the battery fell (the vertical load, above).</li>
<li>The topics recorded from the Orin show no gap over 25 ms in PP-4. On 9 October there was one of 74 to 105 ms, in PP-2.</li>
</ul>

<h3>4.2 Solved problems</h3>

<h4>The Pixhawk data link froze in hover (PP-2)</h4>
<div class="verdict good"><strong>The cause was found on the Orin on 10 October and the fix was committed 27 minutes before PP-4. PP-4 flew with a clean link.</strong></div>
<figure>
<div class="ee3d"><div class="pgrid"><div><div class="plot hmap" id="ffreeze-map"></div></div><div><div class="plot hts" id="ffreeze-ts"></div></div></div></div>
<figcaption><b>Fig. 14.</b> PP-2 after the stream from the Pixhawk stopped at 144.67 s. Left: the airframe's track from above, with a dot every 0.5 s. Right: attitude, speed and height from mocap. The vehicle was hovering at (−0.94, 0.01, 1.00) m with the mission's last leg still to fly.</figcaption>
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
<li><b>What it was.</b> The MicroXRCEAgent is the process on the Orin that carries every topic between the Pixhawk and ROS, in both directions. All of its roughly 30 threads ran at real-time priority on one isolated core, with real-time throttling off. When PX4 re-established its session, the Agent's single processing thread had to wait for a reader thread to stop, and that reader was spinning at 100 %. At the same real-time priority on the same core the spinning thread never gave way, so the waiting thread never ran again. Nothing from the Pixhawk reached ROS and nothing from ROS reached the Pixhawk. Backtraces of both threads on the Orin confirmed it. On 10 October the same lock-up lasted more than 20 minutes on the ground, with the ground station showing the Agent's core at 100 %.</li>
<li><b>How that fits PP-2.</b> Every Pixhawk topic stopped within the same millisecond, EKF2 lost its mocap at the same time, the Pixhawk itself kept running, and one stray sample came through five seconds later. PP-3's stream stopping 7 s after disarm has the same signature. The 9 October bags cannot show the session being re-established, so this is the same symptom with a mechanism that was caught in the act a day later.</li>
<li><b>The fix.</b> The Agent keeps its isolated core and runs at ordinary priority (fsc_autopilot_ros2 <code>71ec97a</code>, 10 October 18:40). Under the ordinary scheduler the same spin lasts microseconds.</li>
<li><b>PP-4.</b> In 69 s of DIRECT the Pixhawk's attitude and sensor topics arrived at {A4["link"]["vatt"]["rate"]:.1f} Hz with a largest gap of {A4["link"]["vatt"]["max_gap_ms"]:.0f} ms, its odometry at {A4["link"]["vodom"]["rate"]:.1f} Hz with {A4["link"]["vodom"]["max_gap_ms"]:.0f} ms, and the control loop ran at {A4["link"]["wb"]["rate"]:.1f} Hz with {A4["link"]["wb"]["max_gap_ms"]:.0f} ms. It is one flight. Section 5.7 keeps two precautions.</li>
</ul>

<h4>PP-3 missed the pick</h4>
<p>EE Offset z had been raised from 0.16 to 0.20 after PP-2, which put the claw above the arch. PP-4 flew 0.16 again and picked the basket. Section 3.1 shows why it should not go above 0.16.</p>

<h4>The basket's stream was dead in PP-1</h4>
<p>On 9 October <code>/vrpn_mocap/obj_0/pose</code> carried no message for the 192 s of PP-1, while the processor republished one frozen pose at 60 Hz, 11 423 identical samples, with <code>mocap_status</code> reading normal. A Get in that state stores a stale pose. Since 10 October both Get buttons refuse a pose that does not change at all and say why on the status line. In PP-4 the stream was live for the whole flight, at 120 Hz with a largest gap of 35 ms.</p>

<h4>The place point was typed without a measurement</h4>
<p>On 9 October the place point was typed by hand, and the three flights used three points up to 6 cm apart. The arm ground station now has a Get on the Place row and editable boxes on the Pick row. PP-4 used both, which is why the place hat's position is known in that flight.</p>

<h4>Arm torque and the torque caps (PP-2, PP-4)</h4>
<div class="verdict"><strong>Raise joint 3's cap by 20 %. Leave joint 2.</strong> No command was clipped on 9 October, but joints 2 and 3 came within 3 % of their caps for a few hundredths of a second. On 10 October, with the caps unchanged, both were cut for a tenth of a second (Section 4.1). Joint 3 has room in the servo. Joint 2's limit is heat, which a higher cap does not help.</div>
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
<li><b>A cap protects the servo and whatever the arm pushes against.</b> After PP-1's touchdown the arm pushed into the floor at exactly these caps for 2.4 s (Section 4.1). With joint 3 at 230 counts it would push 20 % harder there. That is the cost of the raise.</li>
<li><b>For joint 2, reduce the heat instead.</b> Keep the loaded part of a flight short, leave two minutes between loaded flights as the arm repository's rule says, and record the servos' temperature. The temperature is read by the arm's hardware interface but is not in these bags.</li>
<li><b>If you still want more on joint 2,</b> 400 counts (2.67 N·m, +10 %) with its PWM Limit at 550 is the most I would add, and only with the temperature recorded.</li>
<li><b>The controller's own limit does not match.</b> <code>wb_tau_max</code> is 3.0 N·m for every joint, above all four arm caps. If the arm ever clips, the controller is not told. It did not happen here. Per-joint limits equal to the arm's caps would close this, and that needs a code change.</li>
</ul>

<h2>5. Further Improvement</h2>
<p>Your three ideas first, each with what the two days of data say about it. Then what I would add, and one list in order for the next flight.</p>

<h3>5.1 A larger place platform</h3>
<div class="verdict"><strong>Yes, and extending it away from the vehicle is the right direction, as long as the place point moves with it.</strong> What the basket needs is distance between the point it is aimed at and the rim on the vehicle's side. A larger platform alone would not have completed PP-4.</div>
<ul>
<li><b>Where the basket has gone so far.</b> In PP-4 it touched down {PL4["short_total_mm"] - 30:.0f} mm short of the hat's centre and {abs(PL4["side_mm"]):.0f} mm to the side, and was then pulled {SD4["basket_dragged_mm"] / 10:.0f} cm further toward the vehicle. In PP-2 it came down short again, on the rim. Every miss so far is on the vehicle's side.</li>
<li><b>So the room is needed between the aim point and the near rim.</b> Extending the platform toward world −x and typing the place point further in gives that, without bringing the platform closer to the landing gear. Extending it and leaving the place point where it is adds room on the side the basket has not gone to.</li>
</ul>
<div class="tscroll"><table><thead><tr><th>Platform</th><th>Aim point from the near rim [mm]</th><th>Basket's centre may land short / long by, before it is over the rim [mm]</th><th>Short / long by, with all four corners on [mm]</th></tr></thead><tbody>
<tr><td>Today: disc, 200 mm across, aimed at its centre</td><td class="n">100</td><td class="n">100 / 100</td><td class="n">20 / 20</td></tr>
<tr><td>200 mm wide, 300 mm deep</td><td class="n">180</td><td class="n">180 / 120</td><td class="n">125 / 65</td></tr>
<tr><td>200 mm wide, 400 mm deep</td><td class="n">250</td><td class="n">250 / 150</td><td class="n">195 / 95</td></tr>
</tbody></table></div>
<ul>
<li><b>What it has to cover.</b> The basket landed {PL4["short_total_mm"]:.0f} mm short of its aim in PP-4 and 164 mm in PP-2. Of that, the trim and the hang can be taken out (Sections 5.2 and 5.5). What remains is the tracking error, {PL4["short_parts_mm"]["tracking"]:.0f} and 74 mm in the two flights, and the loaded hover, which stayed within {A4["hover"]["pooled"]["payload"]["p95"]:.0f} mm for 95 % of the time on 10 October and within 84 mm on 9 October. 300 mm deep covers that. 400 mm would have held PP-2's basket with nothing else changed.</li>
<li><b>Width and the landing gear.</b> The skids are 284 mm apart on the inside and reach 163 mm ahead of the body origin. The basket lands only {X4["skids"]["basket_ahead_of_skid_tip_mm"]:.0f} mm ahead of their tips. At PP-4's touchdown the tips were {X4["skids"]["tip_behind_rim_mm"]:.0f} mm short of the near rim and the skid bottoms {X4["skids"]["skid_bottom_above_hat_at_td_mm"]:.0f} mm above the hat's top. At the planned bottom of the descent they would have been about 30 mm above it. Keep the platform no wider than 200 mm so that it passes between the skids. Once the aim point is more than about 7 cm from the near rim the skid tips are alongside its near end, with 42 mm to spare on each side when the vehicle is centred. In PP-4 it was {abs(X4["skids"]["lateral_offset_mm"]):.0f} mm off the centre line.</li>
<li><b>Flying higher.</b> In the place pose the skid bottoms are 141 mm below the claw, and the claw has to end below the arch, which is {P4["contact_height"]["mocap"] + 32.5:.0f} mm above the platform. That puts the skids 30 to 40 mm above the platform at the bottom, whatever its size. A place pose that reaches lower raises the vehicle: [0, 25, 45, 0] by 10 mm and [0, 20, 50, 0] by 16 mm with the claw at the same 20° pitch, the second with joint 3 on its limit. A 60° fold such as [0, 20, 40, 0] gives 46 mm but tilts the claw to 30°, and the hook then needs more friction to keep the arch on the fingers. With this hook there are one to two centimetres to gain.</li>
<li><b>What a larger platform does not do.</b> In PP-4 the claw pulled the basket {SD4["basket_dragged_mm"] / 10:.0f} cm to the rim and the vehicle moved {SD4["vehicle_back_max_mm"] / 10:.0f} cm. On a larger platform that is a 20 cm drag across it, not a place. The release has to come first (Section 5.4).</li>
<li><b>Give it a rim or a mat.</b> A shallow rim or a rubber mat keeps a released basket in place while the rotors are still above it.</li>
</ul>

<h3>5.2 The offsets</h3>
<div class="tscroll"><table class="wk"><thead><tr><th>Setting</th><th>9 October</th><th>10 October</th><th>Next flight</th></tr></thead><tbody>
<tr><td>EE Offset (x, y, z)</td><td class="n">−0.02, 0, 0.16</td><td class="n">−0.02, 0, 0.16</td><td class="n"><b>−0.02, +0.02, 0.15</b></td></tr>
<tr><td>Pick point</td><td>Get</td><td>Get x + 0.01</td><td>Get x + 0.01</td></tr>
<tr><td>Place point</td><td>typed</td><td>Get x − 0.03, Get y, Get z − 0.01, yaw −180</td><td><b>Get x − 0.06, Get y, Get z</b>, yaw −180</td></tr>
<tr><td>Vertical / Side Margin</td><td class="n">0.20 / 0.10</td><td class="n">0.20 / 0.10</td><td class="n">0.20 / 0.10</td></tr>
</tbody></table></div>
<ul>
<li><b>The reasons are in Sections 3.1 and 3.2.</b> In short: the basket hangs {PL4["short_parts_mm"]["hang"]:.0f} mm short of where the planner assumes and about 20 mm to the right of the claw, and the fingers passed only {wi("0.16", "slide_gap_min"):.0f} mm under the arch.</li>
<li><b>Check the hang on the ground first.</b> These numbers belong to this handle and to this mocap body (Section 3.2). Hook the basket, put the arm in the place pose and compare the basket's position with the claw's. Expect the basket's centre about 27 mm behind the claw and 20 mm to its right. The x to subtract from Get is 20 mm plus the distance behind, plus 10 mm.</li>
<li><b>EE Offset y depends on the mocap body.</b> If the basket's body is defined again in Motive, check it again. If its pivot is put on the stem, EE Offset y goes back to 0.</li>
<li><b>With a larger platform</b> do the Get with the basket standing where you want it to land, and subtract the same 0.06.</li>
</ul>

<h3>5.3 The payload: lighter, and a handle that hangs level</h3>
<div class="verdict"><strong>It helps with everything the basket disturbs, in proportion. First weigh the basket: the gauges say it got heavier with the new handle.</strong></div>
<ul>
<li><b>What scales with the weight.</b> The push at the lift, 23 and 27 cm, and at the set-down, {SD4["vehicle_back_max_mm"] / 10:.0f} cm. The tilt at both. The loaded hover error. The arm torque. The first three come from one number, the basket's weight times the 0.25 m between the claw and the centre of mass: 0.45 N·m on 9 October and {-M4["above_place"]["dM_delivered"]:.1f} N·m on 10 October by the moment gauge. Half the weight is about half the push, 11 to 13 cm.</li>
<li><b>The arm gains the most.</b> Carrying the basket, joint 2 holds {J24["applied_mean_loaded"][1]:.1f} N·m, of which {J24["applied_mean_loaded"][1] - J24["applied_mean_unloaded"][1]:.1f} is the basket. Its friction grows with that load, which is what makes it stick and slip, and its peaks reach the cap. At half the weight it holds {J24["applied_mean_unloaded"][1] + 0.5 * (J24["applied_mean_loaded"][1] - J24["applied_mean_unloaded"][1]):.1f} N·m.</li>
<li><b>Weigh it.</b> The load gauges read 0.3 to 0.7 N more than on 9 October (Section 4.1). If the new handle added 30 to 60 g, the first saving is in the handle.</li>
<li><b>What gets worse.</b> The rotor wash. It moved this basket {BR4["moved_xy_mm"]:.0f} mm and turned it {abs(BR4["yaw_change"]):.1f}° on the pick hat before the grasp, and it blows on the released basket at the place while the vehicle backs out. A lighter basket moves more. Give both hats a rim or a mat, and keep the basket's walls open.</li>
<li><b>The other way to the same effect</b> is to tell the controller about the basket (Section 5.6). The weight then still loads the arm, but most of the push at the lift and at the set-down goes.</li>
</ul>

<h4>A handle that hangs level</h4>
<div class="verdict"><strong>Tilting the handle is the right kind of change. Lean it forward, away from the vehicle, until the arch is above the basket's centre of mass. Leaning it back toward the vehicle makes the tilt and the offset larger.</strong></div>
<ul>
<li><b>The basket settles with its centre of mass under the arch.</b> That is where it goes after every jolt (Section 3.2). Standing on the hat, its centre is 27.5 mm ahead of the arch. Hanging, it is under the arch or behind it. Only the handle's shape can remove that difference.</li>
<li><b>Leaning the handle back moves the arch, not the basket.</b> The arch ends up further behind the centre of mass, the basket swings until its centre of mass is under the arch again, and it hangs steeper. To hang level it would need the jaws to hold the basket's 1.9 N at a {HCB[0]:.0f} mm lever. They let go at 27.5 mm in the first second after both lifts.</li>
</ul>
<div class="tscroll"><table><thead><tr><th>Handle</th><th>Arch behind the basket's centre [mm]</th><th>Hangs freely at</th><th>Tilt at the touchdown, PP-2 and PP-4</th><th>Basket short of the place point with the claw on target [mm]</th></tr></thead><tbody>
<tr><td>As flown, stem upright</td><td class="n">{HC0[0]:.1f}</td><td class="n">{HC0[1]:.0f}°</td><td class="n">{H2["descent"]["tilt_side"]:.0f} and {H4["descent"]["tilt_side"]:.0f}°, measured</td><td class="n">{20 - H2["descent"]["along"][0]:.0f} and {20 - H4["descent"]["along"][0]:.0f}, measured</td></tr>
<tr><td>Stem leaning back 15°, toward the vehicle</td><td class="n">{HCB[0]:.0f}</td><td class="n">{HCB[1]:.0f}°</td><td class="n">{HCB[2][0][0]:.0f} and {HCB[2][1][0]:.0f}°</td><td class="n">{HCB[2][0][1]:.0f} and {HCB[2][1][1]:.0f}</td></tr>
<tr><td><b>Stem leaning forward 12°, arch above the centre</b></td><td class="n"><b>0</b></td><td class="n"><b>0°</b></td><td class="n"><b>{HCF[2][0][0]:.0f} and {HCF[2][1][0]:.0f}°</b></td><td class="n"><b>{HCF[2][0][1]:.0f} and {HCF[2][1][1]:.0f}</b></td></tr>
</tbody></table></div>
<p class="note">The last two rows are estimates. They assume the basket behaves as in these two flights: it settles at its free-hang angle, the claw then turns it by {HD2:.0f} and {HD4:.0f}° more, and the arch sits on the fingers where it did. Same stem length and the same foot as now.</p>
<ul>
<li><b>With the arch above the centre of mass the basket hangs as it stands.</b> EE Offset x moves forward with the arch, by 27.5 mm to about +0.01, and then describes the pick and the place alike.</li>
<li><b>What is left,</b> {HCF[2][0][1]:.0f} to {HCF[2][1][1]:.0f} mm and {HCF[2][0][0]:.0f} to {HCF[2][1][0]:.0f}°, is the claw turning the gripped basket between the carry and the touchdown. It goes if the fingers have the same pitch in the carry pose and the place pose, or if the jaws do not grip the stem (Section 5.4).</li>
<li><b>Bench check.</b> Hang the basket by its arch on a round rod. With the arch above the centre of mass it hangs level. The present handle hangs at {HC0[1]:.0f}°.</li>
</ul>

<h3>5.4 Release the basket at the touchdown</h3>
<div class="verdict"><strong>This is the change that would have completed PP-4.</strong> The basket was on the hat. It needed the fingers off the stem within about {SD4["t_vehicle_starts"] - SD4["t_touchdown"]:.1f} s of the touchdown, and they came off after {SD4["t_fingers_open"] - SD4["t_touchdown"]:.1f} s.</div>
<ol class="steps">
<li><b>Open on the touchdown, not at the end of the leg.</b> The touchdown shows in the basket's mocap height within a tenth of a second: the basket stops going down while the claw's path continues. The planner already reads <code>obj_0</code>. It can report the touchdown, the ground station opens on that report, and Exit To Place follows as it does now. Two slower signs of the same event are joints 2 and 3 moving by 6° within 0.3 s and the observer's vertical estimate dropping by 1 N within 0.3 s. This is a change in the planner and in the arm ground station. It is not made yet.</li>
<li><b>Do not grip the stem.</b> The hook does not need a grip. The basket hangs from the arch on the fingers, and the stem between them only keeps it from sliding sideways. With the jaws closed to a gap a few millimetres wider than the stem, the fingers can slide out from under the arch when the vehicle moves back, which is the way Exit To Place backs out anyway, and at the pick the closing jaws no longer push the basket. The ground station has only open and closed today. It needs a third position, or the fingers need a mechanical stop. Check it on the ground first: with the basket standing and the fingers under the arch, pull the claw straight back by hand and see whether the basket stays.</li>
<li><b>End the descent sooner after the touchdown.</b> No trim in z, and Place z no lower than needed (Section 3.2). In PP-4 that takes the planned distance after the touchdown from {PL4["claw_minus_goal_at_td_mm"][2] + abs(ST4["trim_mm"]["place"][2]):.0f} to {PL4["claw_minus_goal_at_td_mm"][2]:.0f} mm. It helps a little and cannot replace the first two.</li>
<li><b>Centre the touchdown</b> (Section 5.2). It turns {F4["setdown"]["rim"]:.0f} mm to the rim into 100 mm, which was 0.8 s in PP-4.</li>
<li><b>Take the push away</b> (Sections 5.3 and 5.6).</li>
</ol>
<p>In PP-4 the first of these would have left the basket where it touched down, {PL4["basket_from_hat_centre_mm"]:.0f} mm from the hat's centre with its near edge about 4 cm over the rim. With the fourth as well it stands on the hat with all four corners.</p>

<h3>5.5 The descent trim</h3>
<ul>
<li><b>For flights with this controller, turn it off:</b> <code>pick_place_descent_trim: false</code>. The planner accepts the change while it runs. Its 50 mm check then compares the claw with the target itself, and the bottom of the descent comes 3 s after the press, not 7 s. In PP-4 the claw would have been {PL4["trim_what_if"]["trim off"]["dist_xy"]:.0f} mm from its target. This controller has no steady offset for the trim to remove: its settled loaded hover sat ({sg(A4["hover"]["pooled"]["payload"]["mean_xy"][0])}, {sg(A4["hover"]["pooled"]["payload"]["mean_xy"][1])}) mm from the target on 10 October and (+25, +10) mm on 9 October.</li>
<li><b>The decoupled controller needs the trim</b> for its steady 5 cm offset, and it reads the same planner block. Turn the trim back on for it.</li>
<li><b>If the trim stays on, make the planner wait.</b> Start the average 3 s after the leg has ended, and refuse the press until the window is full, with a status line that counts down. That turns the "wait 8 s" rule into something the planner enforces. It is a small planner change and is not made yet.</li>
<li><b>Leave z out of the trim.</b> The height holds to a few millimetres, and a lower bottom only delays the release.</li>
</ul>

<h3>5.6 The controller and the arm</h3>
<ul>
<li><b>Tell the controller about the basket.</b> Add the basket's mass at the claw to the controller's model while it is hooked, switched by the same events that close and open the gripper. The lift and the set-down are then steps the model expects, and the observer no longer has to learn {-M4["above_place"]["dM_delivered"]:.1f} N·m and forget it again. It needs the weighed mass and the hang offsets of Section 3.2, and it is a code change in the flight node. The simulation shows the same push at the set-down, 11 to 15 cm, so it can be developed there first.</li>
<li><b>Joint 3's torque cap</b> is still 192 duty counts. The 9 October recommendation stands: 230 counts, with its servo PWM Limit from 330 to 380 (Section 4.2). On 10 October its command was cut for {CP4["s_clipped"][2]:.2f} s.</li>
<li><b>Joint 2's cap</b> was reached for {CP4["s_clipped"][1]:.2f} s. Its limit is heat: it carried {L4["current_counts_loaded"]["mean"][1]:.0f} current counts on average for {L4["current_counts_loaded"]["seconds"]:.0f} s, against the 250 of the August overload shutdown. A lighter basket is the cure here, not a higher cap.</li>
<li><b>Joint 2's position guard.</b> The carry pose asks for 38° and the home pose for 40°, 5 to 7° under the 45° guard, and the joint reached the guard in both. A carry pose with joint 2 at 33°, for example [12, 33, 42, 0], leaves 12°. It opens the fold by 5°, so check the landing gear against the basket in simulation first.</li>
<li><b>Joint 2 sticking and slipping.</b> The 8 October offline study of the arm's friction compensation has two candidate changes for this, neither flown. They are worth a ground test with the basket hooked, where the same jumps should show without flying.</li>
</ul>

<h3>5.7 Takeover, landing and the link</h3>
<ol class="steps">
<li>Land as the workflow says: back to SAFETY, then land and disarm from the drone ground station. Revert at 0.8 m or higher, since earlier flights measured a dip of about 20 cm at every revert on this vehicle.</li>
<li>If the pilot takes over in STAB, do it from a steady hover with the throttle stick at hover, and bring the vehicle down under power. In PP-4 the motors stopped at {TK4["z_top"]:.2f} m.</li>
<li>Make the flight node report SAFETY as soon as PX4 leaves OFFBOARD or disarms, and make the planner's guard do nothing when the vehicle is not in OFFBOARD. Three flights now show the planner or the arm acting after the pilot had the vehicle. Neither change is made yet.</li>
<li>Watch the MicroXRCEAgent's core on the Orin over the next flights. One clean flight is not a proof.</li>
<li>Shorten the blind second in case the link ever stops again: PX4's <code>COM_OF_LOSS_T</code> at 0.2 to 0.3 s for direct-actuation flights, and a short timeout on attitude and rate in the flight node, which today wait behind the 1.0 s odometry gate.</li>
</ol>

<h3>5.8 In order, for the next flight</h3>
<ol class="steps">
<li><b>Weigh the basket</b> with the new handle.</li>
<li><b>Handle,</b> if it is printed again: stem leaning forward so the arch is above the basket's centre of mass, and EE Offset x moved forward by the same distance (Section 5.3).</li>
<li><b>Ground check with the basket hooked</b> and the arm in the place pose: where the basket's centre sits against the claw, and the stem against the open and the closed jaws.</li>
<li><b>Settings:</b> EE Offset (−0.02, +0.02, 0.15). Pick = Get x + 0.01. Place = Get x − 0.06, Get y, Get z, yaw −180. Margins 0.20 / 0.10.</li>
<li><b>Descent trim off</b> for the whole-body flight.</li>
<li><b>Release at the touchdown:</b> the planner and ground-station change of Section 5.4, or jaws that do not grip the stem. Without one of them, a basket that reaches the platform will be pulled off it again.</li>
<li><b>Place platform:</b> 200 mm wide and 300 mm deep, extended away from the vehicle, with a rim or a mat. Do the Get with the basket standing 18 cm in from the near rim.</li>
<li><b>Joint 3's cap</b> to 230 counts with its PWM Limit at 380.</li>
<li><b>First flight back</b> over open floor with the platform on a low stand, and land through SAFETY from 0.8 m.</li>
</ol>

<h2>6. Method and data</h2>
<ul>
<li><b>Bags.</b> <code>flight_wb_l1_4d_pick_and_place_20261009_172557</code>, <code>_180228</code>, <code>_181817</code> and <code>flight_wb_l1_4d_pick_and_place_20261010_190719</code>, recorded on the ground-station computer.</li>
<li><b>Measured states.</b> Airframe from the fused odometry the controller flies on, and separately from the vehicle's mocap body. Claw by the planner's kinematic model on either pose with the measured joints. The model reproduces the planner's own claw estimate to 0.05 mm.</li>
<li><b>References.</b> The planner's whole-body reference stream.</li>
<li><b>Basket.</b> Its mocap body. On 9 October the basket's resting pose before the approach is the frame at the pick, because the solved orientation flips when the claw is near. On 10 October the frame is the basket's pose in the hover behind the stem, after the rotor wash had moved it, with the approach direction from the captured pick yaw.</li>
<li><b>The place hat on 10 October.</b> Its centre is taken as the typed place point less the 0.03 typed onto Get x. Its top is 32.5 mm under the basket's centre height while the basket lay flat on it, {PL4["basket_flat_on_hat"]["z"]:.3f} m. The basket's footprint and the hat's radius are from the CAD files.</li>
<li><b>Touchdown and the arch.</b> The touchdown is the first sample at which the basket stops descending. The height at which the fingers meet the arch is the claw's height above the basket's centre when the basket has risen 3 mm at the pick: {P4["contact_height"]["mocap"]:.0f} mm by mocap in PP-4, 163 mm in PP-2.</li>
<li><b>Video.</b> <code>VID_20261010_190704.mp4</code>, 20 frames per second. It runs 14.0 ± 0.05 s ahead of the bag: the basket goes over the rim at 87.6 s in the video and 73.6 s in the bag.</li>
<li><b>Scoring windows.</b> DIRECT entry to the controller's last tick with live feedback: 28.0–187.2 s, 14.6–144.6 s, 12.4–80.7 s and 5.8–74.8 s.</li>
<li><b>Hover windows.</b> Spans in which the planner's centre-of-mass and claw references both move slower than 5 mm/s for at least 1.5 s. Settled means without the first 3 s. With the basket: PP-2 from the lift-off at 47.0 s to its removal at 118.3 s, leaving out 89.5–96.5 s when it rested on the hat's rim, and PP-4 from {P4["t_liftoff"]:.1f} s to the touchdown.</li>
<li><b>Load gauges.</b> Delivered thrust is the sum of the squared rotor speeds the motor commands ask for, times a thrust constant that follows the battery voltage and the throttle as fitted on the seven stepped-payload hovers of 20 August. Battery power is voltage times current from PX4's battery status. The moment gauge is the controller's rotational disturbance estimate about the pitch axis, converted with the same thrust ratio.</li>
<li><b>Applied arm torque.</b> The servos' measured current divided by the calibrated counts per newton-metre. The arm's own weight is taken out with the arm controller's calibrated gravity model. A command counts as cut when the streamed torque plus the arm controller's friction and gravity terms, in duty counts, reaches the cap.</li>
<li><b>Not in the bag.</b> The gripper's commands, the PX4 log, the Orin's system log and the pilot's sticks.</li>
<li><b>Files.</b> Tools, numbers and this page's source are in <code>docs/docs_aerial_manipulator/archive/wb_pick_place_flight_20261009/</code>. The 10 October analysis is in <code>wb_pick_place_flight_20261010/</code> beside it.</li>
</ul>
"""

HEAD = ('<!doctype html>\n<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">\n'
        '<title>1009 + 1010 Pick-and-Place Flights</title>\n'
        '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">\n')
css = open(os.path.join(HERE, "report_style.css")).read().replace("</style>", EXTRA_CSS + "</style>")
def merged(name, key):
    """A 10-09 figure-data file with PP-4's entry of the same layout added under "p4"."""
    dd = J(f"fig_{name}.json"); dd["p4"] = F4[key]
    return json.dumps(dd, separators=(",", ":")).replace("</", "<\\/")


blocks = {"3d": merged("3d", "3d"), "err": merged("err", "err"), "hover": raw("fig_hover.json"), "load": raw("fig_load.json"),
          "views": merged("views", "views"), "freeze": raw("fig_freeze.json"), "land": merged("land", "land"),
          "p4": json.dumps(dict({k: F4[k] for k in ("trim", "setdown", "carry")}, hang=HANG), separators=(",", ":")).replace("</", "<\\/")}
data = "\n".join(f'<script type="application/json" id="data-{k}">{v}</script>' for k, v in blocks.items())
js = ('<script src="https://cdn.jsdelivr.net/npm/plotly.js-gl3d-dist-min@2.35.2/plotly-gl3d.min.js"></script>\n'
      "<script>\n" + open(os.path.join(HERE, "figures.js")).read() + "\n</script>\n"
      "<script>\n" + open(os.path.join(D10, "tools", "figures10.js")).read() + "\n</script>")
html = HEAD + css + "\n</head>\n<body>\n" + BODY + "\n" + data + "\n" + js + "\n</body></html>\n"
out = os.path.join(HERE, "..", "report.html")
open(out, "w").write(html)
print("wrote", os.path.abspath(out), len(html) // 1024, "kB")
