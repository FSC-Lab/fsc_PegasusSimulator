#!/usr/bin/env python3
"""Build the 'Sim-to-Real Flight Performance Tuning' page.

    /usr/bin/python3 build_report.py <out.html> <tag>=<title>=<compare.json> [...]

Reads analysis/fit_plant.json for the identification tables and one
compare_<tag>.json per replayed mission (tools/compare.py). Everything is
inlined; Plotly is loaded from cdn.jsdelivr.net (the artifact CSP allows it).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
HERE_LOGS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
AN = os.path.join(HERE, "..", "analysis")


def fmt(v, n=1):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "–"
    return f"{v:.{n}f}"


def delta(a, b, n=1, unit=""):
    if a is None or b is None:
        return "–"
    d = b - a
    rel = f" ({100 * d / a:+.0f}%)" if abs(a) > 1e-9 else ""
    return f"{d:+.{n}f}{unit}{rel}"


PARAM_ROWS = [
    # (group, key, hardware belief, robustness sim plant, mirror sim plant, source)
    ("Rotors", "thrust constant kf [N/(rad/s)²]", "4.260431e-05 believed", "4.041283e-05 plant, allocator 4.7545e-05 (+17.6 %)",
     "4.19e-05 mean (×1.037); per flight 4.03–4.31e-05", "hover thrust balance kf = kf_bel·mg/u1, all planner holds, 6 flights; same-command DC thrust gain 0.98–1.06"),
    ("Rotors", "battery sag [%/min]", "—", "none", "−3.6 (−2.6…−4.5 per flight)", "linear fit of kf(t) over each DIRECT window; pack 24.1→23.6 V in 83 s on 0918"),
    ("Rotors", "yaw coefficient c [N·m/(rad/s)²]", "km 0.018164 (bench c/kf)", "2.474e-06 = 3.0× bench", "5.77e-07 = 0.70× bench (×0.2333)", "same-command yaw gain 0.43–0.78 of bench c (0921 #3 0.78, best excitation); 0807 bare T650 0.61"),
    ("Rotors", "spin-up lag λ [1/s]", "—", "10.0265", "10.0265 (unchanged)", "delay scan of roll/pitch response: ≤10 ms beyond the lag"),
    ("Airframe", "total mass [kg]", "3.746170", "×1.10 (4.121)", "×1.00", "node mass gate ±0.5 %; a mass error at hover is indistinguishable from kf"),
    ("Airframe", "body inertia", "M_r_d 1.5×I0", "×1.10", "×1.00", "same-command roll/pitch gain 1.11/1.03 on 0921 #3 (corr 0.72/0.83), of which 1.06 is kf"),
    ("Airframe", "bare-airframe CoM [mm]", "y −17.854 (model frame)", "10/10/5 shift, belief 0", "x −17.854 (actual frame) = the belief", "0825 measurement; observer roll/pitch residual ≤0.05 N·m ≈ 1 mm confirms it"),
    ("Airframe", "standing force bias [N], body FLU", "—", "none", "(+0.55, −0.50, 0)", "filtered d̂_t in every hover hold: (+0.57,−0.44) 0918, (+0.57,−0.42) 0921, (+0.47,−0.50) F1, (+0.62,−0.60) F2; body-fixed through a 360° circle"),
    ("Airframe", "standing yaw torque [N·m]", "—", "none", "−0.095", "filtered d̂_r,z −0.07…−0.13 on every flight"),
    ("Arm", "gearbox friction × report model, j1..j4", "FF 1.00× on the arm", "1.05 all joints", "1.0 / 0.95 / 0.75 / 1.5", "0924 F3/F4 in-flight Coulomb + breakaway (j2 0.15–0.21 vs 0.19–0.20; j3 0.07–0.10 vs 0.11); j4 0913 ground test"),
    ("Arm", "arm link mass", "nominal", "×1.05", "×1.00", "hold residual applied − S·g within friction, no consistent sign"),
    ("Arm", "current-loop residual [mA rms]", "—", "9.6/6.2/9.6/9.6", "9.6/6.2/9.6/9.6 (unchanged)", "bench 0911; flights' applied−intended scatter 0.01–0.04 N·m rms agrees"),
    ("Arm", "reported joint velocity", "Present Velocity", "exact PhysX", "48 ms first-order lag + 0.024 rad/s quantum", "cross-correlation of joint_states.velocity vs d/dt position: 44–56 ms, corr ≤0.98; quantum from the value histogram"),
    ("Controller", "allocator kf / km, thrust map, all gains, observer, guards", "as flown", "sim values (+17.6 % kf, sim thrust map, ude gate 0.35)", "hardware file verbatim", "the mirror's section 2 = the hardware yaml; diff shows only vehicle_name, arm-hold stream, go-home service"),
]

ID_ROWS = [("f18", "0918 mission", "raw 60 Hz"), ("c3", "0921 #3 go-to-start", "raw 120 Hz"),
           ("a1", "0924 F1 circle", "fused"), ("a2", "0924 F2 circle", "fused"),
           ("a3", "0924 F3 circle 12 s", "fused"), ("a4", "0924 F4 circle 6 s", "fused")]

STAT_ROWS = [
    ("CoM tracking error, rms [mm]", "com_err_norm_rms_mm", 1),
    ("CoM tracking error, peak [mm]", "com_err_norm_peak_mm", 0),
    ("EE task error, rms [mm]", "ee_task_norm_rms_mm", 1),
    ("EE heading error, rms [°]", "heading_err_rms_deg", 2),
    ("attitude error |e_R|, mean", "eR_norm_mean", 4),
    ("tilt, peak [°]", "tilt_peak_deg", 2),
    ("joint 2 error, rms [°]", ("joint_err_rms_deg", 1), 2),
    ("joint 3 error, rms [°]", ("joint_err_rms_deg", 2), 2),
    ("joint 2 torque, peak [N·m]", ("tau_cmd_peak_nm", 1), 2),
    ("collective u1, mean [N]", "u1_mean_n", 2),
    ("d̂_z, mean [N]", ("dhat_t_mean_n", 2), 2),
    ("motors, mean", "motors_mean", 3),
]


def get(st, key):
    if isinstance(key, tuple):
        v = st.get(key[0])
        return None if v is None else v[key[1]]
    v = st.get(key)
    return float(np.mean(v)) if isinstance(v, list) else v


def stats_table(phases, dr, ds):
    rows = []
    for label, key, n in STAT_ROWS:
        a, b = get(dr, key), get(ds, key)
        rows.append(f"<tr><th>{label}</th><td class=num>{fmt(a, n)}</td><td class=num>{fmt(b, n)}</td><td class=num>{delta(a, b, n)}</td></tr>")
    return ("<table><thead><tr><th>whole DIRECT window</th><th>real</th><th>sim (mirror)</th><th>sim − real</th></tr></thead><tbody>"
            + "".join(rows) + "</tbody></table>")


def phase_table(phases):
    keys = [("CoM rms [mm]", "com_err_norm_rms_mm", 1), ("CoM pk [mm]", "com_err_norm_peak_mm", 0),
            ("EE rms [mm]", "ee_task_norm_rms_mm", 1), ("head rms [°]", "heading_err_rms_deg", 2),
            ("|e_R| mean", "eR_norm_mean", 3), ("tilt pk [°]", "tilt_peak_deg", 1),
            ("q2 rms [°]", ("joint_err_rms_deg", 1), 2), ("q3 rms [°]", ("joint_err_rms_deg", 2), 2),
            ("u1 [N]", "u1_mean_n", 1), ("d̂_z [N]", ("dhat_t_mean_n", 2), 2)]
    head = "<tr><th>phase</th><th></th>" + "".join(f"<th>{k[0]}</th>" for k in keys) + "</tr>"
    body = []
    for n, ph in phases.items():
        for lab, st in (("real", ph["real"]), ("sim", ph["sim"])):
            cells = "".join(f"<td class=num>{fmt(get(st, k[1]), k[2])}</td>" for k in keys)
            body.append(f"<tr class={lab}><td class=s>{n if lab=='real' else ''}</td><td class=s>{lab} {st['dur_s']:.0f} s</td>{cells}</tr>")
    return f"<div class=tscroll><table class=phases><thead>{head}</thead><tbody>{''.join(body)}</tbody></table></div>"


def figure_spec(phases, mission_id):
    """One Plotly figure per quantity: phases concatenated on a phase-relative
    axis with vertical separators; real vs sim."""
    quantities = [("com_err_norm", "CoM tracking error |x_c − x_cd| [m]"), ("ee_err_norm", "EE task error |e_y| [m]"),
                  ("tilt_deg", "tilt [°]"), ("eR_norm", "attitude error |e_R|"), ("u1", "collective u1 [N]"),
                  ("dhat_z", "d̂_z [N]"), ("mot_mean", "mean motor command"),
                  ("q2_deg", "joint 2 [°] (dashed = reference)"), ("q3_deg", "joint 3 [°] (dashed = reference)"),
                  ("tau2", "joint 2 torque [N·m]")]
    figs = []
    for key, title in quantities:
        tr, ts, tref_r, tref_s, seps, ticks = [], [], [], [], [], []
        off = 0.0
        for n, ph in phases.items():
            R, S = ph["traces"]["real"], ph["traces"]["sim"]
            if not R or not S or key not in R or key not in S:
                continue
            span = max(R["t"][-1], S["t"][-1])
            tr += [[x + off for x in R["t"]] + [None]]
            ts += [[x + off for x in S["t"]] + [None]]
            R_y = R[key] + [None]
            S_y = S[key] + [None]
            tr[-1] = (tr[-1], R_y)
            ts[-1] = (ts[-1], S_y)
            if key in ("q2_deg", "q3_deg"):
                rk = key.replace("_deg", "d_deg")
                tref_r.append(([x + off for x in R["t"]] + [None], R[rk] + [None]))
                tref_s.append(([x + off for x in S["t"]] + [None], S[rk] + [None]))
            short = n.replace("hold0", "hold").replace("_move", "").replace("_settle", " s").replace("leg", "L")
            ticks.append((off + span / 2, short))
            off += span + 1.0
            seps.append(off - 0.5)
        figs.append({"id": f"{mission_id}_{key}", "title": title,
                     "real": {"x": sum([a for a, _ in tr], []), "y": sum([b for _, b in tr], [])},
                     "sim": {"x": sum([a for a, _ in ts], []), "y": sum([b for _, b in ts], [])},
                     "real_ref": {"x": sum([a for a, _ in tref_r], []), "y": sum([b for _, b in tref_r], [])} if tref_r else None,
                     "sim_ref": {"x": sum([a for a, _ in tref_s], []), "y": sum([b for _, b in tref_s], [])} if tref_s else None,
                     "seps": seps, "ticks": ticks})
    return figs


CSS = """
:root{--bg:#F4F6F5;--panel:#FFFFFF;--ink:#18232B;--ink2:#4B5B63;--ink3:#7B8A90;--line:#D5DDDF;
--real:#1E6B76;--sim:#B4261F;--real-soft:#DDEBEE;--sim-soft:#F8DCDA;--ok:#2C7A57;--warn:#B36B08;
--mono:"IBM Plex Mono",ui-monospace,Menlo,monospace;--sans:"IBM Plex Sans",system-ui,sans-serif;--disp:"IBM Plex Sans Condensed","IBM Plex Sans",system-ui,sans-serif}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#121A1E;--panel:#19242A;--ink:#E6ECEE;--ink2:#AEBBC1;--ink3:#7F8E95;--line:#2C3A41;--real:#6FC3CE;--sim:#F08A83;--real-soft:#183A41;--sim-soft:#3E1B19;--ok:#69C596;--warn:#E5A44A;color-scheme:dark}}
:root[data-theme="dark"]{--bg:#121A1E;--panel:#19242A;--ink:#E6ECEE;--ink2:#AEBBC1;--ink3:#7F8E95;--line:#2C3A41;--real:#6FC3CE;--sim:#F08A83;--real-soft:#183A41;--sim-soft:#3E1B19;--ok:#69C596;--warn:#E5A44A;color-scheme:dark}
body{background:var(--bg);color:var(--ink);font-family:var(--sans);font-size:16px;line-height:1.55;padding-inline:20px;padding-block:32px 64px;margin:0}
.wrap{max-width:1040px;margin:0 auto;display:grid;gap:40px}
h1,h2,h3{font-family:var(--disp);font-weight:600;line-height:1.15;text-wrap:balance;margin:0}
h1{font-size:2.3rem} h2{font-size:1.45rem;padding-top:6px;border-top:2px solid var(--ink)} h3{font-size:1.1rem}
p{margin:0;max-width:72ch} .eyebrow{font-family:var(--mono);font-size:.78rem;letter-spacing:.08em;text-transform:uppercase;color:var(--ink3)}
header{display:grid;gap:14px} .meta{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:6px 22px;font-family:var(--mono);font-size:.82rem;color:var(--ink2)} .meta b{color:var(--ink);font-weight:500}
td.bad,.bad{color:var(--warn);font-weight:600}section{display:grid;gap:16px} ul,ol{margin:0;padding-left:1.2em;display:grid;gap:8px} li::marker{color:var(--ink3)}
.tscroll{overflow-x:auto} table{border-collapse:collapse;width:100%;font-size:.88rem;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:7px 9px;border-bottom:1px solid var(--line);vertical-align:top}
thead th{font-family:var(--mono);font-size:.72rem;letter-spacing:.06em;text-transform:uppercase;color:var(--ink3);font-weight:500}
tbody th{font-weight:500} td.num{font-family:var(--mono);white-space:nowrap;text-align:right} td.s{white-space:nowrap}
tr.real td{border-bottom:none} tr.sim td{color:var(--ink2)}
code{font-family:var(--mono);font-size:.86em;background:var(--real-soft);padding:1px 5px;border-radius:3px;color:var(--ink)}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}
.tile{background:var(--panel);border:1px solid var(--line);padding:14px 16px;display:grid;gap:6px;border-left:5px solid var(--line)}
.tile .lab{font-family:var(--mono);font-size:.72rem;letter-spacing:.06em;text-transform:uppercase;color:var(--ink3)}
.tile .val{font-family:var(--disp);font-size:1.3rem;font-weight:600} .tile .why{font-size:.88rem;color:var(--ink2)}
.tile.real{border-left-color:var(--real)} .tile.sim{border-left-color:var(--sim)} .tile.ok{border-left-color:var(--ok)} .tile.warn{border-left-color:var(--warn)}
.legend{display:flex;gap:18px;font-family:var(--mono);font-size:.8rem;color:var(--ink2);flex-wrap:wrap}
.legend i{display:inline-block;width:22px;height:0;border-top:3px solid;vertical-align:middle;margin-right:6px}
.figs{display:grid;grid-template-columns:repeat(auto-fit,minmax(460px,1fr));gap:14px}
@media (max-width:520px){.figs{grid-template-columns:1fr}}
.fig{background:var(--panel);border:1px solid var(--line);padding:6px 4px 2px;min-width:0}
.fig .ttl{font-family:var(--mono);font-size:.76rem;color:var(--ink2);padding:2px 10px}
.fig .plot{width:100%;height:230px}
.note{border-left:3px solid var(--real);padding:10px 14px;background:var(--panel);color:var(--ink2);font-size:.92rem}
.small{font-size:.86rem;color:var(--ink2)}
"""

JS = """
const CSSV=(n)=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function draw(f){
  const real=CSSV('--real'), sim=CSSV('--sim'), ink=CSSV('--ink2'), line=CSSV('--line'), panel=CSSV('--panel');
  const tr=[{x:f.real.x,y:f.real.y,name:'real flight',mode:'lines',line:{color:real,width:1.8},connectgaps:false,hovertemplate:'real %{y:.4g}<extra></extra>'},
            {x:f.sim.x,y:f.sim.y,name:'sim (mirror)',mode:'lines',line:{color:sim,width:1.6},connectgaps:false,hovertemplate:'sim %{y:.4g}<extra></extra>'}];
  if(f.real_ref){tr.push({x:f.real_ref.x,y:f.real_ref.y,name:'reference',mode:'lines',line:{color:ink,width:1,dash:'dot'},connectgaps:false,hoverinfo:'skip',showlegend:false});}
  const shapes=f.seps.map(s=>({type:'line',x0:s,x1:s,y0:0,y1:1,yref:'paper',line:{color:line,width:1}}));
  const ann=f.ticks.map(t=>({x:t[0],y:1.02,yref:'paper',text:t[1],showarrow:false,font:{size:10,color:ink},xanchor:'center'}));
  Plotly.newPlot(f.id,tr,{margin:{l:52,r:8,t:26,b:34},paper_bgcolor:panel,plot_bgcolor:panel,showlegend:false,
    font:{family:'IBM Plex Mono, monospace',size:10,color:ink},xaxis:{title:{text:'phase-relative time [s] (plant time)',standoff:4},gridcolor:line,zeroline:false},
    yaxis:{gridcolor:line,zeroline:false},shapes:shapes,annotations:ann,hovermode:'x'},{displaylogo:false,responsive:true,displayModeBar:false});
}
window.addEventListener('load',()=>{if(typeof Plotly==='undefined'){document.querySelectorAll('.fig .plot').forEach(e=>e.textContent='(Plotly did not load — figures unavailable offline)');return;} FIGS.forEach(draw);});
"""



# ------------------------------------------------------------- extra sections
def _load(name):
    fp = os.path.join(AN, name)
    return json.load(open(fp)) if os.path.exists(fp) else None


def audit_section(n):
    a = _load("arm_comp_audit.json")
    if not a:
        return ""
    labels = {"f18": "0918 mission", "a1": "0924 F1", "a2": "0924 F2", "a3": "0924 F3 (12 s)", "a4": "0924 F4 (6 s)"}
    rows = []
    for fk, lab in labels.items():
        f = a["flights"].get(fk)
        if not f:
            continue
        for j in ("j2", "j3"):
            v = f["joints"][j]
            rows.append(f"<tr><td class=s>{lab}</td><td class=s>{j}</td><td class=num>{v['frac_moving']*100:.0f}</td>"
                        f"<td class=num>{v['applied_minus_intended_mean']:+.3f} / {v['applied_minus_intended_rms']:.3f}</td>"
                        f"<td class=num>{v['kinetic_level_median']:+.3f}</td><td class=num>{v['ff_level_at_load_median']:.3f}</td>"
                        f"<td class=num>{v['static_residual_std']:.3f}</td><td class=num>{v['dhat_q_mean']:+.3f}</td>"
                        f"<td class=num>{v['v_abs_mean_deg_s']:.1f}</td></tr>")
    tbl = ("<div class=tscroll><table><thead><tr><th>flight</th><th>joint</th><th>moving %</th><th>applied − intended mean / rms [N·m]</th>"
           "<th>kinetic friction level, median [N·m]</th><th>FF level at that load [N·m]</th><th>hold residual std [N·m]</th><th>d̂_q mean [N·m]</th><th>|q̇| mean [°/s]</th></tr></thead><tbody>"
           + "".join(rows) + "</tbody></table></div>")
    rec = a["recommend"]
    return f"""
<section>
  <h2>{n}. Arm-channel compensation audit (0918, 0924 F1–F4)</h2>
  <p>Three compensation terms act on the streamed joint torque <code>u3</code> before it reaches the servo: the gravity-scale correction <code>(S−1)·g(q)</code>, the friction feed-forward <code>(f_c + µ|τ|)·tanh(q̇_ref/w)</code>, and, inside the law, the observer's joint-row estimate <code>d̂_q</code> and the internal feed-forward. The audit compares, tick by tick, the torque the servo <em>applied</em> (Present Current × K_t) with the torque the chain <em>intended</em>, then back-solves the real kinetic friction from <code>applied − S·g(q,R₀) − J·q̈</code> on the moving samples and reads the standing residual in the holds.</p>
  {tbl}
  <ul>
    <li><b>The pipeline delivers what it intends.</b> <code>applied − intended</code> has a mean of ±0.01 N·m on 0918 / F3 / F4 (rms 0.06–0.14 on j2); F1/F2 show ±0.1 N·m offsets that coincide with their lower move fraction and the stick phases. The calibration (κ, S) is consistent with the flights — nothing to correct there.</li>
    <li><b>The friction feed-forward is 30–35 % too high on j2/j3 and its structure is wrong while the joint sticks.</b> Real kinetic level on j2 is 0.05–0.17 N·m against a 0.19–0.20 N·m feed-forward, on j3 0.00–0.09 against 0.11; pooled free fits are ill-conditioned (negative f_c with µ fixed), so the ratio is trusted, not the split. The relay fires on the <em>reference</em> velocity, so during a stick phase it pushes at full level while the joint is stationary, and it is exactly zero at a hold although the joint parks 1.5–3° short — the 0924 F3/F4 stick–slip signature.</li>
    <li><b>Correction applied (hardware and Isaac arm yamls, UNFLOWN):</b> j2 ×0.70 ({rec['j2']['as_flown_fc']:.4f} → {rec['j2']['as_flown_fc']*0.7:.4f} N·m, µ 0.246 → 0.172), j3 ×0.65 ({rec['j3']['as_flown_fc']:.4f} → {rec['j3']['as_flown_fc']*0.65:.4f}, µ 0.161 → 0.105), and a new <code>friction_velocity_source: measured</code> (width 0.03 rad/s) so the relay follows the joint's own motion; j1/j4 are left alone (their residual is at the current-sensor floor). The mirror plant carries the same ratios (<code>sim_arm_friction_scale_j2/j3</code> 0.70/0.65). Loopback test 24/24.</li>
    <li><b>The joint-row observer estimate is small and consistent</b>: <code>d̂_q</code> on j2 +0.02…+0.08 N·m, j3 +0.01…+0.08, i.e. the residual gravity/friction error the law absorbs is under 10 % of the 0.7 N·m hold torque. The J_arm structure (now joint-diagonal with the bench values) and the faster velocity observer are in the hardware yaml and mirrored into the sim's section 2; neither was flown on 0918–0924, so the audit's numbers are the as-flown model's.</li>
    <li><b>Not changed, on purpose:</b> a breakaway (static) term, a gravity-integral in the pass-through (the L1 arm channel already integrates), and a velocity-dependent j4 back-EMF bias — each needs a bench measurement the flight data cannot separate.</li>
  </ul>
</section>"""


def _sweep_rows(d, names=None):
    rows = []
    for nm, r in d.items():
        if names is not None and nm not in names:
            continue
        if r["verdict"] != "completed":
            rows.append(f"<tr><td class=s>{nm}</td><td colspan=8 class=bad>ABORT at {r['t_end']:.1f} s</td></tr>")
            continue
        je = r["joint_err_rms"]
        rows.append(f"<tr><td class=s>{nm}</td><td class=num><b>{r['ee_abs_rms']*1e3:.1f}</b> / {r['ee_abs_peak']*1e3:.0f}</td><td class=num>{r['com_rms']*1e3:.1f} / {r['com_peak']*1e3:.0f}</td>"
                    f"<td class=num>{r['head_rms']:.2f}</td><td class=num>{r['eR_mean']:.4f}</td><td class=num>{r['tilt_peak']:.1f}</td><td class=num>{r['tau_peak']:.2f}</td><td class=num>{r['sat_pct']:.1f}</td><td class=num>{je[1]:.2f} / {je[2]:.2f}</td></tr>")
    return ("<div class=tscroll><table><thead><tr><th>configuration</th><th>EE absolute rms / peak [mm]</th><th>CoM rms / peak [mm]</th><th>EE heading rms [°]</th><th>|e_R| mean</th><th>tilt peak [°]</th><th>arm τ peak [N·m]</th><th>clamp %</th><th>q2 / q3 err rms [°]</th></tr></thead><tbody>"
            + "".join(rows) + "</tbody></table></div>")


def _gate_table(d):
    """One row per candidate, the gate's tests as columns."""
    cands = []
    for k in d:
        nm = k.split(" | ")[0]
        if nm not in cands:
            cands.append(nm)
    cands.sort(key=lambda c: (not c.startswith("hw"), c))
    def cell(nm, test, what):
        r = d.get(f"{nm} | {test}")
        if r is None:
            return "<td class=num>—</td>"
        if r["verdict"] != "completed":
            return f"<td class='num bad'>ABORT {r['t_end']:.0f} s</td>"
        if what == "ee":
            return f"<td class=num><b>{r['ee_abs_rms']*1e3:.1f}</b></td>"
        if what == "gust":
            return f"<td class=num>+{r['gust_rise']*1e3:.0f} / {r['gust_recover']:.1f}</td>"
        if what == "entry":
            return f"<td class=num>{r['entry_peak_com']*1e3:.0f} / {r['entry_t30']:.1f}</td>"
    rows = []
    for nm in cands:
        rows.append(f"<tr><td class=s>{nm}</td>"
                    + cell(nm, "mirror 16 ms +gust", "ee") + cell(nm, "mirror 16 ms +gust", "gust")
                    + cell(nm, "mirror 16 ms +gust", "entry") + cell(nm, "mirror 28 ms", "ee") + cell(nm, "mirror 32 ms", "ee")
                    + cell(nm, "robustness 16 ms +gust", "ee") + cell(nm, "robustness 16 ms +gust", "gust")
                    + cell(nm, "robustness 16 ms +gust", "entry") + cell(nm, "robustness 24 ms", "ee") + "</tr>")
    return ("<div class=tscroll><table><thead>"
            "<tr><th rowspan=2>candidate</th><th colspan=5>MIRROR plant (identified from the flights)</th><th colspan=4>ROBUSTNESS plant (_sim_robustness stress)</th></tr>"
            "<tr><th>EE abs rms, circle + 3 N gust, 16 ms [mm]</th><th>gust rise / recovery [mm / s]</th><th>entry peak / to &lt;30 mm [mm / s]</th>"
            "<th>EE abs rms at 28 ms delay [mm]</th><th>EE abs rms at 32 ms delay [mm]</th>"
            "<th>EE abs rms, circle + gust, 16 ms [mm]</th><th>gust rise / recovery</th><th>entry peak / to &lt;30 mm</th><th>EE abs rms at 24 ms [mm]</th></tr>"
            "</thead><tbody>" + "".join(rows) + "</tbody></table></div>"
            "<p class=small>Candidates: K1 k_w 1.1 + ω_c_t 4 · K2 k_w 0.9 + ω_c_t 4 · K3 k_w 1.1 + ω_c_t 6 · K4 k_w 0.9 + ω_c_t 6 · K5 K1 + k_R 2.5 · K6 K1 + k_x 40/k_v 22.4. "
            "Entry = DIRECT handover with the observer starting from zero (the circle's first seconds); gust = 3 N world-x step for 3 s, 10 s into the circle. "
            "Horizontal part of the robustness entry peak: shipped 436 mm / 5.6° tilt, K4 539 mm / 9.6° — inside the 0.75 m drift and 20° tilt guards.</p>")


def _isaac_table():
    import glob, re
    cand_of = {}
    for lf in glob.glob(os.path.join(HERE_LOGS, "gain_isaac*.log")):
        for m in re.finditer(r"g_mir_cand_(\d+): profile mirror gains \[([^\]]*)\]", open(lf).read()):
            cand_of[m.group(1)] = m.group(2)
    fs = sorted(glob.glob(os.path.join(AN, "gain_isaac_*.json")))
    if not fs:
        return "<p class=small>Isaac confirmation flights: pending.</p>"
    rows = []
    for fp in fs:
        d = json.load(open(fp))
        for k, r in d.items():
            m = re.search(r"g_(mir|rob)_(hw|cand)_(\d+)", k)
            plant = "mirror" if m.group(1) == "mir" else "robustness"
            law = "shipped" if m.group(2) == "hw" else (cand_of.get(m.group(3), "candidate").replace("wb_", "").replace("l1_", ""))
            if "ee_abs_rms_mm" not in r:
                rows.append(f"<tr><td class=s>{plant}</td><td class=s>{law}</td><td colspan=8 class=bad>{r}</td></tr>"); continue
            rows.append(f"<tr><td class=s>{plant}</td><td class=s>{law}</td><td class=num>{m.group(3)}</td><td class=num>{r['rtf']:.2f}</td>"
                        f"<td class=num>{'<span class=bad>ABORTED</span>' if r['aborted_in_run'] else 'completed'}</td>"
                        f"<td class=num><b>{r['ee_abs_rms_mm']:.1f}</b> / {r['ee_abs_peak_mm']:.0f}</td><td class=num>{r['com_rms_mm']:.1f} / {r['com_peak_mm']:.0f}</td>"
                        f"<td class=num>{r['eR_mean']:.4f}</td><td class=num>{r['tilt_peak_deg']:.1f}</td>"
                        f"<td class=num>{r.get('entry_com_peak_mm', float('nan')):.0f}</td></tr>")
    rows.sort(key=lambda x: x)
    return ("<div class=tscroll><table><thead><tr><th>plant</th><th>law</th><th>chain</th><th>RTF</th><th>circle run</th><th>EE absolute rms / peak [mm]</th><th>CoM rms / peak [mm]</th>"
            "<th>|e_R| mean</th><th>tilt peak [°]</th><th>entry CoM peak [mm]</th></tr></thead><tbody>" + "".join(rows) + "</tbody></table></div>")


def tuning_section(n):
    sw = _load("circle_sweep.json"); kw = _load("circle_kw.json"); fg = _load("circle_final_gate.json")
    if not sw:
        return ""
    t1 = _sweep_rows(sw)
    t_kw = _sweep_rows(kw) if kw else ""
    t_fg = _gate_table(fg) if fg else ""
    return f"""
<section>
  <h2>{n}. Whole-body gain tuning on the circle bench</h2>
  <p><b>Goal.</b> Smaller end-effector <em>absolute</em> error and faster full-state response on a 0.5 m radius circle at 0.13 m/s (24 s lap). On the 0924 flights the absolute EE error is the base CoM error plus a few millimetres of task error (F2: 55.9 mm EE absolute, 7.0 mm task), so the base loops are the target.</p>
  <p><b>Plan.</b> (1) Screen every gain group offline, one at a time, with the recorded 0924 F2 reference stream driving the exact Python law (the C++ port's source of truth: 4-D L1 observer, ω_x 0.25, K_y 80/24, joint-diagonal armature, rescaled M_r_d) against the <b>mirror</b> plant at RTF 1, the clock the hardware has. (2) Find the mechanism behind the residual so the lever is chosen, not guessed. (3) Gate every candidate on transport-delay margin, a 3 N gust, the DIRECT-entry handover and the <b>robustness</b> plant — the <code>_sim_robustness</code> stress configuration (+17.6 % kf belief, mass/inertia ×1.10, CoM 10/10/5 mm, arm ×1.05) with the tuned hardware law on it. (4) Confirm the winner in Isaac on both plants. (5) Hardware, one change per flight, repeat-tested.</p>

  <h3>Step 1 — one group at a time (offline, mirror plant, RTF 1)</h3>
  {t1}
  <p>The bench is sensitive to the translational observer bandwidth and the attitude/position stiffnesses, not to the EE-side gains (K_y, K_ψ, ω_x). k_x 64+ and k_R 4 already diverge against the 16 ms transport delay plus the 100 ms rotor lag, and the improvements do not stack (k_R 3 with ω_c_r 1 rings at 11° tilt).</p>

  <h3>Step 2 — the mechanism: a body-fixed force on a yawing trajectory</h3>
  <p>Ablating the mirror plant one term at a time: removing the identified <b>body-fixed lateral force (+0.55, −0.50) N</b> drops the circle's EE absolute error from 23.5 to 1.9 mm; removing the yaw-torque bias, the yaw-coefficient mismatch or the arm friction changes nothing. The same force fixed in the <em>world</em> frame costs 3.9 mm. The observer is not the problem — it estimates the force to 0.07 N rms. The problem is the law's thrust-direction rate: <code>f_d_dot = −k_x e_v − k_v e_a + m x_cd⁽³⁾</code> treats <code>d̂_t</code> as constant, but a body-fixed force rotates in the world at the heading rate (0.26 rad/s on this circle). The commanded angular velocity <code>ω_0c</code> then carries a spurious 0.014 rad/s roll/pitch rate the body never has; the attitude loop parks at <code>e_R ≈ (k_w/k_R)·e_ω</code> = 0.014 rad (measured 0.0153), and the pure-P position loop turns that into <code>T·e_R/k_x</code> ≈ 18 mm. On the real 0924 F2 flight the same identity holds with |e_R| = 0.042: 36.7 × 0.042 / 32 = 48 mm against the measured 56 mm CoM error.</p>
  <p>So the right levers are the ratio <b>k_w/k_R</b> and the translational observer bandwidth <b>ω_c_t</b>. A structural what-if — feeding the estimate's own rate into <code>f_d_dot</code> — recovers only 18 % and costs delay margin, so it is not pursued as a gain-tuning step.</p>
  {t_kw}
  <p>Lowering k_w is a double win: less error <em>and</em> more delay margin (1.5 aborts at 24 ms; 1.3, 1.1 and 0.9 do not), consistent with the August stability screening that found raising k_w costs margin. Attitude damping at k_w 0.9 is ζ ≈ 0.88 (1.47 before), still well damped.</p>

  <h3>Step 3 — the final gate: delay, gust, entry handover, robustness plant</h3>
  {t_fg}
  <p><b>Offline verdict.</b> K4 (k_w 0.9 + ω_c_t 6) is the best offline: on the mirror plant it more than halves the circle error (23.5 → 10.5 mm) and the error under a 3 N gust (67.7 → 34.8 mm), rejects the gust 40 % faster (+108 mm / 2.8 s vs +183 mm / 4.3 s), cuts the entry peak 53 → 33 mm and survives a 28 ms delay where the shipped law aborts; on the robustness plant it passes every test, at the cost of a larger entry handover (539 vs 436 mm horizontal, 9.6° vs 5.6° tilt). K5 (k_R 2.5) keeps that handover but gives back the delay margin. Both K4 and k_w 0.9 alone went to Isaac.</p>
  <h3>Step 4 — Isaac confirmation (mirror and robustness plants, the 0924 F2 circle)</h3>
  {_isaac_table()}
  <p class=small>Isaac runs at RTF ≈ 0.5 against a wall-clock law, which inflates every attitude-driven error (section on DIRECT-mode mismatches); read the Isaac rows as a relative comparison and as the pass/fail of the robustness plant, and the offline rows for the absolute size of the effect.</p>

  <p><b>Isaac verdict.</b> Both chains completed every flight, with no abort, zero rotor saturation and zero joint clamp; the shipped baseline repeats to within 2–3 mm (81.2 / 83.3 mirror, 86.1 / 86.5 robustness). <b>k_w 0.9 alone improves BOTH plants</b>: mirror 83.3 → 63.5 mm (−24 %), robustness 86.5 → 68.0 mm (−21 %), |e_R| 0.10 → 0.06 on both, tilt peak unchanged; its cost is the predicted larger entry handover on the robustness plant (360 → 427 mm). <b>K4 helps the mirror less than k_w alone (69.7 mm) and makes the robustness circle worse</b> (104 vs 86 mm, peak 185 vs 112 mm, a ringing 0.38 Hz attitude mode). Offline, doubling every observer bandwidth to emulate the wall-clock filters does <em>not</em> reproduce that ringing, so its source is another part of the RTF 0.49 wall-clock coupling (the observer integrates wall-clock steps over plant-time momentum changes; EKF2 runs non-lockstep) — not isolated, and not ignorable either: the robustness test is the gate the user set.</p>
  <h3>Step 5 — recommendation and hardware sequence (NOT flown)</h3>
  <ol>
    <li><b>Adopt <code>wb_k_w</code> 1.5 → 0.9</b> (attitude damping ζ 1.47 → 0.88) — <em>applied 2026-09-27, then superseded the same day by section 8's joint tune H1b, which moves k_w with every other gain group; do not revert k_w alone.</em> The only change that improves both plants offline <em>and</em> in Isaac; it also adds delay margin (offline: survives 24–28 ms where 1.5 aborts) and adds no noise bandwidth. Fly hover, then the circle, twice each; score with <code>tools/score_run.py --real</code>.</li>
    <li><b>Hold <code>wb_l1_omega_c_t</code> at 2.0 for now.</b> Raising it to 4–6 is the next-largest offline lever (the body-fixed force is tracked faster as it rotates), but it degraded the Isaac robustness circle and it passes more of the hardware's noisy deadbeat estimate into f_d. Re-test it once the Isaac rig runs the law on simulated time, or on hardware as a separate, repeated step after k_w with ripple watched.</li>
    <li>Structural follow-up (a law change, not a gain): the rate of the disturbance estimate is missing from <code>f_d_dot</code>. A body-frame translational estimate, or its rotation rate ω × d̂ fed forward, attacks the mechanism directly; the naive finite-difference what-if recovered only 18 %.</li>
  </ol>
</section>"""



# ------------------------------------------------ circle tune round 2 (0927)
CT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "circle_tune_20260927", "analysis")


def _ct(name):
    fp = os.path.join(CT, name)
    return json.load(open(fp)) if os.path.exists(fp) else None


def pareto_fig():
    a = _ct("grid_shipped_cal.json"); b = _ct("grid_tuned_H1b.json")
    if not (a and b):
        return None
    series = []
    for lab, g, role in (("shipped", a["res"], "ship"), ("tuned H1b", b["res"], "tuned")):
        for r in ("050", "075"):
            for cw in (False, True):
                pts = []
                for lap in (24, 28, 32, 40):
                    n = f"r{r}_L{lap}_sync15" + ("_cw" if cw else "")
                    if n in g and g[n]["verdict"] == "completed":
                        pts.append((g[n]["ee_speed_mean"], g[n]["ee_rms"] * 1e3, lap))
                if len(pts) >= 2:
                    series.append({"name": f"{lab}, r {int(r)/100:.2f} m, {'CW' if cw else 'CCW'}", "role": role,
                                   "cw": cw, "r": r, "x": [p[0] for p in pts], "y": [p[1] for p in pts],
                                   "lap": [p[2] for p in pts]})
    return series


PARETO_JS = """
function drawPareto(S){
  const el=document.getElementById('pareto'); if(!el||!S) return;
  const c={ship:CSSV('--ink3'),tuned:CSSV('--sim')}, ink=CSSV('--ink2'), line=CSSV('--line'), panel=CSSV('--panel');
  const tr=S.map(s=>({x:s.x,y:s.y,name:s.name,mode:'lines+markers',
     line:{color:c[s.role],width:2,dash:s.cw?'solid':'dash'},
     marker:{color:c[s.role],size:9,symbol:s.r==='075'?'circle':'square',line:{color:panel,width:2}},
     customdata:s.lap,hovertemplate:s.name+'<br>lap %{customdata} s · %{x:.3f} m/s<br>EE abs rms %{y:.1f} mm<extra></extra>'}));
  Plotly.newPlot(el,tr,{margin:{l:56,r:10,t:10,b:44},paper_bgcolor:panel,plot_bgcolor:panel,
    font:{family:'IBM Plex Mono, monospace',size:11,color:ink},legend:{orientation:'h',y:-0.22,font:{size:10}},
    xaxis:{title:{text:'mean EE speed [m/s]',standoff:6},gridcolor:line,zeroline:false},
    yaxis:{title:{text:'EE absolute error, rms [mm]',standoff:6},gridcolor:line,zeroline:false,rangemode:'tozero'},hovermode:'closest'},
    {displaylogo:false,responsive:true,displayModeBar:false});
}
window.addEventListener('load',()=>{if(typeof Plotly!=='undefined'){drawPareto(PARETO);}});
"""


def circle2_section(n):
    fg = _ct("final_gate_final.json"); abl = _ct("group_ablation.json"); H = _ct("tuned_H1b.json")
    if not (fg and H):
        return ""
    import glob
    import numpy as _np
    P = H["best"]; B = {"k_x": 32.0, "k_v": 20.0, "k_R": 2.0, "k_w": 0.9, "mrd_s": 1.0, "ky": 80.0, "dy": 24.0,
                        "ky_psi": 0.3, "dy_psi": 0.3, "omega_c_t": 2.0, "omega_c_r": 0.5, "omega_c_q": 0.5, "omega_x": 0.25}
    lab = {"k_x": ("translational", "wb_k_x"), "k_v": ("translational", "wb_k_v"), "k_R": ("orientational", "wb_k_r"),
           "k_w": ("orientational", "wb_k_w"), "mrd_s": ("orientational", "wb_mrd_x/y/z (× the shipped vector)"),
           "ky": ("arm channel", "wb_ky_x/y/z"), "dy": ("arm channel", "wb_dy_x/y/z"), "ky_psi": ("arm channel", "wb_ky_psi"),
           "dy_psi": ("arm channel", "wb_dy_psi"), "omega_c_t": ("L1", "wb_l1_omega_c_t"), "omega_c_r": ("L1", "wb_l1_omega_c_r"),
           "omega_c_q": ("L1", "wb_l1_omega_c_q"), "omega_x": ("L1", "wb_l1_omega_x")}
    gtab = ("<div class=tscroll><table><thead><tr><th>group</th><th>yaml key</th><th>before (k_w 0.9)</th><th>H1b (applied)</th><th>change</th></tr></thead><tbody>"
            + "".join(f"<tr><td class=s>{lab[k][0]}</td><td><code>{lab[k][1]}</code></td><td class=num>{B[k]:g}</td>"
                      f"<td class=num><b>{P[k]:.4g}</b></td><td class=num>{100*(P[k]/B[k]-1):+.0f} %</td></tr>" for k in B)
            + "</tbody></table></div>")
    # final gate table
    rows = []
    for cn in ("shipped", "F3", "H1", "H1b"):
        R = fg["res"].get(cn)
        if not R:
            continue
        ok = lambda t: R[t]["verdict"] == "completed"
        e = lambda t: R[t]["ee_rms"] * 1e3
        sh = _np.mean([e(f"show_s{k}") for k in (11, 12, 13)]); fl = _np.mean([e(f"flown_s{k}") for k in (11, 12, 13)])
        dl = " · ".join((f"{e(f'delay{d}'):.0f}" if ok(f"delay{d}") else "<span class=bad>✕</span>") for d in (24, 28, 30))
        g = R["gust"]
        ir = [R[f"isaac_rob_s{sd}"] for sd in (0, 21, 22)]
        irs = ("<span class=bad>aborts 3/3</span>" if all(x["verdict"] != "completed" for x in ir) else
               f"{min(x['ee_rms'] for x in ir)*1e3:.0f}–{max(x['ee_rms'] for x in ir)*1e3:.0f} / "
               f"{min(x['hold_com_rms'] for x in ir)*1e3:.0f}–{max(x['hold_com_rms'] for x in ir)*1e3:.0f}")
        rows.append(f"<tr><td class=s>{'<b>H1b</b>' if cn == 'H1b' else cn}</td><td class=num><b>{sh:.1f}</b></td><td class=num>{fl:.1f}</td><td class=num>{dl}</td>"
                    f"<td class=num>+{g['gust_rise']*1e3:.0f} / {g['gust_rec']:.1f}</td><td class=num>{e('rob16'):.1f} / {R['rob16']['entry_pk']*1e3:.0f}</td>"
                    f"<td class=num>{irs}</td><td class=num>{e('isaac_show'):.0f} / {e('isaac_flown'):.0f}</td></tr>")
    ftab = ("<div class=tscroll><table><thead><tr><th>gains</th><th>showcase EE, 3 seeds [mm]</th><th>flown circle EE [mm]</th>"
            "<th>showcase EE at 24 · 28 · 30 ms delay</th><th>3 N gust rise / recovery [mm / s]</th><th>robustness plant: circle / entry [mm]</th>"
            "<th>robustness plant under the ISAAC clock: EE / hover CoM, 3 seeds [mm]</th><th>Isaac-clock showcase / flown [mm]</th></tr></thead><tbody>"
            + "".join(rows) + "</tbody></table></div>")
    # Isaac flights
    lawname = {"ship": "shipped (k_w 0.9)", "f3": "F3", "h1b": "H1b", "h1": "H1"}
    irows = []
    for fp in sorted(glob.glob(os.path.join(CT, "isaac_*.json"))):
        for k, r in json.load(open(fp)).items():
            base = k.replace("isaac_", "").rsplit("_", 1)[0].replace(".npz", "")
            law = lawname[[x for x in ("h1b", "h1", "f3", "ship") if base.startswith(x)][0]]
            plant = "robustness" if "rob" in base else "flight-matched"
            circ = "showcase (0.75 m, CW)" if "show" in base else "flown (0.5 m, CCW)"
            if "ee_abs_rms_mm" not in r:
                continue
            bad = r["n_sat"] > 0 or r["n_clamp"] > 0 or r["aborted_in_run"] or r["run"][1] - r["run"][0] < 20
            verdict = ("<span class=bad>failed: circle never started</span>" if r["run"][1] - r["run"][0] < 20 else
                       ("<span class=bad>arm clamp + rotor saturation</span>" if r["n_sat"] > 0 else "completed, 0 sat / 0 clamp"))
            eecell = "—" if r["run"][1] - r["run"][0] < 20 else f"<b>{r['ee_abs_rms_mm']:.1f}</b>"
            irows.append((plant, circ, law, f"<tr><td class=s>{plant}</td><td class=s>{circ}</td><td class=s>{'<b>H1b</b>' if law == 'H1b' else law}</td>"
                          f"<td class=num>{eecell}</td><td class=num>{r['ee_task_rms_mm']:.1f}</td>"
                          f"<td class=num>{r['eR_mean']:.3f}</td><td class=num>{r['tilt_peak_deg']:.1f}</td><td class=num>{r.get('entry_com_peak_mm', float('nan')):.0f}</td><td>{verdict}</td></tr>"))
    order = {"shipped (k_w 0.9)": 0, "F3": 1, "H1": 2, "H1b": 3}
    irows.sort(key=lambda x: (x[0] != "flight-matched", x[1], order[x[2]]))
    itab = ("<div class=tscroll><table><thead><tr><th>Isaac plant</th><th>circle</th><th>gains</th><th>EE abs rms [mm]</th><th>EE task [mm]</th>"
            "<th>|e_R| mean</th><th>tilt peak [°]</th><th>entry peak [mm]</th><th>verdict</th></tr></thead><tbody>" + "".join(x[3] for x in irows) + "</tbody></table></div>")
    atab = ""
    if abl:
        SHOW, FLOWN = "r075_L24_sync15_cw", "r050_L24_half10"
        f2 = [k for k in abl["cfg"] if k.startswith("F2 (")][0]
        order2 = (["shipped"] + [k for k in abl["cfg"] if k.startswith("shipped +")] + [f2]
                  + [k for k in abl["cfg"] if k.startswith("F2 -")] + [k for k in abl["cfg"] if "M_r_d" in k])
        rr = []
        for nm in order2:
            R = abl["res"][nm]
            e = lambda k: (f"{R[k]['ee_rms']*1e3:.1f}" if k in R and R[k]["verdict"] == "completed" else ("abort" if k in R else "—"))
            rr.append(f"<tr><td class=s>{nm.replace(' (J 21.9)', '')}</td><td class=num>{e(SHOW+'@16')}</td><td class=num>{e(FLOWN+'@16')}</td>"
                      f"<td class=num>{R[SHOW+'@16'].get('hold_eR', float('nan')):.4f}</td></tr>")
        atab = ("<div class=tscroll><table><thead><tr><th>configuration (stage-2 finalist F2)</th><th>showcase EE [mm]</th><th>flown EE [mm]</th>"
                "<th>hover |e_R|</th></tr></thead><tbody>" + "".join(rr) + "</tbody></table></div>")
    return f"""
<section>
  <h2>{n}. Circle tracking, round 2: all gain groups jointly, and the trajectory as a hyperparameter (2026-09-27)</h2>
  <p><b>Result.</b> The joint tune <b>H1b</b> is applied to the hardware and flight-matched yamls (the robustness yaml is untouched). On the hardware-clock bench it cuts the EE absolute error by 20–32 % on every one of 48 planner-generated circles (mean −25 %); on the recommended showcase circle — r 0.75 m, 24 s lap (0.18 m/s), clockwise, arm ±15° once per lap — it goes from 20.7 to 14.1 mm. In Isaac it completes the worst-case robustness flight with 24 % less EE error than the shipped law, and flies both flight-matched circles clean. Delay margin is the shipped law's. <b>Not flown on hardware.</b></p>
  <p><b>The bench.</b> <code>circle_tune_20260927/tools/circle_bench.py</code> flies the exact Python law (the C++ port's source of truth) against the flight-matched plant at RTF 1, on reference streams the <em>real</em> planner generates offline (<code>gen_circle_stream.py</code>, a loopback of fsc_trajectory_planner on the mirror yaml). The law sees hardware-like feedback: EKF odometry 20 ms late with band-limited velocity noise — <b>calibrated</b>: 2.0 cm/s reproduces the flights' hover |e_R| of 0.021 — attitude jitter, gyro noise, 12-bit encoders, and the arm observer's 12 ms lag. Every candidate sees the same noise; scoring is on the true state. It reproduces Isaac's k_w 1.5 → 0.9 gain independently.</p>

  <h3>The reference as a hyperparameter — 50 planner-generated circles</h3>
  <div class=fig><div class=ttl>EE absolute error vs mean speed · arm ±15° once per lap · 24/28/32/40 s laps · solid = clockwise, dashed = counter-clockwise</div><div class=plot id=pareto style="height:380px"></div></div>
  <ul>
    <li><b>The arm's motion does not matter</b>: static, ±10° half-cycle (flown), ±10° / ±15° once per lap and ±10° twice per lap land within 0.3 mm. Showcase the ±15° lap-synced sweep for free.</li>
    <li><b>Lap time dominates, radius barely matters</b>: the error comes from the heading turning once per lap (section 7's mechanism), so 0.75 m costs ≤ 1 mm over 0.5 m at the same lap — 1.5× the speed for free.</li>
    <li><b>Clockwise is 14–21 % better than counter-clockwise</b>, and not because of the identified biases: with every bias removed the asymmetry stays (7.2 vs 8.8 mm). It is the airframe's own asymmetry (the model's I<sub>xy</sub>, the arm's CoM offsets). All flights so far flew counter-clockwise. The body-fixed lateral force is 60 % of all the error (22.2 → 8.8 mm without it); the yaw-torque bias contributes nothing.</li>
    <li><b>Recommended showcase: r 0.75 m, 24 s lap (0.18 m/s), clockwise, arm ±15° once per lap</b> — planner keys <code>ee_traj_circle_radius 0.75</code>, <code>ee_traj_ccw false</code>, <code>ee_traj_q2_amp_deg 15</code>, <code>ee_traj_q2_period_s 24</code> (the yaml defaults were left as flown).</li>
  </ul>

  <h3>The search: CMA-ES over 13 gains, five stages</h3>
  <p>Log-space CMA-ES (λ = 12) over k_x, k_v, k_R, k_w, an M_r_d scale, K_y, D_y, K_ψ, D_ψ, ω_c,t, ω_c,r, ω_c,q, ω_x. Each candidate flies the two performance circles plus stress runs. Cost = mean EE rms + ¼ mean EE peak, plus penalties for aborts, tilt, saturation, actuator chatter and hover quality. Every stage was stopped or redirected by something the previous one found:</p>
  <ol>
    <li><b>Circle only</b> → 30 % better circles at k_w 0.3–0.4 (attitude ζ ≈ 0.33) that rang in hover. Hover |e_R| and CoM added to the cost.</li>
    <li><b>+ hover quality</b> → −35 %, but M_r_d pressed to its 0.6× bound: 1.5 mm bought with the delay margin (the 2026-08-23 lesson). The ablation below.</li>
    <li><b>M_r_d ≥ 0.9×, margin run at 30 ms</b> → F3: −34 %, more delay margin than shipped. <b>F3 then failed the Isaac robustness flight</b>: a sustained 0.45 Hz attitude limit cycle from the moment DIRECT engaged; the circle never started.</li>
    <li><b>The Isaac clock, modelled.</b> The whole-body node and its L1 observer run on the wall clock; at RTF 0.48 the predictor integrates plant-time momentum changes with the wall step, so d̂ = d − (1 − RTF)·ṗ — a positive acceleration feedback through C(s) that hardware does not have. With that in the bench (<code>rtf=0.48</code>: plant 0.48·dt per wall tick, reference derivatives per wall second, the arm observer's q̇ in wall units) it reproduces Isaac: shipped 104.7 vs 106.4 mm, F3 81.5 vs 80.9 mm, |e_R| 0.071/0.088 vs 0.069/0.088, and F3's robustness failure. The Isaac robustness run became a fifth gate.</li>
    <li><b>The consistent bar</b> — no worse than shipped on every gate (28 ms delay; the Isaac robustness run completes with hover, entry, circle error and tilt within 10 % of shipped and |e_R| bounded) → H1. <b>H1 then failed the flight-matched plant in Isaac</b>: its stiffer arm task (K_y 270, ω_c,q 1.10, ω_x 0.28) oscillated joints 2/3 at ~2.5 Hz onto the 3 N·m clamp once the circle started. The bench could not reproduce this — not with Isaac's 0.020 plant armature, the friction relay on Present Velocity, nor 24 ms of arm-command delay — so where the bench is blind the arm side was taken from F3, which Isaac had already flown clean: <b>H1b</b>. The arm group is worth only ±0.4 mm on the bench.</li>
  </ol>
  <h3>Where the improvement comes from (stage-2 finalist F2, one group at a time)</h3>
  {atab}
  <p>The translational group alone gives −20 % <em>and a quieter hover</em> (a stiffer, less damped position loop: k_v's velocity feedback carries the fused-odometry noise into the commanded tilt); orientational and L1 help 15–18 % each but alone cost hover quality; the groups overlap, so the full set is about −35 %, not the sum.</p>
  <h3>Final gate (bench)</h3>
  {ftab}
  <p class=small>✕ = aborted. Gust = 3 N world-x for 3 s, 10 s into the circle. Entry = DIRECT-handover CoM peak on the robustness plant (+17.6 % kf belief), where the observer starts from zero.</p>
  <h3>Isaac flights (RTF ≈ 0.48; read as relative, and as the pass/fail of each plant)</h3>
  {itab}
  <p class=small>Isaac scores faster circles much worse than the bench because the planner publishes reference derivatives per wall second while the plant moves per plant second — an error proportional to speed that hardware (RTF 1) does not have; section 5. H1b's smaller Isaac gain on the flight-matched plant is that artefact penalising the stiffer attitude response; its hardware-clock gain is the bench's.</p>
  <h3>Applied gains (hardware yaml and its _sim twin)</h3>
  {gtab}
  <p><b>Two sim findings worth acting on.</b> (1) The flight-matched yaml was meant to carry the bench-calibrated arm armature (<code>sim_arm_armature_j1..j4</code>, the law's own [0.010, 0.0194, 0.0097, 0.0097]) and does not, so every Isaac mirror flight has had PhysX's 0.020 on every joint; the bench says it barely matters, but the mirror should carry it. (2) The Isaac robustness failures of stronger tunes (K4 on 09-26, F3 here) are the wall-clock observer artefact above; running the whole-body node and planner on simulated time would remove it and let Isaac test what hardware will fly.</p>
</section>"""


def why_section(n):
    return f"""
<section>
  <h2>{n}. Why DIRECT-mode mismatches remain, and what was done about them</h2>
  <ul>
    <li><b>The clock.</b> The dominant DIRECT-mode residual on the circle (sim CoM 94 mm vs flight 56, |e_R| 0.10 vs 0.04, a constant −5.4° yaw lag) is the wall-clock controller against an RTF 0.48 plant: the planner's reference derivatives are in wall seconds while the plant's velocities are in plant seconds, so on a yawing circle the rate feed-forward is 2× too small and the attitude loop parks at <code>k_w·e_ω = k_R·sin(e_R)</code> — exactly the measured lag. Running Isaac headless (RTF 0.52) already moves the CoM error to 66 mm and |e_R| to 0.066. Force and torque terms are unaffected, which is why the 0918 hold/step replays match. The real fix is a code change (controller and planner on simulated time), not a plant parameter; the offline sweep below sidesteps it by running the law at RTF 1.</li>
    <li><b>Hover wander.</b> The flight's 25–30 mm, 0.1 Hz CoM wander in every hold does not come from the feedback noise — with the mocap noise on, the sim hover moves 4 mm. It scales with |e_R| and the pure-P position loop, i.e. it is a slow attitude-loop mode the mirror under-excites; the gain sweep targets exactly that term.</li>
    <li><b>Arm tracking.</b> The sim arm follows its reference 2–4× better than hardware (stick–slip is only approximated by one tanh level per joint). Per the request, the plant was not fitted further; the compensation terms were corrected from the audit instead.</li>
    <li><b>Model updates since the flights.</b> The hardware yaml now carries the joint-diagonal armature with the bench values, the faster joint-velocity observer, K_y 80 / D_y 24 and the rescaled M_r_d; the mirror's section 2 was updated to match, with the as-flown values kept in comments. The replays in sections 3–4 were flown with the as-flown values (<code>WB_REPLAY_ASFLOWN=1</code>) so they compare like with like.</li>
    <li><b>Inertia (X650 CAD on a T650 airframe).</b> The mass is the weighed 3.746 kg; the inertia is inherited. The 0921 same-command test measured roll/pitch gains of 1.11 / 1.03 relative to the CAD value at the folded home, so the coupled inertia is within ~10 %; the margin sweep carries a ±25 % inertia perturbation to bound the risk.</li>
  </ul>
</section>"""


def main():
    out = sys.argv[1]
    missions = []
    for arg in sys.argv[2:]:
        tag, title, path = arg.split("=", 2)
        missions.append((tag, title, json.load(open(path))))
    fit = json.load(open(os.path.join(AN, "fit_plant.json")))

    # --- identification table -------------------------------------------
    idrows = []
    for nm, lab, fb in ID_ROWS:
        r = fit.get(nm)
        if not r:
            continue
        hb = r["hover_balance"]
        kfe = r.get("kf_drift", {}).get("kf_at_entry")
        sag = r.get("kf_drift", {}).get("frac_per_min")
        sc = r["same_command"]
        dt = hb[0]["dhat_t"] if hb else [np.nan] * 3
        drz = hb[0]["dhat_r"][2] if hb else np.nan
        vl = r["arm_velocity_feedback"]["velocity_lag"]
        lag = np.mean([v["lag_s"] for v in vl.values()]) * 1e3 if vl else np.nan
        idrows.append(f"<tr><td class=s>{lab}</td><td class=s>{fb}</td><td class=num>{kfe*1e5:.3f}e-05</td><td class=num>{sag*100:+.1f}</td>"
                      f"<td class=num>{sc['thrust']['gain_dc']:.3f}</td><td class=num>{sc['roll']['gain']:.2f} / {sc['pitch']['gain']:.2f}</td><td class=num>{sc['roll']['corr']:.2f} / {sc['pitch']['corr']:.2f}</td>"
                      f"<td class=num>{sc['yaw_gain_bench_c']:.2f}</td><td class=num>{dt[0]:+.2f}, {dt[1]:+.2f}</td><td class=num>{drz:+.3f}</td><td class=num>{lag:.0f}</td></tr>")
    id_table = ("<div class=tscroll><table><thead><tr><th>flight</th><th>feedback</th><th>kf at entry</th><th>sag %/min</th><th>thrust DC gain</th>"
                "<th>roll / pitch gain</th><th>corr</th><th>yaw gain vs bench c</th><th>d̂_t x, y [N]</th><th>d̂_r z [N·m]</th><th>vel lag [ms]</th></tr></thead><tbody>"
                + "".join(idrows) + "</tbody></table></div>")

    # --- parameter table --------------------------------------------------
    prow = "".join(f"<tr><td class=s>{g}</td><td>{k}</td><td>{h}</td><td>{r}</td><td><b>{m}</b></td><td class=small>{s}</td></tr>"
                   for g, k, h, r, m, s in PARAM_ROWS)
    param_table = ("<div class=tscroll><table><thead><tr><th>group</th><th>parameter</th><th>controller believes (hardware yaml, as flown)</th>"
                   "<th>robustness sim plant (before)</th><th>mirror sim plant (now)</th><th>identified from</th></tr></thead><tbody>" + prow + "</tbody></table></div>")

    # --- missions ---------------------------------------------------------
    all_figs = []
    mission_html = []
    tiles = []
    for mi, (tag, title, cj) in enumerate(missions):
        ph = cj["phases"]
        dr, ds = cj["direct_real"], cj["direct_sim"]
        figs = figure_spec(ph, tag)
        all_figs += figs
        fig_html = "".join(f"<div class=fig><div class=ttl>{f['title']}</div><div class=plot id='{f['id']}'></div></div>" for f in figs)
        mission_html.append(f"""
<section>
  <h2>{mi + 3}. {title}</h2>
  <p class=small>Sim real-time factor {cj['sim_rtf']:.3f} (plant seconds per wall second); every sim trace and window is in plant time. Phases are the planner's own EXECUTING→HOLD legs, matched by order, so the k-th leg of the flight sits beside the k-th leg of the replay whatever the hold lengths were.</p>
  <div class=legend><span><i style="border-color:var(--real)"></i>real flight</span><span><i style="border-color:var(--sim)"></i>simulation, mirror plant</span><span><i style="border-color:var(--ink2);border-top-style:dotted"></i>reference (joint plots)</span></div>
  <div class=figs>{fig_html}</div>
  <h3>Whole-DIRECT statistics</h3>
  <div class=tscroll>{stats_table(ph, dr, ds)}</div>
  <h3>Per-phase statistics</h3>
  {phase_table(ph)}
</section>""")
        tiles.append(f"<div class='tile real'><div class=lab>{title.split('—')[0].strip()} · CoM rms</div><div class=val>{dr['com_err_norm_rms_mm']:.1f} → {ds['com_err_norm_rms_mm']:.1f} mm</div><div class=why>real → sim, whole DIRECT window; EE {dr['ee_task_norm_rms_mm']:.1f} → {ds['ee_task_norm_rms_mm']:.1f} mm, |e_R| {dr['eR_norm_mean']:.3f} → {ds['eR_norm_mean']:.3f}, d̂_z {dr['dhat_t_mean_n'][2]:+.2f} → {ds['dhat_t_mean_n'][2]:+.2f} N</div></div>")

    html = f"""<title>Sim-to-Real Flight Performance Tuning</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans+Condensed:wght@500;600&family=IBM+Plex+Sans:ital,wght@0,400;0,500;1,400&family=IBM+Plex+Mono:wght@400;500&display=swap">
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>{CSS}</style>
<div class=wrap>
<header>
  <div class=eyebrow>AM-T650 · whole-body L1-adaptive 4-D impedance control · IsaacSim mirror of the 0918 / 0921 / 0924 flights</div>
  <h1>Sim-to-Real Flight Performance Tuning</h1>
  <p>The simulator's plant was re-identified from the three hardware campaigns and written into a new mirror configuration, <code>…whole_body_l1_4d_direct_actuation_t650_sim.yaml</code>, whose controller section is the hardware yaml verbatim. The former stress-test file lives on unchanged as <code>…_sim_robustness.yaml</code>. Below: what changed and where each number came from, then the replayed missions beside the flights they mirror, with the sim-to-real error on every state; then why DIRECT-mode mismatches remain, an audit of the arm-channel compensation against the flights, and a gain-tuning study on the circle bench that found the mechanism behind the base error and a two-gain change that halves it while still passing the robustness plant.</p>
  <div class=meta><span>flights <b>0918 mission · 0921 #3 · 0924 F1–F4</b></span><span>identification <b>tools/fit_plant.py</b></span><span>replays <b>tools/replay.sh · compare.py</b></span><span>control gains <b>hardware, untouched</b></span></div>
</header>
<div class=tiles>{''.join(tiles)}</div>

<section>
  <h2>1. What was tuned, and from what</h2>
  <p>Control gains, observer settings, the allocator's believed thrust and yaw coefficients, the SAFETY thrust map and the guards are the hardware values; only the plant moved. Each row names the identification behind it.</p>
  {param_table}
  <div class=note>Four plant effects had no simulator knob before this work and were added to the Isaac plant (06) and its launcher: a yaw-coefficient scale separate from thrust, a battery-sag drift on kf, a body-fixed standing force/torque, and the Present-Velocity lag and quantum on the joint velocity the law reads; the mocap emulator gained a publish rate and measurement noise, and the servo model a per-joint friction scale.</div>
</section>

<section>
  <h2>2. Identification results per flight</h2>
  <p>Hover thrust balance from the law's own collective and the flown allocator belief; same-command test (recorded motor commands driven through the sim plant model against the measured gyro/accelerometer response, 0.3–8 Hz, coupled inertia at the folded home); observer residuals in the first hover hold; joint-velocity lag from the reported velocity against the differentiated reported position.</p>
  {id_table}
  <p class=small>Roll/pitch gains are only trustworthy where the correlation is high (0921 #3, a 148° heading spin); the circle flights barely excite roll/pitch above the vibration floor. The yaw gain reads the real vehicle's yaw acceleration per command as a fraction of the bench-c model; the sim's former 3.0× bench coefficient delivered 4–7× the real yaw response.</p>
</section>
{''.join(mission_html)}
{why_section(len(missions)+3)}
{audit_section(len(missions)+4)}
{tuning_section(len(missions)+5)}
{circle2_section(len(missions)+6)}
<section>
  <h2>{len(missions)+7}. What the mirror still cannot reproduce</h2>
  <ul>
    <li><b>Real-time factor.</b> Isaac and PX4 run at RTF ≈ 0.5 on this desktop while the law (250 Hz) and planner (100 Hz) run on the wall clock. Force and torque terms are time-invariant, but every filter — the L1 C(s), ω_x, the joint-row trim — is about twice as fast in plant time. The replays scale the planner's kinematic bounds and the EE-trajectory time scale by RTF so the plant sees the flown pace; nothing else can be corrected without running the controller in simulated time.</li>
    <li><b>Feedback noise.</b> The emulated mocap is the exact PhysX state at 250 Hz; the real feeds carried 1.2–2.9 cm/s of hover velocity noise (fused / raw 60 Hz) and 10 cm/s at raw 120 Hz. The sim's actuator ripple is correspondingly lower.</li>
    <li><b>Friction shape.</b> One tanh level per joint, no separate breakaway; the hardware arm's stick–slip bursts (0924 F3/F4) are only approximated.</li>
    <li><b>Yaw asymmetry</b> is a constant body torque, not a per-rotor drag difference; <b>battery sag</b> is linear in time, not a function of current draw.</li>
  </ul>
</section>
<section>
  <h2>{len(missions)+8}. Reproduce</h2>
  <ul>
    <li><code>docs/docs_aerial_manipulator/sim2real_tuning_20260926/tools/extract_all.sh</code> — the six bags → npz.</li>
    <li><code>/usr/bin/python3 tools/fit_plant.py data</code> — identification (analysis/fit_plant.{{json,txt}}).</li>
    <li><code>tools/replay.sh f18|a2|a3 &lt;tag&gt;</code> — one Isaac replay on the mirror plant (records a bag, restores the yaml).</li>
    <li><code>tools/compare.py data/&lt;flight&gt;.npz data/sim_&lt;tag&gt;.npz &lt;tag&gt;</code> then <code>tools/build_report.py</code>.</li>
    <li>Profiles: <code>WB_SIM_PROFILE=mirror</code> (default) or <code>robustness</code> on both the stack script and the Pegasus launcher.</li>
    <li><code>/usr/bin/python3 tools/arm_comp_audit.py</code> — the arm-channel compensation audit (analysis/arm_comp_audit.{{json,txt}}).</li>
    <li><code>/usr/bin/python3 tools/circle_sweep.py --baseline|--sweep|--combo|--gate</code> — the offline circle bench (exact law, mirror or robustness plant, <code>profile=</code>); gust/entry/delay metrics in <code>simulate()</code>.</li>
    <li><code>circle_tune_20260927/tools/</code>: <code>gen_circle_stream.py</code> / <code>gen_grid.py</code> (planner reference streams, offline), <code>circle_bench.py</code> (the bench; <code>rtf=0.48</code> = the Isaac clock), <code>tune_cma.py</code> (the search), <code>final_gate.py</code>, <code>eval_grid.py</code>, <code>gains_to_yaml.py</code> (bench names → yaml keys, <code>--apply</code>), <code>run_isaac_set.sh</code> (Isaac flights from a spec file).</li>
    <li><code>tools/run_gain_isaac.sh "wb_k_w=0.9,wb_l1_omega_c_t=6.0"</code> — the four Isaac confirmation flights (replay.sh with <code>WB_REPLAY_PROFILE</code> / <code>WB_REPLAY_GAINS</code>), scored by <code>tools/score_run.py</code>.</li>
  </ul>
</section>
</div>
<script>const FIGS={json.dumps(all_figs)};const PARETO={json.dumps(pareto_fig())};{JS}{PARETO_JS}</script>
"""
    open(out, "w").write(html)
    print("wrote", out, f"{os.path.getsize(out)/1e6:.2f} MB")


if __name__ == "__main__":
    main()
