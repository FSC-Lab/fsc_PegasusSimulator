"""Short two-point summary page -> ../summary_page.html (data from ../analysis/artifact_data.json)."""
import os, json
HERE = os.path.dirname(os.path.abspath(__file__))
data = json.load(open(os.path.join(HERE, "..", "analysis", "artifact_data.json")))
data["takeovers"] = json.load(open(os.path.join(HERE, "..", "analysis", "takeover_table.json")))
data["sim"] = json.load(open(os.path.join(HERE, "..", "analysis", "takeover_sim.json")))
html = r'''<title>10-14 Pick & Drop Findings</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
/* layout: one column, two findings, each = claim, chart, three figures */
:root{--bg:#000000;--panel:#121211;--fg:#ecebe6;--muted:#a3a29b;--rule:#2a2a27;--ccw:#3987e5;--cw:#d95926;
--sans:"IBM Plex Sans",system-ui,-apple-system,"Segoe UI",sans-serif;--mono:"IBM Plex Mono",ui-monospace,Menlo,monospace;color-scheme:dark}
body{background:var(--bg);color:var(--fg);font:15px/1.55 var(--sans)}
main{max-width:980px;margin:0 auto;padding-block:28px 48px;padding-left:max(16px,env(safe-area-inset-left,0px));padding-right:max(16px,env(safe-area-inset-right,0px));display:grid;gap:34px}
header{display:grid;gap:6px}
.eyebrow{font:500 12px/1.2 var(--mono);letter-spacing:.08em;text-transform:uppercase;color:var(--muted)}
h1{font-size:26px;line-height:1.2;margin:0;text-wrap:balance}
h2{font-size:19px;line-height:1.3;margin:0;text-wrap:balance}
p{margin:0;max-width:70ch}
section{display:grid;gap:12px;border-top:1px solid var(--rule);padding-top:22px}
.chart{background:var(--panel);border-radius:6px;min-width:0;height:340px}
.figs{display:flex;flex-wrap:wrap;gap:10px 28px;font-size:14px}
.figs div{display:grid;gap:2px;min-width:0}
.figs b{font:500 20px/1.2 var(--mono);font-variant-numeric:tabular-nums}
.figs span{color:var(--muted);font-size:13px}
.note{color:var(--muted);font-size:13.5px}
.tscroll{overflow-x:auto;min-width:0}
table{border-collapse:collapse;width:100%;font-size:13.5px}
th,td{padding:6px 10px;border-bottom:1px solid var(--rule);text-align:left;white-space:nowrap}
th{color:var(--muted);font-weight:500;font-size:12.5px}
td.n{font-family:var(--mono);font-variant-numeric:tabular-nums}
tr.ok td{color:var(--muted)}
.tag{font:500 11.5px/1 var(--mono);padding:3px 6px;border-radius:4px;background:#2a1a12;color:var(--cw)}
.tag.ok{background:#11202f;color:var(--ccw)}
ul{margin:0;padding-left:20px;display:grid;gap:6px;max-width:75ch}
h3{font-size:16px;margin:6px 0 0}
code{font-family:var(--mono);font-size:.92em;overflow-wrap:anywhere}
</style>
<main>
<header>
<div class="eyebrow">T650-AM whole-body 4-D L1 · free flights 2026-10-14</div>
<h1>Three findings from the pick-and-drop flights</h1>
</header>

<section>
<h2>1. The bin's airflow shook the drone</h2>
<p>Over the full bin the drone swung in pitch at 0.6 Hz. At the same drop point, height (1.32&nbsp;m), heading and arm pose on 10-09, over the plain hat, it stayed steady. Away from the bin, at the same height, it calmed down again. The swing comes from unsteady airflow off the bin, not a steady ground effect: average lift did not change.</p>
<div id="c1" class="chart" role="img" aria-label="Pitch at the drop point: large swings over the full bin, steady over the hat"></div>
<div class="figs">
<div><b>2.5° vs 0.2°</b><span>pitch swing (rms), bin vs hat</span></div>
<div><b>0.071 vs 0.011 N·m</b><span>unsteady pitch moment felt, bin vs hat</span></div>
<div><b>0.26 vs 0.05–0.13 N</b><span>unsteady vertical force, bin vs elsewhere</span></div>
</div>
<p class="note">The cut-down tray was not flown over (flight 2 stopped before the drop point). Hover over it at the drop height for 10–15&nbsp;s before the next full mission.</p>
</section>

<section>
<h2>2. The rotor commands drifted apart</h2>
<p>To hold heading, the clockwise rotors (front-left, rear-right) have to work harder than the counter-clockwise pair (front-right, rear-left). The gap was small in September and has grown since 10-10, in steps that follow hard landings. A loose front-left arm that tilts its motor gives exactly this pattern.</p>
<div id="c2" class="chart" role="img" aria-label="Median hover command of the four rotors per flight since 09-18"></div>
<div class="figs">
<div><b>0.56 / 0.65</b><span>CCW / CW pair, 09-18</span></div>
<div><b>0.40 / 0.77</b><span>CCW / CW pair, 10-14 flight 2</span></div>
<div><b>~50 %</b><span>of the hover yaw budget spent holding heading</span></div>
</div>
<p class="note">Median command in steady hover (DIRECT, |v|&nbsp;&lt;&nbsp;0.15&nbsp;m/s). Check after tightening: in the first hover the two pairs should be about 0.1 apart again.</p>
</section>

<section>
<h2>3. Taking over in STAB cut the motors (fixed)</h2>
<p>When the pilot switched from whole-body DIRECT to STAB, our flight node stayed in DIRECT and kept sending PX4 a motor command of NaN 250 times a second, meant as "props off". PX4 feeds that topic into the same motor outputs it drives in STAB, takes whichever message came last, and turns NaN into a motor stop. The pilot's throttle was mixed with stop commands until the node was shut down. Every disarm was the pilot's own RC switch; no software disarmed the vehicle.</p>
<div class="tscroll"><table id="tk"><thead><tr><th>Flight</th><th>Rig</th><th>Taken over from</th><th>Stop commands after the switch</th><th>Sink at touchdown</th><th>Tilt after landing</th></tr></thead><tbody></tbody></table></div>
<div class="figs">
<div><b>6 of 6</b><span>takeovers from DIRECT streamed stop commands</span></div>
<div><b>3 of 6 vs 1 of 10</b><span>tipped over: taken over from DIRECT vs from SAFETY</span></div>
</div>
<h3>Fix, checked in the simulator</h3>
<ul>
<li>The node sends the NaN stop command only while PX4 is in OFFBOARD.</li>
<li>When PX4 leaves OFFBOARD (the pilot or a PX4 failsafe), the node switches itself to SAFETY on the same tick and sends nothing to the motors.</li>
<li>Handing back to OFFBOARD holds where the vehicle is at that moment, not where it was taken over.</li>
<li>Applied to the whole-body and the decoupled geometric+L1 controllers (fsc_autopilot_ros2).</li>
</ul>
<p>The same scripted pilot flew both builds in Isaac: hover in DIRECT, switch to STAB and hold height with the stick, land, disarm, re-arm, climb, hand back to OFFBOARD.</p>
<div id="c3" class="chart" role="img" aria-label="Height through the takeover test: the old build falls to the ground after the switch; the fixed build holds, lands, re-arms and climbs"></div>
<div class="figs" id="simfigs"></div>
<p class="note">The decoupled geometric+L1 rig with the fix passed the same test: no stop commands after the switch, SAFETY, height held within 6 cm in STAB, re-armed and climbed to 0.85 m, hand-back drift 5 mm.</p>
<p class="note">Two pilot notes from the same tests. In STAB this vehicle needs the throttle about 0.2 above centre to hover (PX4's hover throttle is set low for the arm's weight), so hold the stick up when switching. After a tip-over PX4 refuses to arm ("attitude failure") until the vehicle is upright. Handing back to OFFBOARD dips about 0.3 m in the simulator before it recovers (the same SAFETY re-entry dip every DIRECT-to-SAFETY revert shows, 19&ndash;23 cm on hardware), so hand back with height to spare or land in STAB.</p>
</section>
</main>
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<script>
const D = __DATA__;
const css = getComputedStyle(document.documentElement);
const C = n => css.getPropertyValue(n).trim();
const font = {family: C('--sans'), color: C('--muted'), size: 12};
const base = {paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', font,
  margin: {l: 52, r: 16, t: 16, b: 48}, hovermode: 'x unified',
  hoverlabel: {bgcolor: '#1c1c1a', bordercolor: C('--rule'), font: {family: C('--mono'), color: C('--fg')}},
  legend: {orientation: 'h', x: 0, y: 1.12, font: {color: C('--fg')}}};
const ax = t => ({title: {text: t, font}, gridcolor: C('--rule'), zerolinecolor: C('--rule'), linecolor: C('--rule'), tickfont: font});
const cfg = {displayModeBar: false, responsive: true};

Plotly.newPlot('c1', [
  {x: D.pitch_t, y: D.pitch_bin, name: 'full bin (10-14)', mode: 'lines', line: {color: C('--cw'), width: 2}, hovertemplate: '%{y:.1f}°'},
  {x: D.pitch_t, y: D.pitch_hat, name: 'plain hat, no bin (10-09)', mode: 'lines', line: {color: C('--ccw'), width: 2}, hovertemplate: '%{y:.1f}°'}
], Object.assign({}, base, {xaxis: ax('seconds hovering at the drop point'), yaxis: ax('pitch (deg)'),
  shapes: [{type: 'line', x0: 6.3, x1: 6.3, yref: 'paper', y0: 0, y1: 1, line: {color: C('--muted'), width: 1, dash: 'dot'}}],
  annotations: [{x: 6.3, yref: 'paper', y: 1.0, yanchor: 'top', text: ' abort: climb to 1.65 m', showarrow: false, xanchor: 'left', font: {color: C('--muted'), size: 11}}]}), cfg);

const F = D.flights;
const lab = F.map(f => f.stamp.slice(4, 6) + '-' + f.stamp.slice(6, 8) + ' ' + f.stamp.slice(9, 11) + ':' + f.stamp.slice(11, 13));
const x = F.map((_, i) => i);
const rot = [['front-right (CCW)', 0, '--ccw', 'solid'], ['rear-left (CCW)', 1, '--ccw', 'dash'],
             ['front-left (CW)', 2, '--cw', 'solid'], ['rear-right (CW)', 3, '--cw', 'dash']];
const tr = rot.map(([n, k, c, dash]) => ({x: lab, y: F.map(f => f.u[k]), name: n, mode: 'lines+markers',
  line: {color: C(c), width: 2, dash}, marker: {size: 7, color: C(c)}, customdata: lab,
  hovertemplate: '%{y:.3f}'}));
const idx = s => F.findIndex(f => f.stamp.startsWith(s));
const ev = [[idx('20260924_120546') - 0.5, '09-24 crash'], [idx('20261010') - 0.5, '10-09 tip-over'],
            [idx('20261014_172845') - 0.5, '10-10 hard landing']];
Plotly.newPlot('c2', tr, Object.assign({}, base, {
  xaxis: Object.assign(ax('flight (date, start time)'), {type: 'category', tickangle: -50, tickfont: {family: C('--mono'), size: 10, color: C('--muted')}}),
  yaxis: Object.assign(ax('rotor command (0–1)'), {range: [0.3, 0.85]}),
  margin: {l: 52, r: 16, t: 16, b: 92},
  shapes: ev.map(([e]) => ({type: 'line', x0: e, x1: e, yref: 'paper', y0: 0, y1: 1, line: {color: C('--muted'), width: 1, dash: 'dot'}})),
  annotations: ev.map(([e, t], i) => ({x: e, yref: 'paper', y: [0.04, 0.04, 0.16][i], text: t, showarrow: false, xanchor: 'right', font: {color: C('--muted'), size: 10}}))
    .concat([{x: lab[F.length - 1], y: F[F.length - 1].u[2], text: 'arm found loose', showarrow: true, ax: -40, ay: -26, arrowcolor: C('--muted'), font: {color: C('--fg'), size: 11}}])
}), cfg);

const T = D.takeovers;
const tb = document.querySelector('#tk tbody');
T.forEach(r => {
  const hit = r.from_mode === 'DIRECT';
  const tr = document.createElement('tr'); if (!hit) tr.className = 'ok';
  tr.innerHTML = `<td class="n">${r.flight}</td><td>${r.rig}</td><td><span class="tag ${hit ? '' : 'ok'}">${r.from_mode}</span></td>` +
    `<td class="n">${r.nan_msgs ? r.nan_msgs.toLocaleString() : '0'}</td><td class="n">${r.sink.toFixed(2)} m/s</td>` +
    `<td class="n">${r.tilt_end.toFixed(0)}°${r.tilt_end > 20 ? ' (tipped)' : ''}</td>`;
  tb.appendChild(tr);
});
const S = D.sim;
const ph = [['stab_hold', 'STAB'], ['stab_land', 'land'], ['rearm', 're-arm'], ['stab_climb', 'climb'], ['handback', 'hand back']];
Plotly.newPlot('c3', [
  {x: S.old.t, y: S.old.z, name: 'old build', mode: 'lines', line: {color: C('--cw'), width: 2}, hovertemplate: '%{y:.2f} m'},
  {x: S.fixed.t, y: S.fixed.z, name: 'fixed build', mode: 'lines', line: {color: C('--ccw'), width: 2}, hovertemplate: '%{y:.2f} m'}
], Object.assign({}, base, {xaxis: ax('seconds from the pilot switching to STAB'), yaxis: Object.assign(ax('height (m)'), {range: [0, 1.15]}),
  shapes: ph.map(([k]) => ({type: 'line', x0: S.fixed.phase_start[k], x1: S.fixed.phase_start[k], yref: 'paper', y0: 0, y1: 1, line: {color: C('--muted'), width: 1, dash: 'dot'}})),
  annotations: ph.map(([k, l]) => ({x: S.fixed.phase_start[k], yref: 'paper', y: 1.0, yanchor: 'top', xanchor: 'left', text: ' ' + l, showarrow: false, font: {color: C('--muted'), size: 10}}))}), cfg);
const sf = document.getElementById('simfigs');
[[S.old.motor_nan_after.toLocaleString() + ' vs ' + S.fixed.motor_nan_after, 'stop commands reaching PX4 after the switch, old vs fixed'],
 [S.old.hold_min.toFixed(2) + ' vs ' + S.fixed.hold_min.toFixed(2) + ' m', 'lowest height in the 8 s STAB hold (from 0.98 m)'],
 [(S.old.rearmed ? 'yes' : 'no') + ' vs ' + (S.fixed.rearmed ? 'yes' : 'no'), 're-armed in STAB and climbed again'],
 [S.fixed.handback_dxy_mm + ' mm', 'drift in 8 s after handing back to OFFBOARD (fixed)']].forEach(([b, t]) => {
  const d = document.createElement('div'); d.innerHTML = `<b>${b}</b><span>${t}</span>`; sf.appendChild(d); });
</script>
'''
html = html.replace("__DATA__", json.dumps(data, separators=(",", ":")))
out = os.path.join(HERE, "..", "summary_page.html")
open(out, "w").write(html)
print("wrote", os.path.abspath(out), len(html), "bytes")
