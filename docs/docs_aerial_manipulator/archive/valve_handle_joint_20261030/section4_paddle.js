
// ---------- 4. the hinged handle (valve frame, mm; numbers from valve_handle_joint_20261030/tools/handle_design.py) ----------
const HG = [160, 0.6, 179];                                   // the grip point = push_pull_ee_offset
const HU = [0, Math.cos(Math.PI / 6), Math.sin(Math.PI / 6)]; // the pin: in the fin's plane, 30 deg up toward +y
const HC = [0, 0.5, -Math.sqrt(3) / 2];                       // the claw axis at the flown pose: +y, 60 deg down
const HX = [1, 0, 0];
const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]], vsc = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const vcross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const vnorm = a => { const n = Math.hypot(a[0], a[1], a[2]); return [a[0] / n, a[1] / n, a[2] / n]; };
const onPin = s => vadd(HG, vsc(HU, s));
const inPlane = (s, c) => vadd(onPin(s), vsc(HC, c));         // a point s along the pin, c along the claw axis
let hMode = "hinged", hAng = 0;
try { const m = localStorage.getItem("valveHandleMode"); if (m === "fixed") hMode = "fixed"; } catch (e) {}

function rotPin(p, a) {     // Rodrigues: about the pin axis through G
  const v = [p[0] - HG[0], p[1] - HG[1], p[2] - HG[2]], k = HU, c = Math.cos(a), s = Math.sin(a);
  const kxv = vcross(k, v), kv = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
  return [0, 1, 2].map(i => HG[i] + v[i] * c + kxv[i] * s + k[i] * kv * (1 - c));
}
function xform(m, f) {
  const x = [], y = [], z = [];
  for (let n = 0; n < m.x.length; n++) { const p = f([m.x[n], m.y[n], m.z[n]]); x.push(p[0]); y.push(p[1]); z.push(p[2]); }
  return { x, y, z, i: m.i, j: m.j, k: m.k };
}
function obox(c, e1, e2, e3, h1, h2, h3) {
  const x = [], y = [], z = [];
  for (const [a, b, d] of [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]]) {
    const p = [0, 1, 2].map(i => c[i] + a * h1 * e1[i] + b * h2 * e2[i] + d * h3 * e3[i]); x.push(p[0]); y.push(p[1]); z.push(p[2]);
  }
  const f = [[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6], [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2], [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0]];
  return { x, y, z, i: f.map(t => t[0]), j: f.map(t => t[1]), k: f.map(t => t[2]) };
}
function ocyl(a, b, r, n) {   // a closed cylinder from a to b
  const w = vnorm([b[0] - a[0], b[1] - a[1], b[2] - a[2]]);
  const p1 = vnorm(vcross(w, Math.abs(w[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0])), p2 = vcross(w, p1);
  const x = [], y = [], z = [], i = [], j = [], k = [];
  for (const e of [a, b]) for (let q = 0; q < n; q++) {
    const t = 2 * Math.PI * q / n, p = [0, 1, 2].map(m => e[m] + r * (Math.cos(t) * p1[m] + Math.sin(t) * p2[m]));
    x.push(p[0]); y.push(p[1]); z.push(p[2]);
  }
  x.push(a[0], b[0]); y.push(a[1], b[1]); z.push(a[2], b[2]); const c0 = 2 * n, c1 = 2 * n + 1;
  for (let q = 0; q < n; q++) { const a0 = q, a1 = (q + 1) % n, t0 = q + n, t1 = (q + 1) % n + n;
    i.push(a0, a1, c0, c1); j.push(a1, t1, a1, t0); k.push(t0, t0, a0, t1); }
  return { x, y, z, i, j, k };
}
function subMesh(m, keep) {
  const i = [], j = [], k = [];
  for (let t = 0; t < m.i.length; t++) if (keep(t)) { i.push(m.i[t]); j.push(m.j[t]); k.push(m.k[t]); }
  return { x: m.x, y: m.y, z: m.z, i, j, k };
}
const finSet = new Set(H.rotor.fin);

const HVIEWS = {
  iso: { eye: { x: 0.72, y: -0.92, z: 0.42 }, up: { x: 0, y: 0, z: 1 }, center: { x: 0.03, y: 0, z: 0.08 } },
  face: { eye: { x: 1.35, y: 0, z: 0.1 }, up: { x: 0, y: 0, z: 1 }, center: { x: 0, y: 0, z: 0.08 } },
  claw: { eye: { x: 0.2, y: -0.62, z: 1.05 }, up: { x: 0, y: 0.87, z: 0.5 }, center: { x: 0.05, y: 0, z: 0.12 } },
  top: { eye: { x: 0, y: 0, z: 1.6 }, up: { x: 1, y: 0, z: 0 }, center: { x: 0, y: 0, z: 0 } },
};
let hCamera = HVIEWS.iso;

function drawHandle3d() {
  if (typeof Plotly === "undefined") { document.getElementById("h3d").textContent = "The 3D view needs the plotting library, which did not load. Fig. 8 shows the same layout."; return; }
  const hinged = hMode === "hinged", a = hinged ? hAng * Math.PI / 180 : 0, R = p => rotPin(p, a);
  const tr = [];
  tr.push(meshTrace(hinged ? subMesh(H.rotor, t => !finSet.has(t)) : H.rotor, css("--rotor"), "rotor"));
  if (hinged) tr.push(meshTrace(subMesh(H.rotor, t => finSet.has(t)), css("--rotor"), "fin", { opacity: 0.12, hoverinfo: "text", hovertext: "today's fin (removed)" }));
  for (const m of H.mounts) tr.push(meshTrace(m, "#2b2d31", "marker mount"));
  for (const m of H.markers) {
    tr.push(meshTrace(ocyl(m.a, m.b, 3, 14), "#8a8d93", "stem"));
    tr.push(meshTrace(sphere(m.c[0], m.c[1], m.c[2], 15, 12), "#d9dce0", "marker " + m.name, { hoverinfo: "text", hovertext: "marker " + m.name }));
    tr.push({ type: "scatter3d", mode: "text", x: [m.c[0]], y: [m.c[1]], z: [m.c[2] + 23], text: [m.name], hoverinfo: "skip",
      textfont: { color: css("--ink-2"), size: 12, family: "IBM Plex Mono, monospace" }, showlegend: false });
  }
  if (hinged) {
    const post = css("--post"), steel = css("--steel");
    tr.push(meshTrace(box(155, 165, -47, -24.4, 54, 74), post, "root, widened", { hoverinfo: "text", hovertext: "root widened to y -47" }));
    tr.push(meshTrace(box(154, 166, -47, -35, 74, 156), post, "post", { hoverinfo: "text", hovertext: "post (fixed)" }));
    tr.push(meshTrace(ocyl(onPin(-46), onPin(-36), 8, 28), post, "housing", { hoverinfo: "text", hovertext: "housing: 2 x MR84ZZ" }));
    tr.push(meshTrace(ocyl(onPin(-48.5), onPin(-47.2), 4, 18), steel, "E-clip", { hoverinfo: "text", hovertext: "E-clip" }));
    tr.push(meshTrace(xform(box(157, 163, -19.4, 20.6, 124, 204), R), css("--paddle"), "paddle", { hoverinfo: "text", hovertext: "paddle, 40 x 80 x 6 mm" }));
    tr.push(meshTrace(xform(ocyl(onPin(-27), onPin(-16), 6, 24), R), css("--paddle"), "boss", { hoverinfo: "text", hovertext: "boss, the pin fixed in it" }));
    tr.push(meshTrace(xform(ocyl(onPin(-47.5), onPin(-16), 2, 16), R), steel, "pin", { hoverinfo: "text", hovertext: "4 mm steel pin" }));
    tr.push(meshTrace(xform(box(156.5, 163.5, -5.4, 6.6, 126, 134), R), steel, "weight", { hoverinfo: "text", hovertext: "10 g steel" }));
    const p0 = onPin(-80), p1 = onPin(65);
    tr.push({ type: "scatter3d", mode: "lines", x: [p0[0], p1[0]], y: [p0[1], p1[1]], z: [p0[2], p1[2]],
      line: { color: css("--thread"), width: 5, dash: "dash" }, hoverinfo: "text", hovertext: "pin axis", showlegend: false });
  }
  // the clamped claw: two fingers and the gripper body (simplified), turned with the paddle
  const claw = css("--claw");
  for (const xs of [155, 165]) tr.push(meshTrace(xform(obox(vadd(inPlane(0, -10), [xs - 160, 0, 0]), HX, HU, HC, 2, 10, 30), R), claw, "finger"));
  tr.push(meshTrace(xform(obox(inPlane(0, -62), HX, HU, HC, 17, 15, 22), R), claw, "gripper", { opacity: 0.45, hoverinfo: "text", hovertext: "gripper (simplified)" }));
  const c0 = R(inPlane(0, -100)), c1 = R(inPlane(0, 32));
  tr.push({ type: "scatter3d", mode: "lines", x: [c0[0], c1[0]], y: [c0[1], c1[1]], z: [c0[2], c1[2]],
    line: { color: css("--ink-3"), width: 3, dash: "dot" }, hoverinfo: "text", hovertext: "claw axis", showlegend: false });
  tr.push({ type: "scatter3d", mode: "markers+text", x: [HG[0] + 10], y: [HG[1]], z: [HG[2]], text: ["G"], textposition: "middle right",
    marker: { size: 4, color: css("--warn") }, textfont: { color: css("--warn"), size: 13, family: "IBM Plex Mono, monospace" }, hoverinfo: "text",
    hovertext: "grip point G: x 160, y 0.6, z 179 mm", showlegend: false });
  const axis = { visible: false };
  const layout = { paper_bgcolor: css("--panel"), margin: { l: 0, r: 0, t: 0, b: 0 }, showlegend: false, uirevision: "keep",
    scene: { aspectmode: "data", xaxis: axis, yaxis: axis, zaxis: axis, bgcolor: css("--panel"), camera: hCamera } };
  Plotly.react("h3d", tr, layout, { displaylogo: false, responsive: true, modeBarButtonsToRemove: ["toImage"] });
}
document.querySelectorAll("[data-hview]").forEach(b => b.addEventListener("click", () => {
  hCamera = HVIEWS[b.dataset.hview];
  if (typeof Plotly !== "undefined") Plotly.relayout("h3d", { "scene.camera": hCamera });
}));

function setHandle(mode) {
  hMode = mode;
  try { localStorage.setItem("valveHandleMode", mode); } catch (e) {}
  document.getElementById("hFixed").setAttribute("aria-pressed", mode === "fixed");
  document.getElementById("hHinged").setAttribute("aria-pressed", mode === "hinged");
  const sl = document.getElementById("hAng"); sl.disabled = mode === "fixed";
  if (mode === "fixed") { sl.value = 0; hAng = 0; document.getElementById("hAngOut").textContent = "0°"; }
  document.getElementById("hNote").innerHTML = mode === "fixed"
    ? "Today: the fin is part of the rotor, so the clamped claw cannot roll. When the drone sways sideways by 10 mm, the arm forces <b>1.3°</b> of roll into the pads, the fin and the shaft."
    : "Hinged: the paddle turns on the pin with the claw, the post stays. Sideways sway and roll are taken by the pin; <b>0.01°</b> is left per 10 mm of sway.";
  drawHandle3d();
}
document.getElementById("hFixed").addEventListener("click", () => setHandle("fixed"));
document.getElementById("hHinged").addEventListener("click", () => setHandle("hinged"));
document.getElementById("hAng").addEventListener("input", e => {
  hAng = +e.target.value; document.getElementById("hAngOut").textContent = (hAng > 0 ? "+" : "") + hAng + "°"; drawHandle3d();
});

// ---------- the face-on drawing ----------
function drawHandleFace() {
  const svg = document.getElementById("hFace"); svg.innerHTML = "";
  const Y0 = -128, Y1 = 146, Z0 = 32, Z1 = 234, S = 3.0;
  svg.setAttribute("viewBox", `0 0 ${(Y1 - Y0) * S} ${(Z1 - Z0) * S}`);
  const px = y => (y - Y0) * S, py = z => (Z1 - z) * S;
  const P = (s, c) => { const p = inPlane(s, c); return [px(p[1]), py(p[2])]; };
  const poly = pts => pts.map(p => p.join(",")).join(" ");
  const ink = css("--ink"), ink2 = css("--ink-2"), ink3 = css("--ink-3"), thread = css("--thread"), warn = css("--warn");
  const steel = css("--steel"), post = css("--post"), paddle = css("--paddle"), rotor = css("--rotor");
  arrowDefs(svg, [ink, ink2, warn, thread]);
  const lab = (y, z, t, anchor, color, size) => el("text", { x: px(y), y: py(z), fill: color || ink2, "font-size": size || 12, "text-anchor": anchor || "start" }, svg, t);
  const R = (y0, y1, z0, z1, fill, extra) => el("rect", Object.assign({ x: px(y0), y: py(z1), width: (y1 - y0) * S, height: (z1 - z0) * S, fill }, extra || {}), svg);
  // the beam end and the root, the table line
  el("line", { x1: px(-120), y1: py(40), x2: px(110), y2: py(40), stroke: ink3, "stroke-width": 1, "stroke-dasharray": "2 4" }, svg);
  R(-25, 25.7, 40, 54, css("--rotor-3"));
  lab(30, 44, "beam, end-on (z 40 to 54)", "start", ink3, 11);
  R(-24.4, 25.6, 54, 74, css("--rotor-2"));
  lab(30, 61, "root (z 54 to 74)", "start", ink3, 11);
  R(-47, -24.4, 54, 74, post, { opacity: 0.85 });
  lab(-49, 58, "root widened to y −47", "end", ink2, 11);
  // marker E (in front)
  el("circle", { cx: px(10.7), cy: py(92.5), r: 15 * S, fill: "none", stroke: ink3, "stroke-width": 1, "stroke-dasharray": "3 3" }, svg);
  lab(10.7, 89, "E", "middle", ink3, 11);
  // today's fin
  R(-19.4, 20.6, 74, 204, "none", { stroke: rotor, "stroke-width": 1.2, "stroke-dasharray": "6 4" });
  lab(23, 113, "today's fin, removed", "start", ink3, 11);
  // the post: column + housing along the pin
  R(-47, -35, 74, 156, post);
  el("polygon", { points: poly([P(-46, -8), P(-36, -8), P(-36, 8), P(-46, 8)]), fill: post }, svg);
  lab(-50, 120, "post (fixed)", "end", ink, 12);
  lab(-50, 107, "12 × 12 mm, y −47 to −35", "end", ink2, 11);
  lab(-52, 172, "housing: 2 × MR84ZZ", "end", ink2, 11);
  el("line", { x1: px(-51), y1: py(170), x2: P(-44, -7)[0], y2: P(-44, -7)[1], stroke: ink3, "stroke-width": 1 }, svg);
  // the paddle, weight, boss, pin
  R(-19.4, 20.6, 124, 204, paddle, { opacity: 0.9 });
  R(-5.4, 6.6, 126, 134, steel);
  lab(27, 128, "10 g steel", "start", ink2, 11);
  el("line", { x1: px(26), y1: py(130), x2: px(6.6), y2: py(130), stroke: ink3, "stroke-width": 1 }, svg);
  el("polygon", { points: poly([P(-27, -6), P(-16, -6), P(-16, 6), P(-27, 6)]), fill: paddle, stroke: css("--bg"), "stroke-width": 1 }, svg);
  const a0 = P(-48, 0), a1 = P(-16, 0);
  el("line", { x1: a0[0], y1: a0[1], x2: a1[0], y2: a1[1], stroke: steel, "stroke-width": 4 * S, "stroke-linecap": "butt" }, svg);
  const e0 = P(-48.5, -4), e1 = P(-48.5, 4);
  el("line", { x1: e0[0], y1: e0[1], x2: e1[0], y2: e1[1], stroke: steel, "stroke-width": 3 }, svg);
  lab(-52, 140, "E-clip", "end", ink2, 11);
  el("line", { x1: px(-51), y1: py(142), x2: P(-48.5, 3)[0], y2: P(-48.5, 3)[1], stroke: ink3, "stroke-width": 1 }, svg);
  // the pin axis
  const q0 = P(-80, 0), q1 = P(72, 0);
  el("line", { x1: q0[0], y1: q0[1], x2: q1[0], y2: q1[1], stroke: thread, "stroke-width": 1.6, "stroke-dasharray": "12 4 2 4" }, svg);
  lab(66, 221, "pin axis", "start", thread, 12);
  // the 30 deg angle, at G
  const rA = 32;
  el("line", { x1: px(HG[1]), y1: py(HG[2]), x2: px(HG[1] + rA + 12), y2: py(HG[2]), stroke: ink3, "stroke-width": 1, "stroke-dasharray": "3 3" }, svg);
  el("path", { d: `M${px(HG[1] + rA)},${py(HG[2])} A${rA * S},${rA * S} 0 0 0 ${px(HG[1] + rA * Math.cos(Math.PI / 6))},${py(HG[2] + rA * Math.sin(Math.PI / 6))}`,
    fill: "none", stroke: thread, "stroke-width": 1.4 }, svg);
  lab(HG[1] + rA + 3, HG[2] + 6, "30°", "start", thread, 12);
  // the closed fingers' footprint and the close gate
  el("polygon", { points: poly([P(-10, -40), P(10, -40), P(10, 20), P(-10, 20)]), fill: "none", stroke: ink2, "stroke-width": 1, "stroke-dasharray": "4 3" }, svg);
  el("circle", { cx: px(HG[1]), cy: py(HG[2]), r: 20 * S, fill: "none", stroke: ink3, "stroke-width": 1, "stroke-dasharray": "2 3" }, svg);
  // the claw axis and the approach
  const k0 = P(0, -58), k1 = P(0, 26);
  el("line", { x1: k0[0], y1: k0[1], x2: k1[0], y2: k1[1], stroke: ink2, "stroke-width": 1.4, "marker-end": "url(#ah" + ink2.slice(1) + ")" }, svg);
  lab(-56, 226, "claw axis, 60° down", "end", ink2, 12);
  el("line", { x1: px(-62), y1: py(224), x2: k0[0], y2: k0[1], stroke: ink3, "stroke-width": 1 }, svg);
  lab(-60, 214, "closed fingers (dashed)", "end", ink3, 11);
  // G
  el("circle", { cx: px(HG[1]), cy: py(HG[2]), r: 4, fill: warn }, svg);
  lab(HG[1] - 4, HG[2] + 4, "G", "end", warn, 13);
  // the push
  el("line", { x1: px(24), y1: py(166), x2: px(50), y2: py(166), stroke: warn, "stroke-width": 2, "marker-end": "url(#ah" + warn.slice(1) + ")" }, svg);
  lab(24, 157, "push: +y, the turn", "start", warn, 12);
  // dimensions on the right
  const dimV = (y, z0, z1, t) => {
    el("line", { x1: px(y), y1: py(z0), x2: px(y), y2: py(z1), stroke: ink2, "stroke-width": 1, "marker-start": "url(#ah" + ink2.slice(1) + ")", "marker-end": "url(#ah" + ink2.slice(1) + ")" }, svg);
    lab(y + 3, (z0 + z1) / 2 - 1.5, t, "start", ink2, 11);
  };
  el("line", { x1: px(20.6), y1: py(204), x2: px(80), y2: py(204), stroke: ink3, "stroke-width": 0.8, "stroke-dasharray": "2 3" }, svg);
  el("line", { x1: px(20.6), y1: py(179), x2: px(80), y2: py(179), stroke: ink3, "stroke-width": 0.8, "stroke-dasharray": "2 3" }, svg);
  el("line", { x1: px(20.6), y1: py(124), x2: px(80), y2: py(124), stroke: ink3, "stroke-width": 0.8, "stroke-dasharray": "2 3" }, svg);
  dimV(78, 179, 204, "25");
  dimV(78, 124, 179, "55");
  lab(84, 205, "top 204", "start", ink2, 11);
  lab(84, 180, "G: z 179, y 0.6", "start", warn, 11);
  lab(84, 125, "paddle bottom 124", "start", ink2, 11);
  lab(84, 113, "paddle 40 × 80 × 6 mm", "start", ink, 12);
  el("line", { x1: px(-19.4), y1: py(214), x2: px(20.6), y2: py(214), stroke: ink2, "stroke-width": 1, "marker-start": "url(#ah" + ink2.slice(1) + ")", "marker-end": "url(#ah" + ink2.slice(1) + ")" }, svg);
  lab(0.6, 217, "40", "middle", ink2, 11);
  // the housing's height and the gap to the gripper
  lab(-52, 159, "housing top z 168", "end", ink2, 11);
  // y axis note
  el("line", { x1: px(-120), y1: py(36), x2: px(-95), y2: py(36), stroke: ink2, "stroke-width": 1.2, "marker-end": "url(#ah" + ink2.slice(1) + ")" }, svg);
  lab(-92, 34, "+y", "start", ink2, 11);
}

// ---------- the tables ----------
function handleTables() {
  const free = [["roll 1°", 0.27, 0.13, 0.01], ["sideways 10 mm", 1.34, 0.67, 0.01], ["pitch 1°", 0.13, 0.13, 0.13],
                ["fore-aft 10 mm", 0.98, 0.98, 0.98, "the valve turns to follow"], ["vertical 10 mm", 3.51, 3.51, 3.51, "height held to about ±5 mm"]];
  let h = "<thead><tr><th>Drone motion</th><th class='n'>Forced claw rotation, fixed fin</th><th class='n'>Left with a horizontal pin</th><th class='n'>Left with the pin 30° up</th><th></th></tr></thead><tbody>";
  for (const [m, a, b, c, note] of free)
    h += `<tr class="${c < 0.05 ? "close" : ""}"><td>${m}</td><td class="n">${a.toFixed(2)}°</td><td class="n">${b.toFixed(2)}°</td><td class="n">${c.toFixed(2)}°</td><td>${note || ""}</td></tr>`;
  document.getElementById("hFreeTable").innerHTML = h + "</tbody>";

  const off = [[0, "exact"], [10, "small: about twice the paddle's own gravity centring"], [20, "noticeable"], [30, "a tenth of today's grip"],
               [105, "a hinge at the fin's base: more than today's pads and fin (about 0.9 N·m/rad: 70 mN·m rms of couple for 4.5° rms of roll on 10-29)"]];
  h = "<thead><tr><th class='n'>Pin misses G by</th><th class='n'>Equivalent stiffness</th><th></th></tr></thead><tbody>";
  for (const [d, note] of off) h += `<tr class="${d <= 10 ? "close" : ""}"><td class="n">${d} mm</td><td class="n">${(120 * (d / 1000) ** 2 * 1000).toFixed(0)} mN·m/rad</td><td>${note}</td></tr>`;
  document.getElementById("hOffsetTable").innerHTML = h + "</tbody>";

  const spec = [
    ["Axis", "through G (x 160, y 0.6, z 179 mm), within 10 mm; in the fin's plane, 30° ± 5° above horizontal, rising toward +y", "square to the claw's axis: 60° below horizontal at the flown pose, 55–60° over the turn"],
    ["Friction", "under 1 mN·m: two shielded MR84ZZ ball bearings", "the fixed fin passed about 70 mN·m rms on 10-29; friction anywhere near that keeps the pin stuck"],
    ["Centring", "gravity only, about 5 mN·m/rad: the paddle's centre of mass 13 mm below the pin, plus 10 g of steel 42 mm below", "hangs straight within ±2–3° for the grasp; a spring would add the stiffness the pin is there to remove"],
    ["Travel", "±20°, two stops on the boss's flange", "10-29: 4.5° rms of claw roll, peaks about 12°"],
    ["Damping", "none (below 0.01 N·m·s/rad)", "a damper passes torque; the drone damps itself once decoupled"],
    ["Axial load", "the turning push pulls the pin out of the post: 4.3 N at the 5 N force guard. E-clip behind the far bearing, shim at the near one", "the push runs mostly along the pin"],
    ["Strength", "12 MPa in the 4 mm steel pin at 5 N", "far below any limit"],
    ["Lock", "a Ø3 dowel through the post and the boss's flange", "locked = today's fin, for a locked/free comparison on one battery"],
    ["Rotor inertia", "about +20 % (some 21 g more at r 160 mm)", "re-run the spin test after fitting it; the friction estimate scales with the inertia"],
  ];
  h = "<thead><tr><th>What</th><th>Value</th><th>Why</th></tr></thead><tbody>";
  for (const [a, b, c] of spec) h += `<tr><td>${a}</td><td>${b}</td><td>${c}</td></tr>`;
  document.getElementById("hSpecTable").innerHTML = h + "</tbody>";

  const clr = [["Closed fingers to the post, claw on G", "26 mm"], ["… claw 10 mm off G", "16 mm"], ["… claw 20 mm off G (close-gate limit)", "6 mm"],
               ["Gripper body to the post on the way in (jaws open, the post passes between the fingers)", "38 mm, 18 at 20 mm off"],
               ["Housing top below G", "11 mm"], ["Paddle at ±20°: reach in x, lowest point", "137–183 mm, z 125"],
               ["Root top / marker E's ball top", "z 74 / z 108: clear"]];
  h = "<thead><tr><th>Between</th><th class='n'>Gap</th></tr></thead><tbody>";
  for (const [a, b] of clr) h += `<tr><td>${a}</td><td class="n">${b}</td></tr>`;
  document.getElementById("hClearTable").innerHTML = h + "</tbody>";
}
drawHandleFace(); handleTables(); setHandle(hMode);
