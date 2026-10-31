// ---------- 4. the roll-joint handle (valve frame, mm; parts and numbers from valve_handle_joint_20261030/tools/roll_grip_design.py) ----------
const HG = [160, 0.6, 179];                                   // the grip point = push_pull_ee_offset
const HU = [0, Math.cos(Math.PI / 6), Math.sin(Math.PI / 6)]; // the joint axis: in the fin's plane, 30 deg up toward +y
const HC = [0, 0.5, -Math.sqrt(3) / 2];                       // the claw axis at the flown pose: +y, 60 deg down
const HX = [1, 0, 0];
const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]], vsc = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const vcross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const vnorm = a => { const n = Math.hypot(a[0], a[1], a[2]); return [a[0] / n, a[1] / n, a[2] / n]; };
const onPin = s => vadd(HG, vsc(HU, s));
const inPlane = (s, c) => vadd(onPin(s), vsc(HC, c));         // a point s along the axis, c along the claw axis
const RS = RG.s;                                              // the joint's stack along the axis (s, mm from G)
let hMode = "joint", hAng = 0;
try { const m = localStorage.getItem("valveHandleMode"); if (m === "fixed") hMode = "fixed"; } catch (e) {}

function rotPin(p, a) {     // Rodrigues: about the joint axis through G
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
function subMesh(m, keep) {
  const i = [], j = [], k = [];
  for (let t = 0; t < m.i.length; t++) if (keep(t)) { i.push(m.i[t]); j.push(m.j[t]); k.push(m.k[t]); }
  return { x: m.x, y: m.y, z: m.z, i, j, k };
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
const finSet = new Set(H.rotor.fin);

const HVIEWS = {
  iso: { eye: { x: 0.72, y: -0.92, z: 0.42 }, up: { x: 0, y: 0, z: 1 }, center: { x: 0.03, y: 0, z: 0.08 } },
  face: { eye: { x: 1.35, y: 0, z: 0.1 }, up: { x: 0, y: 0, z: 1 }, center: { x: 0, y: 0, z: 0.08 } },
  claw: { eye: { x: 0.2, y: -0.62, z: 1.05 }, up: { x: 0, y: 0.87, z: 0.5 }, center: { x: 0.05, y: 0, z: 0.12 } },
  top: { eye: { x: 0, y: 0, z: 1.6 }, up: { x: 1, y: 0, z: 0 }, center: { x: 0, y: 0, z: 0 } },
};
let hCamera = HVIEWS.iso;

function drawHandle3d() {
  if (typeof Plotly === "undefined") { document.getElementById("h3d").textContent = "The 3D view needs the plotting library, which did not load. Figs. 8 and 9 show the same parts."; return; }
  const joint = hMode === "joint", a = joint ? hAng * Math.PI / 180 : 0, R = p => rotPin(p, a);
  const tr = [];
  tr.push(meshTrace(joint ? subMesh(H.rotor, t => !finSet.has(t)) : H.rotor, css("--rotor"), "rotor"));
  for (const m of H.mounts) tr.push(meshTrace(m, "#2b2d31", "marker mount"));
  for (const m of H.markers) {
    tr.push(meshTrace(ocyl(m.a, m.b, 3, 14), "#8a8d93", "stem"));
    tr.push(meshTrace(sphere(m.c[0], m.c[1], m.c[2], 15, 12), "#d9dce0", "marker " + m.name, { hoverinfo: "text", hovertext: "marker " + m.name }));
    tr.push({ type: "scatter3d", mode: "text", x: [m.c[0]], y: [m.c[1]], z: [m.c[2] + 23], text: [m.name], hoverinfo: "skip",
      textfont: { color: css("--ink-2"), size: 12, family: "IBM Plex Mono, monospace" }, showlegend: false });
  }
  if (joint) {
    const post = css("--post"), steel = css("--steel");
    tr.push(meshTrace(box(157.5, 162.5, -19.4, 20.6, 74, RG.z_cut), css("--rotor"), "stump", { hoverinfo: "text", hovertext: "today's fin, sawn off 30 mm above its root" }));
    tr.push(meshTrace(box(157.5, 162.5, -19.4, 20.6, RG.z_cut, 204), css("--rotor"), "removed", { opacity: 0.1, hoverinfo: "text", hovertext: "today's fin above the cut (removed)" }));
    tr.push(meshTrace(RG.handle, post, "handle body", { hoverinfo: "text", hovertext: "handle body (fixed): handle_body.stl" }));
    tr.push(meshTrace(RG.m3, steel, "M3", { hoverinfo: "text", hovertext: "2 × M3 × 16 through the stump" }));
    tr.push(meshTrace(RG.screw, steel, "M4 screw", { hoverinfo: "text", hovertext: "M4 × 70: the joint's axle" }));
    tr.push(meshTrace(RG.washers, steel, "washers", { hoverinfo: "text", hovertext: "M4 washers" }));
    tr.push(meshTrace(xform(RG.sleeve, R), css("--paddle"), "sleeve", { hoverinfo: "text", hovertext: "grip sleeve Ø16 × 50 (turns): grip_sleeve.stl" }));
    const sx = [], sy = [], sz = [];     // a stripe along the sleeve's top, to show its turn
    for (let s = RS.sleeve0 + 1; s <= RS.sleeve1 - 1; s += 2) { const p = R(vadd(onPin(s), vsc(HC, -8.3))); sx.push(p[0]); sy.push(p[1]); sz.push(p[2]); }
    tr.push({ type: "scatter3d", mode: "lines", x: sx, y: sy, z: sz, line: { color: "#eef6f3", width: 6 }, hoverinfo: "skip", showlegend: false });
    const p0 = onPin(-78), p1 = onPin(62);
    tr.push({ type: "scatter3d", mode: "lines", x: [p0[0], p1[0]], y: [p0[1], p1[1]], z: [p0[2], p1[2]],
      line: { color: css("--thread"), width: 5, dash: "dash" }, hoverinfo: "text", hovertext: "joint axis", showlegend: false });
  }
  // the clamped claw: two fingers and the gripper body (simplified), turned with the sleeve
  const claw = css("--claw"), half = joint ? 10 : 4.5;
  for (const sg of [-1, 1]) tr.push(meshTrace(xform(obox(vadd(inPlane(0, -10), [sg * half, 0, 0]), HX, HU, HC, 2, 10, 30), R), claw, "finger", joint ? { opacity: 0.6 } : {}));
  tr.push(meshTrace(xform(obox(inPlane(0, -62), HX, HU, HC, 17, 15, 22), R), claw, "gripper", { opacity: 0.45, hoverinfo: "text", hovertext: "gripper (simplified)" }));
  const c0 = R(inPlane(0, -100)), c1 = R(inPlane(0, 32));
  tr.push({ type: "scatter3d", mode: "lines", x: [c0[0], c1[0]], y: [c0[1], c1[1]], z: [c0[2], c1[2]],
    line: { color: css("--ink-3"), width: 3, dash: "dot" }, hoverinfo: "text", hovertext: "claw axis", showlegend: false });
  tr.push({ type: "scatter3d", mode: "markers+text", x: [HG[0] + 12], y: [HG[1]], z: [HG[2]], text: ["G"], textposition: "middle right",
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
  document.getElementById("hHinged").setAttribute("aria-pressed", mode === "joint");
  const sl = document.getElementById("hAng"); sl.disabled = mode === "fixed";
  if (mode === "fixed") { sl.value = 0; hAng = 0; document.getElementById("hAngOut").textContent = "0°"; }
  document.getElementById("hNote").innerHTML = mode === "fixed"
    ? "Today: the fin is part of the rotor, so the clamped claw cannot roll. When the drone sways sideways by 10 mm, the arm forces <b>1.3°</b> of roll into the pads, the fin and the shaft."
    : "Roll joint: the sleeve turns on the screw with the claw, and the handle body stays. Sideways sway and roll are taken by the joint; <b>0.01°</b> is left per 10 mm of sway.";
  drawHandle3d();
}
document.getElementById("hFixed").addEventListener("click", () => setHandle("fixed"));
document.getElementById("hHinged").addEventListener("click", () => setHandle("joint"));
document.getElementById("hAng").addEventListener("input", e => {
  hAng = +e.target.value; document.getElementById("hAngOut").textContent = (hAng > 0 ? "+" : "") + hAng + "°"; drawHandle3d();
});

// ---------- Fig. 8: face-on, dimensioned ----------
function drawHandleFace() {
  const svg = document.getElementById("hFace"); svg.innerHTML = "";
  const Y0 = -116, Y1 = 128, Z0 = 22, Z1 = 228, S = 3.3;
  svg.setAttribute("viewBox", `0 0 ${(Y1 - Y0) * S} ${(Z1 - Z0) * S}`);
  const px = y => (y - Y0) * S, py = z => (Z1 - z) * S;
  const P = (s, c) => { const p = inPlane(s, c); return [px(p[1]), py(p[2])]; };
  const poly = pts => pts.map(p => p.join(",")).join(" ");
  const ink = css("--ink"), ink2 = css("--ink-2"), ink3 = css("--ink-3"), thread = css("--thread"), warn = css("--warn");
  const steel = css("--steel"), post = css("--post"), sleeve = css("--paddle"), rotor = css("--rotor"), panel = css("--panel"), bad = css("--bad");
  arrowDefs(svg, [ink, ink2, warn, thread]);
  const lab = (y, z, t, anchor, color, size) => el("text", { x: px(y), y: py(z), fill: color || ink2, "font-size": size || 12, "text-anchor": anchor || "start" }, svg, t);
  const R = (y0, y1, z0, z1, fill, extra) => el("rect", Object.assign({ x: px(y0), y: py(z1), width: (y1 - y0) * S, height: (z1 - z0) * S, fill }, extra || {}), svg);
  const L = (y0, z0, y1, z1, stroke, w, dash) => el("line", Object.assign({ x1: px(y0), y1: py(z0), x2: px(y1), y2: py(z1), stroke, "stroke-width": w || 1 }, dash ? { "stroke-dasharray": dash } : {}), svg);
  const Lp = (a, b, stroke, w, dash) => el("line", Object.assign({ x1: a[0], y1: a[1], x2: b[0], y2: b[1], stroke, "stroke-width": w || 1 }, dash ? { "stroke-dasharray": dash } : {}), svg);
  const sRect = (s0, s1, r, fill, extra) => el("polygon", Object.assign({ points: poly([P(s0, -r), P(s1, -r), P(s1, r), P(s0, r)]), fill }, extra || {}), svg);
  const leader = (y, z, to) => el("line", { x1: px(y), y1: py(z), x2: to[0], y2: to[1], stroke: ink3, "stroke-width": 1 }, svg);
  const hidden = "4 3";

  // the beam end, the root, marker E
  R(-25, 25.7, 40, 54, css("--rotor-3"));
  lab(30, 44, "beam, end-on", "start", ink3, 11);
  R(-24.4, 25.6, 54, 74, css("--rotor-2"));
  lab(30, 61, "root (z 54 to 74)", "start", ink3, 11);
  // today's fin above the cut, and the cut
  R(-19.4, 20.6, RG.z_cut, 204, "none", { stroke: rotor, "stroke-width": 1.2, "stroke-dasharray": "6 4" });
  lab(-21.5, 200, "today's fin, top 204", "end", ink3, 11);
  L(-28, RG.z_cut, 28, RG.z_cut, bad, 1.6, "8 4");
  lab(-30, RG.z_cut - 1.2, "saw here: z 104", "end", bad, 12);

  // the handle body: clamp, web (with its window), neck and boss
  const cl = RG.clamp;
  R(cl.y[0], cl.y[1], cl.z[0], cl.z[1], post);
  const webPath = RG.web.map(pl => "M" + pl.map(([y, z]) => `${px(y)},${py(z)}`).join("L") + "Z").join("");
  el("path", { d: webPath, fill: post, "fill-rule": "evenodd" }, svg);
  sRect(RS.boss1, RS.neck1, 4, post);
  sRect(RS.boss0, RS.boss1, 6.5, post);
  // hidden inside it: the stump, the slot ceiling, the bore, the nyloc pocket
  R(-19.4, 20.6, 74, RG.z_cut, "none", { stroke: ink, "stroke-width": 1, "stroke-dasharray": hidden, opacity: 0.7 });
  L(cl.y[0], RG.slot_top, cl.y[1], RG.slot_top, ink, 1, "2 3");
  for (const r of [-2.2, 2.2]) Lp(P(RS.boss0, r), P(RS.neck1, r), ink, 1, hidden);
  el("polygon", { points: poly([P(RS.boss0, -3.62), P(RS.boss0 + 5.3, -3.62), P(RS.boss0 + 5.3, 3.62), P(RS.boss0, 3.62)]), fill: "none", stroke: ink, "stroke-width": 1, "stroke-dasharray": hidden }, svg);
  // the nut pockets on this face (+x), vertex up, with the bolt holes
  for (const [y, z] of RG.bolts) {
    const r = 5.8 / Math.sqrt(3), pts = [];
    for (let k = 0; k < 6; k++) { const t = Math.PI / 2 + k * Math.PI / 3; pts.push([px(y + r * Math.cos(t)), py(z + r * Math.sin(t))]); }
    el("polygon", { points: poly(pts), fill: steel, stroke: css("--bg"), "stroke-width": 1 }, svg);
    el("circle", { cx: px(y), cy: py(z), r: 1.6 * S, fill: "#5a5f66" }, svg);
  }
  // marker E, in front of all of it
  el("circle", { cx: px(10.7), cy: py(92.5), r: 15 * S, fill: "none", stroke: ink2, "stroke-width": 1.4, "stroke-dasharray": "4 3" }, svg);
  lab(28, 79, "marker E (in front)", "start", ink2, 11);
  // the joint: washers, sleeve, screw head and tip
  sRect(RS.neck1, RS.sleeve0, 4.5, steel);
  sRect(RS.sleeve1, RS.head, 4.5, steel);
  sRect(RS.sleeve0, RS.sleeve1, 8, sleeve, { stroke: css("--bg"), "stroke-width": 1 });
  sRect(RS.head, RS.head + 4, 3.5, steel);
  sRect(RS.tip, RS.boss0, 2, steel);
  // the joint axis, the 30 deg angle at G, G
  Lp(P(-76, 0), P(62, 0), thread, 1.6, "12 4 2 4");
  lab(52, 214, "joint axis", "start", thread, 12);
  const rA = 30;
  L(HG[1], HG[2], HG[1] + rA + 10, HG[2], ink3, 1, "3 3");
  el("path", { d: `M${px(HG[1] + rA)},${py(HG[2])} A${rA * S},${rA * S} 0 0 0 ${px(HG[1] + rA * Math.cos(Math.PI / 6))},${py(HG[2] + rA * Math.sin(Math.PI / 6))}`,
    fill: "none", stroke: thread, "stroke-width": 1.4 }, svg);
  lab(HG[1] + rA + 2, HG[2] + 5, "30°", "start", thread, 12);
  el("polygon", { points: poly([P(-10, -40), P(10, -40), P(10, 20), P(-10, 20)]), fill: "none", stroke: ink2, "stroke-width": 1, "stroke-dasharray": "4 3" }, svg);
  el("circle", { cx: px(HG[1]), cy: py(HG[2]), r: 4, fill: warn }, svg);
  lab(HG[1] - 3, HG[2] - 7, "G", "end", warn, 13);
  // the push
  el("line", { x1: px(34), y1: py(150), x2: px(58), y2: py(150), stroke: warn, "stroke-width": 2, "marker-end": "url(#ah" + warn.slice(1) + ")" }, svg);
  lab(34, 142, "push: +y, the turn", "start", warn, 12);

  // labels, left
  lab(-48, 214, "closed fingers (dashed)", "end", ink3, 11);
  leader(-47, 216, P(-6, -40));
  lab(-48, 175, "boss Ø13, M4 nyloc", "end", ink, 12);
  lab(-48, 168, "in its end (hidden)", "end", ink2, 11);
  leader(-47, 171, P(RS.boss0 + 3, -6.5));
  lab(-48, 150, "M4 × 70 tip", "end", ink2, 11);
  leader(-47, 152, P(RS.tip + 0.5, 1));
  lab(-48, 138, "neck Ø8", "end", ink2, 11);
  leader(-47, 140, P((RS.boss1 + RS.neck1) / 2, 4));
  lab(-48, 124, "web 5 mm, the fin's", "end", ink2, 11);
  lab(-48, 117, "thickness; a window", "end", ink2, 11);
  lab(-48, 110, "to save weight", "end", ink2, 11);
  leader(-47, 121, [px(-24), py(128)]);
  lab(-48, 92, "clamp: 2 walls 4 mm,", "end", ink, 12);
  lab(-48, 85, "a 5.4 mm slot over", "end", ink2, 11);
  lab(-48, 78, "the stump (dashed)", "end", ink2, 11);
  leader(-47, 88, [px(cl.y[0]), py(96)]);
  // labels, top
  lab(2, 222, "grip sleeve Ø16 × 50 (turns)", "start", ink, 12);
  leader(6, 219, P(4, -8));
  lab(30, 205, "M4 washer, head", "start", ink2, 11);
  leader(29, 205, P(RS.head + 2, -3.5));
  // the M3s
  lab(-48, 64, "2 × M3 × 16, nuts in pockets", "end", ink2, 11);
  lab(-48, 57, "on this face, y ±12", "end", ink2, 11);
  leader(-47, 62, [px(RG.bolts[0][0] - 3), py(RG.bolts[0][1])]);

  // dimensions, right: heights from the root top
  const dimV = (y, z0, z1, t) => {
    el("line", { x1: px(y), y1: py(z0), x2: px(y), y2: py(z1), stroke: ink2, "stroke-width": 1, "marker-start": "url(#ah" + ink2.slice(1) + ")", "marker-end": "url(#ah" + ink2.slice(1) + ")" }, svg);
    lab(y + 2.5, (z0 + z1) / 2 - 1.5, t, "start", ink2, 11);
  };
  for (const [z, y0] of [[74, 25.6], [RG.bolts[1][1], RG.bolts[1][0] + 3], [RG.z_cut, cl.y[1]], [cl.z[1], cl.y[1]], [HG[2], 66]]) L(y0, z, 92, z, ink3, 0.8, "2 3");
  dimV(80, 74, RG.bolts[1][1], "15");
  dimV(80, RG.bolts[1][1], RG.z_cut, "15");
  dimV(80, RG.z_cut, cl.z[1], "10");
  dimV(80, cl.z[1], HG[2], "65");
  lab(94, 72.5, "root top z 74", "start", ink2, 11);
  lab(94, 87.5, "bolts z 89", "start", ink2, 11);
  lab(94, 102.5, "cut z 104", "start", bad, 11);
  lab(94, 115.5, "clamp top z 114", "start", ink2, 11);
  lab(94, 177.5, "G: z 179, y 0.6", "start", warn, 11);
  // width of the clamp, below
  el("line", { x1: px(cl.y[0]), y1: py(32), x2: px(cl.y[1]), y2: py(32), stroke: ink2, "stroke-width": 1, "marker-start": "url(#ah" + ink2.slice(1) + ")", "marker-end": "url(#ah" + ink2.slice(1) + ")" }, svg);
  lab((cl.y[0] + cl.y[1]) / 2, 34, "clamp 42 wide (y −20.4 to 21.6)", "middle", ink2, 11);
  // +y
  el("line", { x1: px(-112), y1: py(30), x2: px(-90), y2: py(30), stroke: ink2, "stroke-width": 1.2, "marker-end": "url(#ah" + ink2.slice(1) + ")" }, svg);
  lab(-88, 28, "+y", "start", ink2, 11);
}

// ---------- Fig. 9: the joint along its axis, and the clamp through a bolt ----------
function drawHandleSections() {
  const svg = document.getElementById("hSect"); svg.innerHTML = "";
  svg.setAttribute("viewBox", "0 0 1080 400");
  const ink = css("--ink"), ink2 = css("--ink-2"), ink3 = css("--ink-3"), thread = css("--thread"), warn = css("--warn"), bad = css("--bad");
  const steel = css("--steel"), post = css("--post"), sleeve = css("--paddle"), rotor = css("--rotor"), panel = css("--panel");
  arrowDefs(svg, [ink2, thread]);
  const T = (x, y, t, anchor, color, size) => el("text", { x, y, fill: color || ink2, "font-size": size || 12, "text-anchor": anchor || "start" }, svg, t);
  const dimH = (x0, x1, y, t) => {
    el("line", { x1: x0, y1: y, x2: x1, y2: y, stroke: ink2, "stroke-width": 1, "marker-start": "url(#ah" + ink2.slice(1) + ")", "marker-end": "url(#ah" + ink2.slice(1) + ")" }, svg);
    T((x0 + x1) / 2, y - 5, t, "middle", ink2, 11);
  };

  // (a) along the axis: u = s, v = c (down = toward the web)
  const k = 6, s0 = -52, ox = 22, oy = 206, U = s => ox + (s - s0) * k, V = c => oy + c * k;
  const rect = (sa, sb, ca, cb, fill, extra) => el("rect", Object.assign({ x: U(sa), y: V(ca), width: (sb - sa) * k, height: (cb - ca) * k, fill }, extra || {}), svg);
  T(ox, 24, "a. The joint, cut along its axis", "start", ink, 13);
  const sR = (18 - 5) / 0.866, sStep = RS.neck1 - 0.7;
  const webPts = [[RS.boss0, 0], [sStep, 0], [sStep, 10], [sR, 10], [(18 - 11) / 0.866, 22], [RS.boss0, 22]];
  el("polygon", { points: webPts.map(([s, c]) => `${U(s)},${V(c)}`).join(" "), fill: post }, svg);
  el("line", { x1: U(RS.boss0) - 6, y1: V(22), x2: U((18 - 11) / 0.866) + 6, y2: V(22), stroke: panel, "stroke-width": 3, "stroke-dasharray": "10 4" }, svg);
  rect(RS.boss0, RS.boss1, -6.5, 6.5, post);
  rect(RS.boss1, RS.neck1, -4, 4, post);
  rect(RS.boss0, RS.neck1, -2.2, 2.2, panel);                     // the bore
  rect(RS.boss0, RS.boss0 + 5.3, -3.62, 3.62, panel);             // the nyloc pocket
  rect(RS.boss0, RS.boss0 + 5.0, -3.5, 3.5, steel, { stroke: css("--bg"), "stroke-width": 1 });
  rect(RS.boss0, RS.boss0 + 1.5, -3.5, 3.5, "#d8d2b8");           // the nylon ring, outward
  rect(RS.sleeve0, RS.sleeve1, -8, 8, sleeve);
  rect(RS.sleeve0, RS.sleeve1, -2.2, 2.2, panel);
  rect(RS.neck1, RS.sleeve0, -4.5, 4.5, steel);
  rect(RS.sleeve1, RS.head, -4.5, 4.5, steel);
  rect(RS.tip, RS.head, -2, 2, steel);
  rect(RS.head, RS.head + 4, -3.5, 3.5, steel);
  el("line", { x1: U(-50), y1: V(0), x2: U(34), y2: V(0), stroke: thread, "stroke-width": 1.2, "stroke-dasharray": "10 3 2 3" }, svg);
  el("circle", { cx: U(0), cy: V(0), r: 3.5, fill: warn }, svg);
  T(U(0) + 6, V(0) - 6, "G", "start", warn, 13);
  // the pads' band
  el("rect", { x: U(-10), y: V(-11.5), width: 20 * k, height: 2.5 * k, fill: "none", stroke: ink2, "stroke-dasharray": "4 3" }, svg);
  T(U(0), V(-12.5), "pad, 20 mm wide: its centre may sit within ±15 mm", "middle", ink2, 11);
  // the stack's lengths
  const ticks = [RS.boss0, RS.boss1, RS.neck1, RS.sleeve0, RS.sleeve1, RS.head, RS.head + 4];
  for (const s of ticks) el("line", { x1: U(s), y1: V(-21), x2: U(s), y2: V(-8), stroke: ink3, "stroke-width": 0.8, "stroke-dasharray": "2 2" }, svg);
  const lens = ["11.4", "6", "", "50", "", "4"];
  for (let i = 0; i < 6; i++) if (lens[i]) dimH(U(ticks[i]), U(ticks[i + 1]), V(-19), lens[i]);
  T(U(RS.neck1) + 2, V(-23), "0.8 washers", "middle", ink3, 10);
  dimH(U(RS.tip), U(RS.head), V(27), "M4 × 70 socket head: 50 mm smooth shank, then 20 mm thread");
  // labels
  T(U(RS.boss0) + 2, V(9.5), "boss Ø13", "start", ink, 11);
  T(U(RS.boss0) - 4, V(-6), "nyloc", "end", ink2, 11);
  T(U(RS.boss0) - 4, V(-3.8), "nylon out", "end", ink2, 11);
  T((U(RS.boss1) + U(RS.neck1)) / 2, V(-8.6), "neck Ø8", "middle", ink2, 11);
  T(U(-22), V(4.8), "grip sleeve Ø16, bore Ø4.4 (turns)", "start", "#10241f", 11);
  T(U(sStep) + 6, V(15), "web, 2 mm under the sleeve", "start", ink, 11);
  T(U(RS.head + 4) + 6, V(1.4), "head Ø7", "start", ink2, 11);
  el("line", { x1: U(22), y1: V(-27), x2: U(32), y2: V(-27), stroke: thread, "stroke-width": 1.2, "marker-end": "url(#ah" + thread.slice(1) + ")" }, svg);
  T(U(21.5), V(-26.1), "+s: up, toward +y", "end", thread, 11);

  // (b) the clamp through a bolt: x right, z up
  const kb = 3.3, xb0 = 138, oxb = 700, zb0 = 30, oyb = 388;
  const X = x => oxb + (x - xb0) * kb, Z = z => oyb - (z - zb0) * kb;
  const box2 = (x0, x1, z0, z1, fill, extra) => el("rect", Object.assign({ x: X(x0), y: Z(z1), width: (x1 - x0) * kb, height: (z1 - z0) * kb, fill }, extra || {}), svg);
  T(oxb - 40, 24, "b. The clamp, cut through a bolt", "start", ink, 13);
  box2(138, 184, 40, 54, css("--rotor-3"));
  box2(155, 165, 54, 74, css("--rotor-2"));
  box2(157.5, 162.5, 74, RG.z_cut, rotor);
  const cl = RG.clamp;
  box2(cl.x[0], 157.3, 74, cl.z[1], post);
  box2(162.7, cl.x[1], 74, cl.z[1], post);
  box2(cl.x[0], cl.x[1], RG.slot_top, cl.z[1], post);
  box2(157.5, 162.5, cl.z[1], 126, post);
  el("line", { x1: X(155), y1: Z(126), x2: X(165), y2: Z(126), stroke: panel, "stroke-width": 3, "stroke-dasharray": "8 3" }, svg);
  const zb = RG.bolts[0][1];
  box2(cl.x[0] - 3, cl.x[0], zb - 2.75, zb + 2.75, steel);                   // head
  box2(cl.x[0], cl.x[0] + 16, zb - 1.5, zb + 1.5, steel);                    // shank
  box2(cl.x[1] - 2.5, cl.x[1], zb - 2.9, zb + 2.9, steel, { stroke: css("--bg"), "stroke-width": 1 });   // nut
  // z marks, left
  for (const z of [74, zb, RG.z_cut, RG.slot_top, cl.z[1]]) {
    el("line", { x1: X(140), y1: Z(z), x2: X(cl.x[0] - 3.5), y2: Z(z), stroke: ink3, "stroke-width": 0.8, "stroke-dasharray": "2 3" }, svg);
    T(X(139.5), Z(z) + 4, `z ${z}`, "end", z === RG.z_cut ? bad : ink2, 11);
  }
  // labels, right
  const RT = (z, t, color) => T(X(186), Z(z) + 4, t, "start", color || ink2, 11);
  const lead = (z, x) => el("line", { x1: X(185.5), y1: Z(z), x2: X(x), y2: Z(z), stroke: ink3, "stroke-width": 0.8 }, svg);
  RT(122, "web 5 mm"); lead(122, 162.6);
  RT(111, "bridge 6 mm over the slot"); lead(111, cl.x[1]);
  RT(99, "stump 5.0 mm in a 5.4 mm slot", ink); lead(99, 162.6);
  RT(zb, "M3 nut, 2.5 mm pocket"); lead(zb, cl.x[1]);
  RT(80, "walls 4 mm"); lead(80, cl.x[1]);
  RT(70, "walls sit on the root's ledges"); lead(70, 163.5);
  RT(46, "beam"); lead(46, 184);
  T(X(cl.x[0] - 3) - 4, Z(zb) - 10, "M3 × 16", "end", ink2, 11);
  dimH(X(cl.x[0]), X(cl.x[1]), Z(35), "13.4");
  T(X(160), Z(135), "−x (hub)   ·   +x (beam's end)", "middle", ink3, 10);
}

// ---------- the tables ----------
function handleTables() {
  const free = [["roll 1°", 0.27, 0.13, 0.01], ["sideways 10 mm", 1.34, 0.67, 0.01], ["pitch 1°", 0.13, 0.13, 0.13],
                ["fore-aft 10 mm", 0.98, 0.98, 0.98, "the valve turns to follow"], ["vertical 10 mm", 3.51, 3.51, 3.51, "height held to about ±5 mm"]];
  let h = "<thead><tr><th>Drone motion</th><th class='n'>Forced claw rotation, fixed fin</th><th class='n'>Left with a horizontal axle</th><th class='n'>Left with the joint 30° up</th><th></th></tr></thead><tbody>";
  for (const [m, a, b, c, note] of free)
    h += `<tr class="${c < 0.05 ? "close" : ""}"><td>${m}</td><td class="n">${a.toFixed(2)}°</td><td class="n">${b.toFixed(2)}°</td><td class="n">${c.toFixed(2)}°</td><td>${note || ""}</td></tr>`;
  document.getElementById("hFreeTable").innerHTML = h + "</tbody>";

  const off = [[0, "on the axis, as designed"], [10, "small: the claw closing 10 mm too deep or too shallow on the sleeve"], [20, "noticeable"], [30, "a tenth of today's grip"],
               [105, "a hinge at the fin's base: more than today's pads and fin (about 0.9 N·m/rad: 70 mN·m rms of couple for 4.5° rms of roll on 10-29)"]];
  h = "<thead><tr><th class='n'>Claw point off the axis by</th><th class='n'>Equivalent stiffness</th><th></th></tr></thead><tbody>";
  for (const [d, note] of off) h += `<tr class="${d <= 10 ? "close" : ""}"><td class="n">${d} mm</td><td class="n">${(120 * (d / 1000) ** 2 * 1000).toFixed(0)} mN·m/rad</td><td>${note}</td></tr>`;
  document.getElementById("hOffsetTable").innerHTML = h + "</tbody>";

  const spec = [
    ["Axis", "through G (x 160, y 0.6, z 179 mm), within 10 mm; in the fin's plane, 30° ± 5° above horizontal, rising toward +y", "square to the claw's axis: 60° below horizontal at the flown pose"],
    ["Grip", "Ø16 × 50 mm round sleeve, centred on G; the 20 mm pads may sit anywhere within ±15 mm along it", "the jaws open 37 mm at the tips: 10.5 mm a side, against the close gate's 8 mm across"],
    ["Before the grasp", "nothing: no lock, no centring", "the sleeve is round, so its angle does not change where the jaws close"],
    ["Friction", "a plain bearing: the sleeve's Ø4.4 bore on the screw's smooth shank, a washer at each end. About 0.4 mN·m in the turn dry, 0.2 greased; up to 5 at the 5 N force guard", "the fixed fin passed about 70 mN·m rms on 10-29; ball bearings would save under 0.5 mN·m"],
    ["Damping", "none (below 0.01 N·m·s/rad)", "a damper passes torque; the drone damps itself once decoupled"],
    ["Axial load", "the turning push pulls the sleeve toward the screw head: 4.3 N at the force guard, on the washer under the head", "the push runs mostly along the axis"],
    ["Strength", "at 5 N: 52 MPa in the M4 screw, 19 N per M3 bolt in shear, 2 MPa in the web when the claw pushes it sideways", "far below any limit"],
    ["Lock", "tighten the M4 screw: the sleeve is clamped between the washers, about 0.6–1.2 N·m of holding. Half a turn back frees it", "locked is a fixed grip of the same shape, for a locked/free comparison on one battery"],
    ["Rotor", "the fin's top (21 g) comes off and 49 g goes on (handle 27 g, sleeve 10 g, screws 12 g): +28 g at r 160 mm, about +28 % inertia. Masses at 84 % of solid PLA, as the printed rotor weighs", "re-run the spin test if you need the bearing friction again: the friction estimate scales with the inertia"],
    ["Ground station", "push_pull_full_close_is_grasp:=false on hardware, for valve sessions", "the jaws stop on the sleeve: a stall is the grasp, a full close a miss"],
    ["Simulation", "not modelled yet", "if the joint goes into the Isaac asset, its sleeve collider must be at least 20 mm across: the simulated claw cannot close below about 19 mm"],
  ];
  h = "<thead><tr><th>What</th><th>Value</th><th>Why</th></tr></thead><tbody>";
  for (const [a, b, c] of spec) h += `<tr><td>${a}</td><td>${b}</td><td>${c}</td></tr>`;
  document.getElementById("hSpecTable").innerHTML = h + "</tbody>";

  const clr = [["Closed pads to the Ø13 boss", "reached only with the claw at least 21.8 mm off along the axis toward it (close gate 20 mm)"],
               ["Jaws to the sleeve at the close, a side", "10.5 mm"],
               ["Closed fingers to the clamp, claw on G / 20 mm off", "42.7 / 22.7 mm"],
               ["Gripper body to the handle on the way in, claw on G / 20 mm off", "37.5 / 17.5 mm"],
               ["Open fingers past the clamp (13.4 mm wide), a side", "11.8 mm"],
               ["Sleeve to the web below it / to the web's step at its end", "2.0 / 1.5 mm"],
               ["M3 bolt ends (+x) to marker E's ball", "about 7 mm"],
               ["Highest point of the handle", "z 198 (today's fin top: 204)"]];
  h = "<thead><tr><th>Between</th><th>Gap</th></tr></thead><tbody>";
  for (const [a, b] of clr) h += `<tr><td>${a}</td><td>${b}</td></tr>`;
  document.getElementById("hClearTable").innerHTML = h + "</tbody>";
}
drawHandleFace(); drawHandleSections(); handleTables(); setHandle(hMode);
