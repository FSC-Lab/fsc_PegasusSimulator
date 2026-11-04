// ---------- 5. the push plate (valve frame, mm; Pegasus archive/valve_push_plate_20261104/tools/page_data.py) ----------
const PD = /*PAGE_DATA*/null;
const PSIM = /*SIM_DATA*/null;
const PL = { x0: 100, x1: 220, y0: -24.4, y1: -19.4, z0: 125, z1: 204 };       // the plate
const PCH = [[154.5, 157.5], [162.5, 165.5]], PCH_Y = [-19.4, -4.4];           // the clip's cheeks
const PFIN = { x0: 157.5, x1: 162.5, y0: -19.4, y1: 20.6, z0: 74, z1: 204 };
const PROOT = { x0: 155, x1: 165, y0: -24.4, y1: 25.6, z0: 54, z1: 74 };
const PBEAM = { x0: -25, x1: 200.1, y0: -25, y1: 25.7, z0: 40, z1: 54 };
const PMK = H.markers;                                                          // C, D, E at the handle end
const PVIEWS = {
  iso: { eye: { x: 0.95, y: -1.15, z: 0.55 }, up: { x: 0, y: 0, z: 1 }, center: { x: 0.0, y: -0.08, z: 0.05 } },
  drone: { eye: { x: 0.0, y: -1.75, z: 0.25 }, up: { x: 0, y: 0, z: 1 }, center: { x: 0.0, y: 0.0, z: 0.05 } },
  side: { eye: { x: 1.75, y: -0.1, z: 0.2 }, up: { x: 0, y: 0, z: 1 }, center: { x: 0.0, y: -0.08, z: 0.05 } },
  top: { eye: { x: 0.0, y: -0.05, z: 1.9 }, up: { x: 0, y: 1, z: 0 }, center: { x: 0, y: -0.05, z: 0 } },
};
let pCamera = PVIEWS.iso;

function drawPlate3d() {
  const host = document.getElementById("p3d");
  if (typeof Plotly === "undefined") { host.textContent = "The 3D view needs the plotting library, which did not load. Figs. 11 to 13 show the same."; return; }
  const tr = [];
  tr.push(meshTrace(H.rotor, css("--rotor"), "rotor", { hoverinfo: "text", hovertext: "the rotor: beam, root and fin (v20)" }));
  for (const m of H.mounts) tr.push(meshTrace(m, "#2b2d31", "marker mount"));
  for (const m of PMK) {
    tr.push(meshTrace(ocyl(m.a, m.b, 3, 14), "#8a8d93", "stem"));
    tr.push(meshTrace(sphere(m.c[0], m.c[1], m.c[2], 15, 12), "#d9dce0", "marker " + m.name, { hoverinfo: "text", hovertext: "marker " + m.name }));
    tr.push({ type: "scatter3d", mode: "text", x: [m.c[0]], y: [m.c[1]], z: [m.c[2] + 23], text: [m.name], hoverinfo: "skip",
      textfont: { color: css("--ink-2"), size: 12, family: "IBM Plex Mono, monospace" }, showlegend: false });
  }
  const pp = css("--pplate");
  tr.push(meshTrace(box(PL.x0, PL.x1, PL.y0, PL.y1, PL.z0, PL.z1), pp, "push plate",
    { hoverinfo: "text", hovertext: "push plate 120 × 79 × 5 mm, face at y −24.4, z 125 to 204" }));
  for (const [a, b] of PCH) tr.push(meshTrace(box(a, b, PCH_Y[0], PCH_Y[1], PL.z0, PL.z1), pp, "clip", { hoverinfo: "text", hovertext: "the clip's cheek, 3 mm, on the fin's face" }));
  const claw = css("--claw");
  for (const [k, nm] of [["claw_l", "claw"], ["claw_r", "claw"], ["bracket", "gripper bracket"]]) {
    tr.push(meshTrace(PD.hulls[k], claw, nm, { opacity: k === "bracket" ? 0.55 : 0.9, hoverinfo: "text", hovertext: nm + " (closed, outer hull)" }));
  }
  const w = PD.wrist_mm, c = PD.claw_point_mm, n = PD.nose_mm;
  const ext = [0, 1, 2].map(i => c[i] + (c[i] - w[i]) * 0.35);
  tr.push({ type: "scatter3d", mode: "lines", x: [w[0], ext[0]], y: [w[1], ext[1]], z: [w[2], ext[2]],
    line: { color: css("--ink-3"), width: 3, dash: "dot" }, hoverinfo: "text", hovertext: "claw axis, 60° below level", showlegend: false });
  tr.push({ type: "scatter3d", mode: "markers+text", x: [n[0]], y: [n[1]], z: [n[2]], text: ["nose"], textposition: "middle left",
    marker: { size: 4, color: css("--warn") }, textfont: { color: css("--warn"), size: 13, family: "IBM Plex Mono, monospace" },
    hoverinfo: "text", hovertext: "the nose at the hold: 20 mm behind the face, 165 mm up", showlegend: false });
  const a0 = [160, -95, 140], a1 = [160, -55, 140];
  tr.push({ type: "scatter3d", mode: "lines", x: [a0[0], a1[0]], y: [a0[1], a1[1]], z: [a0[2], a1[2]],
    line: { color: css("--warn"), width: 6 }, hoverinfo: "text", hovertext: "the push, +y", showlegend: false });
  tr.push({ type: "scatter3d", mode: "text", x: [160], y: [-100], z: [128], text: ["push"], hoverinfo: "skip",
    textfont: { color: css("--warn"), size: 12, family: "IBM Plex Mono, monospace" }, showlegend: false });
  const axis = { visible: false };
  const layout = { paper_bgcolor: css("--panel"), margin: { l: 0, r: 0, t: 0, b: 0 }, showlegend: false, uirevision: "keep",
    scene: { aspectmode: "data", xaxis: axis, yaxis: axis, zaxis: axis, bgcolor: css("--panel"), camera: pCamera } };
  Plotly.react("p3d", tr, layout, { displaylogo: false, responsive: true, modeBarButtonsToRemove: ["toImage"] });
}
document.querySelectorAll("[data-pview]").forEach(b => b.addEventListener("click", () => {
  pCamera = PVIEWS[b.dataset.pview];
  if (typeof Plotly !== "undefined") Plotly.relayout("p3d", { "scene.camera": pCamera });
}));

// a drawing frame: world (u, v) mm -> svg, u to the right, v up
function pFrame(id, U0, U1, V0, V1, S) {
  const svg = document.getElementById(id); svg.innerHTML = "";
  svg.setAttribute("viewBox", `0 0 ${(U1 - U0) * S} ${(V1 - V0) * S}`);
  const px = u => (u - U0) * S, py = v => (V1 - v) * S;
  const ink = css("--ink"), ink2 = css("--ink-2"), ink3 = css("--ink-3"), warn = css("--warn");
  const defs = el("defs", {}, svg), aid = c => "pah-" + id + "-" + c.replace("#", "");
  for (const c of [ink2, warn]) { const mk = el("marker", { id: aid(c), viewBox: "0 0 10 10", refX: 8, refY: 5, markerWidth: 6, markerHeight: 6, orient: "auto-start-reverse" }, defs);
    el("path", { d: "M0,0L10,5L0,10Z", fill: c }, mk); }
  const f = {
    svg, px, py, S,
    R: (u0, u1, v0, v1, fill, extra) => el("rect", Object.assign({ x: px(Math.min(u0, u1)), y: py(Math.max(v0, v1)), width: Math.abs(u1 - u0) * S, height: Math.abs(v1 - v0) * S, fill }, extra || {}), svg),
    L: (u0, v0, u1, v1, stroke, w, dash) => el("line", Object.assign({ x1: px(u0), y1: py(v0), x2: px(u1), y2: py(v1), stroke, "stroke-width": w || 1 }, dash ? { "stroke-dasharray": dash } : {}), svg),
    T: (u, v, t, anchor, color, size, weight) => el("text", Object.assign({ x: px(u), y: py(v), fill: color || ink2, "font-size": size || 12, "text-anchor": anchor || "start" }, weight ? { "font-weight": weight } : {}), svg, t),
    P: (pts, fill, extra) => el("polygon", Object.assign({ points: pts.map(([u, v]) => `${px(u)},${py(v)}`).join(" "), fill }, extra || {}), svg),
    C: (u, v, r, extra) => el("circle", Object.assign({ cx: px(u), cy: py(v), r: r * S }, extra || {}), svg),
    dim: (u0, v0, u1, v1, t, off, side) => {             // a dimension line with arrows, its label beside it
      el("line", { x1: px(u0), y1: py(v0), x2: px(u1), y2: py(v1), stroke: ink2, "stroke-width": 1,
        "marker-start": `url(#${aid(ink2)})`, "marker-end": `url(#${aid(ink2)})` }, svg);
      const um = (u0 + u1) / 2, vm = (v0 + v1) / 2;
      el("text", { x: px(um) + (side === "r" ? 6 : 0), y: py(vm) + (side === "r" ? 4 : (off || -6)), fill: ink2, "font-size": 11,
        "text-anchor": side === "r" ? "start" : "middle" }, svg, t);
    },
    arrow: (u0, v0, u1, v1, color, w) => el("line", { x1: px(u0), y1: py(v0), x2: px(u1), y2: py(v1), stroke: color, "stroke-width": w || 2,
      "marker-end": `url(#${aid(color)})` }, svg),
    ink, ink2, ink3, warn,
  };
  return f;
}

// ---------- Fig. 11: face-on, from the drone ----------
function drawPlateFace() {
  const f = pFrame("pFace", 30, 330, 6, 246, 3.0);
  const { R, L, T, P, C, dim, ink, ink2, ink3, warn } = f;
  const pp = css("--pplate"), hidden = "4 3";
  // the beam end-on along its length, the root, the fin's edge (behind the plate: dashed above its bottom)
  R(60, 200.1, 40, 54, css("--rotor-3")); T(64, 44.5, "beam (its top 54)", "start", "#c9c2b4", 11);
  R(PROOT.x0, PROOT.x1, 54, 74, css("--rotor-2"));
  R(PFIN.x0, PFIN.x1, 74, PL.z0, css("--rotor"));
  R(PFIN.x0, PFIN.x1, PL.z0, 204, "none", { stroke: ink, "stroke-width": 1, "stroke-dasharray": hidden });
  T(167, 112, "fin edge", "start", ink3, 11);
  // markers: D in front (solid), C and E behind (dashed)
  for (const m of PMK) {
    const front = m.c[1] < PL.y0;
    C(m.c[0], m.c[2], 15, front ? { fill: "#d9dce0", stroke: "none" } : { fill: "none", stroke: ink2, "stroke-width": 1.2, "stroke-dasharray": hidden });
    T(m.c[0], m.c[2] - 4, m.name, "middle", front ? "#111" : ink2, 11, 600);
  }
  // the plate, the cheeks behind it
  R(PL.x0, PL.x1, PL.z0, PL.z1, pp, { "fill-opacity": 0.85 });
  for (const [a, b] of PCH) R(a, b, PL.z0, PL.z1, "none", { stroke: "#2b2350", "stroke-width": 1, "stroke-dasharray": hidden });
  // where the claw's centre may wander and still push with both tips
  R(120, 200, 140, 190, "none", { stroke: "#ffffff", "stroke-width": 1.2, "stroke-dasharray": "6 4" });
  T(124, 143, "claw centre: ±40 mm across, ±25 up/down", "start", "#ffffff", 11);
  // the closed claw's tips, where they meet the face
  for (const k of ["claw_l", "claw_r"]) P(PD.face[k], css("--claw"), { stroke: "#111", "stroke-width": 0.8 });
  // dimensions: width above the plate, height at its right, the radial positions below the beam
  dim(PL.x0, 216, PL.x1, 216, "120 wide", -6);
  dim(232, PL.z0, 232, PL.z1, "", 0, "r"); T(236, 186, "79 tall", "start", ink2, 11);
  for (const [z, t] of [[PL.z1, "top 204 (the fin's top)"], [165, "nose 165"], [PL.z0, "bottom 125"]]) {
    L(PL.x1, z, 262, z, ink3, 0.8, "2 3"); T(264, z - 3.5, t, "start", z === 165 ? warn : ink2, 11);
  }
  for (const [x, t] of [[PL.x0, "100"], [160, "160"], [PL.x1, "220"]]) { L(x, 28, x, 40, ink3, 0.8, "2 3"); T(x, 18, t, "middle", ink2, 11); }
  T(160, 8, "from the shaft along the beam [mm]", "middle", ink3, 11);
  T(36, 238, "seen from the drone, along the push", "start", ink, 12, 600);
  T(36, 200, "the claw's two closed tips", "start", ink2, 11);
  T(36, 190, "(their front corners = the nose)", "start", ink3, 11);
  el("line", { x1: f.px(94), y1: f.py(186), x2: f.px(141), y2: f.py(168), stroke: ink3, "stroke-width": 1 }, f.svg);
}

// ---------- Fig. 12: side, from the beam's outer end, the push to the right ----------
function drawPlateSide() {
  const f = pFrame("pSide", -150, 70, 30, 290, 2.15);
  const { R, L, T, P, dim, ink, ink2, ink3, warn } = f;
  const pp = css("--pplate");
  R(PBEAM.y0, PBEAM.y1, 40, 54, css("--rotor-3"));
  R(PROOT.y0, PROOT.y1, 54, 74, css("--rotor-2"));
  R(PFIN.y0, PFIN.y1, 74, 204, css("--rotor"));
  R(PCH_Y[0], PCH_Y[1], PL.z0, PL.z1, "none", { stroke: "#2b2350", "stroke-width": 1 });
  R(PL.y0, PL.y1, PL.z0, PL.z1, pp);
  T(2, 140, "fin", "middle", "#333", 11, 600);
  T(PL.y0 - 2, 214, "plate", "end", pp, 12, 600);
  el("line", { x1: f.px(-26), y1: f.py(211), x2: f.px(-22), y2: f.py(200), stroke: pp, "stroke-width": 1 }, f.svg);
  // the claw at its hold
  P(PD.side.bracket, css("--claw"), { "fill-opacity": 0.55 });
  for (const k of ["claw_l", "claw_r"]) P(PD.side[k], css("--claw"), { stroke: "#111", "stroke-width": 0.8, "fill-opacity": 0.9 });
  const w = PD.wrist_mm, c = PD.claw_point_mm, n = PD.nose_mm;
  L(w[1], w[2], c[1] + (c[1] - w[1]) * 0.45, c[2] + (c[2] - w[2]) * 0.45, ink3, 1, "3 3");
  el("circle", { cx: f.px(c[1]), cy: f.py(c[2]), r: 3, fill: ink }, f.svg);
  T(c[1] - 6, c[2] + 4, "claw point", "end", ink2, 11);
  el("circle", { cx: f.px(n[1]), cy: f.py(n[2]), r: 3.5, fill: warn }, f.svg);
  T(n[1] - 4, n[2] - 13, "nose", "end", warn, 12, 600);
  // the slack, the push
  dim(n[1], 150, PL.y0, 150, "20", -5);
  f.arrow(n[1] - 40, 112, n[1] - 4, 112, warn, 2.2);
  T(n[1] - 42, 116, "push", "end", warn, 12);
  // the 60 deg
  T(w[1] + 8, w[2] - 10, "claw 60° below level", "start", ink2, 11);
  T(w[1] + 8, w[2] - 22, `pose [${PD.pose_deg.join(", ")}]°`, "start", ink2, 11);
  for (const [z, t] of [[204, "204"], [165, "165"], [125, "125"], [74, "74"]]) { L(25, z, 46, z, ink3, 0.8, "2 3"); T(48, z - 3.5, t, "start", z === 165 ? warn : ink2, 11); }
  T(-146, 282, "side, from the beam's end", "start", ink, 12, 600);
  T(-146, 270, "heights above the table [mm]", "start", ink3, 11);
}

// ---------- Fig. 13: top, the push upward ----------
function drawPlateTop() {
  const f = pFrame("pTop", 70, 285, -82, 66, 2.5);
  const { R, T, C, dim, ink, ink2, ink3, warn } = f;
  const pp = css("--pplate");
  R(70, PBEAM.x1, PBEAM.y0, PBEAM.y1, css("--rotor-3"));
  T(74, -21, "beam", "start", ink3, 11);
  R(PROOT.x0, PROOT.x1, PROOT.y0, PROOT.y1, css("--rotor-2"));
  R(PFIN.x0, PFIN.x1, PFIN.y0, PFIN.y1, css("--rotor"));
  for (const [a, b] of PCH) R(a, b, PCH_Y[0], PCH_Y[1], pp, { "fill-opacity": 0.85, stroke: "#2b2350", "stroke-width": 0.6 });
  R(PL.x0, PL.x1, PL.y0, PL.y1, pp);
  for (const m of PMK) {
    C(m.c[0], m.c[1], 15, Object.assign({ fill: "none", stroke: ink2, "stroke-width": 1.2 }, m.c[2] + 15 < PL.z0 ? { "stroke-dasharray": "4 3" } : {}));
    T(m.c[0], m.c[1] - 4, m.name, "middle", ink2, 11, 600);
  }
  // the claw tips at the hold, from above
  for (const k of ["claw_l", "claw_r"]) {
    const pts = PD.hulls[k].x.map((x, i) => [x, PD.hulls[k].y[i]]);
    const near = pts.filter(p => p[1] > PD.nose_mm[1] - 26);
    const hull = (Q => { Q = Q.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]); const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
      const lo = [], up = []; for (const p of Q) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
      for (const p of Q.slice().reverse()) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
      return lo.slice(0, -1).concat(up.slice(0, -1)); })(near);
    f.P(hull, css("--claw"), { stroke: "#111", "stroke-width": 0.8 });
  }
  dim(PL.x0, -36, PL.x1, -36, "120", 14);
  dim(232, PL.y0, 232, PL.y1, "5", 0, "r");
  dim(170, PCH_Y[0], 170, PCH_Y[1], "", 0, "r"); T(173, -14, "clip 15", "start", ink2, 11);
  f.arrow(86, 26, 86, 56, warn, 2.2);
  T(92, 40, "push", "start", warn, 12);
  T(150, 60, "from above", "start", ink, 12, 600);
  T(74, -72, "← toward the shaft", "start", ink3, 11);
}

function plateTables() {
  const why = [
    "<strong>Width 120 mm.</strong> The closed tips reach 20 mm either side of the claw's centre, and the centre may sit up to about 40 mm off the fin's radius. In Isaac under the drift it wandered up to 25 mm across before the turn and 15 mm during it, and the model's claw point can be 8–16 mm off the real one (11-03 needed about 13). 2 × (20 + 40) = 120.",
    "<strong>Height 79 mm, top flush with the fin's top.</strong> The nose meets the face at 165 mm and the tips' contact spans −5 to +9 mm around it; the box allows ±25 mm of height error above and below that. A flush top is the simplest to fit and keeps the fin's top as the highest point.",
    "<strong>Why 165 mm up, and not higher or lower.</strong> The landing gear sets it. The left skid passes over the hub, where marker B's ball tops out 140 mm up. Leading its circle by 90 mm, the vehicle must still clear it: that needs the body about 0.43 m up, and with the claw 60° down the nose then sits at 165 mm. A plate higher on the fin would allow a flatter claw but would no longer fit the fin.",
    "<strong>5 mm thick, PTFE-faced.</strong> The plate carries under 1 N. The face's friction is the only thing that can push the claw sideways, so keep it slippery.",
  ];
  document.getElementById("pWhy").innerHTML = why.map(s => `<li>${s}</li>`).join("");
  document.getElementById("pPoseText").innerHTML = "Every pose that keeps the gear clear points the claw about 60° below level (fold q2 + q3 = 30°), because the handle is low. Within that family the yaw lever, the nose's lead ahead of the centre of mass, changes by only 5–10 % before q2 nears its −20° stop, so the pose cannot buy yaw margin; the contact does that. What decides the pose is the vehicle's motion against the claw held in the room. In the turn the vehicle runs ahead of and behind its circle, the arm folds and unfolds to take it up, and q3 must stay between the planner's 3° floor and the fin guard's 40° jam limit. <strong>Keep today's [0, 5, 25, 0]°.</strong> It leaves the most even room for that motion: 28 mm forward and 29 mm back before a q3 guard trips (last column). In Isaac under drift its q3 peaked at 31.8–34.5° in the turn; at [0, 0, 30, 0] it reached 37.2–39.2°, within 0.8° of the jam limit. It is also the pose the valve configuration already flies.";
  let h = "<thead><tr><th>Pose [deg]</th><th>Yaw lever</th><th>q2 to −20 / +45</th><th>q3 to 10 / 50</th><th>σ_nd</th><th>Body height</th><th>Gear over the tallest ball, held / leading 90 mm</th><th>Base stroke, claw held, to the joint ranges: fwd / back / up / down</th><th>The same, to the flown q3 guards (3° / 40°)</th></tr></thead><tbody>";
  const same = (a, b) => a.length === b.length && a.every((v, i) => v === b[i]);
  for (const r of PD.poses) {
    const chosen = same(r.pose, PD.pose_deg), today = same(r.pose, [0, 5, 25, 0]);
    const tag = [chosen ? "recommended" : "", today ? "today's" : ""].filter(Boolean).join(", ");
    const sg = r.stroke_guard_mm;
    h += `<tr${chosen ? ' class="close"' : ""}><td>[${r.pose.join(", ")}]${tag ? " · " + tag : ""}</td><td class="n">${r.lever_mm.toFixed(0)} mm</td>` +
      `<td class="n">${r.q2_margin[0]}° / ${r.q2_margin[1]}°</td><td class="n">${r.q3_margin[0]}° / ${r.q3_margin[1]}°</td><td class="n">${r.sigma.toFixed(3)}</td>` +
      `<td class="n">${r.body_z_mm.toFixed(0)} mm</td><td class="n">${r.marker_nominal_mm.toFixed(0)} / ${r.marker_worst_mm.toFixed(0)} mm</td>` +
      `<td class="n">${r.stroke_mm.fwd} / ${r.stroke_mm.back} / ${r.stroke_mm.up} / ${r.stroke_mm.down} mm</td>` +
      `<td class="n">${sg ? `${sg.fwd} / ${sg.back} / ${sg.up} / ${sg.down} mm` : "—"}</td></tr>`;
  }
  document.getElementById("pPoseTable").innerHTML = h + "</tbody>";
  if (PSIM) {
    document.getElementById("pSimText").innerHTML = PSIM.text;
    let s = "<thead><tr>" + PSIM.cols.map(c => `<th>${c}</th>`).join("") + "</tr></thead><tbody>";
    for (const r of PSIM.rows) s += "<tr" + (r.cls ? ` class="${r.cls}"` : "") + ">" + r.cells.map((c, i) => `<td${i >= 3 ? ' class="n"' : ""}>${c}</td>`).join("") + "</tr>";
    document.getElementById("pSimTable").innerHTML = s + "</tbody>";
    document.getElementById("pSimNote").innerHTML = PSIM.note;
  }
}
drawPlate3d(); drawPlateFace(); drawPlateSide(); drawPlateTop(); plateTables();
