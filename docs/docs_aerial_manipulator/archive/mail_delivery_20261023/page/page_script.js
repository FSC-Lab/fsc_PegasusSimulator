(function () {
  "use strict";
  const C = JSON.parse(document.getElementById("core-info").textContent);
  const F = JSON.parse(document.getElementById("frame-info").textContent);
  const SCAD_CORE = JSON.parse(document.getElementById("core-scad").textContent);
  const SCAD_FRAME = JSON.parse(document.getElementById("frame-scad").textContent);
  const HJ = JSON.parse(document.getElementById("holders-info").textContent);   // holders/holders.json
  const VALVE = HJ.valve;
  const fmt = (v, d) => Number(v).toFixed(d);
  const sgn = (v, d) => (v > 0 ? "+" : v < 0 ? "−" : "") + Math.abs(v).toFixed(d);
  const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  const $ = (id) => document.getElementById(id);
  function b64bytes(id) {
    const raw = atob($(id).textContent.trim());
    const u = new Uint8Array(raw.length);
    for (let i = 0; i < raw.length; i++) u[i] = raw.charCodeAt(i);
    return u;
  }
  function row(tbody, cells, cls) {
    const tr = document.createElement("tr");
    if (cls) tr.className = cls;
    cells.forEach(c => {
      const td = document.createElement("td");
      if (c && typeof c === "object") { td.textContent = c.t; if (c.num) td.className = "num"; }
      else td.textContent = c;
      tr.appendChild(td);
    });
    tbody.appendChild(tr);
  }

  /* ================= tables and numbers ================= */
  const L = C.L, W = C.W, T = C.T, SF = C.solid_far;
  $("c-com").textContent = fmt(C.com_from_far_mm, 0) + " mm";
  $("c-com-s").textContent = "from the far end · " + fmt(SF.com_from_far_mm, 0) + " with a solid far band";
  $("c-mass").textContent = fmt(C.mass_g, 0) + " g";
  $("c-mass-s").textContent = "±10 %, weigh it · " + fmt(SF.mass_g, 0) + " g with a solid far band";
  const cb = document.querySelector("#c-budget tbody");
  C.budget.forEach(r => row(cb, [r[0], { t: fmt(r[1], 1), num: 1 }, { t: fmt(L - r[2], 1), num: 1 }]));
  row(cb, ["Total, estimate", { t: fmt(C.mass_g, 1), num: 1 }, { t: fmt(C.com_from_far_mm, 1), num: 1 }], "total");
  row(cb, ["Total, far band at 100 % infill", { t: fmt(SF.mass_g, 1), num: 1 }, { t: fmt(SF.com_from_far_mm, 1), num: 1 }], "total");
  const ch = document.querySelector("#c-holes tbody");
  C.holes.forEach((h, i) => row(ch, ["W" + (i + 1), { t: fmt(h[0], 0), num: 1 }, { t: fmt(h[1], 0), num: 1 }, { t: fmt(h[2], 0), num: 1 }, { t: fmt(h[3], 0), num: 1 }]));
  // task margins from tools/mail_geometry.py (fore-aft +-20, lateral +-40, vertical +-12 mm), run with each
  // option's centre of mass and mass; "deeper" = jaw tips 20 mm from the wall instead of 35
  const cmg = document.querySelector("#c-margins tbody");
  [["Printed as drawn", fmt(C.mass_g, 0), fmt(C.com_from_far_mm, 0), "7", "5", "−25"],
   ["As drawn, release 15 mm deeper", fmt(C.mass_g, 0), fmt(C.com_from_far_mm, 0), "7", "20", "−10"],
   ["Far band at 100 % infill", fmt(SF.mass_g, 0), fmt(SF.com_from_far_mm, 0), "18", "16", "−14"],
   ["Far band solid, release 15 mm deeper", fmt(SF.mass_g, 0), fmt(SF.com_from_far_mm, 0), "18", "31", "0"],
   ["Rev B with the steel bar, for comparison", "65", "40", "40", "38", "+8"]].forEach((r, i) =>
    row(cmg, [r[0], { t: r[1], num: 1 }, { t: r[2], num: 1 }, { t: r[3], num: 1 }, { t: r[4], num: 1 }, { t: r[5], num: 1 }], i === 3 ? "best" : ""));

  $("f-size").textContent = fmt(2 * F.xo, 1) + " × " + fmt(2 * F.hy, 0);
  $("f-fits").textContent = "Ø" + fmt(2 * F.hat_r, 0) + " · " + fmt(F.box_d, 0);
  const MK = F.markers, CEN = MK.centroid;
  $("f-dmin").textContent = fmt(MK.min_distance, 0);
  $("f-ddiff").textContent = fmt(MK.min_difference, 1);
  // the holder names: a design used twice gets a subscript
  const HNAME = F.holders.map((h, i) => {
    const same = F.holders.map((q, j) => [q, j]).filter(([q]) => q.t === h.t);
    return same.length > 1 ? h.t + "₁₂₃"[same.findIndex(([, j]) => j === i)] : h.t;
  });
  const n1 = (v) => fmt(v, 1).replace(/\.0$/, "");
  const DF = { rin: "R " + n1(F.r_in), hatd: "Ø" + n1(2 * F.hat_r), outline: n1(2 * F.xo) + " × " + n1(2 * F.hy),
               xf: "x = ±" + n1(F.xf), xo: "±" + n1(F.xo), stop: "x = " + sgn(F.slot_stop, 1),
               slothy: "y ±" + n1(F.slot_hy) + " (" + n1(2 * F.slot_hy) + " wide)" };
  document.querySelectorAll("[data-f]").forEach(e => { e.textContent = DF[e.dataset.f]; });
  // every dimension
  const fdm = $("f-dims");
  const grp = (t) => { const tr = document.createElement("tr"); tr.className = "grp"; const td = document.createElement("td"); td.colSpan = 4; td.textContent = t; tr.appendChild(td); fdm.appendChild(tr); };
  const dm = (a, b, v, n) => row(fdm, [a, b, v, n]);
  grp("Plate");
  dm("Outline along x", "−" + n1(F.xo) + " to +" + n1(F.xo), n1(2 * F.xo), "= skirt arc radius + " + n1(F.skirt_wall) + " mm wall");
  dm("Outline along y", "−" + n1(F.hy) + " to +" + n1(F.hy), n1(2 * F.hy), (2 * F.hy) + " < the mailbox's " + n1(F.box_w) + ": the ends sit " + n1(F.box_side_gap) + " in from each side");
  dm("Thickness", "z 0 to " + n1(F.plate_t), n1(F.plate_t), "z = 0 is the underside, the datum");
  dm("Corners", "", "sharp", "a corner radius up to 5 mm changes nothing");
  grp("Skirts (two, at the ±x ends, under the plate)");
  dm("Depth below the underside", "z −" + n1(F.skirt_d) + " to 0", n1(F.skirt_d), "shallower than the hat cap's rim (the drawing assumes " + n1(F.hat_t) + " thick)");
  dm("Outer face", "x = ±" + n1(F.xo), n1(F.xo), "flush with the plate's ends");
  dm("Inner face, middle: an arc", "R on O, for |y| ≤ " + n1(F.y_kink), n1(F.r_in), "measured cap Ø" + n1(2 * F.hat_r) + " + " + n1(F.clr) + " a side; drops over the hat's rim");
  dm("Inner face, near the corners: flats", "x = ±" + n1(F.xf) + ", for |y| ≥ " + n1(F.y_kink), n1(2 * F.xf) + " apart", "mailbox depth " + n1(F.box_d) + " + " + n1(F.clr) + " a side; close on its front and back faces");
  dm("Where the arc meets the flats", "y = ±" + fmt(F.y_kink, 1), fmt(F.y_kink, 1), "√(" + n1(F.r_in) + "² − " + n1(F.xf) + "²)");
  dm("Width on the centre line", "x " + n1(F.r_in) + " to " + n1(F.xo), n1(F.xo - F.r_in), "thinnest part of the skirt");
  dm("Extent along y", "−" + n1(F.hy) + " to +" + n1(F.hy), n1(2 * F.hy), "full width of the plate, both skirts");
  grp("U-slot (open to the −x end, through plate and skirt)");
  dm("Width", "y −" + n1(F.slot_hy) + " to +" + n1(F.slot_hy), n1(2 * F.slot_hy), "envelope " + n1(F.env[1]) + " + " + n1(F.clr) + " a side");
  dm("From the −x end to the stop", "x −" + n1(F.xo) + " to " + sgn(F.slot_stop, 1), n1(F.xo + F.slot_stop), "the stop: the envelope's far end; then its near edge hangs " + n1(F.overhang) + " past the cap's rim");
  dm("Stop position", "x = " + sgn(F.slot_stop, 1), sgn(F.slot_stop, 1), "= −(" + n1(F.hat_r) + " + " + n1(F.overhang) + ") + " + n1(F.env[0]));
  grp("Pockets (five, in the plate top; each listed below)");
  dm("Depth", "z " + n1(F.plate_t - F.pocket_d) + " to " + n1(F.plate_t), n1(F.pocket_d), "= the holders' base thickness: the base sits flush");
  dm("Size", "holder base + " + n1(F.pocket_clr) + " a side", "see below", "rounded to the nearest 0.5 mm");
  dm("Keep inside", "|x| < " + n1(F.xf - 1), n1(F.xf - 1), "every holder stands over the mailbox's top");
  const fpk = document.querySelector("#f-pockets tbody");
  F.holders.forEach((h, i) => {
    const tr = document.createElement("tr");
    [[null, HNAME[i]], [1, sgn(h.x, 0) + ", " + sgn(h.y, 0)], [1, fmt(h.pocket[0], 1) + " × " + fmt(h.pocket[1], 1)],
     [1, sgn(h.box[0], 2) + " → " + sgn(h.box[1], 2)], [1, sgn(h.box[2], 2) + " → " + sgn(h.box[3], 2)],
     [1, n1(F.pocket_d)], [1, h.heading + "°"], [1, h.ball.map(v => fmt(v, 1)).join(", ")]].forEach(([num, t], k) => {
      const td = document.createElement("td");
      if (num) td.className = "num";
      if (k === 0) { const sw = document.createElement("span"); sw.className = "sw " + h.t; sw.style.marginRight = "7px"; td.appendChild(sw); }
      td.appendChild(document.createTextNode(t));
      tr.appendChild(td);
    });
    fpk.appendChild(tr);
  });
  const fdv = $("f-derived");
  row(fdv, ["Arc meets the flats, y", { t: "±" + fmt(F.y_kink, 1), num: 1 }, "√(" + n1(F.r_in) + "² − " + n1(F.xf) + "²): where the R " + n1(F.r_in) + " circle crosses x = ±" + n1(F.xf)]);
  row(fdv, ["Skirt width on the centre line", { t: fmt(F.xo - F.r_in, 1), num: 1 }, n1(F.xo) + " − " + n1(F.r_in)]);
  row(fdv, ["U-slot length from the −x edge", { t: fmt(F.xo + F.slot_stop, 1), num: 1 }, n1(F.xo) + " − " + n1(-F.slot_stop)]);
  row(fdv, ["Frame end to the mailbox side", { t: fmt(F.box_side_gap, 1), num: 1 }, "(" + n1(F.box_w) + " − " + n1(2 * F.hy) + ") / 2, each side"]);
  row(fdv, ["Marker centroid from O", { t: CEN.map(v => fmt(v, 1)).join(", "), num: 1 }, "obj_0's pivot (Motive's default): the mean of the five marker centres"]);
  const O = F.offsets;
  const trim = (v) => { const s = (v > 0 ? "+" : v < 0 ? "−" : "") + Math.abs(v).toFixed(2).replace(/0+$/, "").replace(/\.$/, ""); return s === "" ? "0" : s; };
  const off = (v) => v.map(trim).join(", ");
  const ob = document.querySelector("#f-offsets tbody");
  const LOC = F.locator, locs = (v) => v.slice(0, 3).map(x => (x > 0 ? "+" : x < 0 ? "−" : "") + Math.abs(x).toFixed(4)).join(", ");
  row(ob, ["Pick", "the hat, open end toward the drone's start", "Envelope row", locs(LOC.pick), off(O.pick) + " (claw " + off(O.pick_claw) + ")", "the frame's"]);
  row(ob, ["Place", "the mailbox top, +x into the mouth", "Mailbox row", locs(LOC.place), off(O.place) + " (claw " + off(O.place_claw) + ")", "the frame's"]);
  const fd = $("f-derive");
  row(fd, ["Pick dx", { t: sgn(O.pick[0], 1), num: 1 }, `The envelope's centre, set by the U-slot's stop: −(hat radius ${F.hat_r} + overhang ${F.overhang} − half its length ${F.env[0] / 2})`]);
  row(fd, ["Pick dz", { t: sgn(O.pick[2], 2), num: 1 }, `Half the wrapped envelope, ${F.env[2]} mm, above the hat's top`]);
  row(fd, ["Place dx", { t: sgn(O.place[0], 1), num: 1 }, `The envelope's centre 9 mm inside the front face (−${F.box_d / 2} + 9): the jaw tips stay 35 mm out, they reach 16 mm past the claw point, and the claw sits 60 mm behind the centre. For the deeper release in the core's margin table, use ${sgn(O.place[0] + 15, 1)}`]);
  row(fd, ["Place dz", { t: sgn(O.place[2], 1), num: 1 }, "The mouth's centre is 70 mm under the box top (top edge 25 mm down, 90 mm tall). The claw rides 21.5 mm above it, because the envelope hangs 0–20° nose-down in the jaws (drawing 9)"]);
  row(fd, ["dy, both", { t: "0", num: 1 }, "Centred on the hat by the skirts and on the mailbox by eye (17.5 mm in from each side)"]);
  row(fd, ["Planner offsets", { t: "target − centroid", num: 1 }, `The rows above less the marker centroid (${CEN.map(v => fmt(v, 2)).join(", ")}) from O, in metres: pick ${locs(LOC.pick)}, place ${locs(LOC.place)}`]);
  $("src-core").textContent = SCAD_CORE;
  $("src-frame").textContent = SCAD_FRAME;

  /* ================= SVG helpers ================= */
  const NS = "http://www.w3.org/2000/svg";
  function el(parent, tag, attrs, text) {
    const e = document.createElementNS(NS, tag);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    if (text !== undefined) e.textContent = text;
    parent.appendChild(e);
    return e;
  }
  function arrowHead(g, x, y, dx, dy, s) {
    const n = Math.hypot(dx, dy), ux = dx / n, uy = dy / n, px = -uy, py = ux;
    el(g, "polygon", { class: "arrow", points: [
      [x, y], [x - ux * s * 2.2 + px * s * 0.7, y - uy * s * 2.2 + py * s * 0.7],
      [x - ux * s * 2.2 - px * s * 0.7, y - uy * s * 2.2 - py * s * 0.7]].map(p => p.join(",")).join(" ") });
  }
  function dimH(g, x0, x1, yRef, yDim, label, fs, s, yRef1) {
    const d = Math.sign(yDim - yRef) * s * 1.2;
    el(g, "line", { class: "dim", x1: x0, y1: yRef, x2: x0, y2: yDim + d });
    el(g, "line", { class: "dim", x1: x1, y1: yRef1 === undefined ? yRef : yRef1, x2: x1, y2: yDim + d });
    el(g, "line", { class: "dim", x1: x0, y1: yDim, x2: x1, y2: yDim });
    arrowHead(g, x0, yDim, -1, 0, s); arrowHead(g, x1, yDim, 1, 0, s);
    const up = yDim < yRef;
    if (label) el(g, "text", { x: (x0 + x1) / 2, y: up ? yDim - s * 1.1 : yDim + fs + s * 0.6, "font-size": fs, "text-anchor": "middle" }, label);
  }
  function dimV(g, y0, y1, xRef, xDim, label, fs, s, yText) {
    const d = Math.sign(xDim - xRef) * s * 1.2;
    el(g, "line", { class: "dim", x1: xRef, y1: y0, x2: xDim + d, y2: y0 });
    el(g, "line", { class: "dim", x1: xRef, y1: y1, x2: xDim + d, y2: y1 });
    el(g, "line", { class: "dim", x1: xDim, y1: y0, x2: xDim, y2: y1 });
    arrowHead(g, xDim, y0, 0, -1, s); arrowHead(g, xDim, y1, 0, 1, s);
    const tx = xDim + (xDim > xRef ? 1 : -1) * (fs * 0.9), ty = yText === undefined ? (y0 + y1) / 2 : yText;
    if (label) el(g, "text", { x: tx, y: ty, "font-size": fs, "text-anchor": "middle",
                    transform: `rotate(-90 ${tx} ${ty})`, "dominant-baseline": "middle" }, label);
  }
  // ordinate dimensions: a leader from the feature to a common offset, its value at the end
  function ordX(g, x, y0, y1, label, fs) {
    el(g, "line", { class: "dim", x1: x, y1: y0, x2: x, y2: y1 });
    const up = y1 < y0, ty = up ? y1 - 0.8 : y1 + 0.8;
    el(g, "text", { x: x, y: ty, "font-size": fs, "text-anchor": up ? "start" : "end", "dominant-baseline": "middle",
                    transform: `rotate(-90 ${x} ${ty})` }, label);
  }
  function ordY(g, y, x0, x1, label, fs) {
    el(g, "line", { class: "dim", x1: x0, y1: y, x2: x1, y2: y });
    const left = x1 < x0;
    el(g, "text", { x: left ? x1 - 0.8 : x1 + 0.8, y: y, "font-size": fs, "text-anchor": left ? "end" : "start", "dominant-baseline": "middle" }, label);
  }
  function hatch(svg, id, w) {
    const defs = el(svg, "defs", {});
    const pat = el(defs, "pattern", { id: id, width: w, height: w, patternUnits: "userSpaceOnUse", patternTransform: "rotate(45)", class: "hatch" });
    el(pat, "line", { x1: 0, y1: 0, x2: 0, y2: w });
  }
  const pts = (list, X, Y) => list.map(p => X(p[0]) + "," + Y(p[1])).join(" ");

  /* ================= core: drawing 1, top view ================= */
  const yA = (C.holes[0][2] + C.holes[0][3]) / 2;     // section A-A through the first window row
  (function coreTop() {
    const svg = $("c-top");
    const m = { l: 14, r: 30, t: 27, b: 33 };
    svg.setAttribute("viewBox", `0 0 ${L + m.l + m.r} ${W + m.t + m.b}`);
    const X = x => m.l + x, Y = y => m.t + (W - y);       // y from the right long edge = the bottom of the drawing
    const fs = 3.1, s = 0.9;
    hatch(svg, "c-far", 2.2);
    const g = el(svg, "g", {});
    el(g, "rect", { class: "pla", x: X(0), y: Y(W), width: L, height: W });
    el(g, "rect", { x: X(L - C.far), y: Y(W), width: C.far, height: W, fill: "url(#c-far)" });
    C.holes.forEach(h => el(g, "rect", { class: "bay", x: X(h[0]), y: Y(h[3]), width: h[1] - h[0], height: h[3] - h[2] }));
    C.holes.forEach(h => el(g, "rect", { class: "outline", x: X(h[0]), y: Y(h[3]), width: h[1] - h[0], height: h[3] - h[2] }));
    el(g, "rect", { class: "outline", x: X(0), y: Y(W), width: L, height: W });
    // reference marks: the grasp line and the two centres of mass
    const com = L - C.com_from_far_mm, comS = L - SF.com_from_far_mm;
    el(g, "line", { class: "grasp", x1: X(15), y1: Y(W), x2: X(15), y2: Y(0) });
    el(g, "circle", { class: "com", cx: X(com), cy: Y(W / 2), r: 1.25 });
    el(g, "circle", { class: "comline", cx: X(comS), cy: Y(W / 2), r: 1.25 });
    // section line A-A
    [[X(0) - 6, X(0) - 1.2], [X(L) + 1.2, X(L) + 5.6]].forEach(([a, b]) => el(g, "line", { class: "cut", x1: a, y1: Y(yA), x2: b, y2: Y(yA) }));
    el(g, "text", { x: X(0) - 3.6, y: Y(yA) - 1.6, "font-size": fs, "text-anchor": "middle", class: "lbl" }, "A");
    el(g, "text", { x: X(L) + 3.4, y: Y(yA) - 1.6, "font-size": fs, "text-anchor": "middle", class: "lbl" }, "A");
    // datum
    el(g, "circle", { class: "pivot", cx: X(0), cy: Y(0), r: 1.1 });
    el(g, "text", { x: X(0) - 1.8, y: Y(0) + 3.2, "font-size": fs * 0.85, "text-anchor": "end", class: "lbl" }, "0,0");
    // labels
    el(g, "text", { x: X(25), y: Y(W / 2), "font-size": fs, "text-anchor": "middle", class: "lbl", transform: `rotate(-90 ${X(25)} ${Y(W / 2)})`, "dominant-baseline": "middle" }, "GRIP BAND · SOLID");
    const xf = L - C.far / 2;
    el(g, "text", { x: X(xf), y: Y(64), "font-size": fs, "text-anchor": "middle", class: "lbl" }, "FAR BAND · SOLID");
    el(g, "text", { x: X(xf), y: Y(64) + fs * 1.25, "font-size": fs * 0.85, "text-anchor": "middle", class: "muted" }, "optional: 100 % infill here");
    const h1 = C.holes[0];
    el(g, "text", { x: X((h1[0] + h1[1]) / 2), y: Y((h1[2] + h1[3]) / 2) - 0.6, "font-size": fs * 0.95, "text-anchor": "middle", class: "lbl" }, C.holes.length + " × THROUGH");
    el(g, "text", { x: X((h1[0] + h1[1]) / 2), y: Y((h1[2] + h1[3]) / 2) + fs + 0.1, "font-size": fs * 0.95, "text-anchor": "middle" }, fmt(C.hole[0], 0) + " × " + fmt(C.hole[1], 0));
    // dimensions: along x on top, along y on the right, reference ones below
    const top = Y(W), bot = Y(0);
    const xs = [0, C.grip, C.holes[0][1], C.holes[2][0], C.holes[2][1], L];
    for (let i = 0; i < xs.length - 1; i++) dimH(g, X(xs[i]), X(xs[i + 1]), top, top - 8, fmt(xs[i + 1] - xs[i], 0), fs, s);
    dimH(g, X(0), X(L), top, top - 18, fmt(L, 0), fs, s);
    const ys = [0, C.holes[0][2], C.holes[0][3], C.holes[1][2], C.holes[1][3], W];
    for (let i = 0; i < ys.length - 1; i++) dimV(g, Y(ys[i + 1]), Y(ys[i]), X(L), X(L) + 8, fmt(ys[i + 1] - ys[i], 0), fs, s);
    dimV(g, Y(W), Y(0), X(L), X(L) + 18, fmt(W, 0), fs, s);
    dimH(g, X(0), X(15), bot, bot + 7, "(15)", fs, s);
    el(g, "text", { x: X(13.6), y: Y(68), "font-size": fs * 0.8, "text-anchor": "middle", class: "redt", transform: `rotate(-90 ${X(13.6)} ${Y(68)})` }, "grasp line");
    dimH(g, X(0), X(com), bot, bot + 15, "(" + fmt(com, 0) + ") CoM as drawn", fs, s);
    dimH(g, X(0), X(comS), bot, bot + 23, "(" + fmt(comS, 0) + ") CoM, far band at 100 % infill", fs, s);
  })();

  /* ================= core: drawing 2, section A-A ================= */
  (function coreSection() {
    const svg = $("c-sec");
    const k = 2;
    const m = { l: 24, r: 10, t: 18, b: 20 };
    svg.setAttribute("viewBox", `0 0 ${L * k + m.l + m.r} ${T * k + m.t + m.b}`);
    const X = x => m.l + x * k, Z = z => m.t + (T - z) * k;
    const fs = 6.0, s = 1.6;
    hatch(svg, "c-hatch", 3.2);
    const g = el(svg, "g", {});
    const cuts = C.holes.filter(h => h[2] < yA && yA < h[3]).map(h => [h[0], h[1]]).sort((a, b) => a[0] - b[0]);
    const segs = [];
    let x = 0;
    cuts.forEach(c => { segs.push([x, c[0]]); x = c[1]; });
    segs.push([x, L]);
    segs.forEach(([a, b]) => {
      const P = pts([[a, 0], [b, 0], [b, T], [a, T]], X, Z);
      el(g, "polygon", { class: "pla", points: P });
      el(g, "polygon", { points: P, fill: "url(#c-hatch)" });
      el(g, "polygon", { class: "outline", points: P });
    });
    cuts.forEach(c => el(g, "text", { x: X((c[0] + c[1]) / 2), y: Z(T / 2) + fs * 0.35, "font-size": fs * 0.8, "text-anchor": "middle", class: "lbl muted" }, "WINDOW"));
    el(g, "text", { x: X(0), y: Z(T) - 5, "font-size": fs * 0.8, class: "lbl" }, "NEAR EDGE");
    el(g, "text", { x: X(L), y: Z(T) - 5, "font-size": fs * 0.8, "text-anchor": "end", class: "lbl redt" }, "FAR END");
    dimV(g, Z(T), Z(0), X(0), X(0) - 9, fmt(T, 1), fs, s);
    const xs = [0, ...cuts.flat(), L];
    for (let i = 0; i < xs.length - 1; i++) dimH(g, X(xs[i]), X(xs[i + 1]), Z(0), Z(0) + 8, fmt(xs[i + 1] - xs[i], 0), fs, s);
  })();

  /* ================= frame: shared geometry ================= */
  const gIn = y => (Math.abs(y) < F.y_kink ? Math.sqrt(F.r_in * F.r_in - y * y) : F.xf);
  const plate = [[-F.xo, -F.hy], [F.xo, -F.hy], [F.xo, F.hy], [-F.xo, F.hy], [-F.xo, F.slot_hy], [F.slot_stop, F.slot_hy],
                 [F.slot_stop, -F.slot_hy], [-F.xo, -F.slot_hy]];
  function inner(y0, y1, sign) {
    const ys = [y0];
    const n = 60;
    for (let i = 1; i < n; i++) ys.push(y0 + (y1 - y0) * i / n);
    [F.y_kink, -F.y_kink].forEach(k => { if ((k - y0) * (k - y1) < 0) ys.push(k); });
    ys.push(y1);
    ys.sort((a, b) => (y1 > y0 ? a - b : b - a));
    return ys.map(y => [sign * gIn(y), y]);
  }
  const right = inner(F.hy, -F.hy, 1), leftUp = inner(F.hy, F.slot_hy, -1), leftDn = inner(-F.slot_hy, -F.hy, -1);
  const skirts = [[...right, [F.xo, -F.hy], [F.xo, F.hy]],
                  [...leftUp, [-F.xo, F.slot_hy], [-F.xo, F.hy]],
                  [...leftDn, [-F.xo, -F.hy], [-F.xo, -F.slot_hy]]];
  function frameCanvas(id, m) {
    const svg = $(id);
    svg.setAttribute("viewBox", `0 0 ${2 * F.xo + m.l + m.r} ${2 * F.hy + m.t + m.b}`);
    return { svg, X: x => m.l + (x + F.xo), Y: y => m.t + (F.hy - y) };
  }
  function centreLines(g, X, Y, ext) {
    el(g, "line", { class: "cl", x1: X(-F.xo) - ext, y1: Y(0), x2: X(F.xo) + ext, y2: Y(0) });
    el(g, "line", { class: "cl", x1: X(0), y1: Y(F.hy) - ext, x2: X(0), y2: Y(-F.hy) + ext });
  }

  /* ================= frame: drawing 3, top view ================= */
  (function frameCad() {
    const m = { l: 22, r: 34, t: 30, b: 36 };
    const { svg, X, Y } = frameCanvas("f-cad", m);
    const fs = 3.3, s = 0.95;
    const g = el(svg, "g", {});
    el(g, "polygon", { class: "pla", points: pts(plate, X, Y) });
    el(g, "polygon", { class: "outline", points: pts(plate, X, Y) });
    // the skirts' inner faces, under the plate: hidden lines
    [right, leftUp, leftDn].forEach(line => el(g, "polyline", { class: "hidden", points: pts(line, X, Y) }));
    const ext = 7;
    centreLines(g, X, Y, ext);
    [[Y(F.hy) - ext - 0.8, "start"], [Y(-F.hy) + ext + 0.8, "end"]].forEach(([y, a]) =>
      el(g, "text", { x: X(0), y: y, "font-size": fs, "text-anchor": a, "dominant-baseline": "middle", transform: `rotate(-90 ${X(0)} ${y})` }, "0"));
    [[Y(0), X(-F.xo) - ext], [Y(0), X(F.xo) + ext]].forEach(([y, x]) => {
      el(g, "text", { x: x < X(0) ? x - 0.8 : x + 0.8, y: y, "font-size": fs, "text-anchor": x < X(0) ? "end" : "start", "dominant-baseline": "middle" }, "0");
    });
    el(g, "text", { x: X(0) + 1.6, y: Y(0) - 1.8, "font-size": fs, class: "lbl" }, "O");
    // the marker centroid (obj_0's pivot)
    el(g, "circle", { class: "pivot", cx: X(CEN[0]), cy: Y(CEN[1]), r: 2.2 });
    el(g, "line", { class: "pivot", x1: X(CEN[0]) - 3.6, y1: Y(CEN[1]), x2: X(CEN[0]) + 3.6, y2: Y(CEN[1]) });
    el(g, "line", { class: "pivot", x1: X(CEN[0]), y1: Y(CEN[1]) - 3.6, x2: X(CEN[0]), y2: Y(CEN[1]) + 3.6 });
    // pockets: rectangle, design letter, centre ordinates; the marker above, dashed
    // which way each ordinate leader runs: the side whose path crosses the fewest
    // pockets and marker circles (its own marker counts double)
    const segHitsCircle = (ax, ay, bx, by, cx, cy, r) => {
      const dx = bx - ax, dy = by - ay, t = Math.max(0, Math.min(1, ((cx - ax) * dx + (cy - ay) * dy) / (dx * dx + dy * dy)));
      return Math.hypot(ax + t * dx - cx, ay + t * dy - cy) < r;
    };
    const segHitsBox = (ax, ay, bx, by, b) => {   // axis-aligned segments only
      if (ax === bx) return ax > b[0] && ax < b[1] && Math.max(ay, by) > b[2] && Math.min(ay, by) < b[3];
      return ay > b[2] && ay < b[3] && Math.max(ax, bx) > b[0] && Math.min(ax, bx) < b[1];
    };
    const cost = (h, ax, ay, bx, by) => F.holders.reduce((c, q) => c + (q === h
      ? 2 * segHitsCircle(ax, ay, bx, by, q.ball[0], q.ball[1], 15)
      : segHitsCircle(ax, ay, bx, by, q.ball[0], q.ball[1], 15) + 3 * segHitsBox(ax, ay, bx, by, q.box)), 0);
    const segs = [[0, -F.hy - ext, 0, F.hy + ext], [-F.xo - ext, 0, F.xo + ext, 0]];   // centre lines + every ordinate leader, frame mm
    F.holders.forEach((h, hi) => {
      const [x0, x1, y0, y1] = h.box;
      el(g, "rect", { class: "pocket", x: X(x0), y: Y(y1), width: x1 - x0, height: y1 - y0 });
      el(g, "rect", { class: "pktint pk-" + h.t, x: X(x0), y: Y(y1), width: x1 - x0, height: y1 - y0 });
      // in the pocket: the holder (letter = design), the pocket's size, the holder's turn
      const fz = fs * 0.83;
      el(g, "text", { x: X(h.x), y: Y(h.y) - 2.0, "font-size": fs * 1.15, "text-anchor": "middle", class: "lbl halo" }, HNAME[hi]);
      el(g, "text", { x: X(h.x), y: Y(h.y) + 1.9, "font-size": fz, "text-anchor": "middle", class: "halo" }, fmt(h.pocket[0], 1) + "×" + fmt(h.pocket[1], 1));
      el(g, "text", { x: X(h.x), y: Y(h.y) + 1.9 + fz * 1.2, "font-size": fz, "text-anchor": "middle", class: "halo" }, "turn " + h.heading + "°");
      el(g, "circle", { class: "ghost", cx: X(h.ball[0]), cy: Y(h.ball[1]), r: 15 });
      el(g, "line", { class: "dim", x1: X(h.ball[0]) - 1.5, y1: Y(h.ball[1]), x2: X(h.ball[0]) + 1.5, y2: Y(h.ball[1]) });
      el(g, "line", { class: "dim", x1: X(h.ball[0]), y1: Y(h.ball[1]) - 1.5, x2: X(h.ball[0]), y2: Y(h.ball[1]) + 1.5 });
      const top = cost(h, h.x, y1, h.x, F.hy + ext) <= cost(h, h.x, y0, h.x, -F.hy - ext) - (h.y > 0 ? 0 : 1);
      ordX(g, X(h.x), Y(top ? y1 : y0), top ? Y(F.hy) - ext : Y(-F.hy) + ext, sgn(h.x, 0), fs);
      segs.push(top ? [h.x, y1, h.x, F.hy + ext + 9] : [h.x, y0, h.x, -F.hy - ext - 9]);
      const left = cost(h, x0, h.y, -F.xo - ext, h.y) <= cost(h, x1, h.y, F.xo + ext, h.y) - (h.x < 0 ? 0 : 1);
      ordY(g, Y(h.y), X(left ? x0 : x1), left ? X(-F.xo) - ext : X(F.xo) + ext, sgn(h.y, 0), fs);
      segs.push(left ? [x0, h.y, -F.xo - ext - 9, h.y] : [x1, h.y, F.xo + ext + 9, h.y]);
    });
    // each marker's height: the diagonal corner of its circle clear of the leaders and pockets
    const boxHit = (b, c) => b[0] < c[1] && c[0] < b[1] && b[2] < c[3] && c[2] < b[3];
    F.holders.forEach(h => {
      const tw = 12.5, th = 3.2;           // the label's size in mm at this font
      let best = null;
      [[1, 1], [-1, 1], [1, -1], [-1, -1]].forEach(([sx, sy]) => {
        const lx = h.ball[0] + sx * 11.5, ly = h.ball[1] + sy * 12.5;
        const b = [sx > 0 ? lx : lx - tw, sx > 0 ? lx + tw : lx, ly - th / 2, ly + th / 2];
        let c = 0;
        segs.forEach(q => { c += 4 * segHitsBox(q[0], q[1], q[2], q[3], b) || (q[0] === q[2] ? (q[0] > b[0] && q[0] < b[1] && Math.max(q[1], q[3]) > b[2] && Math.min(q[1], q[3]) < b[3]) : (q[1] > b[2] && q[1] < b[3] && Math.max(q[0], q[2]) > b[0] && Math.min(q[0], q[2]) < b[1])) * 4; });
        F.holders.forEach(q => { if (boxHit(b, q.box)) c += 3; if (q !== h && Math.hypot((b[0] + b[1]) / 2 - q.ball[0], ly - q.ball[1]) < 15 + tw / 2) c += 1; });
        if (Math.abs(ly) > F.hy - 1 || b[0] < -F.xo || b[1] > F.xo) c += 0.5;
        if (best === null || c < best.c) best = { c, lx, ly, sx };
      });
      el(g, "text", { x: X(best.lx), y: Y(best.ly), "font-size": fs * 0.8, class: "muted", "dominant-baseline": "middle",
                      "text-anchor": best.sx > 0 ? "start" : "end" }, "z " + fmt(h.ball[2], 1));
    });
    // the U-slot: a chain along y = -48 to O, and its width
    const yc = Y(-F.slot_hy - 5);
    const xs = [-F.xo, F.slot_stop, 0];
    for (let i = 0; i < xs.length - 1; i++) dimH(g, X(xs[i]), X(xs[i + 1]), Y(-F.slot_hy), yc, fmt(xs[i + 1] - xs[i], 1).replace(/\.0$/, ""), fs, s);
    dimV(g, Y(F.slot_hy), Y(-F.slot_hy), X(-F.xo), X(-75), "", fs, s);
    el(g, "text", { x: X(-75) + fs * 0.9, y: Y(22), "font-size": fs, "text-anchor": "middle", transform: `rotate(-90 ${X(-75) + fs * 0.9} ${Y(22)})`, "dominant-baseline": "middle" }, fmt(2 * F.slot_hy, 1));
    el(g, "text", { x: X(-50), y: Y(16), "font-size": fs, "text-anchor": "middle", class: "lbl muted" }, "U-SLOT");
    el(g, "text", { x: X(-50), y: Y(16) + fs * 1.15, "font-size": fs * 0.85, "text-anchor": "middle", class: "muted" }, "open end, through");
    el(g, "line", { class: "dim", x1: X(CEN[0]) + 1.6, y1: Y(CEN[1]) + 1.6, x2: X(CEN[0] + 7), y2: Y(CEN[1] - 13) });
    el(g, "text", { x: X(CEN[0] + 7.6), y: Y(CEN[1] - 14.5), "font-size": fs * 0.8, class: "muted" }, "obj_0 pivot (marker centroid)");
    // the skirts (hidden): the flats' x on the bottom row, the arc's radius on a leader from O
    [-F.xf, F.xf].forEach(x => ordX(g, X(x), Y(-F.hy), Y(-F.hy) + ext, sgn(x, 1), fs));
    const ra = -40 * Math.PI / 180, rx = F.r_in * Math.cos(ra), ry = F.r_in * Math.sin(ra);
    el(g, "line", { class: "dim", x1: X(0), y1: Y(0), x2: X(rx), y2: Y(ry) });
    arrowHead(g, X(rx), Y(ry), Math.cos(ra), -Math.sin(ra), s);
    const rtx = X(rx * 0.64), rty = Y(ry * 0.64) - 1.2;
    el(g, "text", { x: rtx, y: rty, "font-size": fs * 0.85, "text-anchor": "middle", class: "halo",
                    transform: `rotate(${-ra * 180 / Math.PI} ${rtx} ${rty})` }, "R " + fmt(F.r_in, 1) + " on O · skirt arc");
    // overall
    dimH(g, X(-F.xo), X(F.xo), Y(F.hy), Y(F.hy) - 20, fmt(2 * F.xo, 1), fs, s);
    dimV(g, Y(F.hy), Y(-F.hy), X(F.xo), X(F.xo) + 22, fmt(2 * F.hy, 0), fs, s);
    // notes
    const ny = Y(-F.hy) + ext + 20, nf = fs * 0.82;
    [`NOTES  1. Plate ${n1(F.plate_t)} thick (z 0 to ${n1(F.plate_t)}). U-slot through plate and skirt.`,
     `2. Pockets ${n1(F.pocket_d)} deep (z ${n1(F.plate_t - F.pocket_d)} to ${n1(F.plate_t)}), centred on the ordinates, size inside each; turn = the holder's, from your valve CAD.`,
     `3. Dashed: the skirts' inner faces, ${n1(F.skirt_d)} below the underside; arc R ${n1(F.r_in)} for |y| ≤ ${fmt(F.y_kink, 1)}, flats beyond. No fillets.`]
      .forEach((t, i) => el(g, "text", { x: X(-F.xo) - 14, y: ny + i * nf * 1.45, "font-size": nf }, t));
  })();

  /* ================= frame: drawing 4, the skirts ================= */
  (function frameSkirts() {
    const m = { l: 22, r: 34, t: 30, b: 32 };
    const { svg, X, Y } = frameCanvas("f-skirt", m);
    const fs = 3.3, s = 0.95;
    hatch(svg, "f-skh", 2.6);
    const g = el(svg, "g", {});
    el(g, "polygon", { class: "hidden", points: pts(plate, X, Y) });
    el(g, "circle", { class: "ghost", cx: X(0), cy: Y(0), r: F.r_in });
    skirts.forEach(poly => {
      el(g, "polygon", { class: "pla", points: pts(poly, X, Y) });
      el(g, "polygon", { points: pts(poly, X, Y), fill: "url(#f-skh)" });
      el(g, "polygon", { class: "outline", points: pts(poly, X, Y) });
    });
    centreLines(g, X, Y, 4);
    el(g, "text", { x: X(0) + 1.6, y: Y(0) - 1.8, "font-size": fs, class: "lbl" }, "O");
    // the radius
    const a = 32 * Math.PI / 180, px = F.r_in * Math.cos(a), py = F.r_in * Math.sin(a);
    el(g, "line", { class: "dim", x1: X(0), y1: Y(0), x2: X(px), y2: Y(py) });
    arrowHead(g, X(px), Y(py), Math.cos(a), -Math.sin(a), s);
    const tx = X(px * 0.52), ty = Y(py * 0.52) - 1.3;
    el(g, "text", { x: tx, y: ty, "font-size": fs, "text-anchor": "middle", transform: `rotate(${-a * 180 / Math.PI} ${tx} ${ty})` }, "R " + fmt(F.r_in, 1) + " on O");
    // labels
    [[84, -94], [-84, 94]].forEach(([x, y]) => {
      el(g, "text", { x: X(x), y: Y(y) + fs * 0.35, "font-size": fs * 0.85, "text-anchor": "middle", class: "lbl" }, "SKIRT");
    });
    const callout = (tx, ty, px, py, text) => {
      el(g, "line", { class: "dim", x1: X(tx) + 1, y1: Y(ty), x2: X(px), y2: Y(py) });
      el(g, "circle", { class: "arrow", cx: X(px), cy: Y(py), r: 0.7 });
      el(g, "text", { x: X(tx), y: Y(ty), "font-size": fs * 0.85, "text-anchor": "end", "dominant-baseline": "middle", class: "muted" }, text);
    };
    const aa = -40 * Math.PI / 180;
    callout(54, -30, F.r_in * Math.cos(aa), F.r_in * Math.sin(aa), "arc: on the hat's rim");
    callout(54, 62, F.xf, 0.5 * (F.y_kink + F.hy), "flat: on the mailbox face");
    el(g, "text", { x: X(-52), y: Y(0) - 2, "font-size": fs * 0.85, "text-anchor": "middle", class: "muted" }, "slot cuts the skirt too");
    // dimensions
    const top = Y(F.hy), bot = Y(-F.hy);
    dimH(g, X(-F.xf), X(F.xf), top, top - 9, fmt(2 * F.xf, 1) + " between the flats", fs, s);
    dimH(g, X(-F.xo), X(F.xo), top, top - 19, fmt(2 * F.xo, 1), fs, s);
    dimH(g, X(0), X(F.xf), bot, bot + 8, fmt(F.xf, 1), fs, s);
    dimH(g, X(0), X(F.xo), bot, bot + 17, fmt(F.xo, 1), fs, s);
    dimH(g, X(F.r_in), X(F.xo), Y(0), Y(0) - 7, fmt(F.xo - F.r_in, 0), fs, s);
    ordY(g, Y(F.y_kink), X(F.xf), X(F.xo) + 7, "(" + sgn(F.y_kink, 1) + ")", fs * 0.9);
    ordY(g, Y(-F.y_kink), X(F.xf), X(F.xo) + 7, "(" + sgn(-F.y_kink, 1) + ")", fs * 0.9);
    dimV(g, Y(F.hy), Y(-F.hy), X(F.xo), X(F.xo) + 24, fmt(2 * F.hy, 0), fs, s);
    dimV(g, Y(F.slot_hy), Y(-F.slot_hy), X(-F.xo), X(-F.xo) - 8, fmt(2 * F.slot_hy, 1), fs, s);
  })();

  /* ================= frame: drawing 7, the slot as a jig ================= */
  (function frameTop() {
    const svg = $("f-top");
    const xmin = F.slot_stop - F.env[0] - 6, xmax = F.xo;
    const m = { l: 8, r: 12, t: 22, b: 22 };
    svg.setAttribute("viewBox", `0 0 ${xmax - xmin + m.l + m.r} ${2 * F.hy + m.t + m.b}`);
    const X = x => m.l + (x - xmin), Y = y => m.t + (F.hy - y);
    const fs = 3.6, s = 1.0;
    hatch(svg, "f-hatch", 2.6);
    const g = el(svg, "g", {});
    el(g, "circle", { class: "ghost", cx: X(0), cy: Y(0), r: F.hat_r });
    const ex0 = F.slot_stop - F.env[0];
    el(g, "rect", { class: "envghost", x: X(ex0), y: Y(F.env[1] / 2), width: F.env[0], height: F.env[1] });
    el(g, "polygon", { class: "pla", points: pts(plate, X, Y) });
    skirts.forEach(poly => el(g, "polygon", { points: pts(poly, X, Y), fill: "url(#f-hatch)" }));
    [right, leftUp, leftDn].forEach(line => el(g, "polyline", { class: "hidden", points: pts(line, X, Y) }));
    F.holders.forEach(h => {
      const [x0, x1, y0, y1] = h.box;
      el(g, "rect", { class: "post", x: X(x0), y: Y(y1), width: x1 - x0, height: y1 - y0 });
      el(g, "circle", { class: "ghost", cx: X(h.ball[0]), cy: Y(h.ball[1]), r: 15 });
    });
    el(g, "polygon", { class: "outline", points: pts(plate, X, Y) });
    el(g, "circle", { class: "pivot", cx: X(0), cy: Y(0), r: 2.6 });
    el(g, "line", { class: "pivot", x1: X(0) - 5, y1: Y(0), x2: X(0) + 5, y2: Y(0) });
    el(g, "line", { class: "pivot", x1: X(0), y1: Y(0) - 5, x2: X(0), y2: Y(0) + 5 });
    el(g, "text", { x: X(0) - 2, y: Y(0) - 4, "font-size": fs, "text-anchor": "end", class: "lbl" }, "O");
    el(g, "line", { class: "axis", x1: X(5), y1: Y(0), x2: X(58), y2: Y(0) });
    arrowHead(g, X(58), Y(0), 1, 0, 1.4);
    el(g, "text", { x: X(34), y: Y(0) - 2.5, "font-size": fs * 0.85, "text-anchor": "middle", class: "lbl redt" }, "+x · DRONE TRAVEL");
    el(g, "text", { x: X(ex0 + 30), y: Y(0) + fs * 0.35, "font-size": fs * 0.95, "text-anchor": "middle", class: "lbl redt" }, "ENVELOPE");
    el(g, "text", { x: X(-F.hat_r * 0.72), y: Y(-F.hat_r * 0.70) + 5, "font-size": fs * 0.8, "text-anchor": "middle", class: "muted" }, "hat rim Ø" + fmt(2 * F.hat_r, 1));
    dimH(g, X(F.slot_stop), X(0), Y(-F.slot_hy), Y(-F.hy) + 9, "stop " + fmt(-F.slot_stop, 1) + " from O", fs, s);
    dimH(g, X(ex0), X(F.slot_stop), Y(F.env[1] / 2), Y(F.env[1] / 2) - 10, fmt(F.env[0], 0), fs, s);
    dimH(g, X(ex0), X(-F.hat_r), Y(-F.env[1] / 2), Y(-F.env[1] / 2) + 10, fmt(F.overhang, 0) + " past the rim", fs, s);
  })();

  /* ================= frame: drawings 5 and 6, the fit ================= */
  function frameSection(id, k, xa, xb, za, zb, ghost, flat, fs) {
    const svg = $(id);
    const m = { l: 22, r: fs * 4.8, t: 16, b: 16 };
    svg.setAttribute("viewBox", `0 0 ${(xb - xa) * k + m.l + m.r} ${(zb - za) * k + m.t + m.b}`);
    const X = x => m.l + (x - xa) * k, Z = z => m.t + (zb - z) * k;
    const s = fs * 0.28;
    hatch(svg, id + "-h", 4);
    const g = el(svg, "g", {});
    el(g, "rect", { class: "ghostfill", x: X(xa), y: Z(0), width: (ghost.x - xa) * k, height: ghost.d * k });
    el(g, "text", { x: X(xa) + 4, y: Z(-ghost.d) - 4, "font-size": fs * 0.85, class: "lbl muted" }, ghost.label);
    const xin = flat ? F.xf : F.r_in;
    const prof = [[xa, 0], [xin, 0], [xin, -F.skirt_d], [F.xo, -F.skirt_d], [F.xo, F.plate_t], [xa, F.plate_t]];
    const P = pts(prof, X, Z);
    el(g, "polygon", { class: "pla", points: P });
    el(g, "polygon", { points: P, fill: `url(#${id}-h)` });
    el(g, "polygon", { class: "outline", points: P });
    el(g, "line", { class: "dim", x1: X(xa), y1: Z(F.plate_t) - 3, x2: X(xa), y2: Z(0) + 3, "stroke-dasharray": "2 1.5" });
    el(g, "text", { x: X((xa + xin) / 2), y: Z(F.plate_t / 2) + fs * 0.35, "font-size": fs * 0.85, "text-anchor": "middle", class: "lbl" }, "FRAME");
    dimV(g, Z(F.plate_t), Z(0), X(F.xo), X(F.xo) + fs * 1.3, fmt(F.plate_t, 0), fs, s);
    dimV(g, Z(0), Z(-F.skirt_d), X(F.xo), X(F.xo) + fs * 3.2, fmt(F.skirt_d, 0), fs, s);
    if (ghost.dim) dimV(g, Z(0), Z(-ghost.d), X(xa), X(xa) - 8, fmt(ghost.d, 0), fs, s);
    el(g, "line", { class: "dim", x1: X((ghost.x + xin) / 2), y1: Z(-F.skirt_d / 2), x2: X((ghost.x + xin) / 2) - 10, y2: Z(-F.skirt_d) + 14 });
    el(g, "text", { x: X((ghost.x + xin) / 2) - 11, y: Z(-F.skirt_d) + 18, "font-size": fs * 0.85, "text-anchor": "end", class: "redt" }, fmt(xin - ghost.x, 1) + " clearance");
    el(g, "line", { class: "redline", x1: X(xin), y1: Z(0), x2: X(xin), y2: Z(-F.skirt_d) });
    el(g, "text", { x: X(xin) + 1, y: Z(-F.skirt_d) + fs + 2, "font-size": fs * 0.8, class: "redt" }, flat ? "flat face, x = " + fmt(F.xf, 1) : "arc R " + fmt(F.r_in, 1));
  }
  frameSection("f-sec-hat", 6, 84, F.xo + 3, -7.5, F.plate_t + 2.5, { x: F.hat_r, d: F.hat_t, dim: true, label: "HAT PLATFORM Ø" + fmt(2 * F.hat_r, 1) }, false, 5.0);
  frameSection("f-sec-box", 4, 48, F.xo + 3, -16, F.plate_t + 4, { x: F.box_d / 2, d: 12, label: "MAILBOX (BACK FACE)" }, true, 9.0);

  /* ================= frame: drawing 8, a holder pocket ================= */
  (function pocketDetail() {
    const svg = $("f-post");
    const h0 = F.holders[0], w = h0.pocket[0], bw = h0.base[0];   // a section across its long side
    const k = 4, xs = w / 2 + 9, H = F.plate_t + 7;
    const m = { l: 24, r: 62, t: 22, b: 10 };
    svg.setAttribute("viewBox", `0 0 ${2 * xs * k + m.l + m.r} ${(H + 1) * k + m.t + m.b}`);
    const X = x => m.l + (x + xs) * k, Z = z => m.t + (H - z) * k;
    const fs = 6.0, s = 1.7;
    hatch(svg, "f-posth", 3.4);
    const g = el(svg, "g", {});
    const zf = F.plate_t - F.pocket_d, hw = w / 2;
    const prof = [[-xs, 0], [xs, 0], [xs, F.plate_t], [hw, F.plate_t], [hw, zf], [-hw, zf], [-hw, F.plate_t], [-xs, F.plate_t]];
    const P = pts(prof, X, Z);
    el(g, "polygon", { class: "pla", points: P });
    el(g, "polygon", { points: P, fill: "url(#f-posth)" });
    el(g, "polygon", { class: "outline", points: P });
    [-xs, xs].forEach(x => el(g, "line", { class: "cut", x1: X(x), y1: Z(0) + 2, x2: X(x), y2: Z(F.plate_t) - 2, "stroke-dasharray": "3 2" }));
    // the holder's base (ghost) and the start of its tower
    el(g, "rect", { class: "ghostfill", x: X(-bw / 2), y: Z(F.plate_t), width: bw * k, height: F.pocket_d * k });
    el(g, "rect", { class: "ghostfill", x: X(-3.5), y: Z(H), width: 7 * k, height: (H - F.plate_t) * k });
    el(g, "text", { x: X(0), y: Z(H) - 3, "font-size": fs * 0.75, "text-anchor": "middle", class: "lbl muted" }, "HOLDER (glued)");
    el(g, "line", { class: "cl", x1: X(0), y1: Z(H) - 1, x2: X(0), y2: Z(0) + 4 });
    el(g, "text", { x: X(-xs) + 2, y: Z(F.plate_t / 2) + fs * 0.35, "font-size": fs * 0.75, class: "lbl muted" }, "PLATE");
    dimH(g, X(-hw), X(hw), Z(zf), Z(0) + 6, "base + " + fmt(2 * F.pocket_clr, 1), fs, s, Z(zf));
    dimV(g, Z(F.plate_t), Z(zf), X(hw), X(hw) + 8, fmt(F.pocket_d, 0), fs, s);
    dimV(g, Z(F.plate_t), Z(0), X(-xs), X(-xs) - 8, fmt(F.plate_t, 0), fs, s);
    el(g, "text", { x: X(hw) + 2, y: Z(F.plate_t) - 4, "font-size": fs * 0.75, class: "muted" }, fmt(F.pocket_clr, 1) + " a side");
  })();

  /* ================= 3-D views ================= */
  const themeHooks = [];
  function stlGeometry(id) {
    const buf = b64bytes(id), dv = new DataView(buf.buffer);
    const n = dv.getUint32(80, true);
    const pos = new Float32Array(n * 9), nor = new Float32Array(n * 9);
    for (let t = 0; t < n; t++) {
      const o = 84 + t * 50;
      const nn = [dv.getFloat32(o, true), dv.getFloat32(o + 4, true), dv.getFloat32(o + 8, true)];
      for (let v = 0; v < 3; v++) for (let c = 0; c < 3; c++) {
        pos[t * 9 + v * 3 + c] = dv.getFloat32(o + 12 + v * 12 + c * 4, true);
        nor[t * 9 + v * 3 + c] = nn[c];
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    geo.setAttribute("normal", new THREE.BufferAttribute(nor, 3));
    return geo;
  }
  function makeViewer(hostId, controlsId, resetId, home, target, opts) {
    const host = $(hostId);
    let renderer;
    try {
      if (!window.THREE) throw new Error("three.js did not load");
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch (e) {
      host.querySelector(".fallback").hidden = false;
      if (controlsId) $(controlsId).hidden = true;
      return null;
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    host.insertBefore(renderer.domElement, host.firstChild);
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(32, 2, (opts && opts.near) || 1, (opts && opts.far) || 5000);
    camera.up.set(0, 0, 1);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 0.75));
    const sun = new THREE.DirectionalLight(0xffffff, 0.75);
    sun.position.set(80, -160, 260);
    scene.add(sun);
    const view = Object.assign({}, home);
    let tags = [];
    const V = { scene, host };
    function place() {
      const c = Math.cos(view.el);
      camera.position.set(target.x + view.r * c * Math.cos(view.az), target.y + view.r * c * Math.sin(view.az), target.z + view.r * Math.sin(view.el));
      camera.lookAt(target);
    }
    V.render = function () {
      place();
      renderer.render(scene, camera);
      const w = host.clientWidth, h = host.clientHeight;
      tags.forEach(t => {
        const v = t.p.clone().project(camera);
        t.el.style.left = ((v.x + 1) / 2 * w) + "px";
        t.el.style.top = ((1 - v.y) / 2 * h) + "px";
        t.el.hidden = v.z > 1;
      });
    };
    V.setTags = function (list) {
      tags.forEach(t => t.el.remove());
      tags = list.map(t => {
        const d = document.createElement("div");
        d.className = "tag" + (t.cls ? " " + t.cls : "");
        if (t.sw) { const sw = document.createElement("span"); sw.className = "sw " + t.sw; d.appendChild(sw); }
        d.appendChild(document.createTextNode(t.text));
        host.appendChild(d);
        return { p: new THREE.Vector3(t.p[0], t.p[1], t.p[2]), el: d };
      });
      V.render();
    };
    function resize() {
      const w = host.clientWidth, h = host.clientHeight;
      if (!w || !h) return;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      V.render();
    }
    const ptr = new Map();
    let pinch0 = 0, r0 = 0;
    host.addEventListener("pointerdown", e => { host.setPointerCapture(e.pointerId); ptr.set(e.pointerId, [e.clientX, e.clientY]);
      if (ptr.size === 2) { const [a, b] = [...ptr.values()]; pinch0 = Math.hypot(a[0] - b[0], a[1] - b[1]); r0 = view.r; } });
    host.addEventListener("pointermove", e => {
      if (!ptr.has(e.pointerId)) return;
      const prev = ptr.get(e.pointerId);
      ptr.set(e.pointerId, [e.clientX, e.clientY]);
      if (ptr.size === 1) {
        view.az -= (e.clientX - prev[0]) * 0.008;
        view.el = Math.max(-1.45, Math.min(1.45, view.el + (e.clientY - prev[1]) * 0.008));
      } else if (ptr.size === 2 && pinch0) {
        const [a, b] = [...ptr.values()];
        view.r = Math.max(home.r * 0.35, Math.min(home.r * 2.5, r0 * pinch0 / Math.hypot(a[0] - b[0], a[1] - b[1])));
      }
      V.render();
    });
    const up = e => { ptr.delete(e.pointerId); if (ptr.size < 2) pinch0 = 0; };
    host.addEventListener("pointerup", up);
    host.addEventListener("pointercancel", up);
    host.addEventListener("wheel", e => { e.preventDefault(); view.r = Math.max(home.r * 0.35, Math.min(home.r * 2.5, view.r * Math.exp(e.deltaY * 0.001))); V.render(); }, { passive: false });
    if (resetId) $(resetId).addEventListener("click", () => { Object.assign(view, home); V.render(); });
    V.aim = function (h, t) { Object.assign(home, h); target.copy(t); Object.assign(view, home); V.render(); };
    new ResizeObserver(resize).observe(host);
    V.resize = resize;
    return V;
  }
  function mat(opts) { return new THREE.MeshStandardMaterial(Object.assign({ roughness: 0.65, metalness: 0.0 }, opts)); }
  // the marker holders: one material per design (its colour token), the standard marker drawn as a
  // 30 mm ball on its tab; a holder's own frame = the valve CAD's axes, origin at its base's bottom centre
  const HGEO = {};
  function holderMats() {
    const M = { ball: mat({ roughness: 0.35 }), tab: mat({ roughness: 0.7 }) };
    "ABCD".split("").forEach(t => { M[t] = mat({ roughness: 0.6 }); });
    M.theme = () => { "ABCD".split("").forEach(t => M[t].color.set(css("--h" + t))); M.ball.color.set("#f2f2ee"); M.tab.color.set(css("--steel3d")); };
    return M;
  }
  function holderGroup(t, M) {
    if (!HGEO[t]) HGEO[t] = stlGeometry("holder-" + t + "-stl");
    const grp = new THREE.Group();
    grp.add(new THREE.Mesh(HGEO[t], M[t]));
    const d = F.designs[t], tb = F.tabs[t];
    const ball = new THREE.Mesh(new THREE.SphereGeometry(d.ball_r, 32, 20), M.ball);
    ball.position.set(d.ball[0], d.ball[1], d.ball[2]);
    const tab = new THREE.Mesh(new THREE.BoxGeometry(tb.hi[0] - tb.lo[0], tb.hi[1] - tb.lo[1], tb.hi[2] - tb.lo[2]), M.tab);
    tab.position.set((tb.lo[0] + tb.hi[0]) / 2, (tb.lo[1] + tb.hi[1]) / 2, (tb.lo[2] + tb.hi[2]) / 2);
    grp.add(ball, tab);
    return grp;
  }

  // --- the envelope core
  (function coreViewer() {
    const V = makeViewer("c-viewer", "c-controls", "c-reset", { az: -2.25, el: 0.62, r: 285 }, new THREE.Vector3(L / 2, W / 2, T / 2));
    if (!V) return;
    const geo = stlGeometry("core-stl");
    const plaM = mat({}), edgeM = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.35 });
    V.scene.add(new THREE.Mesh(geo, plaM));
    V.scene.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo, 30), edgeM));
    const paperM = mat({ transparent: true, opacity: 0.28, depthWrite: false, roughness: 0.9 });
    const paper = new THREE.Mesh(new THREE.BoxGeometry(L + 0.6, W + 0.6, T + 0.4), paperM);
    paper.position.set(L / 2, W / 2, T / 2);
    paper.visible = false;
    V.scene.add(paper);
    themeHooks.push(() => {
      plaM.color.set(css("--pla3d")); paperM.color.set(css("--red"));
      edgeM.color.set(css("--ink")); V.scene.background = new THREE.Color(css("--bg3d")); V.render();
    });
    V.setTags([{ p: [0, W / 2, T + 2], text: "near edge · jaws" }, { p: [L, W / 2, T + 2], text: "far end", cls: "far" }]);
    $("c-paper").addEventListener("change", e => { paper.visible = e.target.checked; V.render(); });
  })();

  // --- the localization frame
  (function frameViewer() {
    const V = makeViewer("f-viewer", "f-controls", "f-reset", { az: -2.15, el: 0.55, r: 520 }, new THREE.Vector3(-35, 0, 0));
    if (!V) return;
    const geo = stlGeometry("frame-stl");
    const plaM = mat({}), edgeM = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.3 });
    V.scene.add(new THREE.Mesh(geo, plaM));
    V.scene.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo, 30), edgeM));
    const ghostM = mat({ transparent: true, opacity: 0.45, depthWrite: false, roughness: 0.9 });
    const envM = mat({ transparent: true, opacity: 0.7, roughness: 0.8 });
    const darkM = mat({ roughness: 0.9 });
    const dotM = mat({ roughness: 0.4 }), clawM = mat({ roughness: 0.4 });
    // the hat: platform + sleeve
    const hat = new THREE.Group();
    const plat = new THREE.Mesh(new THREE.CylinderGeometry(F.hat_r, F.hat_r, F.hat_t, 96), ghostM);
    plat.rotation.x = Math.PI / 2; plat.position.z = -F.hat_t / 2;
    const sleeve = new THREE.Mesh(new THREE.CylinderGeometry(61, 61, 52, 64), ghostM);
    sleeve.rotation.x = Math.PI / 2; sleeve.position.z = -F.hat_t - 26;
    hat.add(plat, sleeve);
    V.scene.add(hat);
    const envHat = new THREE.Mesh(new THREE.BoxGeometry(F.env[0], F.env[1], F.env[2]), envM);
    envHat.position.set(F.offsets.pick[0], 0, F.offsets.pick[2]);
    V.scene.add(envHat);
    // the mailbox: the top 140 mm, with its mouth
    const box = new THREE.Group();
    const bx = new THREE.Mesh(new THREE.BoxGeometry(F.box_d, F.box_w, 140), ghostM);
    bx.position.z = -70;
    const mouth = new THREE.Mesh(new THREE.BoxGeometry(1.2, 190, 90), darkM);
    mouth.position.set(-F.box_d / 2 - 0.4, 0, -70);
    box.add(bx, mouth);
    V.scene.add(box);
    const envBox = new THREE.Mesh(new THREE.BoxGeometry(F.env[0], F.env[1], F.env[2]), envM);
    envBox.position.set(F.offsets.place[0], 0, F.offsets.place[2]);
    V.scene.add(envBox);
    // the five holders (their STLs) and markers (a ball on its tab), in their pockets
    const HM = holderMats(), z0 = F.plate_t - F.pocket_d;
    F.holders.forEach(h => {
      const grp = holderGroup(h.t, HM);
      grp.rotation.z = h.heading * Math.PI / 180;
      grp.position.set(h.x, h.y, z0);
      V.scene.add(grp);
    });
    const piv = new THREE.Mesh(new THREE.TorusGeometry(6, 0.9, 10, 36), mat({ roughness: 0.6 }));
    piv.position.set(CEN[0], CEN[1], CEN[2]);
    V.scene.add(piv);
    const dot = new THREE.Mesh(new THREE.SphereGeometry(4, 20, 14), dotM);
    const claw = new THREE.Mesh(new THREE.SphereGeometry(3.2, 20, 14), clawM);
    V.scene.add(dot, claw);
    themeHooks.push(() => {
      plaM.color.set(css("--pla3d")); edgeM.color.set(css("--ink")); ghostM.color.set(css("--ghost3d"));
      HM.theme(); piv.material.color.set(css("--muted"));
      envM.color.set(css("--red")); dotM.color.set(css("--red")); clawM.color.set(css("--ink")); darkM.color.set(css("--ink"));
      V.scene.background = new THREE.Color(css("--bg3d")); V.render();
    });
    function setMode(mode) {
      hat.visible = envHat.visible = mode === "hat";
      box.visible = envBox.visible = mode === "box";
      dot.visible = claw.visible = mode !== "alone";
      const o = mode === "box" ? F.offsets.place : F.offsets.pick, c = mode === "box" ? F.offsets.place_claw : F.offsets.pick_claw;
      dot.position.set(o[0], o[1], o[2]); claw.position.set(c[0], c[1], c[2]);
      if (mode === "alone") V.setTags([{ p: [F.slot_stop - 40, 0, 6], text: "U-slot · open end" }, { p: [60, 0, 6], text: "+x · drone travel", cls: "far" },
                                       { p: [CEN[0], CEN[1], CEN[2] + 8], text: "obj_0 pivot" }]);
      else if (mode === "hat") V.setTags([{ p: [o[0], 0, o[2] + 6], text: "pick point", cls: "far" }, { p: [c[0], 0, c[2] + 5], text: "claw" }]);
      else V.setTags([{ p: [o[0], 0, o[2] + 6], text: "place point", cls: "far" }, { p: [c[0], 0, c[2] + 5], text: "claw" }, { p: [-F.box_d / 2, -95, -25], text: "mouth" }]);
    }
    document.querySelectorAll('input[name="f-mode"]').forEach(r => r.addEventListener("change", e => { if (e.target.checked) setMode(e.target.value); }));
    setMode("hat");
  })();

  // --- the marker holders: on the valve (the user's CAD) and on the frame, the same colours
  (function holderViewer() {
    const VH = { az: -1.95, el: 0.38, r: 560 }, VT = new THREE.Vector3(85, 0, 70);
    const V = makeViewer("h-viewer", "h-controls", "h-reset", VH, VT);
    if (!V) return;
    const HM = holderMats();
    const ctxM = mat({ roughness: 0.75 }), plaM = mat({}), edgeM = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.25 });
    // on the valve: its plate, bearing, pad rings and rotor, each holder where it sits there (unturned)
    const valve = new THREE.Group();
    const vgeo = stlGeometry("valve-context-stl");
    valve.add(new THREE.Mesh(vgeo, ctxM), new THREE.LineSegments(new THREE.EdgesGeometry(vgeo, 35), edgeM));
    const vtags = [];
    VALVE.holders.forEach(h => {
      const grp = holderGroup(h.t, HM);
      grp.position.set(h.base_centre[0], h.base_centre[1], h.base_centre[2]);
      valve.add(grp);
      const d = F.designs[h.t], b = h.base_centre;
      vtags.push({ p: [b[0] + d.ball[0], b[1] + d.ball[1], b[2] + d.ball[2] + d.ball_r + 6], text: h.t, sw: h.t });
    });
    VALVE.markers.filter(m => !m.holder).forEach(m => {
      const ball = new THREE.Mesh(new THREE.SphereGeometry(15, 32, 20), HM.ball);
      ball.position.set(m.ball[0], m.ball[1], m.ball[2]);
      valve.add(ball);
      vtags.push({ p: [m.ball[0], m.ball[1], m.ball[2] - 24], text: "5th: rotor's own mount" });
    });
    V.scene.add(valve);
    // on the frame
    const frame = new THREE.Group();
    const fgeo = stlGeometry("frame-stl");
    frame.add(new THREE.Mesh(fgeo, plaM), new THREE.LineSegments(new THREE.EdgesGeometry(fgeo, 30), edgeM));
    const ftags = [], z0 = F.plate_t - F.pocket_d;
    F.holders.forEach((h, i) => {
      const grp = holderGroup(h.t, HM);
      grp.rotation.z = h.heading * Math.PI / 180;
      grp.position.set(h.x, h.y, z0);
      frame.add(grp);
      ftags.push({ p: [h.ball[0], h.ball[1], h.ball[2] + F.designs[h.t].ball_r + 6], text: HNAME[i], sw: h.t });
    });
    ftags.push({ p: [F.xo - 8, 0, F.plate_t + 3], text: "+x", cls: "far" });
    V.scene.add(frame);
    themeHooks.push(() => {
      HM.theme(); ctxM.color.set(css("--ghost3d")); plaM.color.set(css("--pla3d")); edgeM.color.set(css("--ink"));
      V.scene.background = new THREE.Color(css("--bg3d")); V.render();
    });
    function setMode(m) {
      valve.visible = m === "valve"; frame.visible = m === "frame";
      if (m === "valve") V.aim(VH, VT);
      else V.aim({ az: -2.15, el: 0.62, r: 480 }, new THREE.Vector3(-5, 0, 30));
      V.setTags(m === "valve" ? vtags : ftags);
    }
    document.querySelectorAll('input[name="h-mode"]').forEach(r => r.addEventListener("change", e => { if (e.target.checked) setMode(e.target.value); }));
    setMode("valve");
  })();

  // --- one card per design: the holder alone, in its own (= the valve CAD's) axes
  (function holderCards() {
    const host = $("h-cards");
    const c3 = (v) => v.map(x => fmt(Math.abs(x) < 0.05 ? 0 : x, 1)).join(", ");
    Object.keys(F.designs).sort().forEach(t => {
      const d = F.designs[t], onValve = VALVE.holders.filter(h => h.t === t);
      const onFrame = F.holders.map((h, i) => [h, i]).filter(([h]) => h.t === t);
      const card = document.createElement("article");
      card.className = "hcard";
      const h3 = document.createElement("h3");
      const sw = document.createElement("span"); sw.className = "sw " + t;
      h3.append(sw, document.createTextNode("Holder " + t));
      const vid = "h-card-" + t;
      const vw = document.createElement("div");
      vw.className = "viewer"; vw.id = vid; vw.setAttribute("role", "img");
      vw.setAttribute("aria-label", "3-D view of holder design " + t + " with its marker");
      const fb = document.createElement("div"); fb.className = "fallback"; fb.hidden = true; fb.textContent = "Needs WebGL";
      vw.appendChild(fb);
      const dl = document.createElement("dl");
      const add = (k, v) => { const dt = document.createElement("dt"), dd = document.createElement("dd"); dt.textContent = k; dd.textContent = v; dl.append(dt, dd); };
      add("Print", "× " + onFrame.length);
      add("Base", fmt(d.base_size[0], 1) + " × " + fmt(d.base_size[1], 1) + " × 2");
      add("Tower", fmt(d.holder_height, 1) + " tall");
      add("Marker", "(" + c3(d.ball) + ")");
      onValve.forEach(h => add("On the valve", "(" + c3(h.base_centre) + ")"));
      onFrame.forEach(([h, i]) => add(HNAME[i] + " on frame", "(" + sgn(h.x, 0) + ", " + sgn(h.y, 0) + "), " + h.heading + "°"));
      card.append(h3, vw, dl);
      host.appendChild(card);
      // the 3-D view
      const geo = stlGeometry("holder-" + t + "-stl");
      geo.computeBoundingBox();
      const bb = geo.boundingBox.clone();
      bb.expandByPoint(new THREE.Vector3(d.ball[0] - d.ball_r, d.ball[1] - d.ball_r, d.ball[2] - d.ball_r));
      bb.expandByPoint(new THREE.Vector3(d.ball[0] + d.ball_r, d.ball[1] + d.ball_r, d.ball[2] + d.ball_r));
      const ctr = bb.getCenter(new THREE.Vector3()), size = bb.getSize(new THREE.Vector3());
      const R = Math.max(size.x, size.y, size.z) * 2.55;
      const V = makeViewer(vid, null, null, { az: -2.2, el: 0.42, r: R }, ctr);
      if (!V) return;
      const M = holderMats();
      V.scene.add(holderGroup(t, M));
      const L = Math.max(size.x, size.y) * 0.42 + 12;
      const axM = new THREE.LineBasicMaterial({});
      const ax = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(L, 0, 0), new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, L, 0)]);
      V.scene.add(new THREE.LineSegments(ax, axM));
      themeHooks.push(() => { M.theme(); axM.color.set(css("--ink")); V.scene.background = new THREE.Color(css("--bg3d")); V.render(); });
      V.setTags([{ p: [L, 0, 0], text: "+x" }, { p: [0, L, 0], text: "+y" }]);
    });
    const p = document.createElement("p");
    p.className = "small"; p.style.gridColumn = "1 / -1";
    p.textContent = "In each card: the holder in your valve CAD's axes (the +x and +y lines, origin at the centre of the base's bottom face), with its standard marker. Marker: the ball's centre from that point. On the valve: where that point is in your valve CAD. On the frame: the pocket centre from O, and how far the holder is turned about z (counter-clockwise from above). Drag a card to turn it.";
    host.appendChild(p);
  })();

  /* ================= drawing 9: the red mailbox's front face and the mouth to cut ================= */
  (function mailboxFront() {
    const D = JSON.parse($("field-info").textContent), B = D.box, T = D.table;
    const bw = B.w, bh = B.h, mw = B.mouth[0], mh = B.mouth[1], mz = B.mouth[2];   // mouth[2]: its centre above the box bottom
    const mx0 = (bw - mw) / 2, mx1 = (bw + mw) / 2, mz0 = mz - mh / 2, mz1 = mz + mh / 2;
    const top = bh - mz1;                         // the mouth's top edge under the box top
    const zr = bh + F.offsets.place[2];           // the release point (and the claw) above the box bottom
    // the envelope's pass band at the wall plane (tools/mail_geometry.py): about the claw -48..+6 mm for
    // 0-20 deg nose-down in the jaws, +-12 mm vertical and +-40 mm lateral hover error, 5 mm for 2 deg of yaw
    const bz0 = zr - 48 - 12, bz1 = zr + 6 + 12, bhw = F.env[1] / 2 + 40 + 5;
    const DM = { top: n1(top) + " mm", lip: n1(top + mh) + " mm", half: n1(bw / 2) + " mm", side: n1(mx0) + " mm",
                 mhalf: n1(mw / 2) + " mm", size: n1(mw) + " × " + n1(mh) + " mm", rel: n1(zr - mz) };
    document.querySelectorAll("[data-m]").forEach(e => { e.textContent = DM[e.dataset.m]; });
    const tb = document.querySelector("#m-edges tbody");
    const r = (a, b, v) => row(tb, [a, b, { t: v, num: 1 }]);
    r("Top edge", "down from the box top", n1(top));
    r("Bottom edge, the lip", "down from the box top", n1(top + mh));
    r("Left and right edges", "in from each side", n1(mx0));
    r("Centre", "down from the top · in from each side", n1(top + mh / 2) + " · " + n1(bw / 2));
    r("Size", "width × height", n1(mw) + " × " + n1(mh));
    r("Lip", "above the floor, box on the " + n1(T.h) + " table", n1(T.h + mz0));
    r("Top edge", "above the floor", n1(T.h + mz1));

    const svg = $("m-front");
    const zlo = -T.top_t, m = { l: 39, r: 66, t: 26, b: 24 };
    svg.setAttribute("viewBox", `0 0 ${bw + m.l + m.r} ${bh - zlo + m.t + m.b}`);
    const X = x => m.l + x, Z = z => m.t + (bh - z);
    const fs = 6.5, s = 1.7;
    const g = el(svg, "g", {});
    // the table top under the box, its front edge flush with the face
    el(g, "rect", { class: "wood", x: X(-16), y: Z(0), width: bw + 32, height: T.top_t });
    el(g, "text", { x: X(-16), y: Z(zlo) + fs + 2, "font-size": fs * 0.8, class: "lbl muted" }, "TABLE TOP · " + n1(T.h) + " ABOVE THE FLOOR");
    // the face, the mouth, the envelope's pass band
    el(g, "rect", { class: "face", x: X(0), y: Z(bh), width: bw, height: bh });
    el(g, "rect", { class: "hole", x: X(mx0), y: Z(mz1), width: mw, height: mh });
    el(g, "rect", { class: "band", x: X(bw / 2 - bhw), y: Z(bz1), width: 2 * bhw, height: bz1 - bz0 });
    // centre lines
    el(g, "line", { class: "cl", x1: X(bw / 2), y1: Z(bh) - 4, x2: X(bw / 2), y2: Z(zlo) + 3 });
    el(g, "line", { class: "cl", x1: X(-8), y1: Z(mz), x2: X(bw + 6), y2: Z(mz) });
    el(g, "text", { x: X(bw / 2) + 1.5, y: Z(zlo) + fs + 2, "font-size": fs * 0.8, class: "muted" }, "CL");
    el(g, "text", { x: X(-8) - 1, y: Z(mz), "font-size": fs * 0.8, "text-anchor": "end", "dominant-baseline": "middle", class: "muted" }, "CL");
    // the release point, 21.5 above the mouth's centre
    el(g, "circle", { class: "com", cx: X(bw / 2), cy: Z(zr), r: 1.5 });
    el(g, "text", { x: X(bw / 2) + 3.5, y: Z(zr), "font-size": fs * 0.85, "dominant-baseline": "middle", class: "lbl redt halo" }, "RELEASE POINT");
    dimV(g, Z(zr), Z(mz), X(bw / 2), X(bw / 2) - 16, n1(zr - mz), fs * 0.8, s * 0.8);
    el(g, "text", { x: X(bw / 2), y: Z(mz) + 17, "font-size": fs, "text-anchor": "middle", class: "lbl halo" }, "CUT OUT " + n1(mw) + " × " + n1(mh));
    el(g, "text", { x: X(bw / 2), y: Z(mz) + 17 + fs * 1.1, "font-size": fs * 0.8, "text-anchor": "middle", class: "muted halo" }, "through the front wall");
    el(g, "text", { x: X(bw / 2 + bhw) - 2, y: Z(bz0) - 2, "font-size": fs * 0.75, "text-anchor": "end", class: "redt halo" }, "envelope's pass band");
    el(g, "text", { x: X(bw / 2), y: Z(bh * 0.33), "font-size": fs * 1.15, "text-anchor": "middle", class: "lbl" }, "RED MAILBOX · FRONT FACE");
    el(g, "text", { x: X(bw / 2), y: Z(bh * 0.33) + fs * 1.3, "font-size": fs * 0.8, "text-anchor": "middle", class: "muted" }, "as seen from the start");
    // overall size
    dimH(g, X(0), X(bw), Z(bh), Z(bh) - 12, n1(bw), fs, s);
    dimV(g, Z(bh), Z(0), X(0), X(-24), n1(bh), fs, s);
    // across, under the mouth: jamb | mouth | jamb
    const yc = Z(mz0) + 12;
    dimH(g, X(0), X(mx0), Z(mz0), yc, n1(mx0), fs, s);
    dimH(g, X(mx0), X(mx1), Z(mz0), yc, n1(mw), fs, s);
    dimH(g, X(mx1), X(bw), Z(mz0), yc, n1(bw - mx1), fs, s);
    // down the right: top -> mouth top -> lip -> bottom, then the heights above the floor
    const xc = X(bw) + 12, xf = X(bw) + 28;
    dimV(g, Z(bh), Z(mz1), X(mx1), xc, n1(top), fs, s);
    dimV(g, Z(mz1), Z(mz0), X(mx1), xc, n1(mh), fs, s);
    dimV(g, Z(mz0), Z(0), X(mx1), xc, n1(mz0), fs, s);
    el(g, "text", { x: xf, y: Z(bh) - 9, "font-size": fs * 0.75, class: "lbl muted" }, "ABOVE THE FLOOR");
    [bh, mz1, mz0, 0].forEach(z => {
      el(g, "line", { class: "dim", x1: xc + s * 1.2 + 0.6, y1: Z(z), x2: xf - 1.2, y2: Z(z) });
      el(g, "text", { x: xf, y: Z(z), "font-size": fs * 0.9, "dominant-baseline": "middle" }, n1(T.h + z));
    });
  })();

  /* ================= the field: where the pillar and the mailboxes go ================= */
  (function field() {
    const D = JSON.parse($("field-info").textContent);
    const P = D.pillar, T = D.table, B = D.box, WP = D.waypoints;
    const tx0 = T.front_x, tx1 = T.front_x + T.depth_x, txc = (tx0 + tx1) / 2;
    const tyL = T.y + T.len_y / 2, tyR = T.y - T.len_y / 2;          // the table's left (+y) and right ends
    const red = { y: D.red_y }, blue = { y: D.blue_y };
    [red, blue].forEach(b => { b.yL = b.y + B.w / 2; b.yR = b.y - B.w / 2; });
    const env = [P.x + F.offsets.pick[0], P.y + F.offsets.pick[1], P.hat_top + F.offsets.pick[2]];
    const ms = WP.place_start.slice(0, 3).map(v => v * 1000), rel = WP.place_point.slice(0, 3).map(v => v * 1000);
    const m2 = v => fmt(v / 1000, 2);
    const pt = a => "(" + a.slice(0, 3).map(v => fmt(v, 2)).join(", ") + ") m";

    // the key numbers
    $("d-pillar").textContent = m2(P.x) + ", " + m2(P.y);
    $("d-table").textContent = fmt(txc / 1000, 3) + ", " + m2(T.y);
    $("d-table-s").textContent = "x, y in m · front edge on x = " + m2(tx0);
    $("d-red").textContent = fmt(tyL - red.yL, 1);
    $("d-blue").textContent = fmt(tyL - blue.yL, 1);

    // the placement table
    const tb = $("d-table-rows");
    const span = (a, b) => fmt(Math.min(a, b), 1) + " … " + fmt(Math.max(a, b), 1);
    row(tb, ["Takeoff mark", "x 0, y 0", "the drone facing +x", "Anywhere within " + fmt(D.adjust_max, 1) + " m of it: Adjust on the ground moves Start, Land Start and Land to where the drone stands"]);
    row(tb, ["Pick pillar", "centre x " + fmt(P.x, 0) + ", y " + fmt(P.y, 0) + " · hat top z " + fmt(P.hat_top, 0),
             "the frame's slot open toward −x (the start), the envelope sticking out toward the drone",
             "Where convenient: the Envelope Get measures it"]);
    row(tb, ["Console table", "x " + span(tx0, tx1) + ", y " + span(tyR, tyL) + " · centre x " + fmt(txc, 1) + ", y " + fmt(T.y, 0),
             "long side along y, front edge on x = " + fmt(tx0, 0), "Keep the pillar-to-mailbox layout; the whole set may move"]);
    row(tb, ["Red mailbox (target)", "y " + span(red.yR, red.yL) + " (centre " + sgn(red.y, 1) + ") · front face on x = " + fmt(tx0, 0),
             "mouth facing the start (−x), front flush with the table's front edge",
             "Left edge " + fmt(tyL - red.yL, 1) + " mm in from the table's left end. The Mailbox Get measures it, and Mailbox Start follows"]);
    row(tb, ["Blue mailbox (scenery)", "y " + span(blue.yR, blue.yL) + " (centre " + sgn(blue.y, 1) + ") · front face on x = " + fmt(tx0, 0),
             "mouth facing the start, front flush with the table's front edge",
             "Left edge " + fmt(tyL - blue.yL, 1) + " mm in from the table's left end, right edge " + fmt(blue.yR - tyR, 1) +
             " in from its right end; " + fmt(red.yR - blue.yL, 0) + " mm gap to the red box"]);
    const so = D.place_start_offset;
    row(tb, ["Mailbox Start", pt(WP.place_start) + " as designed", "the mailbox's yaw",
             so.length === 4 ? "Follows the Mailbox Get: " + fmt(-so[0], 3) + " m in front of the release point, " + fmt(so[2], 3) + " m above it, turned with the mailbox"
                             : "Typed in the yaml; retype it if the red mailbox moves"]);
    row(tb, ["Release point", pt(WP.place_point) + " as designed", "the frame's yaw", "Written by the Mailbox Get from the frame"]);
    row(tb, ["Land Start · Land", pt(WP.land_start) + " · " + pt(WP.land) + " as designed", "", "Over the takeoff mark: Adjust moves them with Start"]);

    // the 3-D view, in mm
    const HOME = { az: -2.62, el: 0.40, r: 5600 }, TOP = { az: -Math.PI / 2, el: 1.45, r: 6200 };
    const TGT = new THREE.Vector3(950, 220, 420), TGT_TOP = new THREE.Vector3(820, 120, 0);
    const V = makeViewer("d-viewer", "d-controls", "d-reset", Object.assign({}, HOME), TGT.clone(), { near: 20, far: 40000 });
    if (V) {
      const floorM = mat({ roughness: 0.95 }), gridM = new THREE.LineBasicMaterial({});
      const pillarM = mat({}), hatM = mat({}), envM = mat({ roughness: 0.8 }), woodM = mat({ roughness: 0.8 }), legM = mat({ roughness: 0.5 });
      const redM = mat({ roughness: 0.6 }), blueM = mat({ roughness: 0.6 }), darkM = mat({ roughness: 0.9 });
      const inkM = mat({ roughness: 0.6 }), relM = mat({ roughness: 0.4 });
      const box = (sx, sy, sz, x, y, z, m) => { const o = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), m); o.position.set(x, y, z); V.scene.add(o); return o; };
      const cyl = (r, h, x, y, z, m) => { const o = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 64), m); o.rotation.x = Math.PI / 2; o.position.set(x, y, z); V.scene.add(o); return o; };
      // the floor, its 0.5 m grid, the axes and the takeoff mark
      const fx0 = -1000, fx1 = 2500, fy0 = -1500, fy1 = 1500;
      box(fx1 - fx0, fy1 - fy0, 2, (fx0 + fx1) / 2, (fy0 + fy1) / 2, -1, floorM);
      const gp = [];
      for (let x = fx0; x <= fx1; x += 500) gp.push(new THREE.Vector3(x, fy0, 1), new THREE.Vector3(x, fy1, 1));
      for (let y = fy0; y <= fy1; y += 500) gp.push(new THREE.Vector3(fx0, y, 1), new THREE.Vector3(fx1, y, 1));
      V.scene.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(gp), gridM));
      const axX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 3), 650, 0x000000, 110, 55);
      const axY = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 3), 450, 0x000000, 110, 55);
      V.scene.add(axX, axY);
      const mark = new THREE.Mesh(new THREE.RingGeometry(95, 125, 48), inkM);
      mark.position.z = 2; V.scene.add(mark);
      // the pick pillar, its hat and the envelope on it
      cyl(P.d / 2, P.h, P.x, P.y, P.h / 2, pillarM);
      cyl(F.hat_r, F.hat_t, P.x, P.y, P.hat_top - F.hat_t / 2, hatM);
      cyl(61, 52, P.x, P.y, P.hat_top - F.hat_t - 26, hatM);
      box(F.env[0], F.env[1], F.env[2], env[0], env[1], env[2], envM);
      // the console table: top, four legs, shelf
      box(T.depth_x, T.len_y, T.top_t, txc, T.y, T.h - T.top_t / 2, woodM);
      [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(([sx, sy]) =>
        box(T.leg, T.leg, T.h - T.top_t, txc + sx * (T.depth_x - T.leg) / 2, T.y + sy * (T.len_y - T.leg) / 2, (T.h - T.top_t) / 2, legM));
      box(T.depth_x, T.len_y - 2 * T.leg, T.shelf_t, txc, T.y, T.shelf_z - T.shelf_t / 2, woodM);
      // the two mailboxes, fronts flush with the table's front edge, their mouths facing the start
      [[red, redM], [blue, blueM]].forEach(([b, m]) => {
        box(B.d, B.w, B.h, tx0 + B.d / 2, b.y, T.h + B.h / 2, m);
        box(4, B.mouth[0], B.mouth[1], tx0 - 1.5, b.y, T.h + B.mouth[2], darkM);
      });
      // Mailbox Start (typed) and the release point (the Mailbox Get)
      const dMs = new THREE.Mesh(new THREE.SphereGeometry(26, 24, 16), inkM); dMs.position.set(ms[0], ms[1], ms[2]); V.scene.add(dMs);
      const dRel = new THREE.Mesh(new THREE.SphereGeometry(22, 24, 16), relM); dRel.position.set(rel[0], rel[1], rel[2]); V.scene.add(dRel);
      themeHooks.push(() => {
        floorM.color.set(css("--bay")); gridM.color.set(css("--hair"));
        axX.setColor(new THREE.Color(css("--ink"))); axY.setColor(new THREE.Color(css("--ink")));
        pillarM.color.set(css("--ghost3d")); hatM.color.set(css("--pla3d")); envM.color.set(css("--red"));
        woodM.color.set(css("--wood")); legM.color.set(css("--steel3d"));
        redM.color.set(css("--red")); blueM.color.set(css("--mailblue")); darkM.color.set(css("--ink"));
        inkM.color.set(css("--ink")); relM.color.set(css("--red"));
        V.scene.background = new THREE.Color(css("--bg3d")); V.render();
      });
      V.setTags([
        { p: [0, -60, 0], text: "start (0, 0)" },
        { p: [700, 0, 0], text: "+x" }, { p: [0, 500, 0], text: "+y · left" },
        { p: [1000, fy0, 0], text: "x 1 m" }, { p: [2000, fy0, 0], text: "x 2 m" },
        { p: [P.x, P.y, 0], text: "pick pillar (" + m2(P.x) + ", " + m2(P.y) + ")" },
        { p: [env[0] - 40, env[1], env[2] + 6], text: "envelope", cls: "far" },
        { p: [txc, T.y, T.h], text: "table centre (" + fmt(txc / 1000, 3) + ", " + m2(T.y) + ")" },
        { p: [tx1, red.y, T.h + B.h], text: "red mailbox", cls: "far" },
        { p: [tx1, blue.y, T.h + B.h], text: "blue mailbox" },
        { p: [ms[0], ms[1], ms[2] + 30], text: "Mailbox Start" },
      ]);
      document.querySelectorAll('input[name="d-mode"]').forEach(r => r.addEventListener("change", e => {
        if (!e.target.checked) return;
        if (e.target.value === "top") V.aim(Object.assign({}, TOP), TGT_TOP);
        else V.aim(Object.assign({}, HOME), TGT);
      }));
    }

    // drawing 10, the field plan: 1 drawing unit = 10 mm
    const svg = $("d-plan");
    const k = 0.1, xmin = -470, xmax = 2150, ymin = -770, ymax = 1250;
    const mg = { l: 3, r: 3, t: 3, b: 3 };
    svg.setAttribute("viewBox", `0 0 ${(xmax - xmin) * k + mg.l + mg.r} ${(ymax - ymin) * k + mg.t + mg.b}`);
    const X = x => mg.l + (x - xmin) * k, Y = y => mg.t + (ymax - y) * k;
    const fs = 3.3, s = 1.0;
    const g = el(svg, "g", {});
    for (let x = 0; x <= 2000; x += 500) {
      el(g, "line", { class: "fgrid", x1: X(x), y1: Y(-500), x2: X(x), y2: Y(1200) });
      el(g, "text", { x: X(x), y: Y(1200) - 1.2, "font-size": fs * 0.8, "text-anchor": "middle", class: "muted" }, "x " + fmt(x / 1000, 1));
    }
    for (let y = -500; y <= 1000; y += 500) {
      el(g, "line", { class: "fgrid", x1: X(-400), y1: Y(y), x2: X(2000), y2: Y(y) });
      el(g, "text", { x: X(-400) - 1, y: Y(y), "font-size": fs * 0.8, "text-anchor": "end", "dominant-baseline": "middle", class: "muted" }, "y " + fmt(y / 1000, 1));
    }
    // the takeoff mark and the axes
    el(g, "circle", { class: "pivot", cx: X(0), cy: Y(0), r: 11 * k * 10 / 10 });
    el(g, "line", { class: "axis", x1: X(0), y1: Y(0), x2: X(420), y2: Y(0) }); arrowHead(g, X(420), Y(0), 1, 0, 1.4);
    el(g, "line", { class: "axis", x1: X(0), y1: Y(0), x2: X(0), y2: Y(330) }); arrowHead(g, X(0), Y(330), 0, -1, 1.4);
    el(g, "text", { x: X(220), y: Y(0) - 1.6, "font-size": fs * 0.85, "text-anchor": "middle", class: "lbl redt" }, "+x · FORWARD");
    el(g, "text", { x: X(0) + 1.6, y: Y(300), "font-size": fs * 0.85, class: "lbl redt" }, "+y · LEFT");
    el(g, "text", { x: X(0) - 2, y: Y(0) + 5, "font-size": fs, "text-anchor": "end", class: "lbl" }, "START (0, 0)");
    // centre lines: y = 0 through the table, and the pillar's
    el(g, "line", { class: "cl", x1: X(-300), y1: Y(0), x2: X(2090), y2: Y(0) });
    el(g, "line", { class: "cl", x1: X(-330), y1: Y(P.y), x2: X(P.x + 160), y2: Y(P.y) });
    el(g, "line", { class: "cl", x1: X(P.x), y1: Y(P.y + 160), x2: X(P.x), y2: Y(P.y - 160) });
    // the pick pillar (its hat, the pillar under it dashed) and the envelope
    el(g, "circle", { class: "pla", cx: X(P.x), cy: Y(P.y), r: F.hat_r * k });
    el(g, "circle", { class: "outline", cx: X(P.x), cy: Y(P.y), r: F.hat_r * k });
    el(g, "circle", { class: "hidden", cx: X(P.x), cy: Y(P.y), r: P.d / 2 * k });
    el(g, "rect", { class: "envghost", x: X(env[0] - F.env[0] / 2), y: Y(env[1] + F.env[1] / 2), width: F.env[0] * k, height: F.env[1] * k });
    el(g, "text", { x: X(P.x + 130), y: Y(P.y + 40), "font-size": fs, class: "lbl" }, "PICK PILLAR");
    el(g, "text", { x: X(P.x + 130), y: Y(P.y + 40) + fs * 1.1, "font-size": fs * 0.9 }, "(" + fmt(P.x, 0) + ", " + fmt(P.y, 0) + ")");
    el(g, "text", { x: X(env[0]), y: Y(P.y - 120), "font-size": fs * 0.8, "text-anchor": "middle", class: "lbl redt" }, "ENVELOPE");
    // the table, the mailboxes and their mouths
    el(g, "rect", { class: "wood", x: X(tx0), y: Y(tyL), width: T.depth_x * k, height: T.len_y * k });
    [[red, "mred", "RED"], [blue, "mblue", "BLUE"]].forEach(([b, cls, name]) => {
      el(g, "rect", { class: cls, x: X(tx0), y: Y(b.yL), width: B.d * k, height: B.w * k });
      el(g, "line", { class: "mouth", x1: X(tx0) - 0.7, y1: Y(b.y + B.mouth[0] / 2), x2: X(tx0) - 0.7, y2: Y(b.y - B.mouth[0] / 2) });
      el(g, "text", { x: X(tx0 + B.d / 2), y: Y(b.y), "font-size": fs, "text-anchor": "middle", "dominant-baseline": "middle",
                      class: "lbl onfill", transform: `rotate(-90 ${X(tx0 + B.d / 2)} ${Y(b.y)})` }, name);
    });
    el(g, "text", { x: X(tx0) - 1.8, y: Y(red.y + B.mouth[0] / 2) - 1.2, "font-size": fs * 0.75, "text-anchor": "end", class: "muted" }, "mouth");
    el(g, "circle", { class: "pivot", cx: X(txc), cy: Y(T.y), r: 1.3 });
    el(g, "text", { x: X(tx1) + 2, y: Y(T.y) - 0.6, "font-size": fs * 0.9, class: "lbl" }, "TABLE CENTRE");
    el(g, "text", { x: X(tx1) + 2, y: Y(T.y) + fs * 1.0, "font-size": fs * 0.9 }, "(" + fmt(txc, 0) + ", " + fmt(T.y, 0) + ")");
    // Mailbox Start (typed) and the release point (the Mailbox Get)
    el(g, "circle", { class: "wp", cx: X(ms[0]), cy: Y(ms[1]), r: 1.3 });
    el(g, "text", { x: X(ms[0]) + 2, y: Y(ms[1]) - 1.6, "font-size": fs * 0.85, class: "lbl" }, "MAILBOX START (" + fmt(ms[0], 0) + ", " + fmt(ms[1], 0) + ")");
    el(g, "circle", { class: "wpred", cx: X(rel[0]), cy: Y(rel[1]), r: 1.1 });
    // dimensions: from the origin to the pillar and to the table's front
    dimH(g, X(0), X(P.x), Y(0), Y(-560), fmt(P.x, 0), fs, s, Y(P.y - F.hat_r));
    dimH(g, X(0), X(tx0), Y(0), Y(-690), fmt(tx0, 0) + " · table front", fs, s, Y(tyR));
    dimH(g, X(tx0), X(tx1), Y(tyR), Y(tyR - 70), fmt(T.depth_x, 0), fs, s);
    dimV(g, Y(0), Y(P.y), X(0), X(-330), fmt(P.y, 0), fs, s);
    // along the table's front, from its left (+y) end: the boxes' edges
    const chain = [[tyL, red.yL], [red.yL, red.yR], [red.yR, blue.yL], [blue.yL, blue.yR], [blue.yR, tyR]];
    chain.forEach(([a, b]) => dimV(g, Y(a), Y(b), X(tx0), X(tx0) - 7, fmt(a - b, 1), fs, s));
    dimV(g, Y(tyL), Y(blue.yL), X(tx0), X(tx0) - 17, fmt(tyL - blue.yL, 1) + " to blue's left edge", fs, s);
    dimV(g, Y(tyL), Y(tyR), X(tx0), X(tx0) - 27, fmt(T.len_y, 0) + " · table", fs, s);
  })();

  function applyTheme() { themeHooks.forEach(f => f()); }
  try { window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", applyTheme); } catch (e) {}
  new MutationObserver(applyTheme).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
  applyTheme();

  /* ================= files ================= */
  const status = $("status");
  function say(msg, cls) { status.textContent = msg; status.className = "status" + (cls ? " " + cls : ""); }
  function copier(btn, text, pre, name) {
    $(btn).addEventListener("click", () => {
      const fallback = () => {
        $(pre).parentElement.open = true;
        const r = document.createRange(); r.selectNodeContents($(pre));
        const s = getSelection(); s.removeAllRanges(); s.addRange(r);
        say("Selected the " + name + " source below: copy it with your keyboard.");
      };
      try { navigator.clipboard.writeText(text).then(() => say("Copied the " + name + " OpenSCAD source.", "ok"), fallback); }
      catch (e) { fallback(); }
    });
  }
  copier("b-copy-core", SCAD_CORE, "src-core", "envelope core");
  copier("b-copy-frame", SCAD_FRAME, "src-frame", "localization frame");

  const O2 = F.offsets;
  const README = [
    "Envelope Mailing Design (2026-10-29). Units mm.",
    "",
    "envelope_core.stl / .scad (rev C, all PLA): 150 x 90 x 3.8 plate, four 30 x 36",
    "  through-windows (corners at x 35/71, y 6/48 from the near-right corner), 6 mm",
    "  webs, solid grip band 0-35 and far band 101-150. Print flat, no supports,",
    "  0.2 mm layers, 3 walls, 4 top + 4 bottom layers, 20 % infill. Estimate",
    "  " + fmt(C.mass_g, 0) + " g wrapped, centre of mass " + fmt(C.com_from_far_mm, 0) + " mm from the far end; with the far band",
    "  at 100 % infill (slicer modifier) " + fmt(SF.mass_g, 0) + " g, " + fmt(SF.com_from_far_mm, 0) + " mm.",
    "",
    "localization_frame.stl / .scad (rev B): sits on the pick hat (skirt arcs over",
    "  the Ø" + fmt(2 * F.hat_r, 1) + " rim) or on the mailbox top (skirt flats on the 125 mm front",
    "  and back faces; ends " + fmt(F.box_side_gap, 1) + " mm in from each side). Print skirts down, supports",
    "  under the plate. Five pockets, " + fmt(F.pocket_d, 0) + " mm deep, take the marker holders",
    "  (holders/holder_A..D.stl: one A, B, C, two D), glued in, turned as listed:",
    ...F.holders.map(h => "    " + h.t + "  pocket centre (" + fmt(h.x, 0) + ", " + fmt(h.y, 0) + ")  " + fmt(h.pocket[0], 1) + " x " + fmt(h.pocket[1], 1) +
      "  turned " + h.heading + " deg  ->  marker centre (" + h.ball.map(v => fmt(v, 1)).join(", ") + ")"),
    "  Frame axes: O = centre of the underside, +x = the drone's travel, z up.",
    "  obj_0 in Motive: the five markers, frame level with +x along the room's +x,",
    "  DEFAULT pivot (their centroid, (" + CEN.map(v => fmt(v, 2)).join(", ") + ") from O).",
    "",
    "The planner adds (hardware mail-delivery yaml), frame axes, m, from the centroid:",
    "  pick_place_pick_locator:  [" + LOC.pick.join(", ") + "]  -> envelope centre " + O2.pick.map(v => fmt(v, 2)).join(", ") + " mm from O",
    "  pick_place_place_locator: [" + LOC.place.join(", ") + "]  -> release point " + O2.place.map(v => fmt(v, 2)).join(", ") + " mm from O",
    "Envelope row: Get with the frame on the hat; Mailbox row: Get with it on the",
    "  mailbox top, +x into the mouth (the yaw comes from the frame too).",
    "On the hat, slide the envelope into the U-slot, far end first, to the stop.",
    ""].join("\n");

  function crc32(u8) {
    let crc = 0xffffffff;
    for (let i = 0; i < u8.length; i++) {
      crc ^= u8[i];
      for (let k = 0; k < 8; k++) crc = (crc & 1) ? (crc >>> 1) ^ 0xedb88320 : crc >>> 1;
    }
    return (crc ^ 0xffffffff) >>> 0;
  }
  function zip(files) {
    const enc = new TextEncoder(), chunks = [], central = [];
    let off = 0;
    const DATE = ((2026 - 1980) << 9) | (10 << 5) | 29, TIME = 12 << 11;
    files.forEach(f => {
      const name = enc.encode(f.name), data = f.data, crc = crc32(data);
      const h = new DataView(new ArrayBuffer(30));
      h.setUint32(0, 0x04034b50, true); h.setUint16(4, 20, true); h.setUint16(6, 0, true); h.setUint16(8, 0, true);
      h.setUint16(10, TIME, true); h.setUint16(12, DATE, true); h.setUint32(14, crc, true);
      h.setUint32(18, data.length, true); h.setUint32(22, data.length, true); h.setUint16(26, name.length, true); h.setUint16(28, 0, true);
      chunks.push(new Uint8Array(h.buffer), name, data);
      const c = new DataView(new ArrayBuffer(46));
      c.setUint32(0, 0x02014b50, true); c.setUint16(4, 20, true); c.setUint16(6, 20, true); c.setUint16(8, 0, true); c.setUint16(10, 0, true);
      c.setUint16(12, TIME, true); c.setUint16(14, DATE, true); c.setUint32(16, crc, true);
      c.setUint32(20, data.length, true); c.setUint32(24, data.length, true); c.setUint16(28, name.length, true);
      c.setUint16(30, 0, true); c.setUint16(32, 0, true); c.setUint16(34, 0, true); c.setUint16(36, 0, true);
      c.setUint32(38, 0, true); c.setUint32(42, off, true);
      central.push(new Uint8Array(c.buffer), name);
      off += 30 + name.length + data.length;
    });
    const cdSize = central.reduce((a, b) => a + b.length, 0);
    const e = new DataView(new ArrayBuffer(22));
    e.setUint32(0, 0x06054b50, true); e.setUint16(4, 0, true); e.setUint16(6, 0, true);
    e.setUint16(8, files.length, true); e.setUint16(10, files.length, true);
    e.setUint32(12, cdSize, true); e.setUint32(16, off, true); e.setUint16(20, 0, true);
    return new Blob([...chunks, ...central, new Uint8Array(e.buffer)]);
  }
  const zipBtn = $("b-zip");
  if (window.claude && typeof window.claude.use === "function") {
    window.claude.use("downloads").then(dl => {
      if (!dl) return;
      zipBtn.hidden = false;
      zipBtn.addEventListener("click", () => {
        const enc = new TextEncoder();
        const blob = zip([{ name: "envelope_core.stl", data: b64bytes("core-stl") },
                          { name: "envelope_core.scad", data: enc.encode(SCAD_CORE) },
                          { name: "localization_frame.stl", data: b64bytes("frame-stl") },
                          { name: "localization_frame.scad", data: enc.encode(SCAD_FRAME) },
                          { name: "localization_frame.json", data: enc.encode(JSON.stringify(F, null, 1)) },
                          ...["A", "B", "C", "D"].map(t => ({ name: "holders/holder_" + t + ".stl", data: b64bytes("holder-" + t + "-stl") })),
                          { name: "README.txt", data: enc.encode(README) }]);
        say("Waiting for you to confirm the download…");
        dl.save({ filename: "envelope_mailing_design.zip", data: blob }).then(
          () => say("Saved envelope_mailing_design.zip.", "ok"),
          err => {
            const c = err && err.code;
            if (c === "declined") say("Download cancelled.");
            else if (c === "rate_limited") say("A download prompt is already open. Finish it, then try again.");
            else { say("Downloads are not available in this view. The files are in the repository folder below.", "err"); zipBtn.hidden = true; }
          });
      });
    }).catch(() => {});
  }
})();
