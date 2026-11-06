#!/usr/bin/env python3
"""Add the Onshape sketches (Figs. 14-16) to section 5 of the Valve Turning Design page."""
import sys

src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding="utf-8").read()


def sub(old, new, count=1):
    global s
    n = s.count(old)
    if n != count:
        raise SystemExit(f"expected {count} x {old[:70]!r}, found {n}")
    s = s.replace(old, new)


# ---- CSS: the sketch SVGs may be narrower than the page's 640 px minimum
sub("#pSimTable tr.warn td { color: var(--warn); }",
    "#pSimTable tr.warn td { color: var(--warn); }\n"
    ".scroll.sk svg { min-width: 300px; max-height: 500px; }\n"
    ".hpic { display: block; width: 100%; max-width: 832px; margin: 0 auto; border-radius: 4px; }\n"
    ".viewbar button[aria-pressed=\"true\"] { color: var(--ink); background: #1d2a3a; }\n"
    ".skdim { font-variant-numeric: tabular-nums; }")

# ---- Fig. 13's caption and the Print-and-fit card: two bolts, drawn from the sketches
sub("the clip's two cheeks hug the fin for 15 mm, one M3 bolt through cheeks and fin.",
    "the clip's two cheeks hug the fin for 15 mm, two M3 bolts through cheeks and fin (Figs. 14 to 16).")

old_print = s[s.index("        <li>Print the plate and clip as one part"):s.index("        <li>Cover the face with PTFE tape.")]
sub(old_print,
    "        <li>Draw the plate from Figs. 14 and 15 and print it face down, so the pushing face comes out flat and the cheeks stand up. "
    "<span class=\"mono\">Valve_Assembly_v21.stl</span> is this part as drawn in Onshape, with a 5.0 mm slot that touches the fin; widen the slot to 5.3 mm in the print file, or it will not slide on.</li>\n"
    "        <li>Drill the fin (Figs. 16 and 17): two 3.2 mm holes through it, 7.5 mm from the pushing edge, 15 and 60 mm below its top. "
    "Slide the plate onto the pushing edge, on marker D's side (the side the drone comes from), tops flush, and fit two M3 × 16 screws with nuts. "
    "Tighten lightly: the cheeks are 3 mm of PLA.</li>\n")

# ---- the new block, after Figs. 12 and 13
anchor = "  <h3>Why this size</h3>\n  <ul id=\"pWhy\"></ul>"
block = r'''  <h3 id="plate-cad">Onshape sketches</h3>
  <p>Everything needed to draw the plate and drill the handle, in mm. The plate is one new printed part made from two sketches. The handle, today's fin, gets two holes.</p>
  <div class="two">
    <div class="card">
      <h3>The push plate (new part)</h3>
      <ol>
        <li><strong>Sketch 1</strong> on the Top plane: the profile of Fig. 14. Put the pushing face on the X axis and centre the part on the origin. <strong>Extrude</strong> 79 mm up as a new part.</li>
        <li><strong>Sketch 2</strong> on the Right plane, which is the slot's mid-plane: the two Ø3.2 circles of Fig. 15. <strong>Extrude, remove</strong>, through all, symmetric.</li>
        <li>Optional: a 0.5 mm chamfer round the pushing face. Keep the slot's inside corners sharp, so the fin's edge seats flat on the plate's back.</li>
      </ol>
    </div>
    <div class="card">
      <h3>The handle and the assembly</h3>
      <ol>
        <li><strong>Sketch</strong> on the fin's face: the two Ø3.2 circles of Fig. 16 (pictures in Fig. 17), 7.5 mm from the pushing edge and 15 and 60 mm below the top. <strong>Extrude, remove</strong>, through all. On the printed fin, drill 3.2 mm.</li>
        <li><strong>Fasten</strong> the parts: the slot's bottom on the fin's pushing edge, the two tops flush, and the slot's mid-plane on the fin's mid-plane. The holes then line up.</li>
        <li>Hardware: 2 × M3 × 16 socket-head screws and 2 × M3 nuts. With nyloc nuts, use M3 × 18.</li>
      </ol>
      <p class="why">Two bolts, not one. One bolt would hold the plate, but two keep it square to the push without relying on the fin's edge alone.</p>
    </div>
  </div>
  <figure>
    <div class="two">
      <div class="scroll sk"><svg id="pkProf" role="img" aria-label="Sketch 1 of the push plate: the profile on the Top plane, dimensioned"></svg></div>
      <div class="scroll sk"><svg id="pkDet" role="img" aria-label="Detail of the clip: a section through one bolt, from above"></svg></div>
    </div>
    <figcaption><b>Fig. 14.</b> <b>Sketch 1</b>, Top plane, extruded 79 mm up. Left: the whole profile, to scale. The plate is 120 × 5 mm. The clip is two 3 mm cheeks round a 5.3 mm slot, 15 mm deep, so the part is 20 mm deep overall. The pushing face is the bottom line; the claw's nose pushes it upward. Right: the clip, magnified, cut through one bolt. It shows the 5.0 mm fin in the slot with 0.15 mm of clearance each side, and the M3 screw 7.5 mm behind the plate's back face.</figcaption>
  </figure>
  <div class="two">
    <figure>
      <div class="scroll sk"><svg id="pkHoles" role="img" aria-label="Sketch 2 of the push plate: the two bolt holes on the Right plane, dimensioned"></svg></div>
      <figcaption><b>Fig. 15.</b> <b>Sketch 2</b>, Right plane: the part seen from the side, to scale, with the pushing face on the left. Two Ø3.2 holes, 12.5 mm from the pushing face (7.5 mm behind the plate's back face), 15 and 60 mm below the top. Remove through all, both ways.</figcaption>
    </figure>
    <figure>
      <div class="scroll sk"><svg id="pkFin" role="img" aria-label="The handle, the fin, with the two new holes for the push plate, dimensioned"></svg></div>
      <figcaption><b>Fig. 16.</b> The handle, today's fin from <span class="mono">Valve_Assembly_v20.stl</span>, face-on, to scale. It is 40 × 150 × 5 mm, with its foot 20 mm deep in the socket. New: two Ø3.2 holes (amber), 7.5 mm from the pushing edge and 15 and 60 mm below the top, so 189 and 144 mm above the table. The dashed violet box is where the plate's cheeks sit. Drawn with the pushing edge on the left; mirror the sketch if yours is on the right. The pushing edge is the one on marker D's side, toward −Y. In the assembly as exported to the STL, the holes are at x 158.9, y −8.9, z 184.5 and 139.5 mm.</figcaption>
    </figure>
  </div>
  <figure>
    <div class="viewbar" role="group" aria-label="Pictures of the handle">
      <button type="button" data-hpic="0" aria-pressed="true">a · the handle on its own</button>
      <button type="button" data-hpic="1" aria-pressed="false">b · on the valve</button>
      <button type="button" data-hpic="2" aria-pressed="false">c · fitted</button>
    </div>
    <img class="hpic" data-k="0" src="data:image/png;base64,@H1@" alt="The handle on its own, with the two new holes ringed in amber and dimensioned from its top and its pushing edge">
    <img class="hpic" data-k="1" hidden src="data:image/png;base64,@H2@" alt="The handle end of the valve seen from the drone's side, the holes and the pushing edge marked">
    <img class="hpic" data-k="2" hidden src="data:image/png;base64,@H3@" alt="The push plate fitted on the fin, its cheeks round the fin and two screws through them, seen from above and behind">
    <figcaption><b>Fig. 17.</b> Where to drill, rendered from <span class="mono">Valve_Assembly_v20.stl</span> with the holes and the plate added. <b>a</b>: the handle on its own. The two new holes (amber rings) are 15 and 60 mm below the top and 7.5 mm in from the pushing edge; the four holes at the foot are the existing socket bolts. <b>b</b>: on the valve, seen from where the drone comes from. The pushing edge (amber line) is the fin's edge on marker D's side. <b>c</b>: fitted, from above and behind. The cheeks close round the fin, the two screws pass through both cheeks and the fin, the nuts are on the far side, and the pushing face faces the drone.</figcaption>
  </figure>

'''
sub(anchor, block + anchor)

# ---- the drawing code
js = r'''
// ---------- Figs. 14-16: the Onshape sketches (mm; the part's own frames) ----------
const PK = { plateW: 120, plateT: 5, plateH: 79, slot: 5.3, cheek: 3, cheekL: 15, finT: 5.0, finW: 40, finH: 150,
  socket: 20, edgeIn: 7.5, holeTop: [15, 60], holeD: 3.2, footHoles: [-14, -6, 6, 14], footZ: 5 };

function skExt(f, u0, v0, u1, v1) { f.L(u0, v0, u1, v1, f.ink3, 0.7, "2 2"); }
function skCross(f, u, v, r) { f.L(u - r * 1.8, v, u + r * 1.8, v, f.ink3, 0.6); f.L(u, v - r * 1.8, u, v + r * 1.8, f.ink3, 0.6); }

// Fig. 14 left: Sketch 1, the profile (u = X across the plate, v = Y from the pushing face back)
function drawSketchProfile() {
  const f = pFrame("pkProf", -76, 80, -34, 56, 3.4);
  const { P, R, T, dim, ink, ink2, ink3, warn } = f;
  const K = PK, pp = css("--pplate"), s2 = K.slot / 2, co = s2 + K.cheek, yb = K.plateT, yc = K.plateT + K.cheekL;
  P([[-60, 0], [60, 0], [60, yb], [co, yb], [co, yc], [s2, yc], [s2, yb], [-s2, yb], [-s2, yc], [-co, yc], [-co, yb], [-60, yb]],
    pp, { stroke: "#e4dcff", "stroke-width": 1 });
  R(-K.finT / 2, K.finT / 2, yb, 44, "none", { stroke: ink3, "stroke-width": 1, "stroke-dasharray": "4 3" });
  T(4, 38, "fin (not part of the sketch)", "start", ink3, 11);
  // the origin
  f.C(0, 0, 0.9, { fill: ink }); T(1.5, -5.5, "origin", "start", ink3, 10);
  // dimensions
  skExt(f, -60, 0, -60, -16); skExt(f, 60, 0, 60, -16);
  dim(-60, -12, 60, -12, ""); T(0, -16.5, "120", "middle", ink2, 13);
  skExt(f, -60, 0, -70, 0); skExt(f, -60, yb, -70, yb);
  dim(-67, 0, -67, yb, ""); T(-69.5, 0, "5", "end", ink2, 13);
  skExt(f, 60, 0, 72, 0); skExt(f, co, yc, 72, yc);
  dim(69, 0, 69, yc, "", 0, "r"); T(71.5, 8, "20", "start", ink2, 13);
  // the push
  for (const u of [-34, 34]) f.arrow(u, -32, u, -17, warn, 2);
  T(-31, -27, "push", "start", warn, 11);
  T(-74, 50, "Sketch 1 · Top plane", "start", ink, 13, 600);
  T(-74, 42, "extrude 79 mm up", "start", ink2, 12);
  T(12, 26, "slot + cheeks: see the detail →", "start", ink2, 11);
}

// Fig. 14 right: the clip, magnified, cut through one bolt (seen from above)
function drawSketchDetail() {
  const f = pFrame("pkDet", -15, 21, -7, 34, 12);
  const { R, L, T, dim, ink, ink2, ink3, warn } = f;
  const K = PK, pp = css("--pplate"), s2 = K.slot / 2, co = s2 + K.cheek, yb = K.plateT, yc = K.plateT + K.cheekL;
  const yh = yb + K.edgeIn, steel = css("--steel");
  R(-14, 14, 0, yb, pp);
  for (const u of [-14, 14]) L(u, -0.6, u, yb + 0.6, ink3, 1, "1.5 1.5");
  R(-co, -s2, yb, yc, pp); R(s2, co, yb, yc, pp);
  R(-K.finT / 2, K.finT / 2, yb, 24.5, css("--rotor"));
  T(0, 21, "fin 5.0", "middle", "#333", 10, 600);
  // the screw: head, shank, nut
  R(-co - 3, -co, yh - 2.75, yh + 2.75, steel, { stroke: "#111", "stroke-width": 0.6 });
  R(-co, co + 4.7, yh - 1.5, yh + 1.5, steel, { stroke: "#111", "stroke-width": 0.6 });
  R(co, co + 2.4, yh - 2.75, yh + 2.75, "#9aa0a8", { stroke: "#111", "stroke-width": 0.6 });
  T(-co - 3.4, yh - 0.9, "M3 × 16", "end", ink2, 10);
  T(co + 1.2, yh + 4.2, "nut", "middle", ink2, 10);
  // dimensions above the clip
  skExt(f, -s2, yc, -s2, 28.5); skExt(f, s2, yc, s2, 28.5); skExt(f, -co, yc, -co, 32.5); skExt(f, co, yc, co, 32.5);
  dim(-s2, 27, s2, 27, ""); T(0, 28.1, "5.3", "middle", ink2, 10);
  dim(s2, 27, co, 27, ""); T(co + 0.6, 27.9, "3", "start", ink2, 10);
  dim(-co, 31, co, 31, ""); T(0, 32.1, "11.3", "middle", ink2, 10);
  T(7.5, 23.5, "clearance 0.15 a side", "start", ink3, 9);
  // depths, on the right
  skExt(f, co, yb, 14, yb); skExt(f, co, yc, 14, yc); skExt(f, co + 2.4, yh, 18.5, yh);
  dim(12.5, yb, 12.5, yc, "", 0, "r"); T(13, 13.2, "15", "start", ink2, 10);
  dim(17.5, yb, 17.5, yh, "", 0, "r"); T(18, 8.2, "7.5", "start", warn, 10);
  // the plate's thickness, on the left
  f.arrow(-8, -6.5, -8, -1, warn, 2); T(-7, -5, "push", "start", warn, 10);
  T(5, -5, "detail, cut through a bolt", "start", ink3, 10);
}

// Fig. 15: Sketch 2, the holes (u = Y from the pushing face, v = Z up)
function drawSketchHoles() {
  const f = pFrame("pkHoles", -36, 62, -18, 94, 3.6);
  const { R, L, T, C, dim, ink, ink2, ink3, warn } = f;
  const K = PK, pp = css("--pplate"), yb = K.plateT, yc = K.plateT + K.cheekL, H = K.plateH, yh = yb + K.edgeIn;
  R(0, yb, 0, H, pp); R(yb, yc, 0, H, pp, { "fill-opacity": 0.72 });
  L(yb, 0, yb, H, "#2b2350", 1);
  T(yb / 2, -5, "plate", "middle", ink3, 10); T((yb + yc) / 2, -5, "cheek", "middle", ink3, 10);
  for (const d of K.holeTop) { C(yh, H - d, K.holeD / 2, { fill: css("--panel"), stroke: warn, "stroke-width": 1.4 }); skCross(f, yh, H - d, K.holeD / 2); }
  T(yh, 45, "2 × Ø3.2", "middle", warn, 11, 600); T(yh, 39, "thru all", "middle", warn, 10);
  // along the depth, above
  skExt(f, 0, H, 0, 90); skExt(f, yh, H - 15, yh, 90); skExt(f, yc, H, yc, 84);
  dim(0, 87, yh, 87, "12.5", -5);
  dim(0, 81, yc, 81, ""); T(yc + 1.5, 79.6, "20", "start", ink2, 11);
  // heights, on the right
  skExt(f, yc, H, 58, H); skExt(f, yh, H - 15, 34, H - 15); skExt(f, yh, H - 60, 44, H - 60); skExt(f, yc, 0, 58, 0);
  dim(31, H, 31, H - 15, "", 0, "r"); T(32.5, H - 9, "15", "start", ink2, 11);
  dim(41, H, 41, H - 60, "", 0, "r"); T(42.5, H - 31, "60", "start", ink2, 11);
  dim(55, 0, 55, H, "", 0, "r"); T(56.5, 36, "79", "start", ink2, 11);
  // the push, from the left
  f.arrow(-30, 52, -4, 52, warn, 2); T(-30, 55.5, "push", "start", warn, 11);
  T(-34, 14, "Sketch 2", "start", ink, 12, 600);
  T(-34, 6, "Right plane", "start", ink2, 11);
}

// Fig. 16: the handle (the fin's face; origin at its bottom centre, the pushing edge on the left)
function drawSketchFin() {
  const f = pFrame("pkFin", -96, 112, -26, 172, 2.45);
  const { R, L, T, C, dim, ink, ink2, ink3, warn } = f;
  const K = PK, pp = css("--pplate"), xe = -K.finW / 2, top = K.finH, zt = 204 - top;   // zt: the fin's foot above the table
  R(-20, 20, K.socket, top, css("--rotor"));
  R(-20, 20, 0, K.socket, css("--rotor-2"));
  T(0, 10.5, "in the socket", "middle", "#222", 10);
  for (const u of K.footHoles) C(u, K.footZ, K.holeD / 2, { fill: "#3a3d42", stroke: "#111", "stroke-width": 0.8 });
  // where the cheeks sit
  R(xe, xe + K.cheekL, top - K.plateH, top, "none", { stroke: pp, "stroke-width": 1.6, "stroke-dasharray": "5 3" });
  // the new holes
  for (const d of K.holeTop) { C(xe + K.edgeIn, top - d, K.holeD / 2 + 0.3, { fill: warn, stroke: "#111", "stroke-width": 0.8 }); skCross(f, xe + K.edgeIn, top - d, K.holeD / 2); }
  T(23, top - 30, "2 × Ø3.2", "start", warn, 11, 600); T(23, top - 38, "thru, new", "start", warn, 10);
  L(22, top - 34, xe + K.edgeIn + 2.2, top - 58, warn, 0.8);
  // from the pushing edge, above the top
  skExt(f, xe, top, xe, 166); skExt(f, xe + K.edgeIn, top - 15, xe + K.edgeIn, 166);
  dim(xe, 162, xe + K.edgeIn, 162, ""); T(xe + K.edgeIn + 2, 160.5, "7.5", "start", warn, 11, 600);
  // from the top, on the left
  skExt(f, xe, top, -66, top); skExt(f, xe + K.edgeIn, top - 15, -38, top - 15); skExt(f, xe + K.edgeIn, top - 60, -52, top - 60);
  dim(-35, top, -35, top - 15, ""); T(-37, top - 9.5, "15", "end", warn, 11, 600);
  dim(-49, top, -49, top - 60, ""); T(-51, top - 32, "60", "end", warn, 11, 600);
  // width and height
  skExt(f, -20, 0, -20, -16); skExt(f, 20, 0, 20, -16);
  dim(-20, -12, 20, -12, "40", 14);
  skExt(f, -20, 0, -66, 0);
  dim(-63, 0, -63, top, ""); T(-65, 70, "150", "end", ink2, 11);
  // the edges
  T(-24, 54, "pushing edge", "end", ink, 11, 600); T(-24, 46, "marker D's side", "end", ink2, 10); T(-24, 39, "toward −Y", "end", ink2, 10);
  T(24, 54, "trailing edge", "start", ink2, 11);
  T(xe + 1, top - K.plateH - 7, "cheeks", "start", "#5b45c9", 10, 600);
  // heights above the table, on the right
  T(52, 166, "above the table", "start", ink3, 10);
  for (const [v, t] of [[top, "204 fin top"], [top - 15, "189"], [top - 60, "144"], [top - K.plateH, "125 plate bottom"], [K.socket, "74 socket top"], [0, "54 fin foot"]]) {
    L(20, v, 50, v, ink3, 0.6, "2 3"); T(52, v - 3, t, "start", (v === top - 15 || v === top - 60) ? warn : ink2, 10);
  }
  f.C(0, 0, 1.1, { fill: ink }); T(1.8, -6, "origin", "start", ink3, 10);
}
'''
sub("function plateTables() {", js + "\nfunction plateTables() {")
sub("drawPlate3d(); drawPlateFace(); drawPlateSide(); drawPlateTop(); plateTables();",
    "drawPlate3d(); drawPlateFace(); drawPlateSide(); drawPlateTop(); plateTables();\n"
    "drawSketchProfile(); drawSketchDetail(); drawSketchHoles(); drawSketchFin();\n"
    "document.querySelectorAll(\"[data-hpic]\").forEach(b => b.addEventListener(\"click\", () => {\n"
    "  document.querySelectorAll(\"[data-hpic]\").forEach(o => o.setAttribute(\"aria-pressed\", o === b ? \"true\" : \"false\"));\n"
    "  document.querySelectorAll(\".hpic\").forEach(im => { im.hidden = im.dataset.k !== b.dataset.hpic; });\n"
    "}));")

import base64, os
IMGD = "/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_push_plate_20261104/img"
for k, f in (("@H1@", "h1_handle.png"), ("@H2@", "h2_on_valve.png"), ("@H3@", "h3_fitted.png")):
    s = s.replace(k, base64.b64encode(open(os.path.join(IMGD, f), "rb").read()).decode())
open(dst, "w", encoding="utf-8").write(s)
print("ok", len(s))
