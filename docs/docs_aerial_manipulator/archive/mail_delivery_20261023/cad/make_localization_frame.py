#!/usr/bin/env python3
"""THE MAIL-DELIVERY LOCALIZATION FRAME, rev B (2026-10-29): one printed plate
that carries FIVE MARKERS on the user's own marker holders (the four designs of
the valve, extracted by extract_marker_holders.py into holders/) and sits on the
pick hat AND on the red mailbox's top, so the ground station's Get on each row
reads where that target is. Written as a binary STL of the plate (to print), an
assembly STL (plate + holders + markers, to look at), an OpenSCAD source (to
edit) and a JSON of every dimension, marker position and offset (for the drawing
page and the planner's locator parameters).

    /usr/bin/python3 make_localization_frame.py [--out DIR]

Frame axes: origin O at the centre of the frame's UNDERSIDE (the face it rests
on), +x = the way the drone flies at that step (into the envelope on the hat,
into the mouth on the mailbox), +y left, z up.

The mocap rigid body (obj_0) is the five markers with Motive's DEFAULT pivot,
their centroid, created with the frame level and its +x along the room's +x.
The planner adds the offset from that centroid to each target
(pick_place_pick_locator / pick_place_place_locator, printed below): nothing is
measured from O on the day.

How it registers (one frame, both targets; unchanged from rev A):
  * Two skirts hang 3 mm below the underside at the +-x ends. Their inner face
    is an arc of radius R_IN = hat radius + clearance in the middle, so on the
    hat they drop over the platform's rim and centre the frame on it (yaw is
    free: set it along the drone's approach).
  * Near the frame's corners the same skirts end in FLAT faces at
    x = +-(box depth / 2 + clearance). On the mailbox those flats close on its
    front and back faces: x and yaw are fixed, y is centred by eye (the frame's
    ends sit the same distance in from each side of the box).
  * A U-slot open to the -x edge is the envelope's placement jig on the hat: slide
    the envelope in from the drone's side until its far end touches the slot's
    end, then lift the frame off. Its centre is then at (-95.3, 0) from the hat
    centre, exactly where the planner wants it.

Rev B replaces rev A's five square posts (and its weight-saving window, which
the holders now cover) with five rectangular POCKETS, 2 mm deep, each the
holder's base + 0.2 mm a side: a holder glued into its pocket stands at a
fixed height and heading, so the marker positions below are exact. The layout
was searched on a 5 mm grid (scratch tool, not kept) for: every pairwise marker
distance different from every other by >= 13 mm (Motive identifies a body by
them, and a near-symmetric layout flips), markers >= 60 mm apart, none over
the envelope's slot, every base over the mailbox's top (inside the skirt
flats).

The geometry is 2.5-D: a structured grid of quadrilateral columns whose x
boundaries are functions of y (the skirt arc is one of them), times a set of z
levels. The solid is a union of grid cells and its boundary is written cell
face by cell face, so every vertex is a grid node and the mesh is closed
(checked before writing).
"""
import argparse
import itertools
import json
import math
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- what it fits [mm] (from the task: tools/mail_geometry.py, the 07 mail scene) ----
HAT_R = 100.0           # pick hat platform radius: the real cap, MEASURED 200 mm across (2026-10-29;
                        # the simulation's Top_Hat asset is 200.6, so the sim hat is 0.3 mm larger a side)
HAT_T = 5.0             # platform thickness (Top_Hat; not measured: the 3 mm skirt must stay shallower)
BOX_D, BOX_W = 125.0, 235.0   # red mailbox top, depth (front to back) x width
CLR = 0.3               # clearance a side on every registering face
ENV_L, ENV_W, ENV_T = 150.0, 90.0, 4.1   # the envelope as wrapped (core 3.8 + paper)
OVERHANG = 70.0         # its near edge beyond the hat's rim

# ---- the frame -------------------------------------------------------------------
R_IN = HAT_R + CLR                  # skirt arc
XF = 0.5 * BOX_D + CLR              # skirt flats
SKIRT_WALL = 4.0
XO = R_IN + SKIRT_WALL              # outer half-length along x
HY = 100.0                          # outer half-width along y
PLATE_T = 4.0
SKIRT_D = 3.0                       # below the underside
SLOT_HY = 0.5 * ENV_W + CLR         # U-slot half-width
SLOT_STOP = -(HAT_R + OVERHANG) + ENV_L   # the envelope's far end, from the hat centre
POCKET_D = 2.0                      # holder pockets: depth (= the holders' base thickness)
POCKET_CLR = 0.2                    # ... clearance a side (rounded to a 0.5 mm pocket size)
# the five holders: design, base centre x, y [mm, from O], heading [deg] (the
# holder turned about z from its CAD orientation)
HOLDERS = (("A", 10.0, 35.0, 90), ("B", 25.0, 80.0, 270), ("C", -25.0, -75.0, 90),
           ("D", -35.0, 80.0, 90), ("D", 40.0, -65.0, 180))
HOLDER_DIR = os.path.join(HERE, "holders")

Y_KINK = math.sqrt(R_IN ** 2 - XF ** 2)   # where the arc meets the flats
RHO_PLA = 1.24e-3


def designs():
    with open(os.path.join(HOLDER_DIR, "holders.json")) as f:
        return json.load(f)["designs"]


def rot(deg):
    c, s_ = round(math.cos(math.radians(deg))), round(math.sin(math.radians(deg)))
    return ((c, -s_), (s_, c))


def holders():
    """Each holder in frame axes: its pocket (x0, x1, y0, y1), its ball centre."""
    D = designs()
    out = []
    for t, x, y, h in HOLDERS:
        if h % 90:
            raise SystemExit("holder headings must be multiples of 90 deg")
        bx, by = D[t]["base_size"]
        if h % 180:
            bx, by = by, bx
        px, py = (round(2 * (b + 2 * POCKET_CLR)) / 2 for b in (bx, by))
        R = rot(h)
        bl = D[t]["ball"]
        ball = (x + R[0][0] * bl[0] + R[0][1] * bl[1], y + R[1][0] * bl[0] + R[1][1] * bl[1],
                PLATE_T - POCKET_D + bl[2])
        out.append(dict(t=t, x=x, y=y, heading=h, base=(bx, by), pocket=(px, py),
                        box=(x - px / 2, x + px / 2, y - py / 2, y + py / 2), ball=ball))
    return out


def g(y):
    """The skirt's inner boundary |x| at this y: the arc, then the flat."""
    return math.sqrt(R_IN ** 2 - y * y) if abs(y) < Y_KINK else XF


def columns(y):
    """Column boundaries (x) at this y, increasing. The holders' pockets are
    kept inside the skirt flats (|x| < XF), so the straight boundaries never
    cross the skirt arc and the order is the same on every row."""
    straight = {SLOT_STOP}
    for h in holders():
        straight |= {h["box"][0], h["box"][1]}
    return [-XO, -g(y), *sorted(straight), g(y), XO]


def rows():
    r = {-HY, HY, -Y_KINK, Y_KINK, -SLOT_HY, SLOT_HY}
    for h in holders():
        r |= {h["box"][2], h["box"][3]}
    th_k = math.asin(Y_KINK / R_IN)
    n = 80
    for i in range(1, n):
        r.add(R_IN * math.sin(-th_k + 2 * th_k * i / n))
    out = []
    for v in sorted(r):
        if not out or v - out[-1] > 1e-6:
            out.append(v)
    return out


def zlevels():
    return [-SKIRT_D, 0.0, PLATE_T - POCKET_D, PLATE_T]


def occupied(xm, ym, zm):
    in_slot = xm < SLOT_STOP and abs(ym) < SLOT_HY
    if in_slot:
        return False
    if zm < 0.0:
        return abs(xm) > g(ym)
    if zm < PLATE_T - POCKET_D:
        return True
    if zm < PLATE_T:
        return not any(b[0] < xm < b[1] and b[2] < ym < b[3] for b in (h["box"] for h in holders()))
    return False


def design_tabs():
    """Each design's marker TAB (the flat plate the ball sits on) as a box, from
    its marker STL: every vertex farther than the ball radius from the ball
    centre. For the drawing page's 3-D view (the markers are drawn as a ball +
    this box, not shipped as meshes)."""
    out = {}
    for t, d in designs().items():
        c, r = d["ball"], d["ball_r"]
        V = [p for tri in read_stl(os.path.join(HOLDER_DIR, f"marker_{t}.stl")) for p in tri
             if math.dist(p, c) > r + 0.3]
        out[t] = dict(lo=[round(min(v[i] for v in V), 2) for i in range(3)],
                      hi=[round(max(v[i] for v in V), 2) for i in range(3)])
    return out


def check_layout(H):
    """Every pocket inside the skirt flats and the plate, off the slot, not
    overlapping another; every ball off the slot."""
    for h in H:
        x0, x1, y0, y1 = h["box"]
        if x0 <= -XF + 1.0 or x1 >= XF - 1.0 or y0 <= -HY + 2.0 or y1 >= HY - 2.0:
            raise SystemExit(f"holder {h['t']} at ({h['x']}, {h['y']}): pocket {h['box']} too near an edge")
        if x0 < SLOT_STOP + 3.0 and y1 > -SLOT_HY - 3.0 and y0 < SLOT_HY + 3.0:
            raise SystemExit(f"holder {h['t']} at ({h['x']}, {h['y']}): pocket over the envelope slot")
        bx, by, _ = h["ball"]
        if bx < SLOT_STOP + 20.0 and abs(by) < SLOT_HY + 20.0:
            raise SystemExit(f"holder {h['t']}: ball over the envelope slot")
    for a, b in itertools.combinations(H, 2):
        A, B = a["box"], b["box"]
        if A[0] < B[1] + 3 and B[0] < A[1] + 3 and A[2] < B[3] + 3 and B[2] < A[3] + 3:
            raise SystemExit(f"pockets {a['t']} at ({a['x']}, {a['y']}) and {b['t']} at ({b['x']}, {b['y']}) overlap")


def marker_stats(H):
    B = [h["ball"] for h in H]
    d = sorted(math.dist(p, q) for p, q in itertools.combinations(B, 2))
    cen = [sum(p[i] for p in B) / len(B) for i in range(3)]
    return dict(distances=d, min_distance=d[0], min_difference=min(b - a for a, b in zip(d, d[1:])), centroid=cen)


def mesh():
    ys, zs = rows(), zlevels()
    cols = [columns(y) for y in ys]
    nk = len(cols[0]) - 1
    if any(len(c) - 1 != nk or any(b <= a for a, b in zip(c, c[1:])) for c in cols):
        raise SystemExit("column boundaries are not ordered on every row")
    nj, nl = len(ys) - 1, len(zs) - 1

    def node(k, j, l):
        return (cols[j][k], ys[j], zs[l])

    occ = {}
    for k, j, l in itertools.product(range(nk), range(nj), range(nl)):
        ym = 0.5 * (ys[j] + ys[j + 1])
        cm = columns(ym)
        xm = 0.5 * (cm[k] + cm[k + 1])
        occ[k, j, l] = occupied(xm, ym, 0.5 * (zs[l] + zs[l + 1]))

    def full(k, j, l):
        return 0 <= k < nk and 0 <= j < nj and 0 <= l < nl and occ[k, j, l]

    tris = []

    def emit(quad, centre):
        a, b, c, d = quad
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        fc = [(a[i] + b[i] + c[i] + d[i]) / 4 for i in range(3)]
        if sum(n[i] * (fc[i] - centre[i]) for i in range(3)) < 0:
            a, b, c, d = a, d, c, b
            n = [-x for x in n]
        ln = math.sqrt(sum(x * x for x in n)) or 1.0
        n = tuple(x / ln for x in n)
        tris.append((n, a, b, c))
        tris.append((n, a, c, d))

    for (k, j, l), on in occ.items():
        if not on:
            continue
        corners = [node(kk, jj, ll) for kk in (k, k + 1) for jj in (j, j + 1) for ll in (l, l + 1)]
        centre = [sum(p[i] for p in corners) / 8 for i in range(3)]
        if not full(k - 1, j, l):
            emit([node(k, j, l), node(k, j + 1, l), node(k, j + 1, l + 1), node(k, j, l + 1)], centre)
        if not full(k + 1, j, l):
            emit([node(k + 1, j, l), node(k + 1, j + 1, l), node(k + 1, j + 1, l + 1), node(k + 1, j, l + 1)], centre)
        if not full(k, j - 1, l):
            emit([node(k, j, l), node(k + 1, j, l), node(k + 1, j, l + 1), node(k, j, l + 1)], centre)
        if not full(k, j + 1, l):
            emit([node(k, j + 1, l), node(k + 1, j + 1, l), node(k + 1, j + 1, l + 1), node(k, j + 1, l + 1)], centre)
        if not full(k, j, l - 1):
            emit([node(k, j, l), node(k + 1, j, l), node(k + 1, j + 1, l), node(k, j + 1, l)], centre)
        if not full(k, j, l + 1):
            emit([node(k, j, l + 1), node(k + 1, j, l + 1), node(k + 1, j + 1, l + 1), node(k, j + 1, l + 1)], centre)
    return tris


def check_closed(tris):
    edges = {}
    for _, a, b, c in tris:
        for p, q in ((a, b), (b, c), (c, a)):
            edges[(p, q)] = edges.get((p, q), 0) + 1
    bad = [e for e, n in edges.items() if n != 1 or edges.get((e[1], e[0]), 0) != 1]
    if bad:
        raise SystemExit(f"mesh not closed: {len(bad)} bad directed edges, e.g. {bad[:3]}")


def volume(tris):
    v = 0.0
    for _, a, b, c in tris:
        v += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
              + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    return v


def write_stl(tris, path):
    with open(path, "wb") as f:
        f.write(b"mail localization frame rev B, mm".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for n, a, b, c in tris:
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))


SCAD = """// Mail-delivery localization frame, rev B (2026-10-29), units mm. Generated
// by make_localization_frame.py; the same dimensions as localization_frame.stl.
// Origin O = centre of the UNDERSIDE (the face it rests on); +x = the drone's
// travel at that step; +y left; z up. The five marker holders (holders/*.stl)
// glue into the pockets; obj_0 = their markers, Motive's default pivot.
HAT_R = {HAT_R}; BOX_D = {BOX_D}; CLR = {CLR};           // what it fits
ENV_L = {ENV_L}; ENV_W = {ENV_W}; OVERHANG = {OVERHANG};      // the envelope it places
R_IN = HAT_R + CLR;          // skirt arc (drops over the hat's rim)
XF = BOX_D / 2 + CLR;        // skirt flats (close on the mailbox's front and back faces)
XO = R_IN + {SKIRT_WALL}; HY = {HY};
PLATE_T = {PLATE_T}; SKIRT_D = {SKIRT_D};
SLOT_HY = ENV_W / 2 + CLR;                 // U-slot, open to the -x edge
SLOT_STOP = -(HAT_R + OVERHANG) + ENV_L;   // the envelope's far end stop
POCKET_D = {POCKET_D};
POCKETS = [{pockets}];   // centre x, y, size x, size y (holder design in the comment)

difference() {{
    union() {{
        translate([-XO, -HY, 0]) cube([2 * XO, 2 * HY, PLATE_T]);              // plate
        difference() {{                                                        // the two skirts
            translate([-XO, -HY, -SKIRT_D]) cube([2 * XO, 2 * HY, SKIRT_D]);
            translate([0, 0, -SKIRT_D - 1]) cylinder(r = R_IN, h = SKIRT_D + 2, $fn = 720);
            translate([-XF, -HY - 1, -SKIRT_D - 1]) cube([2 * XF, 2 * HY + 2, SKIRT_D + 2]);
        }}
    }}
    translate([-XO - 1, -SLOT_HY, -SKIRT_D - 1]) cube([SLOT_STOP + XO + 1, 2 * SLOT_HY, PLATE_T + SKIRT_D + 2]);
    for (p = POCKETS) translate([p[0] - p[2] / 2, p[1] - p[3] / 2, PLATE_T - POCKET_D])
        cube([p[2], p[3], POCKET_D + 1]);
}}
"""


def offsets():
    """The targets in frame axes [mm], from O. Pick point = the envelope centre;
    place point = the envelope centre at the release (the sim mail task block:
    mouth 190 x 90, top edge 25 mm under the box top; claw 21.5 mm above the
    mouth centre; jaw tips 35 mm short of the wall). Claw = point + (-60, 0, 0)
    (EE Offset x -0.06)."""
    pick = (SLOT_STOP - 0.5 * ENV_L, 0.0, 0.5 * ENV_T)
    mouth_centre_below_top = 25.0 + 0.5 * 90.0
    place = (-0.5 * BOX_D + 9.0, 0.0, -(mouth_centre_below_top - 21.5))
    ee = -60.0
    return dict(pick=pick, place=place, pick_claw=(pick[0] + ee, 0.0, pick[2]),
                place_claw=(place[0] + ee, 0.0, place[2]))


def read_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        raw = f.read(50 * n)
    out = []
    for i in range(n):
        v = struct.unpack_from("<12f", raw, 50 * i)
        out.append(((v[3], v[4], v[5]), (v[6], v[7], v[8]), (v[9], v[10], v[11])))
    return out


def assembly(frame_tris, H):
    """The plate + each holder and its marker, placed (to look at, not to print)."""
    tris = [(a, b, c) for _, a, b, c in frame_tris]
    for h in H:
        R = rot(h["heading"])
        z0 = PLATE_T - POCKET_D
        for part in ("holder", "marker"):
            for tri in read_stl(os.path.join(HOLDER_DIR, f"{part}_{h['t']}.stl")):
                tris.append(tuple((h["x"] + R[0][0] * p[0] + R[0][1] * p[1],
                                   h["y"] + R[1][0] * p[0] + R[1][1] * p[1], z0 + p[2]) for p in tri))
    return tris


def write_stl_plain(tris, path, name):
    with open(path, "wb") as f:
        f.write(name.encode()[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            ln = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack("<12fH", *(x / ln for x in n), *a, *b, *c, 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=HERE)
    a = ap.parse_args()
    H = holders()
    check_layout(H)
    tris = mesh()
    check_closed(tris)
    vol = volume(tris)
    os.makedirs(a.out, exist_ok=True)
    stl = os.path.join(a.out, "localization_frame.stl")
    write_stl(tris, stl)
    write_stl_plain(assembly(tris, H), os.path.join(a.out, "localization_frame_assembly.stl"),
                    "mail localization frame rev B + holders + markers (view only), mm")
    with open(os.path.join(a.out, "localization_frame.scad"), "w") as f:
        f.write(SCAD.format(HAT_R=HAT_R, BOX_D=BOX_D, CLR=CLR, ENV_L=ENV_L, ENV_W=ENV_W, OVERHANG=OVERHANG,
                            SKIRT_WALL=SKIRT_WALL, HY=HY, PLATE_T=PLATE_T, SKIRT_D=SKIRT_D, POCKET_D=POCKET_D,
                            pockets=", ".join(f"[{h['x']}, {h['y']}, {h['pocket'][0]}, {h['pocket'][1]}] /* {h['t']} */"
                                              for h in H)))
    st = marker_stats(H)
    off = offsets()
    c = st["centroid"]
    loc = {k: [round((off[k][i] - c[i]) / 1e3, 4) for i in range(3)] + [0.0] for k in ("pick", "place")}
    info = dict(rev="B", hat_r=HAT_R, hat_t=HAT_T, box_d=BOX_D, box_w=BOX_W, clr=CLR, env=[ENV_L, ENV_W, ENV_T],
                overhang=OVERHANG, r_in=R_IN, xf=XF, xo=XO, hy=HY, plate_t=PLATE_T, skirt_d=SKIRT_D,
                skirt_wall=SKIRT_WALL, slot_hy=SLOT_HY, slot_stop=SLOT_STOP, pocket_d=POCKET_D, pocket_clr=POCKET_CLR,
                holders=[dict(t=h["t"], x=h["x"], y=h["y"], heading=h["heading"], base=list(h["base"]),
                              pocket=list(h["pocket"]), box=list(h["box"]), ball=[round(v, 2) for v in h["ball"]])
                         for h in H],
                designs=designs(), tabs=design_tabs(), markers=st, y_kink=Y_KINK, triangles=len(tris), volume_mm3=vol,
                pla_g=vol * RHO_PLA, box_side_gap=0.5 * BOX_W - HY, offsets=off, locator=loc)
    with open(os.path.join(a.out, "localization_frame.json"), "w") as f:
        json.dump(info, f, indent=1)
    print(f"closed mesh, {len(tris)} triangles -> {stl}")
    print(f"outline {2 * XO:.1f} x {2 * HY:.1f} mm, plate {PLATE_T} + skirts {SKIRT_D} below; "
          f"skirt arc R {R_IN:.1f}, flats at x +-{XF:.1f} (from |y| {Y_KINK:.1f})")
    print(f"volume {vol / 1e3:.1f} cm3 = {vol * RHO_PLA:.0f} g solid PLA (less with infill)")
    print(f"U-slot {2 * SLOT_HY:.1f} wide, stop at x {SLOT_STOP:.1f}; on the box the ends sit "
          f"{0.5 * BOX_W - HY:.1f} mm in from each side")
    for h in H:
        print(f"   holder {h['t']} base ({h['x']:6.1f}, {h['y']:6.1f}) heading {h['heading']:3d}: pocket "
              f"{h['pocket'][0]} x {h['pocket'][1]} x {POCKET_D}, ball at {[round(v, 1) for v in h['ball']]}")
    print(f"markers: min distance {st['min_distance']:.1f} mm, distances differ by >= {st['min_difference']:.1f} mm; "
          f"centroid (Motive's default pivot) {[round(v, 2) for v in c]} mm from O")
    for k, v in off.items():
        print(f"   {k:11s} {[round(x, 2) for x in v]} mm from O")
    print("planner locator parameters (frame axes, from the marker centroid) [m, m, m, deg]:")
    print(f"   pick_place_pick_locator:  {loc['pick']}")
    print(f"   pick_place_place_locator: {loc['place']}")


if __name__ == "__main__":
    main()
