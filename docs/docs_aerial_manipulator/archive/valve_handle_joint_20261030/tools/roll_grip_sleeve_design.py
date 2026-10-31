#!/usr/bin/env python3
"""The roll-joint handle (2026-10-30, second version): printable parts + the numbers behind them.

Replaces the hinged paddle of handle_design.py. Today's fin is sawn off 40 mm above its root; a printed
HANDLE BODY clamps over the stump with two M3 bolts; a printed round GRIP SLEEVE turns on an M4 screw
along the roll axis (in the fin's plane, 30 deg above horizontal, rising toward +y, through the grip
point G). The claw grips the sleeve, so the clamped claw can roll; the sleeve is round, so it needs no
lock or centring before the grasp.

Frame: the valve's (mm): origin on the bearing axis at the plate bottom, +x along the beam to the handle,
+z up; the turn is +90 deg, so the claw's push is +y. Geometry from Valve_Assembly_v2.stl (checked here):
fin x 157.5..162.5, y -19.4..20.6, z 74..204; root x 155..165, y -24.4..25.6, z 54..74; marker E's ball
centre (192.5, 10.7, 92.5) r 15. G = push_pull_ee_offset (0.160, 0.0006, 0.179) m. Claw axis at the
flown pose [0, 5, 25, 0]: along +y and 60 deg down (handle_joint_mobility.py).

Writes, next to this tool's folder:
  stl/handle_body.stl, stl/grip_sleeve.stl   print orientation, mm (handle upright, sleeve on its end)
  roll_grip_parts.json                       the parts in the valve frame, for the page's 3D view
  roll_grip_design.txt                       this run's printout

    PYTHONPATH=~/sources/pymanifold /usr/bin/python3 roll_grip_design.py

manifold3d 3.5.4 lives in ~/sources/pymanifold (pip install --no-deps --target; no venv module here).
"""
import json
import os
import struct
import sys

import numpy as np
import manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
SEG = 64                                           # circle segments for the printed parts

G = np.array([160.0, 0.6, 179.0])
N = np.array([0.0, np.cos(np.radians(30)), np.sin(np.radians(30))])    # the roll axis
C = np.array([0.0, 0.5, -np.sqrt(3) / 2])                             # the claw axis (in-plane normal to N)
X = np.array([1.0, 0.0, 0.0])
K_Y = 120.0                                        # [N/m] the law's EE task stiffness (valve yaml)
GATE, GATE_ACROSS = 20.0, 8.0                      # [mm] the Valve tab's hardware close gate
FINGER_W, FINGER_BACK, FINGER_TIP = 20.0, 40.0, 20.0   # assumed OM-X finger (as handle_design.py)
JAW_OPEN = 37.0                                    # [mm] measured open gap at the fingertips
APPROACH = np.array([0.0, -120.0, 100.0])          # Ready starts 0.12 m behind, 0.10 m above
RHO = 1.04e-3                                      # [g/mm^3] printed PLA, the 10-14 payload estimate

# today's fin and its root (CAD), and where to cut it
FIN = dict(x=(157.5, 162.5), y=(-19.4, 20.6), z=(74.0, 204.0))
ROOT = dict(x=(155.0, 165.0), y=(-24.4, 25.6), z=(54.0, 74.0))
Z_CUT = 104.0                                      # 30 mm above the root top; anything 98..106 fits

# the handle body
SLOT_W = 5.4                                       # fin 5.0 + 0.2 a side
WALL = 4.0
CLAMP = dict(x=(160 - SLOT_W / 2 - WALL, 160 + SLOT_W / 2 + WALL), y=(FIN["y"][0] - 1.0, FIN["y"][1] + 1.0), z=(74.0, 114.0))   # a 6 mm bridge above the slot
SLOT_TOP = 108.0                                   # 2 mm above the highest cut
BOLTS = [(-12.0, 89.0), (12.0, 89.0)]              # (y, z): M3, heads on -x, nuts in pockets on +x
M3_HOLE, M3_NUT_AF, M3_NUT_DEPTH = 3.3, 5.8, 2.5
WEB_T, WEB_GAP = 5.0, 2.0                          # web = the old fin's thickness; its top edge 2 mm under the sleeve
WINDOW_BORDER = 5.0                                # the lightening window keeps 5 mm of web all round

# the joint, along the axis (s = mm along N from G)
SLEEVE_D, SLEEVE_L, BORE_D = 16.0, 50.0, 4.4
WASHER_T, WASHER_D = 0.8, 9.0                      # M4 ISO 7089
NECK_D, NECK_L = 8.0, 6.0                          # a thin neck before the boss: the pads cannot reach the boss
BOSS_D, BOSS_L = 13.0, 11.4
NYLOC_AF, NYLOC_H = 7.25, 5.3                      # M4 nyloc 7 AF x 5 high + clearance
SCREW_L, HEAD_D, HEAD_H = 70.0, 7.0, 4.0           # M4 x 70 ISO 4762 (b = 20: a 50 mm smooth shank)

s_sleeve = (-SLEEVE_L / 2, SLEEVE_L / 2)
s_neck1 = s_sleeve[0] - WASHER_T                   # the neck's face (washer between)
s_boss1 = s_neck1 - NECK_L
s_boss0 = s_boss1 - BOSS_L                         # the boss's far end: the nyloc pocket opens here
s_head = s_sleeve[1] + WASHER_T                    # under the screw head
s_tip = s_head - SCREW_L


def P(s, c=0.0):
    return G + s * N + c * C


def frame(origin, ez, ex=X):
    """3x4 matrix taking local (x, y, z) to the valve frame with local z along ez."""
    ez = ez / np.linalg.norm(ez)
    ey = np.cross(ez, ex)
    ey /= np.linalg.norm(ey)
    ex = np.cross(ey, ez)
    return np.column_stack([ex, ey, ez, origin])


def cyl_along(s0, s1, d, seg=SEG):
    return mf.Manifold.cylinder(s1 - s0, d / 2, d / 2, seg).transform(frame(P(s0), N))


def hex_prism_along(s0, s1, af, vertex_dir):
    """A hexagonal prism on the axis, one vertex pointing along vertex_dir (in the plane normal to N)."""
    r = af / np.sqrt(3)
    pts = [(r * np.cos(np.radians(90 + 60 * k)), r * np.sin(np.radians(90 + 60 * k))) for k in range(6)]
    # local y points along vertex_dir: build the frame with ex chosen so ey = vertex_dir
    ez = N
    ey = vertex_dir / np.linalg.norm(vertex_dir)
    ex = np.cross(ey, ez)
    M = np.column_stack([ex, ey, ez, P(s0)])
    return mf.CrossSection([pts]).extrude(s1 - s0).transform(M)


def box(x, y, z):
    return mf.Manifold.cube((x[1] - x[0], y[1] - y[0], z[1] - z[0])).translate((x[0], y[0], z[0]))


def ccw(poly):
    a = 0.5 * sum(p[0] * q[1] - q[0] * p[1] for p, q in zip(poly, poly[1:] + poly[:1]))
    return poly if a > 0 else poly[::-1]


def yz(p):
    return (float(p[1]), float(p[2]))


def web_polygon():
    s_right = (CLAMP["y"][1] - 3.0 - G[1] - (SLEEVE_D / 2 + WEB_GAP) * C[1]) / N[1]   # right edge 3 mm in
    top = SLEEVE_D / 2 + WEB_GAP
    s_step = s_neck1 - 0.7                         # where the web steps up into the neck/boss
    zb = CLAMP["z"][1] - 2.0
    return ccw([(CLAMP["y"][0], zb), (CLAMP["y"][1] - 3.0, zb), yz(P(s_right, top)),
                yz(P(s_step, top)), yz(P(s_step, 0.0)), yz(P(s_boss0, 0.0)), yz(P(s_boss0, BOSS_D / 2))])


def web_section():
    """The web in the fin plane (y, z), with its lightening window."""
    sec = mf.CrossSection([web_polygon()])
    zb = CLAMP["z"][1] + 6.0                        # a floor 6 mm above the clamp
    yl, yr = CLAMP["y"][0] + 3.0, CLAMP["y"][1] - 9.0
    h = (yr - yl) / 2 * np.tan(np.radians(60))
    tri = mf.CrossSection([ccw([(yl, zb), (yr, zb), ((yl + yr) / 2, zb + h)])])
    # keep clear of the neck/boss: only below the line WINDOW_BORDER under the boss's underside
    far = 400.0
    below = mf.CrossSection([ccw([yz(P(-far, BOSS_D / 2 + WINDOW_BORDER)), yz(P(far, BOSS_D / 2 + WINDOW_BORDER)),
                                  yz(P(far, far)), yz(P(-far, far))])])
    window = tri ^ sec.offset(-WINDOW_BORDER) ^ below
    return sec - window


def handle_body():
    web = web_section().extrude(WEB_T).transform(
        np.column_stack([[0, 1, 0], [0, 0, 1], [1, 0, 0], [160 - WEB_T / 2, 0, 0]]).astype(float))
    body = box(CLAMP["x"], CLAMP["y"], CLAMP["z"]) + web + cyl_along(s_boss0, s_boss1, BOSS_D) \
        + cyl_along(s_boss1, s_neck1, NECK_D)
    cut = box((160 - SLOT_W / 2, 160 + SLOT_W / 2), (-60, 60), (60, SLOT_TOP))
    cut += cyl_along(s_tip - 5, s_neck1 + 1, BORE_D, 32)
    cut += hex_prism_along(s_boss0 - 1, s_boss0 + NYLOC_H, NYLOC_AF, -C)
    for y, z in BOLTS:
        hole = mf.Manifold.cylinder(40, M3_HOLE / 2, M3_HOLE / 2, 32).transform(frame(np.array([140.0, y, z]), X, np.array([0, 1.0, 0])))
        r = M3_NUT_AF / np.sqrt(3)
        pts = [(r * np.cos(np.radians(90 + 60 * k)), r * np.sin(np.radians(90 + 60 * k))) for k in range(6)]  # vertex up (+z)
        nut = mf.CrossSection([pts]).extrude(M3_NUT_DEPTH + 1).transform(
            np.column_stack([[0, 1, 0], [0, 0, 1], [1, 0, 0], [CLAMP["x"][1] - M3_NUT_DEPTH, y, z]]).astype(float))
        cut += hole + nut
    out = body - cut
    assert out.status() == mf.Error.NoError and out.genus() >= 0
    return out


def grip_sleeve_local():
    """Along local z from 0 to SLEEVE_L, 0.6 mm end chamfers (elephant's foot, the pads' lead-in)."""
    ch, r = 0.6, SLEEVE_D / 2
    s = mf.Manifold.cylinder(ch, r - ch, r, SEG) \
        + mf.Manifold.cylinder(SLEEVE_L - 2 * ch, r, r, SEG).translate((0, 0, ch)) \
        + mf.Manifold.cylinder(ch, r, r - ch, SEG).translate((0, 0, SLEEVE_L - ch))
    s -= mf.Manifold.cylinder(SLEEVE_L + 2, BORE_D / 2, BORE_D / 2, 32).translate((0, 0, -1))
    return s


def mesh_arrays(m):
    me = m.to_mesh()
    return np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts)


def write_stl(path, m):
    V, T = mesh_arrays(m)
    with open(path, "wb") as f:
        f.write(b"fsc_PegasusSimulator roll-joint valve handle, mm".ljust(80, b" "))
        f.write(struct.pack("<I", len(T)))
        for t in T:
            a, b, c = V[t]
            nrm = np.cross(b - a, c - a)
            nrm = nrm / (np.linalg.norm(nrm) or 1.0)
            f.write(struct.pack("<12fH", *nrm, *a, *b, *c, 0))


def to_json(m, digits=2):
    V, T = mesh_arrays(m)
    return {"x": V[:, 0].round(digits).tolist(), "y": V[:, 1].round(digits).tolist(), "z": V[:, 2].round(digits).tolist(),
            "i": T[:, 0].tolist(), "j": T[:, 1].tolist(), "k": T[:, 2].tolist()}


def poly_dist(A, B):
    """min distance between two convex quads in (u, v); 0 if they overlap (separating-axis test first)."""
    def edges(Pl):
        return [(Pl[i], Pl[(i + 1) % len(Pl)]) for i in range(len(Pl))]
    for Pl in (A, B):
        for a, b in edges(Pl):
            nrm = np.array([b[1] - a[1], a[0] - b[0]])
            pa, pb = A @ nrm, B @ nrm
            if pa.max() < pb.min() or pb.max() < pa.min():
                break
        else:
            continue
        break
    else:
        return 0.0
    best = np.inf
    for P1, P2 in ((A, B), (B, A)):
        for q in P1:
            for a, b in edges(P2):
                t = np.clip((q - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
                best = min(best, np.linalg.norm(q - (a + t * (b - a))))
    return best


def sc(p):
    """valve-frame point -> in-plane (s, c) about G."""
    d = p - G
    return np.array([d @ N, d @ C])


def quad(s0, s1, c0, c1):
    return np.array([[s0, c0], [s1, c0], [s1, c1], [s0, c1]])


def main():
    out = []
    say = lambda *a: (print(*a), out.append(" ".join(str(x) for x in a)))

    hb = handle_body()
    sl_local = grip_sleeve_local()
    sl = sl_local.transform(frame(P(s_sleeve[0]), N))
    v_hb, v_sl = hb.volume(), sl.volume()

    say("THE JOINT, along the axis s (mm from G, +s = up toward +y):")
    say(f"  boss Ø{BOSS_D} s {s_boss0:.1f}..{s_boss1:.1f} (nyloc pocket {s_boss0:.1f}..{s_boss0 + NYLOC_H:.1f}),"
        f" neck Ø{NECK_D} {s_boss1:.1f}..{s_neck1:.1f}, washer, sleeve Ø{SLEEVE_D} {s_sleeve[0]:.1f}..{s_sleeve[1]:.1f},"
        f" washer, head {s_head:.1f}..{s_head + HEAD_H:.1f}; M4 x {SCREW_L:.0f} tip at s {s_tip:.1f}"
        f" ({s_boss0 - s_tip:.1f} mm past the boss end, through the nylon)")
    for s, what in ((s_boss0, "boss end"), (s_sleeve[0], "sleeve -end"), (s_sleeve[1], "sleeve +end"), (s_head + HEAD_H, "screw head top")):
        p = P(s)
        say(f"  {what:15s} s {s:6.1f}: y {p[1]:6.1f}, z {p[2]:6.1f}")
    top = max(P(s_sleeve[1], -SLEEVE_D / 2)[2], P(s_head + HEAD_H, -HEAD_D / 2)[2])
    say(f"  the handle's highest point z {top:.1f} (today's fin top 204.0)")
    say(f"  smooth shank of the M4 x {SCREW_L:.0f}: s {s_head:.1f} down to {s_head - (SCREW_L - 20):.1f}; the sleeve ends at {s_sleeve[0]:.1f}"
        f" ({max(0.0, (s_head - (SCREW_L - 20)) - s_sleeve[0]):.1f} mm of it on the thread run-out)")

    say(f"\nTHE CLAMP: fin cut at z {Z_CUT:.0f} ({Z_CUT - 74:.0f} mm above the root top; {Z_CUT - 6:.0f}..{SLOT_TOP - 2:.0f} fits), slot {SLOT_W} mm x {SLOT_TOP - 74:.0f} deep,"
        f" walls {WALL} mm, x {CLAMP['x'][0]:.1f}..{CLAMP['x'][1]:.1f}, sits on the root's ledges")
    say(f"  bolts M3 at (y, z) {BOLTS}; grip length {CLAMP['x'][1] - CLAMP['x'][0]:.1f} mm -> M3 x 16 (heads on -x, nuts in 2.5 mm pockets on +x, the bolt end 2 mm proud)")

    say(f"\nPRINTED PARTS: handle body {v_hb / 1e3:.1f} cm^3 -> {RHO * v_hb:.0f} g; sleeve {v_sl / 1e3:.1f} cm^3 -> {RHO * v_sl:.0f} g")
    fin_cut = np.ptp(FIN["x"]) * np.ptp(FIN["y"]) * (FIN["z"][1] - Z_CUT)
    steel = 7.5 + 1.0 + 2 * 0.4 + 2 * 1.2 + 2 * 0.3   # M4x70, nyloc, 2 washers, 2 M3x16, 2 M3 nuts [g]
    dm = RHO * (v_hb + v_sl) + steel - RHO * fin_cut
    say(f"  the fin's top removed: {fin_cut / 1e3:.1f} cm^3 -> {RHO * fin_cut:.0f} g; screws {steel:.1f} g; net {dm:+.0f} g at r ~160 mm"
        f" -> {dm * 1e-3 * 0.16 ** 2 / 2.6e-3 * 100:+.0f} % of the rotor's 2.6e-3 kg.m^2")

    say("\nAXIS OFFSET: the sleeve is centred on the axis through G, so the claw point is on it by construction;"
        f" a claw sitting d off it costs K_y d^2 (10 mm -> {K_Y * 0.01 ** 2 * 1e3:.0f} mN.m/rad)")

    # clearances in the fin plane (fingers straddle x = 160)
    fing = lambda e: quad(-FINGER_W / 2 + e[0], FINGER_W / 2 + e[0], -FINGER_BACK + e[1], FINGER_TIP + e[1])
    body = lambda e: quad(-15 + e[0], 15 + e[0], -80 + e[1], -40 + e[1])
    boss = quad(s_boss0, s_boss1, -BOSS_D / 2, BOSS_D / 2)
    clamp_top = np.array([sc(np.array([160, y, z])) for y, z in ((CLAMP["y"][0], 0), (CLAMP["y"][1], 0), (CLAMP["y"][1], CLAMP["z"][1]), (CLAMP["y"][0], CLAMP["z"][1]))])
    say("\nCLEARANCES (fin plane; the closed pads touch only what is about as wide as the sleeve):")
    say(f"  the pads reach the Ø{BOSS_D} boss only with the claw {-(s_boss1 + FINGER_W / 2):.1f} mm off along the axis toward it"
        f" (the close gate allows {GATE:.0f}); the neck Ø{NECK_D}, washers Ø{WASHER_D}, head Ø{HEAD_D}, web {WEB_T} mm are all thinner than the sleeve")
    say(f"  jaw side clearance at the close: ({JAW_OPEN:.0f} - {SLEEVE_D:.0f}) / 2 = {(JAW_OPEN - SLEEVE_D) / 2:.1f} mm (close gate across {GATE_ACROSS:.0f})")
    for err in (0.0, 10.0, GATE):
        dc = dbo = np.inf
        for a in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            e = err * np.array([np.cos(a), np.sin(a)])
            dc = min(dc, poly_dist(fing(e), clamp_top))
            dbo = min(dbo, poly_dist(fing(e), boss))
        say(f"  claw {err:4.1f} mm off G (worst direction): fingers to the clamp {dc:5.1f} mm, fingers' footprint to the boss {dbo:5.1f} mm")
    ap = sc(G + APPROACH)
    for err in (0.0, GATE):
        db = np.inf
        for a in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            e = err * np.array([np.cos(a), np.sin(a)])
            for t in np.linspace(0, 1, 26):
                db = min(db, poly_dist(body(e + t * ap), boss), poly_dist(body(e + t * ap), clamp_top))
        say(f"  gripper body to the boss/clamp all the way in, claw {err:4.1f} mm off: {db:5.1f} mm (open fingers pass the clamp, {np.ptp(CLAMP['x']):.1f} mm wide, with"
            f" {(JAW_OPEN - np.ptp(CLAMP['x'])) / 2:.1f} mm a side)")
    E = np.array([192.5, 10.7, 92.5])
    nut_face = np.array([CLAMP["x"][1], BOLTS[1][0], BOLTS[1][1]])
    say(f"  M3 nut face (+x) to marker E's ball: {np.linalg.norm(nut_face - E) - 15 - M3_NUT_AF / 2:.1f} mm")

    say("\nLOADS at the 5 N force guard (push along +y at G):")
    F = 5.0
    lever = -(s_boss1) * 1e-3
    M = F * lever
    say(f"  along the axis {F * N[1]:.2f} N (pulls the sleeve against the washer under the head), across {F * N[2]:.2f} N")
    say(f"  M4 screw bending at the boss face: {M:.3f} N.m -> {32 * M / (np.pi * 3.14e-3 ** 3) / 1e6:.0f} MPa on the thread core")
    m_in = F * (G[2] - np.mean(BOLTS, axis=0)[1]) * 1e-3
    say(f"  in-plane moment at the bolts {m_in:.3f} N.m -> {m_in / (abs(BOLTS[1][0] - BOLTS[0][0]) * 1e-3):.0f} N per M3 bolt in shear")
    Z = (CLAMP["y"][1] - 3 - CLAMP["y"][0] - 0) * WEB_T ** 2 / 6
    say(f"  the same 5 N radially (claw sway) bends the web at the clamp: {F * (G[2] - CLAMP['z'][1]) / Z:.1f} MPa")
    for mu, tag in ((0.25, "dry"), (0.1, "greased")):
        r_ax = (WASHER_D / 2 + BORE_D / 2) / 2
        t_turn = mu * (0.45 * N[2] * 2.0 + 0.45 * N[1] * r_ax) * 1e-3
        t_guard = mu * (F * N[2] * 2.0 + F * N[1] * r_ax) * 1e-3
        say(f"  joint friction ({tag}, mu {mu}): {t_turn * 1e3:.2f} mN.m in the turn (0.45 N push), {t_guard * 1e3:.1f} mN.m at the force guard")
    say("  locked (screw tightened, ~300-600 N clamp): 0.3 x F x 3.3 mm x 2 faces = "
        f"{0.3 * 300 * 3.3e-3 * 2:.1f}-{0.3 * 600 * 3.3e-3 * 2:.1f} N.m of holding")

    os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
    lo = np.array(hb.bounding_box()[:3])
    write_stl(os.path.join(OUT, "stl", "handle_body.stl"), hb.translate((-160.0, -float(G[1]), -float(lo[2]))))
    write_stl(os.path.join(OUT, "stl", "grip_sleeve.stl"), sl_local)
    bb = hb.bounding_box()
    say(f"\nSTL: handle_body.stl {bb[3] - bb[0]:.1f} x {bb[4] - bb[1]:.1f} x {bb[5] - bb[2]:.1f} mm upright (clamp down),"
        f" {hb.num_tri()} triangles; grip_sleeve.stl Ø{SLEEVE_D} x {SLEEVE_L} on its end, {sl_local.num_tri()} triangles")

    # the page's 3D: the parts and the screws in the valve frame
    screw = cyl_along(s_tip, s_head, 4.0, 20) + cyl_along(s_head, s_head + HEAD_H, HEAD_D, 24)
    washers = cyl_along(s_sleeve[1], s_head, WASHER_D, 24) + cyl_along(s_neck1, s_sleeve[0], WASHER_D, 24)
    m3 = None
    for y, z in BOLTS:
        b = mf.Manifold.cylinder(16, 1.5, 1.5, 12).transform(frame(np.array([CLAMP["x"][0], y, z]), X, np.array([0, 1.0, 0]))) \
            + mf.Manifold.cylinder(3, 2.75, 2.75, 16).transform(frame(np.array([CLAMP["x"][0] - 3, y, z]), X, np.array([0, 1.0, 0])))
        m3 = b if m3 is None else m3 + b
    data = {"handle": to_json(hb), "sleeve": to_json(sl), "screw": to_json(screw), "washers": to_json(washers), "m3": to_json(m3),
            "s": {"boss0": s_boss0, "boss1": s_boss1, "neck1": s_neck1, "sleeve0": s_sleeve[0], "sleeve1": s_sleeve[1],
                  "head": s_head, "tip": s_tip},
            "web": [[[round(float(a), 2), round(float(b), 2)] for a, b in pl] for pl in web_section().to_polygons()], "z_cut": Z_CUT, "clamp": CLAMP, "bolts": BOLTS, "slot_top": SLOT_TOP}
    with open(os.path.join(OUT, "roll_grip_parts.json"), "w") as f:
        json.dump(data, f, separators=(",", ":"))
    with open(os.path.join(OUT, "roll_grip_design.txt"), "w") as f:
        f.write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
