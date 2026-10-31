#!/usr/bin/env python3
"""The roll-fin handle (2026-10-31, third version): printable parts + the numbers behind them.

User request: fewer parts, fewer printed parts, and a FIN that stands vertical before the grasp.
Today's fin is sawn off 30 mm above its root. A printed BASE clamps over the stump with two M3 bolts
and holds an M4 screw along the roll axis (in the fin's plane, 30 deg above horizontal, rising toward
+y, through the grip point G). A printed ROLL FIN, a flat 8 mm plate with a bore along that axis, turns
on the screw. The claw grips its flat faces at G, as it grips today's fin. An O-ring between the fin
and the base, squeezed by the screw, holds the fin by friction: that keeps it standing where it was set
(vertical) until the claw takes it, and is small enough not to matter once it does. Tightened fully,
the same screw locks the fin (the locked/free comparison).

Two printed parts is the floor: a joint needs one part that turns and one that holds the screw on the
30-deg line, and the 5 mm stump cannot hold a screw along a line in its own plane.

Frame: the valve's (mm): origin on the bearing axis at the plate bottom, +x along the beam to the handle,
+z up; the turn is +90 deg, so the claw's push is +y. Geometry from Valve_Assembly_v2.stl (checked in
check_on_model.py): fin x 157.5..162.5, y -19.4..20.6, z 74..204; root x 155..165, y -24.4..25.6,
z 54..74. G = push_pull_ee_offset (0.160, 0.0006, 0.179) m. Claw axis at the flown pose [0, 5, 25, 0]:
along +y and 60 deg down (handle_joint_mobility.py).

Writes, next to this tool's folder:
  stl/handle_base.stl, stl/roll_fin.stl   print orientation, mm (base upright, fin flat on its face)
  roll_grip_parts.json                    the parts in the valve frame, for the page's 3D view
  roll_grip_design.txt                    this run's printout

    PYTHONPATH=~/sources/pymanifold /usr/bin/python3 roll_grip_design.py

manifold3d 3.5.4 lives in ~/sources/pymanifold (pip install --no-deps --target; no venv module here).
The second version (a round sleeve) is kept as roll_grip_sleeve_design.py.
"""
import json
import os
import struct

import numpy as np
import manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
SEG = 64

G = np.array([160.0, 0.6, 179.0])
N = np.array([0.0, np.cos(np.radians(30)), np.sin(np.radians(30))])    # the roll axis
C = np.array([0.0, 0.5, -np.sqrt(3) / 2])                             # in the fin's plane, square to N (down, +y)
X = np.array([1.0, 0.0, 0.0])
K_Y = 120.0                                        # [N/m] the law's EE task stiffness (valve yaml)
GATE, GATE_ACROSS = 20.0, 8.0                      # [mm] the Valve tab's hardware close gate
FINGER_W, FINGER_BACK, FINGER_TIP = 20.0, 40.0, 20.0   # assumed OM-X finger (as handle_design.py)
PAD_C = (-15.0, 15.0)                              # assumed pad contact band along C about the claw point
JAW_OPEN = 37.0                                    # [mm] measured open gap at the fingertips
APPROACH = np.array([0.0, -120.0, 100.0])          # Ready starts 0.12 m behind, 0.10 m above
RHO = 1.04e-3                                      # [g/mm^3] printed PLA, as the printed rotor/payload weigh

FIN = dict(x=(157.5, 162.5), y=(-19.4, 20.6), z=(74.0, 204.0))     # today's fin
ROOT = dict(x=(155.0, 165.0), y=(-24.4, 25.6), z=(54.0, 74.0))
Z_CUT = 104.0                                      # 30 mm above the root top; anything 98..106 fits

# the base: a clamp over the stump, a web, a neck and a boss on the axis
SLOT_W = 5.4                                       # stump 5.0 + 0.2 a side
WALL = 4.0
CLAMP = dict(x=(160 - SLOT_W / 2 - WALL, 160 + SLOT_W / 2 + WALL), y=(FIN["y"][0] - 1.0, FIN["y"][1] + 1.0), z=(74.0, 114.0))
SLOT_TOP = 108.0
BOLTS = [(-12.0, 89.0), (12.0, 89.0)]              # (y, z): M3 x 16, heads on -x, nuts in pockets on +x
M3_HOLE, M3_NUT_AF, M3_NUT_DEPTH = 3.3, 5.8, 2.5
WEB_T, WEB_GAP = 5.0, 3.0                          # web = today's fin thickness; 3 mm under the roll fin

# the joint, along the axis (s = mm along N from G)
FIN_T = 8.0                                        # the roll fin's thickness (the jaws close on it)
FIN_S, FIN_C, FIN_R = (-34.0, 22.0), (-22.0, 30.0), 4.0   # its outline in (s, c), corner radius
# (2026-10-31, from the three hardware grasps: the jaws closed 10, 22 and 13 mm SHORT along the axis, toward
# the side the claw comes from (-s), and within 4 mm along the claw: the fin runs 34 mm to -s, 22 to +s, and
# 22 mm above G along the claw -- 8 mm more than the first cut, as today's fin had 23-35)
BORE_D = 4.4                                       # clearance on the M4 shank: a plain bearing
ORING_ID, ORING_CS, ORING_SQUEEZED = 4.0, 1.0, 0.8 # NBR O-ring 4 x 1 between the neck and the fin
HEAD_D, HEAD_H, SCREW_L = 7.0, 4.0, 70.0           # M4 x 70 ISO 4762 socket head
NECK_D, NECK_L = 6.0, 6.0                          # thinner than the fin: the pads cannot grip it
BOSS_D, BOSS_L, PILOT_D = 9.0, 7.0, 3.5            # the M4 cuts its own thread in the Ø3.5 pilot

s_neck1 = FIN_S[0] - ORING_SQUEEZED                # the O-ring sits between the neck's face and the fin
s_boss1 = s_neck1 - NECK_L
s_boss0 = s_boss1 - BOSS_L
s_head = FIN_S[1]                                  # the head bears on the fin's other end
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


def box(x, y, z):
    return mf.Manifold.cube((x[1] - x[0], y[1] - y[0], z[1] - z[0])).translate((x[0], y[0], z[0]))


def ccw(poly):
    a = 0.5 * sum(p[0] * q[1] - q[0] * p[1] for p, q in zip(poly, poly[1:] + poly[:1]))
    return poly if a > 0 else poly[::-1]


def yz(p):
    return (float(p[1]), float(p[2]))


def hexagon(af, vertex_up=True):
    r = af / np.sqrt(3)
    a0 = 90 if vertex_up else 0
    return [(r * np.cos(np.radians(a0 + 60 * k)), r * np.sin(np.radians(a0 + 60 * k))) for k in range(6)]


# the fin plane's (y, z) coordinates extruded along x
PLANE = lambda x0: np.column_stack([[0, 1, 0], [0, 0, 1], [1, 0, 0], [x0, 0, 0]]).astype(float)
# the (s, c) plane extruded along x, s along N and c along C, origin at G
SC = lambda x0: np.column_stack([N, C, X, G + (x0 - G[0]) * X]).astype(float)


def web_polygon():
    zb = CLAMP["z"][1] - 2.0
    yr = CLAMP["y"][1] - 3.0
    top = FIN_C[1] + WEB_GAP                       # the web stays this far below the roll fin
    s_r = (yr - G[1] - top * C[1]) / N[1]
    s_step = FIN_S[0] - 1.0
    return ccw([(CLAMP["y"][0], zb), (yr, zb), yz(P(s_r, top)), yz(P(s_step, top)),
                yz(P(s_step, 0.0)), yz(P(s_boss0, 0.0)), yz(P(s_boss0, BOSS_D / 2))])


def handle_base():
    web = mf.CrossSection([web_polygon()]).extrude(WEB_T).transform(PLANE(160 - WEB_T / 2))
    body = box(CLAMP["x"], CLAMP["y"], CLAMP["z"]) + web + cyl_along(s_boss0, s_boss1, BOSS_D) + cyl_along(s_boss1, s_neck1, NECK_D)
    cut = box((160 - SLOT_W / 2, 160 + SLOT_W / 2), (-60, 60), (60, SLOT_TOP))
    cut += cyl_along(s_boss0 - 2, s_boss1 + 0.01, PILOT_D, 32)       # the M4 cuts its thread in the boss only
    cut += cyl_along(s_boss1, s_neck1 + 1, BORE_D, 32)               # clearance through the neck
    for y, z in BOLTS:
        cut += mf.Manifold.cylinder(40, M3_HOLE / 2, M3_HOLE / 2, 32).transform(frame(np.array([140.0, y, z]), X, np.array([0, 1.0, 0])))
        cut += mf.CrossSection([hexagon(M3_NUT_AF)]).extrude(M3_NUT_DEPTH + 1).transform(
            np.column_stack([[0, 1, 0], [0, 0, 1], [1, 0, 0], [CLAMP["x"][1] - M3_NUT_DEPTH, y, z]]).astype(float))
    out = body - cut
    assert out.status() == mf.Error.NoError
    return out


def fin_section():
    """The roll fin's outline in (s, c), rounded corners."""
    s0, s1 = FIN_S
    c0, c1 = FIN_C
    sq = mf.CrossSection([[(s0, c0), (s1, c0), (s1, c1), (s0, c1)]])
    return sq.offset(-FIN_R, mf.JoinType.Round).offset(FIN_R, mf.JoinType.Round)


def roll_fin():
    fin = fin_section().extrude(FIN_T).transform(SC(160 - FIN_T / 2))
    fin -= cyl_along(FIN_S[0] - 2, FIN_S[1] + 2, BORE_D, 32)
    assert fin.status() == mf.Error.NoError
    return fin


def m4_screw():
    """M4 x 70 socket head, along the axis: shank to the tip, head with a hex socket at its top."""
    head = cyl_along(s_head, s_head + HEAD_H, HEAD_D, 32)
    sock = mf.CrossSection([hexagon(3.0, False)]).extrude(2.5).transform(frame(P(s_head + HEAD_H - 2.5), N))
    return cyl_along(s_tip, s_head, 4.0, 24) + head - sock


def o_ring():
    r_mid = (ORING_ID + ORING_CS) / 2
    ring = mf.CrossSection.circle(ORING_CS / 2, 24).translate((r_mid, 0)).revolve(48)    # a torus about local z
    return ring.translate((0, 0, ORING_CS / 2)).transform(frame(P(s_neck1), N))


def m3_bolts():
    out = None
    for y, z in BOLTS:
        b = mf.Manifold.cylinder(16, 1.5, 1.5, 16).transform(frame(np.array([CLAMP["x"][0], y, z]), X, np.array([0, 1.0, 0]))) \
            + mf.Manifold.cylinder(3, 2.75, 2.75, 24).transform(frame(np.array([CLAMP["x"][0] - 3, y, z]), X, np.array([0, 1.0, 0])))
        out = b if out is None else out + b
    return out


def m3_nuts():
    out = None
    for y, z in BOLTS:
        n = mf.CrossSection([hexagon(5.5)]).extrude(2.4).transform(
            np.column_stack([[0, 1, 0], [0, 0, 1], [1, 0, 0], [CLAMP["x"][1] - 2.45, y, z]]).astype(float)) \
            - mf.Manifold.cylinder(10, 1.5, 1.5, 16).transform(frame(np.array([CLAMP["x"][1] - 5, y, z]), X, np.array([0, 1.0, 0])))
        out = n if out is None else out + n
    return out


def mesh_arrays(m):
    me = m.to_mesh()
    return np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts)


def write_stl(path, m, scale=1.0, shift=np.zeros(3)):
    V, T = mesh_arrays(m)
    V = (V + shift) * scale
    with open(path, "wb") as f:
        f.write(b"fsc_PegasusSimulator roll-fin valve handle".ljust(80, b" "))
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
    """min distance between two convex polygons in (u, v); 0 if they overlap."""
    def edges(Pl):
        return [(Pl[i], Pl[(i + 1) % len(Pl)]) for i in range(len(Pl))]
    sep = False
    for Pl in (A, B):
        for a, b in edges(Pl):
            nrm = np.array([b[1] - a[1], a[0] - b[0]])
            pa, pb = A @ nrm, B @ nrm
            if pa.max() < pb.min() or pb.max() < pa.min():
                sep = True
    if not sep:
        return 0.0
    best = np.inf
    for P1, P2 in ((A, B), (B, A)):
        for q in P1:
            for a, b in edges(P2):
                t = np.clip((q - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
                best = min(best, np.linalg.norm(q - (a + t * (b - a))))
    return best


def sc(p):
    d = p - G
    return np.array([d @ N, d @ C])


def quad(s0, s1, c0, c1):
    return np.array([[s0, c0], [s1, c0], [s1, c1], [s0, c1]])


def main():
    out = []
    say = lambda *a: (print(*a), out.append(" ".join(str(x) for x in a)))
    base, fin = handle_base(), roll_fin()
    screw, ring, bolts, nuts = m4_screw(), o_ring(), m3_bolts(), m3_nuts()
    v_b, v_f = base.volume(), fin.volume()

    say("THE JOINT, along the axis s (mm from G, +s = up toward +y):")
    say(f"  boss Ø{BOSS_D:.0f} s {s_boss0:.1f}..{s_boss1:.1f} (Ø{PILOT_D} pilot: the M4 cuts its thread), neck Ø{NECK_D:.0f} {s_boss1:.1f}..{s_neck1:.1f},"
        f" O-ring {ORING_ID:.0f} x {ORING_CS:.0f} squeezed to {ORING_SQUEEZED}, roll fin {FIN_S[0]:.0f}..{FIN_S[1]:.0f} (bore Ø{BORE_D}),"
        f" head {s_head:.1f}..{s_head + HEAD_H:.1f}; M4 x {SCREW_L:.0f} tip at s {s_tip:.1f} ({s_boss0 - s_tip:.1f} mm past the boss)")
    bb = np.array(fin.bounding_box())
    say(f"  the roll fin: {np.ptp(FIN_S):.0f} along the axis x {np.ptp(FIN_C):.0f} across x {FIN_T:.0f} thick; in the valve frame"
        f" y {bb[1]:.1f}..{bb[4]:.1f}, z {bb[2]:.1f}..{bb[5]:.1f} (today's fin top 204)")
    cg_c = (FIN_C[0] + FIN_C[1]) / 2
    m_f = RHO * v_f
    k_g = m_f * 1e-3 * 9.81 * cg_c * 1e-3 * N[1]
    say(f"  its centre of mass {cg_c:.0f} mm below the axis: gravity alone centres it with {k_g * 1e3:.1f} mN.m/rad (weak: the O-ring holds it)")

    say(f"\nTHE CLAMP: fin cut at z {Z_CUT:.0f} ({Z_CUT - 74:.0f} mm above the root top), slot {SLOT_W} x {SLOT_TOP - 74:.0f} deep, walls {WALL} mm,"
        f" x {CLAMP['x'][0]:.1f}..{CLAMP['x'][1]:.1f}; M3 x 16 at (y, z) {BOLTS}, nuts in 2.5 mm pockets on +x")

    fin_cut = np.ptp(FIN["x"]) * np.ptp(FIN["y"]) * (FIN["z"][1] - Z_CUT)
    steel = 6.6 + 0.1 + 2 * 1.2 + 2 * 0.3
    dm = RHO * (v_b + v_f) + steel - RHO * fin_cut
    say(f"\nPRINTED: base {v_b / 1e3:.1f} cm^3 -> {RHO * v_b:.0f} g; roll fin {v_f / 1e3:.1f} cm^3 -> {m_f:.0f} g."
        f" Fin top removed {RHO * fin_cut:.0f} g, screws {steel:.1f} g: net {dm:+.0f} g at r ~160 mm"
        f" -> {dm * 1e-3 * 0.16 ** 2 / 2.6e-3 * 100:+.0f} % of the rotor's 2.6e-3 kg.m^2")

    # the hold before the grasp and the coupling after it
    say("\nTHE HOLD (O-ring between the neck and the fin, the screw head on the fin's other end):")
    r_or, r_head = (ORING_ID + ORING_CS) / 2 * 1e-3, 0.5 * (HEAD_D / 2 + BORE_D / 2) * 1e-3
    for F in (0.5, 1.0, 2.0):
        t = F * (0.6 * r_or + 0.25 * r_head)
        say(f"  squeeze {F:.1f} N -> friction {t * 1e3:.1f} mN.m (rubber 0.6 at the O-ring, steel on PLA 0.25 under the head)")
    for Fa, tag in ((0.45 * N[1], "the turn's 0.45 N push"), (5.0 * N[1], "the 5 N force guard")):
        say(f"  {tag} pulls the fin {Fa:.2f} N up the screw: off the O-ring, onto the head -> at most {0.25 * (1.0 + Fa) * r_head * 1e3:.1f} mN.m"
            " (from a 1 N squeeze)")
    say("  aim: 2-4 mN.m. Holds the fin wherever it is set; today's fixed fin passed ~70 mN.m rms on 10-29 (20-35x more)")
    t_w = 0.010 * 9.81 * (FIN_C[1] - 3) * 1e-3
    say(f"  bench setting: a 10 g weight taped to the fin's bottom edge ({FIN_C[1] - 3:.0f} mm from the axis) = {t_w * 1e3:.1f} mN.m: it should just turn the fin")

    # clearances in the fin plane
    fing = lambda e: quad(-FINGER_W / 2 + e[0], FINGER_W / 2 + e[0], -FINGER_BACK + e[1], FINGER_TIP + e[1])
    body = lambda e: quad(-15 + e[0], 15 + e[0], -80 + e[1], -40 + e[1])
    bossq = quad(s_boss0, s_boss1, -BOSS_D / 2, BOSS_D / 2)
    clamp_top = np.array([sc(np.array([160, y, z])) for y, z in ((CLAMP["y"][0], 0), (CLAMP["y"][1], 0), (CLAMP["y"][1], CLAMP["z"][1]), (CLAMP["y"][0], CLAMP["z"][1]))])
    say("\nCLEARANCES (fin plane):")
    say(f"  pads on the fin: {np.ptp(FIN_S):.0f} mm along the axis for {FINGER_W:.0f} mm pads -> slop {-FIN_S[0] - FINGER_W / 2:.0f} mm toward -s (the measured misses: 10, 22, 13), {FIN_S[1] - FINGER_W / 2:.0f} toward +s;"
        f" across: fin c {FIN_C[0]:.0f}..{FIN_C[1]:.0f} vs pad band {PAD_C[0]:.0f}..{PAD_C[1]:.0f}")
    say(f"  the pads reach the Ø{BOSS_D:.0f} boss only with the claw {-(s_boss1 + FINGER_W / 2):.1f} mm off along the axis toward it"
        f" (close gate {GATE:.0f}); the neck (Ø{NECK_D:.0f}) and the screw head (Ø{HEAD_D:.0f}) are thinner than the fin ({FIN_T:.0f})")
    say(f"  jaw side clearance at the close: ({JAW_OPEN:.0f} - {FIN_T:.0f}) / 2 = {(JAW_OPEN - FIN_T) / 2:.1f} mm (close gate across {GATE_ACROSS:.0f})")
    say(f"  fingertips (c +{FINGER_TIP:.0f}) to the web (c +{FIN_C[1] + WEB_GAP:.0f}): {FIN_C[1] + WEB_GAP - FINGER_TIP:.0f} mm with the claw on G")
    for err in (0.0, 10.0, GATE):
        dc = dbo = np.inf
        for a in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            e = err * np.array([np.cos(a), np.sin(a)])
            dc = min(dc, poly_dist(fing(e), clamp_top))
            dbo = min(dbo, poly_dist(fing(e), bossq))
        say(f"  claw {err:4.1f} mm off G (worst direction): fingers to the clamp {dc:5.1f} mm, fingers' footprint to the boss {dbo:5.1f} mm")
    ap = sc(G + APPROACH)
    for err in (0.0, GATE):
        db = np.inf
        for a in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            e = err * np.array([np.cos(a), np.sin(a)])
            for t in np.linspace(0, 1, 26):
                db = min(db, poly_dist(body(e + t * ap), bossq), poly_dist(body(e + t * ap), clamp_top))
        say(f"  gripper body to the boss/clamp all the way in, claw {err:4.1f} mm off: {db:5.1f} mm")
    th = np.radians(30)
    say(f"  the fin turned 30 deg: its bottom edge moves {FIN_C[1] * np.sin(th):.0f} mm sideways and {FIN_C[1] * (1 - np.cos(th)):.0f} mm toward the axis,"
        f" away from the web; the open jaws ({JAW_OPEN:.0f} mm) still take it and squeeze it back flat")

    say("\nLOADS at the 5 N force guard (push along +y at G):")
    F = 5.0
    M = F * (-s_boss1) * 1e-3
    say(f"  along the axis {F * N[1]:.2f} N (pulls the fin against the screw head), across {F * N[2]:.2f} N")
    say(f"  M4 screw bending at the boss: {M:.3f} N.m -> {32 * M / (np.pi * 3.14e-3 ** 3) / 1e6:.0f} MPa on the thread core")
    m_in = F * (G[2] - np.mean(BOLTS, axis=0)[1]) * 1e-3
    say(f"  in-plane moment at the bolts {m_in:.3f} N.m -> {m_in / (abs(BOLTS[1][0] - BOLTS[0][0]) * 1e-3):.0f} N per M3 bolt in shear")
    say(f"  the M4 thread in the PLA pilot: {BOSS_L:.0f} mm engaged, the axial load {F * N[1]:.1f} N at most")

    os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
    lo = np.array(base.bounding_box()[:3])
    write_stl(os.path.join(OUT, "stl", "handle_base.stl"), base.translate((-160.0, -float(G[1]), -float(lo[2]))))
    fin_flat = fin_section().extrude(FIN_T) - mf.Manifold.cylinder(np.ptp(FIN_S) + 4, BORE_D / 2, BORE_D / 2, 32).transform(
        np.column_stack([[0, 1, 0], [0, 0, 1], [1, 0, 0], [FIN_S[0] - 2, 0, FIN_T / 2]]).astype(float))
    write_stl(os.path.join(OUT, "stl", "roll_fin.stl"), fin_flat.translate((-FIN_S[0], -FIN_C[0], 0)))
    bb = base.bounding_box()
    say(f"\nSTL: handle_base.stl {bb[3] - bb[0]:.1f} x {bb[4] - bb[1]:.1f} x {bb[5] - bb[2]:.1f} mm upright;"
        f" roll_fin.stl {np.ptp(FIN_S):.0f} x {np.ptp(FIN_C):.0f} x {FIN_T:.0f} mm flat on its face (the bore runs along the {np.ptp(FIN_S):.0f} mm side)")

    data = {"base": to_json(base), "fin": to_json(fin), "screw": to_json(screw), "oring": to_json(ring),
            "m3": to_json(bolts), "nuts": to_json(nuts),
            "s": {"boss0": s_boss0, "boss1": s_boss1, "neck1": s_neck1, "fin0": FIN_S[0], "fin1": FIN_S[1],
                  "head": s_head, "tip": s_tip},
            "fin_c": FIN_C, "fin_t": FIN_T, "fin_r": FIN_R,
            "web": [[[round(float(a), 2), round(float(b), 2)] for a, b in pl] for pl in mf.CrossSection([web_polygon()]).to_polygons()],
            "z_cut": Z_CUT, "clamp": CLAMP, "bolts": BOLTS, "slot_top": SLOT_TOP,
            "mass": {"base": round(RHO * v_b), "fin": round(m_f), "net": round(dm)}}
    with open(os.path.join(OUT, "roll_grip_parts.json"), "w") as f:
        json.dump(data, f, separators=(",", ":"))
    with open(os.path.join(OUT, "roll_grip_design.txt"), "w") as f:
        f.write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
