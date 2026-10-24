#!/usr/bin/env python3
"""Build the mail-delivery DESIGN PAGE (2026-10-23): one HTML page with every
dimension, mass, position and margin of the experiment, and to-scale drawings
generated from mail_geometry.py's constants (so the page cannot disagree with
the tool, the scene or the yaml).

    PYTHONNOUSERSITE=1 /usr/bin/python3 build_design_page.py [out.html]

Default output: ../design_page.html (published as an artifact with
../figures/isaac_scene.png and ../shots/gs_live_mail_tab.png beside it).
"""
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mail_geometry as G                                   # noqa: E402

TP = G.TP
P = TP.make_params_t650()
Q = np.radians(G.POSE_DEG)

# ── derived geometry (the same expressions as mail_geometry.main) ─────────────
_, R0E, RE = TP.arm_fk_model(Q, P)
FWD, LAT, DZ = R0E[1], -R0E[0], R0E[2]
PITCH = math.degrees(math.asin(-RE[:, 2][2]))               # claw direction, + = up
SIGMA = TP._sigma_nd(Q, P)


def arm_points():
    """(forward, up) of every chain point relative to the body origin at the pose."""
    import compatible_trajectory as CTJ
    R = [np.eye(3)]
    for i in range(P["n"]):
        R.append(R[i] @ CTJ._rot(P["h_i_im1"][i], Q[i]))
    O = [np.zeros(3)]
    for i in range(1, P["n"] + 1):
        O.append(O[i - 1] + R[i] @ P["l_i"][i - 1])
    return [(float(o[1]), float(o[2])) for o in O]


ARM = arm_points()
HAT_TOP = G.PILLAR_TOP + G.HAT_ABOVE
RIM_X = G.PILLAR_XY[0] - G.HAT_RADIUS
NEAR_X = RIM_X - G.OVERHANG
ENV_C = (NEAR_X + 0.5 * G.ENV_L, G.PILLAR_XY[1], HAT_TOP + 0.5 * G.ENV_T)
CLAW_PICK = (NEAR_X + G.GRASP_IN, G.PILLAR_XY[1], ENV_C[2])
BODY_PICK = (CLAW_PICK[0] - FWD, CLAW_PICK[1] - LAT, CLAW_PICK[2] - DZ)
CLAW_REL = (G.TABLE_FRONT_X - G.JAW_CLEAR - G.JAW_TIP_BEYOND_CLAW, G.RED_Y, G.Z_CLAW_PLACE)
BODY_REL = (CLAW_REL[0] - FWD, CLAW_REL[1] - LAT, CLAW_REL[2] - DZ)
PLACE_POINT = (CLAW_REL[0] + 0.5 * G.ENV_L - G.GRASP_IN, G.RED_Y, G.Z_CLAW_PLACE)
BOX_TOP = G.TABLE_TOP + G.BOX_H
MOUTH_LO = G.TABLE_TOP + G.MOUTH_Z - 0.5 * G.MOUTH_H
MOUTH_HI = G.TABLE_TOP + G.MOUTH_Z + 0.5 * G.MOUTH_H
MOUTH_C = G.TABLE_TOP + G.MOUTH_Z
ROTOR_XY = 0.22990663                    # rotor centres (+-x, +-y) of the T650 / X650 frame
PROP_R = 0.1905                          # 15 in props
SKID_X = 0.16                            # skids +-0.16 m fore-aft of the body origin
LAND_START = (0.0, 0.0, 1.0)
MAILBOX_START = (1.0, G.RED_Y, 1.1)
RTP_BODY = (BODY_PICK[0] - 0.10, BODY_PICK[1], BODY_PICK[2])
RTI_BODY = (BODY_REL[0] - G.INSERT_MARGIN, BODY_REL[1], BODY_REL[2])
EXIT_BODY = (BODY_REL[0] - 0.10, BODY_REL[1], BODY_REL[2] + 0.20)
GRIP_TOL_MM = 20
GRIP_VERT_TOL_MM = 8

# The tool's margin numbers (same formulas, printed by mail_geometry.py)
S_TIP = G.ENV_L - G.GRASP_IN
S_WALL = (G.TABLE_FRONT_X - CLAW_REL[0]) / math.cos(math.radians(-PITCH))
TIP_X10 = CLAW_REL[0] + S_TIP * math.cos(math.radians(-PITCH))
COM_X10 = CLAW_REL[0] + (S_TIP - G.ENV_COM_FROM_FAR) * math.cos(math.radians(-PITCH))
M_BACK = (G.TABLE_FRONT_X + G.BOX_D - G.WALL) - TIP_X10
M_LIP = COM_X10 - (G.TABLE_FRONT_X + G.WALL)
PROPS_OVER = BODY_REL[2] + G.PROP_BOTTOM_ABOVE_BODY - BOX_TOP
BAND_LO = -S_TIP * math.sin(math.radians(G.PITCH_RANGE_DEG[1])) - 0.5 * G.ENV_T
BAND_HI = max(-S_WALL * math.sin(math.radians(G.PITCH_RANGE_DEG[1])), 0.0) + 0.5 * G.ENV_T + G.MARKER_H
MOUTH_REL_LO = MOUTH_LO - G.Z_CLAW_PLACE
MOUTH_REL_HI = MOUTH_HI - G.Z_CLAW_PLACE

# ── SVG helpers ──────────────────────────────────────────────────────────────
_mid = [0]


def f(v):
    return f"{v:.1f}"


class Fig:
    """World (metres) -> SVG px. kind 'side': (x, z); 'plan': (x, y); 'front': (u, z)."""

    def __init__(self, x0, x1, y0, y1, scale, pad=(40, 30, 40, 30)):
        self.x0, self.y1, self.s = x0, y1, scale
        self.pl, self.pt, self.pr, self.pb = pad
        self.w = (x1 - x0) * scale + self.pl + self.pr
        self.h = (y1 - y0) * scale + self.pt + self.pb
        _mid[0] += 1
        self.id = f"m{_mid[0]}"
        self.parts = []

    def X(self, x):
        return self.pl + (x - self.x0) * self.s

    def Y(self, y):
        return self.pt + (self.y1 - y) * self.s

    def add(self, s):
        self.parts.append(s)

    def rect(self, x0, y0, x1, y1, cls, extra=""):
        X0, X1 = sorted((self.X(x0), self.X(x1)))
        Y0, Y1 = sorted((self.Y(y0), self.Y(y1)))
        self.add(f'<rect class="{cls}" x="{f(X0)}" y="{f(Y0)}" width="{f(X1 - X0)}" height="{f(Y1 - Y0)}"{extra}/>')

    def line(self, x0, y0, x1, y1, cls, extra=""):
        self.add(f'<line class="{cls}" x1="{f(self.X(x0))}" y1="{f(self.Y(y0))}" x2="{f(self.X(x1))}" '
                 f'y2="{f(self.Y(y1))}"{extra}/>')

    def poly(self, pts, cls, closed=False, extra=""):
        tag = "polygon" if closed else "polyline"
        p = " ".join(f"{f(self.X(x))},{f(self.Y(y))}" for x, y in pts)
        self.add(f'<{tag} class="{cls}" points="{p}"{extra}/>')

    def circle(self, x, y, r_m, cls, r_px=None):
        r = r_px if r_px is not None else r_m * self.s
        self.add(f'<circle class="{cls}" cx="{f(self.X(x))}" cy="{f(self.Y(y))}" r="{f(r)}"/>')

    def text(self, x, y, s, cls="t", anchor="middle", dx=0, dy=0):
        self.add(f'<text class="{cls}" x="{f(self.X(x) + dx)}" y="{f(self.Y(y) + dy)}" text-anchor="{anchor}">{s}</text>')

    def arrow(self, x0, y0, x1, y1, cls="route", label=None, ldx=0, ldy=-6, lcls="t", anchor="middle"):
        self.add(f'<line class="{cls}" x1="{f(self.X(x0))}" y1="{f(self.Y(y0))}" x2="{f(self.X(x1))}" '
                 f'y2="{f(self.Y(y1))}" marker-end="url(#{self.id}r)"/>')
        if label:
            self.text(0.5 * (x0 + x1), 0.5 * (y0 + y1), label, lcls, anchor, ldx, ldy)

    def dim(self, x0, y0, x1, y1, label, ldx=0, ldy=-5, anchor="middle", ext=None):
        """Dimension line with arrowheads; ext = (dx, dy) in m to draw extension lines from the
        measured points to the dimension line (given as the offset of the dim line)."""
        self.add(f'<line class="dim" x1="{f(self.X(x0))}" y1="{f(self.Y(y0))}" x2="{f(self.X(x1))}" '
                 f'y2="{f(self.Y(y1))}" marker-start="url(#{self.id}d)" marker-end="url(#{self.id}d)"/>')
        if ext:
            ex, ey = ext
            for (px, py) in ((x0, y0), (x1, y1)):
                self.line(px - ex, py - ey, px, py, "ext")
        self.text(0.5 * (x0 + x1), 0.5 * (y0 + y1), label, "td", anchor, ldx, ldy)

    def svg(self, label, extra_defs=""):
        defs = (f'<defs><marker id="{self.id}d" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="7" '
                f'markerHeight="7" orient="auto-start-reverse"><path class="dimhead" d="M0,1 L9,5 L0,9 z"/></marker>'
                f'<marker id="{self.id}r" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" '
                f'orient="auto"><path class="routehead" d="M0,1 L9,5 L0,9 z"/></marker>{extra_defs}</defs>')
        return (f'<svg class="dwg" viewBox="0 0 {f(self.w)} {f(self.h)}" width="{f(self.w)}" role="img" '
                f'aria-label="{label}">{defs}{"".join(self.parts)}</svg>')


# ── the airframe, drawn ──────────────────────────────────────────────────────
def drone_plan(fg, bx, by, cls="frame"):
    for sx in (-1, 1):
        for sy in (-1, 1):
            fg.line(bx, by, bx + sx * ROTOR_XY, by + sy * ROTOR_XY, cls)
            fg.circle(bx + sx * ROTOR_XY, by + sy * ROTOR_XY, PROP_R, cls + " disc")
    fg.rect(bx - 0.07, by - 0.07, bx + 0.07, by + 0.07, cls + " body")


def drone_side(fg, bx, bz, ghost=False):
    g = " ghost" if ghost else ""
    # props (to scale: disc bottom / top above the body origin, reach fore and aft)
    for side in (-1, 1):
        xa = bx + side * (ROTOR_XY - PROP_R)
        xb = bx + side * (ROTOR_XY + PROP_R)
        fg.rect(xa, bz + G.PROP_BOTTOM_ABOVE_BODY, xb, bz + G.PROP_TOP_ABOVE_BODY, "prop" + g)
        fg.line(bx + side * ROTOR_XY, bz + 0.02, bx + side * ROTOR_XY, bz + G.PROP_BOTTOM_ABOVE_BODY, "frame" + g)
    fg.line(bx - ROTOR_XY, bz + 0.02, bx + ROTOR_XY, bz + 0.02, "frame" + g)
    # body (outline schematic) and the gear (skid height and length to scale)
    fg.rect(bx - 0.09, bz - 0.035, bx + 0.09, bz + 0.03, "bodyside" + g)
    for side in (-1, 1):
        fg.line(bx + side * 0.06, bz - 0.035, bx + side * (SKID_X - 0.03), bz - G.GEAR_DEPTH, "frame" + g)
    fg.line(bx - SKID_X, bz - G.GEAR_DEPTH, bx + SKID_X, bz - G.GEAR_DEPTH, "skid" + g)
    # the arm, to scale, at [0, 36, 44, 90] deg
    pts = [(bx + a, bz + b) for a, b in ARM]
    fg.poly(pts[:4], "arm" + g)
    for (x, z) in pts[1:4]:
        fg.circle(x, z, 0, "joint" + g, r_px=3.2)
    d = np.array([math.cos(math.radians(PITCH)), math.sin(math.radians(PITCH))])
    n = np.array([-d[1], d[0]])
    claw = np.array(pts[4])
    palm = claw - 0.035 * d
    fg.poly([pts[3], tuple(palm)], "arm" + g)
    for sgn in (-1, 1):
        a = palm + sgn * 0.007 * n
        b = claw + G.JAW_TIP_BEYOND_CLAW * d + sgn * 0.0028 * n
        fg.poly([tuple(palm), tuple(a), tuple(b)], "finger" + g)
    fg.circle(claw[0], claw[1], 0, "clawpt" + g, r_px=2.6)
    return claw, d


def envelope_side(fg, near_x, near_z, pitch_deg, cls):
    d = np.array([math.cos(math.radians(pitch_deg)), -math.sin(math.radians(pitch_deg))])
    n = np.array([-d[1], d[0]])
    p0 = np.array([near_x, near_z])
    t = 0.5 * G.ENV_T
    pts = [p0 - t * n, p0 + G.ENV_L * d - t * n, p0 + G.ENV_L * d + t * n, p0 + t * n]
    fg.poly([tuple(p) for p in pts], cls, closed=True)
    return p0, d


# ── Figure 1: plan view of the field ─────────────────────────────────────────
def fig_plan():
    fg = Fig(-0.50, 2.02, -0.70, 1.46, 400, pad=(54, 24, 24, 40))
    for gx in np.arange(-0.5, 2.01, 0.5):
        fg.line(gx, -0.70, gx, 1.46, "grid")
        fg.text(gx, -0.70, f"{gx:.1f}".replace("-0.0", "0.0"), "tg", "middle", 0, 16)
    for gy in np.arange(-0.5, 1.01, 0.5):
        fg.line(-0.50, gy, 2.02, gy, "grid")
        fg.text(-0.50, gy, f"{gy:.1f}".replace("-0.0", "0.0"), "tg", "end", -6, 4)
    fg.text(2.02, -0.70, "x [m], forward", "tg", "end", 0, 32)
    fg.text(-0.50, 1.46, "y [m], left", "tg", "start", -48, -8)
    # table and mailboxes
    fg.rect(G.TABLE_FRONT_X, -0.5, G.TABLE_FRONT_X + 0.150, 0.5, "wood")
    fg.rect(G.TABLE_FRONT_X, G.RED_Y - 0.5 * G.BOX_W, G.TABLE_FRONT_X + G.BOX_D, G.RED_Y + 0.5 * G.BOX_W, "redbox")
    fg.rect(G.TABLE_FRONT_X, -0.25 - 0.5 * G.BOX_W, G.TABLE_FRONT_X + G.BOX_D, -0.25 + 0.5 * G.BOX_W, "bluebox")
    for cy in (G.RED_Y, -0.25):
        fg.line(G.TABLE_FRONT_X, cy - 0.5 * G.MOUTH_W, G.TABLE_FRONT_X, cy + 0.5 * G.MOUTH_W, "mouthline")
    # pillar + hat + envelope
    fg.circle(G.PILLAR_XY[0], G.PILLAR_XY[1], G.HAT_RADIUS, "hat")
    fg.circle(G.PILLAR_XY[0], G.PILLAR_XY[1], 0.049, "pillarhidden")
    fg.rect(NEAR_X, G.PILLAR_XY[1] - 0.5 * G.ENV_W, NEAR_X + G.ENV_L, G.PILLAR_XY[1] + 0.5 * G.ENV_W, "env")
    # the vehicle at the start, the pick and the release
    drone_plan(fg, 0.0, 0.0, "frame")
    drone_plan(fg, BODY_PICK[0], BODY_PICK[1], "frame ghost")
    drone_plan(fg, BODY_REL[0], BODY_REL[1], "frame ghost")
    for (bx, by), (cx, cy) in (((BODY_PICK[0], BODY_PICK[1]), (CLAW_PICK[0], CLAW_PICK[1])),
                               ((BODY_REL[0], BODY_REL[1]), (CLAW_REL[0], CLAW_REL[1]))):
        fg.line(bx, by, cx, cy, "armplan")
        fg.circle(cx, cy, 0, "clawpt", r_px=3)
    # the route (body positions)
    route = [(0.0, 0.0), RTP_BODY[:2], BODY_PICK[:2], MAILBOX_START[:2], RTI_BODY[:2], BODY_REL[:2]]
    for a, b in zip(route[:-1], route[1:]):
        if math.hypot(b[0] - a[0], b[1] - a[1]) > 0.02:
            fg.arrow(a[0], a[1], b[0], b[1], "route")
    fg.arrow(EXIT_BODY[0], EXIT_BODY[1] - 0.012, 0.06, -0.012, "route back", "back to Land Start", 0, 16, "tm")
    for p in route[1:]:
        fg.circle(p[0], p[1], 0, "routept", r_px=4)
    # labels
    fg.text(0.0, -0.44, "Start mark (0, 0): take-off, Land Start, Land", "t", "middle", 0, 14)
    fg.text(G.PILLAR_XY[0], G.PILLAR_XY[1] + G.HAT_RADIUS, "Pick pillar + hat (1.0, 1.0)", "t", "middle", 0, -10)
    fg.text(ENV_C[0] - 0.02, G.PILLAR_XY[1] - 0.5 * G.ENV_W, "envelope", "tr", "middle", 0, 16)
    fg.text(RTP_BODY[0], RTP_BODY[1], "Ready To Pick", "t", "end", -10, 22)
    fg.text(BODY_PICK[0], BODY_PICK[1], "Pick", "t", "start", 8, 22)
    fg.text(MAILBOX_START[0], MAILBOX_START[1], "Mailbox Start", "t", "end", -10, -8)
    fg.text(RTI_BODY[0], RTI_BODY[1], "Ready To Insert", "t", "middle", 0, -12)
    fg.text(BODY_REL[0], BODY_REL[1], "Release", "t", "middle", 6, 22)
    fg.text(G.TABLE_FRONT_X + 0.075, 0.5, "table", "tg", "middle", 0, -8)
    fg.text(G.TABLE_FRONT_X + 0.5 * G.BOX_D, G.RED_Y + 0.5 * G.BOX_W, "RED (target)", "tr", "middle", 0, -8)
    fg.text(G.TABLE_FRONT_X + 0.5 * G.BOX_D, -0.25 - 0.5 * G.BOX_W, "BLUE (scenery)", "tb", "middle", 0, 18)
    # dimensions
    fg.dim(0.0, -0.58, G.TABLE_FRONT_X, -0.58, f"{G.TABLE_FRONT_X:.2f} m to the table's front edge", 0, -6,
           ext=(0, -0.58 + 0.30))
    fg.dim(G.TABLE_FRONT_X + 0.22, -0.25, G.TABLE_FRONT_X + 0.22, G.RED_Y, "0.50", 18, 4)
    return fg.svg("Plan view of the field: start mark, pick pillar with the envelope, the console table with "
                  "the red and blue mailboxes, the vehicle at the start, pick and release, and the route between them.")


# ── Figure 2: the envelope ───────────────────────────────────────────────────
MARKERS_MM = [(48, 20), (60, 72), (86, 24)]


def fig_envelope_top():
    k = 3.4                                   # px per mm
    L, W = G.ENV_L * 1e3, G.ENV_W * 1e3
    fg = Fig(-40, L + 30, -48, W + 44, k, pad=(10, 10, 10, 10))
    # world units here are mm
    pat = (f'<pattern id="{fg.id}h" width="6" height="6" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">'
           f'<line class="hatch" x1="0" y1="0" x2="0" y2="6"/></pattern>')
    fg.rect(0, 0, L, W, "env")
    fg.rect(0, 0, G.GRASP_IN * 1e3 + G.JAW_TIP_BEYOND_CLAW * 1e3, W, "jawzone", f' fill="url(#{fg.id}h)"')
    fg.rect(120, 5, 150, 85, "steel")
    fg.rect(40, 6, 90, 84, "markzone")
    for (u, v) in MARKERS_MM:
        fg.circle(u, v, 0, "marker", r_px=3.4 * 3.5)
    cx, cy = 0.5 * L, 0.5 * W
    fg.circle(cx, cy, 0, "pivot", r_px=4)
    fg.arrow(cx, cy, cx + 24, cy, "axis")
    fg.text(cx + 26, cy, "x", "tax", "start", 2, 4)
    fg.arrow(cx, cy, cx, cy + 20, "axis")
    fg.text(cx, cy + 22, "y", "tax", "middle", 0, -2)
    com_u = L - G.ENV_COM_FROM_FAR * 1e3
    fg.add(f'<g class="com"><circle cx="{f(fg.X(com_u))}" cy="{f(fg.Y(cy))}" r="7"/>'
           f'<line x1="{f(fg.X(com_u) - 7)}" y1="{f(fg.Y(cy))}" x2="{f(fg.X(com_u) + 7)}" y2="{f(fg.Y(cy))}"/>'
           f'<line x1="{f(fg.X(com_u))}" y1="{f(fg.Y(cy) - 7)}" x2="{f(fg.X(com_u))}" y2="{f(fg.Y(cy) + 7)}"/></g>')
    fg.circle(G.GRASP_IN * 1e3, cy, 0, "clawpt", r_px=4)
    # labels
    fg.text(15.5, W, "jaws", "t", "middle", 0, -26)
    fg.text(15.5, W, "(top + bottom)", "tm", "middle", 0, -12)
    fg.text(65, W, "mocap markers", "t", "middle", 0, -12)
    fg.text(135, W, "steel weight", "t", "middle", 0, -12)
    fg.text(cx + 3, cy, "obj_0", "tax", "start", 6, 18)
    fg.text(com_u, cy, "CoM", "t", "middle", 0, 24)
    fg.text(G.GRASP_IN * 1e3, cy, "grasp", "t", "middle", 0, 22)
    fg.text(0, W * 0.5, "near edge", "tm", "end", -8, -3)
    fg.text(0, W * 0.5, "(drone side)", "tm", "end", -8, 12)
    # dimensions (mm)
    fg.dim(0, -14, L, -14, f"{L:.0f}", 0, 16)
    fg.dim(0, -30, G.GRASP_IN * 1e3, -30, "15", 0, 16)
    fg.dim(0, -42, (G.GRASP_IN + G.JAW_TIP_BEYOND_CLAW) * 1e3, -42, "31", 0, 16)
    fg.dim(com_u, -30, L, -30, "40", 0, 16)
    fg.dim(120, -42, 150, -42, "30", 0, 16)
    fg.dim(L + 14, 0, L + 14, W, f"{W:.0f}", 14, 4)
    fg.dim(40, W + 22, 90, W + 22, "40 to 90", 0, -6)
    return fg.svg("Envelope, top view, to scale: 150 by 90 millimetres, jaws take the near 31 millimetres, "
                  "markers between 40 and 90 millimetres, the steel weight in the far 30 millimetres, centre of mass "
                  "40 millimetres from the far end.", pat)


def fig_envelope_section():
    kx, kz = 3.4, 10.2                        # thickness drawn x3
    L = G.ENV_L * 1e3
    fg = Fig(-40, L + 30, -9, 12, kx, pad=(10, 10, 10, 10))
    # separate vertical scale: draw with a helper
    def Yz(z):
        return fg.pt + (12 - z) * kz
    fg.Y = Yz
    fg.h = fg.pt + 21 * kz + fg.pb
    fg.rect(0, 0, L, 2.0, "core")
    fg.rect(120, 2.0, 150, 3.9, "steel")
    fg.add(f'<rect class="wrap" x="{f(fg.X(-0.3))}" y="{f(Yz(4.2))}" width="{f((L + 0.6) * kx)}" '
           f'height="{f(4.5 * kz)}" rx="2"/>')
    for (u, _) in MARKERS_MM:
        fg.add(f'<path class="markerside" d="M{f(fg.X(u - 3.2))},{f(Yz(2.3))} A {f(3.2 * kx)} {f(4.0 * kz)} 0 0 1 '
               f'{f(fg.X(u + 3.2))},{f(Yz(2.3))} Z"/>')
    # jaws (schematic, closing vertically on the near 31 mm)
    tip = (G.GRASP_IN + G.JAW_TIP_BEYOND_CLAW) * 1e3
    fg.poly([(-38, 6.5), (tip, 2.5)], "finger")
    fg.poly([(-38, -4.5), (tip, -0.2)], "finger")
    fg.text(-38, 6.5, "upper jaw", "tm", "start", 0, -8)
    fg.text(-38, -4.5, "lower jaw", "tm", "start", 0, 16)
    fg.text(60, 0, "PLA core, 2 mm (or thinner, see the mass rule)", "tm", "middle", 0, 16)
    fg.text(135, 3.9, "steel 80 x 30 x 1.9", "t", "middle", 0, -10)
    fg.text(60, 6.5, "markers on top, &#8804; 4 mm tall", "tm", "middle", 0, -6)
    fg.text(L, 4.2, "red paper wrap, bottom face flat", "tm", "end", 0, -40)
    fg.add(f'<line class="dim" x1="{f(fg.X(L + 12))}" y1="{f(Yz(-0.15))}" x2="{f(fg.X(L + 12))}" y2="{f(Yz(4.05))}" '
           f'marker-start="url(#{fg.id}d)" marker-end="url(#{fg.id}d)"/>')
    fg.text(L + 12, 2, "&#8776; 4", "td", "start", 8, 4)
    return fg.svg("Envelope, side section with the thickness drawn three times larger: a 2 millimetre PLA core, "
                  "the steel weight on top at the far end, markers on top in the middle, the jaws pinching the near end.")


# ── Figure 3: the mailboxes ──────────────────────────────────────────────────
def fig_front_table():
    """Looking along +x from the start: +y (the red box) is on the LEFT."""
    fg = Fig(-0.56, 0.56, 0.70, 1.13, 560, pad=(10, 22, 10, 30))
    # page u = -y; Fig.X takes u
    fg.rect(-0.5, G.TABLE_TOP - 0.018, 0.5, G.TABLE_TOP, "wood")
    for u in (-0.5, 0.475):
        fg.rect(u, 0.70, u + 0.025, G.TABLE_TOP - 0.018, "wood")
    for y, cls, name in ((G.RED_Y, "redbox", "RED, target"), (-0.25, "bluebox", "BLUE, scenery")):
        u = -y
        fg.rect(u - 0.5 * G.BOX_W, G.TABLE_TOP, u + 0.5 * G.BOX_W, BOX_TOP, cls)
        fg.rect(u - 0.5 * G.MOUTH_W, MOUTH_LO, u + 0.5 * G.MOUTH_W, MOUTH_HI, "mouth")
        fg.text(u, G.TABLE_TOP + 0.06, name, "tw", "middle")
    fg.dim(-G.RED_Y, BOX_TOP + 0.025, 0.25, BOX_TOP + 0.025, "500 between centres", 0, -6,
           ext=(0, 0.025 - 0.006))
    fg.dim(-0.5, 0.735, 0.5, 0.735, "table 1000 x 150, top at 0.80 m", 0, -6)
    fg.text(-0.5, 0.70, "y = +0.5", "tg", "start", 0, 16)
    fg.text(0.5, 0.70, "y = -0.5", "tg", "end", 0, 16)
    return fg.svg("Front view of the table from the start: the red mailbox on the left at y plus 0.25 metres, "
                  "the blue one on the right at y minus 0.25 metres, both with the same mouth.")


def fig_red_face():
    k = 1.35
    W, H = G.BOX_W * 1e3, G.BOX_H * 1e3
    fg = Fig(-30, W + 96, -34, H + 26, k, pad=(10, 10, 10, 10))
    fg.rect(0, 0, W, H, "redbox")
    mx0, mx1 = 0.5 * (W - G.MOUTH_W * 1e3), 0.5 * (W + G.MOUTH_W * 1e3)
    my0, my1 = (G.MOUTH_Z - 0.5 * G.MOUTH_H) * 1e3, (G.MOUTH_Z + 0.5 * G.MOUTH_H) * 1e3
    fg.rect(mx0, my0, mx1, my1, "mouth")
    fg.line(mx0, G.MOUTH_Z * 1e3, mx1, G.MOUTH_Z * 1e3, "cl")
    fg.text(0.5 * W, G.MOUTH_Z * 1e3, "mouth", "tw", "middle", 0, 4)
    fg.dim(0, -16, W, -16, f"{W:.0f}", 0, 16)
    fg.dim(W + 50, 0, W + 50, H, f"{H:.1f}", 10, 4, "start")
    fg.dim(mx0, H - 12, mx1, H - 12, f"{G.MOUTH_W * 1e3:.0f}", 0, -5)
    fg.dim(mx0 - 12, my0, mx0 - 12, my1, f"{G.MOUTH_H * 1e3:.0f}", -6, 4, "end")
    fg.dim(W + 22, my1, W + 22, H, f"{(G.BOX_H - G.MOUTH_Z - 0.5 * G.MOUTH_H) * 1e3:.0f}", 6, 4, "start")
    fg.dim(W + 22, 0, W + 22, my0, f"{my0:.1f}", 6, 4, "start")
    fg.dim(0, my0 - 14, mx0, my0 - 14, f"{mx0:.1f}", 0, -5)
    fg.text(0.5 * W, 40, "5 mm walls (assumed)", "tw", "middle")
    return fg.svg("The red mailbox's front face to scale: 235 by 272.5 millimetres, mouth 190 by 90 millimetres "
                  "centred across, its top edge 25 millimetres under the box top and its lower lip 157.5 millimetres "
                  "above the box bottom.")


# ── Figure 4: side views at the pick and at the release ──────────────────────
def box_section(fg):
    x0, x1 = G.TABLE_FRONT_X, G.TABLE_FRONT_X + G.BOX_D
    z0, z1, w = G.TABLE_TOP, BOX_TOP, G.WALL
    fg.rect(x0, z0 - 0.018, x0 + 0.150, z0, "wood")
    for a in ((x0, z0, x0 + w, MOUTH_LO), (x0, MOUTH_HI, x0 + w, z1), (x0, z1 - w, x1, z1),
              (x1 - w, z0, x1, z1), (x0, z0, x1, z0 + w)):
        fg.rect(*a, "wallcut")
    fg.rect(x0 + w, z0 + w, x1 - w, z1 - w, "boxinside")


def fig_side_pick():
    fg = Fig(0.10, 1.16, 0.54, 0.98, 900, pad=(10, 18, 10, 26))
    # pillar + hat
    fg.rect(G.PILLAR_XY[0] - 0.049, 0.54, G.PILLAR_XY[0] + 0.049, G.PILLAR_TOP, "pillar")
    fg.rect(G.PILLAR_XY[0] - 0.061, G.PILLAR_TOP - 0.052, G.PILLAR_XY[0] + 0.061, HAT_TOP - 0.005, "hat2")
    fg.rect(RIM_X, HAT_TOP - 0.005, G.PILLAR_XY[0] + G.HAT_RADIUS, HAT_TOP, "hat2")
    p0, d = envelope_side(fg, NEAR_X, HAT_TOP + 0.5 * G.ENV_T, 0.0, "env")
    com = NEAR_X + G.ENV_L - G.ENV_COM_FROM_FAR
    fg.circle(com, HAT_TOP + 0.002, 0, "comdot", r_px=3.5)
    # ready-to-pick ghost claw and the vehicle at the pick
    claw, _ = drone_side(fg, BODY_PICK[0], BODY_PICK[2])
    fg.arrow(CLAW_PICK[0] - 0.10, CLAW_PICK[2] + 0.045, CLAW_PICK[0], CLAW_PICK[2] + 0.045, "route")
    fg.text(CLAW_PICK[0] - 0.10, CLAW_PICK[2] + 0.045, "Pick: slides in 100 mm, level", "t", "start", 0, -8)
    # dimensions
    fg.dim(NEAR_X, HAT_TOP - 0.022, RIM_X, HAT_TOP - 0.022, "70", 0, 15)
    tip = NEAR_X + G.GRASP_IN + G.JAW_TIP_BEYOND_CLAW
    fg.dim(NEAR_X, HAT_TOP + 0.024, tip, HAT_TOP + 0.024, "31", 0, -5)
    fg.dim(tip, HAT_TOP + 0.024, RIM_X, HAT_TOP + 0.024, "39", 0, -5)
    fg.dim(RIM_X, HAT_TOP - 0.022, com, HAT_TOP - 0.022, "40", 0, 15)
    pb = BODY_PICK[2] + G.PROP_BOTTOM_ABOVE_BODY
    fg.dim(1.06, HAT_TOP, 1.06, pb, f"{(pb - HAT_TOP) * 1e3:.0f}", 7, 4, "start")
    sk = BODY_PICK[0] + SKID_X
    gz = BODY_PICK[2] - G.GEAR_DEPTH
    fg.dim(sk, gz + 0.012, RIM_X, gz + 0.012, f"{(RIM_X - sk) * 1e3:.0f}", 0, -5)
    # labels
    fg.text(1.06, 0.54, "pillar 0.77 m + hat (platform top 0.778 m)", "tm", "end", 0, -6)
    fg.text(NEAR_X + G.ENV_L, HAT_TOP, "envelope", "tr", "start", 6, -10)
    fg.text(com, HAT_TOP, "CoM", "t", "middle", 0, -22)
    fg.text(BODY_PICK[0], pb + 0.012, "props (15 in)", "tm", "middle", 0, -6)
    fg.text(BODY_PICK[0] - SKID_X, gz, "gear skid", "tm", "start", 0, 16)
    return fg.svg("Side view at the pick, to scale: the claw slides in level and pinches the envelope 15 millimetres "
                  "inside its near edge; the jaw tips stop 39 millimetres short of the hat's rim; the props pass 128 "
                  "millimetres over the hat.")


def fig_side_release():
    fg = Fig(0.98, 2.02, 0.76, 1.205, 900, pad=(10, 18, 10, 26))
    box_section(fg)
    # the envelope at the release: 10 deg (the claw's angle) solid, 20 deg (worst assumed) dashed
    near = CLAW_REL[0] - G.GRASP_IN * math.cos(math.radians(10))
    envelope_side(fg, CLAW_REL[0] - G.GRASP_IN * math.cos(math.radians(20)),
                  CLAW_REL[2] + G.GRASP_IN * math.sin(math.radians(20)), 20.0, "env ghost")
    envelope_side(fg, near, CLAW_REL[2] + G.GRASP_IN * math.sin(math.radians(10)), 10.0, "env")
    fg.circle(COM_X10, CLAW_REL[2] - (S_TIP - G.ENV_COM_FROM_FAR) * math.sin(math.radians(10)), 0, "comdot",
              r_px=3.5)
    # where the envelope waits at Ready To Insert, then the vehicle at the release
    envelope_side(fg, near - G.INSERT_MARGIN, CLAW_REL[2] + G.GRASP_IN * math.sin(math.radians(10)), 10.0, "env rti")
    fg.text(near - G.INSERT_MARGIN, CLAW_REL[2] - 0.03, "envelope at Ready To Insert", "tax", "start", 0, 12)
    claw, _ = drone_side(fg, BODY_REL[0], BODY_REL[2])
    fg.arrow(CLAW_REL[0] - G.INSERT_MARGIN, CLAW_REL[2] + 0.11, CLAW_REL[0], CLAW_REL[2] + 0.11, "route")
    fg.text(CLAW_REL[0] - G.INSERT_MARGIN, CLAW_REL[2] + 0.11, "Insert: slides in 150 mm, level", "t", "start", 2, 15)
    # dimensions
    tipx = CLAW_REL[0] + G.JAW_TIP_BEYOND_CLAW
    fg.dim(tipx, CLAW_REL[2] - 0.034, G.TABLE_FRONT_X, CLAW_REL[2] - 0.034, "35", 0, 15)
    zfar = CLAW_REL[2] - S_TIP * math.sin(math.radians(10))
    fg.dim(TIP_X10, zfar - 0.045, G.TABLE_FRONT_X + G.BOX_D - G.WALL, zfar - 0.045, f"{M_BACK * 1e3:.0f}", 0, 15)
    fg.dim(G.TABLE_FRONT_X + G.WALL, zfar - 0.075, COM_X10, zfar - 0.075, f"{M_LIP * 1e3:.0f}", 0, 15)
    pb = BODY_REL[2] + G.PROP_BOTTOM_ABOVE_BODY
    fg.dim(1.96, BOX_TOP, 1.96, pb, f"{PROPS_OVER * 1e3:.0f}", 7, 4, "start")
    fg.dim(1.905, MOUTH_LO, 1.905, MOUTH_HI, "90", 7, 4, "start")
    fg.dim(1.905, MOUTH_HI, 1.905, BOX_TOP, "25", 7, 4, "start")
    fg.line(G.TABLE_FRONT_X - 0.012, MOUTH_C, 1.86, MOUTH_C, "cl")
    fg.line(CLAW_REL[0] + 0.02, CLAW_REL[2], 1.86, CLAW_REL[2], "cl")
    fg.dim(1.85, MOUTH_C, 1.85, CLAW_REL[2], "21.5", 5, 4, "start")
    sk = BODY_REL[0] + SKID_X
    gz = BODY_REL[2] - G.GEAR_DEPTH
    fg.dim(sk, gz + 0.012, G.TABLE_FRONT_X, gz + 0.012, f"{(G.TABLE_FRONT_X - sk) * 1e3:.0f}", 0, -5)
    # labels
    fg.text(COM_X10, CLAW_REL[2] - (S_TIP - G.ENV_COM_FROM_FAR) * math.sin(math.radians(10)), "CoM", "t", "middle", 0, 18)
    fg.text(G.TABLE_FRONT_X + 0.075, G.TABLE_TOP - 0.018, "red mailbox cut at the mouth, on the table (top 0.80 m)", "tm", "middle", 0, 15)
    return fg.svg("Side view at the release, to scale: the claw 21.5 millimetres above the mouth's centre, the "
                  "jaw tips 35 millimetres short of the wall, the envelope's far end 38 millimetres short of the "
                  "back wall and its centre of mass 38 millimetres past the lip; the props pass 77 millimetres over "
                  "the box top.")


# ── tables ───────────────────────────────────────────────────────────────────
def table(head, rows, cls=""):
    th = "".join(f"<th>{h}</th>" for h in head)
    def num(j, c):
        return j > 0 and isinstance(c, str) and 0 < len(c) < 32 and re.match(r"^[-+0-9&]", c) is not None
    trs = "".join("<tr>" + "".join(f"<td{' class=n' if num(j, c) else ''}>{c}</td>"
                                   for j, c in enumerate(r)) + "</tr>" for r in rows)
    return f'<div class="tscroll"><table class="{cls}"><thead><tr>{th}</tr></thead><tbody>{trs}</tbody></table></div>'


def c3(p):
    return f"{p[0]:.3f}", f"{p[1]:.3f}", f"{p[2]:.3f}"


def build():
    coord_rows = [
        ["Start mark", "0.000", "0.000", "1.000 (hover)", "0", "take-off, Land Start and Land; the sim spawns at (0.10, &minus;0.07) so Adjust has a shift"],
        ["Pick pillar", "1.000", "1.000", f"{G.PILLAR_TOP:.3f} top", "&ndash;", f"hat platform &#216;200.6 mm, top {HAT_TOP:.3f} m"],
        ["Envelope (obj_0, its centre)", *c3(ENV_C), "0", f"long axis along +x, near edge at x = {NEAR_X:.3f} ({G.OVERHANG * 1e3:.0f} mm past the rim)"],
        ["Ready To Pick (body)", *c3(RTP_BODY), "0", "claw 0.10 m behind the grasp point, level"],
        ["Pick (body)", *c3(BODY_PICK), "0", f"claw at ({CLAW_PICK[0]:.3f}, {CLAW_PICK[1]:.3f}, {CLAW_PICK[2]:.3f})"],
        ["Mailbox Start (body)", f"{MAILBOX_START[0]:.3f}", f"{MAILBOX_START[1]:.3f}", f"{MAILBOX_START[2]:.3f}", "0", "pick_place_place_start"],
        ["Ready To Insert (body)", *c3(RTI_BODY), "0", f"claw {G.INSERT_MARGIN * 1e3:.0f} mm behind the release, level with it"],
        ["Release (body)", *c3(BODY_REL), "0", f"claw at ({CLAW_REL[0]:.3f}, {CLAW_REL[1]:.3f}, {CLAW_REL[2]:.3f})"],
        ["Place point (envelope centre at the release)", *c3(PLACE_POINT), "0", "pick_place_place_point"],
        ["Mouth centre (drop_0)", f"{G.TABLE_FRONT_X:.3f}", f"{G.RED_Y:.3f}", f"{MOUTH_C:.4f}", "&ndash;", f"opening z {MOUTH_LO:.4f} to {MOUTH_HI:.4f}"],
        ["Console table", f"{G.TABLE_FRONT_X:.3f} to 1.850", "&minus;0.500 to 0.500", f"{G.TABLE_TOP:.3f} top", "&ndash;", "front edge faces the start"],
        ["Red mailbox", f"{G.TABLE_FRONT_X:.3f} to {G.TABLE_FRONT_X + G.BOX_D:.3f}", f"{G.RED_Y - 0.5 * G.BOX_W:.4f} to {G.RED_Y + 0.5 * G.BOX_W:.4f}", f"{G.TABLE_TOP:.3f} to {BOX_TOP:.4f}", "&ndash;", "front flush with the table's front edge"],
        ["Blue mailbox", f"{G.TABLE_FRONT_X:.3f} to {G.TABLE_FRONT_X + G.BOX_D:.3f}", f"{-0.25 - 0.5 * G.BOX_W:.4f} to {-0.25 + 0.5 * G.BOX_W:.4f}", f"{G.TABLE_TOP:.3f} to {BOX_TOP:.4f}", "&ndash;", "scenery"],
        ["Land Start / Land", "0.000", "0.000", "1.000 / 0.350", "0", "back over the start mark"],
    ]
    bom_rows = [
        ["Core", "150 &times; 90 &times; 2 mm", "PLA", "~20 g", "full length"],
        ["Wrap", "150 &times; 90 mm, glued flat; no loose flap at the near edge", "red paper", "~5 g", "all faces"],
        ["End weight", "80 &times; 30 &times; 1.9 mm", "steel", "35 g", "top face, far end, its centre 15 mm from the far edge"],
        ["Mocap markers", "&le; 4 mm tall, 3 or more, asymmetric", "low-profile", "~1 g each", "top face, 40 to 90 mm from the near edge"],
        ["<b>Envelope</b>", "<b>150 &times; 90 &times; &asymp; 4 mm</b> (+ markers)", "", "<b>60 g</b> (55 to 65)", "<b>CoM 40 mm from the far end</b> (48 at most)"],
    ]
    rule_rows = [
        ["20 g", "28 g (1.5 mm steel)", "48 g", "lighter than flown; fine"],
        ["25 g", "35 g (1.9 mm steel)", "60 g", "the design, flown in simulation"],
        ["30 g", "42 g (2.2 mm steel)", "72 g", "above the 65 g ceiling: print the core thinner"],
    ]
    box_rows = [
        ["Outer size", "235 W &times; 125 D &times; 272.5 H mm", "width along the table (y), depth away from the drone (x)"],
        ["Walls", "5 mm (assumed)", "every distance on this page is from the outer faces; measure the real boxes"],
        ["Mouth", "190 &times; 90 mm", "centred across the front face; both boxes have it"],
        ["Mouth height", "lower lip 157.5 mm, top edge 247.5 mm above the box bottom", f"z = {MOUTH_LO:.4f} to {MOUTH_HI:.4f} m; top edge 25 mm under the box top"],
        ["Side jambs", "22.5 mm each", ""],
        ["Inside depth", "115 mm", "the envelope (150 mm) ends standing on its end inside"],
        ["Placement", "fronts flush with the table's front edge (x = 1.70 m)", "red at y = +0.25 m (target), blue at y = &minus;0.25 m (scenery); fix both to the table"],
    ]
    arm_rows = [
        ["Joint angles q1 .. q4", "0, 36, 44, 90&#176;", "pick = carry = insert, one pose for the whole mission"],
        ["Claw position", f"{FWD * 1e3:.0f} mm ahead, {-DZ * 1e3:.1f} mm below the body origin", "the grasp point; fingertips 16 mm further"],
        ["Claw direction", f"{-PITCH:.0f}&#176; below level", "jaws close about vertically (10&#176; off)"],
        ["Distance to the joint stops", "q2 9&#176; (+45), q3 6&#176; (+50), q4 30&#176; (&plusmn;120)", "the real arm's limits"],
        ["Singularity measure &sigma;<sub>nd</sub>", f"{SIGMA:.3f}", "keep-out 0.10; q4 = 90&#176; is not singular for the law"],
        ["Pinch moment", "56 mN&middot;m", "60 g with its CoM 94 mm ahead of the pads' middle"],
        ["Gravity load on J4", "24 mN&middot;m at 40 mm lateral grip offset", "about 3&#176; of wrist roll against 0.445 N&middot;m/rad"],
    ]
    clear_rows = [
        ["Pick", "jaw tips &rarr; hat rim", "39 mm", "the lower jaw never passes over the hat"],
        ["Pick", "envelope CoM inside the rim", "40 mm", "it rests stably with the near edge out 70 mm"],
        ["Pick", "props &rarr; hat top", "128 mm", ""],
        ["Pick", "gear skid &rarr; hat rim, horizontal", "142 mm", "skids sit below the hat's height but beside it"],
        ["Release", "jaw tips &rarr; mailbox wall", "35 mm", ""],
        ["Release", "envelope far end &rarr; back wall", f"{M_BACK * 1e3:.0f} mm", "at 10&#176; pitch in the jaws"],
        ["Release", "envelope CoM past the mouth's lip", f"{M_LIP * 1e3:.0f} mm", "it falls in when the jaws open"],
        ["Release", "props &rarr; mailbox top", f"{PROPS_OVER * 1e3:.0f} mm", "55 mm with a 3&#176; nose-down pitch"],
        ["Release", "gear skid &rarr; table front, horizontal", f"{(G.TABLE_FRONT_X - BODY_REL[0] - SKID_X) * 1e3:.0f} mm", f"skids {(BODY_REL[2] - G.GEAR_DEPTH - G.TABLE_TOP) * 1e3:.0f} mm above the table top"],
        ["Ready To Insert", "envelope tip &rarr; mouth", "68 mm", "the slide starts clear of the box"],
    ]
    budget_rows = [
        ["Across the mouth (slide-in, not gated)", "mouth 190 mm, envelope 90 mm, 2&#176; yaw in the jaws", "&plusmn;45", "p95 26 to 29, p99 41 to 43, max 49", "&plusmn;40", "+5 a side"],
        ["Height in the mouth (slide-in, not gated)", "envelope band &minus;48 to +6 mm about the claw (0 to 20&#176; pitch, markers), mouth &minus;67 to +23", "&minus;18 / +17", "p99 8, max 16", "&plusmn;12", "+6 below, +5 above"],
        ["Fore-aft at the release (gated: opens within 20 mm)", "jaw tips at the wall / CoM 10 mm past the lip", "+35 / &minus;28", "at a gripper command: median 10 (3-D), max 32", "&plusmn;20", "+15 / +8"],
        ["Fore-aft at the pick (gated: 20 mm, 8 mm vertical)", "tips 10 mm short of the rim / 10 mm of grip left", "+29 / &minus;21", "as above", "&plusmn;20", "+9 / +1"],
        ["Props over the mailbox", "77 mm nominal", "77 down", "body tilt p95 3.0&#176;", "3&#176; nose-down = 22 down", "+55"],
    ]
    sim_rows = [
        ["mail2", "raw mocap", "in the red box, on its floor", "36 / 17 / 42", "35 / 35", "75", "8.2&#176; (6 to 11)", "6 / 9", "2.9 / 2.7 / 3.4&#176;"],
        ["mail3", "raw mocap", "in the red box, on its floor", "30 / 18 / 45", "35 / 27", "78", "10.9&#176; (8 to 14)", "10 / 11", "3.1 / 2.8 / 4.2&#176;"],
        ["mail4", "EKF2-fused (hardware path)", "in the red box, on its floor", "29 / 24 / 45", "41 / 43", "74", "12.9&#176; (12 to 14)", "17 / 15", "4.0 / 2.9 / 3.1&#176;"],
    ]
    steps = [
        ("Adjust", "the Start row measures where the vehicle actually took off."),
        ("Envelope &rarr; Get", "averages obj_0 resting on the hat (the frozen-feed and flipping-yaw checks apply)."),
        ("Mailbox", "the place point; in simulation it comes from the yaml (1.709, 0.250, 1.024, 0)."),
        ("Insert Margin", "0.15 m: how far in front of the release the claw waits, level."),
        ("Plan", "switches the planner to the insert place and plans all six legs."),
        ("Go To Start", "the take-off hover over the start mark."),
        ("Execute To Pick", "Ready To Pick (claw 0.10 m behind the near edge), Pick (slides in; the gripper closes within 20 mm and 8 mm vertically), Exit To Pick starts by itself on the grasp and lifts the envelope."),
        ("Go To Mailbox Start", "carries the envelope to (1.0, 0.25, 1.1) with the arm unchanged."),
        ("Execute To Mailbox", "Ready To Insert, then Insert (slides in; the gripper opens within 20 mm); Exit To Insert starts by itself on the OPEN report, backs out 0.10 m and climbs 0.20 m; the gripper closes after it."),
        ("Go To Land Start &rarr; Execute To Land", "folds the arm home and lands on the start mark."),
    ]
    steps_html = "".join(f"<li><b>{a}</b>: {b}</li>" for a, b in steps)

    parts = {
        "plan": fig_plan(), "etop": fig_envelope_top(), "esec": fig_envelope_section(),
        "front": fig_front_table(), "face": fig_red_face(), "pick": fig_side_pick(), "rel": fig_side_release(),
    }
    html = PAGE.format(
        **parts,
        coords=table(["Point", "x [m]", "y [m]", "z [m]", "yaw [&#176;]", "Notes"], coord_rows, "coords"),
        bom=table(["Part", "Size", "Material", "Mass", "Where"], bom_rows),
        rule=table(["Core + wrap (spread)", "End weight for CoM 40 mm", "Total", ""], rule_rows),
        box=table(["Item", "Value", "Notes"], box_rows),
        arm=table(["Item", "Value", "Notes"], arm_rows),
        clear=table(["Where", "Clearance", "Nominal", "Notes"], clear_rows),
        budget=table(["Error axis", "Bounded by", "Tolerated [mm]", "Measured on hardware [mm]", "Budget used [mm]",
                      "Margin left [mm]"], budget_rows),
        sim=table(["Run", "Feedback", "Envelope ended", "Mouth margin below / above / side [mm]",
                   "Jaw tips &rarr; wall / far end &rarr; back wall [mm]", "Props over box [mm]",
                   "Pitch in the jaws, mean (range)", "Claw error at close / open [mm]",
                   "Peak tilt pick / carry / insert"], sim_rows),
        steps=steps_html,
    )
    return html


PAGE = """<title>Mail Delivery Experiment</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans+Condensed:wght@500;600&family=IBM+Plex+Sans:ital,wght@0,400;0,600;1,400&display=swap">
<style>
/* Layout: one full-width drawing-sheet column; figures are to-scale plates, tables sit under them. */
:root {{
  --bg: #000000; --panel: #141413; --line: #34332f; --fg: #ecebe6; --muted: #a3a199;
  --dim: #e3b341; --red: #ff5f57; --redfill: #4a1412; --blue: #7aa2ff; --bluefill: #13244d;
  --wood: #2f261c; --woodline: #8c7154; --route: #59c2c9; --good: #63c28c;
  --display: "IBM Plex Sans Condensed", "Arial Narrow", "Helvetica Neue", Arial, sans-serif;
  --body: "IBM Plex Sans", "Helvetica Neue", Arial, sans-serif;
  --mono: "IBM Plex Mono", ui-monospace, "SFMono-Regular", Menlo, monospace;
  color-scheme: dark;
}}
html {{ background: var(--bg); }}
body {{ background: var(--bg); color: var(--fg); font-family: var(--body); font-size: 15px; line-height: 1.55; }}
.wrap {{ max-width: 1180px; margin: 0 auto; padding-block: 28px 64px;
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); }}
header {{ display: grid; gap: 14px; padding-bottom: 22px; border-bottom: 1px solid var(--line); }}
.eyebrow {{ font-family: var(--mono); font-size: 12px; letter-spacing: .08em; text-transform: uppercase; color: var(--muted); }}
h1 {{ font-family: var(--display); font-weight: 600; font-size: clamp(30px, 4.4vw, 46px); line-height: 1.05; margin: 0; text-wrap: balance; }}
h2 {{ font-family: var(--display); font-weight: 600; font-size: 25px; line-height: 1.2; margin: 46px 0 10px; text-wrap: balance;
  padding-top: 12px; border-top: 1px solid var(--line); }}
h3 {{ font-family: var(--display); font-weight: 600; font-size: 18px; margin: 26px 0 8px; }}
p, li {{ margin: 0 0 10px; }}
ul, ol {{ padding-left: 22px; margin: 0 0 12px; }}
a {{ color: var(--route); }}
a:focus-visible {{ outline: 2px solid var(--dim); outline-offset: 2px; }}
code {{ font-family: var(--mono); font-size: 13px; background: var(--panel); padding: 1px 5px; border-radius: 3px; overflow-wrap: anywhere; }}
pre {{ font-family: var(--mono); font-size: 13px; background: var(--panel); border: 1px solid var(--line); padding: 12px 14px;
  overflow-x: auto; line-height: 1.5; border-radius: 4px; }}
.lede {{ font-size: 16.5px; }}
.chips {{ display: flex; flex-wrap: wrap; gap: 8px; }}
.chip {{ font-family: var(--mono); font-size: 12px; border: 1px solid var(--line); border-radius: 999px; padding: 3px 10px; }}
.chip.ok {{ border-color: var(--good); color: var(--good); }}
.chip.wait {{ border-color: var(--dim); color: var(--dim); }}
.spec {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(210px, 1fr)); gap: 1px; background: var(--line);
  border: 1px solid var(--line); margin-top: 6px; }}
.spec div {{ background: var(--bg); padding: 12px 14px; display: grid; gap: 3px; min-width: 0; }}
.spec .k {{ font-family: var(--mono); font-size: 11.5px; letter-spacing: .06em; text-transform: uppercase; color: var(--muted); }}
.spec .v {{ font-family: var(--display); font-size: 21px; font-weight: 600; }}
.spec .s {{ font-size: 13px; color: var(--muted); }}
nav.toc {{ margin-top: 22px; display: grid; gap: 4px; }}
nav.toc ol {{ columns: 2 280px; column-gap: 32px; margin: 0; }}
nav.toc li {{ margin: 0 0 4px; break-inside: avoid; }}
figure {{ margin: 18px 0 20px; }}
.plate {{ background: var(--panel); border: 1px solid var(--line); border-radius: 4px; overflow-x: auto; padding: 8px; }}
.plate svg {{ display: block; max-width: 100%; height: auto; margin: 0 auto; }}
.plate.wide svg {{ min-width: 720px; }}
.pair {{ display: grid; grid-template-columns: minmax(0, 1.55fr) minmax(0, 1fr); gap: 12px; }}
@media (max-width: 760px) {{ .pair {{ grid-template-columns: minmax(0, 1fr); }} }}
.stack {{ display: grid; gap: 12px; }}
.plate.narrow svg {{ max-width: min(100%, 860px); }}
figcaption {{ font-size: 13.5px; color: var(--muted); margin-top: 8px; }}
figcaption b {{ color: var(--fg); }}
.tscroll {{ overflow-x: auto; margin: 10px 0 18px; border: 1px solid var(--line); border-radius: 4px; }}
table {{ width: 100%; border-collapse: collapse; font-size: 14px; }}
th {{ text-align: left; font-family: var(--mono); font-weight: 500; font-size: 11.5px; letter-spacing: .05em; text-transform: uppercase;
  color: var(--muted); background: var(--panel); padding: 9px 10px; border-bottom: 1px solid var(--line); vertical-align: bottom; }}
td {{ padding: 8px 10px; border-bottom: 1px solid var(--line); vertical-align: top; }}
tbody tr:last-child td {{ border-bottom: 0; }}
td.n {{ font-family: var(--mono); font-size: 13px; font-variant-numeric: tabular-nums; white-space: nowrap; }}
table.coords td:first-child {{ white-space: nowrap; }}
.note {{ border-left: 3px solid var(--dim); background: var(--panel); padding: 10px 14px; margin: 14px 0; border-radius: 0 4px 4px 0; }}
.note.red {{ border-left-color: var(--red); }}
ol.seq li {{ padding-left: 4px; }}
img.shot {{ display: block; width: 100%; height: auto; border: 1px solid var(--line); border-radius: 4px; }}
.foot {{ margin-top: 48px; font-size: 13px; color: var(--muted); border-top: 1px solid var(--line); padding-top: 14px; }}
/* drawing classes (inline SVG) */
.dwg text {{ font-family: var(--body); font-size: 12.5px; paint-order: stroke; stroke: var(--panel); stroke-width: 4px; stroke-linejoin: round; }}
.dwg .t {{ fill: var(--fg); }} .dwg .tm {{ fill: var(--muted); }} .dwg .tg {{ fill: var(--muted); font-family: var(--mono); font-size: 11.5px; }}
.dwg .td {{ fill: var(--dim); font-family: var(--mono); font-size: 12px; }}
.dwg .tr {{ fill: var(--red); font-weight: 600; }} .dwg .tb {{ fill: var(--blue); font-weight: 600; }}
.dwg .tw {{ fill: var(--fg); stroke: none; font-size: 12px; }}
.dwg .tax {{ fill: var(--route); font-family: var(--mono); font-size: 12px; }}
.dwg .grid {{ stroke: var(--line); stroke-width: 1; }}
.dwg .dim {{ stroke: var(--dim); stroke-width: 1; fill: none; }}
.dwg .ext {{ stroke: var(--dim); stroke-width: .7; stroke-dasharray: 2 3; fill: none; }}
.dwg .dimhead {{ fill: var(--dim); }} .dwg .routehead {{ fill: var(--route); }}
.dwg .route {{ stroke: var(--route); stroke-width: 1.8; fill: none; }}
.dwg .route.back {{ stroke-dasharray: 6 5; opacity: .65; }}
.dwg .routept {{ fill: var(--route); stroke: var(--panel); stroke-width: 1.5; }}
.dwg .axis {{ stroke: var(--route); stroke-width: 1.6; fill: none; }}
.dwg .frame {{ stroke: var(--fg); stroke-width: 1.3; fill: none; }}
.dwg .frame.disc {{ stroke: var(--muted); stroke-width: 1; fill: none; }}
.dwg .frame.body {{ fill: var(--panel); }}
.dwg .ghost {{ opacity: .45; stroke-dasharray: 4 3; }}
.dwg .armplan {{ stroke: var(--dim); stroke-width: 2.2; }}
.dwg .clawpt {{ fill: var(--dim); stroke: var(--panel); stroke-width: 1; }}
.dwg .wood {{ fill: var(--wood); stroke: var(--woodline); stroke-width: 1; }}
.dwg .redbox {{ fill: var(--redfill); stroke: var(--red); stroke-width: 1.4; }}
.dwg .bluebox {{ fill: var(--bluefill); stroke: var(--blue); stroke-width: 1.4; }}
.dwg .mouthline {{ stroke: var(--bg); stroke-width: 5; }}
.dwg .mouth {{ fill: var(--bg); stroke: var(--fg); stroke-width: 1; }}
.dwg .hat {{ fill: #2a2a27; stroke: var(--fg); stroke-width: 1; }}
.dwg .hat2 {{ fill: #2a2a27; stroke: var(--fg); stroke-width: 1; }}
.dwg .pillar {{ fill: #1d1d1b; stroke: var(--muted); stroke-width: 1; }}
.dwg .pillarhidden {{ fill: none; stroke: var(--muted); stroke-width: .8; stroke-dasharray: 3 3; }}
.dwg .env {{ fill: var(--red); fill-opacity: .85; stroke: var(--red); stroke-width: 1; }}
.dwg .env.ghost {{ fill: none; stroke-width: 1.2; opacity: .8; }}
.dwg .env.rti {{ fill: none; stroke: var(--route); stroke-width: 1.2; stroke-dasharray: 3 3; }}
.dwg .jawzone {{ stroke: var(--fg); stroke-width: 1; }}
.dwg .hatch {{ stroke: var(--fg); stroke-width: 1.2; opacity: .55; }}
.dwg .steel {{ fill: #8d939b; stroke: var(--fg); stroke-width: 1; }}
.dwg .markzone {{ fill: none; stroke: var(--fg); stroke-width: 1; stroke-dasharray: 5 4; }}
.dwg .marker {{ fill: #d9d9d4; stroke: var(--panel); stroke-width: 1.5; }}
.dwg .markerside {{ fill: #d9d9d4; stroke: none; }}
.dwg .pivot {{ fill: var(--route); }}
.dwg .com circle {{ fill: none; stroke: var(--fg); stroke-width: 1.6; }} .dwg .com line {{ stroke: var(--fg); stroke-width: 1.6; }}
.dwg .comdot {{ fill: var(--fg); stroke: var(--panel); stroke-width: 1; }}
.dwg .core {{ fill: #c9c3b5; stroke: none; }}
.dwg .wrap {{ fill: none; stroke: var(--red); stroke-width: 1.6; }}
.dwg .finger {{ stroke: var(--fg); stroke-width: 2.2; fill: none; stroke-linejoin: round; }}
.dwg .prop {{ fill: var(--muted); fill-opacity: .35; stroke: var(--muted); stroke-width: 1; }}
.dwg .bodyside {{ fill: var(--bg); stroke: var(--fg); stroke-width: 1.3; }}
.dwg .skid {{ stroke: var(--fg); stroke-width: 3; stroke-linecap: round; }}
.dwg .arm {{ stroke: var(--dim); stroke-width: 3.2; fill: none; stroke-linejoin: round; stroke-linecap: round; }}
.dwg .joint {{ fill: var(--bg); stroke: var(--dim); stroke-width: 1.6; }}
.dwg .wallcut {{ fill: var(--red); fill-opacity: .55; stroke: var(--red); stroke-width: 1; }}
.dwg .boxinside {{ fill: var(--redfill); fill-opacity: .45; stroke: none; }}
.dwg .cl {{ stroke: var(--muted); stroke-width: .8; stroke-dasharray: 10 3 2 3; }}
@media (prefers-reduced-motion: reduce) {{ * {{ scroll-behavior: auto; }} }}
</style>

<div class="wrap">
<header>
  <div class="eyebrow">T650 aerial manipulator &middot; whole-body 4-D L1 controller &middot; final pick-and-place experiment &middot; 2026-10-23</div>
  <h1>Mail Delivery Experiment</h1>
  <p class="lede">The drone takes off, picks a red envelope lying flat on the pick pillar's hat with its wrist rolled 90&#176;, and slides it through a mouth cut in the front of the red mailbox on the console table. The blue mailbox stands beside it as scenery. This page gives every part's size, mass and position, the clearances, and the hover-error budget behind the mouth's size.</p>
  <div class="chips"><span class="chip ok">Simulation: 3 of 3 delivered</span><span class="chip wait">Hardware: not flown yet</span><span class="chip">Tightest margin +1 mm (pick grip depth)</span></div>
  <div class="spec">
    <div><span class="k">Envelope</span><span class="v">150 &times; 90 &times; 4 mm</span><span class="s">60 g, far end weighted, CoM 40 mm from it</span></div>
    <div><span class="k">Mailbox mouth</span><span class="v">190 &times; 90 mm</span><span class="s">centred, top edge 25 mm under the box top</span></div>
    <div><span class="k">Mailbox</span><span class="v">235 &times; 125 &times; 272.5 mm</span><span class="s">W &times; D &times; H, front flush with the table</span></div>
    <div><span class="k">Arm pose</span><span class="v">0, 36, 44, 90&#176;</span><span class="s">claw 10&#176; down, jaws close vertically</span></div>
    <div><span class="k">Insert Margin</span><span class="v">0.15 m</span><span class="s">level slide-in to the release</span></div>
  </div>
  <nav class="toc" aria-label="Contents"><span class="eyebrow">Contents</span>
    <ol>
      <li><a href="#layout">Field layout</a></li>
      <li><a href="#envelope">The envelope</a></li>
      <li><a href="#mailbox">The mailboxes and the mouth</a></li>
      <li><a href="#geometry">Pick and insert geometry</a></li>
      <li><a href="#arm">Arm pose and loads</a></li>
      <li><a href="#budget">Why the mouth is this size</a></li>
      <li><a href="#run">Running the mission</a></li>
      <li><a href="#sim">Simulation results</a></li>
      <li><a href="#open">Before the first flight</a></li>
    </ol>
  </nav>
</header>

<h2 id="layout">1. Field layout</h2>
<p>All positions are in the room frame of the mocap: <b>x</b> forward from the start mark (the drone's heading, kept at 0&#176; for the whole mission), <b>y</b> to the left, <b>z</b> up, in metres. The positions are the simulation scene's defaults and the yaml's drone points. On hardware the Envelope and Mailbox points come from Get, but the drone points (Start, Mailbox Start, Land Start, Land) are typed: if the table or the pillar moves, keep Mailbox Start on the line through the mouth, about 0.25 m behind the Ready To Insert hold, and leave the route clear of the pillar and the table.</p>
<figure>
  <div class="plate wide">{plan}</div>
  <figcaption><b>Figure 1. Plan view, to scale.</b> The vehicle is drawn at the start (solid) and at the pick and the release (dashed); its yellow arm line ends at the claw. Rotor discs reach 0.423 m ahead of the body, so at the release they pass over the red mailbox, 77 mm above its top. Cyan arrows are the legs between holds, drawn straight.</figcaption>
</figure>
{coords}

<h2 id="envelope">2. The envelope</h2>
<p>A 3D-printed core wrapped in red paper, heavy at the far end. The weight is what makes the insert possible. The mailbox is only 115 mm deep inside, so at the release three things must hold at once: the jaw tips stay outside the wall, the far end stays short of the back wall, and the centre of mass is already past the mouth's lip so the envelope falls in when the jaws open. A uniform envelope has its centre of mass at L/2 and cannot satisfy all three with &plusmn;20 mm of hover error. Moving the centre of mass to 40 mm from the far end can.</p>
<figure>
  <div class="stack">
    <div class="plate narrow">{etop}</div>
    <div class="plate narrow">{esec}</div>
  </div>
  <figcaption><b>Figure 2. The envelope, to scale (top) and in section with its thickness drawn three times larger (bottom).</b> The jaws take the near 31 mm, top and bottom. The mocap rigid body obj_0 has its pivot at the geometric centre, mid-thickness, x toward the weighted end, z up. Keep the bottom face flat: it rests on the hat and slides over the mouth's lower lip.</figcaption>
</figure>
<h3>Parts and masses</h3>
{bom}
<h3>Mass and balance rule</h3>
<p>With the end weight centred 15 mm from the far edge and the rest of the mass spread evenly along the 150 mm, the centre of mass lands 40 mm from the far end when <b>end weight = 1.4 &times; (core + wrap)</b>. A full 2 mm PLA core prints at about 28 g at the lab's measured print density (1.04 g/cm&#179;), so print it at 1.5 mm or with lower infill to stay near the 60 g design.</p>
{rule}
<p><b>Check before flight:</b> balance the finished envelope on a ruler's edge across its width. The balance point must be <b>no more than 48 mm from the far end</b>. Further from it, the centre of mass may not clear the lip at the worst fore-aft error. Weigh it too: 55 to 65 g is the flown range.</p>

<h2 id="mailbox">3. The mailboxes and the mouth</h2>
<figure>
  <div class="pair">
    <div class="plate">{front}</div>
    <div class="plate">{face}</div>
  </div>
  <figcaption><b>Figure 3. The table seen from the start (left) and the red mailbox's front face, to scale (right).</b> Looking along +x, +y is on the left, so the red target box appears on the left. The mouth sits high because the props pass over the box: lowering it would lower the drone and bring the props down onto the box top.</figcaption>
</figure>
{box}

<h2 id="geometry">4. Pick and insert geometry</h2>
<p>The same arm pose is used for the whole mission, so the claw approaches the pick and the mailbox the same way: level, from behind, along the drone's heading.</p>
<figure>
  <div class="plate wide">{pick}</div>
  <figcaption><b>Figure 4a. At the pick, to scale.</b> Ready To Pick holds the claw 100 mm behind the grasp point; Pick slides it in level. The jaw tips reach 31 mm into the envelope and stop 39 mm short of the hat's rim, so the lower jaw never passes over the hat. Arm joints, claw, props and gear height are to scale; the body outline is schematic.</figcaption>
</figure>
<figure>
  <div class="plate wide">{rel}</div>
  <figcaption><b>Figure 4b. At the release, to scale, cut through the red mailbox at the mouth.</b> The envelope is drawn at 10&#176; nose-down (the claw's own angle) and dashed at 20&#176;, the worst pitch assumed. Its weight rotates it in the pinch: the simulation measured 8 to 14&#176;. The claw sits 21.5 mm above the mouth's centre so that both cases pass the opening with room above and below.</figcaption>
</figure>
{clear}

<h2 id="arm">5. Arm pose and loads</h2>
{arm}
<p>J4's low stiffness does not limit this payload. With the wrist rolled 90&#176; and the envelope gripped by its edge, the envelope's centre of mass lies on the claw's own axis, so gravity barely loads J4. The only J4 load is the weight times the lateral grip offset. A lighter envelope also keeps the lift and release kicks on J2 and J3 small; the 190 g basket kicked q2 by 18 to 25&#176; at its release.</p>

<h2 id="budget">6. Why the mouth is this size</h2>
<p>The claw's hold error was measured on 13 hardware flights of this controller (10-09 to 10-21, the claw's forward kinematics on the encoders, odometry pose). The slide-in is not gated, so across and in height the mouth must pass the envelope at the measured worst case. The release and the pick are gated: the arm ground station opens or closes the gripper only with the claw within 20 mm of its target, which bounds the fore-aft error at those moments.</p>
{budget}
<div class="note">The two thinnest margins are the pick's grip depth (+1 mm: with a 20 mm fore-aft error short of the target the jaws still hold 10 mm of the envelope) and the side and height margins of the mouth (+5 mm). All are positive at the budget; none is large.</div>

<h2 id="run">7. Running the mission</h2>
<h3>Arm ground station, Mail Envelope tab, in order</h3>
<ol class="seq">{steps}</ol>
<h3>Setting the Mailbox point on hardware</h3>
<p>Lay the envelope flat on the mouth's lower lip, push it in until its far end touches the back wall, hold it level (its centre of mass is inside, so it would tip in), and press Get on the Mailbox row. The release is 36 mm short of that position and the claw rides 64.5 mm above an envelope lying on the lip, so the place point is <b>Get x &minus; 0.036, Get y, Get z + 0.065, yaw 0</b>. The tab's boxes show two decimals, so set it exactly from a terminal:</p>
<pre>ros2 param set /uav_0/whole_body_trajectory_planner pick_place_place_point "[&lt;x&minus;0.036&gt;, &lt;y&gt;, &lt;z+0.065&gt;, 0.0]"</pre>
<p>Rounded to &minus;0.04 and +0.06, the claw would sit 4.5 mm low and use up most of the 6 mm room below the mouth. Hover at least 6 s at Ready To Insert before pressing Insert: the descent trim averages the claw's error over 5 s on hardware.</p>
<h3>Launch</h3>
<pre># hardware (Orin): EKF2-fused whole-body stack with the mail block
AM_HW_PROFILE=mail_delivery start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator_fused.sh
# rebuild first: fsc_trajectory_planner on the Orin, utils_custom_ground_station on the laptop

# simulation (Command.md section 19)
WB_SIM_PROFILE=mail_delivery start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh shiqi_machine uav_0
WB_SIM_PROFILE=mail_delivery start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh shiqi_machine</pre>

<h2 id="sim">8. Simulation results</h2>
<p>Isaac Sim, headless, real time, the whole-body 4-D L1 controller and the arm ground station's sequence. Every flight on the final design put the envelope on the floor of the red mailbox, with 0 aborts. A first flight on an earlier mouth (80 mm tall) found the envelope pitches 7 to 18&#176; in the jaws; the mouth was then made 90 mm tall and the claw raised to its current height.</p>
{sim}
<figure>
  <img class="shot" src="isaac_scene.png" alt="Isaac Sim view of the scene: the T650 on its start mark, the pick pillar with the red envelope on its hat, and the console table with the red and blue mailboxes and their mouths." width="1440" height="900">
  <figcaption><b>Figure 5. The Isaac Sim scene.</b> The simulated claw cannot close below about 19 mm, so the envelope carries an invisible 20 mm grip pad on its near end, switched off when the jaws open at the mailbox. The real gripper's foam pads close on the 4 mm envelope directly.</figcaption>
</figure>
<figure>
  <img class="shot" src="gs_live_mail_tab.png" alt="The arm ground station's Mail Envelope tab with the Envelope, Mailbox Start and Mailbox rows, the Insert Margin box and the mission buttons." width="1743" height="1190">
  <figcaption><b>Figure 6. The arm ground station's Mail Envelope tab</b> (last tab), with the vehicle on the ground before Plan.</figcaption>
</figure>

<h2 id="open">9. Before the first flight</h2>
<ul>
  <li><b>Gripper opening.</b> Measure the real fingertip gap when open. The envelope is 4 mm thick and the vertical error budget is &plusmn;12 mm, so the open gap should be at least about 30 mm at the tips.</li>
  <li><b>Pitch in the foam pads.</b> Hold the envelope by its near 31 mm in the closed gripper and measure how far its far end droops. The design assumes 0 to 20&#176;. More than 20&#176; uses up the room below the mouth.</li>
  <li><b>Balance and mass.</b> Balance point no more than 48 mm from the far end; mass 55 to 65 g.</li>
  <li><b>Mailbox walls.</b> Measure the real wall thickness and box depth; every distance here is from the outer faces with 5 mm walls. Fix both boxes to the table so the drop cannot move them.</li>
  <li><b>Mocap body for the envelope.</b> Pivot at the geometric centre and mid-thickness, x toward the weighted end, z up, markers no taller than 4 mm on the top face between 40 and 90 mm from the near edge. At rest it must read level and steady, with no 180&#176; flips, before Get.</li>
  <li><b>Downwash over the box top.</b> The props pass 77 mm over the red box for about 3 s. Watch for a pitch oscillation like the 0.6 Hz one the plastic bin caused on 10-14, and abort if it grows.</li>
</ul>

<div class="foot">Every number on this page is computed by <code>docs/docs_aerial_manipulator/archive/mail_delivery_20261023/tools/mail_geometry.py</code> and drawn by <code>tools/build_design_page.py</code> in the same folder. The hold-error statistics are in <code>hover_error/README.md</code>; the full record is the folder's <code>README.md</code>.</div>
</div>
"""


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "design_page.html")
    with open(out, "w") as fh:
        fh.write(build())
    print("wrote", os.path.abspath(out))


if __name__ == "__main__":
    main()
