#!/usr/bin/env python3
"""The hinged valve handle (2026-10-30): the numbers behind the design on the Valve Turning Design page.

Frame: the valve's (mm): origin on the bearing axis at the plate bottom, +x along the beam to the handle,
+z up; the turn is +90 deg, so the vehicle's nose (and the claw's push) is +y. Geometry from
Valve_Assembly_v2.stl: fin x 157.5..162.5 (5 mm), y -19.4..20.6, z 74..204; root x 155..165, y -24.4..25.6,
z 54..74; marker E's ball centre (192.5, 10.7, 92.5) r 15. The grip point G = the planner's
push_pull_ee_offset (0.160, 0.0006, 0.179) m. The claw axis at the flown pose [0, 5, 25, 0] points along
the nose and 60 deg down (handle_joint_mobility.py).

    PYTHONNOUSERSITE=1 /usr/bin/python3 -I handle_design.py
"""
import numpy as np

G = np.array([160.0, 0.6, 179.0])
U = np.array([0.0, np.cos(np.radians(30)), np.sin(np.radians(30))])    # the pin axis
C = np.array([0.0, np.cos(np.radians(-60)), np.sin(np.radians(-60))])  # the claw axis
X = np.array([1.0, 0.0, 0.0])
K_Y = 120.0            # [N/m] the law's EE task stiffness (valve yaml wb_ky_x/y/z)
GRIP_ERR = 20.0        # [mm] the GS close gate: the claw within 20 mm of G (8 mm across)
FINGER_W, FINGER_BACK, FINGER_TIP, FINGER_T = 20.0, 40.0, 20.0, 4.0   # assumed OM-X finger [mm]

PADDLE = dict(t=6.0, y=(-19.4, 20.6), z=(124.0, 204.0))
BOSS = dict(d=12.0, s=(-27.0, -16.0))
# the post on the CLAW side (-y): the pin line descends that way, so the post sits low, under the
# finger bodies (which rise toward the wrist) and away from the fingertips (which point +y, down)
HOUSING = dict(d=16.0, s=(-46.0, -36.0))
COLUMN = dict(x=(154.0, 166.0), y=(-47.0, -35.0), z=(54.0, 156.0))
APPROACH = np.array([0.0, -120.0, 100.0])   # push_pull_approach_back 0.12 m, approach_dz 0.10 m
WEIGHT = dict(m=0.010, p=np.array([160.0, 0.6, 130.0]))
PLA = 1.24e-3 * 0.5    # [g/mm^3] printed at ~50 % infill


def on_axis(s):
    return G + s * U


def rot(axis, a):
    k = axis / np.linalg.norm(axis); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def main():
    print("THE AXIS: through G", G, "along", U.round(3), "(30 deg up toward +y, square to the claw axis", C.round(3), ")")
    for s in (BOSS["s"][0], BOSS["s"][1], HOUSING["s"][0], HOUSING["s"][1]):
        print(f"  s {s:5.1f} mm -> {on_axis(s).round(1)}")
    top = on_axis(HOUSING["s"][1])[2] + HOUSING["d"] / 2 * abs(C[2])
    print(f"  housing top z {top:.1f} mm (the old fin top: 204.0; the grip point 179.0)")

    print("\nAXIS OFFSET from G: the claw point must move d*theta, against the task spring -> K_y d^2")
    for d in (0, 10, 20, 30, 105):
        print(f"  d {d:3d} mm: {K_Y * (d * 1e-3) ** 2:.3f} N.m/rad" + ("   (a hinge at the fin's base)" if d == 105 else ""))

    m_p = PLA * PADDLE["t"] * np.ptp(PADDLE["y"]) * np.ptp(PADDLE["z"])
    cg = np.array([160.0, np.mean(PADDLE["y"]), np.mean(PADDLE["z"])])
    d_p = abs((cg - G) @ C); d_w = abs((WEIGHT["p"] - G) @ C)
    k_p = U[1] * m_p * 1e-3 * 9.81 * d_p * 1e-3     # cos(30 deg) = the axis's horizontal share
    k_w = U[1] * WEIGHT["m"] * 9.81 * d_w * 1e-3
    print(f"\nGRAVITY CENTRING: paddle {m_p:.1f} g, CoM {d_p:.1f} mm below the axis -> {1e3 * k_p:.2f} mN.m/rad;"
          f" + {1e3 * WEIGHT['m']:.0f} g at its bottom ({d_w:.1f} mm) -> {1e3 * k_w:.2f}; total {1e3 * (k_p + k_w):.2f} mN.m/rad")
    for f in (0.2, 0.5, 1.0):
        print(f"  bearing friction {f} mN.m -> rests within +-{np.degrees(f * 1e-3 / (k_p + k_w)):.1f} deg")

    # clearances: the fingers' in-plane footprint (along U, along C) shifted by an in-plane error,
    # against the post (housing + column), projected on the fin plane (the fingers overlap its x)
    def finger_poly(e):
        pts = []
        for a in (-FINGER_W / 2, FINGER_W / 2):
            for b in (-FINGER_BACK, FINGER_TIP):
                pts.append(G + e + a * U + b * C)
        return np.array(pts)[:, 1:]

    def post_pts():
        pts = []
        for s in np.linspace(*HOUSING["s"], 6):
            for a in np.linspace(0, 2 * np.pi, 24, endpoint=False):
                p = on_axis(s) + HOUSING["d"] / 2 * (np.cos(a) * X + np.sin(a) * C)
                if abs(p[0] - 160) < 7.5:
                    pts.append(p[1:])
        for y in np.linspace(*COLUMN["y"], 7):
            for z in np.linspace(*COLUMN["z"], 40):
                pts.append([y, z])
        return np.array(pts)

    def dist_poly(P, Q):
        """min distance of points Q to the convex quad P (y, z); 0 if inside."""
        from itertools import combinations
        hull = P[[0, 1, 3, 2]]
        best = np.inf
        for q in Q:
            inside = True
            for i in range(4):
                a, b = hull[i], hull[(i + 1) % 4]
                cr = (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0])
                if cr < 0:
                    inside = False
            if inside:
                return 0.0
            for i in range(4):
                a, b = hull[i], hull[(i + 1) % 4]
                t = np.clip((q - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
                best = min(best, np.linalg.norm(q - (a + t * (b - a))))
        return best

    Q = post_pts()
    print("\nCLEARANCE, finger (20 mm wide, 40 mm back to the wrist, 20 mm past G) to the post,"
          " at the grasp (jaws closed) and along the Ready approach (from 120 mm behind, 100 mm above):")
    for err in (0.0, 10.0, GRIP_ERR):
        d_end = d_path = np.inf
        for a in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            e = err * (np.cos(a) * U + np.sin(a) * C)
            d_end = min(d_end, dist_poly(finger_poly(e), Q))
            for t in np.linspace(0, 1, 26):
                d_path = min(d_path, dist_poly(finger_poly(e + t * APPROACH), Q))
        print(f"  claw {err:4.1f} mm off G (worst direction): {d_end:5.1f} mm at the grasp, {d_path:5.1f} mm on the way in")
    # on the way in the jaws are OPEN (37 mm apart): the post (16 mm wide) passes between the fingers,
    # and only the gripper body behind them (assumed 30 mm wide along the pin, from 40 to 80 mm back)
    # spans the whole width
    def body_poly(e):
        pts = [G + e + a * U + b * C for a in (-15.0, 15.0) for b in (-80.0, -40.0)]
        return np.array(pts)[:, 1:]
    for err in (0.0, GRIP_ERR):
        d_path = np.inf
        for a in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            e = err * (np.cos(a) * U + np.sin(a) * C)
            for t in np.linspace(0, 1, 26):
                d_path = min(d_path, dist_poly(body_poly(e + t * APPROACH), Q))
        print(f"  gripper body, claw {err:4.1f} mm off: {d_path:5.1f} mm from the post all the way in (jaws open: the post fits between the fingers)")
    print(f"  the pads reach s = -{FINGER_W / 2 + GRIP_ERR:.0f} mm along the pin at the worst; the boss starts at s = {BOSS['s'][1]:.0f}"
          " (pads on the boss are fine: it is part of the paddle, on the axis)")

    # the paddle's swing: corners out of plane at +-20 deg, vs marker E's ball and the root top
    corners = [np.array([x, y, z]) for x in (157.0, 163.0) for y in PADDLE["y"] for z in PADDLE["z"]]
    corners.append(WEIGHT["p"])
    print("\nSWING +-20 deg about the axis:")
    for ang in (-20, 20):
        Rm = rot(U, np.radians(ang))
        P = np.array([G + Rm @ (p - G) for p in corners])
        print(f"  {ang:+d} deg: x {P[:, 0].min():.1f}..{P[:, 0].max():.1f}, z min {P[:, 2].min():.1f} mm")
    E = np.array([192.5, 10.7, 92.5])
    print(f"  marker E's ball: x >= {E[0] - 15:.1f}, z {E[2] - 15:.1f}..{E[2] + 15:.1f}; the root top z 74")

    # loads: the turn's push at the force guard 5 N along +y
    F = 5.0
    print(f"\nLOADS at the force guard ({F} N along +y): along the pin {F * U[1]:.2f} N (pulls the pin out of the post: an E-clip takes it),"
          f" across {F * U[2]:.2f} N; pin bending at the near bearing "
          f"{F * abs(HOUSING['s'][1] - np.mean(BOSS['s'])) * 1e-3:.3f} N.m -> "
          f"{32 * F * abs(HOUSING['s'][1] - np.mean(BOSS['s'])) * 1e-3 / (np.pi * 0.004 ** 3) / 1e6:.0f} MPa in a 4 mm steel pin")


if __name__ == "__main__":
    main()
