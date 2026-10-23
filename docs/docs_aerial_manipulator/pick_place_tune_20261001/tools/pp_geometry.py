#!/usr/bin/env python3
"""Offline collision geometry of the six pick-and-place legs on 07's scene.

    /usr/bin/python3 pp_geometry.py [--pick-pose 0 -15 15 0] [--carry-pose ...]
        [--place-pose ...] [--pick-off 0 0 0.211] [--place-off 0 0 0.231]
        [--sweep-lo -25 -35 10 0 --sweep-hi 25 -10 35 0 --sweep-phase 0 90 270 0]
        [--approach-dz 0.10] [--start-xy 0.10 -0.07] [--plot out.png]

Rebuilds the planner's goals exactly as fsc_trajectory_planner/pick_place.cpp
does (baseRest / clawRest / retreatRest, the drone setpoints shifted by the
Adjust offset = the hover position), plans the transition legs with the
parity-locked Python flat B-spline planner and the execute_place leg with a
port of ArmSweepTrajectory::flat(), then sweeps the GRIPPED PAYLOAD (rigid in
the claw frame from the grasp on) and the open gripper against the two
pillars, the payload on its pillar, and the vehicle's landing gear.

Geometry (07 + its probes): pillar r 0.05, top z 1.0. Payload = box
110 x 110 x 65 mm + handle 30 (closing axis) x 60 (along the claw's long
axis) x 200 mm, the claw point `grasp_below_top` below the handle top.
Attitude is taken as heading only (tilts at a_max 0.15 are < 1 deg).

Gripper (07's probes, 2026-10-02): pad inner faces +-21.65 mm off the claw
axis, spanning 15 mm below to 51 mm above the claw point; the wrist link is
the first body above, 62 mm over the claw point. Landing gear: two skids at
|y| 0.11..0.17 m, x +-0.17 m, bottom 0.302 m under the body origin.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser("~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import compatible_trajectory as CT  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import flat_bspline_planner as FB  # noqa: E402

D = math.pi / 180.0
PICK_XY, PLACE_XY = np.array([1.0, 1.0]), np.array([-1.0, -1.0])
PILLAR_R, PILLAR_TOP = 0.05, 1.0
BOX = np.array([0.055, 0.055, 0.0325])          # half extents (c, l, up)
HANDLE = np.array([0.010, 0.030, 0.100])          # 20 mm handle (07 default since 2026-10-02)
HANDLE_H = 0.200
PAD_C = (0.02165, 0.02865)                       # pad slab across the closing axis [m] (7 mm assumed)
PAD_L = 0.012                                    # pad half-width along the handle (assumed)
PAD_H = (-0.015, 0.051)                          # pad faces vs the claw point, + = up the claw axis
WRIST_H, WRIST_HALF = 0.062, 0.025               # first body above the pads (probe), half-size assumed
GEAR = [(-0.17, 0.17, 0.11, 0.17, -0.31, -0.20), (-0.17, 0.17, -0.17, -0.11, -0.31, -0.20)]
LEGS = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place",
        "go_to_land_start", "execute_land"]


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def kin(q, P):
    r0c, r0e = CT._arm_kin(np.asarray(q, float), P)
    Re = np.eye(3)
    for i in range(P["n"]):
        Re = Re @ CT._rot(P["h_i_im1"][i], q[i])
    return r0c, r0e, Re


def base_rest(xyzyaw, q):
    return {"x_b": np.array(xyzyaw[:3], float), "phi": xyzyaw[3] * D - 0.5 * math.pi,
            "q": np.asarray(q, float)}


def claw_rest(P, q, ee, frm):
    _, r0e, _ = kin(q, P)
    dx, dy = ee[0] - frm["x_b"][0], ee[1] - frm["x_b"][1]
    phi = frm["phi"]
    if math.hypot(dx, dy) > 1e-3:
        reach = math.hypot(r0e[0], r0e[1])
        alpha = math.atan2(r0e[1], r0e[0]) if reach > 1e-3 else 0.5 * math.pi
        phi = math.atan2(dy, dx) - alpha
    return {"x_b": ee - Rz(phi) @ r0e, "phi": phi, "q": np.asarray(q, float)}


def retreat(r, dz, back):
    nose = np.array([-math.sin(r["phi"]), math.cos(r["phi"]), 0.0])
    return dict(r, x_b=r["x_b"] + np.array([0, 0, dz]) - back * nose)


def nonic(u):
    u = min(1.0, max(0.0, u))
    return u ** 5 * (126 + u * (-420 + u * (540 + u * (-315 + u * 70))))


def sweep_samples(P, r0, r1, lo, hi, phase, cycles=2, ramp_frac=0.2, n=801):
    """(x_b, phi, q) along ArmSweepTrajectory, u = t/T in [0, 1] (geometry is T-free)."""
    c0, _, _ = kin(r0["q"], P)
    c1, _, _ = kin(r1["q"], P)
    xc0, xc1 = r0["x_b"] + Rz(r0["phi"]) @ c0, r1["x_b"] + Rz(r1["phi"]) @ c1
    phi0 = r0["phi"]
    phi1 = r1["phi"] + 2 * math.pi * round((phi0 - r1["phi"]) / (2 * math.pi))
    out = []
    for u in np.linspace(0.0, 1.0, n):
        s = nonic(u)
        if u < ramp_frac:
            w = nonic(u / ramp_frac)
        elif u > 1 - ramp_frac:
            w = nonic((1 - u) / ramp_frac)
        else:
            w = 1.0
        q = np.zeros(4)
        for j in range(4):
            ql = r0["q"][j] + (r1["q"][j] - r0["q"][j]) * s
            if hi[j] - lo[j] <= 1e-12:
                q[j] = ql
                continue
            c, a = 0.5 * (lo[j] + hi[j]), 0.5 * (hi[j] - lo[j])
            arg = 2 * math.pi * cycles * (u - ramp_frac) / (1 - 2 * ramp_frac) + phase[j]
            q[j] = (1 - w) * ql + w * (c + a * math.sin(arg))
        phi = phi0 + (phi1 - phi0) * s
        xc = xc0 + (xc1 - xc0) * s
        r0c, _, _ = kin(q, P)
        out.append((xc - Rz(phi) @ r0c, phi, q))
    return out


def bspline_samples(P, r0, r1, opts, n=801):
    plan = FB.plan_flat_transition(P, r0, r1, opts)
    out = []
    for t in np.linspace(0.0, plan["T"], n):
        ref = plan["ref"](t)
        b1 = ref["b1_d"]
        phi = math.atan2(b1[1], b1[0])
        q = np.asarray(ref["q_d"], float)
        r0c, _, _ = kin(q, P)
        out.append((ref["x_cd"] - Rz(phi) @ r0c, phi, q))
    return out, plan["T"]


def claw_frame(P, x_b, phi, q):
    """claw point, claw axis a (out, = down when claw-down), closing axis c, long axis l (world)."""
    _, r0e, Re = kin(q, P)
    R = Rz(phi) @ Re
    return x_b + Rz(phi) @ r0e, R @ np.array([0, 0, -1.0]), R @ np.array([1.0, 0, 0]), R @ np.array([0, 1.0, 0])


def box_points(center, axes, half, step=0.005):
    """Surface samples of an oriented box (axes columns = its unit axes)."""
    pts = []
    for k in range(3):
        i, j = [m for m in range(3) if m != k]
        ni, nj = max(2, int(2 * half[i] / step) + 1), max(2, int(2 * half[j] / step) + 1)
        for si in np.linspace(-half[i], half[i], ni):
            for sj in np.linspace(-half[j], half[j], nj):
                for sk in (-half[k], half[k]):
                    v = np.zeros(3)
                    v[i], v[j], v[k] = si, sj, sk
                    pts.append(center + axes @ v)
    return np.array(pts)


def payload_points(claw, a, c, l, below_top):
    """Box + handle surface samples, payload rigid in the claw frame (upright when a = down)."""
    up = -a
    axes = np.column_stack([c, l, up])
    box_top = claw - (HANDLE_H - below_top) * up          # box top below the claw
    box = box_points(box_top - BOX[2] * up, axes, BOX)
    handle = box_points(box_top + HANDLE[2] * up, axes, HANDLE)
    return box, handle


def gripper_points(claw, a, c, l):
    """Surface samples of the two pads and the wrist block, world frame."""
    up = -a
    axes = np.column_stack([c, l, up])
    pts = []
    for sgn in (1.0, -1.0):
        cc = 0.5 * (PAD_C[0] + PAD_C[1]) * sgn
        half = np.array([0.5 * (PAD_C[1] - PAD_C[0]), PAD_L, 0.5 * (PAD_H[1] - PAD_H[0])])
        pts.append(box_points(claw + axes @ np.array([cc, 0.0, 0.5 * (PAD_H[0] + PAD_H[1])]), axes, half, 0.002))
    pts.append(box_points(claw + axes @ np.array([0.0, 0.0, WRIST_H + 0.02]), axes,
                          np.array([WRIST_HALF, WRIST_HALF, 0.02]), 0.003))
    return np.vstack(pts)


def box_signed(pts, center, axes, half):
    """Signed distance of points to an oriented box (< 0 = inside)."""
    p = (pts - center) @ axes
    q = np.abs(p) - half
    out = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(np.max(q, axis=1), 0.0)
    return float((out + inside).min())


def pillar_clearance(pts, xy):
    """Signed clearance of points to a vertical cylinder (r, top) at xy; < 0 = inside."""
    hd = np.hypot(pts[:, 0] - xy[0], pts[:, 1] - xy[1])
    dz = pts[:, 2] - PILLAR_TOP
    over = hd < PILLAR_R
    cl = np.where(over, dz, np.where(dz < 0, hd - PILLAR_R, np.hypot(hd - PILLAR_R, dz)))
    return float(cl.min())


def gear_clearance(pts, x_b, phi, gear):
    """Min distance of points to the gear boxes (body frame, x = nose, y = left), < 0 = inside."""
    R = Rz(phi + 0.5 * math.pi)                          # actual yaw: body x = nose
    pb = (pts - x_b) @ R
    best = 1e9
    for (x0, x1, y0, y1, z0, z1) in gear:
        dx = np.maximum.reduce([x0 - pb[:, 0], pb[:, 0] - x1, np.zeros(len(pb))])
        dy = np.maximum.reduce([y0 - pb[:, 1], pb[:, 1] - y1, np.zeros(len(pb))])
        dz = np.maximum.reduce([z0 - pb[:, 2], pb[:, 2] - z1, np.zeros(len(pb))])
        d = np.sqrt(dx * dx + dy * dy + dz * dz)
        inside = (dx == 0) & (dy == 0) & (dz == 0)
        if inside.any():
            pen = np.minimum.reduce([pb[:, 0] - x0, x1 - pb[:, 0], pb[:, 1] - y0, y1 - pb[:, 1],
                                     pb[:, 2] - z0, z1 - pb[:, 2]])
            d = np.where(inside, -pen, d)
        best = min(best, float(d.min()))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pick-pose", type=float, nargs=4, default=[0, -15, 15, 0])
    ap.add_argument("--place-pose", type=float, nargs=4, default=None)
    ap.add_argument("--carry-pose", type=float, nargs=4, default=[0, -30, 30, 0])
    ap.add_argument("--home", type=float, nargs=4, default=[0, 40, 40, 0])
    ap.add_argument("--pick-off", type=float, nargs=3, default=[0, 0, 0.211])
    ap.add_argument("--place-off", type=float, nargs=3, default=[0, 0, 0.231])
    ap.add_argument("--grasp-below-top", type=float, default=0.054,
                    help="claw point below the handle top at the grasp (07's GRASP_BELOW_TOP)")
    ap.add_argument("--sweep-lo", type=float, nargs=4, default=[-25, 10, 0, 0])
    ap.add_argument("--sweep-hi", type=float, nargs=4, default=[25, 45, 0, 0])
    ap.add_argument("--sweep-phase", type=float, nargs=4, default=[0, 90, 0, 0])
    ap.add_argument("--sweep-cycles", type=int, default=2)
    ap.add_argument("--start-xy", type=float, nargs=2, default=[0.10, -0.07],
                    help="hover position at Adjust (= the start's shift)")
    ap.add_argument("--retreat", type=float, nargs=2, default=[0.15, 0.20], help="dz, back")
    ap.add_argument("--approach-dz", type=float, default=0.0,
                    help="pick_place_approach_dz: claw legs via a point this far above the goal")
    ap.add_argument("--gear", type=float, nargs=6, action="append", default=None,
                    help="gear box x0 x1 y0 y1 z0 z1 in the body frame (repeatable)")
    ap.add_argument("--plot", default="")
    a = ap.parse_args()
    place_pose = a.place_pose if a.place_pose is not None else a.pick_pose
    gear = a.gear or GEAR

    P = TP.make_params_t650(base_com=[0.0, -0.017854, 0.0], armature_diag=[0.010, 0.0194, 0.0097, 0.0097])
    opts = {"v_max": 0.30, "a_max": 0.15, "w_max": 0.30, "dw_max": 0.60, "tau_joint_max": 3.0}
    q = {k: np.array(v) * D for k, v in dict(home=a.home, pick=a.pick_pose, place=place_pose,
                                                carry=a.carry_pose).items()}
    for k in ("pick", "place", "carry"):
        print(f"{k:6s} pose {np.round(q[k] / D, 1).tolist()} deg  sigma_nd {TP._sigma_nd(q[k], P):.3f}")
    off = np.array([a.start_xy[0], a.start_xy[1], 0.0])
    pick = np.array([*PICK_XY, PILLAR_TOP]) + np.array(a.pick_off)
    place = np.array([*PLACE_XY, PILLAR_TOP]) + np.array(a.place_off)
    g0 = base_rest([0 + off[0], 0 + off[1], 1.0, 0.0], q["home"])
    g1 = claw_rest(P, q["pick"], pick, g0)
    g2 = base_rest([0 + off[0], -1 + off[1], 1.0, 0.0], q["carry"])
    g3 = claw_rest(P, q["place"], place, g2)
    g4 = base_rest([-1 + off[0], 0 + off[1], 1.0, 0.0], q["home"])
    for name, g in (("pick", g1), ("place", g3)):
        claw, ax, c, l = claw_frame(P, g["x_b"], g["phi"], g["q"])
        print(f"{name} goal: body {np.round(g['x_b'], 3).tolist()} yaw {(g['phi'] / D + 90):.1f} deg, "
              f"claw {np.round(claw, 3).tolist()}, claw axis {np.round(ax, 3).tolist()}")

    # the payload resting on the pick pillar (static until the grasp)
    up = np.array([0, 0, 1.0])
    claw1, a1, c1, l1 = claw_frame(P, g1["x_b"], g1["phi"], g1["q"])
    rest_claw = np.array([*PICK_XY, PILLAR_TOP + 2 * BOX[2] + HANDLE_H - a.grasp_below_top])
    rest_box, rest_handle = payload_points(rest_claw, -up, c1, l1, a.grasp_below_top)
    print(f"payload at rest: handle top {PILLAR_TOP + 2 * BOX[2] + HANDLE_H:.3f} m; designed grasp claw "
          f"z {rest_claw[2]:.3f}, planned claw z {claw1[2]:.3f} (pick offset z {a.pick_off[2]:.3f})")

    def raised(r, dz):
        return dict(r, x_b=r["x_b"] + np.array([0.0, 0.0, dz]))

    legs = []
    if a.approach_dz > 0:
        s1, T1 = bspline_samples(P, g0, raised(g1, a.approach_dz), opts)
        s2, T2 = bspline_samples(P, raised(g1, a.approach_dz), g1, opts)
        s, T = s1 + s2, T1 + T2
    else:
        s, T = bspline_samples(P, g0, g1, opts)
    legs.append(("execute_pick", s, T, False))
    via = retreat(g1, *a.retreat)
    s1, T1 = bspline_samples(P, g1, via, opts)
    s2, T2 = bspline_samples(P, via, g2, opts)
    legs.append(("go_to_place_start", s1 + s2, T1 + T2, True))
    g3v = raised(g3, a.approach_dz) if a.approach_dz > 0 else g3
    s = sweep_samples(P, g2, g3v, np.array(a.sweep_lo) * D, np.array(a.sweep_hi) * D,
                      np.array(a.sweep_phase) * D, a.sweep_cycles)
    if a.approach_dz > 0:
        s2, _ = bspline_samples(P, g3v, g3, opts)
        s = s + s2
    legs.append(("execute_place", s, float("nan"), True))
    via = retreat(g3, *a.retreat)
    s1, T1 = bspline_samples(P, g3, via, opts)
    s2, T2 = bspline_samples(P, via, g4, opts)
    legs.append(("go_to_land_start", s1 + s2, T1 + T2, False))

    # payload pose in the claw frame at the grasp (rigid from then on)
    hz_top = PILLAR_TOP + 2 * BOX[2] + HANDLE_H
    h_axes = np.column_stack([c1, l1, up])
    h_center = np.array([*PICK_XY, hz_top - HANDLE[2]])
    b_center = np.array([*PICK_XY, PILLAR_TOP + BOX[2]])
    print("\nleg                  T [s] | payload vs PICK pillar | vs PLACE pillar | vs gear | "
          "gear vs payload-at-rest | claw tilt max | gripper vs handle / box")
    tracks = {}
    for name, samples, T, carrying in legs:
        cl_pick = cl_place = cl_gear = cl_gear_rest = cl_grip = 1e9
        tilt = 0.0
        tr = []
        for x_b, phi, qq in samples[::4]:
            claw, ax, c, l = claw_frame(P, x_b, phi, qq)
            tilt = max(tilt, math.degrees(math.acos(max(-1, min(1, -ax[2])))))
            tr.append((claw, x_b, ax))
            if carrying:
                box, handle = payload_points(claw, ax, c, l, a.grasp_below_top)
                pts = np.vstack([box, handle])
                cl_pick = min(cl_pick, pillar_clearance(pts, PICK_XY))
                cl_place = min(cl_place, pillar_clearance(pts, PLACE_XY))
                cl_gear = min(cl_gear, gear_clearance(pts, x_b, phi, gear))
            else:
                cl_gear_rest = min(cl_gear_rest, gear_clearance(np.vstack([rest_box, rest_handle]), x_b, phi, gear))
                if name == "execute_pick":
                    gp = gripper_points(claw, ax, c, l)
                    cl_grip = min(cl_grip, box_signed(gp, h_center, h_axes, HANDLE),
                                  box_signed(gp, b_center, h_axes, BOX))
        tracks[name] = tr
        f = lambda v: f"{v * 1e3:+7.0f} mm" if v < 1e8 else "      -   "   # noqa: E731
        print(f"{name:18s} {T:6.1f} | {f(cl_pick):>22s} | {f(cl_place):>15s} | {f(cl_gear):>7s} | "
              f"{f(cl_gear_rest):>23s} | {tilt:6.1f} deg      | {f(cl_grip)}")

    # the final approach of the pick: claw relative to the handle (handle frame c, l, up)
    print("\nexecute_pick, last 15 cm of claw travel, in the HANDLE frame (c = closing axis, "
          "l = along the handle, z = up from the designed grasp point):")
    tr = tracks["execute_pick"]
    goal = tr[-1][0]
    dist = np.array([np.linalg.norm(p - goal) for p, _, _ in tr])
    for target in (0.15, 0.10, 0.06, 0.03, 0.0):
        i = int(np.argmin(np.abs(dist - target)))
        d = tr[i][0] - rest_claw
        print(f"  {dist[i] * 1e3:5.0f} mm out: c {d @ c1 * 1e3:+6.1f}  l {d @ l1 * 1e3:+6.1f}  "
              f"z {d[2] * 1e3:+6.1f} mm")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(12, 5))
        for name, tr in tracks.items():
            p = np.array([t[0] for t in tr])
            b = np.array([t[1] for t in tr])
            ax[0].plot(p[:, 0], p[:, 1], label=f"{name} claw")
            ax[0].plot(b[:, 0], b[:, 1], ":", lw=0.8)
            h = np.hypot(p[:, 0] - PLACE_XY[0], p[:, 1] - PLACE_XY[1]) if name == "execute_place" else None
            if h is not None:
                ax[1].plot(h, p[:, 2] - (HANDLE_H - a.grasp_below_top) - 2 * BOX[2], label="box bottom")
        for xy in (PICK_XY, PLACE_XY):
            ax[0].add_patch(plt.Circle(xy, PILLAR_R, color="grey"))
        ax[0].set_aspect("equal")
        ax[0].legend(fontsize=7)
        ax[1].axhline(PILLAR_TOP, color="grey")
        ax[1].axvline(PILLAR_R + BOX[0], color="grey", ls=":")
        ax[1].set_xlabel("horizontal distance, claw to place pillar axis [m]")
        ax[1].set_ylabel("box bottom z [m]")
        ax[1].legend()
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print(f"plot: {a.plot}")


if __name__ == "__main__":
    main()
