#!/usr/bin/env python3
"""Replay the planned CLEARANCE GUARD on recorded Isaac valve runs (2026-11-03): the distance from every
vehicle part EXCEPT the gripper (the wrist-to-claw link, which pushes the fin by design) to the valve's
fin, beam, hub and mocap marker balls -- with the quantities the planner has (odometry pose, the arm's
joint states through the model chain, the valve's mocap obj_0).

Vehicle (actual body frame, x forward; AM_T650.usda): two skids (capsules x -0.163..0.163, y +-0.1565,
r 0.0115, bottom at -GEAR_DEPTH), two struts (0, +-0.110, -0.05) -> (0, +-0.1565, skid centre), r 0.008,
the four rotor discs (centres (+-0.2299, +-0.2299, 0.1822), r 0.1905: rim + centre points), the arm links
base -> joint 2 -> joint 3 -> joint 4 (r 0.02; model chain, R_MODEL to the body frame).
Valve (rotor frame: axis on the plate bottom, +x to the fin): fin box (0.160, 0.0006, 0.139) +-(0.0025,
0.020, 0.065), beam (0.0875, 0, 0.047) +-(0.1125, 0.025, 0.007), hub (0, 0, 0.027) +-(0.025, 0.025,
0.027), the five marker balls (r 0.015, Valve_Assembly_v2.usda fsc:mocap_markers_m).
Prints per run and phase the smallest clearance and the pair; --gear 0.248 = the hardware gear.
Run: PYTHONNOUSERSITE=1 /usr/bin/python3 clearance_replay.py [--gear D] [runs/*.npz]"""
import sys, glob, os, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, *[".."] * 5, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP   # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import compatible_trajectory as CT  # noqa: E402

args = sys.argv[1:]
GEAR = 0.2425
if args and args[0] == "--gear":
    GEAR = float(args[1]); args = args[2:]
AXIS_OFFSET = np.array([-0.0935, -0.0014, -0.0899])
MARKERS = np.array([[0.00494, 0.01504, 0.09249], [0.04006, -0.01983, 0.12549], [0.10483, 0.04341, 0.07049],
                    [0.12515, -0.04252, 0.06849], [0.19245, 0.01072, 0.09249]])
BOXES = {"fin": ((0.160, 0.0006, 0.139), (0.0025, 0.020, 0.065)),
         "beam": ((0.0875, 0.0, 0.047), (0.1125, 0.025, 0.007)),
         "hub": ((0.0, 0.0, 0.027), (0.025, 0.025, 0.027))}
SK_R = 0.0115
sk_z = -(GEAR - SK_R)
CAPS = {"skid L": ((-0.163, 0.1565, sk_z), (0.163, 0.1565, sk_z), SK_R),
        "skid R": ((-0.163, -0.1565, sk_z), (0.163, -0.1565, sk_z), SK_R),
        "strut L": ((0.0, 0.110, -0.05), (0.0, 0.1565, sk_z), 0.008),
        "strut R": ((0.0, -0.110, -0.05), (0.0, -0.1565, sk_z), 0.008)}
ROTORS = [(0.2299, -0.2299), (-0.2299, 0.2299), (-0.2299, -0.2299), (0.2299, 0.2299)]
ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
ROTOR_PTS = np.array([(cx + r * np.cos(a), cy + r * np.sin(a), 0.1822)
                      for cx, cy in ROTORS for r in (0.0, 0.095, 0.1905) for a in (ang if r > 0 else [0.0])])
PARAMS = TP.make_params_t650()


def Rq(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def chain_points(q):
    Rm, pts = np.eye(3), [np.zeros(3)]
    for i in range(PARAMS["n"]):
        Rm = Rm @ CT._rot(PARAMS["h_i_im1"][i], q[i])
        pts.append(pts[-1] + Rm @ np.asarray(PARAMS["l_i"][i], float))
    return pts


def pt_box(P, c, h):            # P (k,3) in the box frame -> distance to the box (0 inside)
    d = np.maximum(np.abs(P - c) - h, 0.0)
    return np.linalg.norm(d, axis=1)


def seg_pts(a, b, n=21):
    s = np.linspace(0, 1, n)[:, None]
    return a + s * (b - a)


def clearance(xb, Rb, q, po, Ro):
    """min over (vehicle part, valve part) of the surface distance, and the pair."""
    axis = po + Ro @ AXIS_OFFSET
    toV = lambda W: (Ro.T @ (W - axis).T).T          # world -> valve (rotor) frame
    parts = {}
    for k, (a, b, r) in CAPS.items():
        parts[k] = (xb + (Rb @ seg_pts(np.array(a), np.array(b)).T).T, r)
    parts["rotors"] = (xb + (Rb @ ROTOR_PTS.T).T, 0.0)
    cp = [TP.R_MODEL @ p for p in chain_points(q)]
    for i, nm in enumerate(("arm base-j2", "arm j2-j3", "arm j3-j4")):
        parts[nm] = (xb + (Rb @ seg_pts(cp[i], cp[i + 1]).T).T, 0.02)
    bb, bm = (9.0, ""), (9.0, "")
    for nm, (W, r) in parts.items():
        V = toV(W)
        for bn, (c, h) in BOXES.items():
            d = pt_box(V, np.array(c), np.array(h)).min() - r
            if d < bb[0]:
                bb = (d, f"{nm} / {bn}")
        dm = np.linalg.norm(V[:, None, :] - MARKERS[None], axis=2).min() - r - 0.015
        if dm < bm[0]:
            bm = (dm, f"{nm} / marker")
    return bb, bm


if __name__ == "__main__":
    files = args or sorted(glob.glob(os.path.join(HERE, "..", "sim", "runs", "*.npz")))
    print(f"gear depth {GEAR} m | {'run':20s} {'phase':6s} min clearance (mm), the pair")
    for f in files:
        z = np.load(f, allow_pickle=True)
        od, js, bx = z["odom"], z["joints"], z["box"]
        mt = dict(zip(z["marks_name"], z["marks_t"]))
        name = os.path.basename(f)[:-4]
        segs = [("ready", mt.get("ready:start"), mt.get("ready:end")),
                ("hold", mt.get("ready:end"), mt.get("push:start")),
                ("turn", mt.get("push:start"), mt.get("push:end")),
                ("exit", mt.get("exit:start"), mt.get("exit:end"))]
        ab = [t for n_, t in mt.items() if "abort" in n_]
        if ab:
            segs.append(("abort", ab[0], ab[0] + 6.0))
        for nm, a, b in segs:
            if a is None or b is None:
                continue
            bb, bm = (9.0, ""), (9.0, "")
            for t in np.arange(a, b, 0.05):
                i = min(np.searchsorted(od[:, 0], t), len(od) - 1)
                j = min(np.searchsorted(js[:, 0], t), len(js) - 1)
                k = min(np.searchsorted(bx[:, 0], t), len(bx) - 1)
                cb, cm = clearance(od[i, 1:4], Rq(od[i, 7:11]), js[j, 1:5], bx[k, 1:4], Rq(bx[k, 4:8]))
                if cb[0] < bb[0]:
                    bb = (cb[0], cb[1] + f" @ +{t - a:.1f} s")
                if cm[0] < bm[0]:
                    bm = (cm[0], cm[1] + f" @ +{t - a:.1f} s")
            print(f"{'':4s} {name:20s} {nm:6s} boxes {bb[0]*1e3:7.1f} {bb[1]:34s} | markers {bm[0]*1e3:7.1f} {bm[1]}")
