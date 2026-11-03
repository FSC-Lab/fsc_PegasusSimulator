#!/usr/bin/env python3
"""U-push geometry screen (2026-11-02, archive/valve_upush_20261102).

With the jaws NOT closed on the fin, the only part of the claw that can push the fin along the
turn is material that crosses the fin's mid-plane between the claws: the bracket (link5 = bracket +
hub crank + rods, baked). For each arm pose (q1 = q4 = 0) and grasp depth d (the claw point d below
the fin top), straight-line approximation of the fin (aligned with the claw's heading):
  - the first bracket point the fin's BACK edge meets as the claw moves forward (within the fin's height),
  - the slack (the claw's travel before it touches), how far below the fin top it touches,
  - the push direction (the bracket's surface normal there, in the side view),
  - the landing gear above the beam top, joint margins, sigma_nd.
Run with PYTHONNOUSERSITE=1 /usr/bin/python3 -I.
"""
import os, sys, struct, numpy as np
sys.path.insert(0, os.path.expanduser(
    "~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_planner"))
import transition_planner as TP

D = os.path.expanduser("~/ros2_ws/src/fsc_open_manipulator/open_manipulator_x_description/meshes/custom/")
def load(fn):
    b = open(D + fn, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float) * 1e-3
TRI = load("link5.stl")                                    # the bracket + hub + rods, link5 frame [m]
N = np.cross(TRI[:, 1] - TRI[:, 0], TRI[:, 2] - TRI[:, 0])
N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
rng = np.random.default_rng(0); K = 40
u = rng.random((K, 2)); u = np.where(u.sum(1, keepdims=True) > 1, 1 - u, u)
S = (TRI[:, None, 0] + u[None, :, :1] * (TRI[:, None, 1] - TRI[:, None, 0])
     + u[None, :, 1:] * (TRI[:, None, 2] - TRI[:, None, 0]))          # (ntri, K, 3)
SN = np.repeat(N[:, None, :], K, axis=1)
S = S.reshape(-1, 3); SN = SN.reshape(-1, 3)
FIN_T, FIN_L, FIN_H = 0.005, 0.040, 0.130
GRASP_Z = 0.108                                            # link5 z of the claw point
P = TP.make_params_t650()
GEAR = 0.2425; BEAM_TOP = 0.054; FIN_TOP = 0.204           # above the plate bottom (= the table top)

def side(Q, p):            # link5 points -> (along the turn, up) about the claw point, claw p rad below level
    a = np.array([np.cos(p), -np.sin(p)]); b = np.array([np.sin(p), np.cos(p)])
    return (Q[:, 2] - GRASP_Z) * a[0] + Q[:, 0] * b[0], (Q[:, 2] - GRASP_Z) * a[1] + Q[:, 0] * b[1]

def screen(q_deg, d, s=0.0, y_off=0.0):
    q = np.radians(q_deg)
    _, r0e, re = TP.arm_fk_model(q, P)
    ax = -re[:, 2]                                        # claw axis (wrist -> claw point), model frame
    p = float(np.arctan2(-ax[2], np.hypot(ax[0], ax[1])))  # pitch below level
    down = -r0e[2]
    m = np.abs(S[:, 1] - y_off) < FIN_T / 2               # bracket material in the fin's slab
    fw, up = side(S[m], p)
    inspan = (up <= d) & (up >= d - FIN_H)
    out = dict(q=q_deg, d=d, pitch=np.degrees(p), down=down, sigma=TP._sigma_nd(q, P),
               gear_beam=(FIN_TOP - d) + down - GEAR - BEAM_TOP)
    if not inspan.any():
        out.update(contact=False, lowest=float(up.min()) - d)
        return out
    k = np.argmax(np.where(inspan, fw, -1e9))
    back = -FIN_L / 2 - s
    # the bracket's surface normal around the contact point (side view), averaged
    near = inspan & (np.hypot(fw - fw[k], up - up[k]) < 0.003)
    nn = SN[m][near]
    nside = np.c_[nn[:, 2] * np.cos(p) + nn[:, 0] * np.sin(p), -nn[:, 2] * np.sin(p) + nn[:, 0] * np.cos(p)]
    nside = nside[nside[:, 0] > 0.2] if (nside[:, 0] > 0.2).any() else nside    # faces looking forward
    nv = nside.mean(0); nv /= max(np.linalg.norm(nv), 1e-9)
    out.update(contact=True, slack=back - fw[k], below_top=d - up[k], push_down_deg=np.degrees(np.arctan2(-nv[1], nv[0])))
    return out

if __name__ == "__main__":
    poses = [(0, 5, 25, 0), (0, 10, 30, 0), (0, 15, 30, 0), (0, 15, 35, 0), (0, 20, 30, 0), (0, 20, 40, 0), (0, 28, 32, 0)]
    print("pose deg            pitch  down   sig  | d mm: below-top / slack / push-down-angle / gear-over-beam")
    for qd in poses:
        row = []
        for dmm in (25, 35, 45, 55, 65):
            r = screen(qd, dmm / 1e3)
            if r["contact"]:
                row.append(f"{dmm}: {r['below_top']*1e3:4.0f}/{r['slack']*1e3:4.0f}/{r['push_down_deg']:3.0f}/{r['gear_beam']*1e3:4.0f}")
            else:
                row.append(f"{dmm}: none(+{r['lowest']*1e3:.0f})/{r['gear_beam']*1e3:4.0f}")
        r0 = screen(qd, 0.025)
        print(f"{str(qd):18s} {r0['pitch']:5.1f}  {r0['down']*1e3:4.0f}  {r0['sigma']:.3f} | " + " | ".join(row))
