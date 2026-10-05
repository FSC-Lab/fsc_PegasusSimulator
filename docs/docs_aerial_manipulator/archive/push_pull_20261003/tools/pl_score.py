#!/usr/bin/env python3
"""Score a push-and-pull run recorded by pl_mission.py, per step:

    /usr/bin/python3 pl_score.py ../runs/pl_1.npz [more.npz ...]

Per step (the driver's step tag): duration, peak tilt (odometry), |e_R|,
CoM error |x_c - x_cd|, EE task error |e_y[0:3]| (mm) and EE heading error
asin(e_y[3]) (deg) -- never normed together, they carry different units --,
peak arm torque, rotor saturation and joint-clamp ticks, the contact flag chi
and the raw / rendered force reading. Then the box: its displacement and yaw
change during the push, its tilt (tipping), and how far it moved in every
other step (disturbance by the descent / the release / the exit).
"""
import sys

import numpy as np

STEPS = ["go_to_start", "ready", "close", "push", "release", "exit", "go_to_land"]
# wb_control_debug, as wb_l1_metrics.py (column = index + 2 in the recorder's rows)
D_TAU, D_EY, D_ER = slice(13, 17), slice(24, 28), slice(28, 31)
D_NSAT, D_NCLAMP = 51, 88
D_XCD, D_XC = slice(45, 48), slice(48, 51)
D_FY, D_FRAW, D_CHI = slice(58, 62), slice(97, 101), 105


def col(dbg, s):
    if isinstance(s, slice):
        return dbg[:, s.start + 2:s.stop + 2]
    return dbg[:, s + 2]


def tilt_deg(qw, qx, qy, qz):
    # angle between body z and world z
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return np.degrees(np.arccos(np.clip(r22, -1.0, 1.0)))


def yaw_deg(qw, qx, qy, qz):
    return np.degrees(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))


def score(path):
    z = np.load(path, allow_pickle=True)
    print(f"\n=== {path}")
    print(f"aborted={bool(z['aborted'])}  reason={str(z['reason'])!r}")
    for e in z["events"]:
        print("  " + str(e))
    odom, dbg, box = z["odom"], z["dbg"], z["box"]
    hdr = (f"{'step':12s} {'T[s]':>6s} {'tilt':>6s} {'|eR|':>6s} {'CoMpk':>6s} {'CoMrms':>6s} "
           f"{'EEpk':>6s} {'EErms':>6s} {'hdg':>5s} {'tau':>5s} {'sat':>4s} {'clmp':>4s} "
           f"{'chi':>4s} {'Fraw':>6s} {'Fy':>6s} {'box':>6s}")
    print(hdr)
    print("             " + "        deg         mm     mm     mm     mm   deg   N.m              "
          "    N      N     mm")
    for k, name in enumerate(STEPS):
        o = odom[odom[:, -1] == k] if odom.size > 1 else np.zeros((0, 13))
        d = dbg[dbg[:, 1] == k] if dbg.size > 1 else np.zeros((0, 130))
        b = box[box[:, -1] == k] if box.size > 1 else np.zeros((0, 9))
        if len(o) == 0:
            continue
        T = o[-1, 0] - o[0, 0]
        tilt = tilt_deg(o[:, 7], o[:, 8], o[:, 9], o[:, 10]).max()
        row = f"{name:12s} {T:6.1f} {tilt:6.2f}"
        if len(d):
            eR = np.linalg.norm(col(d, D_ER), axis=1).max()
            ecom = np.linalg.norm(col(d, D_XC) - col(d, D_XCD), axis=1) * 1e3
            ey = col(d, D_EY)
            eee = np.linalg.norm(ey[:, :3], axis=1) * 1e3
            hdg = np.degrees(np.arcsin(np.clip(np.abs(ey[:, 3]), 0, 1))).max()
            tau = np.abs(col(d, D_TAU)).max()
            nsat = int(np.count_nonzero(col(d, D_NSAT) > 0))
            ncl = int(np.count_nonzero(col(d, D_NCLAMP) > 0))
            chi = col(d, D_CHI)
            frw = np.linalg.norm(col(d, D_FRAW)[:, :3], axis=1).max()
            fy = np.linalg.norm(col(d, D_FY)[:, :3], axis=1).max()
            row += (f" {eR:6.3f} {ecom.max():6.0f} {np.sqrt(np.mean(ecom ** 2)):6.1f} "
                    f"{eee.max():6.1f} {np.sqrt(np.mean(eee ** 2)):6.1f} {hdg:5.1f} {tau:5.2f} "
                    f"{nsat:4d} {ncl:4d} {np.mean(chi):4.2f} {frw:6.2f} {fy:6.2f}")
        else:
            row += "   (no DIRECT debug)"
        if len(b) > 1:
            row += f" {np.linalg.norm(b[-1, 1:4] - b[0, 1:4]) * 1e3:6.1f}"
        print(row)
    # the box
    if box.size > 1:
        b0, bf = box[0], box[-1]
        bt = tilt_deg(box[:, 4], box[:, 5], box[:, 6], box[:, 7])
        print(f"box: start {np.round(b0[1:4], 4).tolist()}  end {np.round(bf[1:4], 4).tolist()}")
        d = (bf[1:4] - b0[1:4]) * 1e3
        print(f"box: net move [{d[0]:+.1f}, {d[1]:+.1f}, {d[2]:+.1f}] mm, yaw "
              f"{yaw_deg(*b0[4:8]):+.2f} -> {yaw_deg(*bf[4:8]):+.2f} deg, peak tilt {bt.max():.2f} deg")
        p = box[box[:, -1] == STEPS.index("push")]
        if len(p) > 1:
            dp = (p[-1, 1:4] - p[0, 1:4]) * 1e3
            v = np.linalg.norm(np.diff(p[:, 1:3], axis=0), axis=1) / np.maximum(np.diff(p[:, 0]), 1e-3)
            print(f"push: box [{dp[0]:+.1f}, {dp[1]:+.1f}, {dp[2]:+.1f}] mm, yaw change "
                  f"{yaw_deg(*p[-1, 4:8]) - yaw_deg(*p[0, 4:8]):+.2f} deg, peak tilt "
                  f"{tilt_deg(p[:, 4], p[:, 5], p[:, 6], p[:, 7]).max():.2f} deg, peak speed "
                  f"{v.max():.3f} m/s")
    # the claw against the box's handle during the push (both ground truth)
    claw = z["claw"]
    if claw.size > 1 and box.size > 1:
        c = claw[claw[:, -1] == STEPS.index("push")]
        p = box[box[:, -1] == STEPS.index("push")]
        if len(c) > 2 and len(p) > 2:
            rel = np.column_stack([np.interp(c[:, 0], p[:, 0], p[:, i]) for i in (1, 2, 3)])
            off = (c[:, 1:4] - rel) * 1e3
            span = off.max(axis=0) - off.min(axis=0)
            print(f"push: claw - box span (slip) [{span[0]:.1f}, {span[1]:.1f}, {span[2]:.1f}] mm")
    g = z["grip"]
    if g.size > 1:
        for k in (STEPS.index("push"), STEPS.index("exit")):
            s = g[g[:, -1] == k]
            if len(s):
                print(f"gripper in {STEPS[k]}: position {s[:, 1].min():+.4f}..{s[:, 1].max():+.4f}, "
                      f"|effort| max {np.nanmax(np.abs(s[:, 2])):.3f}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        score(p)
