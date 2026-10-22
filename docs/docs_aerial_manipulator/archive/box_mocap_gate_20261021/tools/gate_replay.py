#!/usr/bin/env python3
"""Replay the proposed box-mocap sample gate (2026-10-21) on recorded obj_0 streams: the drone
mocap guard's jump test (fsc_drone_state_estimator_ros2 MocapGuard) applied to the box --

    reject a sample when |p - p_good| > jump_m + v dt + 1/2 a dt^2
                      or angle(q, q_good) > jump_deg + w dt,      dt = t - t_good,

measured from the LAST GOOD sample, so the allowance grows over a gap and a new pose that
persists is eventually let back in. Pure numpy, no ROS.

    /usr/bin/python3 gate_replay.py [--v 1.0] [--a 2.0] [--w 90] [--jump-m 0.020] [--jump-deg 10]

Sources: the 10-21 hardware bags (archive/wb_box_pushing_flight_20261021/npz, topic obj = the
processor's /obj_0/mocap, receive time) and the 10-21 Isaac runs (the driver's box record).
"""
import argparse
import glob
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", ".."))
HW = os.path.join(ARCH, "wb_box_pushing_flight_20261021", "npz")
SIM = [os.path.join(ARCH, "box_pushing_gain_tune_20261021", "runs"),
       os.path.join(ARCH, "box_pushing_release_delay_20261021", "runs")]


def gate(t, P, Q, jump_m, jump_deg, v, a, w):
    """Per sample: accepted?, the distance [m] and angle [deg] from the last good sample, and
    the allowance in effect."""
    n = len(t)
    ok = np.zeros(n, bool)
    dp = np.zeros(n)
    dr = np.zeros(n)
    gp = np.zeros(n)
    gr = np.zeros(n)
    good = None
    for i in range(n):
        if good is None:
            ok[i] = True
        else:
            tg, pg, qg = good
            dt = max(t[i] - tg, 0.0)
            gp[i] = jump_m + v * dt + 0.5 * a * dt * dt
            gr[i] = jump_deg + w * dt
            dp[i] = np.linalg.norm(P[i] - pg)
            dr[i] = 2.0 * np.degrees(np.arccos(min(1.0, abs(float(Q[i] @ qg)))))
            ok[i] = dp[i] <= gp[i] and dr[i] <= gr[i]
        if ok[i]:
            good = (t[i], P[i], Q[i])
    return ok, dp, dr, gp, gr


def runs_of(bad):
    out, i = [], 0
    while i < len(bad):
        if bad[i]:
            j = i
            while j + 1 < len(bad) and bad[j + 1]:
                j += 1
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def report(name, t, P, Q, args, mark=None):
    ok, dp, dr, gp, gr = gate(t, P, Q, args.jump_m, args.jump_deg, args.v, args.a, args.w)
    bad = ~ok
    print(f"{name}: {len(t)} samples, {bad.sum()} rejected in {len(runs_of(bad))} runs")
    for i, j in runs_of(bad):
        k = i + int(np.argmax(dr[i:j + 1]))
        where = "" if mark is None else f"  [{mark(t[i])}]"
        print(f"   t {t[i]:8.3f} s: {j - i + 1} sample(s) over {(t[j] - t[i]) * 1e3:5.1f} ms, "
              f"worst {dp[k] * 1e3:6.1f} mm / {dr[k]:6.1f} deg (allowance {gp[k] * 1e3:5.1f} mm / {gr[k]:5.1f} deg)"
              f"{where}")
    # how close the ACCEPTED samples came to the gate: the margin left for real motion
    if ok.sum() > 1:
        m = ok.copy()
        m[0] = False
        print(f"   accepted samples: max step {dp[m].max() * 1e3:.2f} mm / {dr[m].max():.2f} deg "
              f"(the gate {args.jump_m * 1e3:.0f} mm / {args.jump_deg:.0f} deg at dt -> 0)")
    return bad.sum()


def hw(args):
    for f in sorted(glob.glob(os.path.join(HW, "b*.npz"))):
        d = np.load(f, allow_pickle=True)
        if "obj__recv" not in d.files or len(d["obj__recv"]) < 2:
            continue
        t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
        t = d["obj__recv"] - t0
        P = np.column_stack([d[f"obj__pose.position.{c}"] for c in "xyz"])
        Q = np.column_stack([d[f"obj__pose.orientation.{c}"] for c in "wxyz"])
        Q /= np.linalg.norm(Q, axis=1, keepdims=True)
        ev = []
        if "pl_pstatus__recv" in d.files:
            ev = list(zip(d["pl_pstatus__recv"] - t0, [str(s) for s in d["pl_pstatus__data"]]))

        def mark(ti, ev=ev):
            last = [s for tt, s in ev if tt <= ti]
            return last[-1][:40] if last else "-"
        report(f"HW {os.path.basename(f)[1:-4]}", t, P, Q, args, mark)


def sim(args):
    for d_ in SIM:
        for f in sorted(glob.glob(os.path.join(d_, "*.npz"))):
            z = np.load(f, allow_pickle=True)
            if "box" not in z.files or len(z["box"]) < 2:
                continue
            B = z["box"]
            Q = B[:, 4:8] / np.linalg.norm(B[:, 4:8], axis=1, keepdims=True)
            report(f"Isaac {os.path.basename(f)[:-4]}", B[:, 0] - B[0, 0], B[:, 1:4], Q, args)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jump-m", type=float, default=0.020)
    ap.add_argument("--jump-deg", type=float, default=10.0)
    ap.add_argument("--v", type=float, default=1.0, help="the position allowance's growth [m/s]")
    ap.add_argument("--a", type=float, default=2.0, help="... its acceleration term [m/s^2]")
    ap.add_argument("--w", type=float, default=90.0, help="the angle allowance's growth [deg/s]")
    ap.add_argument("--sim", action="store_true", help="also the Isaac runs")
    args = ap.parse_args()
    print(f"gate: {args.jump_m * 1e3:.0f} mm + {args.v} m/s dt + 1/2 {args.a} m/s^2 dt^2, "
          f"{args.jump_deg:.0f} deg + {args.w} deg/s dt, from the last good sample")
    hw(args)
    if args.sim:
        sim(args)


if __name__ == "__main__":
    main()
