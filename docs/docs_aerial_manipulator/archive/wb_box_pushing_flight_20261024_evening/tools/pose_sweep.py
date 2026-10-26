#!/usr/bin/env python3
"""Offline push-pose screen (2026-10-25): every (q2, q3) with q1 = q4 = 0 inside the REAL arm's range
(q2 [-20, 45] deg, q3 between the planner's 10 deg q3 guard and the 50 deg stop), scored on the
geometry the 10-24 hardware pushes ran out of. Model: the planner's T650 model (make_params_t650).

Per pose:
  * claw below the body origin -> the vehicle's hover height for the grasp (233 mm above the table:
    the CAD fin, grasped 25 mm under its 258 mm top) and the landing-gear skids above the table top
    (gear 0.248 m under the body origin, measured on the real T650). The planner refuses < 0.05 m
    (push_pull_table_clearance), so that is a hard limit; >= 0.08 m is wanted.
  * h, the claw below the CoM: the lever that turns a horizontal contact force into a pitch moment.
  * sigma_nd (the planner's singularity margin; keep-out 0.10).
  * the ARM STROKE along the push: with the claw held where it is and the vehicle moved along its
    nose (level), how far the vehicle can BACK AWAY from the box (the arm stretches: q3 toward its
    10 deg guard, q2 toward its 45 deg wall -- the 10-24 failure) and how far it can come TOWARD the
    box (the arm folds: q3 toward its 50 deg stop) before a joint leaves its range. Planar 2R inverse
    kinematics on the model's own chain (q2, q3 about the same axis).
  * the arm's static torques j2 / j3 [N.m] (gravity + a 1.5 N push at the claw along the nose)
    against the hardware caps 2.81 / 1.92 N.m.
  * the same stroke vertically (vehicle bobbing), the smaller of up / down.
The hardware 280 g push moved the vehicle +-80 mm along the push (CoM p-p 167 mm); the free-axis
anchor carries 75 % of that to the claw, so the arm itself must absorb roughly 25 % plus whatever the
box's friction holds back.
Usage: pose_sweep.py [--fold-min 40 --fold-max 80]"""
import argparse
import os
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), *[".."] * 5))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP   # noqa: E402

P = TP.make_params_t650()
GRASP_H, GEAR, TABLE_CLR = 0.233, 0.248, 0.05
Q2_LO, Q2_HI, Q3_LO, Q3_HI = -20.0, 45.0, 10.0, 50.0


def claw(q2, q3):
    """claw (EE) and system CoM relative to the body origin, model frame (y = nose, z = up)."""
    r0c, r0e = TP.CT._arm_kin(np.radians([0.0, q2, q3, 0.0]), P)
    return np.asarray(r0e, float), np.asarray(r0c, float)


def ik(target, q_guess):
    """(q2, q3) [deg] putting the claw at `target` (model y, z) relative to the body origin, the
    elbow branch of q_guess; None if unreachable."""
    q = np.array(q_guess, float)
    for _ in range(40):
        e, _ = claw(*q)
        r = np.array([e[1], e[2]]) - target
        if np.linalg.norm(r) < 1e-7:
            return q
        J = np.zeros((2, 2)); h = 1e-4
        for k in range(2):
            dq = q.copy(); dq[k] += h
            e2, _ = claw(*dq); J[:, k] = (np.array([e2[1], e2[2]]) - np.array([e[1], e[2]])) / h
        try:
            step = np.linalg.solve(J, -r)
        except np.linalg.LinAlgError:
            return None
        q = q + np.clip(step, -5, 5)
    return q if np.linalg.norm(r) < 1e-5 else None


def stroke(q2, q3, axis, sign, lim=0.25, ds=0.001):
    """How far the VEHICLE can move along `axis` (0 = nose, 1 = up) in direction `sign` with the claw
    fixed in the room before a joint leaves its range; returns (distance [m], what binds)."""
    e0, _ = claw(q2, q3)
    tgt0 = np.array([e0[1], e0[2]])
    q = np.array([q2, q3])
    s = 0.0
    while s < lim:
        s += ds
        d = np.zeros(2); d[axis] = sign * s
        q = ik(tgt0 - d, q)                       # the vehicle moves +d: the claw moves -d in its frame
        if q is None:
            return s, "reach"
        if q[0] > Q2_HI:
            return s, "q2 wall"
        if q[0] < Q2_LO:
            return s, "q2 low"
        if q[1] < Q3_LO:
            return s, "q3 guard"
        if q[1] > Q3_HI:
            return s, "q3 stop"
    return lim, "none"


def torques(q2, q3, push=1.5):
    """static joint torques [N.m]: the arm's gravity (the law's own model) + the push force at the claw
    (the box resisting: a force on the claw pointing back, -nose)."""
    from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C
    X = np.zeros(12 + 3 * 4 + 6); X[3:12] = np.eye(3).flatten(order="F"); X[12:16] = np.radians([0, q2, q3, 0])
    d = C.dynamics(X, P)
    g = np.asarray(d["g"], float)[6:10]
    Jy = d["J_y"] @ d["T"]                       # generalized velocity -> claw velocity (world)
    F = np.array([0.0, -push, 0.0, 0.0])         # the box pushes the claw back along -nose (model y)
    return np.abs(g + (Jy.T @ F)[6:10])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fold-min", type=float, default=40.0)
    ap.add_argument("--fold-max", type=float, default=85.0)
    ap.add_argument("--step", type=float, default=2.0)
    a = ap.parse_args()
    rows = []
    for q2 in np.arange(Q2_LO, Q2_HI + 1e-9, a.step):
        for q3 in np.arange(Q3_LO, Q3_HI + 1e-9, a.step):
            if not a.fold_min <= q2 + q3 <= a.fold_max:
                continue
            e, c = claw(q2, q3)
            drop = -e[2]
            skid = GRASP_H + drop - GEAR
            if skid < TABLE_CLR:
                continue
            sig = TP._sigma_nd(np.radians([0.0, q2, q3, 0.0]), P)
            tow, why_t = stroke(q2, q3, 0, +1)      # the vehicle moves along its nose = TOWARD the box
            away, why_a = stroke(q2, q3, 0, -1)     # backs away: the arm stretches
            up, _ = stroke(q2, q3, 1, +1); dn, _ = stroke(q2, q3, 1, -1)
            tq = torques(q2, q3)
            rows.append(dict(q2=q2, q3=q3, fold=q2 + q3, reach=e[1], drop=drop, h=c[2] - e[2], skid=skid, sig=sig,
                             away=away, why_a=why_a, tow=tow, why_t=why_t, vert=min(up, dn), t2=tq[1], t3=tq[2]))
    return rows


def show(rows, title, key, n=12):
    print(f"\n{title}")
    print(f"{'pose [q2,q3]':>14s} {'fold':>4s} {'reach':>6s} {'drop':>6s} {'h':>6s} {'skids':>6s} {'sig_nd':>6s} | "
          f"{'BACK AWAY (binds)':>18s} {'toward (binds)':>17s} {'vert':>5s} | {'tau2':>5s} {'tau3':>5s}")
    for r in sorted(rows, key=key)[:n]:
        print(f"[0,{r['q2']:3.0f},{r['q3']:3.0f},0] {r['fold']:4.0f} {1e3*r['reach']:6.0f} {1e3*r['drop']:6.0f} "
              f"{1e3*r['h']:6.0f} {1e3*r['skid']:6.0f} {r['sig']:6.3f} | {1e3*r['away']:5.0f} mm {r['why_a']:>9s} "
              f"{1e3*r['tow']:5.0f} mm {r['why_t']:>9s} {1e3*r['vert']:5.0f} | {r['t2']:5.2f} {r['t3']:5.2f}")


if __name__ == "__main__":
    rows = main()
    cur = [r for r in rows if r["q2"] == 32 and r["q3"] == 28]
    show(cur, "THE FLOWN POSE", key=lambda r: 0)
    ok = [r for r in rows if r["sig"] >= 0.15 and r["skid"] >= 0.08 and r["tow"] >= 0.025]
    show(ok, "largest BACK-AWAY stroke (the 10-24 failure direction); sigma_nd >= 0.15, skids >= 80 mm, toward >= 25 mm",
         key=lambda r: -r["away"])
    show(ok, "largest min(back away, toward)", key=lambda r: -min(r["away"], r["tow"]))
    near = [r for r in ok if 55 <= r["fold"] <= 70]
    show(near, "folds 55-70 (claw pitch 35-20 deg down, as flown 30), by back-away stroke", key=lambda r: -r["away"])
    print(f"\n{len(rows)} poses inside the range with skids >= {1e3*TABLE_CLR:.0f} mm; {len(ok)} with sigma_nd >= 0.15, skids >= 80 mm, toward >= 25 mm")
