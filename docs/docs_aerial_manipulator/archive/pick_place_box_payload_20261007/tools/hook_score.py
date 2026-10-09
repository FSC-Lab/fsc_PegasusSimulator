#!/usr/bin/env python3
"""Score the box payload's HOOK pick-and-place runs (2026-10-07).

    PYTHONNOUSERSITE=1 /usr/bin/python3 hook_score.py ../runs/hook1.npz ...

On top of the 10-03 scorer (pnp_score.py: completed / picked / placed, place
offset, tilts, EE errors): what the arm-pose question needs --
  * q2 / q3 over the DIRECT flight against the real arm's q2 range [-20, 45]
    and q3's +50 stop: min / max per step and the closest approach;
  * the kinematic sigma_nd along the flown joints (keep-out 0.10);
  * the hanging basket: its tilt and swing while carried, how far it slid
    along the fingers (claw-to-basket distance), and its CLEARANCE to the
    landing-gear skids (the basket's corners in the body frame against the
    skid boxes measured off AM_xfwd.usda).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "pick_place_controllers_20261003", "tools"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "..", "extensions", "fsc_aerial_manipulation"))
import pnp_score as PS                                              # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

Q2_RANGE = (-20.0, 45.0)        # the real arm (U2D2 board behind the upper arm)
Q3_MAX = 50.0
# skids, body frame (AM_xfwd.usda, measured 2026-10-07): x [-0.163, 0.163],
# |y| [0.142, 0.168], z [-0.3125, -0.275]
SKID_X, SKID_Y, SKID_Z = (-0.163, 0.163), (0.142, 0.168), (-0.3125, -0.275)
# the basket (payload frame, origin = its centre)
BASKET = np.array([[x, y, z] for x in (-0.055, 0.055) for y in (-0.0576, 0.0576) for z in (-0.0325, 0.0325)])


def quat_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def box_gap(p_lo, p_hi, q_lo, q_hi):
    """Euclidean gap between two axis-aligned boxes (0 = touching / overlapping)."""
    d = np.maximum(0.0, np.maximum(p_lo - q_hi, q_lo - p_hi))
    return float(np.linalg.norm(d))


def hook_extra(path):
    d = np.load(path, allow_pickle=True)
    out = {}
    names, mt = list(d["marks_name"]), d["marks_t"]
    J = d["joints"]
    od = d["odom"]
    if "direct" not in names or len(J) < 10:
        return out
    t_dir = mt[names.index("direct")]
    t_end = mt[-1]
    m = (J[:, 0] >= t_dir) & (J[:, 0] <= t_end)
    q = np.degrees(J[m, 1:5])
    stepcol = J[m, -1]
    out["q2_min_max"] = (round(float(q[:, 1].min()), 1), round(float(q[:, 1].max()), 1))
    out["q3_min_max"] = (round(float(q[:, 2].min()), 1), round(float(q[:, 2].max()), 1))
    out["q2_to_stops"] = (round(float(q[:, 1].min() - Q2_RANGE[0]), 1), round(float(Q2_RANGE[1] - q[:, 1].max()), 1))
    out["q3_to_stop"] = round(float(Q3_MAX - q[:, 2].max()), 1)
    per = {}
    for k, s in enumerate(PS.STEPS):
        mk = stepcol == k
        if mk.sum() > 5:
            per[s] = (round(float(q[mk, 1].min()), 1), round(float(q[mk, 1].max()), 1),
                      round(float(q[mk, 2].min()), 1), round(float(q[mk, 2].max()), 1))
    out["q2min_q2max_q3min_q3max_per_step"] = per
    P = TP.make_params_t650()
    sig = [TP._sigma_nd(np.radians(qq), P) for qq in q[::25]]
    out["sigma_nd_min"] = round(float(min(sig)), 3)
    # the hanging basket while carried (exit_pick end .. place start)
    pay = d["payload"]
    if "exit_pick:end" in names and "place:start" in names and pay.shape[1] >= 8 and od.shape[1] >= 11:
        t0, t1 = mt[names.index("exit_pick:end")], mt[names.index("place:start")]
        mp = (pay[:, 0] > t0) & (pay[:, 0] < t1)
        if mp.sum() > 10:
            tl = PS.tilt_deg(pay[mp, 4:8])
            out["carry_basket_tilt_mean_pp"] = (round(float(tl.mean()), 1), round(float(np.ptp(tl)), 1))
            gaps = []
            for row in pay[mp][::5]:
                i = min(np.searchsorted(od[:, 0], row[0]), len(od) - 1)
                pb, Rb = od[i, 1:4], quat_R(*od[i, 7:11])
                Rp = quat_R(*row[4:8])
                corners = (row[1:4] + BASKET @ Rp.T - pb) @ Rb          # body frame
                lo, hi = corners.min(0), corners.max(0)
                g = min(box_gap(lo, hi, np.array([SKID_X[0], s * SKID_Y[1] if s < 0 else SKID_Y[0], SKID_Z[0]]),
                                np.array([SKID_X[1], s * SKID_Y[0] if s < 0 else SKID_Y[1], SKID_Z[1]])) for s in (1, -1))
                gaps.append(g)
            out["carry_basket_to_skid_min_mm"] = round(1e3 * min(gaps), 1)
    # the SHOWCASE (2026-10-07): how far the hung basket moved in the body frame
    # between the pick pose (the end of exit_pick) and the carry pose (the end of
    # go_to_place_start), each averaged over its last second; and q1 there
    if all(f"{s}:end" in names for s in ("exit_pick", "go_to_place_start")) and pay.shape[1] >= 8:
        def body_frame_mean(t_end):
            m = (pay[:, 0] > t_end - 1.0) & (pay[:, 0] <= t_end)
            v = []
            for row in pay[m]:
                i = min(np.searchsorted(od[:, 0], row[0]), len(od) - 1)
                v.append((row[1:4] - od[i, 1:4]) @ quat_R(*od[i, 7:11]))
            return np.mean(v, axis=0) if v else None
        a_ = body_frame_mean(mt[names.index("exit_pick:end")])
        b_ = body_frame_mean(mt[names.index("go_to_place_start:end")])
        if a_ is not None and b_ is not None:
            dvec = b_ - a_
            out["showcase_basket_move_body_mm"] = (round(1e3 * float(np.linalg.norm(dvec)), 0),
                                                   np.round(1e3 * dvec, 0).tolist())
        mq = (J[:, 0] > mt[names.index("go_to_place_start:end")] - 1.0) & (J[:, 0] <= mt[names.index("go_to_place_start:end")])
        if mq.any():
            out["carry_hold_q_deg"] = np.round(np.degrees(J[mq, 1:5].mean(0)), 1).tolist()
    # the set-down: once the basket rests, its weight leaves the claw and the
    # vehicle lurches (the observer unlearning it) -- how far, and whether the
    # basket was disturbed afterwards (a finger re-catching the stem)
    if "place:start" in names and "exit_place:end" in names and pay.shape[1] >= 8 and od.shape[1] >= 11:
        t0, t1 = mt[names.index("place:start")], mt[names.index("exit_place:end")]
        mp = (pay[:, 0] > t0) & (pay[:, 0] < t1)
        if mp.sum() > 10:
            low = pay[mp, 3] < PS.CAP_TOP + PS.BOX_HALF + 0.002     # resting on the place cap
            if not low.any():
                return out
            t_td = pay[mp][low][0, 0]
            mo = (od[:, 0] > t_td) & (od[:, 0] < t_td + 5.0)
            if mo.sum() > 5:
                out["setdown_lurch_body_xy_mm"] = round(1e3 * float(
                    np.linalg.norm(od[mo, 1:3] - od[mo][0, 1:3], axis=1).max()), 0)
            ma = (pay[:, 0] > t_td + 0.3) & (pay[:, 0] < t1)
            if ma.sum() > 5:
                out["after_touchdown_basket_tilt_max"] = round(float(PS.tilt_deg(pay[ma, 4:8]).max()), 1)
                out["after_touchdown_basket_moved_mm"] = round(1e3 * float(
                    np.linalg.norm(pay[ma, 1:4] - pay[ma][0, 1:4], axis=1).max()), 1)
    if "exit_pick:end" in names and "place:start" in names and "claw" in d and pay.shape[1] >= 8:
        claw = d["claw"]
        t0, t1 = mt[names.index("exit_pick:end")], mt[names.index("place:start")]
        mp = (pay[:, 0] > t0) & (pay[:, 0] < t1)
        if mp.sum() > 10 and len(claw) > 10:
            c = PS.interp(pay[mp, 0], claw[:, 0], claw[:, 1:4])
            dist = np.linalg.norm(pay[mp, 1:4] - c, axis=1)
            out["carry_claw_to_basket_mm"] = (round(1e3 * float(dist.min()), 1), round(1e3 * float(dist.max()), 1))
    return out


def main():
    for p in sys.argv[1:]:
        s = PS.score(p)
        s.update(hook_extra(p))
        print(f"== {s.pop('run')}")
        for k, v in s.items():
            print(f"   {k}: {v}")


if __name__ == "__main__":
    main()
