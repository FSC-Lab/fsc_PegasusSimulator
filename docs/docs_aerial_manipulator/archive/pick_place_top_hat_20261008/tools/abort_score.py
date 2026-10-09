#!/usr/bin/env python3
"""Score an ABORT test flight (run_abort_tests.sh, 2026-10-09).

    PYTHONNOUSERSITE=1 /usr/bin/python3 tools/abort_score.py runs/abort_wb_carry.npz [...]

From the driver's npz: the payload ground truth (the basket's centre), the claw ground truth
(claw_0, the grasp point), the odometry (tilt) and the joints, around the driver's marks
op_abort:start (gripper open + abort pressed) -> op_abort:end (the planner reports ABORTED, the
gripper is closed) -> post_abort_hold:end.

RELEASED means: at the end of the hold the basket is at rest and no longer moves with the claw
(its offset from the claw has changed by more than 5 cm since the press and it is not rising with
the vehicle). Where it rests: a hat (z ~1.04 m: PICK at (1, 1), PLACE at (-1, -1)) or the floor.
"""
import sys

import numpy as np

HAT_Z, FLOOR_Z = 1.04, 0.05
HATS = {"PICK hat": (1.0, 1.0), "PLACE hat": (-1.0, -1.0)}


def at(T, X, t):
    i = int(np.clip(np.searchsorted(T, t), 0, len(T) - 1))
    return X[i]


def main(paths):
    for p in paths:
        d = np.load(p, allow_pickle=False)
        m = {str(n): float(t) for t, n in zip(d["marks_t"], d["marks_name"])}
        print(f"== {p}  ({d['reason']})")
        if "op_abort:start" not in m:
            print("   no op_abort mark -- the abort was never pressed")
            continue
        t0, t1, t2 = m["op_abort:start"], m["op_abort:end"], m.get("post_abort_hold:end", m["op_abort:end"])
        pay, claw, od, js = d["payload"], d["claw"], d["odom"], d["joints"]
        Tp, Pp = pay[:, 0], pay[:, 1:4]
        Tc, Pc = claw[:, 0], claw[:, 1:4]
        off0 = at(Tp, Pp, t0) - at(Tc, Pc, t0)
        print(f"   pressed at {t0:.2f} s; ABORTED after {t1 - t0:.2f} s; watched {t2 - t1:.1f} s more")
        print(f"   at the press: basket {np.round(at(Tp, Pp, t0), 3).tolist()}, claw->basket "
              f"{np.round(off0, 3).tolist()} m")
        # when did the basket stop following the claw
        sep = None
        for t in np.arange(t0, t2, 0.02):
            off = at(Tp, Pp, t) - at(Tc, Pc, t)
            if np.linalg.norm(off - off0) > 0.05:
                sep = t
                break
        w = (Tp >= t2 - 1.0) & (Tp <= t2)
        v_end = np.linalg.norm(np.diff(Pp[w], axis=0), axis=1).sum() / max(Tp[w][-1] - Tp[w][0], 1e-3) if w.sum() > 2 else np.nan
        pe = at(Tp, Pp, t2)
        ce = at(Tc, Pc, t2)
        follows = np.linalg.norm((pe - ce) - off0) < 0.05
        if pe[2] < FLOOR_Z:
            where = "on the FLOOR"
        elif abs(pe[2] - HAT_Z) < 0.03:
            name, c = min(HATS.items(), key=lambda kv: np.hypot(pe[0] - kv[1][0], pe[1] - kv[1][1]))
            where = f"on the {name}, {1e3 * np.hypot(pe[0] - c[0], pe[1] - c[1]):.0f} mm off its axis"
        else:
            where = f"at z {pe[2]:.3f} m (neither a hat nor the floor)"
        q = pay[np.searchsorted(Tp, t2) - 1, 4:8]       # wxyz
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[1] ** 2 + q[2] ** 2), -1, 1)))
        released = (not follows) and v_end < 0.02
        print(f"   basket left the claw at {'--' if sep is None else f'{sep - t0:+.2f} s'} after the press; "
              f"at the end it is {where}, upright within {tilt:.0f} deg, moving {1e3 * v_end:.0f} mm/s")
        print(f"   RELEASED: {released}  (claw->basket offset changed {1e3 * np.linalg.norm((pe - ce) - off0):.0f} mm)")
        # the vehicle during the abort
        wo = (od[:, 0] >= t0) & (od[:, 0] <= t2)
        qo = od[wo, 7:11]
        tilt_v = np.degrees(np.arccos(np.clip(1 - 2 * (qo[:, 1] ** 2 + qo[:, 2] ** 2), -1, 1)))
        z0, z2 = at(od[:, 0], od[:, 3], t0), at(od[:, 0], od[:, 3], t2)
        print(f"   vehicle: tilt max {tilt_v.max():.1f} deg, z {z0:.3f} -> {z2:.3f} m")
        names = [str(n) for n in d["joint_names"]]
        order = [names.index(f"joint{i}") for i in range(1, 5)] if all(f"joint{i}" in names for i in range(1, 5)) else [0, 1, 2, 3]
        qj = np.degrees(at(js[:, 0], js[:, 1:5][:, order], t2))
        g = d["grip"]
        ge = np.degrees(at(g[:, 0], g[:, 1], t2)) if g.shape[1] > 1 else np.nan
        print(f"   arm at the end {np.round(qj, 1).tolist()} deg (home [0, 40, 40, 0]); gripper {ge:.1f} deg "
              f"(closed -50, open 0)")


if __name__ == "__main__":
    main(sys.argv[1:])
