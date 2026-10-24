#!/usr/bin/env python3
"""Score one mail-delivery flight (pnp_mission_v2.py --mail npz) against the 07
mail-delivery scene's geometry (its defaults; pass --scene-* to match a run that
overrode them).

    /usr/bin/python3 mail_score.py <run.npz> [...]

Per run: where the ENVELOPE crossed the mailbox's front-wall plane during the
insert, as margins to the mouth's edges (below / above / each side); the jaw
tips' closest approach to the wall; the envelope's far end vs the back wall;
its pitch in the jaws while carried; the claw error at the close and the open;
q4 while carried; the props' clearance over the mailbox top at the release;
the vehicle's peak tilt per phase; and where the envelope ended.
"""
import argparse
import math

import numpy as np

# the 07 mail-delivery scene's defaults
ENV_L, ENV_W, ENV_T = 0.150, 0.090, 0.004
TABLE_FRONT_X, TABLE_TOP = 1.70, 0.80
RED_Y, BOX_W, BOX_D, BOX_H, WALL = 0.25, 0.235, 0.125, 0.2725, 0.005
MOUTH_W, MOUTH_H, MOUTH_Z = 0.190, 0.090, 0.2025
JAW_TIP = 0.016                      # fingertips beyond the claw point
PROP_BOTTOM = 0.058                  # rotor disc bottom above the body origin


def quat_rot(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def window(d, a, b):
    names = list(d["marks_name"])
    t = dict(zip(names, d["marks_t"]))
    return t.get(a), t.get(b)


def score(path):
    d = np.load(path, allow_pickle=True)
    pl = d["payload"]          # t, x, y, z, qw, qx, qy, qz, step
    od = d["odom"]
    claw = d["claw"]
    js = d["joints"]
    arr = d["arrival"]
    out = {"run": path, "completed": not bool(d["aborted"]), "reason": str(d["reason"])}
    t_ins0, _ = window(d, "place:start", "place:end")
    t_open, t_open_end = window(d, "gripper_open:start", "gripper_open:end")
    t_close, _ = window(d, "gripper_close:start", "gripper_close:end")
    t_exit_pick, _ = window(d, "exit_pick:start", "exit_pick:end")
    t_ready_place, _ = window(d, "ready_place:start", "ready_place:end")
    mouth_lo = TABLE_TOP + MOUTH_Z - 0.5 * MOUTH_H
    mouth_hi = TABLE_TOP + MOUTH_Z + 0.5 * MOUTH_H
    # ---- the envelope through the wall plane (insert start .. gripper open) ----
    if t_ins0 is not None and t_open is not None:
        sel = (pl[:, 0] >= t_ins0) & (pl[:, 0] <= t_open + 0.5)
        zmin, zmax, ymax = np.inf, -np.inf, 0.0
        far_x = -np.inf
        for r in pl[sel]:
            p, R = r[1:4], quat_rot(*r[4:8])
            a, b, n = R[:, 0], R[:, 1], R[:, 2]
            far_x = max(far_x, (p + 0.5 * ENV_L * a)[0])
            if abs(a[0]) < 1e-3:
                continue
            for w in (-0.5 * ENV_W, 0.5 * ENV_W):
                s = (TABLE_FRONT_X - p[0] - w * b[0]) / a[0]
                if abs(s) > 0.5 * ENV_L:
                    continue
                q = p + s * a + w * b
                for h in (-0.5 * ENV_T, 0.5 * ENV_T):
                    z = (q + h * n)[2]
                    zmin, zmax = min(zmin, z), max(zmax, z)
                ymax = max(ymax, abs(q[1] - RED_Y))
        if np.isfinite(zmin):
            out["mouth_below_mm"] = (zmin - mouth_lo) * 1e3
            out["mouth_above_mm"] = (mouth_hi - zmax) * 1e3
            out["mouth_side_mm"] = (0.5 * MOUTH_W - ymax) * 1e3
        out["far_end_to_back_wall_mm"] = (TABLE_FRONT_X + BOX_D - WALL - far_x) * 1e3
        cs = (claw[:, 0] >= t_ins0) & (claw[:, 0] <= (t_open_end or t_open) + 0.3)
        if cs.any():
            out["jaw_tips_to_wall_mm"] = (TABLE_FRONT_X - (claw[cs, 1].max() + JAW_TIP)) * 1e3
        # props over the mailbox top at the open
        o = od[np.argmin(np.abs(od[:, 0] - t_open))]
        out["props_over_box_mm"] = (o[3] + PROP_BOTTOM - (TABLE_TOP + BOX_H)) * 1e3
    # ---- the envelope's pitch in the jaws while carried ----
    if t_exit_pick is not None and t_ins0 is not None:
        sel = (pl[:, 0] >= t_exit_pick + 3.0) & (pl[:, 0] <= t_ins0)
        if sel.any():
            pitch = []
            for r in pl[sel]:
                a = quat_rot(*r[4:8])[:, 0]
                pitch.append(math.degrees(math.atan2(-a[2], math.hypot(a[0], a[1]))))
            out["carry_pitch_deg"] = (float(np.mean(pitch)), float(np.min(pitch)), float(np.max(pitch)))
        sel = (js[:, 0] >= t_exit_pick + 3.0) & (js[:, 0] <= t_ins0)
        if sel.any():
            q4 = np.degrees(js[sel, 4])
            out["carry_q4_deg"] = (float(np.mean(q4)), float(np.min(q4)), float(np.max(q4)))
    # ---- the claw error at the close and the open (the planner's own) ----
    for name, t0 in (("close", t_close), ("open", t_open)):
        if t0 is None:
            continue
        r = arr[np.argmin(np.abs(arr[:, 0] - t0))]
        out[f"claw_err_at_{name}_mm"] = (float(r[3] * 1e3), float(r[4] * 1e3), float(r[5] * 1e3))
    # ---- the vehicle's tilt per phase ----
    tilts = {}
    for ph, (a_, b_) in {"pick": ("pick:start", "exit_pick:end"), "carry": ("go_to_place_start:start", "ready_place:end"),
                         "insert": ("place:start", "exit_place:end")}.items():
        a, b = window(d, a_, b_)
        if a is None or b is None:
            continue
        s = (od[:, 0] >= a) & (od[:, 0] <= b)
        if s.any():
            R22 = [quat_rot(*q)[2, 2] for q in od[s, 7:11]]
            tilts[ph] = float(np.degrees(np.arccos(np.clip(np.min(R22), -1.0, 1.0))))
    out["tilt_max_deg"] = tilts
    # ---- where the envelope ended ----
    r = pl[-1]
    p, R = r[1:4], quat_rot(*r[4:8])
    c = p + R @ np.array([0.5 * ENV_L - 0.040, 0.0, 0.0])
    inside = (TABLE_FRONT_X + WALL < c[0] < TABLE_FRONT_X + BOX_D - WALL and abs(c[1] - RED_Y) < 0.5 * BOX_W - WALL
              and c[2] < mouth_lo)
    low_end = min((p + s * 0.5 * ENV_L * R[:, 0])[2] for s in (-1, 1))
    out["final"] = {"in_red_mailbox": bool(inside), "centre": np.round(p, 3).tolist(),
                    "tilt_deg": float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))),
                    "low_end_above_floor_mm": (low_end - (TABLE_TOP + WALL)) * 1e3}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    a = ap.parse_args()
    for path in a.runs:
        s = score(path)
        print(f"== {s['run']}: {'COMPLETED' if s['completed'] else 'ABORTED (' + s['reason'] + ')'}")
        for k, v in s.items():
            if k in ("run", "completed", "reason"):
                continue
            if isinstance(v, float):
                v = round(v, 1)
            elif isinstance(v, tuple):
                v = tuple(round(x, 1) for x in v)
            print(f"   {k}: {v}")


if __name__ == "__main__":
    main()
