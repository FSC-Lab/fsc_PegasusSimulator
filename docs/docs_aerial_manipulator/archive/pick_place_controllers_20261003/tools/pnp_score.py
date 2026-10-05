#!/usr/bin/env python3
"""Score autonomous pick-and-place runs (pnp_mission_v2.py npz files).

    /usr/bin/python3 pnp_score.py ../runs/wb_1.npz ../runs/geo_1.npz ...

Per run: completed or where it aborted; PICKED (the payload left the pick cap
with the claw), PLACED (it ends on the place cap, upright), its final offset
from the place pillar's axis; the vehicle's peak tilt per step; the claw's
error against the planner's EE reference (FK of the measured state vs the
streamed r_ed) while flying, and its arrival error when the gripper acted;
the payload's slip against the claw while carried; the mission time.
"""
import sys

import numpy as np

PLACE_XY = np.array([-1.0, -1.0])
PICK_XY = np.array([1.0, 1.0])
CAP_TOP, BOX_HALF, CAP_R = 1.0, 0.0325, 0.08
STEPS = ["go_to_start", "ready_pick", "pick", "exit_pick", "go_to_place_start",
         "ready_place", "place", "exit_place", "go_to_land_start", "execute_land"]


def tilt_deg(q):          # q = (w, x, y, z) -> angle between body z and world z
    w, x, y, z = q.T
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(r22, -1.0, 1.0)))


def interp(t, T, X):
    return np.stack([np.interp(t, T, X[:, i]) for i in range(X.shape[1])], axis=1)


def score(path):
    d = np.load(path, allow_pickle=True)
    rig = str(d["rig"])
    out = dict(run=path.split("/")[-1], rig=rig, aborted=bool(d["aborted"]), reason=str(d["reason"]))
    pay = d["payload"]
    if pay.shape[1] >= 8:
        p_end = pay[-1, 1:4]
        q_end = pay[-1, 4:8]
        out["picked"] = bool(np.max(pay[:, 3]) > CAP_TOP + BOX_HALF + 0.10)
        on_place = (np.linalg.norm(p_end[:2] - PLACE_XY) < CAP_R and
                    abs(p_end[2] - (CAP_TOP + BOX_HALF)) < 0.01)
        out["placed"] = bool(on_place and tilt_deg(q_end[None])[0] < 5.0)
        out["place_off_mm"] = float(np.linalg.norm(p_end[:2] - PLACE_XY) * 1e3)
        out["payload_tilt_end"] = float(tilt_deg(q_end[None])[0])
        out["payload_end"] = np.round(p_end, 3).tolist()
    od = d["odom"]
    if od.shape[1] >= 13:
        direct = od[:, 11] > 0.5
        tl = tilt_deg(od[:, 7:11])
        out["tilt_max_direct"] = float(tl[direct].max()) if direct.any() else float("nan")
        per = {}
        for k, s in enumerate(STEPS):
            m = direct & (od[:, 12] == k)
            if m.any():
                per[s] = float(tl[m].max())
        out["tilt_per_step"] = per
    ee, ref = d["ee"], d["ref"]
    if ee.shape[1] >= 9 and ref.shape[1] >= 12 and len(ref) > 10:
        rp = interp(ee[:, 0], ref[:, 0], ref[:, 1:4])
        live = (ee[:, 0] > ref[0, 0]) & (ee[:, 0] < ref[-1, 0])
        err = np.linalg.norm(ee[:, 1:4] - rp, axis=1)
        per = {}
        for k, s in enumerate(STEPS):
            m = live & (ee[:, 8] == k)
            if m.sum() > 5:
                per[s] = (float(np.sqrt(np.mean(err[m] ** 2)) * 1e3), float(err[m].max() * 1e3))
        out["ee_err_rms_max_mm"] = per
    # the arrival error when the gripper acted (the gripper marks)
    names, mt = list(d["marks_name"]), d["marks_t"]
    arr = d["arrival"]
    for what in ("close", "open"):
        key = f"gripper_{what}:start"
        if key in names and arr.shape[1] >= 4:
            t = mt[names.index(key)]
            i = np.searchsorted(arr[:, 0], t) - 1
            out[f"ee_err_at_{what}_mm"] = float(arr[max(i, 0), 3] * 1e3)
    # the claw's error while HOLDING above the target (the last 1.5 s before
    # Pick / Place): each law's steady offset, what the descent trim cancels
    if arr.shape[1] >= 11:
        for step, leg in (("pick", 1), ("place", 3)):
            # arrival columns: t, leg, claw, |e|, ex, ey, ez, tol, within, settled, phase
            m = (arr[:, 1] == leg) & (arr[:, 10] == 2)        # phase 2 = holding above
            if m.sum() > 3:
                tm = arr[m, 0]
                mm = m.copy()
                mm[m] = tm > tm[-1] - 1.5
                out[f"hover_err_{step}_mm"] = float(np.mean(arr[mm, 3]) * 1e3)
    # slip of the payload against the claw while carried (exit_pick .. place)
    claw = d["claw"]
    if "exit_pick:start" in names and "place:start" in names and claw.shape[1] >= 4 and pay.shape[1] >= 8:
        t0, t1 = mt[names.index("exit_pick:start")] + 1.0, mt[names.index("place:start")]
        m = (pay[:, 0] > t0) & (pay[:, 0] < t1)
        if m.sum() > 10:
            c = interp(pay[m, 0], claw[:, 0], claw[:, 1:4])
            dist = np.linalg.norm(pay[m, 1:4] - c, axis=1)
            # slip = the claw-payload DISTANCE changing (a rigid grasp only rotates)
            out["carry_slip_mm"] = float((dist.max() - dist.min()) * 1e3)
            out["carry_payload_tilt_max"] = float(tilt_deg(pay[m, 4:8]).max())
    if "direct" in names:
        t_dir = mt[names.index("direct")]
        out["mission_s"] = float(mt[-1] - t_dir)
    # step times
    st = {}
    for s in STEPS:
        a, b = f"{s}:start", f"{s}:end"
        if a in names and b in names:
            st[s] = round(float(mt[names.index(b)] - mt[names.index(a)]), 1)
    out["step_s"] = st
    return out


def main():
    for p in sys.argv[1:]:
        r = score(p)
        print(f"== {r['run']} ({r['rig']}): {'ABORTED: ' + r['reason'] if r['aborted'] else 'completed'}")
        for k, v in r.items():
            if k in ("run", "rig", "aborted", "reason"):
                continue
            if isinstance(v, dict):
                print(f"   {k}:")
                for kk, vv in v.items():
                    print(f"      {kk:18s} {np.round(vv, 2).tolist() if isinstance(vv, tuple) else round(vv, 2)}")
            else:
                print(f"   {k:20s} {round(v, 2) if isinstance(v, float) else v}")


if __name__ == "__main__":
    main()
