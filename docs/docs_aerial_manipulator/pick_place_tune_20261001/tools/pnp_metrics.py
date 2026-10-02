#!/usr/bin/env python3
"""Score pnp_mission_driver.py runs.

    /usr/bin/python3 pnp_metrics.py runs/pnp_r1.npz [runs/pnp_r2.npz ...]

Per run: did the vehicle disturb the payload before the grasp (approach
collision), how the claw sat on the handle, did the payload lift, slip or
tilt in the grasp, where it ended up on the place pillar, and how the vehicle
flew (tilt, attitude error, saturation, joint torque, claw and CoM tracking per
leg, the claw's lateral error during the two vertical descents, and what the
L1 observer booked for the payload).
"""
import sys

import numpy as np

PICK_XY, PLACE_XY = np.array([1.0, 1.0]), np.array([-1.0, -1.0])
BOX_H, HANDLE_H, PILLAR_TOP = 0.065, 0.200, 1.0
LEGS = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place",
        "go_to_land_start", "execute_land"]
O = 2   # dbg columns: [t, leg, *debug]
D_TAU, D_U1, D_ER, D_DHAT, D_MOT, D_XCD, D_XC, D_NSAT, D_NCLAMP = (
    slice(O + 13, O + 17), O + 17, slice(O + 28, O + 31), slice(O + 31, O + 41), slice(O + 41, O + 45),
    slice(O + 45, O + 48), slice(O + 48, O + 51), O + 51, O + 88)


def R_of(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(q):
    return np.degrees(np.arccos(np.clip(R_of(q)[2, 2], -1, 1)))


def at(arr, t):
    """row of arr (col 0 = time) nearest to t"""
    return arr[int(np.argmin(np.abs(arr[:, 0] - t)))]


def interp(arr, cols, t):
    return np.column_stack([np.interp(t, arr[:, 0], arr[:, c]) for c in cols])


def score(path):
    d = np.load(path, allow_pickle=True)
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    pl, ee, ref, dbg, grip, odom = d["payload"], d["ee"], d["ref"], d["dbg"], d["grip"], d["odom"]
    p0 = d["payload0"]
    print(f"\n=== {path}: {'ABORTED: ' + str(d['reason']) if bool(d['aborted']) else 'completed'}")
    if "info" in d and d["info"].size > 13:
        print(f"  planned leg durations {np.round(d['info'][8:14], 1).tolist()} s")
    t_close = marks.get("gripper_close:start", np.inf)
    t_closed = marks.get("gripper_close:end", np.inf)
    t_open = marks.get("gripper_open:start", np.inf)
    t_opened = marks.get("gripper_open:end", np.inf)

    # 1 -- the payload before the grasp: any motion = the approach touched it
    pre = pl[pl[:, 0] < t_close]
    dmax = np.max(np.linalg.norm(pre[:, 1:4] - p0[:3], axis=1)) if len(pre) else np.nan
    dang = max(tilt_deg(r[4:8]) for r in pre) if len(pre) else np.nan
    print(f"  payload before the close: moved <= {dmax * 1e3:.2f} mm, tilt <= {dang:.2f} deg  "
          f"({'UNDISTURBED' if dmax < 0.002 else 'DISTURBED -- approach contact'})")

    # 2 -- the grasp
    if np.isfinite(t_close):
        g = at(grip, t_closed)
        Rp = R_of(at(pl, t_close)[4:8])
        top = at(pl, t_close)[1:4] + Rp @ np.array([0, 0, 0.5 * BOX_H + HANDLE_H])
        claw = at(ee, t_close)[1:4]
        rel = Rp.T @ (claw - top)
        print(f"  grasp: gripper stalled at {np.degrees(g[1]):.1f} deg (effort {g[2]:.3f}); claw vs handle top "
              f"[x {rel[0] * 1e3:+.1f}, y {rel[1] * 1e3:+.1f}, z {rel[2] * 1e3:+.1f}] mm (payload frame; "
              f"designed z -15)")
        moved = np.linalg.norm(at(pl, t_closed)[1:4] - at(pl, t_close)[1:4])
        print(f"  closing moved the payload {moved * 1e3:.2f} mm")

    # 3 -- carry: lift, slip in the claw frame, tilt
    if np.isfinite(t_closed) and np.isfinite(t_open):
        tt = pl[(pl[:, 0] > t_closed + 0.5) & (pl[:, 0] < t_open), 0]
        if len(tt) > 10:
            P = interp(pl, range(1, 8), tt)
            src = d["claw"] if "claw" in d.files and d["claw"].shape[1] > 3 else ee
            E = interp(src, [1, 2, 3], tt)
            rel = P[:, :3] - E                      # box centre below the claw, WORLD frame
            lift = P[:, 2].max() - p0[2]
            hang = -rel[:, 2]
            sway = np.linalg.norm(rel[:, :2] - rel[0, :2], axis=1)
            tilts = np.array([tilt_deg(q) for q in P[:, 3:7]])
            print(f"  carry: payload lifted up to {lift * 1e3:.0f} mm above its rest; box centre "
                  f"{hang.min() * 1e3:.0f}-{hang.max() * 1e3:.0f} mm below the claw (vertical slip "
                  f"{(hang.max() - hang.min()) * 1e3:.1f} mm), lateral sway max {sway.max() * 1e3:.0f} mm "
                  f"(median {np.median(sway) * 1e3:.0f}); payload tilt max {tilts.max():.2f} deg")
            leg_tilt = [(LEGS[int(k)], tilts[(interp(pl, [8], tt)[:, 0] == k)].max())
                        for k in (2, 3) if np.any(interp(pl, [8], tt)[:, 0] == k)]
            print("         per leg tilt: " + ", ".join(f"{n} {v:.2f} deg" for n, v in leg_tilt))

    # 4 -- the place
    if np.isfinite(t_opened):
        fin = pl[-1]
        hd = np.hypot(fin[1] - PLACE_XY[0], fin[2] - PLACE_XY[1])
        rest_z = PILLAR_TOP + 0.5 * BOX_H
        # on the pillar = resting at its height, upright, the box centre (its CoM
        # in plan) over the 50 mm-radius pillar top
        ok = hd < 0.050 and abs(fin[3] - rest_z) < 0.005 and tilt_deg(fin[4:8]) < 2
        drop = at(pl, t_open)[3] - rest_z
        print(f"  place: box centre {hd * 1e3:.1f} mm off the pillar axis, z {fin[3] - rest_z:+.4f} m vs resting, "
              f"tilt {tilt_deg(fin[4:8]):.2f} deg, released {drop * 1e3:.1f} mm above rest -> "
              f"{'ON THE PILLAR, UPRIGHT' if ok else 'NOT ON THE PILLAR'} (pillar radius 50 mm)")

    # 5 -- the vehicle
    dd = odom[odom[:, 11] > 0.5]
    tv = np.array([tilt_deg(q) for q in dd[:, 7:11]]) if len(dd) else np.zeros(1)
    eR = np.linalg.norm(dbg[:, D_ER], axis=1)
    tau = np.abs(dbg[:, D_TAU])
    print(f"  vehicle (DIRECT {dd[-1, 0] - dd[0, 0] if len(dd) else 0:.0f} s): tilt max {tv.max():.2f} deg, |e_R| max "
          f"{eR.max():.3f}, saturated ticks {int((dbg[:, D_NSAT] > 0).sum())}, joint-clamp ticks "
          f"{int((dbg[:, D_NCLAMP] > 0).sum())}, peak |tau| {np.round(tau.max(0), 2).tolist()} N.m, "
          f"motors {dbg[:, D_MOT].min():.3f}..{dbg[:, D_MOT].max():.3f}")

    # 6 -- tracking per leg (claw: measured FK vs the streamed reference; CoM)
    print("  leg                  claw err mean/max [mm]   CoM err mean/max [mm]   tilt max   |e_R| max")
    for k, name in enumerate(LEGS):
        s = ee[ee[:, 8] == k]
        b = dbg[dbg[:, 1] == k]
        if len(s) < 5 or len(b) < 5:
            continue
        r = interp(ref, [1, 2, 3], s[:, 0])
        e = np.linalg.norm(s[:, 1:4] - r, axis=1)
        c = np.linalg.norm(b[:, D_XC] - b[:, D_XCD], axis=1)
        o = odom[odom[:, 12] == k]
        tl = max(tilt_deg(q) for q in o[:, 7:11]) if len(o) else np.nan
        print(f"  {name:18s}   {e.mean() * 1e3:6.1f} / {e.max() * 1e3:6.1f}         {c.mean() * 1e3:6.1f} / "
              f"{c.max() * 1e3:6.1f}        {tl:5.2f}     {np.linalg.norm(b[:, D_ER], axis=1).max():.3f}")

    # 7 -- the two vertical descents: lateral claw error vs the target, from
    # the REAL claw (claw_0, between the pads) when recorded, else the model's
    claw = d["claw"] if "claw" in d.files and d["claw"].shape[1] > 3 else None
    for k, xy, lab in ((1, PICK_XY, "pick"), (3, PLACE_XY, "place")):
        end = marks.get(f"{LEGS[k]}:end")
        if end is None:
            continue
        src = claw if claw is not None else ee
        s = src[(src[:, 0] > end - 4.0) & (src[:, 0] <= end + 2.0)]
        lat = np.hypot(s[:, 1] - xy[0], s[:, 2] - xy[1])
        print(f"  {lab} descent (last 4 s of the leg + 2 s), {'REAL' if claw is not None else 'model'} claw: "
              f"lateral error to the pillar axis max {lat.max() * 1e3:.1f} mm, at the end {lat[-1] * 1e3:.1f} mm "
              f"(fingertip clearance 3.5-4 mm a side); z at the end {s[-1, 3]:.4f}")
        if claw is not None:
            m = (ee[:, 0] > end) & (ee[:, 0] < end + 2.0)
            if m.sum() > 3:
                c = interp(claw, [1, 2, 3], ee[m, 0])
                off = (c - ee[m, 1:4]).mean(0)
                print(f"    model claw (current_ee) vs real claw at the {lab} hold: real - model = "
                      f"{np.round(off * 1e3, 1).tolist()} mm (world)")

    # 8 -- what the observer booked for the payload (filtered d_hat, mean of a 3 s hold)
    def hold(t0, t1):
        b = dbg[(dbg[:, 0] > t0) & (dbg[:, 0] < t1)]
        return b[:, D_DHAT].mean(0) if len(b) else np.full(10, np.nan)
    if np.isfinite(t_close) and np.isfinite(t_open):
        before = hold(t_close - 3.5, t_close - 0.5)
        carry_end = marks.get("go_to_place_start:end", t_open)
        during = hold(carry_end - 0.5, carry_end + 2.5)
        after = hold(t_opened + 1.0, t_opened + 4.0)
        np.set_printoptions(precision=3, suppress=True)
        print(f"  d_hat (filtered) before grasp {before[[2, 3, 4, 5]]} (t_z, r_xyz)  joints {before[6:]}")
        print(f"  d_hat (filtered) carrying     {during[[2, 3, 4, 5]]}  joints {during[6:]}")
        print(f"  d_hat (filtered) after release {after[[2, 3, 4, 5]]}  joints {after[6:]}")
        print(f"  payload booked on the translational z channel: {during[2] - before[2]:+.2f} N "
              f"(weight 1.96 N); u1 mean before {dbg[(dbg[:, 0] > t_close - 3.5) & (dbg[:, 0] < t_close - 0.5), D_U1].mean():.2f}"
              f" / carrying {dbg[(dbg[:, 0] > carry_end - 0.5) & (dbg[:, 0] < carry_end + 2.5), D_U1].mean():.2f} N")
    return d


if __name__ == "__main__":
    for p in sys.argv[1:]:
        score(p)
