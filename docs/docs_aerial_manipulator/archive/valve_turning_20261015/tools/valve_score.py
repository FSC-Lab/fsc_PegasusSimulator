#!/usr/bin/env python3
"""Score a valve-turning run (pl_mission.py's npz, flown by run_valve.sh):

    PYTHONNOUSERSITE=1 /usr/bin/python3 valve_score.py runs/<tag>.npz [--plot out.png]

Prints, per step and for the TURN (the "push" step):
  * the beam's rotation from obj_0's yaw (unwrapped): total, and how much of
    it happened in the turn step; the beam's final angle vs the start
  * the claw's error vs the planned claw (the streamed reference r_ed), the
    planned radius vs the measured one about the captured axis, the claw
    height
  * the CoM tracking error (odometry vs x_cd), the heading error, the tilt
  * the vehicle-rotor contact force (the PhysX truth, /push_pull_truth/contact,
    its tangential and radial parts about the axis), the joints' range and the
    arm-torque peak, the law's debug clamp / saturation flags
  * completed / aborted, with the reason
The axis is obj_0's position at rest (the captured body) + Rz(yaw) * the planner's
push_pull_valve_axis_offset (2026-10-28: obj_0 = the markers' centroid, 72.6 mm off the
shaft) -- read from the run's <tag>.yaml beside the npz, or --axis-offset; [0, 0, 0] = obj_0
on the axis (every run before 2026-10-28). The start is the box's first sample. Columns
follow pl_mission.py's `L` dict.
"""
import os
import re
import argparse

import numpy as np


def yaw_of(q):           # q = [w, x, y, z]
    return np.arctan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                      1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))


def unwrap(a):
    return np.unwrap(a)


def tilt_of(q):
    w, x, y, z = q.T
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(r22, -1.0, 1.0)))


def window(t, t0, t1):
    return (t >= t0) & (t <= t1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--plot", default="")
    ap.add_argument("--axis-offset", default="",
                    help="'x,y,z': the planner's push_pull_valve_axis_offset (default: the run's <tag>.yaml, else 0)")
    a = ap.parse_args()
    off = np.zeros(3)
    if a.axis_offset:
        off = np.array([float(v) for v in a.axis_offset.split(",")])
    else:
        y = os.path.splitext(a.npz)[0] + ".yaml"
        m = re.search(r"^    push_pull_valve_axis_offset:\s*\[([^\]]*)\]", open(y).read(), re.M) if os.path.exists(y) else None
        if m:
            off = np.array([float(v) for v in m.group(1).split(",")])
    d = np.load(a.npz, allow_pickle=True)
    steps = ["go_to_start", "ready", "close", "push", "release", "exit", "go_to_land"]
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    print(f"{a.npz}: {'ABORTED: ' + str(d['reason']) if bool(d['aborted']) else 'completed'}")

    box = d["box"]                       # t, x, y, z, qw, qx, qy, qz, step
    t_box = box[:, 0]
    yb = yaw_of(box[:, 4:8])
    ax_all = box[:, 1:4] + np.c_[np.cos(yb) * off[0] - np.sin(yb) * off[1],
                                 np.sin(yb) * off[0] + np.cos(yb) * off[1], np.full(len(yb), off[2])]
    axis = ax_all[0]
    beam = np.degrees(unwrap(yb))
    beam -= beam[0]
    print(f"  axis (obj_0 + Rz(yaw) * [{off[0]:+.4f}, {off[1]:+.4f}, {off[2]:+.4f}]) at rest [{axis[0]:.4f}, "
          f"{axis[1]:.4f}, {axis[2]:.4f}] m; moved {np.linalg.norm(ax_all - axis, axis=1).max() * 1e3:.1f} mm at most "
          f"(an axis must not move); obj_0 itself moved {np.linalg.norm(box[:, 1:4] - box[0, 1:4], axis=1).max() * 1e3:.1f} mm")
    print(f"  BEAM: turned {beam[-1]:+.1f} deg over the run (min {beam.min():+.1f}, max {beam.max():+.1f})")
    if "push:start" in marks and "push:end" in marks:
        m = window(t_box, marks["push:start"], marks["push:end"])
        if m.any():
            b = beam[m]
            print(f"        in the turn step ({marks['push:end'] - marks['push:start']:.1f} s): "
                  f"{b[-1] - b[0]:+.1f} deg, peak rate "
                  f"{np.abs(np.gradient(b, t_box[m])).max():.1f} deg/s")
    for name in ("ready", "release", "exit", "go_to_land"):
        k0, k1 = f"{name}:start", f"{name}:end"
        if k0 in marks and k1 in marks:
            m = window(t_box, marks[k0], marks[k1])
            if m.any():
                print(f"        during {name}: {beam[m][-1] - beam[m][0]:+.1f} deg")

    # the claw vs the plan
    claw = d["claw"]                     # t, x, y, z, step
    ref = d["ref"]                       # t, r_ed xyz, x_cd xyz, q_d(4), b1_d xy, ..., step
    odom = d["odom"]                     # t, p xyz, v xyz, q wxyz, direct, step
    if len(ref) and len(claw):
        def at(tq, ts, X):
            return np.column_stack([np.interp(tq, ts, X[:, k]) for k in range(X.shape[1])])
        for name, k in (("go_to_start", 0), ("ready", 1), ("push", 3), ("exit", 5), ("go_to_land", 6)):
            k0, k1 = f"{name}:start", f"{name}:end"
            if k0 not in marks or k1 not in marks:
                continue
            m = window(claw[:, 0], marks[k0], marks[k1])
            if m.sum() < 5:
                continue
            r = at(claw[m, 0], ref[:, 0], ref[:, 1:4])
            e = np.linalg.norm(claw[m, 1:4] - r, axis=1)
            mo = window(odom[:, 0], marks[k0], marks[k1])
            xc = at(odom[mo, 0], ref[:, 0], ref[:, 4:7])
            ec = np.linalg.norm(odom[mo, 1:4] - xc, axis=1) if mo.any() else np.zeros(1)
            tl = tilt_of(odom[mo, 7:11]) if mo.any() else np.zeros(1)
            line = (f"  {name:11s}: claw err mean {e.mean() * 1e3:5.1f} / max {e.max() * 1e3:5.1f} mm; "
                    f"odom-vs-x_cd mean {ec.mean() * 1e3:5.1f} / max {ec.max() * 1e3:5.1f} mm; tilt max {tl.max():.1f} deg")
            if name == "push":
                rad_m = np.hypot(claw[m, 1] - axis[0], claw[m, 2] - axis[1])
                rad_r = np.hypot(r[:, 0] - axis[0], r[:, 1] - axis[1])
                line += (f"\n               claw radius about the axis: planned {rad_r.mean():.4f} m, measured "
                         f"{rad_m.mean():.4f} +- {rad_m.std():.4f} m (min {rad_m.min():.4f}, max {rad_m.max():.4f}); "
                         f"claw height {claw[m, 3].mean():.4f} +- {claw[m, 3].std() * 1e3:.1f} mm")
            print(line)

    # THE TURN TABLE (v1-v5): the base's lead along its circle vs the planned
    # yaw rate (the lead scaled ~2.5 mm per deg/s at 36 s), the beam vs the
    # plan, the arm's margin to its stops, the heading error (the reference's
    # b1_d is the MODEL heading: actual yaw - 90 deg) and the tilt, every 3 s
    J = d["joints"]                      # t, q1..4, eff1..4, step (broadcaster order)
    if len(ref) and len(odom) and len(J) and "push:start" in marks:
        # an aborted turn has no push:end: table it to the last DIRECT sample
        t0 = marks["push:start"]
        t1 = marks.get("push:end", odom[odom[:, 11] > 0.5, 0].max() if (odom[:, 11] > 0.5).any() else t0)
        rb = np.degrees(unwrap(np.arctan2(ref[:, 2] - axis[1], ref[:, 1] - axis[0])))
        rb -= np.interp(t0, ref[:, 0], rb)

        def base_lead(tt):
            o = at([tt], odom[:, 0], odom[:, 1:4])[0]
            xc = at([tt], ref[:, 0], ref[:, 4:7])[0]
            rad = o[:2] - axis[:2]
            rad /= max(np.linalg.norm(rad), 1e-9)
            return (o[:2] - xc[:2]) @ np.array([-rad[1], rad[0]])
        b0 = base_lead(t0)
        print("  TURN TABLE: t[s] | plan deg | rate deg/s | beam-plan mm | base lead mm | q2 q3 deg | heading err deg | tilt")
        worst = {"lead": 0.0, "q2": 99.0, "q3": -99.0, "herr": 0.0}
        for tt in np.arange(t0, t1 + 1e-6, 3.0):
            rate = (np.interp(tt + 0.25, ref[:, 0], rb) - np.interp(tt - 0.25, ref[:, 0], rb)) / 0.5
            jq = np.degrees(at([tt], J[:, 0], J[:, 1:5])[0])
            o = at([tt], odom[:, 0], odom[:, 7:11])[0]
            tl = np.degrees(np.arccos(np.clip(1.0 - 2.0 * (o[1] ** 2 + o[2] ** 2), -1.0, 1.0)))
            psi = np.degrees(np.arctan2(2.0 * (o[0] * o[3] + o[1] * o[2]), 1.0 - 2.0 * (o[2] ** 2 + o[3] ** 2)))
            b1 = at([tt], ref[:, 0], ref[:, 11:13])[0]
            herr = (psi - (np.degrees(np.arctan2(b1[1], b1[0])) + 90.0) + 180.0) % 360.0 - 180.0
            lead = (base_lead(tt) - b0) * 1e3
            lag = (np.interp(tt, t_box, beam) - np.interp(tt, ref[:, 0], rb)) * np.pi / 180.0 * 0.16 * 1e3
            worst["lead"] = max(worst["lead"], abs(lead))
            worst["q2"] = min(worst["q2"], jq[1])
            worst["q3"] = max(worst["q3"], jq[2])
            worst["herr"] = max(worst["herr"], abs(herr))
            print(f"    {tt - t0:5.1f} | {np.interp(tt, ref[:, 0], rb):6.1f} | {rate:5.1f} | {lag:6.1f} | {lead:6.1f} | "
                  f"{jq[1]:5.1f} {jq[2]:5.1f} | {herr:6.1f} | {tl:4.1f}")
        print(f"  TURN: worst base lead {worst['lead']:.0f} mm, q2 min {worst['q2']:.1f} (stop -20), q3 max "
              f"{worst['q3']:.1f} (stop 50), heading err max {worst['herr']:.1f} deg")

    # the contact truth: vehicle-rotor force, tangential / radial about the axis
    ct = d["contact"]                    # t, [0:3] normal, [3:6] friction, ..., [18] T, [19] npts, step
    if len(ct) and "push:start" in marks and "push:end" in marks:
        m = window(ct[:, 0], marks["push:start"], marks["push:end"])
        if m.any():
            F = ct[m, 1:4] + ct[m, 4:7]
            # sign: the box push's analysis calibrates it on the table; here
            # only the magnitude and the tangential / radial split are read
            cl = at(ct[m, 0], claw[:, 0], claw[:, 1:4]) if len(claw) else None
            if cl is not None:
                radial = cl[:, :2] - axis[:2]
                radial /= np.maximum(np.linalg.norm(radial, axis=1, keepdims=True), 1e-9)
                tang = np.column_stack([-radial[:, 1], radial[:, 0]])
                ft = np.einsum("ij,ij->i", F[:, :2], tang)
                fr = np.einsum("ij,ij->i", F[:, :2], radial)
                print(f"  CONTACT (truth) in the turn: |F| mean {np.linalg.norm(F, axis=1).mean():.2f} N, "
                      f"tangential {np.abs(ft).mean():.2f} N (max {np.abs(ft).max():.2f}), radial "
                      f"{np.abs(fr).mean():.2f} N, vertical {np.abs(F[:, 2]).mean():.2f} N")

    # joints and torques
    if len(J) and "push:start" in marks and "push:end" in marks:
        m = window(J[:, 0], marks["push:start"], marks["push:end"])
        if m.any():
            q = np.degrees(J[m, 1:5])
            print(f"  JOINTS in the turn (broadcaster order): min {np.round(q.min(0), 1).tolist()} max "
                  f"{np.round(q.max(0), 1).tolist()} deg; |tau| peak {np.nanmax(np.abs(J[m, 5:9])):.2f} N.m")
    dbg = d["dbg"]
    if len(dbg) and dbg.shape[1] > 101:
        m = dbg[:, 1] == 3
        if m.any():
            F = dbg[m, 2 + 97:2 + 100]
            print(f"  LAW in the turn: raw F_hat |.| mean {np.linalg.norm(F, axis=1).mean():.2f} N; "
                  f"[105] chi_free (1 = free, 0 = CONTACT) mean {np.nanmean(dbg[m, 2 + 105]):.2f}")
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        ax[0].plot(t_box, beam)
        ax[0].set_ylabel("beam [deg]")
        if len(claw) and len(ref):
            r = at(claw[:, 0], ref[:, 0], ref[:, 1:4])
            ax[1].plot(claw[:, 0], np.linalg.norm(claw[:, 1:4] - r, axis=1) * 1e3)
            ax[1].set_ylabel("claw err [mm]")
        if len(odom):
            ax[2].plot(odom[:, 0], tilt_of(odom[:, 7:11]))
            ax[2].set_ylabel("tilt [deg]")
        for k, (t0, n) in enumerate(zip(d["marks_t"], d["marks_name"])):
            if str(n).endswith(":start"):
                for x in ax:
                    x.axvline(t0, color="k", lw=0.5, alpha=0.4)
        ax[2].set_xlabel("t [s]")
        fig.tight_layout()
        fig.savefig(a.plot, dpi=120)
        print(f"  plot: {a.plot}")


if __name__ == "__main__":
    main()
