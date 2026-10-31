#!/usr/bin/env python3
"""The mocap guard's predicted centre (2026-10-30), replayed over every recorded hardware flight that
has both the drone's /uav_0/mocap and PX4's fmu/out/vehicle_odometry (EKF2), old gate vs new.

Both streams are as RECORDED ON THE LAPTOP (receive times), so EKF2's velocity reaches the replay a
WiFi hop later than it reaches the Orin's guard -- older, i.e. the harder case for the prediction.
Per flight: samples rejected and the guard's episodes, without (the 10-21 guard) and with EKF2's
velocity, and every sample the two classify differently (time, distance from the last good position,
distance from the predicted centre, EKF2 speed).

    PYTHONNOUSERSITE=1 /usr/bin/python3 -I guard_predict_replay.py
"""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ARCH, "mocap_safety_guard_20261021", "tools"))
import guard_reference as G  # noqa: E402

AIR_Z = 0.45


def ekf_velocity(z):
    """(t, v ENU) from vehicle_odometry; NaN where the frame is unknown or the velocity invalid."""
    t = z["vodom__recv"].astype(float)
    v = z["vodom__velocity"].astype(float)
    fr = z["vodom__velocity_frame"].astype(float)
    q = z["vodom__q"].astype(float)   # w, x, y, z, NED <- FRD
    out = np.full_like(v, np.nan)
    ned = fr == 1
    out[ned] = np.c_[v[ned, 1], v[ned, 0], -v[ned, 2]]
    frd = fr == 3
    for i in np.where(frd)[0]:
        w, x, y, zq = q[i]
        R = np.array([[1 - 2 * (y * y + zq * zq), 2 * (x * y - w * zq), 2 * (x * zq + w * y)],
                      [2 * (x * y + w * zq), 1 - 2 * (x * x + zq * zq), 2 * (y * zq - w * x)],
                      [2 * (x * zq - w * y), 2 * (y * zq + w * x), 1 - 2 * (x * x + y * y)]])
        vn = R @ v[i]
        out[i] = [vn[1], vn[0], -vn[2]]
    o = np.argsort(t, kind="stable")
    return t[o], out[o]


def load(path):
    z = np.load(path, allow_pickle=True)
    t = z["mocap__recv"].astype(float)
    p = np.c_[z["mocap__pose.position.x"], z["mocap__pose.position.y"], z["mocap__pose.position.z"]]
    v = np.c_[z["mocap__twist.linear.x"], z["mocap__twist.linear.y"], z["mocap__twist.linear.z"]]
    q = np.c_[z["mocap__pose.orientation.w"], z["mocap__pose.orientation.x"],
              z["mocap__pose.orientation.y"], z["mocap__pose.orientation.z"]]
    o = np.argsort(t, kind="stable")
    return (t[o], p[o], v[o], q[o]), ekf_velocity(z)


def classify(t, p, v, q, ekf):
    """Per-sample verdicts ('' good) through the reference's own check, plus the distances."""
    chk = G.SampleCheck()
    out = []
    for k in range(len(t)):
        vp = G.ekf_velocity_at(ekf, t[k]) if ekf is not None else None
        gp, gt, gv = chk.good_p, chk.good_t, chk.good_v
        r = chk.check(t[k], p[k], v[k], q[k], vp)
        d_old = d_new = np.nan
        if gp is not None:
            dt = t[k] - gt
            d_old = float(np.linalg.norm(p[k] - gp))
            if vp is not None:
                va = vp if gv is None else 0.5 * (gv + vp)
                sp = float(np.linalg.norm(va))
                if sp > G.PRED_V_MIN:
                    if sp > G.PRED_V_MAX:
                        va = va * (G.PRED_V_MAX / sp)
                    d_new = float(np.linalg.norm(p[k] - (gp + va * dt)))
        out.append((r, d_old, d_new, np.nan if vp is None else float(np.linalg.norm(vp))))
    return out


def main():
    files = sorted(glob.glob(os.path.join(ARCH, "*", "npz", "**", "*.npz"), recursive=True))
    seen = set()
    tot = {"old": 0, "new": 0, "n": 0}
    for f in files:
        try:
            z = np.load(f, allow_pickle=True)
            if not {"mocap__recv", "vodom__velocity", "vodom__recv"} <= set(z.files):
                continue
        except Exception:
            continue
        (t, p, v, q), ekf = load(f)
        key = (len(t), round(float(t[0]), 3))
        if key in seen:           # 10-10's folder carries copies of 10-09's bags
            continue
        seen.add(key)
        air = np.where(p[:, 2] > AIR_Z)[0]
        if len(air) < 200:
            continue
        t0, t1 = t[air[0]] + 1.0, t[air[-1]]
        lab = os.path.relpath(f, ARCH).replace("/npz/", " ").replace(".npz", "")
        ev_old, c_old, _ = G.run_ladder(t, p, v, q, t_start=t0, t_end=t1)
        ev_new, c_new, _ = G.run_ladder(t, p, v, q, t_start=t0, t_end=t1, ekf=ekf)
        n_old = c_old.n_jump + c_old.n_frozen
        n_new = c_new.n_jump + c_new.n_frozen
        tot["old"] += n_old
        tot["new"] += n_new
        tot["n"] += 1
        fresh = np.mean([G.ekf_velocity_at(ekf, x) is not None for x in t[(t >= t0) & (t <= t1)][::20]])

        def summ(ev):
            return ", ".join(f"{e.kind[:4]}/{e.reason}@{e.t0 - t0 + 1.0:.2f}" for e in ev) or "-"
        print(f"{lab:48s} air {t1 - t0:6.1f} s  EKF2 fresh {100 * fresh:3.0f} %   rejected {n_old:4d} -> {n_new:4d}"
              f"   episodes {len(ev_old):2d} -> {len(ev_new):2d}")
        if summ(ev_old) != summ(ev_new):
            print(f"      old: {summ(ev_old)}")
            print(f"      new: {summ(ev_new)}")
            w = (t >= t0) & (t <= t1)
            vo = classify(t[w], p[w], v[w], q[w], None)
            vn = classify(t[w], p[w], v[w], q[w], ekf)
            tw = t[w]
            diff = [i for i in range(len(tw)) if bool(vo[i][0]) != bool(vn[i][0])]
            for i in diff[:6]:
                print(f"        t {tw[i] - t0 + 1.0:7.3f} s: old '{vo[i][0] or 'good'}' new '{vn[i][0] or 'good'}'"
                      f"  |p - last good| {1e3 * vo[i][1]:6.1f} mm (old), {1e3 * vn[i][1]:6.1f} mm / "
                      f"predicted {1e3 * vn[i][2]:6.1f} mm (new), EKF2 {vn[i][3]:.2f} m/s")
            if len(diff) > 6:
                print(f"        ... {len(diff)} samples differ in all")
    print(f"\n{tot['n']} flights: rejected samples {tot['old']} -> {tot['new']}")


if __name__ == "__main__":
    main()
