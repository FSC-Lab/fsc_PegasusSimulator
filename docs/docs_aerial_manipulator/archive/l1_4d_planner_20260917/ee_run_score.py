#!/usr/bin/env python3
"""Score an END-EFFECTOR TRAJECTORY run recorded by fsc_trajectory_planner's
ee_trajectory_sim_driver.py, with the metrics Command.md 7.15.12 used for the
6-D circle flights, so the two tables can sit side by side.

    PYTHONNOUSERSITE=1 /usr/bin/python3 ee_run_score.py <run.npz> [...]

npz `ee` columns: [t, measured EE xyz (planner's current_ee: measured joints on
measured odometry), reference EE xyz (planner's reference_pose)], RUN phase only.

  raw err     |measured - reference| at the same instant, mean / max
  lag         the time shift tau minimising mean |measured(t) - reference(t - tau)|
              (0..5 s, 10 ms grid) -- a pure first-order-lag reading
  residual    mean error at that tau
  radius      distance of the measured / reference EE from the world origin in
              xy (the circle is centred on it: ee_traj_center_origin), mean + sd
  centre      mean measured xy over the run
  z err       mean |z| error
"""
import sys

import numpy as np


def score(path):
    d = np.load(path, allow_pickle=True)
    ee = d["ee"]
    out = {"file": path.split("/")[-1], "aborted": bool(d["aborted"]),
           "reason": str(d["reason"]), "shape": str(d["shape"]),
           "scale_frac": float(d["scale"])}
    info = d["info"]
    if info.size >= 4:
        out.update(s=float(info[0]), s_max=float(info[1]), T=float(info[2]), lap=float(info[3]))
    if ee.shape[0] < 50:
        out["samples"] = int(ee.shape[0])
        return out
    t, cur, ref = ee[:, 0], ee[:, 1:4], ee[:, 4:7]
    err = np.linalg.norm(cur - ref, axis=1)
    out.update(samples=int(ee.shape[0]), duration=float(t[-1] - t[0]),
               raw_mean_mm=1e3 * err.mean(), raw_max_mm=1e3 * err.max(),
               z_err_mm=1e3 * np.abs(cur[:, 2] - ref[:, 2]).mean())
    v = np.gradient(ref, t, axis=0)
    out["ref_speed_mps"] = float(np.median(np.linalg.norm(v[:, :2], axis=1)))
    best = (np.inf, 0.0)
    for tau in np.arange(0.0, 5.0, 0.01):
        tq = t - tau
        m = tq >= t[0]
        if m.sum() < 0.5 * len(t):
            break
        rs = np.column_stack([np.interp(tq[m], t, ref[:, k]) for k in range(3)])
        e = np.linalg.norm(cur[m] - rs, axis=1).mean()
        if e < best[0]:
            best = (e, tau)
    out.update(lag_s=best[1], residual_mm=1e3 * best[0])
    if out["shape"] == "circle":
        rc, rr = np.hypot(cur[:, 0], cur[:, 1]), np.hypot(ref[:, 0], ref[:, 1])
        out.update(radius_flown_m=rc.mean(), radius_flown_sd_mm=1e3 * rc.std(),
                   radius_ref_m=rr.mean(), radius_ref_sd_mm=1e3 * rr.std(),
                   centre_x_m=cur[:, 0].mean(), centre_y_m=cur[:, 1].mean())
    # path extent along the reference's own principal axes (shape-agnostic:
    # for the figure-8 this is flown vs commanded length and width)
    c0 = ref[:, :2].mean(axis=0)
    _, _, vt = np.linalg.svd(ref[:, :2] - c0, full_matrices=False)
    pr, pc = (ref[:, :2] - c0) @ vt.T, (cur[:, :2] - c0) @ vt.T
    out.update(extent_ref_major_m=float(np.ptp(pr[:, 0])), extent_flown_major_m=float(np.ptp(pc[:, 0])),
               extent_ref_minor_m=float(np.ptp(pr[:, 1])), extent_flown_minor_m=float(np.ptp(pc[:, 1])))
    return out


def main():
    for p in sys.argv[1:]:
        o = score(p)
        print(f"== {o['file']}  shape={o['shape']} aborted={o['aborted']} {o['reason']}")
        for k, v in o.items():
            if k in ("file", "shape", "aborted", "reason"):
                continue
            print(f"   {k:<20s} {v:.4f}" if isinstance(v, float) else f"   {k:<20s} {v}")


if __name__ == "__main__":
    main()
