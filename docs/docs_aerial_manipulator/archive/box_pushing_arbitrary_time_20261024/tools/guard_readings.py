#!/usr/bin/env python3
"""What this rig's planner guards READ in a run (2026-10-25, the guard tuning): over the contact
phase (end of Ready -> start of Exit, the driver's marks) the box lag along the push (arrival [11],
recorded col 12), the box tilt (arrival [12], col 13), the force guard's input (|F_hat| from
atc_control_debug [97..99] = recorded dbg cols 99..101, low-passed at 3 rad/s as the planner does)
and the lowest q3. The guard values are set from these nominal readings, not from another rig's.

    PYTHONNOUSERSITE=1 /usr/bin/python3 tools/guard_readings.py runs/t6_adm3_g100.npz [...]
"""
import sys
import numpy as np


def window(d):
    names = list(d["marks_name"]); ts = d["marks_t"]
    t0 = ts[names.index("ready:end")] if "ready:end" in names else -np.inf
    t1 = ts[names.index("exit:start")] if "exit:start" in names else np.inf
    return t0, t1


def main():
    for p in sys.argv[1:]:
        d = np.load(p, allow_pickle=True)
        t0, t1 = window(d)
        a = d["arrival"]; m = (a[:, 0] >= t0) & (a[:, 0] <= t1)
        lag, tilt = a[m, 12], a[m, 13]
        g = d["dbg"].astype(float); mg = (g[:, 0] >= t0) & (g[:, 0] <= t1)
        F = g[:, 99:102]; t = g[:, 0]; lp = np.zeros(3); fmax = 0.0
        for i in range(1, len(t)):
            dt = max(t[i] - t[i - 1], 0.0)
            if not np.all(np.isfinite(F[i])):
                continue
            lp += (1.0 - np.exp(-3.0 * dt)) * (F[i] - lp)
            if mg[i]:
                fmax = max(fmax, float(np.linalg.norm(lp)))
        j = d["joints"]; mj = (j[:, 0] >= t0) & (j[:, 0] <= t1)
        names = list(d["joint_names"]); iq3 = 1 + names.index("joint3") if "joint3" in names else 3
        q3 = np.degrees(np.nanmin(j[mj, iq3])) if mj.any() else np.nan
        print("%-28s contact %.1f s | box lag max %+6.1f / min %+6.1f mm | box tilt max %.2f deg | "
              "|F_hat| (3 rad/s) max %.2f N | q3 min %.1f deg"
              % (p.split("/")[-1], t1 - t0, 1e3 * np.nanmax(lag), 1e3 * np.nanmin(lag),
                 np.nanmax(tilt), fmax, q3))


if __name__ == "__main__":
    main()
