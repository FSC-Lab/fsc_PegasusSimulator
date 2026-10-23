#!/usr/bin/env python3
"""Per-leg tracking error of EVERY reference-tracked state, 6-D vs 4-D.

    /usr/bin/python3 traj_errors.py [run.npz ...]        (default: the 5 headline runs)
    /usr/bin/python3 traj_errors.py --csv errors.csv ...

WHY THIS EXISTS. wb_compare_metrics.py reported one number per leg, the CoM
translation error |x_c - x_cd|, plus an "EE" column that was the norm of the
WHOLE 4-vector task error e_y -- i.e. three components in METRES added to one
component that is sin(heading error), DIMENSIONLESS. A heading error of 8.7 deg
contributes 0.151 to that norm and was printed as "151 mm" of position error.
That column is not a position error and must not be read as one.

Every leg of this mission is a COMPATIBLE TRAJECTORY: the whole-body planner
solves a dynamically consistent transition and streams the full reference set
(x_cd and derivatives, platform heading b1_d, EE position r_ed, EE heading
b1_de, joints q_d), so a step in x is no less a trajectory than the EE legs.
This scores each reference-tracked channel on its own, in its own unit:

  CoM position   |x_c - x_cd|                      [mm]   dbg[48..50] - dbg[45..47]
  CoM velocity   |v_c - x_cd_dot|                  [mm/s] log vs wbref
  attitude       asin|e_R|, the angle between R_0  [deg]  dbg[28..30]
                 and the commanded R_0c (roll and
                 pitch are what the position loop
                 demands; yaw is the commanded
                 platform heading)
  EE position    |e_y[0:3]|                        [mm]   dbg[24..26]
  EE heading     asin|e_y[3]|                      [deg]  dbg[27]
  joints         max_j |q_j - q_dj|                [deg]  log vs wbref

and reports, per leg, PEAK (max over the leg), RMS (over the whole leg) and
SETTLED (RMS over the last SETTLE s of the leg).

||e_R|| = |sin theta| exactly for the rotation angle theta between R_0 and
R_0c, and e_y[3] = -(b1ec . b2e) = sin of the EE heading error, so both invert
through asin rather than being read as radians.
"""

import sys
import numpy as np

D_MODE = 0
D_EY = slice(24, 28)
D_ER = slice(28, 31)
D_XCD = slice(45, 48)
D_XC = slice(48, 51)

LOG_T, LOG_V, LOG_Q0 = 0, slice(4, 7), 13
LOG_Q_OF_JOINT = (2, 0, 1, 3)      # broadcaster order [q2, q3, q1, q4]
WB_T, WB_XCD_DOT, WB_QD = 0, slice(4, 7), 19

SETTLE = 2.0

CHANNELS = [
    ("CoM pos",  "mm"),
    ("CoM vel",  "mm/s"),
    ("attitude", "deg"),
    ("EE pos",   "mm"),
    ("EE head",  "deg"),
    ("joints",   "deg"),
]


def load(path):
    z = np.load(path, allow_pickle=True)
    dbg, log = z["dbg"], z["log"]
    wbref = z["wbref"]
    t, d = dbg[:, 0], dbg[:, 1:]
    marks = []
    for s in z["leg_marks"]:
        a, b = str(s).split(" ", 1)
        marks.append((float(a), b))
    return dict(name=path.split("/")[-1].replace(".npz", ""),
                t=t, d=d, log=log, wbref=wbref, marks=marks)


def legs(run):
    m = run["marks"]
    out = []
    for i, (t0, name) in enumerate(m):
        t1 = m[i + 1][0] if i + 1 < len(m) else run["t"][-1]
        out.append((name, t0, t1))
    return out


def channels_over(run, t0, t1):
    """Every reference-tracked channel, sampled on the debug clock."""
    t, d, log, wbref = run["t"], run["d"], run["log"], run["wbref"]
    # DIRECT ONLY. The last leg runs past the abort into the SAFETY descent,
    # where the node publishes the SHORTER debug prefix and the recorder
    # zero-pads it: x_c and x_cd then read 0 (a 0 mm "error") while q_d goes
    # stale against an arm folding home (a 65 deg "error"). Both are recording
    # artifacts, not tracking.
    m = (t >= t0) & (t < t1) & (np.nan_to_num(d[:, D_MODE]) > 0.5)
    if m.sum() < 5:
        return None
    tt = t[m]

    com = np.linalg.norm(d[m][:, D_XC] - d[m][:, D_XCD], axis=1) * 1e3
    att = np.degrees(np.arcsin(np.clip(
        np.linalg.norm(d[m][:, D_ER], axis=1), 0.0, 1.0)))
    ee = np.linalg.norm(d[m][:, D_EY][:, :3], axis=1) * 1e3
    head = np.degrees(np.arcsin(np.clip(np.abs(d[m][:, 27]), 0.0, 1.0)))

    # CoM velocity and joints live on the log / wbref clocks -> interpolate
    lt, wt = log[:, LOG_T], wbref[:, WB_T]
    vel = np.full_like(com, np.nan)
    jnt = np.full_like(com, np.nan)
    if wbref.size and wbref.shape[1] > WB_QD + 3:
        v_meas = np.stack([np.interp(tt, lt, log[:, 4 + k]) for k in range(3)], 1)
        v_ref = np.stack([np.interp(tt, wt, wbref[:, 4 + k]) for k in range(3)], 1)
        vel = np.linalg.norm(v_meas - v_ref, axis=1) * 1e3
        errs = []
        for j in range(4):
            q = np.interp(tt, lt, log[:, LOG_Q0 + LOG_Q_OF_JOINT[j]])
            qd = np.interp(tt, wt, wbref[:, WB_QD + j])
            errs.append(np.degrees(np.abs(q - qd)))
        jnt = np.max(np.stack(errs, 1), axis=1)

    settled = tt >= (tt[-1] - SETTLE)
    out = {}
    for (name, _), v in zip(CHANNELS, [com, vel, att, ee, head, jnt]):
        if np.all(np.isnan(v)):
            out[name] = (np.nan, np.nan, np.nan)
            continue
        out[name] = (float(np.nanmax(v)),
                     float(np.sqrt(np.nanmean(v ** 2))),
                     float(np.sqrt(np.nanmean(v[settled] ** 2))))
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    csv = None
    if "--csv" in sys.argv:
        csv = sys.argv[sys.argv.index("--csv") + 1]
        args = [a for a in args if a != csv]
    paths = args or ["l1_6d_A.npz", "l1_6d_B.npz", "l1_4d_4d_wx0p25.npz",
                     "l1_4d_4d_best_A.npz", "l1_4d_4d_best_B.npz"]
    runs = [load(p) for p in paths]

    rows = []
    for r in runs:
        for name, t0, t1 in legs(r):
            ch = channels_over(r, t0, t1)
            if ch:
                rows.append((r["name"], name, ch))

    leg_order = []
    for _, leg, _ in rows:
        if leg not in leg_order:
            leg_order.append(leg)

    for ch_name, unit in CHANNELS:
        print(f"\n=== {ch_name}  [{unit}]   peak / rms / settled-rms "
              f"(settled = last {SETTLE:.0f} s of the leg)")
        hdr = f"  {'leg':<16}"
        for r in runs:
            hdr += f"{r['name'].replace('l1_4d_4d_', '4d_').replace('l1_', ''):>26}"
        print(hdr)
        for leg in leg_order:
            line = f"  {leg:<16}"
            for r in runs:
                cell = next((c for n, l, c in rows
                             if n == r["name"] and l == leg), None)
                if cell is None:
                    line += f"{'-':>26}"
                else:
                    p, rm, s = cell[ch_name]
                    line += f"{p:>8.1f} /{rm:>7.1f} /{s:>7.1f}"
            print(line)

    if csv:
        with open(csv, "w") as f:
            f.write("run,leg,channel,unit,peak,rms,settled_rms\n")
            for run_name, leg, ch in rows:
                for ch_name, unit in CHANNELS:
                    p, rm, s = ch[ch_name]
                    f.write(f"{run_name},{leg},{ch_name},{unit},"
                            f"{p:.4f},{rm:.4f},{s:.4f}\n")
        print(f"\nwrote {csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
