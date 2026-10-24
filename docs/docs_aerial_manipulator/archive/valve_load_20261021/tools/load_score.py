#!/usr/bin/env python3
"""(2026-10-21) Valve LOAD screen scorer: one row per run (valve_turning_20261015/runs/<tag>.npz).

    PYTHONNOUSERSITE=1 /usr/bin/python3 load_score.py tag [tag ...]

TURN (push:start -> push:end, or -> the abort for an aborted turn):
  beam  = how far the beam turned in the turn [deg]
  lag   = the beam behind its plan at the fin (r 0.160 m), rms / max [mm]
  lead  = the base's lead along its circle, max [mm] (valve_score's definition)
  stop  = % of the turn with q2 < -19.5 or q3 > 49.5 deg; q3min = the lowest q3 (< 0 = elbow branch)
  swing = std of (q2 - q3)/2 [deg], the arm's antiphase swing
  tilt  = max [deg]; Ft = mean tangential contact force on the fin (PhysX truth) [N]
RELEASE (from push:end, or the abort, to the end of the record):
  back  = how far the beam turned BACK from its in-turn maximum [deg], peak back rate [deg/s]
  q3r / tiltr = the lowest q3 and the highest tilt in the 8 s after it
"""
import sys

import numpy as np

R = "/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015/runs/"


def yaw_of(q):
    return np.arctan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))


def tilt_of(q):
    return np.degrees(np.arccos(np.clip(1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1.0, 1.0)))


def at(tq, ts, X):
    return np.column_stack([np.interp(tq, ts, X[:, k]) for k in range(X.shape[1])])


def row(tag):
    d = np.load(R + tag + ".npz", allow_pickle=True)
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    aborted = bool(d["aborted"])
    box, ref, odom, J, ct, claw = d["box"], d["ref"], d["odom"], d["joints"], d["contact"], d["claw"]
    axis = box[0, 1:4]
    beam = np.degrees(np.unwrap(yaw_of(box[:, 4:8])))
    beam -= beam[0]
    t0 = marks["push:start"]
    if "push:end" in marks:
        t1 = marks["push:end"]
    else:                                   # aborted in the turn: the driver log's first ABORTING line
        import re
        t1 = None
        with open(R + tag + ".log") as f:
            for line in f:
                mm = re.match(r"\[\s*([0-9.]+)s\].*ABORTING", line)
                if mm and float(mm.group(1)) > t0:
                    t1 = float(mm.group(1))
                    break
        if t1 is None:
            t1 = odom[odom[:, 11] > 0.5, 0].max()
    out = {"tag": tag, "outcome": ("ABORT " + str(d["reason"]).split("(")[-1].split(")")[0][:26]) if aborted else "complete",
           "t_turn": t1 - t0}
    m = (box[:, 0] >= t0) & (box[:, 0] <= t1)
    out["beam"] = beam[m].max() - beam[m][0] if m.any() else np.nan
    rb = np.degrees(np.unwrap(np.arctan2(ref[:, 2] - axis[1], ref[:, 1] - axis[0])))
    rb -= np.interp(t0, ref[:, 0], rb)
    lag = (np.interp(box[m, 0], ref[:, 0], rb) - beam[m]) * np.pi / 180 * 0.160 * 1e3
    out["lag_rms"], out["lag_max"] = np.sqrt(np.mean(lag ** 2)), np.abs(lag).max()
    # base lead (valve_score's)
    mo = (odom[:, 0] >= t0) & (odom[:, 0] <= t1)
    o = odom[mo, 1:4]
    xc = at(odom[mo, 0], ref[:, 0], ref[:, 4:7])
    rad = o[:, :2] - axis[:2]
    rad /= np.maximum(np.linalg.norm(rad, axis=1, keepdims=True), 1e-9)
    tang = np.column_stack([-rad[:, 1], rad[:, 0]])
    lead = np.einsum("ij,ij->i", o[:, :2] - xc[:, :2], tang)
    out["lead_max"] = np.abs(lead - lead[0]).max() * 1e3
    out["tilt_max"] = tilt_of(odom[mo, 7:11]).max()
    mj = (J[:, 0] >= t0) & (J[:, 0] <= t1)
    q = np.degrees(J[mj, 1:5])
    dtj = np.gradient(J[mj, 0])
    out["stop"] = 100 * dtj[(q[:, 1] < -19.5) | (q[:, 2] > 49.5)].sum() / max(dtj.sum(), 1e-9)
    out["q3min"] = q[:, 2].min()
    out["swing"] = np.std((q[:, 1] - q[:, 2]) / 2)
    mc = (ct[:, 0] >= t0) & (ct[:, 0] <= t1)
    F = ct[mc, 1:4] + ct[mc, 4:7]
    cl = at(ct[mc, 0], claw[:, 0], claw[:, 1:4])
    r2 = cl[:, :2] - axis[:2]
    r2 /= np.maximum(np.linalg.norm(r2, axis=1, keepdims=True), 1e-9)
    out["Ft"] = np.abs(np.einsum("ij,ij->i", F[:, :2], np.column_stack([-r2[:, 1], r2[:, 0]]))).mean()
    # release
    mr = box[:, 0] >= t1
    if mr.any():
        bmax = beam[m].max() if m.any() else beam[mr][0]
        b_after = beam[mr]
        out["back"] = bmax - b_after.min()
        rate = np.gradient(b_after, box[mr, 0])
        out["back_rate"] = max(0.0, -rate.min())
    w8 = (J[:, 0] >= t1) & (J[:, 0] <= t1 + 8)
    out["q3r"] = np.degrees(J[w8, 3]).min() if w8.any() else np.nan
    w8o = (odom[:, 0] >= t1) & (odom[:, 0] <= t1 + 8)
    out["tiltr"] = tilt_of(odom[w8o, 7:11]).max() if w8o.any() else np.nan
    return out


def main():
    print(f"{'tag':12s} {'outcome':28s} {'turn s':>6s} {'beam':>6s} {'lag rms/max':>12s} {'lead':>5s} "
          f"{'stop%':>5s} {'q3min':>6s} {'swing':>5s} {'tilt':>5s} {'Ft':>5s} | {'back deg':>8s} {'back d/s':>8s} "
          f"{'q3r':>6s} {'tiltr':>5s}")
    for tag in sys.argv[1:]:
        try:
            r = row(tag)
        except Exception as exc:              # a half-written or old-format run: say so, keep going
            print(f"{tag:12s} could not score: {exc!r}")
            continue
        print(f"{r['tag']:12s} {r['outcome']:28s} {r['t_turn']:6.1f} {r['beam']:6.0f} "
              f"{r['lag_rms']:5.1f}/{r['lag_max']:5.1f} {r['lead_max']:5.0f} {r['stop']:5.1f} {r['q3min']:6.1f} "
              f"{r['swing']:5.1f} {r['tilt_max']:5.1f} {r['Ft']:5.2f} | {r.get('back', np.nan):8.0f} "
              f"{r.get('back_rate', np.nan):8.0f} {r['q3r']:6.1f} {r['tiltr']:5.1f}")


if __name__ == "__main__":
    main()
