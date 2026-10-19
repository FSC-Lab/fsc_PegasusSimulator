"""(2026-10-18, Opus) Guard calibration for the valve: per run, q3's minimum in the turn (with the
time it spent below 10 / 6 / 3 deg), the outcome, and -- for aborted runs -- what happened after
the abort: the abort's trigger time, tilt / q4 / contact force in the 5 s after it, and whether the
node stayed in DIRECT. Run from the repo root:
    PYTHONNOUSERSITE=1 /usr/bin/python3 docs/.../tools/guard_signals.py [tags...]   (default: all)"""
import glob, os, sys
import numpy as np
R = "docs/docs_aerial_manipulator/archive/valve_turning_20261015/runs/"


def tilt(q):
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


def main():
    tags = sys.argv[1:] or sorted(os.path.basename(p)[:-4] for p in glob.glob(R + "*.npz")
                                  if "profile" not in p)
    print(f"{'run':14s} {'outcome':48s} {'q3min':>6s} {'<10s':>5s} {'<6s':>5s} {'<3s':>5s} | after the end of DIRECT / abort")
    for tag in tags:
        try:
            d = np.load(R + tag + ".npz", allow_pickle=True)
        except Exception:
            continue
        if "joints" not in d.files or len(d["joints"]) == 0:
            continue
        marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
        if "push:start" not in marks:
            continue
        J, od = d["joints"], d["odom"]
        t0 = marks["push:start"]
        direct = od[:, 11] > 0.5
        tdir_end = od[direct, 0].max() if direct.any() else t0
        t1 = marks.get("push:end", tdir_end)
        m = (J[:, 0] >= t0) & (J[:, 0] <= t1)
        if m.sum() < 10:
            continue
        q3 = np.degrees(J[m, 3]); dt = np.gradient(J[m, 0])
        below = [float(dt[q3 < th].sum()) for th in (10, 6, 3)]
        out = "complete" if not bool(d["aborted"]) else str(d["reason"])[:47]
        tail = ""
        ev = [e for e in d["events"].tolist() if "ABORTING" in e or "SAFETY GUARD" in e]
        if bool(d["aborted"]):
            # the abort / DIRECT-loss instant
            ta = None
            for e in d["events"].tolist():
                if "ABORTING" in e:
                    try:
                        ta = float(e.split("s]")[0].strip("[ ")); break
                    except Exception:
                        pass
            if ta is None:
                ta = tdir_end
            w = (od[:, 0] >= ta) & (od[:, 0] <= ta + 6.0)
            mj = (J[:, 0] >= ta) & (J[:, 0] <= ta + 6.0)
            c = d["contact"]; mc = (c[:, 0] >= ta) & (c[:, 0] <= ta + 6.0)
            fmax = np.linalg.norm(c[mc, 1:4] + c[mc, 4:7], axis=1).max() if mc.any() else float("nan")
            still_direct = od[w, 11].min() > 0.5 if w.any() else False
            tail = (f"t_abort {ta - t0:5.1f} s into the turn: tilt max {tilt(od[w, 7:11]).max():5.1f} deg, "
                    f"|q4| max {np.degrees(np.abs(J[mj, 4])).max():5.1f}, q3 min {np.degrees(J[mj, 3]).min():5.1f}, "
                    f"contact max {fmax:5.1f}, {'DIRECT kept' if still_direct else 'left DIRECT'}")
        print(f"{tag:14s} {out:48s} {q3.min():6.1f} {below[0]:5.2f} {below[1]:5.2f} {below[2]:5.2f} | {tail}")


if __name__ == "__main__":
    main()
