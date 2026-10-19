#!/usr/bin/env python3
"""The 2026-10-16 box-pushing campaign's scoring table, one line per run (../runs):

    PYTHONNOUSERSITE=1 /usr/bin/python3 table.py [tag ...] [--csv out.csv] [--group]

Per run: the 10-09 tune scorer (push_pull_tune_20261009/tools/tune_table.py: outcome, the
impedance residual over SLIDE / CONTACT / RELEASE + the RELEASE peak, the true contact force,
rho_UAV, tilt, q2 / q3 extremes, q3 at the release, the box's movement in the exit), plus
  mass / dist     the run's MASS and DIST (fly.sh's .args)
  trav            the box's travel along the push in the push window / the commanded distance [%]
  lat             the box's largest sideways (x) excursion from its start in the push [mm]
  dyaw            the box's largest yaw change in the push [deg]
  tq2 / tq3       the time q2 / q3 spent within 2 deg of its stop (45 / 50 deg) in the push [s]
--group prints the mean (and the spread) of the main columns per identical setting
(the .args minus the run's tag), completed runs only, with the completion count.
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_tune_20261009", "tools"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_cad_box_20261007", "tools"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_20261003", "tools"))
import tune_table as TT        # noqa: E402
import summarize as S          # noqa: E402
from pl_metrics import mark    # noqa: E402

TT.RUNS = S.RUNS = RUNS


def yaw_deg(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def args_of(tag):
    p = os.path.join(RUNS, tag + ".args")
    kv = {}
    if os.path.isfile(p):
        for ln in open(p).read().split():
            if "=" in ln:
                k, v = ln.split("=", 1)
                kv[k] = v
    return kv


def one(tag):
    row = TT.one(tag)
    kv = args_of(tag)
    row["mass"] = float(kv.get("MASS", "0.4"))
    row["dist"] = float(kv.get("DIST", kv.get("push_pull_push_distance", "0.5")))
    # the setting = every arg except the push distance / time fly.sh derives
    row["setting"] = " ".join(f"{k}={v}" for k, v in sorted(kv.items())
                              if k not in ("push_pull_push_distance", "push_pull_push_time_s", "BOX_Y0"))
    z = np.load(os.path.join(RUNS, tag + ".npz"), allow_pickle=True)
    ps, pe = mark(z, "push:start"), mark(z, "push:end")
    b, j = z["box"], z["joints"]
    if ps is not None:
        t1 = pe if pe is not None else b[-1, 0]
        bb = b[(b[:, 0] >= ps) & (b[:, 0] <= t1)]
        if len(bb) > 1:
            row["trav"] = 100.0 * (bb[0, 2] - bb[-1, 2]) / row["dist"]
            row["lat"] = 1e3 * np.abs(bb[:, 1] - bb[0, 1]).max()
            row["dyaw"] = np.abs(yaw_deg(bb[:, 4:8]) - yaw_deg(bb[:1, 4:8])[0]).max()
        m = (j[:, 0] > ps) & (j[:, 0] < t1)
        if m.sum() > 1:
            q2 = np.degrees(j[m, 2])
            dt = np.diff(j[m, 0], append=j[m, 0][-1])
            row["tq2"] = float(dt[q2 > 43.0].sum())
            row["tq3"] = float(dt[np.degrees(j[m, 3]) > 48.0].sum())
    return row


COLS = ("slide", "contact", "release", "release_pk", "F", "ey", "rho_slide", "rho_exit", "tilt",
        "q2max", "q3min", "q3max", "relq3", "exitbox", "trav", "lat", "dyaw", "tq2", "tq3")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="*")
    ap.add_argument("--csv")
    ap.add_argument("--group", action="store_true")
    a = ap.parse_args()
    tags = a.tags or sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(RUNS, "*.npz")))
    rows = [one(t) for t in tags]
    f = lambda r, k, n=2: (f"{r[k]:.{n}f}" if k in r and np.isfinite(r[k]) else "-")
    print(f"{'run':12s} {'ok':2s} {'mass':>4s} {'dist':>4s} {'trav%':>5s} | {'slide':>5s} {'cont':>5s} {'rel':>5s} {'relpk':>5s} | "
          f"{'F':>4s} {'ey':>4s} | {'rhoSl':>5s} {'rhoEx':>5s} | {'tilt':>4s} {'q2max':>5s} {'q3':>11s} {'relq3':>5s} "
          f"{'tq2':>4s} {'tq3':>4s} | {'lat':>4s} {'dyaw':>4s} {'exbox':>5s} | change / reason")
    for r in rows:
        q3 = f"{r['q3min']:4.1f}..{r['q3max']:4.1f}" if np.isfinite(r.get("q3min", np.nan)) else "-"
        print(f"{r['tag']:12s} {'y' if r['ok'] else 'N':2s} {r['mass']*1e3:4.0f} {r['dist']:4.2f} {f(r, 'trav', 0):>5s} | "
              f"{f(r, 'slide'):>5s} {f(r, 'contact'):>5s} {f(r, 'release'):>5s} {f(r, 'release_pk', 1):>5s} | "
              f"{f(r, 'F'):>4s} {f(r, 'ey', 1):>4s} | {f(r, 'rho_slide', 3):>5s} {f(r, 'rho_exit', 3):>5s} | "
              f"{f(r, 'tilt', 1):>4s} {f(r, 'q2max', 1):>5s} {q3:>11s} {f(r, 'relq3', 0):>5s} {f(r, 'tq2', 1):>4s} {f(r, 'tq3', 1):>4s} | "
              f"{f(r, 'lat', 0):>4s} {f(r, 'dyaw', 1):>4s} {f(r, 'exitbox', 0):>5s} | "
              f"{r['setting'] if r['ok'] else r['reason']}")
    if a.group:
        print("\nper setting (completed runs; mean [min..max]):")
        groups = {}
        for r in rows:
            groups.setdefault(r["setting"], []).append(r)
        for s, rs in groups.items():
            ok = [r for r in rs if r["ok"]]
            parts = [f"{len(ok)}/{len(rs)}"]
            for k in ("slide", "release", "release_pk", "rho_slide", "tilt", "q2max", "q3max", "tq3", "relq3", "lat", "dyaw", "exitbox"):
                v = np.array([r[k] for r in ok if k in r and np.isfinite(r[k])])
                if len(v):
                    parts.append(f"{k} {v.mean():.2f} [{v.min():.2f}..{v.max():.2f}]")
            print(f"  {s}\n      " + "; ".join(parts))
    if a.csv:
        keys = ["tag", "ok", "mass", "dist", *COLS, "setting", "reason"]
        with open(a.csv, "w") as fh:
            fh.write(",".join(keys) + "\n")
            for r in rows:
                fh.write(",".join(str(r.get(k, "")).replace(",", ";") for k in keys) + "\n")


if __name__ == "__main__":
    main()
