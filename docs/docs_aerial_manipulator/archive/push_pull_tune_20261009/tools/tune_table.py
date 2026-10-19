#!/usr/bin/env python3
"""The tune's scoring table, one line per run (this campaign's runs/):

    PYTHONNOUSERSITE=1 /usr/bin/python3 tune_table.py [tag ...] [--csv out.csv]

Columns: outcome; the box pushed (mm); the impedance residual (pl_metrics, the norm's rms over
SLIDE / CONTACT / RELEASE and the RELEASE peak, N); the true contact force in the slide (rms N);
rho_UAV max over SLIDE and over the EXIT; tilt max in the push; q2 / q3 extremes in the push
(the stops: q2 45, q3 0 and 50); q3's peak at the release; the box's movement in the exit (mm);
the EE task error |e_y| rms in the slide (mm). The baseline rows are the A/B pushes.
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_cad_box_20261007", "tools"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_20261003", "tools"))
import summarize as S          # noqa: E402
import pl_metrics as PM        # noqa: E402

S.RUNS = RUNS


def one(tag):
    try:
        r = S.one(tag)
    except Exception:                      # an aborted run with no usable push window
        z = np.load(os.path.join(RUNS, tag + ".npz"), allow_pickle=True)
        r = dict(aborted=bool(z["aborted"]), reason=str(z["reason"]))
    row = dict(tag=tag, ok=not r["aborted"], box=r.get("box_mm", np.nan), tilt=r.get("tilt_max", np.nan),
               q2min=r.get("q2", (np.nan, np.nan))[0], q2max=r.get("q2", (np.nan, np.nan))[1],
               q3min=r.get("q3", (np.nan, np.nan))[0], q3max=r.get("q3", (np.nan, np.nan))[1],
               relq3=r.get("rel_q3", np.nan), exitbox=r.get("exit_box_mm", np.nan), reason=r["reason"][:60])
    try:
        a = PM.analyse(os.path.join(RUNS, tag + ".npz"), PM.yaml_gains(os.path.join(RUNS, tag + ".yaml")), 5.0)
        for w, k in (("SLIDE", "slide"), ("CONTACT", "contact"), ("RELEASE", "release"), ("HOLD", "hold")):
            if w in a["imp"]:
                row[k] = a["imp"][w]["e_xyz"][0]
                row[k + "_pk"] = a["imp"][w]["e_xyz"][1]
        if "SLIDE" in a["imp"]:
            row["F"] = a["imp"]["SLIDE"]["F_xyz"][0]
            row["ey"] = a["imp"]["SLIDE"]["ey_mm"][0]
        row["rho_slide"] = a["eps"].get("SLIDE", (np.nan,))[0] / PM.L
        row["rho_exit"] = a["eps"].get("EXIT", (np.nan,))[0] / PM.L
        row["rho_contact"] = a["eps"].get("CONTACT", (np.nan,))[0] / PM.L
    except Exception as exc:               # noqa: BLE001
        row["err"] = str(exc)[:50]
    args = os.path.join(RUNS, tag + ".args")
    row["change"] = " ".join(open(args).read().split()) if os.path.isfile(args) else ""
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="*")
    ap.add_argument("--csv")
    a = ap.parse_args()
    tags = a.tags or sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(RUNS, "*.npz")))
    rows = [one(t) for t in tags]
    f = lambda r, k, n=2: (f"{r[k]:.{n}f}" if k in r and np.isfinite(r[k]) else "-")
    print(f"{'run':10s} {'ok':2s} {'box':>4s} | {'slide':>5s} {'cont':>5s} {'hold':>5s} {'rel':>5s} {'relpk':>5s} | {'F':>4s} {'ey':>4s} | "
          f"{'rhoSl':>5s} {'rhoCt':>5s} {'rhoEx':>5s} | {'tilt':>4s} {'q2':>11s} {'q3':>11s} {'relq3':>5s} {'exbox':>5s} | change")
    for r in rows:
        q2 = f"{r['q2min']:4.1f}..{r['q2max']:4.1f}" if np.isfinite(r["q2min"]) else "-"
        q3 = f"{r['q3min']:4.1f}..{r['q3max']:4.1f}" if np.isfinite(r["q3min"]) else "-"
        print(f"{r['tag']:10s} {'y' if r['ok'] else 'N':2s} {f(r, 'box', 0):>4s} | {f(r, 'slide'):>5s} {f(r, 'contact'):>5s} "
              f"{f(r, 'hold'):>5s} {f(r, 'release'):>5s} {f(r, 'release_pk', 1):>5s} | {f(r, 'F'):>4s} {f(r, 'ey', 1):>4s} | "
              f"{f(r, 'rho_slide', 3):>5s} {f(r, 'rho_contact', 3):>5s} {f(r, 'rho_exit', 3):>5s} | {f(r, 'tilt', 1):>4s} "
              f"{q2:>11s} {q3:>11s} {f(r, 'relq3', 0):>5s} {f(r, 'exitbox', 0):>5s} | {r['change'] if r['ok'] else r['reason']}")
    if a.csv:
        keys = ["tag", "ok", "box", "slide", "contact", "hold", "release", "release_pk", "F", "ey", "rho_slide", "rho_contact",
                "rho_exit", "tilt", "q2min", "q2max", "q3min", "q3max", "relq3", "exitbox", "change", "reason"]
        with open(a.csv, "w") as fh:
            fh.write(",".join(keys) + "\n")
            for r in rows:
                fh.write(",".join(str(r.get(k, "")) for k in keys) + "\n")
        print("wrote", a.csv)


if __name__ == "__main__":
    main()
