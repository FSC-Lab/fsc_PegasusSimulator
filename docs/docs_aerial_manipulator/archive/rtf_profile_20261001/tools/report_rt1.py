#!/usr/bin/env python3
"""report_rt1.py -- the RTF-1 Isaac flights for the comparison report
(modular_adaptive_20260930/report.html): chart series + path geometry.

    /usr/bin/python3 report_rt1.py --plot WB ../data/wb_rt1a.npz --plot MOD ../data/modular_rt1a.npz \
        --geom ../data/wb_rt1a.npz ../data/wb_rt1b.npz ../data/modular_rt1a.npz ... \
        --old ../../modular_adaptive_20260930/analysis/report_payload.json --out ../analysis/report_payload_rt1.json

Payload keys: isaac (RTF-1 flights, plan-time EE error + top-view path; the old
report's series format), isaac048 (the RTF 0.48 flights, copied from --old),
bench (copied from --old), geom {file: radial/tangential/vertical EE offset [mm]}.
Path geometry over the EXECUTING window: reference-circle centre = least-squares
circle fit to the reference xy; radial = mean measured radius - mean reference radius; tangential
= mean along-track component of (measured - reference); vertical = mean dz.
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "modular_adaptive_20260930", "tools"))
import report_data as RD  # noqa: E402


def geometry(path):
    d = np.load(path, allow_pickle=True)
    mk = RD.marks(d)
    e = d["ee"]; e = e[(e[:, 0] >= mk["run_start"]) & (e[:, 0] <= mk["run_end"])]
    m, r = e[:, 1:4], e[:, 4:7]
    # circle centre by an algebraic (Kasa) least-squares fit to the reference:
    # the window carries the speed ramps, so a plain mean of the points is biased
    A = np.column_stack([2 * r[:, 0], 2 * r[:, 1], np.ones(len(r))])
    sol, *_ = np.linalg.lstsq(A, r[:, 0] ** 2 + r[:, 1] ** 2, rcond=None)
    c = sol[:2]
    rr = r[:, :2] - c; rad = rr / np.linalg.norm(rr, axis=1)[:, None]
    # direction of travel from the reference itself (sign-correct for either sense)
    tan = np.gradient(r[:, :2], axis=0); tan /= np.linalg.norm(tan, axis=1)[:, None] + 1e-12
    dm = m - r
    return dict(ref_r=float(np.linalg.norm(rr, axis=1).mean()),
                meas_r=float(np.linalg.norm(m[:, :2] - c, axis=1).mean()),
                radial_mm=float(1e3 * np.sum(dm[:, :2] * rad, 1).mean()),
                tang_mm=float(1e3 * np.sum(dm[:, :2] * tan, 1).mean()),
                z_mm=float(1e3 * dm[:, 2].mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", nargs=2, action="append", metavar=("LABEL", "NPZ"), required=True)
    ap.add_argument("--geom", nargs="+", default=[])
    ap.add_argument("--old", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    old = json.load(open(a.old))
    runs = []
    for lbl, p in a.plot:
        o = RD.one(p, lbl, n=360)
        runs.append({"label": lbl, "t": o["ee"]["t"], "err": o["ee"]["err"],
                     "mx": o["xy"]["mx"], "my": o["xy"]["my"], "rx": o["xy"]["rx"], "ry": o["xy"]["ry"]})
    geom = {os.path.basename(p).replace(".npz", ""): geometry(p) for p in a.geom}
    for k, g in geom.items():
        print(f"{k:16s} r {g['meas_r']:.4f} of {g['ref_r']:.4f} m | radial {g['radial_mm']:+6.1f} tang {g['tang_mm']:+6.1f} z {g['z_mm']:+6.1f} mm")
    json.dump({"isaac": runs, "isaac048": old["isaac"], "bench": old["bench"], "geom": geom}, open(a.out, "w"))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
