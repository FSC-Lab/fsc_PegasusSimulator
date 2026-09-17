#!/usr/bin/env python3
"""The propeller catalogue's zero-thrust advance ratio J0, fitted from measured C_T(J) data.

  ~/envs/isaac_env/bin/python3 scripts/ana_prop_j0.py --db <UIUC-propDB dir> [--pd 0.333] [--family apce]
  ~/envs/isaac_env/bin/python3 scripts/ana_prop_j0.py --csv fan_test.csv        (columns J, CT -- a fan or roof run)

WHY THIS EXISTS. `vtol.py`'s momentum thrust model is
    T = rho D^2 ( CT0 (nD)^2 - (CT0/J0) (nD) V_ax ),
which non-dimensionalises EXACTLY to C_T = C_T0 (1 - J/J0): a straight line from the static C_T0 to
zero at J0. So a measured C_T(J) curve gives J0 as its x-intercept directly, and the fit's residual
says how good the sim's LINEAR form is for a real propeller. The catalogue in `vtol.py` carries J0 per
prop, and for props not in it a fitted line J0 = 0.72*(pitch/D) + 0.24 -- which was itself a line
through three 12-inch entries, never checked against measurement. This tool checks it.

DATA: the UIUC Propeller Data Site (Selig et al.), https://m-selig.ae.illinois.edu/props/propDB.html
-- wind-tunnel C_T, C_P, eta vs J for hundreds of small propellers. Download UIUC-propDB.zip and
point --db at the unpacked directory. A static stand cannot produce J0 (no airflow); this can.
"""
import argparse
import csv
import glob
import os
import re
import sys

import numpy as np

FAMILY = {"apce": "APC Thin Electric", "apcsf": "APC Slow Flyer", "apcff": "APC Free Flight",
          "apcsp": "APC Sport", "apccf": "APC Carbon Fiber", "apce4": "APC Thin Electric",
          "ancf": "Aero-Naut CAM folding", "ance": "Aero-Naut CAM", "gwsdd": "GWS DD",
          "gwssf": "GWS SF", "mas": "Master Airscrew", "ma": "Master Airscrew",
          "gr": "Graupner", "ef": "E-flite", "kavfk": "Kavan", "kyosho": "Kyosho",
          "pl": "Plantraco", "vapor": "Vapor", "zin": "Zingali", "da4": "DA4", "grcp": "Graupner CAM"}
NAME = re.compile(r"^([a-z0-9]+?)_(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)_")


def parse_name(fn):
    """-> (family, D_in, pitch_in) or None. The database drops the decimal point in the pitch
    ('13x65' is 13x6.5) but NOT always ('10x12' really is a 1.2 pitch ratio), so the reading is
    chosen by which one lands in the physically possible band."""
    m = NAME.match(os.path.basename(fn))
    if not m:
        return None
    fam, d, p = m.group(1), float(m.group(2)), m.group(3)
    pv = float(p)
    if d > 0 and pv / d > 1.6 and "." not in p and len(p) > 1:
        alt = float(p[:-1] + "." + p[-1])
        if 0.15 <= alt / d <= 1.6:
            pv = alt
    return (fam, d, pv) if 0 < pv / d <= 1.6 else None


def fit_file(path):
    """Linear C_T vs J -> (J0, r2, n, J_range). None if the file is not a dynamic sweep."""
    try:
        rows = [l.split() for l in open(path) if l.strip()]
    except OSError:
        return None
    if not rows or rows[0][:2] != ["J", "CT"]:
        return None
    a = np.array([[float(x) for x in r] for r in rows[1:] if len(r) >= 2 and
                  all(re.match(r"^-?[\d.eE+-]+$", x) for x in r[:2])])
    if len(a) < 5:
        return None
    J, CT = a[:, 0], a[:, 1]
    m = np.isfinite(J) & np.isfinite(CT)
    J, CT = J[m], CT[m]
    if len(J) < 5 or np.ptp(J) < 0.05:
        return None
    b, c = np.polyfit(J, CT, 1)
    if b >= 0:
        return None
    pred = b * J + c
    ss = float(np.sum((CT - CT.mean()) ** 2))
    r2 = 1.0 - float(np.sum((CT - pred) ** 2)) / ss if ss > 0 else np.nan
    return (-c / b, r2, len(J), (float(J.min()), float(J.max())))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--db"); ap.add_argument("--csv")
    ap.add_argument("--pd", type=float, default=None, help="pitch/diameter to predict J0 for")
    ap.add_argument("--family", default=None, help="restrict the fit to one family prefix")
    ap.add_argument("--min_r2", type=float, default=0.95)
    ap.add_argument("--min_d", type=float, default=0.0, help="ignore propellers under this diameter [in]")
    a = ap.parse_args()
    if a.csv:
        r = fit_file(a.csv)
        print(f"{a.csv}: J0 {r[0]:.3f} (linear R2 {r[1]:.4f}, {r[2]} points over J {r[3][0]:.2f}-{r[3][1]:.2f})"
              if r else f"{a.csv}: not a J/CT sweep")
        return 0 if r else 1
    if not a.db:
        ap.error("--db or --csv")

    props = {}
    for f in glob.glob(os.path.join(a.db, "**", "data", "*.txt"), recursive=True):
        if "_static_" in f or "_geom" in f:
            continue
        nm = parse_name(f)
        if not nm:
            continue
        r = fit_file(f)
        if not r or r[1] < a.min_r2:
            continue
        props.setdefault(nm, []).append(r)

    rows = []
    for (fam, d, p), fits in sorted(props.items()):
        if a.family and fam != a.family:
            continue
        if d < a.min_d:
            continue
        j0 = float(np.median([x[0] for x in fits]))
        rows.append(dict(fam=fam, d=d, p=p, pd=p / d, j0=j0, n=len(fits),
                         sd=float(np.std([x[0] for x in fits])), r2=float(np.median([x[1] for x in fits]))))
    if not rows:
        print("no propellers fitted"); return 1
    rows.sort(key=lambda r: r["pd"])
    print(f"{'propeller':>22s} {'family':>22s} {'p/D':>6s} {'J0':>6s} {'sd':>5s} {'runs':>5s} {'linear R2':>9s}")
    for r in rows:
        print(f"{r['d']:.0f}x{r['p']:g}".rjust(22) + f" {FAMILY.get(r['fam'], r['fam']):>22s} "
              f"{r['pd']:6.3f} {r['j0']:6.3f} {r['sd']:5.3f} {r['n']:5d} {r['r2']:9.4f}")
    pd = np.array([r["pd"] for r in rows]); j0 = np.array([r["j0"] for r in rows])
    m, c = np.polyfit(pd, j0, 1)
    resid = j0 - (m * pd + c)
    print(f"\nFIT over {len(rows)} propellers: J0 = {m:.3f}*(p/D) + {c:.3f}   "
          f"(residual sd {resid.std():.3f}, worst {np.abs(resid).max():.3f})")
    print(f"vtol.py's catalogue line:      J0 = 0.720*(p/D) + 0.240")
    for name, (dd, pp) in {"12x6": (12, 6), "12x8": (12, 8), "12x10": (12, 10)}.items():
        k = [r for r in rows if abs(r["pd"] - pp / dd) < 1e-6]
        meas = f"{np.median([r['j0'] for r in k]):.3f}" if k else "  --  "
        print(f"   {name}: catalogue {0.72 * pp / dd + 0.24:.3f}   measured {meas}   this fit {m * pp / dd + c:.3f}")
    if a.pd is not None:
        near = [r for r in rows if abs(r["pd"] - a.pd) < 0.08]
        print(f"\nFOR p/D = {a.pd:.3f}: this fit {m * a.pd + c:.3f}, catalogue line {0.72 * a.pd + 0.24:.3f}")
        if near:
            v = [r["j0"] for r in near]
            print(f"  measured props within +-0.08 of that p/D ({len(near)}): J0 {np.min(v):.3f}-{np.max(v):.3f}, "
                  f"median {np.median(v):.3f}")
            for r in near:
                print(f"    {r['d']:.0f}x{r['p']:g} ({FAMILY.get(r['fam'], r['fam'])}), p/D {r['pd']:.3f}: J0 {r['j0']:.3f}")
        else:
            print("  no measured propeller within +-0.08 of that p/D -- the fit is an EXTRAPOLATION")
    return 0


if __name__ == "__main__":
    sys.exit(main())
