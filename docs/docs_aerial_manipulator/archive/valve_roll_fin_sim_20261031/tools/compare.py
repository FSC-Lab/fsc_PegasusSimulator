#!/usr/bin/env python3
"""(2026-10-31) The roll fin's campaign, grouped: v2 fixed fin (the 10-31 control) / v3 LOCKED / v3 FREE,
nominal and hardware-like drift (PEGASUS_PLANT_FORCE_NOISE 0.32 N @ 0.12 Hz, seeds 2-4). Per run, then
group means over the runs that reached the turn. Metrics from grip_score.py (the couple at G, the sway,
e_R, q1/q4, the roll fin) and the 10-28 sim_contact.py (valve - plan, fore-aft / lateral sd, growth).
    PYTHONNOUSERSITE=1 /usr/bin/python3 compare.py            (writes ../compare.txt too)
"""
import glob
import importlib.util
import io
import os
import sys
from contextlib import redirect_stdout

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import grip_score as gs  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "sim_contact", os.path.join(HERE, "..", "..", "wb_valve_turning_flight_20261028", "tools", "sim_contact.py"))
sc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sc)


def runs():
    out = []
    for p in sorted(glob.glob(gs.RUNS + "*.npz")):
        t = os.path.basename(p)[:-4]
        if t == "rf_smoke":                           # flown before the contact-truth fix: no contact record
            continue
        out.append((gs.RUNS, t, "drift" if "_s" in t else "nominal"))
    for t in gs.V2_TAGS:
        out.append((gs.V2, t, "drift" if "_hw_" in t else "nominal"))
    return out


def one(path, tag):
    d = np.load(path + tag + ".npz", allow_pickle=True)
    gs.cs.R = path
    row = gs.cs.row(tag)
    m = gs.metrics(path, tag)
    try:
        c = sc.run(path, tag)
    except Exception:
        c = {}
    m.update(outcome=row.split()[1] if len(row.split()) > 1 else "?", row=row, aborted=bool(d["aborted"]),
             tilt=float(row.split("tilt")[1].split()[0]) if "tilt" in row else np.nan,
             beam=float(row.split("beam")[1].split()[0]) if "beam" in row else np.nan, **{f"c_{k}": v for k, v in c.items() if k != "q"})
    return m


def main():
    buf = io.StringIO()
    rows = []
    with redirect_stdout(buf):
        for path, tag, cond in runs():
            try:
                m = one(path, tag)
            except Exception as exc:
                print(f"{tag:12s} ERROR {exc!r}")
                continue
            rows.append((tag, cond, m))
            print(m["row"])
            if "M_rms" in m:
                print(gs.fmt(m))
                print(f"   valve - plan rms {m.get('c_vrms', np.nan):4.1f} deg ({m.get('c_vmin', np.nan):+5.1f}..{m.get('c_vmax', np.nan):+5.1f}), "
                      f"end {m.get('c_final', np.nan):5.1f} deg | fore-aft sd {m.get('c_ef', np.nan):5.1f} mm, lateral sd "
                      f"{m.get('c_el', np.nan):5.1f} mm (p-p {m.get('c_elpp', np.nan):4.0f}) at {m.get('c_fl', np.nan):.2f} Hz, "
                      f"growth {m.get('c_gr', np.nan):+.2f} 1/s | turned by F_tan {m.get('F_tan', np.nan):+.2f} N, "
                      f"M_z {m.get('M_z', np.nan) * 1e3:+4.0f} mN.m; CoM lon {m.get('lon_mean', np.nan):+4.0f} mm, e_R pitch "
                      f"{m.get('eRp_mean', np.nan):+4.0f} mrad; hover side rms {m.get('side_hover', np.nan):4.1f} mm")
        print("\nGROUP MEANS (runs that reached the turn; the couple and sway over the turn or up to the abort)")
        hdr = (f"{'group':20s} {'cond':8s} {'done':>5s} | {'couple X/N/C mN.m':>19s} | {'side':>5s} {'rate99':>6s} | "
               f"{'eR roll':>7s} {'pitch':>6s} | {'q1':>4s} {'q4':>4s} {'corr':>5s} | {'q3min':>5s} {'tilt':>5s} | "
               f"{'v-p rms':>7s} {'end':>5s} | {'roll fin':>12s} | {'F_tan':>5s} {'M_z':>5s} {'lon':>5s} {'eRp':>5s} | {'hover side':>10s} {'exit F max':>10s}")
        print(hdr)
        for grp in ("v2 fixed", "v3 LOCKED", "v3 free 2mNm"):
            for cond in ("nominal", "drift"):
                G = [m for t, c, m in rows if c == cond and m.get("mode") == grp and "M_rms" in m]
                if not G:
                    continue
                done = sum(not m["aborted"] for m in G)
                mean = lambda k, i=None: np.nanmean([(m[k][i] if i is not None else m[k]) for m in G if k in m])
                Mx = mean("M_rms", 0) * 1e3; Mn = mean("M_rms", 1) * 1e3; Mc = mean("M_rms", 2) * 1e3
                rf = ""
                if all("roll" in m for m in G):
                    rf = f"{np.mean([m['roll'][0] for m in G]):+4.1f}..{np.mean([m['roll'][1] for m in G]):+4.1f}"
                print(f"{grp:20s} {cond:8s} {done:2d}/{len(G):<2d} | {Mx:5.0f}/{Mn:5.0f}/{Mc:5.0f}       | "
                      f"{mean('side_rms'):5.1f} {mean('sway_rate_p99'):6.0f} | {mean('eR_roll'):7.1f} {mean('eR_pitch'):6.1f} | "
                      f"{mean('q1_std'):4.1f} {mean('q4_std'):4.1f} {mean('q14_corr'):+5.2f} | "
                      f"{np.nanmin([m['q3_min'] for m in G]):5.1f} {np.nanmax([m['tilt'] for m in G]):5.1f} | "
                      f"{mean('c_vrms'):7.1f} {mean('c_final'):5.1f} | {rf:>12s} | {mean('F_tan'):+5.2f} {mean('M_z') * 1e3:+5.0f} "
                      f"{mean('lon_mean'):+5.0f} {mean('eRp_mean'):+5.0f} | {mean('side_hover'):10.1f} {mean('F_exit_max'):10.2f}")
    txt = buf.getvalue()
    print(txt)
    open(os.path.join(HERE, "..", "compare.txt"), "w").write(txt)


if __name__ == "__main__":
    main()
