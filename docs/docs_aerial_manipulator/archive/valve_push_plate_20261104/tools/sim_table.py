#!/usr/bin/env python3
"""The push-plate campaign's table (2026-11-04): one row per run, from plate_score.py's readings and the fin-guard
scorer's summary, written to ../figures/sim_rows.json (the page's sim table is assembled from it by hand-written
text in sim_data.json) and printed.  Run: PYTHONNOUSERSITE=1 /usr/bin/python3 sim_table.py [tags...]"""
import glob
import io
import json
import os
import re
import subprocess
import sys
import contextlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plate_score as PS  # noqa: E402

RUNS = os.path.join(HERE, "..", "sim", "runs")


def row(f):
    r = subprocess.run([sys.executable, PS.GUARD, f], capture_output=True, text=True).stdout
    tag = os.path.basename(f)[:-4]
    out = {"tag": tag}
    m = re.search(r"^\S+\s+(completed|ABORT[^|]*?)\s*\|\s*(.*)$", r, re.M)
    out["outcome"] = (m.group(1).strip() if m else "?")
    out["reason"] = (m.group(2).strip() if m else "")
    m = re.search(r"valve turned\s+([-\d.]+) deg \| tilt max\s+([\d.]+) deg \| q3 min\s+([-\d.]+) deg \| DIRECT (\w+)", r)
    if m:
        out.update(valve=float(m.group(1)), tilt=float(m.group(2)), q3min=float(m.group(3)), direct=m.group(4))
    m = re.search(r"clear boxes\s+([-\d.]+)\.\.\s*([-\d.]+) \| balls\s+([-\d.]+)\.\.", r)
    if m:
        out.update(clear_box=float(m.group(1)), clear_ball=float(m.group(3)))
    m = re.search(r"OTHER (.*)$", r, re.M)
    out["other"] = m.group(1).strip() if m else ""
    z = np.load(f, allow_pickle=True)
    mt = dict(zip(z["marks_name"], z["marks_t"]))
    t0 = mt.get("push:start")
    # BEFORE the turn (the 11-03 failure happened there): the pusher's closest approach to the face it pushes
    # (the guard telemetry's gap, arrival [36] -> recorder column 38, from Ready's end) and the valve's motion
    b = z["box"]; qb = b[:, 4:8]
    yv = np.degrees(np.unwrap(np.arctan2(2 * (qb[:, 0] * qb[:, 3] + qb[:, 1] * qb[:, 2]), 1 - 2 * (qb[:, 2] ** 2 + qb[:, 3] ** 2))))
    r0, r1 = mt.get("ready:start"), mt.get("ready:end")
    tp = t0 if t0 is not None else b[-1, 0]
    if r0 is not None:
        out["pre_valve"] = float(np.interp(tp, b[:, 0], yv) - np.interp(r0, b[:, 0], yv))
    if r1 is not None and "arrival" in z.files:
        a = z["arrival"]; g = a[:, 38]; w = (a[:, 0] >= r1) & (a[:, 0] <= tp) & np.isfinite(g)
        if w.any():
            out["pre_gap_min"] = float(g[w].min() * 1e3)
    if "tilt" not in out:
        # aborted before the turn: the flight's own numbers from Ready on
        od = z["odom"]; q = od[:, 7:11]
        tl = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
        tr = mt.get("ready:start", mt.get("direct", 0.0))
        js = z["joints"]
        b = z["box"]; qb = b[:, 4:8]
        yv = np.degrees(np.unwrap(np.arctan2(2 * (qb[:, 0] * qb[:, 3] + qb[:, 1] * qb[:, 2]), 1 - 2 * (qb[:, 2] ** 2 + qb[:, 3] ** 2))))
        # the guard scorer's windows: tilt from DIRECT entry, q3 from Ready, both to the end of the run
        out.update(tilt=float(tl[od[:, 0] > mt.get("direct", 0.0)].max()), q3min=float(np.degrees(js[js[:, 0] > tr, 3]).min()),
                   valve=float(yv[-1] - np.interp(tr, b[:, 0], yv)), direct="kept")
        d = z["dbg"]; w = d[:, 0] > tr
        out.update(tz_p95=float(np.percentile(np.abs(d[w, 23 + PS.DBG]), 95)), tz_max=float(np.abs(d[w, 23 + PS.DBG]).max()))
    if t0 is None:
        return out
    t1 = mt.get("push:end", z["box"][-1, 0])
    for e in z["events"]:
        if re.search(r"push_pull: ABORTING", e):
            t1 = min(t1, float(re.match(r"\[\s*([\d.]+)s\]", e).group(1)))
            break
    js = z["joints"]; wj = (js[:, 0] >= t0) & (js[:, 0] <= t1)
    if wj.any():                      # the turn's highest q3 (the fin's q3 jam guard is 40 deg), up to an abort
        out["q3max"] = float(np.degrees(js[wj, 3]).max())
    lines = PS.nose_lines(f, t0, t1, z) + PS.yaw_lines(z, t0, t1)
    txt = "\n".join(lines)
    m = re.search(r"radial x\s+([-\d.]+)\.\.\s*([-\d.]+) mm.*height z\s+([-\d.]+)\.\.\s*([-\d.]+) mm", txt)
    if m:
        out.update(nose_x=(float(m.group(1)), float(m.group(2))), nose_z=(float(m.group(3)), float(m.group(4))))
    m = re.search(r"along the push \(\+y\) mean\s+([-\d.]+) p95\s+([-\d.]+)", txt)
    if m:
        out.update(push_mean=float(m.group(1)), push_p95=float(m.group(2)))
    m = re.search(r"tau_z \|\.\| p95 ([\d.]+) max ([\d.]+) N.m \| unallocated yaw max ([\d.]+) N.m \| a motor on a limit\s+([\d.]+) %.*sideways \|\.\| p95 ([\d.]+) max ([\d.]+) N", txt)
    if m:
        out.update(tz_p95=float(m.group(1)), tz_max=float(m.group(2)), unal=float(m.group(3)), mlim=float(m.group(4)),
                   fs_p95=float(m.group(5)), fs_max=float(m.group(6)))
    out["fin_contact"] = "on the FIN" in txt
    out["no_plate"] = "NO contact on the plate" in txt
    return out


def main():
    files = [os.path.join(RUNS, t + ".npz") for t in sys.argv[1:]] or sorted(glob.glob(os.path.join(RUNS, "*.npz")))
    rows = [row(f) for f in files]
    json.dump(rows, open(os.path.join(HERE, "..", "figures", "sim_rows.json"), "w"), indent=1)
    for r in rows:
        nx = r.get("nose_x"); nz = r.get("nose_z")
        print(f"{r['tag']:20s} {r['outcome'][:28]:28s} valve {r.get('valve', float('nan')):6.1f} tilt {r.get('tilt', float('nan')):4.1f} "
              f"q3min {r.get('q3min', float('nan')):5.1f} DIRECT {r.get('direct', '?'):4s} | nose x {nx} z {nz} | push {r.get('push_mean')}/{r.get('push_p95')} N | "
              f"tz p95/max {r.get('tz_p95')}/{r.get('tz_max')} unal {r.get('unal')} mlim {r.get('mlim')}% | Fside p95/max {r.get('fs_p95')}/{r.get('fs_max')} | "
              f"other {r.get('other')} | {r['reason'][:80]}")


if __name__ == "__main__":
    main()
