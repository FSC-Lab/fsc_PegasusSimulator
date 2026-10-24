#!/usr/bin/env python3
"""(2026-10-21) Series for the valve-load report page, written as JSON next to this folder:
tilt over the turn (baseline / damper / weight 0.20 + ratchet) and the beam angle
(baseline / weight 0.20 without a ratchet), time from the turn's start, 10 Hz.
    PYTHONNOUSERSITE=1 /usr/bin/python3 report_data.py"""
import json
import os
import re

import numpy as np

R = "/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015/runs/"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "report_data.json")


def yaw_of(q):
    return np.arctan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))


def tilt_of(q):
    return np.degrees(np.arccos(np.clip(1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1.0, 1.0)))


def load(tag):
    d = np.load(R + tag + ".npz", allow_pickle=True)
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    t0 = marks["push:start"]
    abort = None
    for line in open(R + tag + ".log"):
        m = re.match(r"\[\s*([0-9.]+)s\].*ABORTING", line)
        if m and float(m.group(1)) > t0:
            abort = float(m.group(1)) - t0
            break
    end = marks.get("push:end", None)
    return d, t0, abort, (end - t0) if end else None


def tilt_series(tag, t_end):
    d, t0, abort, end = load(tag)
    od = d["odom"]
    tt = np.arange(0.0, t_end + 1e-9, 0.1)
    t_last = (abort if abort is not None else end) or t_end
    tt = tt[tt <= t_last + 0.5]                     # stop just after the abort / the turn's end
    tl = tilt_of(od[:, 7:11])
    # 0.1 s bins: the max inside each, so a peak is not interpolated away
    y = [float(tl[(od[:, 0] >= t0 + t - 0.05) & (od[:, 0] < t0 + t + 0.05)].max()) for t in tt]
    return {"t": [round(float(t), 2) for t in tt], "y": [round(v, 2) for v in y],
            "abort": None if abort is None else round(abort, 2), "end": None if end is None else round(end, 2)}


def beam_series(tag, t_end):
    d, t0, abort, end = load(tag)
    box = d["box"]
    b = np.degrees(np.unwrap(yaw_of(box[:, 4:8])))
    b -= b[0]
    tt = np.arange(0.0, t_end + 1e-9, 0.1)
    y = np.interp(t0 + tt, box[:, 0], b)
    return {"t": [round(float(t), 2) for t in tt], "y": [round(float(v), 1) for v in y],
            "abort": None if abort is None else round(abort, 2), "end": None if end is None else round(end, 2)}


out = {
    "tilt": {k: tilt_series(k, 40.0) for k in ("ld_base1", "ld_d050", "ld_w020r")},
    "beam": {k: beam_series(k, 45.0) for k in ("ld_base1", "ld_w020")},
}
with open(OUT, "w") as f:
    json.dump(out, f, separators=(",", ":"))
for k, v in out["tilt"].items():
    print(k, "tilt: n", len(v["t"]), "max", max(v["y"]), "abort", v["abort"], "end", v["end"])
for k, v in out["beam"].items():
    print(k, "beam: min", min(v["y"]), "max", max(v["y"]), "abort", v["abort"])
print("wrote", os.path.abspath(OUT), os.path.getsize(OUT), "bytes")
