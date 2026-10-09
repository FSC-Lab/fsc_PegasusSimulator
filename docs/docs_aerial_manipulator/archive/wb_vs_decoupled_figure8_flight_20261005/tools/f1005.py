"""Shared setup for the 2026-10-05 figure-8 flights (whole-body 4-D L1 vs decoupled geometric + L1): reuses the 0928
analysis (../../wb_vs_decoupled_flight_20260928/tools: common.py's model + loaders, metrics.analyse) on the 1005 bags.

Runs (npz names in $AM_NPZ; several names point at one extracted bag, like the 1002 analysis):
  w3, w4  flight_wb_l1_4d_figure8_vel010_20261005_162406   runs 1 and 2, 0.10 m/s (planner EXECUTING T=46.7 s)
  r5      flight_decoupled_l1_figure8_vel010_20261005_163734  0.10 m/s
  r6      flight_decoupled_l1_figure8_vel013_20261005_165431  0.13 m/s (EXECUTING T=36.8 s)
  w5, w6  flight_wb_l1_4d_figure8_vel013_20261005_170129   runs 1 and 2, 0.13 m/s
Each run is scored over its whole EXECUTING span (the planner status 'EXECUTING T=...' with T >= 20 s, which is the
figure-8 run including its 4 s speed-up and slow-down), 0.1 s trimmed at each end by metrics.analyse.
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_vs_decoupled_flight_20260928", "tools"))
import common as C            # noqa: E402
import metrics as M           # noqa: E402
from common import *          # noqa: E402,F401

RUNS = {  # key: (tag, date/time, group, speed [m/s], bag, index of the figure-8 run in the bag)
    "w3": ("WB-3", "10-05 16:24", "wb", 0.10, "flight_wb_l1_4d_figure8_vel010_20261005_162406", 0),
    "w4": ("WB-4", "10-05 16:24", "wb", 0.10, "flight_wb_l1_4d_figure8_vel010_20261005_162406", 1),
    "r5": ("DEC-5", "10-05 16:37", "new", 0.10, "flight_decoupled_l1_figure8_vel010_20261005_163734", 0),
    "r6": ("DEC-6", "10-05 16:54", "new", 0.13, "flight_decoupled_l1_figure8_vel013_20261005_165431", 0),
    "w5": ("WB-5", "10-05 17:01", "wb", 0.13, "flight_wb_l1_4d_figure8_vel013_20261005_170129", 0),
    "w6": ("WB-6", "10-05 17:01", "wb", 0.13, "flight_wb_l1_4d_figure8_vel013_20261005_170129", 1),
}
C.FLIGHTS.update({k: f"{v[0]} {v[1]}" for k, v in RUNS.items()})
_orig_windows = C.windows


def windows(nm):
    if nm not in RUNS:
        return _orig_windows(nm)
    d, t0 = load(nm)
    mk = "wbmode" if "wbmode__recv" in d.files else "gmode"
    md = edges(d, t0, mk)
    st = edges(d, t0, "pl_status")
    runs = []
    for i, (t, v) in enumerate(st):
        if v.startswith("EXECUTING T=") and float(v.split("=")[1].rstrip("s")) >= 20.0:
            runs.append((t, st[i + 1][0]))
    c0, c1 = runs[RUNS[nm][5]]
    a = max(t for t, v in md if v == "DIRECT" and t < c0)          # the DIRECT engagement that contains the run
    b = next((t for t, v in md if v == "SAFETY" and t > c0), float(d[f"{mk}__recv"][-1] - t0))
    return a, b, c0, c1


C.windows = windows; M.windows = windows
