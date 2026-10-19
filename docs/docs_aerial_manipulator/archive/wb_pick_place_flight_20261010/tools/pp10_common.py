"""Shared setup for the 2026-10-10 whole-body pick-and-place hardware flight (T650-AM, 4-D L1, fused feedback).

Reuses the 2026-10-09 analysis (../../wb_pick_place_flight_20261009/tools: pp_common, tracking, grasp, grasp_geom ...)
and adds the one 10-10 run:
  p4  flight_wb_l1_4d_pick_and_place_20261010_190719   pick hooked, carried, place attempted, pilot took over
npz: ../npz/p4.npz (extract_bag.py + np1_compat.py of the 10-09 tools); p1..p3 are symlinks to the 10-09 npz.
Run with PYTHONNOUSERSITE=1 /usr/bin/python3 (numpy 1.21 + apt matplotlib/scipy).
"""
import os, sys
HERE10 = os.path.dirname(os.path.abspath(__file__))
os.environ["AM_NPZ"] = os.path.abspath(os.path.join(HERE10, "..", "npz"))
T09 = os.path.abspath(os.path.join(HERE10, "..", "..", "wb_pick_place_flight_20261009", "tools"))
sys.path.insert(0, T09)
import pp_common as PC          # noqa: E402
from pp_common import *         # noqa: E402,F401
PC.RUNS["p4"] = ("PP-4", "19:07", "flight_wb_l1_4d_pick_and_place_20261010_190719")
PC.C.FLIGHTS["p4"] = "PP-4 19:07"
OUT10 = os.path.abspath(os.path.join(HERE10, "..", "analysis"))
OUT09 = os.path.abspath(os.path.join(HERE10, "..", "..", "wb_pick_place_flight_20261009", "analysis"))
