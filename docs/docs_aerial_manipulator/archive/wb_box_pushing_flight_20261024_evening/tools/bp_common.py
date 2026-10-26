"""Shared helpers for the 2026-10-24 EVENING box-pushing flights (280 g 194310, 510 g 195657):
the 10-24 helpers, with this folder's npz looked up first, so the earlier days load the same way."""
import os, importlib.util
HERE = os.path.dirname(os.path.abspath(__file__))
PREV = os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261024", "tools")
_spec = importlib.util.spec_from_file_location("bp_common_1024", os.path.join(PREV, "bp_common.py"))
_p = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_p)
NAV, STEPS, GUARD = _p.NAV, _p.STEPS, _p.GUARD
tilt_deg, yaw_deg, window = _p.tilt_deg, _p.yaw_deg, _p.window
_p.NPZ_DIRS.insert(0, os.path.join(HERE, "..", "npz"))
Bag = _p.Bag
BAGS = ["194310", "195657"]
MASS = {"194310": "280 g", "195657": "510 g"}
