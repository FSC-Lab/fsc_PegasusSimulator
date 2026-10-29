"""Shared helpers for the 2026-10-28 valve-turning flights (180652, 181833): the 10-21 box-pushing Bag
helper pointed at this folder's npz."""
import os, importlib.util
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "bp_common_1021", os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools", "bp_common.py"))
_c = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_c)
_c.NPZ = os.path.join(HERE, "..", "npz")
NAV, GUARD = _c.NAV, _c.GUARD
tilt_deg, yaw_deg, window = _c.tilt_deg, _c.yaw_deg, _c.window
Bag = _c.Bag
BAGS = ["180652", "181833"]
