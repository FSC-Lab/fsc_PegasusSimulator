"""Shared helpers for the 2026-10-27 box-pushing flights (171114, 172227): the 10-24 evening helpers,
with this folder's npz looked up first, so the earlier days load the same way."""
import os, importlib.util
HERE = os.path.dirname(os.path.abspath(__file__))
PREV = os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261024_evening", "tools")
_spec = importlib.util.spec_from_file_location("bp_common_1024ev", os.path.join(PREV, "bp_common.py"))
_p = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_p)
NAV, STEPS, GUARD = _p.NAV, _p.STEPS, _p.GUARD
tilt_deg, yaw_deg, window = _p.tilt_deg, _p.yaw_deg, _p.window
_p._p.NPZ_DIRS.insert(0, os.path.join(HERE, "..", "npz"))
Bag = _p.Bag
BAGS = ["171114", "172227"]
MASS = {"171114": "280 g", "172227": "280 g"}
