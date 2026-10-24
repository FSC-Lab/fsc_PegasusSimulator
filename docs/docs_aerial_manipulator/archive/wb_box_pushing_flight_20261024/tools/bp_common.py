"""Shared helpers for the 2026-10-24 box-pushing flight analysis: the 10-21 helpers, with the npz
looked up in this folder (10-24), the 10-23 or the 10-21 folder, so every day loads the same way."""
import os, importlib.util
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
OLD = os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools")
_spec = importlib.util.spec_from_file_location("bp_common_1021", os.path.join(OLD, "bp_common.py"))
_old = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_old)
NAV, STEPS, GUARD = _old.NAV, _old.STEPS, _old.GUARD
tilt_deg, yaw_deg, window = _old.tilt_deg, _old.yaw_deg, _old.window

NPZ_DIRS = [os.path.join(HERE, "..", "npz"), os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261023", "npz"), os.path.join(OLD, "..", "npz")]
BAGS = ["150423", "151159", "151634", "162850", "163413", "164419"]
PUSHES = {"170123": "10-21", "172853": "10-23", "150423": "10-24"}


class Bag(_old.Bag):
    def __init__(self, tag):
        self.tag = tag
        for d in NPZ_DIRS:
            p = os.path.join(d, f"b{tag}.npz")
            if os.path.exists(p):
                break
        self.z = np.load(p, allow_pickle=True)
        self.t0 = min(self.z[k][0] for k in self.z.files if k.endswith("__recv") and len(self.z[k]))
