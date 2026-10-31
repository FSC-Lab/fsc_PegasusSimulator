#!/usr/bin/env python3
"""rosbag2 -> npz for the 2026-10-27 box-pushing flights: the 10-21 box-pushing extractor, unchanged
(the generic list already carries the raw VRPN poses and the arm's velocity observer)."""
import os, sys, importlib.util
HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "extract_1021", os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools", "extract.py"))
X = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(X)
if __name__ == "__main__":
    X.E.main(sys.argv[1], sys.argv[2])
