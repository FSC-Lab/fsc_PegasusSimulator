"""How the basket hangs on the claw in THIS campaign's runs (../runs/<tag>.npz): the fingers' pitch, the basket's
tilt and its centre against the claw point, by the 10-10 flight analysis' own tool (sim_hang.run), which is only
pointed at this folder -- nothing is written.
usage: PYTHONNOUSERSITE=1 /usr/bin/python3 hang_check.py <tag> [<tag> ...]
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "..", "wb_pick_place_flight_20261010", "tools")))
import sim_hang
sim_hang.RUNS_DIR = os.path.normpath(os.path.join(HERE, "..", "runs"))
for tag in sys.argv[1:]:
    sim_hang.run(tag, table=False)
