#!/usr/bin/env python3
"""One line per run of THIS campaign (the CAD-box campaign's summarize, pointed at ../runs):
    PYTHONNOUSERSITE=1 /usr/bin/python3 summarize.py [tag ...]
"""
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_cad_box_20261007", "tools"))
import summarize as S          # noqa: E402

S.RUNS = os.path.join(HERE, "..", "runs")
if __name__ == "__main__":
    if len(sys.argv) == 1:
        sys.argv += sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(S.RUNS, "*.npz")))
    S.main()
