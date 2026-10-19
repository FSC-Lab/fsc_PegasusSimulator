#!/usr/bin/env python3
"""2026-10-17: what happens after the jaws open, per run, cruise vs min-snap.

    PYTHONNOUSERSITE=1 /usr/bin/python3 release_window.py

Per run, over the 3 s after the gripper release ends (the open claw still held in the world
until Exit): the base's drift along the push (the range of its y) [mm], q2's peak and q3's
lowest [deg]; and the arm's swing (q2 std) while gripped and still (the last 1 s before the
Push press to the end of the 1.5 s settle) and while sliding (6 s after the press to 4 s
before the push ends).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(A, "push_pull_20261003", "tools"))
from pl_metrics import mark  # noqa: E402

OLD = os.path.join(A, "box_pushing_campaign_20261016", "runs")
NEW = os.path.join(HERE, "..", "runs")
GROUPS = [
    ("min-snap 10-16", OLD, ["s1_m500_1", "s1_m500_2", "s1_m500_3", "s3_P_1", "s3_P_2", "s3_P_3",
                             "s3_PH_1", "s3_PH_2", "g_mu035", "g_stick", "g_m600"]),
    ("min-snap 10-17", NEW, ["m_nom_1", "m_nom_2"]),
    ("cruise 10-17", NEW, ["c_nom_1", "c_nom_2", "c_nom_3"]),
]


def deg(j, c, a, b):
    t = np.arange(a, b, 0.02)
    return np.degrees(np.interp(t, j[:, 0], j[:, c]))


def one(d, tag):
    z = np.load(os.path.join(d, tag + ".npz"), allow_pickle=True)
    ps, pe, gr = mark(z, "push:start"), mark(z, "push:end"), mark(z, "gripper_release:end")
    j, bt = z["joints"], z["base_truth"]
    tt = np.arange(gr, gr + 3.0, 0.02)
    by = np.interp(tt, bt[:, 0], bt[:, 2])
    return dict(drift=np.ptp(by) * 1e3, q2max=deg(j, 2, gr, gr + 3).max(), q3min=deg(j, 3, gr, gr + 3).min(),
                still=deg(j, 2, ps - 1.0, ps + 1.5).std(), slide=deg(j, 2, ps + 6, pe - 4).std())


def main():
    print(f"{'run':10s} {'group':15s} | after the release (3 s): drift  q2 max  q3 min | q2 std: still  sliding")
    for name, d, tags in GROUPS:
        for t in tags:
            if not os.path.isfile(os.path.join(d, t + ".npz")):
                continue
            r = one(d, t)
            print(f"{t:10s} {name:15s} |                  {r['drift']:6.1f}  {r['q2max']:6.1f}  {r['q3min']:6.1f} |"
                  f"         {r['still']:5.2f}   {r['slide']:5.2f}")


if __name__ == "__main__":
    main()
