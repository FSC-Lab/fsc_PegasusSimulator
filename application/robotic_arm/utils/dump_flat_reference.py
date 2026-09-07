#!/usr/bin/env python3
"""Dump a flat_bspline_planner plan for the C++ port's parity test.

The C++ planner (client_lib/src/flat_planner.cpp) is a port, so it is held to
the Python the same way wb_controller is held to controller.py: a fixture of
control points and sampled references, compared field by field. Regenerate
whenever either side changes.

    python3 dump_flat_reference.py > .../tests/data/flat_plan_t650.txt
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(
    os.path.join(_HERE, "..", "..", "..", "extensions", "fsc_aerial_manipulation")))

from fsc_aerial_manipulation.robotic_arm.utils_planner import (  # noqa: E402
    flat_bspline_planner as FB, transition_planner as TP)

FIELDS = ["x_cd", "x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4",
          "b1_d", "b1_d_dot", "b1_d_ddot",
          "r_ed", "r_ed_dot", "r_ed_ddot",
          "b1_de", "b1_de_dot", "b1_de_ddot", "q_d", "qdot_d"]

# Two cases: a plain transition, and one whose kinematic sizing is relaxed so
# the input-bound dilation actually runs.
CASES = [
    ("nominal", dict(Ncheck=101), np.radians([15.0, 25.0, 20.0, 35.0])),
    ("dilated", dict(Ncheck=101, v_max=9.0, a_max=9.0, w_max=9.0, dw_max=9.0,
                     tau_joint_max=0.7625), np.radians([15.0, 25.0, 20.0, 35.0])),
]


def main():
    P = TP.make_params_t650()
    home = np.array([0.0, np.radians(40.0), np.radians(40.0), 0.0])
    r0 = {"x_b": np.array([0.0, 0.0, 1.2]), "phi": 0.0, "q": home}
    out = [f"cases {len(CASES)}"]
    for name, opts, goal in CASES:
        r1 = {"x_b": np.array([0.6, -0.35, 1.55]), "phi": np.radians(25.0),
              "q": goal}
        pl = FB.plan_flat_transition(P, r0, r1, opts)
        out.append(f"case {name}")
        out.append(f"T {pl['T']!r}")
        for key in ("xc", "psi", "q"):
            c = np.atleast_2d(pl["spl"][key][0].c)
            if c.shape[0] == 1 and key == "psi":
                c = c.T
            out.append(f"ctrl {key} {c.shape[0]} {c.shape[1]}")
            out.append(" ".join(repr(float(v)) for v in c.reshape(-1)))
        ts = np.linspace(0.0, pl["T"], 37)
        out.append(f"samples {len(ts)}")
        for t in ts:
            r = pl["ref"](t)
            row = [float(t)]
            for f in FIELDS:
                row.extend(float(v) for v in np.atleast_1d(r[f]))
            out.append(" ".join(repr(v) for v in row))
    print("\n".join(out))


if __name__ == "__main__":
    main()
