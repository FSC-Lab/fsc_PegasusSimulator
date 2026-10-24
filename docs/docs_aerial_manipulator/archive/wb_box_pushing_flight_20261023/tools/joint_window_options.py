#!/usr/bin/env python3
"""Claw room OUT / IN along the arm (constant height) before the first q2/q3 limit, for other push
poses at the same 60 deg fold and for a lower q3 guard. Imports joint_window.py's model/IK.
Usage: /usr/bin/python3 joint_window_options.py"""
import io, contextlib
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import joint_window as J


def room(pose, q3_guard=10.0, q2_wall=45.0, q3_wall=50.0):
    p0 = J.claw(*pose)
    out = {}
    for name, d in (("out", np.array([0, 1, 0])), ("in", np.array([0, -1, 0]))):
        q = pose
        for s in np.arange(0.0005, 0.0805, 0.0005):
            sol = J.ik(p0 + s * d, *q)
            if sol is None:
                out[name] = (s, "reach")
                break
            q = sol
            why = [w for c, w in ((q[0] > q2_wall, "q2 wall"), (q[1] > q3_wall, "q3 wall"),
                                  (q[1] < q3_guard, "q3 guard")) if c]
            if why:
                out[name] = (s, "+".join(why))
                break
        else:
            out[name] = (0.08, ">80")
    return p0, out


print(f"{'pose / limits':38s} {'claw out':>9s} {'claw z':>7s} | {'room OUT':>16s} | {'room IN':>16s}")
for label, pose, kw in (("today [0,32,28,0]", (32.0, 28.0), {}),
                        ("[0,30,30,0]", (30.0, 30.0), {}),
                        ("[0,27.5,32.5,0]", (27.5, 32.5), {}),
                        ("[0,26,34,0] (the 10-07 pose)", (26.0, 34.0), {}),
                        ("today, q3 guard 5 deg", (32.0, 28.0), {"q3_guard": 5.0}),
                        ("today, q3 guard 5 + q2 wall 50", (32.0, 28.0), {"q3_guard": 5.0, "q2_wall": 50.0})):
    p0, r = room(pose, **kw)
    print(f"{label:38s} {p0[1]*1e3:7.1f}mm {p0[2]*1e3:6.1f}mm | {r['out'][0]*1e3:5.1f} mm {r['out'][1]:>9s} | "
          f"{r['in'][0]*1e3:5.1f} mm {r['in'][1]:>9s}")
