"""(2026-10-18, Opus) The 14 s after an abort (injected, guard or watchdog), from the npz:
truth tilt / altitude, arm, gripper, net contact on the rotor, the beam, DIRECT. From the repo root:
    PYTHONNOUSERSITE=1 /usr/bin/python3 docs/.../tools/abort_trace.py tag [tag ...]"""
import sys
import numpy as np
R = "docs/docs_aerial_manipulator/archive/valve_turning_20261015/runs/"


def tilt(q):
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


for tag in sys.argv[1:]:
    d = np.load(R + tag + ".npz", allow_pickle=True)
    od, J, g, c, box, bt = d["odom"], d["joints"], d["grip"], d["contact"], d["box"], d["base_truth"]
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    ev = d["events"].tolist()
    ta = None
    for e in ev:
        if "INJECT abort at" in e or "ABORTING" in e:
            ta = float(e.split("s]")[0].strip("[ ")); break
    if ta is None:
        ta = od[od[:, 11] > 0.5, 0].max()
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (box[:, 4] * box[:, 7] + box[:, 5] * box[:, 6]),
                                          1 - 2 * (box[:, 6] ** 2 + box[:, 7] ** 2))))
    print(f"== {tag}: abort {ta - marks.get('push:start', ta):.1f} s into the push step")
    for dt in [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3, 3.5, 4, 5, 6, 8, 14]:
        t = ta + dt
        idx = lambda X: min(np.searchsorted(X[:, 0], t), len(X) - 1)
        i, j, k, ic, ib, m = idx(od), idx(J), idx(g), idx(c), idx(box), idx(bt)
        F = c[ic, 1:4] + c[ic, 4:7]
        print(f"  t{dt:+5.1f}: tilt {tilt(bt[m:m + 1, 4:8])[0]:5.1f} z {bt[m, 3]:5.2f} "
              f"q {np.degrees(J[j, 1:5]).round(0)} grip {g[k, 1]:6.3f} |F| {np.linalg.norm(F):5.2f} "
              f"beam {yaw[ib] - yaw[0]:6.0f} direct {od[i, 11]:.0f}")
    print(f"  final: tilt {tilt(bt[-50:, 4:8]).max():.1f} deg, z {bt[-1, 3]:.3f} m")
