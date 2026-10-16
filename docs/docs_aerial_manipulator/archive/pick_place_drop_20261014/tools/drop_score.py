#!/usr/bin/env python3
"""The DROP itself, for any rig (2026-10-15: written to compare the decoupled rig with the
whole-body tray runs). pnp_score.py still gives the tilt, tracking and step times; its bin
constants describe the 10-14 172 mm box, so the drop is scored here.

    /usr/bin/python3 drop_score.py runs/tray6.npz runs/geo_drop1.npz ...

Per run: the claw's error when the gripper opened and the payload's offset from the tray
centre then; the payload's lowest bottom over the tray's outline on the way in; the body's
lurch after the release and how close the claw came back to the release point during the
exit; where the payload came to rest (flat = on the tray floor, tilt < 5 deg). The tray: the
2026-10-15 default, 35 mm walls, rim 0.813 m, TURNED 90 deg (290 mm along x, 371 along y),
centred on the place hat (-1, -1). DROP_RIM overrides the rim (0.950 for drop1..drop7).
"""
import os
import sys

import numpy as np

TRAY_C = np.array([-1.0, -1.0])
TRAY_HALF_OUT = np.array([0.145, 0.1855])
TRAY_HALF_IN = TRAY_HALF_OUT - 0.0025
PAY_HALF = np.array([0.1135, 0.153])   # payload footprint with its marker balls, at yaw +-180
PAY_BOTTOM = 0.0325                    # payload origin above its bottom
RIM = float(os.environ.get("DROP_RIM", "0.813"))
FLOOR_REST = RIM - 0.035 + 0.0025 + PAY_BOTTOM   # payload origin resting flat on the tray floor


def tilt_deg(q):
    w, x, y, z = np.atleast_2d(q).T
    return np.degrees(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))


def yaw_deg(q):
    w, x, y, z = np.atleast_2d(q).T
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def at(T, X, t):
    return np.array([np.interp(t, T, X[:, i]) for i in range(X.shape[1])])


def score(path):
    d = np.load(path, allow_pickle=True)
    names, mt = list(d["marks_name"]), d["marks_t"]
    mark = lambda n: mt[names.index(n)] if n in names else None
    rig = str(d["rig"])
    print(f"== {os.path.basename(path)} ({rig}): "
          f"{'ABORTED: ' + str(d['reason']) if bool(d['aborted']) else 'completed'}")
    pay, od, claw, arr = d["payload"], d["odom"], d["claw"], d["arrival"]
    t_open = mark("gripper_open:start")
    if t_open is None:
        print("   no gripper open: the drop was not reached")
        return
    # the hover on the drop point: the claw's arrival error over the last 1.5 s before the open
    m = (arr[:, 1] == 3) & (arr[:, 0] > t_open - 1.5) & (arr[:, 0] <= t_open)
    if m.any():
        e = arr[m, 4:7].mean(axis=0) * 1e3
        print(f"   claw vs the drop point, last 1.5 s before the open: |e| {arr[m, 3].mean() * 1e3:5.1f} mm "
              f"(target - claw x {e[0]:+.1f}, y {e[1]:+.1f}, z {e[2]:+.1f} mm)")
    # the payload when the gripper was commanded open and when it started to fall (its height
    # 10 mm below where it hung at the command -- the fingers have let go)
    z_open = np.interp(t_open, pay[:, 0], pay[:, 3])
    after = (pay[:, 0] >= t_open) & (pay[:, 3] < z_open - 0.010)
    t_rel = pay[after, 0][0] if after.any() else None
    if t_rel is not None:
        print(f"   payload starts to fall {t_rel - t_open:.2f} s after the open command")
    for lab, t in (("open cmd", t_open), ("release", t_rel)):
        if t is None:
            continue
        p = at(pay[:, 0], pay[:, 1:8], t)
        off = (p[:2] - TRAY_C) * 1e3
        print(f"   payload at {lab:9s}: off the tray centre x {off[0]:+6.1f} y {off[1]:+6.1f} mm, "
              f"bottom {(p[2] - PAY_BOTTOM - RIM) * 1e3:+6.1f} mm vs the rim, tilt {tilt_deg(p[3:7])[0]:4.1f} deg, "
              f"yaw {yaw_deg(p[3:7])[0]:+7.1f} deg")
    # the payload's lowest bottom while over the tray's outline, Ready To Place .. the open
    t0 = mark("ready_place:start")
    if t0 is not None:
        mm = (pay[:, 0] >= t0) & (pay[:, 0] <= t_open)
        over = np.all(np.abs(pay[mm, 1:3] - TRAY_C) < TRAY_HALF_OUT + PAY_HALF, axis=1)
        if over.any():
            zb = pay[mm, 3][over] - PAY_BOTTOM
            print(f"   lowest payload bottom over the tray's outline before the open: {zb.min():.3f} m "
                  f"({(zb.min() - RIM) * 1e3:+.0f} mm vs the rim)")
    # the body's lurch after the release and the claw's closest return to the release point
    o0 = at(od[:, 0], od[:, 1:4], t_open)
    t_ex0 = mark("exit_place:start")
    w = (od[:, 0] >= t_open) & (od[:, 0] <= (t_ex0 if t_ex0 is not None else t_open + 4.0))
    dxy = np.linalg.norm(od[w, 1:3] - o0[:2], axis=1)
    k = int(np.argmax(dxy))
    print(f"   body lurch after the open, before the exit: {dxy[k] * 1e3:5.0f} mm at +{od[w, 0][k] - t_open:.1f} s "
          f"(z {(od[w, 3].min() - o0[2]) * 1e3:+.0f} / {(od[w, 3].max() - o0[2]) * 1e3:+.0f} mm)")
    c0 = at(claw[:, 0], claw[:, 1:4], t_open)
    t_ex1 = mark("exit_place:end")
    if t_ex0 is not None:
        we = (claw[:, 0] >= t_ex0) & (claw[:, 0] <= (t_ex1 or claw[-1, 0]))
        dc = np.linalg.norm(claw[we, 1:4] - c0, axis=1)
        print(f"   exit: started {t_ex0 - t_open:.1f} s after the open; during it the claw's closest return to "
              f"the release point {dc.min() * 1e3:.0f} mm, then {dc[-1] * 1e3:.0f} mm away")
    # the rest pose
    p = pay[-1, 1:8]
    off = (p[:2] - TRAY_C) * 1e3
    inside = bool(np.all(np.abs(p[:2] - TRAY_C) < TRAY_HALF_IN) and p[2] < RIM + PAY_BOTTOM)
    flat = bool(abs(p[2] - FLOOR_REST) < 0.005 and tilt_deg(p[3:7])[0] < 5.0)
    verdict = "FLAT in the tray" if flat else ("in the tray, NOT flat" if inside else "NOT in the tray")
    print(f"   payload at rest: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}), off the centre x {off[0]:+.0f} "
          f"y {off[1]:+.0f} mm, tilt {tilt_deg(p[3:7])[0]:.1f} deg -> {verdict}")


if __name__ == "__main__":
    for f in sys.argv[1:]:
        score(f)
