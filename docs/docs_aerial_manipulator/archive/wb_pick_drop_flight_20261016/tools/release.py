"""D-1's release: the basket's velocity when the jaws opened, its fall, and where an instantaneous release at
each moment of the drop hold would have put it (ballistic: p + v*t_fall, t_fall from its height above the rest).
Also the grasp of every run: gripper position at the close, the basket relative to the claw after it.
usage: release.py"""
import numpy as np
from pd15_common import *   # noqa: F401,F403
from pp_common import quat_R

PLACE_DROP_H = 0.20                       # pick_place_drop_height (hardware pick-and-drop yaml)
EE_OFF = np.array([-0.02, 0.0, 0.16])     # pick_place_ee_offset


def vel(t, P, w=0.05):
    """Central difference over +-w s (mocap is 120 Hz; a 0.1 s span keeps the noise below ~1 cm/s)."""
    V = np.zeros_like(P)
    for k in range(len(t)):
        a = np.searchsorted(t, t[k] - w); b = min(np.searchsorted(t, t[k] + w), len(t) - 1)
        if b > a:
            V[k] = (P[b] - P[a]) / (t[b] - t[a])
    return V


def drop_point(d, t0):
    I = d["pp_info__data"]
    ok = np.where(np.isfinite(I[:, 14 + 7 * 3 + 4]))[0]
    xb, yaw, ee = info_goal(I[ok[-1]], 3)
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    off = np.array([c * EE_OFF[0] - s * EE_OFF[1], s * EE_OFF[0] + c * EE_OFF[1], EE_OFF[2] + PLACE_DROP_H])
    return ee - off, ee, yaw


def main():
    d, t0 = load("d1")
    win = leg_windows(d, t0)
    a, b = win["execute_place (approach)"]
    pt, ee, yaw = drop_point(d, t0)
    print(f"D-1 drop point (claw target - Rz(yaw) ee_offset - drop height): {np.round(pt, 3)}, yaw {yaw:.0f}")
    to, Po, Qo = obj_pose(d, t0)
    Vo = vel(to, Po)
    t_open = [t for t, n, m in rosout(d, t0, contains="gripper OPEN") if t > a][0]
    i = np.searchsorted(to, t_open)
    k0 = i + np.argmax(Po[i:, 2] < Po[i, 2] - 0.01)
    rest = Po[-1]
    print(f"click {t_open:.2f}; basket leaves {to[k0]:.2f}: p {np.round(Po[k0], 3)}  v {np.round(Vo[k0], 3)} m/s "
          f"(horizontal {np.linalg.norm(Vo[k0, :2]):.3f})")
    for k in range(k0, min(k0 + 80, len(to)), 6):
        print(f"   {to[k]:6.2f}  p {np.round(Po[k], 3)}  v {np.round(Vo[k], 2)}  tilt {tilt_of(Qo[k:k + 1])[0]:5.1f}")
    print(f"rest {np.round(rest, 3)} tilt {tilt_of(Qo[-1:])[0]:.1f};  rest - drop point (xy, mm) {np.round((rest - pt)[:2] * 1e3, 0)}"
          f";  at release - drop point {np.round((Po[k0] - pt)[:2] * 1e3, 0)}")
    # what an instantaneous release would have done over the last 4 s of the leg and the hold
    print("\nballistic landing point - drop point (mm), if released at t:")
    zr = rest[2]
    for t in np.arange(b - 4.0, t_open + 0.01, 0.25):
        k = np.searchsorted(to, t)
        h = max(Po[k, 2] - zr, 0.0)
        tf = np.sqrt(2 * h / 9.80665)
        land = Po[k, :2] + Vo[k, :2] * tf
        print(f"   {t:6.2f}  basket-pt {np.round((Po[k, :2] - pt[:2]) * 1e3, 0)}  v_xy {np.round(Vo[k, :2], 3)}"
              f"  t_fall {tf:.2f}  landing-pt {np.round((land - pt[:2]) * 1e3, 0)}  |.| {np.linalg.norm(land - pt[:2]) * 1e3:5.0f}")


def grasps():
    print("\n=== the grasps: gripper position at the close and the basket in the claw frame after it")
    for run in ("d1", "d2"):
        d, t0 = load(run)
        tg, gp = gripper(d, t0)
        tc = [t for t, n, m in rosout(d, t0, contains="gripper CLOSE")][0]
        tx = [t for t, n, m in rosout(d, t0, contains="closed around the stem")][0]
        g = gp[:, 0]
        print(f"{run}: CLOSE {tc:.2f}, reported closed {tx:.2f};  gripper pos before {g[np.searchsorted(tg, tc) - 5]:.4f}"
              f"  at report {g[np.searchsorted(tg, tx)]:.4f}  +1 s {g[np.searchsorted(tg, tx + 1.0)]:.4f}  +2 s {g[min(np.searchsorted(tg, tx + 2.0), len(g) - 1)]:.4f}")
        to, Po, Qo = obj_pose(d, t0)
        te = d["pl_current_ee__recv"] - t0
        Pe = np.column_stack([d[f"pl_current_ee__pose.position.{k}"] for k in "xyz"])
        Qe = np.column_stack([d[f"pl_current_ee__pose.orientation.{k}"] for k in "wxyz"])
        for dt in (-0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5):
            k = np.searchsorted(to, tx + dt); j = min(np.searchsorted(te, tx + dt), len(te) - 1)
            rel = quat_R(*Qe[j]).T @ (Po[k] - Pe[j])
            print(f"     t {tx + dt:6.2f}  basket-claw (claw frame, mm) {np.round(rel * 1e3, 0)}  basket z {Po[k, 2]:.3f}"
                  f"  claw z {Pe[j, 2]:.3f}  basket tilt {tilt_of(Qo[k:k + 1])[0]:5.1f}")


if __name__ == "__main__":
    main()
    grasps()
