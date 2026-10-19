"""2026-10-16 pick-and-drop: the grasp, the basket's hang in the claw during the carry, and the drop
(drop point, claw and basket at the click / release, where it came to rest).
Drop point = claw target of execute_place - Rz(yaw) * pick_place_ee_offset - [0, 0, pick_place_drop_height]
(hardware _pick_and_drop.yaml: ee_offset [-0.02, 0, 0.16], drop_height 0.20; the drop row is Get-captured, not in info).
usage: drop16.py [run ...]  (default d1 d2; e1 has no basket in the jaws)"""
import sys
import numpy as np
from pd15_common import *   # noqa: F401,F403
from pp_common import quat_R
from release import vel

EE_OFF = np.array([-0.02, 0.0, 0.16]); DROP_H = 0.20


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def drop_point(d):
    I = d["pp_info__data"]
    ok = np.where(np.isfinite(I[:, 14 + 7 * 3 + 4]))[0]
    xb, yaw, ee = info_goal(I[ok[-1]], 3)
    return ee - Rz(np.radians(yaw)) @ EE_OFF - np.array([0, 0, DROP_H]), ee, yaw


def claw(d, t0):
    te = d["pl_current_ee__recv"] - t0
    Pe = np.column_stack([d[f"pl_current_ee__pose.position.{k}"] for k in "xyz"])
    Qe = np.column_stack([d[f"pl_current_ee__pose.orientation.{k}"] for k in "wxyz"])
    return te, Pe, Qe


def main(run):
    d, t0 = load(run)
    win = leg_windows(d, t0)
    pt, ee_t, yaw = drop_point(d)
    to, Po, Qo = obj_pose(d, t0)
    Vo = vel(to, Po)
    te, Pe, Qe = claw(d, t0)
    tb, _, _, yb = px4_rpy(d, t0)
    print(f"\n===== {run} {RUNS[run][0]}  drop point {np.round(pt, 3)}  yaw {yaw:.0f}  claw target {np.round(ee_t, 3)}")
    # grasp
    tc = [t for t, n, m in rosout(d, t0, contains="gripper CLOSE")][0]
    tx = [t for t, n, m in rosout(d, t0, contains="Exit To Pick at once")][0]
    tg, gp = gripper(d, t0)
    msg = [m for t, n, m in rosout(d, t0, contains="Exit To Pick at once")][0]
    print(f"grasp: CLOSE {tc:.2f}  grasp reported {tx:.2f} (+{tx - tc:.2f} s, '{msg.split(' -- ')[0]}')  gripper pos at report "
          f"{gp[np.searchsorted(tg, tx), 0]:.4f}, +2 s {gp[min(np.searchsorted(tg, tx + 2), len(tg) - 1), 0]:.4f}")
    # hang in the vehicle frame (yaw only): basket - claw, during the carry legs
    def hang(a, b):
        m = (to >= a) & (to < b)
        Pc = np.column_stack([np.interp(to[m], te, Pe[:, k]) for k in range(3)])
        ps = np.interp(to[m], tb, np.unwrap(yb))
        rel = np.array([Rz(-p) @ (x - c) for p, x, c in zip(ps, Po[m], Pc)])
        return rel * 1e3, tilt_of(Qo[m])
    for lab in ("exit_pick (up the vertical margin)", "go_to_place_start", "execute_place (approach)"):
        a, b = win[lab]
        rel, tl = hang(a, b)
        print(f"  hang {lab[:22]:22s}: basket - claw (vehicle x fwd / y left / z, mm) mean {np.round(rel.mean(0), 0)} "
              f"std {np.round(rel.std(0), 0)}   basket tilt mean {tl.mean():5.1f} p-p {np.ptp(tl):5.1f}")
    # the drop
    a, b = win["execute_place (approach)"]
    t_open = [t for t, n, m in rosout(d, t0, contains="gripper OPEN") if t > a][0]
    i = np.searchsorted(to, t_open)
    k0 = i + np.argmax(Po[i:, 2] < Po[i, 2] - 0.01)
    j = np.searchsorted(te, t_open)
    rest = Po[-1]
    rel_click, _ = hang(t_open - 0.5, t_open)
    print(f"drop: leg end {b:.2f}  click {t_open:.2f} (+{t_open - b:.2f} s)  basket leaves {to[k0]:.2f} (+{to[k0] - t_open:.2f} s)")
    print(f"  claw at click - claw target (mm) {np.round((Pe[j] - ee_t) * 1e3, 0)}   basket - claw last 0.5 s (vehicle, mm) {np.round(rel_click.mean(0), 0)}")
    print(f"  basket at release - drop point (xy mm) {np.round((Po[k0] - pt)[:2] * 1e3, 0)}  |.| {np.linalg.norm((Po[k0] - pt)[:2]) * 1e3:.0f}"
          f"  v_xy {np.round(Vo[k0, :2], 3)} m/s  z {Po[k0, 2]:.3f} (rest z {rest[2]:.3f}, fall {Po[k0, 2] - rest[2]:.3f} m)")
    print(f"  rest - drop point (xy mm) {np.round((rest - pt)[:2] * 1e3, 0)}  |.| {np.linalg.norm((rest - pt)[:2]) * 1e3:.0f}"
          f"   rest tilt {tilt_of(Qo[-1:])[0]:.1f} deg   rest yaw {np.degrees(yaw_of(Qo[-1:]))[0]:.1f}   rest - release (xy mm) {np.round((rest - Po[k0])[:2] * 1e3, 0)}")
    # basket trajectory after release
    for k in range(k0, min(k0 + 200, len(to)), 15):
        print(f"     {to[k]:6.2f}  p-pt (mm) {np.round((Po[k] - pt) * 1e3, 0)}  v {np.round(Vo[k], 2)}  tilt {tilt_of(Qo[k:k + 1])[0]:5.1f}")


if __name__ == "__main__":
    for r in (sys.argv[1:] or ["d1", "d2"]):
        main(r)
