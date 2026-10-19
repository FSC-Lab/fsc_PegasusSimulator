"""The drop hold (end of execute_place -> gripper OPEN) of E-1 / D-1: the planner's claw error (arrival_error),
the basket relative to the drop point, and what an EE-error trigger would have done.
usage: drop_hold.py [run ...]  (default e1 d1)"""
import sys
import numpy as np
from pd15_common import *   # noqa: F401,F403


def main(run):
    d, t0 = load(run)
    win = leg_windows(d, t0)
    a, b = win["execute_place (approach)"]
    I = d["pp_info__data"]
    ok = np.where(np.isfinite(I[:, 14 + 7 * 3 + 4]))[0]
    info = I[ok[-1]]
    xb, yaw, ee = info_goal(info, 3)
    cap = info[56 + 3 * 4:56 + 3 * 4 + 4]
    print(f"\n===== {run} {RUNS[run][0]}  execute_place {a:.2f}-{b:.2f} s")
    print(f"drop claw target {np.round(ee, 3)}  base goal {np.round(xb, 3)} yaw {yaw:.1f}  drop Get (capture) {np.round(cap, 3)}")
    ta = d["pp_arr__recv"] - t0; A = d["pp_arr__data"]
    opens = [t for t, n, m in rosout(d, t0, contains="gripper OPEN") if t > a]
    t_open = opens[0] if opens else np.nan
    exit_t = [t for t, n, m in rosout(d, t0, contains="exit_place: ") if t > a]
    print(f"leg end {b:.2f}  Drop click {t_open:.2f} ({t_open - b:+.2f} s after the leg end)  exit start {exit_t[0] if exit_t else np.nan:.2f}")
    m = (ta >= b - 1.0) & (ta <= t_open + 1.5)
    print("   t     |e| mm   e_xyz (target - claw) mm         within settled")
    for k in np.where(m)[0][::2]:
        print(f"  {ta[k]:6.2f}  {A[k, 2] * 1e3:6.1f}   [{A[k, 3] * 1e3:6.1f} {A[k, 4] * 1e3:6.1f} {A[k, 5] * 1e3:6.1f}]   {A[k, 7]:.0f} {A[k, 8]:.0f}")
    if run == "e1":
        return
    to, Po, Qo, _ = mocap_pose(d, t0, "obj_vrpn")
    i = np.searchsorted(to, t_open)
    print(f"basket at the click: {np.round(Po[i], 3)}  tilt {tilt_of(Qo[i:i + 1])[0]:.1f}")
    # when did it leave the jaws (z starts falling), and where it came to rest
    z = Po[:, 2]
    k0 = i + np.argmax(z[i:] < z[i] - 0.01)
    print(f"basket starts to fall at {to[k0]:.2f} s ({to[k0] - t_open:+.2f} s after the click), at {np.round(Po[k0], 3)}")
    rest = Po[-1]
    print(f"basket at rest {np.round(rest, 3)} tilt {tilt_of(Qo[-1:])[0]:.1f};  rest - basket-at-release (xy) "
          f"{np.round((rest - Po[k0])[:2] * 1e3, 0)} mm;  rest - drop Get (xy) {np.round((rest[:2] - cap[:2]) * 1e3, 0)} mm")
    # basket xy over the hold, around the drop Get point
    mh = (to >= b) & (to <= to[k0])
    rel = (Po[mh, :2] - cap[:2]) * 1e3
    print(f"basket - Get over the hold (mm): mean {np.round(rel.mean(axis=0), 0)}  std {np.round(rel.std(axis=0), 0)} "
          f" min {np.round(rel.min(axis=0), 0)} max {np.round(rel.max(axis=0), 0)}")


if __name__ == "__main__":
    for r in (sys.argv[1:] or ["e1", "d1"]):
        main(r)
