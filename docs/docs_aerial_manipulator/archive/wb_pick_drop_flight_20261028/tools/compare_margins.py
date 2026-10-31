#!/usr/bin/env python3
"""2026-10-28 pick-and-drop, four flights at drop margin 12 / 8 / 6 / 5 cm (the bag names): which margin drops best?

Per flight, from the bag: the Drop Start height and the drop leg's claw target (pick_place/info goals), the claw at
the gripper's OPEN command, the basket at the OPEN command (centre height, tilt), the fall, the peak fall speed,
the bounce, where it first came to rest (3-5 s after the OPEN) and how far it slid on the way there.

What the bag does NOT hold: the drop margin and the Drop point are parameters typed / Get-captured on the arm GS
(pick_place_drop_height, pick_place_place_point). info [68..71] stays invalid for the drop row, the bag has no
/parameter_events, and the GS reports "Drop Margin set" only on its screen. So:
  - the drop point is reconstructed as claw target - Rz(yaw) [ee_offset x, y, 0] - [0, 0, ee_offset z + margin],
    margin = the bag's name. Its xy is exact (the claw target and the EE offset are known); its z is only as
    good as the name. The claw-target z's are 1.241 / 1.219 / 1.178 / 1.168 m: steps of 22 / 41 / 10 mm against
    the names' 40 / 20 / 10, so the "8 cm" flight's claw was ~2 cm higher than its name says (either the margin
    reached the planner as ~0.10, the yaml's value, or the Drop point was re-captured 2 cm higher).
  - every height below is therefore ABSOLUTE (world z), measured on the basket: the tray floor is where the
    basket rests in it (0.930-0.932 m for a flat basket).
usage (PYTHONNOUSERSITE=1 /usr/bin/python3): compare_margins.py [run ...]   (default m12 m8 m6 m5)"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.abspath(os.path.join(HERE, "..", "npz"))
T16 = os.path.abspath(os.path.join(HERE, "..", "..", "wb_pick_drop_flight_20261016", "tools"))
sys.path.insert(0, T16)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", "..", "results", "utils")))
from pd15_common import leg_windows, obj_pose, rosout, tilt_of, yaw_of  # noqa: E402
from release import vel                                                 # noqa: E402
from experiment_pick_and_place import basket_outliers                   # noqa: E402

RUNS = {"m12": ("12 cm", "16:16", "flight_wb_l1_4d_pick_and_drop_12offset_20261028_161640"),
        "m8": ("8 cm", "16:27", "flight_wb_l1_4d_pick_and_drop_8offset_20261028_162711"),
        "m6": ("6 cm", "16:48", "flight_wb_l1_4d_pick_and_drop_6offset_20261028_164816"),
        "m5": ("5 cm", "16:56", "flight_wb_l1_4d_pick_and_drop_5offset_20261028_165603")}
EE_OFF = np.array([-0.02, 0.0, 0.16])   # hardware _pick_and_drop.yaml (fsc_autopilot_ros2 a12e454) pick_place_ee_offset
MARGIN = {"m12": 0.12, "m8": 0.08, "m6": 0.06, "m5": 0.05}   # the bag names
HALF_H = 0.0325                          # basket centre above its bottom
TRAY_REST_Z = 0.931                      # basket centre resting flat on the tray floor (M6 0.932, M5 0.930)


def load(run):
    d = np.load(os.path.join(NPZ, f"{run}.npz"), allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
    return d, t0


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def analyse(run):
    d, t0 = load(run)
    I = d["pp_info__data"]
    ok = np.where(np.isfinite(I[:, 14 + 7 * 3 + 4]))[0]
    goals = I[ok[-1], 14:56].reshape(6, 7)
    claw_t, yaw = goals[3, 4:7], goals[3, 3]
    pt = claw_t - Rz(np.radians(yaw)) @ np.array([EE_OFF[0], EE_OFF[1], 0.0]) - np.array([0, 0, EE_OFF[2] + MARGIN[run]])
    win = leg_windows(d, t0)
    a, b = next(v for k, v in win.items() if k.startswith("execute_place"))
    to, Po, Qo = obj_pose(d, t0)
    bad = basket_outliers(Po, yaw_of(Qo))
    to, Po, Qo = to[~bad], Po[~bad], Qo[~bad]
    Vo = vel(to, Po)
    te = d["pl_current_ee__recv"] - t0
    Pe = np.column_stack([d[f"pl_current_ee__pose.position.{k}"] for k in "xyz"])
    t_open = [t for t, n, m in rosout(d, t0, contains="gripper OPEN") if t > a][0]
    j = min(np.searchsorted(te, t_open), len(te) - 1)
    pre = (to > t_open - 0.2) & (to <= t_open)
    p_open = np.array([np.interp(t_open, to, Po[:, k]) for k in range(3)])   # as the MATLAB export's outcome
    tilt_open = float(np.median(tilt_of(Qo[pre])))
    # the release: the first sample 1 cm below the OPEN pose; first contact: the downward speed stops after > 0.3 m/s
    k0 = int(np.argmax((to > t_open) & (Po[:, 2] < p_open[2] - 0.01)))
    vz = Vo[:, 2]
    k_fast = k0 + int(np.argmax(vz[k0:] < -0.3))
    k_hit = k_fast + int(np.argmax(vz[k_fast:] > -0.05))
    after = (to > to[k_hit]) & (to < to[k_hit] + 2.0)
    bounce = float(Po[after, 2].max() - Po[k_hit, 2])
    rest_m = (to >= t_open + 3.0) & (to <= t_open + 5.0)       # the first rest, before anyone touches it
    rest = np.median(Po[rest_m], axis=0)
    rest_tilt = float(np.median(tilt_of(Qo[rest_m])))
    slide = float(np.linalg.norm(rest[:2] - Po[k_hit, :2]))
    still = np.where(np.linalg.norm(Po[k0:] - rest, axis=1) > 0.005)[0]
    t_settle = float(to[k0 + still[still + k0 < np.searchsorted(to, t_open + 5.0)][-1]] - t_open) if len(still) else 0.0
    in_tray = abs(rest[2] - TRAY_REST_Z) < 0.03
    return dict(run=run, label=RUNS[run][0], flown=RUNS[run][1], bag=RUNS[run][2], margin_name=MARGIN[run],
                drop_start_z=goals[2, 2], claw_t=claw_t, pt=pt, yaw=yaw, leg=(a, b), t_open=t_open, wait=t_open - b,
                claw_err=(Pe[j] - claw_t) * 1e3, p_open=p_open, open_off_xy=(p_open - pt)[:2] * 1e3, tilt_open=tilt_open,
                bottom_above_floor=(p_open[2] - TRAY_REST_Z) * 1e3, v_hit=float(vz[k_fast:k_hit + 1].min()),
                bounce=bounce * 1e3, rest=rest, rest_off_xy=(rest - pt)[:2] * 1e3, rest_tilt=rest_tilt, slide=slide * 1e3,
                t_settle=t_settle, in_tray=in_tray)


def main(runs):
    out = [analyse(r) for r in runs]
    f = lambda v: "(" + ", ".join(f"{x:+.0f}" for x in v) + ")"    # noqa: E731
    for o in out:
        print(f"\n===== {o['run']}  {o['label']} (flown {o['flown']}), Drop Start z {o['drop_start_z']:.3f}, drop claw target "
              f"{np.round(o['claw_t'], 3)}, yaw {o['yaw']:.0f}")
        print(f"  drop point (xy exact, z from the name) {np.round(o['pt'], 3)};  Ready To Drop {o['leg'][0]:.2f}-{o['leg'][1]:.2f} s,"
              f" OPEN {o['t_open']:.2f} (+{o['wait']:.2f} s after the leg)")
        print(f"  claw at OPEN - target {f(o['claw_err'])} mm")
        print(f"  basket at OPEN: z {o['p_open'][2]:.3f} m ({o['bottom_above_floor']:.0f} mm above its flat rest on the tray floor), "
              f"xy from the drop point {f(o['open_off_xy'])} |{np.linalg.norm(o['open_off_xy']):.0f}| mm, tilt {o['tilt_open']:.1f} deg")
        print(f"  peak fall speed {o['v_hit']:.2f} m/s, bounce {o['bounce']:.0f} mm, slid {o['slide']:.0f} mm after the first contact, "
              f"settled {o['t_settle']:.1f} s after the OPEN")
        print(f"  first rest (OPEN + 3..5 s): {np.round(o['rest'], 3)}  {'IN THE TRAY' if o['in_tray'] else 'NOT IN THE TRAY'}, "
              f"xy from the drop point {f(o['rest_off_xy'])} |{np.linalg.norm(o['rest_off_xy']):.0f}| mm, tilt {o['rest_tilt']:.1f} deg")
    print("\nsummary: name | claw target z | basket z at OPEN, above rest | claw err at OPEN mm | fall speed | bounce | slide | rest")
    for o in out:
        rest = (f"in the tray, {np.linalg.norm(o['rest_off_xy']):.0f} mm from the drop point, {o['rest_tilt']:.1f} deg"
                if o['in_tray'] else f"off the tray, z {o['rest'][2]:.2f} m")
        print(f"  {o['label']:>5s} | {o['claw_t'][2]:.3f} | {o['p_open'][2]:.3f}, {o['bottom_above_floor']:3.0f} mm | {f(o['claw_err']):16s} | "
              f"{o['v_hit']:.2f} | {o['bounce']:3.0f} | {o['slide']:3.0f} | {rest}")
    return out


if __name__ == "__main__":
    main(sys.argv[1:] or list(RUNS))
