# 2026-10-28 pick-and-drop: the Drop Margin lowered 12 → 8 → 6 → 5 cm

Four hardware flights of the whole-body 4-D L1 pick-and-drop (T650-AM, EKF2-fused feedback, `AM_HW_PROFILE=pick_and_drop`,
fsc_autopilot_ros2 `a12e454`, arm `f812b48`). Mission, controller and payload are the same in all four. Only the
Drop Margin changed: how far above the Drop point the gripper opens. Bags:
`docs/experimental_data_ros2_bag/1028 - T650-AM whole-body Tracking, Pick&Drop, Valve-Turning-*/…/flight_wb_l1_4d_pick_and_drop_<N>offset_*`.

**Verdict: 5 cm is the best drop, and it is exported for MATLAB** to
`results/experiment_results/matlab_experiment_data/pick_and_drop/` (`results/utils/experiment_pick_and_drop.py`). The bag is
copied unchanged to `results/experiment_results/flight_test_data/pick_and_drop/whole_body_l1_pnd_M5/`.

## Results (`tools/compare_margins.py`, output in `compare_margins.txt`)

Heights are world z, measured on the basket. A flat basket rests on the tray floor at z = 0.931 m (M6 0.932, M5 0.930).
"Released" = the basket centre at the gripper's OPEN command, above that rest height.

| run | flown | claw target z [m] | released [mm] | peak fall speed [m/s] | bounce [mm] | slide [mm] | where it came to rest | ‖ε<sub>UAV</sub>‖ max / RMS [mm] | EE RMSE [mm] |
|---|---|---|---|---|---|---|---|---|---|
| M12 | 16:16 | 1.241 | 138 | 1.05 | 12 | — | bounced off the tray, fell to the floor (z 0.11 m) | 209.9 / 58.1 | 59.7 |
| M8 | 16:27 | 1.219 | 120 | 1.03 | 13 | 27 | in the tray, 71 mm from the drop point, tilted 13° | 168.8 / 53.7 | 56.1 |
| M6 | 16:48 | 1.178 | 81 | 0.81 | 8 | 66 | in the tray, 129 mm from the drop point, tilted 5° | 165.4 / 50.5 | 53.4 |
| **M5** | 16:56 | 1.168 | **75** | **0.73** | **1** | **16** | **in the tray, 58 mm from the drop point, tilted 2.9°** | **158.9 / 47.2** | **50.2** |

- **The drop improves as the margin comes down.** Each step down gives a slower impact, less bounce and a flatter rest.
  The 12 cm basket hit the tray wall and went over the side.
- **5 cm wins every measure:** the lowest release, the slowest impact, no bounce, the least slide, the flattest rest,
  the closest to the drop point. It also has the best whole-mission tracking.
- **6 and 5 cm released at nearly the same height** (81 vs 75 mm): the basket's hang under the claw varies by a few
  mm per flight. The difference in outcome is the slide after landing (66 vs 16 mm).
- **All four drops were pressed by hand** (the arm GS's Drop button), 0.5–2.7 s after Ready To Drop ended. The planner's
  automatic drop was not used. Ready To Drop was pressed 0.6–2.7 s after arriving at Drop Start, so the planner never
  aimed at the measured basket ("NOT aimed: 0.0 s / 0 basket samples" in M5; the aim needs a 4 s hover there).
- **The claw sat 45–59 mm forward of its target at the OPEN command** (world −x at yaw −180; M12 23 mm). Every
  in-tray basket came to rest ~55–70 mm forward of the drop point.
- **Pilot landing:** in each flight the pilot switched to STAB during the land leg, at ~0.36 m.

## What the bag cannot say

- **The Drop Margin and the Drop point are arm-GS parameters** (`pick_place_drop_height`, `pick_place_place_point`).
  pick_place/info holds no valid drop row, the bag has no `/parameter_events`, and the GS shows "Drop Margin set" only on
  its screen. The margins above are the bag names.
- **The drop point is reconstructed** from the drop leg's claw target (info goals):
  claw target − Rz(yaw)·[−0.02, 0, 0] − [0, 0, 0.16 + margin]. Its xy is exact; its z assumes the name.
- **M8 does not fit its name:** the claw-target steps are 22 / 41 / 10 mm against the names' 40 / 20 / 10 mm. M8's claw
  sat ~2 cm higher than an 8 cm margin gives. Either the planner kept the yaml's 0.10, or the Drop point was re-captured
  higher. The xy of the drop point moved by up to 3 cm between flights, so the Drop point was re-captured (or typed) each time.
- **The M12 basket was put back on the tray by hand** after the landing; its "rest" is taken at OPEN + 3–5 s.

## Files

- `tools/run_extract.sh`: bag → `npz/m12|m8|m6|m5.npz`, using the 10-16 extractor and np1_compat.
- `tools/compare_margins.py`: the table above; run with `PYTHONNOUSERSITE=1 /usr/bin/python3`.
- `npz/` is gitignored.
