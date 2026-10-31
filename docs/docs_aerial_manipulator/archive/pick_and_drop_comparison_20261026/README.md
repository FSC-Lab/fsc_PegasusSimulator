# Pick-and-drop, three controllers, 200 g: the test (2026-10-26)

User request: compare all three controllers on the pick-and-drop task at one payload mass, 200 g.
Then (mid-campaign): only TEST whether the comparison works first, do not produce all the results.
The full campaign (3 runs each) was stopped during the whole-body rig's run 2; the test flew one run
per controller (MAC: up to 3 attempts).

## Setup, the same for every rig

- 07 scene with `PEGASUS_PNP_TASK=drop`: the Box_Assembly payload at **200 g** (`PEGASUS_PNP_PAYLOAD_MASS`),
  clamped by its fin, 0.77 m pillars, the 35 mm tray on the PLACE hat.
- One planner block (the whole-body `_sim_pick_and_drop.yaml`; the MAC yaml carries it verbatim, the
  geometric rig reads it via `WB_PLANNER_YAML`).
- Headless Isaac at RTF 1, raw mocap, the mirror plant.
- `PEGASUS_ARM_SPIN_DRAIN=1` on every rig (archive/pick_place_mac_20261026).
- Driver `--drop-place --grip-via-action --grip-axis-tol 0.003 --timetable`.

The drop timetable (new in `pnp_mission_v2.py`):
- The clamped payload is lifted at once: no hold after the close, unlike the hook's timetable.
- Ready To Drop starts 54 s after Go To Start, so every rig hovers ≥ 6 s at Drop Start and the
  planner's aim-at-the-basket applies to all.
- The release comes 2 s after the drop leg ends.
- Go To Land Start is at +26 s from Ready To Drop, Land at +18.3 s.

Measured: those phase starts agree across rigs to 0.011 s. Only the lift time varies (25.5 / 28.9 s).

## Result

| controller | attempts | completed | outcome |
|---|---|---|---|
| whole-body 4-D L1 | 1 | 1 | **flat in the tray**, 41 mm off centre; claw 7.8 mm off its target at release; eps_UAV max/rms 146/22 mm; EE RMSE 25 mm |
| Geo-L1 | 1 | 1 | **flat in the tray**, 38 mm off centre; claw 44.6 mm off at release; eps_UAV 317/62 mm; EE RMSE 66 mm |
| MAC | 3 | 0 | every attempt picked and lifted, then left DIRECT 5.6-7.5 s into the carry with every arm torque on the 3 N·m clamp |

MAC's failure is the j1 (arm yaw) oscillation seen on this morning's drops (mac_drop2/3, at the
pick hover). It starts first in j1 (1.4-5.0 s into the carry), then j4, then j2/j3; the carry
includes a 180° turn. It also carries a standing j2/j3 sag of 5.5-6.8°.

MAC on pick-and-drop today, all runs:
- 1 of 3 at 180 g without the timetable: the one success left the payload in the tray but tilted
  18°, with the claw 167 mm off at release.
- 0 of 3 at 200 g.

## Verdict

The comparison pipeline works end to end: the driver's drop timetable, `run_pick_and_drop_campaign.sh`,
and `build_pick_and_drop.py` (MATLAB export, tables, figures). Whole-body and Geo-L1 both complete the
200 g drop. MAC does not complete it reliably: the stiff arm module's j1 loop is the binding problem,
not the payload's position or the place geometry.

## UPDATE: the drain was only half a fix, and MAC's j1 oscillation was the remaining delay

User question afterwards: "can MAC achieve a lighter payload?".

**The finding.** A passive probe (`tools/arm_lag_probe.py`, run by `tools/probe_when_flying.sh`)
recorded three points on the torque path with their publish stamps:
- **A:** MAC's command, `.../external_torque_controller/joint_torque_command`.
- **B:** the arm stack's output, `/uav_0/isaacsim_manipulator/effort_commands`.
- **C:** Isaac's applied effort, `/uav_0/isaacsim_manipulator/joint_states`.

On the first 100 g flight:
- A -> B: 0-2 ms.
- B -> C: 6-14 ms calm, **36-38 ms in the oscillation** (r ≈ 1.00). C is visibly B shifted by 36 ms.

**The cause.** With a zero timeout, rclpy's `wait_for_ready_callbacks` hands out ONE message per ready
subscription per wait and then raises `TimeoutException`. The first drain returned at that exception,
so it took one effort message per step. A backlog therefore never emptied (the arm stack publishes at
exactly the 250 Hz physics rate) and sat at the queue depth: 10 x 4 ms.

Loopback proof (system rclpy, same Humble code, ROS domain 77), with a 10-message backlog:

| version | messages taken |
|---|---|
| first drain | 1 per step, every step |
| fixed drain (waits again after every wait that yielded work) | all 10 in the first step |

`spin_once` reproduces the persistent 40 ms delay exactly. **06 fixed the same day**:
`_drain_arm_node` now stops only at a wait that finds nothing. The probe then read B -> C 0-6 ms,
and MAC's j1 stayed calm through the whole flight.

MAC pick-and-drop with the FIXED drain (`tools/chain_mac_mass.sh`, prefix `macfix`):

| run | payload | outcome | claw off at release | payload at rest |
|---|---|---|---|---|
| macfix_m200_1 | 200 g | completed | 173 mm | in the tray, 66 mm off centre, tilted 8.7° |
| macfix_m200_2 | 200 g | completed | 172 mm | **flat**, 73 mm off centre |
| macfix_m100_1 | 100 g | hover trip 10.8 s after DIRECT entry, before the pick | -- | -- |
| macfix_m100_2 | 100 g | completed | 133 mm | **flat**, 78 mm off centre |

- **MAC can do the 200 g pick-and-drop.** Every j1 failure above, and probably part of the 10-08
  pick-and-place failures, was the simulator's delay.
- **A lighter payload only trims its error:** the claw is 172 mm off at 200 g and 133 mm at 100 g.
  MAC sits ~12 cm low and ~11 cm behind its target (no integral action, no arm gravity model).
- **The remaining failure is mass-independent.** macfix_m100_1 is the attitude module's slow
  divergence in hover (body torques to ±3-4 N·m over ~6 s, adaptive gains climbing) — the same trip
  as the free-flight campaign's 6 of 24 and mac_drain_m50.
- **The whole-body and Geo-L1 200 g test runs above flew the HALF fix.** They are moved to
  `results/simulation_results/Isaac_simulation_data/pick_and_drop/partial_drain_20261026/`, and their generated MATLAB and
  table outputs were deleted; a comparison campaign must re-fly all three rigs. The flawed-drain MAC
  100 g flights are in `runs/partial_drain_20261026/`.

## Files

- `tools/run_pnd.sh`: one flight on any rig (exports the scene knobs before the stack starts the tmux server).
- `results/utils/run_pick_and_drop_campaign.sh [N] [rigs]`: the campaign; `MAX_ATTEMPTS`, `PNP_MASS`.
- `results/utils/build_pick_and_drop.py`: imports `build_pick_and_place.py`'s loader and metrics.
  - Raw runs: `results/simulation_results/Isaac_simulation_data/pick_and_drop/`.
  - MATLAB export: `results/simulation_results/matlab_simulation_data/pick_and_drop/`.
- `runs/`: every flight; `runs/stale/` = the stopped whole-body run-2 attempt.
- MAC pick-and-drop yaml: generated by `modular_adaptive_20260930/tools/make_modular_yaml.py --profile pick_and_drop`
  into fsc_autopilot_ros2/config.

## THE CAMPAIGN (2026-10-26 evening): all three controllers, 3 runs each

Drop Margin 0.20, Drop Start 1.0 for all (a 0.05 margin was tried and not kept: MAC's ~125 mm standing
height error would put its basket into the tray wall; `../drop_margin_20261026/`). 9 of 11 attempts kept:

| | eps_UAV max / rms [mm] | EE RMSE [mm] | claw at release [mm] | payload flat in the tray |
|---|---|---|---|---|
| Proposed | 133 / 22 | 25 | 7 | 3 of 3 |
| Geo-L1 | 331 / 65 | 68 | 40 | 2 of 3 |
| MAC | 139 / 93 | 119 | 169 | 1 of 3 (all inside the tray) |

Not kept: whole-body attempt 3.1 (the sim-only 1.47 Hz roll mode in the hover before the pick) and Geo-L1
attempt 1.1 (a step 0.6 s late). Data: `results/simulation_results/Isaac_simulation_data/pick_and_drop/`; MATLAB:
`results/simulation_results/matlab_simulation_data/pick_and_drop/` (its README has every run).
