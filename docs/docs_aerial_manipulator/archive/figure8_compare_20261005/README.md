# Figure-8 comparison at 0.20 m/s: whole-body L1 vs geometric L1 adaptive (2026-10-05)

Isaac at RTF 1, the circle comparison's launch condition (headless, pacer, P-core pinning, mirror
plant), flying a figure-8 sized for a 0.20 m/s mean end-effector speed. Report: the comparison
artifact "Simulation: Free-flight Comparison", v9 (https://claude.ai/artifact/Wc4gqiUpWU8YGXsRrufTRT),
built from `report.html` here. Commands: Command.md §14.

## The trajectory

| | circle (2026-10-01) | figure-8 (this campaign) |
|---|---|---|
| shape | r 0.5 m | A 0.75 m half-length, B 0.375 m half-width (1.50 × 0.75 m) |
| path / lap | 3.142 m / 24.0 s | 4.573 m / 22.865 s |
| mean EE speed | 0.131 m/s | 0.200 m/s |
| q2 | 25 ± 15° @ 6.0 s (4 per lap) | 25 ± 15° @ 5.716 s (4 per lap) |
| fold / ramps / laps / s | 55° / 4 s / 1 / 1.0 | same |
| peak yaw rate / CoM accel. | 15.1°/s / 0.075 m/s² | 51.3°/s / 0.33 m/s² |
| takeoff heading | 0° | 45° (long axis on world x) |

Sizing (`analysis/sizing_candidates.txt`, `tools/fig8_probe.cpp` linked to the planner library):
at 0.20 m/s the CoM speed peak exceeds the end effector's (the airframe swings around the EE at the
lobe tops), and the shared 0.30 m/s bound leaves only A ≥ 0.70 m plannable at s = 1. A = 0.75 m has
the most margin (s_max 1.07) and the lowest yaw rate / CoM acceleration. B = A/2 = the planner's
default proportion; a thinner 8 lowers the peak yaw rate only 6–12 %.

## Code changes this needed (all default-off in code)

- **fsc_trajectory_planner** (`whole_body_trajectory_planner_node.cpp`, uncommitted on `main`):
  `ee_traj_v_max` / `ee_traj_a_max` / `ee_traj_w_max` (EE run only; 0 = inherit the shared
  v_max/a_max/w_max, which also size every transition) and `ee_traj_q2_cycles_per_lap`
  (> 0 sets the q2 period = lap_time / cycles on every plan, so the arm GS's Mean Velocity field,
  which rewrites the lap time, keeps the arm on whole cycles). Rebuilt; its 21 tests pass.
  The shared bounds would cap this figure-8 at s = 0.34 (0.068 m/s, yaw-rate bound).
- **am_ee_compare_driver.py**: `--ee-v-max/--ee-a-max/--ee-w-max`.
- `run_rt1.sh` / `run_isaac.sh` in the two 2026-10-01 campaigns resolved the repo root one level
  short after the move into `archive/` — fixed.

## Hardware files (2026-10-05, after the sim validation, user request)

- `params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml` (the ONE planner
  config both hardware rigs read): `ee_traj_fig8_a/b` 0.75 / 0.375, `ee_traj_q2_cycles_per_lap` 4
  (the circle's 24.166 s lap still gives exactly its 6.0415 s period), `ee_traj_a_max` 0.40,
  `ee_traj_w_max` 1.0, `ee_traj_v_max` 0 (= the shared 0.30). The mirror `_sim.yaml` carries the
  same keys, except `ee_traj_q2_cycles_per_lap` 0, which keeps its own 48 s arm design.
- The two whole-body hardware stacks (raw/fused) REFUSE a planner build without
  `ee_traj_q2_cycles_per_lap` (their existing fatal build check); the two decoupled hardware
  stacks print the new keys and refuse the same way.
- `tools/hw_planner_check.py` loads the hardware yaml into a real planner on a fake rig
  (ROS_DOMAIN_ID 77, /uav_hwcheck): circle at s = 1 unchanged (its s_max rises 1.13 -> 1.78, so the
  GS slider now allows a faster circle), figure-8 at the 22.865 s lap READY at s = 1 (s_max 1.056),
  and with cycles 0 the same figure-8 is refused ("does not start/end at rest") -- ALL PASS.
- On the Orin: pull, rebuild `fsc_trajectory_planner`, take off at yaw 45°, arm GS Figure-8 page:
  Mean Velocity 0.20 (it shows 0.189 until edited, from the circle's lap), Laps 1.
  (Superseded the same afternoon -- see "Flight-test readiness" below.)

## Flights (`data/`, all completed, RTF 1.000)

| run | EE rms / peak [mm] | EE heading rms / peak [deg] | base [mm] | tilt [deg] | actuators |
|---|---|---|---|---|---|
| wb_f8a/b/c | 24.3 / 23.4 / 22.9 (peak 46) | 0.82 / 0.78 / 0.73 (peak 1.9) | 22.7 / 21.9 / 21.7 | ≤ 2.6 | τ_j ≤ 1.05 N·m, 0 clamp, 0 sat |
| decoupled_f8a/b/c | 58.4 / 59.8 / 60.3 (peak 121) | 6.76 / 6.97 / 6.79 (peak 15.9) | 39.5 / 40.3 / 42.4 | 3.8 | motors 0.40–0.79, 0 sat |

Mechanism (`analysis/fig8_mechanism.json`, `analysis/fig8_budget.json`):
- geometric heading error = −0.257 s × reference yaw rate (R² 0.95) vs the ω_d = 0 lag
  K_ω,z/K_R,z = 0.237 s (circle −0.251 s); whole-body slope +0.023 s.
- exact EE split (airframe position / airframe attitude / joints, closure < 0.003 mm):
  WB 22.1 / 8.5 / 7.8 mm (circle 13.4 / 5.0 / 5.2); GEO 40.7 / 31.8 / 9.7 mm (circle 33.4 / 15.9 / 9.4)
  — the geometric attitude term doubles: yaw lag × the arm's ~0.25 m reach.
- geometric mean EE offset along / across the path +30 / −28 mm = the unmatched body-fixed force
  (+0.55, −0.50) N / K_p 20.1 = (+27, −25) mm (the nose follows the tangent, so body = path frame).

## Tools (`tools/`)

`fig8_probe.cpp` (build line in its header; `DUMP=` writes the plan) · `fig8_score.py` (needs the
user-site numpy 2 for the pickled debug array: run with plain `/usr/bin/python3`) ·
`fig8_mechanism.py` · `fig8_budget.py` (source ROS + `~/ros2_ws/install`; `PYTHONNOUSERSITE=1`) ·
`fig8_figures.py` (`PYTHONNOUSERSITE=1`) · `build_report.py` + `section3_template.html` (needs
`latex2mathml`: `pip install --target <dir> latex2mathml`, then `PYTHONPATH=<dir>`).

```bash
./run_fig8.sh wb:<tag> decoupled:<tag>      # headless; PEGASUS_HEADLESS=0 for a window
```

## Conservative sweep (2026-10-05, user request: the geometric peak of 12 cm is ~20 cm in flight)

A 0.60 / 0.70 m (B = A/2) at 0.10 / 0.15 / 0.20 m/s (`run_sweep.sh`, shortened by `run_sweep_tail.sh`
to one flight per controller at A 0.70), scored by `tools/fig8_envelope.py`
(`analysis/envelope_sweep.json`; planner numbers `analysis/sizing_sweep.json`). Flight error =
sim / 0.6; footprint = planned airframe path + predicted base peak, joined with the planned EE path +
predicted EE peak, in a 2 x 2 m square (long axis on world x, centred ~(+0.25, +0.18) m from takeoff).

| A, v | geometric sim rms / peak | predicted flight peak | geo margin / side | WB sim rms / peak | WB margin |
|---|---|---|---|---|---|
| 0.60, 0.10 | 45.2 / 80.5 mm | 134 mm | +169 mm | 16.8 / 34.3 | +217 |
| 0.60, 0.15 | 56.4 / 115.0 | 192 | +164 | 22.7 / 46.0 | +199 |
| 0.60, 0.20 | not plannable (CoM 0.33 m/s > 0.30) | | | | |
| **0.70, 0.10** | **44.1 / 72.8** | **121** | **+110** | **14.3 / 31.2** | **+139** |
| 0.70, 0.15 | 50.6 / 91.0 | 152 | +79 | 19.4 / 37.1 | +128 |
| 0.70, 0.20 | 62.0 / 128.4 | 214 | +67 | 24.1 / 49.0 | +108 |
| 0.75, 0.20 | 59.5 / 120.6 | 201 | +19 | 23.5 / 46.3 | +74 |

Recommended first flight: **A 0.70 m, B 0.35 m, 0.10 m/s** (lap 42.68 s, q2 period 10.67 s). The
geometric peak follows the peak YAW RATE, not the size (smaller 8 at the same speed = worse). Two
whole-body flights (`wb_A060_v010_a`, `wb_A060_v015_b`) tripped the 20 deg watchdog while HOVERING
before their run: the rig's 1.5 Hz pitch mode grew (|e_R| 0.10 -> 0.57 p-p in 16 s, RTF 1.000);
the hover config was identical to the morning's 3 clean flights (only planner keys changed).
Both points were re-flown (`run_refly.sh`: `wb_A060_v010_c`, `wb_A060_v015_c`), both completed, so
every A 0.60 point has 2 flights per controller (WB rms 16.7 / 22.5 mm, peaks unchanged). Report v11.

## Long axis on world x (2026-10-05, user request: "it is diagonal")

The planner anchored the 8 by its START TANGENT along the nose, so the long axis sat atan2(2B, A)
= 45 deg off the nose (diagonal from a yaw-0 hover; the flights above took off at 45 deg to avoid
it). New planner keys `ee_traj_fig8_world_axis` (bool) + `ee_traj_fig8_axis_deg` (world azimuth,
0 = x): the crossing stays at the hovering EE, and of the four start tangents that trace the same 8
(+-alpha, +pi) the one closest to the nose is flown, so go-to-start yaws <= 45 deg. Default off in
code; SET in both 4-D yamls (hardware + mirror sim). gtest `EeTrajectoryWorldAxis` (axis 0 / 90 deg
x 5 hover yaws: extents 2A / 2B within 1 cm, turn <= 45 deg); planner tests 22/22. The hardware
stacks' stale-build check now probes `ee_traj_fig8_axis_deg`.

## Flight-test readiness (2026-10-05 afternoon, user request)

- **Defaults = the recommended first flight.** Both 4-D yamls (hardware + mirror sim):
  `ee_traj_fig8_a/b` 0.70 / 0.35. Arm GS EE Trajectory panel (fsc_open_manipulator
  `ee_trajectory_panel.cpp`): Figure-8 page A 0.70 / B 0.35 / Mean Velocity 0.10 / Laps 1.
  The panel used to derive BOTH pages' speeds from the planner's one shared `ee_traj_lap_time`
  (the circle's 24.17 s on the hardware yaml, i.e. 0.177 m/s for this 8); the lap and lap
  count now go only to the page of the shape the planner has selected (the circle's when none
  is), and the other page keeps its own defaults.
- **Tilt watchdog 15 deg on both rigs** (`system_wd_max_tilt_deg`, single-sample trip on
  acos(R22)): whole-body + geometric HARDWARE yamls, the whole-body mirror `_sim` (it must equal
  the hardware file) and the geometric `_sim`. Was 20. The whole-body stack scripts expected
  `20.0` in their config check (now `15.0`); the two decoupled stack scripts gained a positive
  check of it. The scenario sim yamls (pick_place, push_pull, ps4test, interaction, robustness,
  comparison) keep their own values.
- **The whole hardware workflow checked without a vehicle** (`tools/hw_fig8_workflow_check.py`
  + `tools/gs_harness/`, the REAL arm-GS panel offscreen): the planner launched with each
  hardware stack's own arguments on the hardware yaml, the decoupled bridge in the loop, the
  panel selecting Figure-8 and pressing Go To Start / Start Trajectory / Back To Origin / Start
  Transition only when it enabled them. **whole-body 13/13, decoupled 14/14**: panel opens on
  0.70 / 0.35 / 0.100 / 1 and writes a 42.681 s lap (q2 period 10.670 s); READY at s = 1
  (s_max 2.02), run 46.7 s, peak |v| 0.147 m/s, |a| 0.092, |qdot| 9.0 deg/s, tau_j 0.84 N.m,
  sigma_nd 0.236; EE extents 1.400 x 0.700 m along world x, centred on the EE hover point
  (0.0 mm); q2 10..40, q3 15..45 deg, start = end; arm stream on the rig's own topic; bridge
  same-motion residual 0.001 mm; Back To Origin flown. Both stack scripts' pre-launch checks
  pass on the edited yamls.
- **Where the 8 goes**: its crossing is the END EFFECTOR's hover point, 0.25 m in front of the
  base at fold 55 / q2 25. From a yaw-0 hover at the base point (0, 0) the planned footprint
  (EE + CoM reference) is x -0.55..+1.06, y -0.42..+0.47 m = 1.61 x 0.89 m centred at
  (+0.25, +0.02). To centre it on the grid, hover the base ~0.25 m behind the grid centre
  (-x). Go To Start then yaws exactly 45 deg (with B = A/2 the start tangent sits 45 deg off
  the long axis) and moves the base to (+0.07, -0.18).

```bash
# station: colcon build --packages-select utils_custom_ground_station; harness: tools/gs_harness/CMakeLists.txt
ROS_DOMAIN_ID=77 FASTRTPS_DEFAULT_PROFILES_FILE=<udp-only profile> /usr/bin/python3 tools/hw_fig8_workflow_check.py \
    --rig wb|decoupled --harness <build>/gs_fig8_harness --png <prefix>
```
