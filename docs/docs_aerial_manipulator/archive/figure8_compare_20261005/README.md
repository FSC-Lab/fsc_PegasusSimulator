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
