# Physical interaction with the whole-body 4-D L1 law (2026-09-27)

Question: can the 4-D attribution's phase flag chi (`wb_l1_contact`, threshold
fallback `wb_l1_collision_threshold_n`) carry pick-and-place and box pushing,
what should trigger it, and what payload / push force does the H1b tune carry?
Report (artifact "Interaction Feasibility, AM-T650"): `report.html`
(built from `report_src.html` + `analysis/report_data.json` by `tools/build_report.py`).

Nothing existing was edited. New files only:
- `application/robotic_arm/07_px4_direct_t650_aerial_manipulator_interaction.py`
  -- loads 06 as a module and adds an EE force injector
  (`/uav_0/isaacsim_manipulator/ee_force_cmd`, world N, latched 0.5 s sim;
  truth on `.../ee_force_state`).
- `scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_interaction_sitl.sh`
  -- copy of the 4-D launcher, entrypoint 07.
- `fsc_autopilot_ros2/config/params_..._4d_direct_actuation_t650_sim_interaction_{thr2,contact}.yaml`
  -- mirror yaml + `wb_l1_omega_x 2.0`, `wb_l1_omega_x_t/r/q 0.2072`, and
  either `wb_l1_collision_threshold_n 2.0` or `wb_l1_contact true`.

## Tools
- `tools/interaction_bench.py` -- offline: exact law (wb_entry_sim.Law + the
  j4 estimate cap), 4-D observer, mirror plant, circle_bench feedback, hardware
  servo caps; scenarios `force`, `box`, `payload` (exact rigid attachment +
  desk spring); runtime chi policies `free|contact|gripper|task[+thrN]`;
  `rd=` reading bandwidth; `base_ff=True|"rot"` what-if (not in the flown law).
- `tools/sweep_*.py` -> `analysis/sweep_*.json`; `tools/summarize.py <json>`.
- `tools/hw_fread_extract.py` / `hw_fread_noise.py` -- the reading's free-flight
  noise floor from the 0918/0924 bags (`data/hw_*.npz`).
- `tools/interaction_cycle.sh <mirror|thr2|contact> <tag> --seg t0:hold:dir:F ...`
  -- one Isaac flight (clean slate, stack with WB_SIM_YAML, 07, hover soak +
  `interaction_force_driver.py`); `tools/score_isaac.py data/int_<tag>.npz`.
  Let a cycle FINISH before starting the next: its hover driver outlives the
  force profile, and a second cycle's clean slate does not kill it.

## Results
- chi is read once at startup in the C++ node; only the threshold switches at
  runtime. The rendered F_hat shares `wb_l1_omega_x` (H1b 0.2072, tau 4.8 s)
  with the u3 feed-forward; with M_y 1 kg vs natural vertical EE inertia
  0.31-0.41 kg a 100 g placement delivered ~1.5 N of the 3.1 N commanded.
  Reading at 2 rad/s + per-block 0.2072 fixes it; free flight identical.
- Hardware noise floor of |C_x F_hat| (p99.9, 5 flights): 0.97 / 1.32 / 1.86 N
  at 0.207 / 2 / 5 rad/s -> threshold 2 N; cannot see 100 g (0.98 N).
- Capacity (bench, hardware caps): payload 0.3 kg recommended (76 % of cap),
  0.4 kg edge, >= 0.5 kg j2/j3 saturate; box friction along the arm 6 N
  recommended, 8-10 N near the 20 deg tilt guard, 12 N aborts; across the arm
  ~1 N (j1 0.34 N.m cap).
- Isaac (real node): T1 thr2 latched 0.6-0.8 s, rendered 89-94 %; T3 static
  contact 96-103 %, deflection = F/K_y within ~1 mm; T2 shipped (free) held the
  EE stiffly but drove q3 to its +50 deg stop on every push >= 3 N and, after
  the 7 N push, oscillated with j2 at the 3.0 N.m clamp (hardware cap 2.44).
