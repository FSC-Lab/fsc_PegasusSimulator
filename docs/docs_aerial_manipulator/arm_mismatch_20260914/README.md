# Imperfect friction + gravity compensation on the arm — 2026-09-14

**Question (user):** the calibration report (`fsc_open_manipulator/doc/Calibration
Result for PWM Torque Control.pdf`) says friction and gravity compensation are the
dominant corrections on the real arm. Put both into the simulated plant with a 5 %
mismatch against what the arm controller compensates, and see whether the whole
system still stabilizes. No back-EMF: the arm's 1.5 Hz current loop removes it on
hardware (Command.md §7.15.10), only its residual is simulated.

**Plant (Pegasus `06` + `servo_model.py`):** the report's eq. (5) gearbox friction
`[fc + µ|τ|]·tanh(q̇/w)` on the measured joint velocity, momentum-clamped so an
explicit application at 250 Hz can never reverse a joint, scaled by
`sim_arm_friction_scale`; arm link masses and inertias × `sim_arm_mass_scale` while
every model (the flight law's, the arm controller's, 06's own hold) keeps the
nominal arm. Everything else is the shipped `_sim` plant (+15 % allocator kf, body
mass/inertia ×1.10 with the 10/10/5 mm CoM shift, MN4010 rotor lag, current-loop
residual).

**Compensation (fsc_open_manipulator, `torque_controller_isaac_aerial.yaml`):** the
same `fc`/`µ` in N·m, `passthrough_auxiliary_terms: true`, driven by the
whole-body planner's reference velocity; coupled to the plant by the launcher
(`passthrough_corrections:=` on `torque_control_isaac.launch.py`). Dither off —
the sim plant has no stiction for it to break.

| run | plant friction | arm mass | compensation |
|---|---|---|---|
| `l1_mismatch_A` | ×1.05 | ×1.05 | ON (×1.00) — the shipped `_sim` yaml |
| `l1_matched_B`  | ×1.00 | ×1.00 | ON |
| `l1_uncomp_C`   | ×1.05 | ×1.05 | OFF |
| `l1_mission_kx32` (posture_ablation_20260909) | none | ×1.00 | — ideal-arm baseline, same mission and 16 s holds |

```bash
docs/docs_aerial_manipulator/arm_mismatch_20260914/run_mismatch.sh shiqi_machine
/usr/bin/python3 application/robotic_arm/utils/wb_l1_metrics.py arm_mismatch_20260914/*.npz posture_ablation_20260909/l1_mission_kx32.npz
/usr/bin/python3 docs/docs_aerial_manipulator/arm_mismatch_20260914/hold_chatter.py <the same npz>
```

Results and the mechanism: Command.md §7.15.12. Figure `compare_mismatch.png`,
full metrics `metrics.txt`, launcher/driver logs in `logs/`. Raw `.npz` stay out
of git (`.gitignore`).
