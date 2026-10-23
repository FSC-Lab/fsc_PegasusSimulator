# Joint-posture-PID root cause, removal, and entry tune — 2026-09-09

Full analysis: `../Removing the Joint-Posture Term.md`. Command.md §7.15.9.
Offline reproduction tool: `application/robotic_arm/utils/wb_entry_sim.py`.

Answers three questions about the whole-body L1 law's joint-posture PID
(present in every flight before this date): which disturbance makes the
published law fail without it, whether gain tuning alone fixes it, and what
minimal law change removes the need for it while keeping the manuscript's
proof structure. Two additions to `u3` — `wb_ee_anchor_com` (anchor the
free-flight EE reference to the CoM) and `wb_u3_internal_ff` (feed the
internal-disturbance estimate into `u3`'s coupling feedforward) — together
remove the need for the term. Both are default-off; parity-locked (`WbParityTest`
/ `WbL1ParityTest` / `WbReferenceBuilderTest`, 9/9, unchanged with the options
off). The five `wb_posture_*` keys were then deleted from
`params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml`
— absent, not merely zeroed, so the term is unreachable from that file. A
follow-up entry-transient tune (`wb_k_x` 16→32, `wb_k_v` 12→20) then halved the
residual SAFETY→DIRECT transient with no UDE seed.

Headline flight counts, all with the posture term off and both options on
unless noted: 3 mismatch-A/B fix flights, 2 feedforward-source (lumped vs.
internal-estimate) flights, 9-cell thrust×model mismatch grid (10/15% thrust,
5/10% model, 2 runs/cell), 2 full 8-leg missions, 8 entry-transient tuning
flights. Every flown `.npz` is gitignored (`*.npz`, ~230 MB total); regenerate
with the scripts below against a live rig.

## Files

- `run_ablation.sh` — Q1: one disturbance source at a time (motor delay only,
  ±5%/±10% model, ±5%/±10%/±15% thrust), posture off, no compensation.
- `run_fix.sh` — Q3: the two-option fix flown at the full mismatch (15%
  thrust allocator-side + 10% model plant-side), several K_y/D_y points.
- `run_ff_source.sh` — A/B of the feedforward source: filtered lumped
  estimate vs. the design note's internal `T⁻ᵀŵ` estimate.
- `run_grid.sh` — the 9-cell thrust×model mismatch grid at the shipped fix
  configuration.
- `run_fullmission.sh` — the 8-leg standard mission (steps + compatible
  trajectory) as the gate before deleting the `wb_posture_*` keys.
- `run_entry_tune.sh` / `run_entry_tune2.sh` — the entry-transient gain sweep
  (position loop `k_x`/`k_v`, L1 rotational-channel bandwidth `omega_c_r`);
  round 2 exists because round 1's best offline candidate (`omega_c_r` 2.0)
  diverged in flight after ~8 s despite a clean 5-second entry.
- `run_entry_final.sh` — full-mission confirmation of the shipped tune.
- `score.py`, `entry_score.py`, `grid_table.py` — metrics from a run's
  `.npz` (peak/settled end-effector and base error, arm-from-home, joint
  clamp, DIRECT-entry recovery time).
- `compensation_metrics.json`, `grid_metrics.json` — the last generated
  metrics tables (regenerate with the `*_table.py` / `score.py` scripts
  against fresh `.npz` files — these two are illustrative snapshots, not the
  source of truth for numbers quoted in the report).
- `runs.txt` — one line per flight: tag, injected mismatch, gains, config.

## Running one

```bash
# full step-0 clean between EVERY run — PX4 stays armed otherwise and the
# next takeoff produces no lift
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y
# each run_*.sh restores the yaml to its pre-campaign state on exit (trap),
# and is itself a full launch+fly+teardown cycle — no manual steps needed
./run_grid.sh shiqi_machine
```
