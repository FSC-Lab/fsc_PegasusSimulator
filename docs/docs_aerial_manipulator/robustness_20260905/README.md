# Whole-body hover robustness — 2026-09-05

Question: does the AM-T650 whole-body law hold a hover under combined model
uncertainty, motor delay and thrust loss, with and without the joint-posture
PID term that is not in the published law?

## Injection

Set in **section 1** of `params_single_aerial_manipulator_whole_body_direct_actuation_t650_sim.yaml`.

| deviation | knob | value | what it does |
|---|---|---|---|
| mass | `sim_plant_mass_scale` | 1.10 | plant flies 4.120787 kg; the controller believes 3.746170 |
| inertia | `sim_plant_inertia_scale` | 1.10 | body tensor ×1.10, model unchanged |
| base CoM | `sim_plant_com_shift_{x,y,z}` | 0.010 / 0.010 / 0.005 m | 0.40 N·m of standing moment at hover |
| thrust loss | `alloc_thrust_coeff` | 4.7544506e-05 | allocator believes kf/0.85 → 85% delivered |
| motor delay | `LaggedQuadraticThrustCurve` | λ = 10.0265 1/s | untouched, always on |
| arm back-EMF | `sim_arm_backemf_enable` | false | off for this campaign |

The plant is perturbed, not the controller's config — the mismatch is the
experiment. `1.0`/`0.0` is nominal, so every other rig is unaffected.

## Result

| run | posture PID | tune | outcome |
|---|---|---|---|
| A1 A2 A3 | on | shipped | **stable, 3/3**, 60 s each |
| B1 | off | shipped | diverged, 4.9 s |
| B2 | off | `D_y` 4→16 | diverged, 3.7 s |
| B3 | off | `ko_t` 0.5→2.0 | diverged, 3.8 s |

Numbers, per-run event logs: `metrics.json`. Scorer: `score_hover.py`.
Full write-up: `../Command.md` §7.14.8.

## Reproducing

```bash
# full step-0 clean between EVERY run — PX4 stays armed otherwise
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y
# then Command.md §7.14.1 steps 2 and 3, then the hover soak driver:
/usr/bin/python3 ../posture_removal_20260905/wb_direct_soak.py --soak 60 --out run.json
/usr/bin/python3 score_hover.py run.json=label
```
