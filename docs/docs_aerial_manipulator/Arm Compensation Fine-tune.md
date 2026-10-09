# Arm Compensation Fine-tune

**Status (2026-10-09): prepared, NOT applied.** Every file below still holds the current settings.
These are the settings the 0928 and 1005 hardware flights used, and the first pick-and-place flight uses them too.

This document covers one candidate change to the whole-body rig's arm compensation. It records:

- the current settings, so they can be restored;
- how to apply the change;
- how to test it;
- how to revert it.

## 1. The change in one line

In the hardware arm controller config, change the friction feed-forward's trigger velocity from the
joint's **measured** velocity to the **reference** velocity:

```yaml
friction_velocity_source: measured   # current
friction_velocity_source: reference  # proposed
```

Nothing else changes:

- the friction levels stay at the in-flight correction, ×0.70 on j2 and ×0.65 on j3;
- the whole-body law, the L1 observer, the flight-stack yaml and the planner stay untouched.

| | |
|---|---|
| Rig | Whole-body only, with the arm in torque mode. The decoupled rig's position-mode arm has no friction feed-forward, so it is unaffected. |
| Expected effect | Less stick-slip while the arm moves, which is most of the q3 error in the paper's Figs. 9 and 11. |
| Not affected | The 1.5–2.4° offset the arm holds when it stops. No friction term acts at zero velocity. |
| Applies to | Free flight and pick-and-place alike, since both use the same arm config. |

## 2. Why

The friction feed-forward on each joint is

```
tau_ff = (friction_ff + friction_load_coeff * |load torque|) * tanh(v / width)
```

The `v` and `width` depend on the source:

| source | v | width |
|---|---|---|
| `reference` | the planner's reference joint velocity | `friction_ff_width` = 0.015 rad/s |
| `measured` | the arm's velocity observer | `friction_measured_width` = 0.03 rad/s |

On 2026-09-26 two things changed in one step, and the 09-28 flights were the first to fly both:

1. The friction levels were scaled down to what the 0924 flights measured, ×0.70 on j2 and ×0.65 on j3.
2. The trigger moved from the reference to the measured velocity.

The same flights also changed the law's gains, armature and joint-velocity source. So the trigger change has never been tested on hardware by itself.

The offline arm model scores each option at the current hardware law (K_y 211.9 / D_y 26.82, joint-diagonal armature, observer velocity). It says the scaling was the fix and the trigger switch was the cost:

| friction feed-forward | q2 / q3 error while moving, deg | share of moving time stuck |
|---|---|---|
| **measured trigger, ×0.70/0.65 (current)** | **0.70 / 0.66** | 36 / 40 % |
| measured trigger, ×1.0 | 0.95 / 0.96 | 54 / 53 % |
| measured trigger, ×0.50/0.45 | 0.83 / 0.88 | 39 / 42 % |
| measured trigger, ×0.35/0.33 | 0.90 / 1.02 | 37 / 42 % |
| measured trigger, width 0.015 or 0.06 | 0.68 / 0.64 or 0.73 / 0.74 | 40–47 % |
| measured + 0.6 × reference, needs a code change | 0.35 / 0.31 | 31 / 32 % |
| reference trigger, ×1.0 (flown 09-18 to 09-24, older law) | 0.35 / 0.75 | 20 / 19 % |
| **reference trigger, ×0.70/0.65 (proposed)** | **0.23 / 0.16** | 25 / 27 % |

The measured trigger is zero while a joint is stuck, so nothing pushes it through the stiction when the reference starts moving. The joint waits for the task spring to wind up, then jumps.

The reference trigger was abandoned on 09-26 because at ×1.0 it was 30–35 % above the real friction. At the corrected level that reason is gone.

**Limits of this evidence:**

- The offline model only ranks options. It underestimates the flown joint error 3 to 5 times.
- It reproduces only about 0.2° of the flown 1.5–2.4° hold offset.
- It does not model the arm's 100 Hz dither, which is active on hardware while the reference moves.
- Only a hardware A/B settles it.

Sources:

- `archive/pick_place_top_hat_20261008/README.md`, section "Joint tracking on the 0928 / 1005 flights".
- `archive/pick_place_top_hat_20261008/analysis/ff_sweep.txt` and `ff_sweep_extra.txt`.
- The model: `archive/arm_armature_20260926/arm_stiffness_sim.py`.
- To re-run: `PYTHONNOUSERSITE=1 /usr/bin/python3 archive/pick_place_top_hat_20261008/tools/ff_sweep.py ref_hw meas_hw`.

## 3. Current settings (the baseline to restore)

Baseline commits, all pushed 2026-10-09:

- `fsc_open_manipulator` `omx-torque-control` **6c5ef48**;
- `fsc_autopilot_ros2` `dev_robotic_arm` **c837599**.

### 3.1 Hardware arm controller

File: `fsc_open_manipulator/open_manipulator_x_custom_controller/config/external_torque_controller_hardware_aerial_pwm.yaml`.
On the Orin it lives under `~/dev_ws/src/`. Section `/**/external_torque_controller`. Line numbers are as of 6c5ef48.

| line | key | current value | changes? |
|---|---|---|---|
| 542 | `friction_ff` | `[2.9, 3.294, 5.055, 7.7776]` duty counts | no |
| 554 | `friction_load_coeff` | `[0.0, 0.172, 0.105, 0.0]` | no |
| **565** | **`friction_velocity_source`** | **`measured`** | **yes → `reference`** |
| 566 | `friction_measured_width` | `0.03` rad/s | no; unused once the source is `reference` |
| 570 | `friction_ff_width` | `0.015` rad/s | no; becomes the active width |
| 573 | `viscous_ff` | `[0.0, 0.0, 0.0, 0.0]` | no |
| 580 | `dither_amplitude` | `[0.0, 48.0098, 19.2039, 14.4029]` duty counts | no |
| 581–582 | `dither_frequency_hz`, `dither_v_gate` | `100.0`, `0.05` | no |
| 586 | `stiction_ramp_rate` | `[0.0, 0.0, 0.0, 0.0]` (off) | no |
| 598–599 | `velocity_source`, `velocity_observer_bandwidth_hz` | `observer`, `20.0` | no |
| 607 | `reference_lead_s` | `0.006` | no |
| 420 | `current_loop_bandwidth_hz_joints` | `[0.0, 1.5, 1.5, 0.2]` | no |
| 657 | `passthrough_auxiliary_terms` | `true` | no |
| 667 | `passthrough_gravity_correction` | `true` | no |
| 674 | `passthrough_integral` | `false` | no |

### 3.2 Flight-stack arm-side check

The two whole-body hardware stack scripts check the arm yaml before launch. The check is non-fatal.

Files:

- `fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh`
- `fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator_fused.sh`

Both hold this check, at lines 716–717 in the raw script and 757–758 in the fused one:

```bash
  arm_kv_ok "friction_velocity_source" "measured" \
            "friction_velocity_source = measured (relay on the joint's own velocity)"
```

### 3.3 Isaac arm controller (only for a simulation A/B)

File: `fsc_open_manipulator/open_manipulator_x_isaac_bridge/config/torque_controller_isaac_aerial.yaml`, line 165:
`friction_velocity_source: measured`. It uses the same ×0.70/0.65 levels in N·m and `friction_ff_width` 0.015, with dither 0.

### 3.4 Unchanged whole-body law (for reference)

In both the free-flight and pick-and-place hardware yamls:

- `wb_ky_x/y/z` 211.9 and `wb_dy_x/y/z` 26.82;
- `wb_ky_psi` 0.2484 and `wb_dy_psi` 0.2903;
- `wb_l1_omega_c_q` 0.8479 and `wb_l1_omega_x` 0.2072;
- `wb_armature_joint_diag` true with `[0.010, 0.0194, 0.0097, 0.0097]`;
- `wb_arm_velocity_topic` = the arm's `velocity_observer`;
- `wb_tau_max` 3.0.

### 3.5 Baseline numbers to beat

Paper experiment table, whole-body rows, joint RMSE in degrees (`results/utils/tables/free_flight_tracking_exp.tex`):

| run | q1 | q2 | q3 | q4 |
|---|---|---|---|---|
| circle 0.13 m/s (WB-1, 09-28) | 0.53 | 1.68 | 1.42 | 0.16 |
| figure-8 0.10 m/s (WB-3, 10-05) | 0.67 | 1.66 | 1.33 | 0.24 |
| figure-8 0.13 m/s (WB-5, 10-05) | 0.70 | 1.77 | 1.47 | 0.36 |

For comparison, the decoupled rig's position-mode arm reaches 0.75–1.12° on q2 and 0.48–0.82° on q3.

Stick statistics for the circle runs, from `archive/wb_vs_decoupled_flight_20260928/analysis/arm_stats.txt`:

| run | joint | rms err | hold offset | stuck | burst speed vs reference, deg/s | overshoot |
|---|---|---|---|---|---|---|
| WB-1 | j2 | 2.04 | −2.38 | 13 % | 26.7 vs 15.8 | −1.2 / +2.1 |
| WB-1 | j3 | 1.42 | +2.43 | 27 % | 33.8 vs 15.6 | +0.1 / +1.3 |
| WB-2 | j2 | 1.69 | −1.45 | 9 % | 26.8 vs 15.8 | +0.1 / +0.9 |
| WB-2 | j3 | 1.44 | +1.82 | 26 % | 33.8 vs 15.6 | −0.3 / +1.6 |

The rms error here uses a different window from the paper table.

## 4. Applying the change (on the Orin)

Do this on a day planned as an A/B test, not alongside another first-time test.

1. **Arm yaml.** Edit line 565 of the hardware arm controller yaml. Change only that line; leave the comment block above it.
   ```bash
   cd ~/dev_ws/src/fsc_open_manipulator/open_manipulator_x_custom_controller/config
   sed -i 's/^    friction_velocity_source: measured$/    friction_velocity_source: reference/' \
       external_torque_controller_hardware_aerial_pwm.yaml
   grep -n "^    friction_velocity_source" external_torque_controller_hardware_aerial_pwm.yaml
   # expect: 565:    friction_velocity_source: reference
   ```
2. **Stack checks.** In both whole-body hardware stack scripts from section 3.2, change the expected value. Otherwise every launch prints a red `MISMATCH friction_velocity_source` line.
   ```bash
   cd ~/dev_ws/src/fsc_autopilot_ros2/scripts/indoor_exp
   for f in start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh \
            start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator_fused.sh; do
     sed -i 's/arm_kv_ok "friction_velocity_source" "measured" \\/arm_kv_ok "friction_velocity_source" "reference" \\/; s/"friction_velocity_source = measured (relay on the joint.s own velocity)"/"friction_velocity_source = reference (A\/B 2026-10, see Arm Compensation Fine-tune.md)"/' "$f"
     grep -n -A1 'arm_kv_ok "friction_velocity_source"' "$f"
   done
   ```
3. **Rebuild.** The yaml is installed into the package share directory, so rebuild the arm package. This is harmless if the workspace uses symlink-install.
   ```bash
   cd ~/dev_ws && colcon build --packages-select open_manipulator_x_custom_controller <the workspace's usual flags>
   ```
4. **Restart the arm stack.** The controller reads the source once at configure, so a live `ros2 param set` does nothing.
   ```bash
   ~/dev_ws/src/fsc_open_manipulator/scripts/indoor_exp/stop_open_manipulator_stack.sh
   ~/dev_ws/src/fsc_open_manipulator/scripts/indoor_exp/start_open_manipulator_inverted_wb_torque.sh uav_0
   ```
5. **Verify**, with props off:
   - Read the parameter back:
     ```bash
     ros2 param get /uav_0/fsc_open_manipulator/external_torque_controller friction_velocity_source
     ```
     It should read `reference`. Find the node with `ros2 node list` if the path differs.
   - With `measured`, the controller logs this WARN at startup:
     ```
     Friction feed-forward on the MEASURED velocity (blend width 0.030 rad/s) ...
     ```
     With `reference` that line must be absent.
   - The flight stack's arm-side check prints `OK   friction_velocity_source = reference ...`.
     The `friction_measured_width` check still prints OK and no longer matters.

For a simulation A/B first, change line 165 of the Isaac yaml from section 3.3 the same way. Rebuild `open_manipulator_x_isaac_bridge` and relaunch the whole-body sim. Isaac's arm is optimistic: it realised about 105 % of the joint span where hardware realised 44–100 %. So a sim result can rule the change out but cannot confirm it.

## 5. Test plan (hardware A/B)

**Mission:** repeat a paper run exactly, so the baseline in 3.5 applies. Two choices:

- the circle at 0.13 m/s;
- the figure-8, A 0.70 / B 0.35 m, at 0.10 m/s.

Keep everything else as flown: whole-body 4-D L1, `AM_HW_PROFILE=free_flight`, fused stack, and q2 25 ± 15° with four cycles per lap at a 55° fold.

Fly a same-day baseline run with `measured` first if the battery budget allows. Day-to-day spread on this rig is about 20 % on q2, as WB-1 against WB-2 shows.

**Watch while flying:**

- **Overshoot past the reference turning points.** This is how the old reference trigger failed, by pushing through every stick.
- **q3 near its +50° stop, or q2 near its +45° guard.**
- **Arm torque near the caps**, 2.44 N·m on j2 and 1.42 N·m on j3.
- **Any new tilt oscillation.**

At a hold the reference-triggered term is exactly zero, so it cannot chatter there.

**Abort to SAFETY if:**

- q3 comes within 2° of +50°;
- a joint repeatedly overshoots by more than 3°;
- the arm hums or rings.

**Scoring:** use the paper's definitions.

1. Add the run to `RUNS` in `results/utils/experiment_tracking.py` with `eligible=False` and a note saying `arm friction source = reference (A/B)`. That keeps the paper table unchanged.
2. Run `/usr/bin/python3 results/utils/experiment_tracking.py` without `--paper`. Every listed run's q1–q4 RMSE lands in `$AM_EXP_CACHE/scores.json`, `/tmp/am_experiment_cache` by default.
3. For stuck share, burst speed and overshoot, extract the bag with `archive/wb_vs_decoupled_flight_20260928/tools/extract_bag.py`. Add it to that folder's `common.py` run map and run `tools/arm_stats.py <name>`.

**Adopt** the change only if all of these hold:

- q2 and q3 RMSE both drop by at least 20 % against the same mission's baseline;
- stuck share and burst speed drop;
- no stop or guard contact occurs;
- EE and platform errors do not get worse.

If it is adopted:

1. Update both the hardware and Isaac yamls.
2. Make the section-4 edit to the stack checks permanent.
3. Re-fly the paper runs if the paper table should use it.

If the reference trigger overshoots, lower the friction levels next. Measure them from that flight's data rather than guessing.

## 6. Reverting to the current settings

1. Arm yaml back to `measured`:
   ```bash
   cd ~/dev_ws/src/fsc_open_manipulator/open_manipulator_x_custom_controller/config
   sed -i 's/^    friction_velocity_source: reference$/    friction_velocity_source: measured/' \
       external_torque_controller_hardware_aerial_pwm.yaml
   ```
   If the file has no other local edits, `git checkout -- external_torque_controller_hardware_aerial_pwm.yaml` does the same. Check first with `git diff`, since checkout discards everything.
2. Stack checks back to `measured`, in both scripts. This is the exact inverse of section 4 step 2:
   ```bash
   cd ~/dev_ws/src/fsc_autopilot_ros2/scripts/indoor_exp
   for f in start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh \
            start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator_fused.sh; do
     sed -i 's/arm_kv_ok "friction_velocity_source" "reference" \\/arm_kv_ok "friction_velocity_source" "measured" \\/; s/"friction_velocity_source = reference (A\/B 2026-10, see Arm Compensation Fine-tune.md)"/"friction_velocity_source = measured (relay on the joint\x27s own velocity)"/' "$f"
     grep -n -A1 'arm_kv_ok "friction_velocity_source"' "$f"
   done
   ```
   If nothing else in the scripts was edited, `git checkout --` on the two files does the same.
3. Rebuild `open_manipulator_x_custom_controller` and restart the arm stack, as in steps 3 and 4 of section 4.
4. Verify with `ros2 param get ... friction_velocity_source`, which should read `measured`. The startup WARN `Friction feed-forward on the MEASURED velocity (blend width 0.030 rad/s)` is back, and the stack check prints `OK friction_velocity_source = measured`.

If the change was committed and pushed in between, `git revert <that commit>` in each repo returns to the state at 6c5ef48 / c837599.

## 7. Considered and not proposed

| option | why not |
|---|---|
| A smaller friction level on the measured trigger | Worse offline at both ×0.50 and ×0.35. The trigger's timing is the problem, not its size. |
| Changing `friction_measured_width` | Within a few percent offline at both 0.015 and 0.06. |
| Measured + reference blend | Needs a `friction_velocity_source: blend` code change, and ranks below the plain reference trigger offline. |
| Arm-side integral `passthrough_integral: true` against the hold offset | Off on purpose: the L1 observer's arm channel already integrates the joint residual, and a second integrator would corrupt its estimate. The paper's law has no joint-space term. |
| Stiffer EE task `wb_ky_*` against the hold offset | 211.9 is the 2026-09-27 tuned value at its stability margin. |

The hold offset is harmless for pick-and-place, because the descent trim measures the claw.
