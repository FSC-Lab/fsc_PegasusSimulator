# T650 sim-to-real: IsaacSim vs two indoor flights, same step commands

> **STALE SIMULATION DATA.** Flights A and B were simulated with the pre-tuning parameters
> and the older 2.95 kg *total* mass. The `sensor_combined` vibration and battery-sag
> conclusions still hold (neither depends on those constants), but the step numbers do not
> correspond to the shipped parameter set -- see [`TUNING_t650.md`](TUNING_t650.md).
>
> **See also [`REPORT_flightC.md`](REPORT_flightC.md).** A third flight
> (`debug_recording_20260806_164742`) records
> `/uav_0/state_estimator/local_position/odom`, so it supports the direct position /
> velocity / attitude comparison this report had to work around. Read that one first for
> trajectory-level fidelity; this report remains the reference for `sensor_combined`,
> vibration and battery sag, which flight C does not cover.

Comparison of `sensor_combined` and the baseline controller's step responses between two
recorded indoor T650 flights and IsaacSim replays of the same reference sequences, run
**2026-08-06**.

- **Real:** `docs/experimental_data_ros2_bag/debug_recording_20260806_134620` (flight A,
  117 s) and `.../debug_recording_20260806_140303` (flight B, 169 s).
- **Sim:** `scripts/indoor_sim/start_single_drone_t650.sh` (PX4 SITL + Pegasus/IsaacSim
  T650 plant, MN4010 + 15x5", 2.95 kg, lockstep on) driven by the real ROS 2 stack,
  `fsc_autopilot_ros2/scripts/isaacsim/start_baseline_t650_stack_fused.sh` — same
  controller build, same EKF2-fused estimator, same `params_single_vehicle_baseline_t650.yaml`.
- **Closed loop.** The recorded `position_controller/reference` waypoints were replayed
  into the live stack and `fsc_autopilot_ros2` regenerated its own attitude setpoints. The
  real flights' attitude setpoints were **not** injected — they are the thing being predicted.
- Tooling, figures and simulated data: `tools/`, `figures/`, `data/` beside this file.

---

## 1. Headline

For **translational** step response the simulator reproduces the T650's closed-loop
behaviour well in shape but is consistently **softer and slower**: it commands ~23% more
peak horizontal acceleration and takes ~1.9x as long to build it. For **yaw** it is
materially wrong — 1.7x the rise time and 1.3x the peak, with a ringing that hardware does
not show. For **vibration** it is not a model of the real vehicle at all: real airframe
vibration above 2 Hz is **25-45x** larger than simulated.

| | real | sim | bias |
|---|---|---|---|
| Horizontal step, peak \|a_h\| [m/s²] (n=13) | 0.649 | 0.796 | **+22.6%** |
| Horizontal step, rise 10-90% [s] (n=13) | 0.159 | 0.307 | **+92.7%** |
| Horizontal step, \|undershoot\| [m/s²] (n=13) | 0.360 | 0.491 | +36.6% |
| Horizontal step, shape RMSE [m/s²] (n=13) | — | — | 0.095 (≈15% of peak) |
| Yaw step, \|peak\| [deg] (n=4) | 21.79 | 29.01 | **+33.1%** |
| Yaw step, rise 10-90% [s] (n=4) | 0.713 | 1.245 | **+74.7%** |
| Hover thrust command | 0.5118 ± 0.0029 | **0.5032 ± 0.0001** | −0.9 %-pts |
| Hover thrust drift over 95 s | **+0.80 %-pts** | −0.01 %-pts | battery sag, unmodelled |
| Accelerometer RMS > 2 Hz [m/s²] | 1.45–1.65 | 0.036 | **sim is 3% of real** |

**Claims worth making:** the thrust/mass calibration is right; horizontal step *shape* is
reproduced to ~15% of peak; battery sag is now directly demonstrated rather than
hypothesised.
**Claims to avoid:** that sim predicts yaw, that sim predicts vibration or anything derived
from it, or that one run per flight characterises the hardware.

---

## 2. What could and could not be compared

**Neither bag contains measured position or velocity.** The six recorded topics are
`fmu/out/{sensor_combined, vehicle_attitude, vehicle_status_v1}`,
`fmu/in/vehicle_attitude_setpoint`, `fsc_autopilot_ros2/attitude_setpoint_debug` and
`fsc_autopilot_ros2/position_controller/reference`. `vehicle_local_position` and
`vehicle_odometry` were not recorded, so the position step response plotted in the earlier
X650 campaign **cannot be reproduced for these flights**.

Each per-step panel therefore shows the quantity that *is* recorded on both sides and that
a position step actually commands:

| step type | quantity plotted | why |
|---|---|---|
| x / y | horizontal specific acceleration projected on the step direction, `a_h = −g·(R₀₂,R₁₂)/R₂₂` | this is what a horizontal position step commands; heading-independent; available from the attitude setpoint (commanded) and `vehicle_attitude` (achieved) |
| z | normalized thrust command | the real vertical accelerometer is vibration-dominated (§5) and buries a 0.3–0.5 m step entirely |
| yaw | yaw angle | directly recorded on both sides |

Simulated position is shown only in the full-run figures, labelled sim-only.

Flight A flew a 1 m hover, a 2x2 m square, a 20° yaw step, an x step and a descent;
flight B an altitude staircase 0.7→1.0→1.5→2.0→1.0 m, then yaw and x steps and a descent.
12 and 11 discrete steps respectively.

---

## 3. Figures

| file | content |
|---|---|
| `figures/A_per_step.png`, `figures/B_per_step.png` | one panel per reference step, real vs sim, commanded and achieved, pre-step level removed |
| `figures/A_full_run.png`, `figures/B_full_run.png` | roll / pitch / yaw / thrust command over the whole run, plus simulated position against the reference |
| `figures/A_sensor_steps.png`, `figures/B_sensor_steps.png` | `sensor_combined` gyro magnitude per step, raw and filtered |

The thrust panel of the full-run figures is the clearest single result in this study: the
real command climbs monotonically 0.489 → 0.519 across the flight while the simulated one
sits flat at 0.5032.

---

## 4. Step-response fidelity

Full per-step table: `data/step_metrics.txt`. All metrics are computed on the
baseline-removed response and anchored on each signal's own 10% crossing, so the persistent
hover trim (§6) and any residual onset jitter drop out — the same choice the X650 campaign
made.

**Horizontal (13 steps, both flights).** Response *shape* agrees well: RMSE 0.095 m/s²
against a typical peak of 0.65 m/s², i.e. ~15%. The two systematic biases are:

- **Peak +22.6%.** Sim commands more acceleration for the same step.
- **Rise 10-90% +92.7%** (0.159 s → 0.307 s). Sim takes nearly twice as long to build tilt.
  This is an inner-loop/plant property, not a position-loop one: the simulated vehicle
  carries the bench-measured rotor lag (λ = 10.0265 s⁻¹, τ ≈ 100 ms) *and* the softened PX4
  gains (`MC_*RATE_K = 0.3`, `MC_ROLL_P/MC_PITCH_P = 3.25`) that were introduced to keep
  that lag stable. Together those cost roughly the factor observed. The real vehicle builds
  tilt faster than the model says it should.

These two biases partly cancel in position terms — a larger but slower acceleration command
integrates to a similar trajectory — which is consistent with the X650 campaign finding
position-step RMSE of only 6–7% while this study finds a much larger discrepancy in the
acceleration that produces it. **Sim agreement at the trajectory level is partly the result
of two errors compensating**, so it should not be read as inner-loop fidelity.

**Vertical (6 steps).** Peak thrust deviation agrees to −4.2%, but the rise metric is not
trustworthy: a 0.3–0.5 m altitude step moves the normalized thrust command by only ~0.01,
and on hardware that is close to the noise floor. Treat the z rows in the table as
indicative only.

**Yaw (4 steps) — the one axis sim gets materially wrong.**

| | real | sim | ratio |
|---|---|---|---|
| \|peak\| on a 20° step [deg] | 21.79 | 29.01 | 1.33x |
| rise 10-90 [s] | 0.713 | 1.245 | 1.75x |
| shape RMSE [deg] | — | — | 3.70 (18% of step) |

Hardware reaches the setpoint in ~1 s with ~9% overshoot and settles. Sim takes ~2.3 s to
peak, overshoots to ~45%, and rings with a ~4 s period still visible 6 s later. This
**reproduces the X650 campaign's §5 finding independently on a different airframe and motor
set** (there: 1.8x slower rise, 1.6x overshoot). Yaw authority comes from rotor drag torque,
not thrust, so this points at the sim's yaw-torque coefficient (`rolling_moment_coefficient`
= 8.247e-07 for MN4010) or rotor inertia rather than at the controller — the thrust-driven
axes agree well and the thrust constant is independently validated (§6).

**Do not tune yaw gains in sim and carry them to hardware.**

---

## 5. `sensor_combined`

Full table: `data/sensor_metrics.txt`. Split at 2 Hz because the two bands answer different
questions.

**Below 2 Hz — rigid-body motion. Good agreement.**

| channel | real RMS | sim RMS | sim/real |
|---|---|---|---|
| gyro_x | 0.0246 | 0.0354 | 1.44 |
| gyro_y | 0.0333 | 0.0375 | 1.13 |
| gyro_z | 0.0563 | 0.0676 | 1.20 |
| accel_z (std) | 0.0563 | 0.0499 | 0.89 |

(flight A; flight B is within the same range). Mean specific force matches almost exactly —
real a_z −9.792 / −9.796 m/s², sim −9.798 / −9.796 — confirming the thrust and mass model.
Simulated gyro is 13–44% *higher* below 2 Hz, consistent with the larger commanded
accelerations of §4.

**Above 2 Hz — vibration. Not modelled.**

| channel | real RMS | sim RMS | sim/real |
|---|---|---|---|
| accel_x | 1.654 | 0.036 | **0.02** |
| accel_y | 1.286 | 0.035 | **0.03** |
| accel_z | 0.570 | 0.037 | **0.06** |
| gyro_x | 0.0400 | 0.0064 | 0.16 |
| gyro_y | 0.0359 | 0.0087 | 0.24 |

The real airframe carries 1.3–1.7 m/s² RMS of broadband accelerometer vibration; the
simulated one carries 0.036 m/s², which is the configured IMU noise and nothing else. The
simulated rotors are rigid, perfectly balanced, and the thrust model
(`LaggedQuadraticThrustCurve`) produces a smooth force with no blade-passing content. Any
result that depends on IMU noise — filter tuning (`IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF`),
notch placement, vibration-triggered failsafes, EKF innovation gating — **cannot be
developed in this simulator**. Note also that this is why §4 uses the thrust command rather
than measured `a_z` for vertical steps.

---

## 6. Two findings the flights settle

### Battery sag is real, and it is the dominant unmodelled effect

The hover thrust command over the same maneuver:

| | mean | drift over 95 s | spread |
|---|---|---|---|
| real | 0.5118 | **+0.80 %-pts** | ±0.0029 |
| sim | 0.5032 | −0.01 %-pts | ±0.0001 |

Same controller, same mass in config, same maneuver, same duration. The real vehicle needs
progressively more throttle for the same hover; the simulated one does not drift at all.
The X650 campaign listed battery sag as "a hypothesis, not a result" for its observed
loss of damping over a session — **these two flights are direct evidence for it**, measured
on the actuator command rather than inferred from overshoot.

Practical consequence: a hover command recorded early in a flight is not the same as one
recorded late. Flight B, being longer, drifts further still — 0.5033 over its first 5 s
against 0.5358 over its last 20 s, **+3.3 %-pts end to end**.

### The thrust calibration is confirmed a third time

Sim hover command **0.5032 ± 0.0001** against the T650 calibration's predicted
`T650_HOVER_COMMAND = 0.503188` — agreement to 4 decimal places. The *real* command agrees
too once sag is excluded: over the first 5 s of flight B it averages **0.5033**, and over
the first 5 s of flight A 0.5018. This is now the third and fourth independent
confirmation, after the QGC/PX4 mixer takeoff (0.503341) and the `fsc_autopilot_ros2`
baseline hover (0.50320) recorded in `CLAUDE.md` — and the first from real hardware.

Note the controller's own config implies a different hover point:
`vehicle_thrust_scaling = 0.038545`, `vehicle_idle_thrust = 0.218657`, `vehicle_mass = 2.9`
give `0.038545·(2.9·9.81)/4 + 0.218657 ≈ 0.4926`, about 1 %-pt below the plant's true
0.5032. The velocity-based UDE absorbs the difference, which is why hover works, but the
feedforward is carrying a known offset.

### Both vehicles hover tilted — in opposite directions

| | roll | pitch | trim \|a_h\| |
|---|---|---|---|
| real | −1.309° ± 0.124 | +1.072° ± 0.262 | 0.286 m/s² |
| sim | +0.977° ± 0.310 | −1.584° ± 0.405 | 0.318 m/s² |

Similar magnitude, **161° apart in direction**. A vehicle holding position with a persistent
tilt means the attitude estimate's "level" is offset from the direction in which the thrust
axis produces no horizontal force — on hardware that is IMU/mocap levelling, in sim it is the
USD asset's body frame versus its rotor plane. It is a constant bias, not a dynamic error,
which is why every metric in §4 is baseline-removed. It is worth chasing separately: ~1.8° of
built-in tilt in the simulated asset is large enough to matter for any study of trim,
disturbance estimation or UDE behaviour.

---

## 7. Method and provenance

**Matched initial condition.** The real bags start mid-flight, already hovering. The
simulation took off to the sequence's first waypoint and settled before t = 0: flight A
settled at `[-0.025, 0.011, 0.994]` against a `[0, 0, 1]` target (2.8 cm error).

**Matched timeline.** Reference switches were issued on PX4's clock, and the achieved
sequence spans match the recorded ones to **0.02 s over 90 s** (flight A) and **0.01 s over
147 s** (flight B).

**Two timing traps, both of which silently corrupt results:**

1. **The simulator does not run in real time** — measured 0.839x (A) and 0.840x (B). The
   first attempt scheduled the reference sequence on wall clock, which compressed it to 77%
   of its intended duration *in simulation time*, shortening every hold by 23%. Those runs
   were discarded. `stack_driver.py` now drives the schedule from
   `sensor_combined.timestamp` (see its `sim_now`).
2. **In simulation the recorded topics are on two different clocks.** `sensor_combined` and
   `vehicle_attitude` are stamped by PX4 (simulation time); the attitude setpoints are
   stamped by the controller on ROS wall clock. Mixing them misplaces every commanded trace
   by tens of seconds. `plot_steps.load_side` maps wall → simulation time by interpolating
   the clock pairs recorded with each odometry sample, validated against the independent
   pairs in the reference log. An affine fit is *not* adequate (59 ms RMS) because the ratio
   varies through the run. On hardware this does not arise — PX4 and ROS share a clock there.

Also confirmed from the earlier campaign's trap list: rosbag2 receive timestamps are bursty
and unusable for step onsets, and the reference topic's header stamps are on a different
epoch from the PX4 topics. Onsets are recovered by fitting receive-time → PX4-time on the
attitude-setpoint stream (which carries both) and applying it to the reference messages;
residual 3.0 ms (A) and 4.4 ms (B).

**Reproducing:**

```bash
# 1. extract the real bags and the reference sequences (needs ROS 2 + px4_msgs sourced)
tools/extract_bag.py <bag_dir> exp_A.npz
tools/extract_refs_npz.py <scratch_dir>

# 2. one closed-loop simulation case (brings up agent, PX4+Isaac, the stack, drives it)
tools/run_stack_case.sh ref_A.npz sim_stack_A.npz stackA

# 3. figures and metrics (system python3; do NOT source ROS 2 for matplotlib on some hosts)
tools/plot_steps.py A sim_stack_A.npz figures
tools/metrics.py ; tools/sensor_compare.py
```

`run_stack_case.sh` **auto-arms**. It is simulation-only, as is `virtual_remote`, whose
`/uav_0/rc/{arm,offboard}` services it calls. Note that the safety comment in
`start_baseline_t650_stack_fused.sh:31-34` describing a "press Enter" gate in
`_wait_for_arm_trigger()` is stale — no such function exists; the services arm whenever
anyone calls them, with no human gate.

Run the runner under `tmux`, not `nohup`/`setsid` — a killed tool call otherwise takes the
whole process group with it. Do not `cp -a` the PX4 `rootfs` (39 GB of accumulated logs).

## 8. Limitations

- **One run per flight, on both sides.** No run-to-run spread, so none of the biases above
  carry an error bar. Given battery sag, hardware spread is probably not small.
- **The real flights' controller gains are not recorded in the bags.** The current
  `params_single_vehicle_baseline_t650.yaml` was assumed. If those flights used different
  gains, the §4 numbers shift.
- **No measured position on the real side**, so this study cannot report the trajectory-level
  RMSE / overshoot / settling that the X650 campaign reported. Record
  `state_estimator/local_position/odom` or `fmu/out/vehicle_odometry` next time and the two
  campaigns become directly comparable.
- The final descent step is not meaningful on either side (ground effect, land detector) and
  is excluded from the discussion, though it remains in the per-step table.
- An earlier open-loop variant — replaying the recorded attitude setpoints directly into PX4
  rather than regenerating them — was built and validated end to end, but is not reported
  here: without position feedback the vehicle diverges by design (151 m of altitude and
  421 m laterally over flight A). It remains the right tool for isolating plant response
  from controller response if that is ever wanted.
