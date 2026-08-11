# T650 sim-to-real, flight C — position, velocity and attitude on the controller-feedback topic

Comparison of `/uav_0/state_estimator/local_position/odom` between a recorded indoor T650
flight and an IsaacSim replay of the same reference sequence, run **2026-08-06**.

> **The simulated numbers below are the PRE-TUNING baseline** (bench constants, 2.95 kg
> total). `rotorcraft/t650_params.py` was subsequently fitted to this flight -- see
> [`TUNING_t650.md`](TUNING_t650.md) for the shipped parameters and the after numbers
> (yaw shape RMSE 4.109 deg -> 1.268 deg). The figures in `figures/C_*.png` show the TUNED
> simulation; `figures/C_tuning_before_after.png` shows both.

This supersedes [`REPORT.md`](REPORT.md) methodologically. That study (flights A and B) had
to work around bags that contained no measured position; **this bag records the controller
feedback topic**, so the position / velocity / attitude comparison the earlier X650 campaign
did is directly reproducible here, and the numbers below are directly comparable to
`fsc_autopilot_ros2/docs/sim_to_real_fidelity.md`.

- **Real:** `docs/experimental_data_ros2_bag/debug_recording_20260806_164742` — 211 s,
  21 078 odometry samples at a clean 100 Hz, 18 reference waypoints → **18 steps**.
- **Sim:** `scripts/indoor_sim/start_single_drone_t650.sh` (PX4 SITL + Pegasus/IsaacSim
  T650 plant, MN4010 + 15x5", 2.95 kg, lockstep on) plus
  `fsc_autopilot_ros2/scripts/isaacsim/start_baseline_t650_stack_fused.sh` — same controller
  build, same EKF2-fused estimator, same `params_single_vehicle_baseline_t650.yaml`.
- The 18 recorded waypoints were replayed into the live stack; `fsc_autopilot_ros2`
  regenerated its own attitude setpoints closed-loop.

---

## 1. Headline

**Translational fidelity is good and, on settling time, better than the X650 campaign
reported. Yaw remains the one axis the simulator gets materially wrong.**

| | real | sim | bias |
|---|---|---|---|
| Position-step trajectory RMSE (n=14) | — | — | **5.5 cm** (5.5% of a 1 m step) |
| Position-step overshoot [%] (n=16) | 12.93 | 15.66 | **+2.73 pts** |
| Position-step rise 10-90 [s] (n=14) | 1.864 | 1.625 | **−0.238 s (−12.8%)** |
| Position-step settling ±5% [s] (n=13) | 6.231 | 6.235 | **+0.004 s** |
| Yaw-step overshoot [%] (n=2) | 9.16 | 44.88 | **+35.7 pts** |
| Yaw-step rise 10-90 [s] (n=2) | 0.555 | 0.840 | +0.285 s (+51%) |
| Yaw-step RMSE (n=2) | — | — | **4.11°** (20% of a 20° step) |

Whole-run agreement on the feedback topic, resampled onto the real timeline
(−4.0 … 207.0 s, n = 21 078):

| quantity | RMS difference | bias | correlation |
|---|---|---|---|
| pos x / y / z | 3.4 / 4.5 / 6.4 cm | +0.1 / +0.0 / +0.7 cm | 0.998 / 0.996 / 0.976 |
| vel x / y / z | 3.7 / 5.6 / 3.6 cm/s | +0.2 / −0.2 / +0.1 cm/s | 0.963 / 0.917 / 0.899 |
| roll / pitch | 2.64° / 2.49° | **+2.58° / +2.46°** | 0.742 / 0.844 |
| yaw | 1.66° | −0.01° | 0.984 |

The roll and pitch rows are dominated almost entirely by a **constant** offset, not by
dynamic error — see §4.

---

## 2. Figures

| file | content |
|---|---|
| `figures/C_per_step.png` | one panel per reference step: the axis that stepped, real vs sim, reference as a dashed target |
| `figures/C_trajectory.png` | whole run in x, y, z and yaw, real vs sim vs reference |
| `figures/C_per_step_vel.png` | the same steps, in velocity along the stepping axis (yaw rate for yaw steps) |
| `figures/C_attitude.png` | whole run in roll, pitch and yaw |

Per-step and whole-run numbers: `data/odom_metrics_C_baseline.txt` (these numbers) and
`data/odom_metrics_C_tuned.txt` (after the parameter fit).

---

## 3. Step-by-step

**Position (x, y, z).** Sixteen steps: eight 1 m x/y steps in each direction, a diagonal
1 m×1 m pair out and back, and a four-step altitude sequence 1.0 → 2.0 → 0.7 → 0.5 → 0.25 m.
Shape RMSE is 5.5 cm mean / 5.7 cm median / 7.6 cm worst against 1 m steps — slightly better
than the X650 campaign's 6–7%, on a different airframe.

Two differences from that campaign are worth recording because they point the opposite way:

- **Rise time bias reverses sign.** X650 found sim consistently *slower* (+0.19 s, +12% at
  both gains, behaving like a fixed lag). Here sim is consistently **faster** (−0.24 s,
  −12.8%), on 14 of 14 paired steps. Same simulator, same controller family, opposite sign
  — so the X650 study's "fixed lag that calibrates out" is **not** a property of this
  simulator in general and should not be carried over as a correction factor.
- **Settling time now agrees.** X650 reported the settling bias as the metric sim got
  "wildly" wrong (+2.21 s / +1.08 s, and 3 of 12 sim steps never settled at all). Here the
  paired bias is **+0.004 s over 13 steps** — essentially exact. The likely reason is that
  this comparison uses the T650's shipped `posctl_k_vel = 3.0`, whereas that campaign was
  probing `k_vel` = 5 and 7, well into the lightly-damped regime where settling is most
  sensitive.

Overshoot is the one consistent translational error: sim overshoots ~2.7 points more, in the
same direction and roughly the same magnitude the X650 campaign found (+0.6 to +2.6 pts).

**Altitude steps** behave slightly differently from horizontal ones: sim *undershoots*
overshoot on the large climbs (step 14: 15.6% vs 20.5% real) and settles noticeably faster
(4.3–4.9 s vs 5.4–7.6 s). Small altitude steps are near the noise floor and steps 8 and 17
are excluded from aggregates — step 8's hold is only 1.4 s (a duplicate waypoint in the
recorded sequence truncates it) and step 17 is the final descent into ground effect, which
neither side lands cleanly.

**Yaw — unchanged verdict, worse magnitude.** Real yaw reaches the setpoint with ~9%
overshoot and settles in ~4 s. Sim overshoots to ~45%, rings with a ~4 s period and needs
~10 s. On a 20° step the shape RMSE is 4.11° — **20% of the commanded step**, against the
X650 campaign's 11–12%. Yaw authority comes from rotor drag torque rather than thrust, so
this points at `rolling_moment_coefficient` (8.247e-07 N·m/(rad/s)² for MN4010) or rotor
inertia, not at the controller: every thrust-driven axis agrees well and the thrust constant
is independently validated.

**Do not tune yaw gains in sim and carry them to hardware.** Third independent confirmation.

---

## 4. The persistent trim offset, confirmed a third time

Roll and pitch differ by a near-constant **+2.58° / +2.46°** (sim − real) across the whole
run, while their correlations stay at 0.74 / 0.84 — i.e. the transients line up, the datum
does not. `figures/C_attitude.png` shows this directly: two traces of the same shape, offset
vertically.

This reproduces the finding from flights A and B, where the two vehicles' hover trim was
measured at similar magnitude (0.29 vs 0.32 m/s² of horizontal specific acceleration) but
**161° apart in direction**. A vehicle holding position with a persistent tilt means the
attitude estimate's "level" is offset from the direction in which the thrust axis produces
no horizontal force — on hardware that is IMU/mocap levelling, in simulation it is the USD
asset's body frame versus its rotor plane.

It is a constant bias, not a dynamic error, and it does not affect any step metric above
(all of which are measured from each trace's own pre-step level). But ~2.5° of built-in tilt
in the simulated asset is large enough to matter for any study of trim, disturbance
estimation or UDE behaviour, and it is worth chasing separately.

---

## 5. Velocity

Velocity agreement is good but visibly noisier than position: RMS difference 3.6–5.6 cm/s
against real RMS of 7.7–12.6 cm/s, correlation 0.90–0.96. The correlation ordering
(x 0.963 > y 0.917 > z 0.899) is expected — velocity is the differentiated quantity in the
estimator, so it carries more of the sensor-noise difference between the two vehicles, and
the earlier study measured real airframe vibration at 25–45× the simulated level.

Nothing here contradicts the position result; it is the same agreement seen one derivative up.

---

## 6. Method and provenance

**Matched initial condition.** The real bag starts mid-flight. The simulation took off to
the sequence's first waypoint and settled before t = 0: `[-0.198, -0.186, 0.993]` against a
`[-0.18, -0.19, 1.00]` target — **2.0 cm** error.

**Matched timeline.** Reference switches were issued on PX4's clock (not wall clock — see
below); the achieved sequence spans **183.32 s against the recorded 183.30 s, a 23 ms error
over three minutes.**

**Real-time factor 0.898** for this run (0.839 / 0.840 in the earlier flights). The
simulator does not run in real time, so the waypoint schedule *must* be driven from
`sensor_combined.timestamp` rather than wall clock; scheduling on wall clock compresses the
sequence in simulation time by exactly this factor and silently shortens every hold. See
`tools/stack_driver.py::sim_now`.

**Timing traps handled** (all three cost the earlier campaigns real time):

1. **rosbag2 receive timestamps are bursty.** Measured on this bag: odometry `header.stamp`
   dt is a clean 10.00 ms (p1 9.97, p99 10.03), while receive dt reaches **50.8 ms**.
   All real-side traces use `header.stamp`.
2. **The reference topic's header stamps are on a different clock epoch** from odometry's.
   Reference onsets are recovered by mapping the reference's receive time onto the odometry
   timeline through odometry's own receive → header correspondence.
3. **In simulation the recorded topics span two clocks** — PX4-stamped telemetry versus
   ROS-wall-clock controller output. The odometry record carries both per sample, so they
   can be mapped; an affine fit is not adequate because the ratio varies through the run.

**Reproducing:**

```bash
tools/extract_odom_bag.py <bag_dir> exp_C.npz      # needs ROS 2 + px4_msgs sourced
tools/run_stack_case.sh ref_C.npz sim_stack_C.npz stackC
tools/plot_odom.py C sim_stack_C.npz figures       # system python3, ROS not sourced
tools/metrics_odom.py C
```

`run_stack_case.sh` **auto-arms** and is simulation-only. Run it under `tmux`, not
`nohup`/`setsid`. Do not `cp -a` the PX4 `rootfs` (39 GB of accumulated logs).

## 7. Limitations

- **One run per side.** No run-to-run spread, so none of the biases carry an error bar.
- **Only two yaw steps**, both ±20°. The yaw numbers are a strong qualitative result but a
  weak quantitative one.
- **The real flight's controller gains are not recorded in the bag.** The current
  `params_single_vehicle_baseline_t650.yaml` (`posctl_k_vel = 3.0`, `k_pos = 1.0`) was
  assumed. If that flight used different gains, §3's numbers shift — and note that the
  rise-time and settling comparisons against the X650 campaign are cross-gain as well as
  cross-airframe.
- **Battery sag is present in this flight too** but is not quantified here; the flight A/B
  report measures it directly (+0.80 %-pts of hover throttle over 95 s, absent in sim) and
  it remains the dominant unmodelled effect.
- Steps 8 and 17 are excluded from aggregates for the reasons given in §3.
