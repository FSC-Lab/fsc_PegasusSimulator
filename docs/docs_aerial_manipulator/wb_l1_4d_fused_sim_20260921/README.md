# Whole-body L1 4-D rig with EKF2-FUSED feedback — simulation check (2026-09-21)

Question: does the hardware workflow
`fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator_fused.sh`
(controller position/velocity from PX4 EKF2's fused odometry instead of the raw
mocap message) work end to end in Isaac before it flies?

**Answer: yes.** Two fused missions flew the full standard mission (SAFETY
takeoff, gated DIRECT entry, 10 planner legs, SAFETY revert, landing) with no
abort, no rotor saturation and no joint clamp. Large motions fly the same as the
raw-mocap baseline; small, quiet legs are measurably worse, and that traces to
the fused feedback being a coarser signal (below).

## What was flown

Same plant (config A: +15 % allocator kf, body mass/inertia x1.10, 10/10/5 mm
CoM shift, MN4010 rotor lag, current-loop residual, gearbox friction x1.05, arm
mass x1.05), same `_sim` yaml, same mission (`wb_l1_campaign_driver.py`, 6 s
holds), same box (shiqi-desktop, **RTF 0.50** with rendering). Only the
estimator differs.

| run | stack | feedback |
|---|---|---|
| A | `..._stack_fused.sh` | EKF2 fused (recorder failed, see traps) |
| B | `..._stack.sh` | raw mocap |
| C | `..._stack_fused.sh` | EKF2 fused |

## The fused feedback itself (run C vs Isaac ground truth, DIRECT)

| | fused (C) | raw mocap (B) |
|---|---|---|
| rate | **51 Hz** | 250 Hz (50 % exact repeats at RTF 0.5) |
| position error rms / max | **4.4 / 19 mm** | 0.36 / 2.6 mm |
| position bias | < 0.3 mm per axis | 0.0 |
| velocity error rms / max | **7.7 / 41 mm/s** | 0.6 / 6.8 mm/s |
| effective lag, pos / vel | **32 / 14 ms** | 4 / 4 ms |
| EKF2 cs_ev_pos/hgt/vel/yaw airborne | 100 % | 100 % |
| EKF2 resets in flight (jumps > 20 mm) | 0 | 0 |

No `fmu/out/timesync_status` is published (the Pegasus launcher sets
`UXRCE_DDS_SYNCT=0`), so the fused bridge stamps with arrival time and the
estimator sends EV stamps with a zero offset. EKF2 fused regardless (flags
100 %), but **the hardware timestamp path (offset conversion) is NOT exercised
here** — check `timesync_status` and the EV flags on the vehicle.

The 51 Hz is EKF2's output cadence in non-lockstep SITL at RTF 0.5; PX4 puts no
rate limit on `vehicle_odometry`. On hardware, measure it
(`ros2 topic hz /uav_0/state_estimator/local_position/odom`).

## Control (`control_metrics_ABC.txt`; errors are against each run's OWN feedback)

| | fused A | fused C | raw B |
|---|---|---|---|
| aborted | no | no | no |
| DIRECT entry peak CoM error | 520 mm | 558 mm | 524 mm |
| CoM error mean / max | 44.6 / 247 mm | 43.7 / 228 mm | 39.6 / 240 mm |
| EE error mean / max | 7.5 / 30 mm | 7.7 / 35 mm | 7.7 / 28 mm |
| tilt p-p, max e_R | 3.6°, 0.162 | 6.0°, 0.165 | 3.4°, 0.146 |
| peak arm torque | 0.86 N·m | 0.95 N·m | 0.85 N·m |
| saturation / joint clamp | 0 / 0 % | 0 / 0 % | 0 / 0 % |
| d̂_z mean / std | −11.18 / 0.089 N | −11.20 / 0.094 N | −11.18 / 0.055 N |
| yaw steps peak / settle | 22–24 / 12–15 mm | 30–35 / 18–19 mm | 6–14 / 4–7 mm |
| EE trajectory peak | 35 mm | 39 mm | 24 mm |
| x/y steps peak | 175–247 mm | 160–228 mm | 182–240 mm |

- The x/y steps, both-traj legs and the entry transient are inside the
  run-to-run spread: the 0.5 m transitions are planner/handover-dominated and
  the feedback does not matter there.
- The quiet legs (yaw steps, arm-only EE trajectory, post-hold) are worse on
  BOTH fused runs, 2–5x on the yaw steps. The observer's d̂_z std is 1.7x
  higher, which is the fused feedback's 7.7 mm/s velocity noise and ~30 ms lag
  entering the loop. The ~4 mm fused estimation error also adds to these
  numbers, which are measured against the estimate.
- One raw baseline only; the fused-vs-raw ordering holds on both fused repeats.

## Not covered

Mocap faults (the emulated mocap is perfect — no noise, dropout or the 0921
rigid-body flips; the fused path's claimed advantage there is untested), the
hardware timesync path, RTF near 1, contact. The fused odometry's 50 Hz and 30 ms
lag are SITL numbers at RTF 0.5.

## Reproduce

    ./run_fused.sh l1_4d_fused <tag>     # fused stack
    ./run_fused.sh l1_4d <tag>           # raw-mocap baseline
    /usr/bin/python3 tools/fused_feedback_score.py fb_<which>_<tag>.npz
    /usr/bin/python3 ../../../application/robotic_arm/utils/wb_l1_metrics.py <which>_<tag>.npz

Manual launch (Command.md 7.17.1 procedure):

    fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh shiqi_machine
    fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_fused_direct_actuation_sitl.sh shiqi_machine

The Pegasus launcher refuses unless `indoor_state_estimator_node` (fused) is
the running estimator, then hands over to the unchanged 4-D launcher.

## Traps hit (shiqi-desktop)

- **Fast DDS shared memory.** A participant started from a shell received PX4's
  agent-bridged topics but nothing from the ROS nodes (run A's recorder came back
  empty; `ros2 topic echo` hung). A UDP-only participant received everything at
  once. Orphaned `/dev/shm/sem.fastrtps_port*_mutex` files from killed processes
  are the likely cause; `run_fused.sh` now pre-cleans them, and the recorder and
  harness clients use `tools/fastdds_udp_only.xml` (the stack keeps its default
  transport).
- **The ROS 2 daemon wedges during bring-up** and a daemon-backed
  `ros2 topic echo` then never returns, stranding `wb_l1_tune_cycle.sh` at
  "waiting for odometry". Its readiness checks now use `--no-daemon` with
  explicit types.
- Driver npz files are written with the user-site numpy 2.x: read them with
  `/usr/bin/python3` WITHOUT `PYTHONNOUSERSITE=1`.
