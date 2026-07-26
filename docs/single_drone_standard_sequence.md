# Standard Single-Drone PX4 SITL Sequence

This runbook starts a single Iris or bare X650 using standard PX4 SITL control,
with either outdoor GPS/magnetometer or indoor OptiTrack estimator profiles. It
does not use APL20 or ROS 2 direct-actuator throttle control.

## Iris SITL

```bash
cd "$HOME/Source/fsc_PegasusSimulator"
./scripts/outdoor_sim/start_single_drone_sitl.sh fsc_lab_machine
```

This starts:

- PX4 SITL with the `none_iris` target.
- `application/px4_base/01_px4_single_drone.py` in Isaac Sim.
- The Isaac Sim GUI.

## Bare X650 SITL

```bash
cd "$HOME/Source/fsc_PegasusSimulator"
./scripts/outdoor_sim/start_x650_single_drone.sh fsc_lab_machine
```

This starts:

- PX4 SITL with the `none_iris` target and X650-specific PX4 gains.
- `application/px4_base/03_px4_single_drone_x650.py` in Isaac Sim.
- The calibrated MN4014 + 15x5 X650 thrust and rotor-lag model.
- The Isaac Sim GUI.

## Indoor Iris SITL

```bash
cd "$HOME/Source/fsc_PegasusSimulator"
./scripts/indoor_sim/start_single_drone_iris.sh fsc_lab_machine
```

This uses an isolated `rootfs_fsc_indoor` PX4 parameter database with external
vision position, velocity, height, and yaw fusion. GPS and magnetometer aiding
are disabled for the indoor OptiTrack configuration.

## Indoor X650 SITL with motor lag

```bash
cd "$HOME/Source/fsc_PegasusSimulator"
./scripts/indoor_sim/start_single_drone_x650.sh fsc_lab_machine
```

This runs `application/px4_base/03_px4_single_drone_x650.py` with corrected
`x650_new.usd` model rotation and PX4 motor ordering. Runtime mass properties
preserve the established 3.5 kg total and CAD-derived inertia. Its calibrated
MN4014 + 15x5 plant includes the measured first-order motor lag
(`lambda=10.51 1/s`). PX4 starts with both the indoor external-vision settings
and the validated X650 gains:

```text
MC_ROLLRATE_K=0.3   MC_PITCHRATE_K=0.3   MC_YAWRATE_K=0.3
MC_ROLL_P=3.25      MC_PITCH_P=3.25      MC_YAW_P=1.4
```

The settings persist only in `rootfs_fsc_indoor_x650`, independently of the
indoor Iris and outdoor profiles. Set `PX4_INDOOR_EV_DELAY_MS` before launching
to override the default zero-millisecond external-vision delay.

## Control Ownership

PX4 owns attitude, rate, and motor control in every standard sequence. Control
the vehicle through QGroundControl, the PX4 console, or another normal PX4
command interface.

Do not run `apl20_ros/autopilot_node` and do not publish normalized motors to
`/uav_0/fmu/in/actuator_motors` while using these standard launchers.

## Isaac Ground-Truth ROS Topics

The simulator publishes the true vehicle state on:

```text
/uav_0/state/pose
/uav_0/state/twist_inertial
/uav_0/state/twist
```

These contain ENU position and quaternion attitude, ENU inertial linear velocity, and FLU body
velocity/angular rate, respectively.

## Stop the Simulation

All four standard launchers use the `px4_isaac` tmux session:

```bash
tmux kill-session -t px4_isaac
```

Check active sessions if necessary:

```bash
tmux list-sessions
```

Starting any launcher automatically replaces an existing `px4_isaac` session,
so these standard launchers cannot run simultaneously without assigning
separate PX4 ports, instances, vehicle IDs, and tmux session names. The indoor
launchers write their topic checks to `/tmp/indoor_iris_groundtruth.log` and
`/tmp/indoor_x650_groundtruth.log`, respectively.
