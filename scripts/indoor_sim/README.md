# Indoor simulation launchers

These launchers cover OptiTrack/ROS 2, aerial-manipulator, slung-load,
variable-cable, multi-drone, and direct-actuator indoor scenarios.

Run them from any directory and pass a machine config where supported, for
example:

```bash
./scripts/indoor_sim/start_x650_ros_offboard_hover_test.sh fsc_lab_machine
```

Shared configuration and helper scripts remain in the parent `scripts/`
directory.

## Standard indoor Iris with a separate PX4 parameter profile

```bash
./scripts/indoor_sim/start_single_drone_iris.sh fsc_lab_machine
```

On its first run, this creates
`build/px4_sitl_default/rootfs_fsc_indoor/` inside the configured PX4 checkout.
That directory has its own persistent `parameters.bson`; it is not shared with
the outdoor launchers. PX4 still runs as instance 0, so the simulator keeps TCP
port 4560, MAVLink system ID 1, and `/uav_0`.

The launcher applies the indoor OptiTrack profile during PX4 startup:

```text
EKF2_HGT_REF=3       # Vision
EKF2_MAG_TYPE=5      # None
EKF2_GPS_CTRL=0      # GPS aiding disabled
EKF2_EV_CTRL=15      # Position, height, velocity, and yaw
EKF2_EV_DELAY=0      # Override with PX4_INDOOR_EV_DELAY_MS
COM_ARM_WO_GPS=1     # Warning only
```

These numeric values were checked against the active PX4 build metadata and
match the indoor estimator package's OptiTrack instructions. The overrides are
applied before EKF2 starts, including settings marked reboot-required. After
boot, the launcher prints all six effective values in the PX4 pane and saves
them to the isolated indoor `parameters.bson`.

Isaac publishes the true vehicle state through its telemetry-only ROS backend:

```text
/uav_0/state/pose             geometry_msgs/msg/PoseStamped
/uav_0/state/twist_inertial   geometry_msgs/msg/TwistStamped
/uav_0/state/twist            geometry_msgs/msg/TwistStamped
```

`state/pose` contains ENU `map` position and quaternion attitude.
`state/twist_inertial` contains ENU `map` linear velocity; `state/twist`
contains FLU body angular rate. The launcher samples and type-checks the first
two required inertial-frame topics on every run and writes the result to
`/tmp/indoor_iris_groundtruth.log`.

## Indoor X650 with motor lag

```bash
./scripts/indoor_sim/start_single_drone_x650.sh fsc_lab_machine
```

This uses corrected `x650_new.usd` rotation and PX4 motor ordering while
preserving the calibrated 3.5 kg mass and CAD-derived inertia. The plant uses
the measured first-order motor lag (`lambda=10.51 1/s`). Its isolated PX4 profile is stored in
`build/px4_sitl_default/rootfs_fsc_indoor_x650/`. The profile combines the same
indoor OptiTrack estimator settings listed above with the validated X650 rate
and attitude gains. Ground-truth verification is written to
`/tmp/indoor_x650_groundtruth.log`.

## Controller-neutral X650 direct actuation

Use this wrapper when an external ROS 2 controller replaces APL20 and owns the
Micro XRCE-DDS Agent plus the PX4 OFFBOARD/direct-actuator topics:

```bash
./scripts/indoor_sim/start_x650_direct_actuator_sitl.sh fsc_lab_machine
```

Start `MicroXRCEAgent udp4 -p 8888` from the external controller stack before
running the launcher. The wrapper refuses to start if the agent is not present.
It reuses the standard indoor X650 PX4/Isaac launcher, automatically exports
`PEGASUS_PX4_LOCKSTEP=0`, and applies the per-run PX4 settings required by a
wall-clock DDS controller (`UXRCE_DDS_SYNCT=0`, `COM_DISARM_LAND=0`, and
`COM_DISARM_PRFLT=0`). It does not start APL20, a setpoint publisher, or any
other motor-command source.

The external controller is responsible for prestreaming stopped actuator
commands, requesting and confirming OFFBOARD, requesting and confirming
arming, and only then publishing nonzero motors.
