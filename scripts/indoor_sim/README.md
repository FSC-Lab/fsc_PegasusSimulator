# Indoor simulation launchers

Run any launcher from any directory and pass a machine config, for example:

```bash
./scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh shiqi_machine
```

Folder layout:

- Top level: the actively used aerial-manipulator launchers (pairs below).
- `lib/`: shared pieces the launchers use, not normally run by hand.
  - `start_single_drone_x650.sh`: the base PX4 + Isaac orchestration that
    every launcher here (active and archived) ends up running. It still runs
    on its own as the bare indoor X650 (see the reference section below).
  - `am_plant_from_yaml.sh`: reads the plant knobs from the paired
    controller yaml.
- `archive/`: inactive launchers, still runnable (list below).

Shared configuration and helper scripts remain in the parent `scripts/`
directory.

## Sim-and-real launch pairs (active aerial-manipulator scripts)

Each Isaac launcher below pairs with one simulation flight stack in
`fsc_autopilot_ros2/scripts/isaacsim/` (start it FIRST; it owns
MicroXRCEAgent) and, where one exists, the real-hardware flight stack in
`fsc_autopilot_ros2/scripts/indoor_exp/`. "none" = no hardware stack.
"raw mocap" / "EKF2-fused" = which feedback the controller is fed.

- `start_t650_aerial_manipulator_baseline_sitl.sh`
  - Sim stack: `start_baseline_t650_aerial_manipulator_stack_fused.sh`
  - Hardware stack: none
- `start_t650_aerial_manipulator_direct_actuation_sitl.sh`
  - Sim stack: `start_direct_actuation_t650_aerial_manipulator_stack.sh`
  - Hardware stack: none
- `start_t650_aerial_manipulator_geometric_direct_actuation_sitl.sh`
  - Sim stack: `start_geometric_direct_actuation_t650_aerial_manipulator_stack.sh`
  - Hardware stack: `start_geometric_direct_actuation_stack_t650_aerial_manipulator.sh`
- `start_t650_aerial_manipulator_geometric_L1_adaptive_direct_actuation_sitl.sh`
  - Sim stack: `start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh` (raw mocap)
  - Hardware stack: `start_geometric_l1_direct_actuation_stack_t650_aerial_manipulator.sh` (raw mocap)
    and `start_geometric_l1_direct_actuation_stack_t650_aerial_manipulator_fused.sh`
    (EKF2-fused; no fused sim twin exists)
- `start_t650_aerial_manipulator_modular_adaptive_direct_actuation_sitl.sh`
  - Sim stack: `start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh`
  - Hardware stack: none (simulation-only comparison rig)
- `start_t650_aerial_manipulator_whole_body_GMO_6D_direct_actuation_sitl.sh`
  - Sim stack: `start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh`
  - Hardware stack: `start_whole_body_direct_actuation_stack_t650_aerial_manipulator.sh`
- `start_t650_aerial_manipulator_whole_body_L1_adaptive_6D_direct_actuation_sitl.sh`
  - Sim stack: `start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh`
  - Hardware stack: `start_whole_body_l1_direct_actuation_stack_t650_aerial_manipulator.sh`
- `start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh`
  - Sim stack: `start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh` (raw mocap)
  - Hardware stack: `start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh` (raw mocap)
- `start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_fused_sitl.sh`
  - Sim stack: `start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh` (EKF2-fused)
  - Hardware stack: `start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator_fused.sh` (EKF2-fused)

## Archived launchers (`archive/`)

Inactive, but still runnable from their new location:
`./scripts/indoor_sim/archive/<name>.sh <machine_config>`. On 2026-10-01 every
one was checked to find its machine config, its helper scripts and the base
launcher in `lib/` (no simulation was started).

- Bare drones:
  - `start_single_drone_iris.sh`
  - `start_single_drone_t650.sh`
  - `start_single_drone_t650_gate_splat.sh` (needs the gate-splat assets in
    `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/assets/`,
    which are distributed separately and are not on every machine)
- Bare-drone direct actuation:
  - `start_x650_direct_actuator_sitl.sh`
  - `start_t650_direct_actuator_sitl.sh`
  - `start_t650_gate_splat_direct_actuator_sitl.sh` (same asset note)
  - `start_t650_geometric_direct_actuator_sitl.sh`
  - `start_t650_geometric_L1_adaptive_direct_actuation_sitl.sh`
- X650 tests:
  - `start_x650_pinned_direct_actuator_test.sh`
  - `start_x650_ros_offboard_hover_test.sh`
- Slung load:
  - `start_single_drone_sitl_payload.sh`
  - `start_single_drone_sitl_payload_test.sh`
  - `start_single_drone_sitl_payload_variable_cable.sh`
  - `start_single_drone_sitl_payload_variable_cable_x650.sh`
  - `start_single_drone_sitl_payload_x650.sh`
- Multi-drone:
  - `start_multi_drone_sitl.sh`
  - `start_3_drone_point_mass_payload_sitl.sh`
  - `start_3_drone_rigid_body_payload_variable_cable_sitl.sh`
  - `start_3_drone_rigid_body_payload_sitl.sh` (legacy: takes no machine
    config and has another machine's absolute paths written in)

## Reference: base launcher and archived single-drone launchers

### Standard indoor Iris with a separate PX4 parameter profile

```bash
./scripts/indoor_sim/archive/start_single_drone_iris.sh fsc_lab_machine
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

### Indoor X650 with motor lag (the base launcher)

```bash
./scripts/indoor_sim/lib/start_single_drone_x650.sh fsc_lab_machine
```

This uses corrected `x650_new.usd` rotation and PX4 motor ordering while
preserving the calibrated 3.5 kg mass and CAD-derived inertia. The plant uses
the measured first-order motor lag (`lambda=10.51 1/s`). Its isolated PX4 profile is stored in
`build/px4_sitl_default/rootfs_fsc_indoor_x650/`. The profile combines the same
indoor OptiTrack estimator settings listed above with the validated X650 rate
and attitude gains. Ground-truth verification is written to
`/tmp/indoor_x650_groundtruth.log`.

### Controller-neutral X650 direct actuation

Use this wrapper when an external ROS 2 controller replaces APL20 and owns the
Micro XRCE-DDS Agent plus the PX4 OFFBOARD/direct-actuator topics:

```bash
./scripts/indoor_sim/archive/start_x650_direct_actuator_sitl.sh fsc_lab_machine
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
