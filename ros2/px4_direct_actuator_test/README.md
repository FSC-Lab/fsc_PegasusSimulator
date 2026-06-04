# px4_direct_actuator_test

Standalone ROS 2 (Humble) node that exercises PX4 **`direct_actuator` offboard
control** over uXRCE-DDS, for use with PX4 SITL (`none_iris`) + Isaac Sim /
Pegasus HIL.

It does **not** depend on, import, or modify any other package in the
workspace. `px4_msgs` is used only as a message dependency.

## Why this exists

PX4 cannot be put into `direct_actuator` offboard over MAVLink (the MAVLink
receiver only maps position/attitude offboard setpoints). The only path is
uXRCE-DDS: publish `OffboardControlMode{direct_actuator: true}` together with
`ActuatorMotors`. This node streams both, requests OFFBOARD + ARM, and prints
the resulting arming/nav state so you can confirm PX4 accepts the mode. The
motor outputs PX4 then computes reach Isaac Sim through the existing Pegasus
HIL bridge unchanged.

## What it publishes / subscribes (namespace `uav_0`)

| Direction | Topic | Type |
|-----------|-------|------|
| pub | `/uav_0/fmu/in/offboard_control_mode` | `OffboardControlMode` |
| pub | `/uav_0/fmu/in/actuator_motors`       | `ActuatorMotors` |
| pub | `/uav_0/fmu/in/vehicle_command`       | `VehicleCommand` |
| sub | `/uav_0/fmu/out/vehicle_status`       | `VehicleStatus` |

## Build

This package lives in the PegasusSimulator repo under `ros2/`. Symlink (or copy)
it into a ROS 2 workspace that also has `px4_msgs`, then build:

```bash
ln -s <repo>/ros2/px4_direct_actuator_test ~/ros2_ws/src/px4_direct_actuator_test
cd ~/ros2_ws
colcon build --packages-select px4_direct_actuator_test
source install/setup.bash
```

## Run

Prerequisites (the architecture in `scripts/start_single_drone_sitl.sh`):
1. **Micro XRCE-DDS Agent** running, e.g. `MicroXRCEAgent udp4 -p 8888`
2. **PX4 SITL** `none_iris` with `PX4_UXRCE_DDS_NS=uav_0`
3. **Isaac Sim / Pegasus** connected on the HIL link (TCP 4560)

Then:

```bash
ros2 launch px4_direct_actuator_test direct_actuator_test.launch.py \
    px4_ns:=uav_0 motor_value:=0.5
```

Or directly:

```bash
ros2 run px4_direct_actuator_test direct_actuator_test --ros-args -p motor_value:=0.5
```

## Parameters

| Param | Default | Meaning |
|-------|---------|---------|
| `px4_ns` | `uav_0` | uXRCE-DDS client namespace |
| `pub_rate_hz` | `50.0` | setpoint stream rate (must be > 2 Hz) |
| `num_motors` | `4` | active motor channels (rest sent as NaN) |
| `motor_value` | `0.5` | normalized command per motor, range `[-1, 1]` |
| `auto_arm` | `true` | auto-request OFFBOARD + ARM after streaming |
| `arm_after_setpoints` | `25` | setpoints streamed before requesting OFFBOARD+ARM |

## Interpreting the output

- `arming_state=2 (armed=True)` and `nav_state=14 (offboard=True)` → PX4
  accepted direct-actuator offboard. Pair this with `PEGASUS_HIL_DEBUG=1` on the
  Isaac side: you should see non-zero `controls`/`rotor_ref`.
- Stays `armed=False` → PX4 rejected the mode/arming (check the PX4 console for
  the rejection reason).

> Safety: with `motor_value` non-zero the motors spin immediately on arming.
> Keep the value modest while tuning. Ctrl-C sends a best-effort DISARM.
