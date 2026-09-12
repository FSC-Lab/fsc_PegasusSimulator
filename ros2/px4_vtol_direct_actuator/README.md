# px4_vtol_direct_actuator

Direct-actuator offboard control for a **dual independent-tiltrotor** VTOL,
simulated in Isaac Sim through the Pegasus MAVLink HIL backend.

This is the VTOL counterpart of `px4_direct_actuator_test`. The key difference:
a tiltrotor has **servos as well as motors**, so this node publishes **both**
`ActuatorMotors` and `ActuatorServos` (the original node published motors only,
which left the tilt/control surfaces dead).

## Airframe / channels

| group  | index | channel      | range  | notes |
|--------|-------|--------------|--------|-------|
| motors | 0     | `motor_left` | [0, 1] | left rotor |
| motors | 1     | `motor_right`| [0, 1] | right rotor |
| servos | 0     | `tilt_left`  | [-1, 1]| -1 = hover, +1 = cruise |
| servos | 1     | `tilt_right` | [-1, 1]| independent → yaw via differential tilt |
| servos | 2     | `aileron`    | [-1, 1]| roll |
| servos | 3     | `elevator`   | [-1, 1]| pitch |
| servos | 4     | `rudder`     | [-1, 1]| yaw (cruise) |

The index order here must agree with **(a)** PX4's actuator-output assignment for
the airframe and **(b)** the Pegasus channel map
(`default_dual_tiltrotor_channel_map`). Verify the full chain with
`PEGASUS_HIL_DEBUG=1` in the Isaac pane.

## Build

```sh
# symlink or copy this folder into ~/ros2_ws/src, then:
cd ~/ros2_ws
colcon build --packages-select px4_vtol_direct_actuator
source install/setup.bash
```

Requires `px4_msgs` (must provide `ActuatorMotors`, `ActuatorServos`,
`OffboardControlMode.direct_actuator`, `VehicleCommand`, `VehicleStatus`).

## PX4 one-time params (see docs/direct_actuator_offboard.md)

```sh
param set UXRCE_DDS_SYNCT 0
param set COM_DISARM_LAND 0
param set COM_DISARM_PRFLT 0
param save
```

## Run

```sh
# 1) DDS agent
MicroXRCEAgent udp4 -p 8888
# 2) PX4 SITL (dual-tiltrotor airframe) + Isaac (application/vtol/03_vtol_px4_hil.py)
PEGASUS_HIL_DEBUG=1 scripts/start_single_drone_sitl.sh <machine-config>
# 3) this node — hover throttle, rotors up:
ros2 run px4_vtol_direct_actuator vtol_direct_actuator \
    --ros-args -p motor_left:=0.6 -p motor_right:=0.6 -p tilt_left:=-1.0 -p tilt_right:=-1.0
```

Change setpoints live, e.g. begin tilting forward:

```sh
ros2 param set /vtol_direct_actuator tilt_left 0.0
ros2 param set /vtol_direct_actuator tilt_right 0.0
```

> `direct_actuator` is **open-loop** — no stabilization. Equal motor values with
> rotors up will climb and tumble; that is expected. Stable flight needs either
> your own controller computing per-actuator values, or PX4's normal VTOL
> flight modes (not direct_actuator).

## Parameters

`px4_ns` (default `uav_0`), `pub_rate_hz` (50), `auto_arm` (true → auto
OFFBOARD+ARM), `arm_after_setpoints` (25), and the per-channel setpoints listed
in the table above.
