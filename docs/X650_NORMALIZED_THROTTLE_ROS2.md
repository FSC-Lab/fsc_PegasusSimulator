# X650 ROS 2 Normalized-Throttle Interface

This runbook records the ROS 2 interface used to send normalized propeller throttle through PX4
to the calibrated X650 in Isaac Sim.

## Start the Simulation

For a custom controller that starts and owns `MicroXRCEAgent`, first start that
controller stack's agent and then launch the controller-neutral PX4 SITL and
X650 plant:

```bash
cd "$HOME/Source/fsc_PegasusSimulator"
./scripts/indoor_sim/start_x650_direct_actuator_sitl.sh fsc_lab_machine
```

This launcher automatically disables Pegasus lockstep and applies the PX4
wall-clock DDS/OFFBOARD parameters. It does not start APL20 or publish any
setpoints or actuator commands.

For the validated APL20-owned test instead, use the following launcher. It
starts its own Micro XRCE-DDS Agent, so do not run a second agent:

The APL20 launcher starts a controller that already owns the actuator topics:

```bash
cd "$HOME/Source/fsc_PegasusSimulator"

FSC_AUTOPILOT_WS="$HOME/Workspaces/fsc_autopilot_ws" \
X650_HOVER_X=1 \
X650_HOVER_Y=-1 \
X650_HOVER_ALT=1.5 \
./scripts/indoor_sim/start_x650_ros_offboard_hover_test.sh fsc_lab_machine
```

Do not run a second normalized-throttle publisher alongside `apl20_ros/autopilot_node`. Only one
node may own the motor-command stream. A custom throttle node must replace APL20 in the launch
workflow, not run concurrently with it.

The free-flight controller and motor allocator are implemented in the external APL20 workspace,
principally in `src/apl20/apl20_ros/src/autopilot_node.cpp` and
`src/apl20/apl20/include/apl/control_allocator.hpp`. The X650-specific controller and rotor
geometry parameters are in `src/apl20/apl20_ros/config/x650.yaml`. See
[X650 APL20 Simulation Start Commands](X650_APL20_START.md#source-code-ownership) for the complete
source ownership map. The separate `$FSC_AUTOPILOT_WS/src/x650_direct_actuator_test_node.py` is
only a pinned motor-pulse diagnostic, not the free-flight controller.

## PX4 Direct-Actuator Topics

Publish normalized throttle to:

```text
/uav_0/fmu/in/actuator_motors
```

Message type:

```text
px4_msgs/msg/ActuatorMotors
```

Set `control[0]` through `control[3]` to normalized values in `[0.0, 1.0]`. Fill every unused
element with `NaN`, which is PX4's stopped/unused sentinel.

Motor order for the X650 is:

| Index | Position | Reaction group |
| --- | --- | --- |
| `control[0]` | Front-right | Positive (`km > 0`) |
| `control[1]` | Rear-left | Positive (`km > 0`) |
| `control[2]` | Front-left | Negative (`km < 0`) |
| `control[3]` | Rear-right | Negative (`km < 0`) |

Continuously publish the OFFBOARD heartbeat on:

```text
/uav_0/fmu/in/offboard_control_mode
```

Message type:

```text
px4_msgs/msg/OffboardControlMode
```

Set `direct_actuator=true`. Prestream the heartbeat and stopped actuator commands before requesting
OFFBOARD and arming. Nonzero motors must only be sent after PX4 reports both OFFBOARD and armed on
`/uav_0/fmu/out/vehicle_status_v1`.

The complete path is:

```text
ROS 2 ActuatorMotors
  -> PX4 OFFBOARD/arming/saturation gate
  -> HIL_ACTUATOR_CONTROLS over MAVLink
  -> Pegasus PX4 backend
  -> calibrated X650 rotor-speed and thrust model
```

## Inspect the Interfaces

```bash
source /opt/ros/humble/setup.bash
source "$HOME/Workspaces/fsc_autopilot_ws/install/setup.bash"

ros2 topic info /uav_0/fmu/in/actuator_motors -v
ros2 topic info /uav_0/fmu/in/offboard_control_mode -v
ros2 interface show px4_msgs/msg/ActuatorMotors
ros2 interface show px4_msgs/msg/OffboardControlMode
ros2 interface show px4_msgs/msg/VehicleStatus
```

## Isaac Ground Truth

Monitor the true simulator state on:

```text
/uav_0/state/pose
/uav_0/state/twist_inertial
/uav_0/state/twist
/uav_0/mocap
```

The first three topics come directly from Isaac Sim. `/uav_0/mocap` is produced by the OptiTrack
emulator from those ground-truth feeds.

## Direct Pegasus Rotor Topics

Pegasus also defines individual `std_msgs/msg/Float64` inputs:

```text
/uav_0/control/rotor0/ref
/uav_0/control/rotor1/ref
/uav_0/control/rotor2/ref
/uav_0/control/rotor3/ref
```

These values represent rotor angular velocity, not normalized throttle. They are intentionally
disabled for the X650 (`sub_control=False`) because PX4 is the primary motor-command backend. Use
`ActuatorMotors` for the current PX4/HIL simulation.

## Stop the Simulation

```bash
tmux kill-session -t x650_ros_hover
```
