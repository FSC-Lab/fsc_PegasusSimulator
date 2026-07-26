# X650 APL20 Simulation Start Commands

This runbook starts the bare X650 with the external APL20 controller through the complete
ROS 2 → PX4 direct-actuator → MAVLink HIL → Pegasus path.

## Source-Code Ownership

The X650 free-flight direct-actuation controller is source code in the external APL20 workspace,
not in this Pegasus Simulator repository. The workspace root is selected with
`FSC_AUTOPILOT_WS`; on the FSC Jupiter machine its canonical location is
`$HOME/Workspaces/fsc_autopilot_ws`.

The main implementation files, relative to that workspace, are:

- `src/apl20/apl20_ros/src/autopilot_node.cpp`: runs the position, attitude, and rate cascade,
  publishes `OffboardControlMode` with `direct_actuator=true`, and publishes the four normalized
  motor commands as `px4_msgs/msg/ActuatorMotors`.
- `src/apl20/apl20/include/apl/control_allocator.hpp`: maps normalized body torque and collective
  thrust to per-motor commands, including allocation, desaturation, and output clamping.
- `src/apl20/apl20/src/position_controller.cpp`, `attitude_controller.cpp`, and
  `rate_controller.cpp`: implement the individual flight-control loops.
- `src/apl20/apl20_ros/config/x650.yaml`: defines the X650 hover command, control gains, rotor
  positions, motor spin signs, moment ratio, and motor limits.

This repository owns the launcher and simulated plant. In particular,
`scripts/indoor_sim/start_x650_ros_offboard_hover_test.sh` starts
`apl20_ros/autopilot_node`, while `application/px4_base/03_px4_single_drone_x650.py` and the FSC
X650 rotorcraft modules implement the Isaac/Pegasus side.

The complete control path is:

```text
APL20 position/attitude/rate control
  -> APL20 control allocator
  -> /uav_0/fmu/in/actuator_motors
  -> PX4 OFFBOARD/arming/saturation gate
  -> HIL_ACTUATOR_CONTROLS over MAVLink
  -> Pegasus calibrated X650 motor/thrust simulation
```

Do not confuse the free-flight controller with
`$FSC_AUTOPILOT_WS/src/x650_direct_actuator_test_node.py`. That standalone node only generates
the pinned fixture's motor-pulse sequence for checking motor order and roll, pitch, and yaw signs;
it is not the APL20 flight controller.

## Prerequisites

APL20 is built in `$HOME/Workspaces/fsc_autopilot_ws`. It requires CMake 3.28 or newer; keep
CMake below 4 for compatibility with the other workspace projects:

```bash
python3 -m pip install --user "cmake>=3.28,<4"
export PATH="$HOME/.local/bin:$PATH"
hash -r
cmake --version
```

Build and test APL20 against the existing `px4_msgs` installation:

```bash
cd "$HOME/Workspaces/fsc_autopilot_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install --packages-select apl20 apl20_ros \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON

source install/setup.bash
ros2 pkg prefix apl20_ros
colcon test --packages-select apl20 apl20_ros
colcon test-result --verbose
```

Confirm that the NVIDIA driver is ready before starting Isaac Sim:

```bash
nvidia-smi
```

If this reports `Driver/library version mismatch` immediately after a driver update, reboot and
check again.

## Start With the Isaac Sim GUI

The following verified command requests an ENU target of 1 m east, 1 m south, and 1.5 m up:

```bash
cd "$HOME/Source/fsc_PegasusSimulator"

FSC_AUTOPILOT_WS="$HOME/Workspaces/fsc_autopilot_ws" \
X650_HOVER_X=1 \
X650_HOVER_Y=-1 \
X650_HOVER_ALT=1.5 \
./scripts/indoor_sim/start_x650_ros_offboard_hover_test.sh fsc_lab_machine
```

For a vertical hover at the origin, omit the target overrides:

```bash
cd "$HOME/Source/fsc_PegasusSimulator"

FSC_AUTOPILOT_WS="$HOME/Workspaces/fsc_autopilot_ws" \
./scripts/indoor_sim/start_x650_ros_offboard_hover_test.sh fsc_lab_machine
```

## Start Headless

Append `headless` to the launcher command:

```bash
cd "$HOME/Source/fsc_PegasusSimulator"

FSC_AUTOPILOT_WS="$HOME/Workspaces/fsc_autopilot_ws" \
X650_HOVER_X=1 \
X650_HOVER_Y=-1 \
X650_HOVER_ALT=1.5 \
./scripts/indoor_sim/start_x650_ros_offboard_hover_test.sh fsc_lab_machine headless
```

The target coordinates use ENU: positive x is east, positive y is north, and positive z is up.

## Isaac Sim Ground-Truth ROS Topics

The launcher starts `isaacsim_optitrack_ros2_emulator` automatically. The simulator publishes:

- `/uav_0/state/pose` (`geometry_msgs/msg/PoseStamped`): position and quaternion in ENU `map`.
- `/uav_0/state/twist_inertial` (`geometry_msgs/msg/TwistStamped`): linear velocity in ENU `map`.
- `/uav_0/state/twist` (`geometry_msgs/msg/TwistStamped`): body-frame FLU linear velocity and
  angular rate.

The emulator combines those feeds into `/uav_0/mocap`
(`fsc_autopilot_ros2_msgs/msg/Mocap`). Its pose and linear velocity are inertial ENU; its angular
rate is body FLU.

Inspect one message from each feed in another ROS-sourced terminal:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/Workspaces/fsc_autopilot_ws/install/setup.bash"
ros2 topic echo --once /uav_0/state/pose
ros2 topic echo --once /uav_0/state/twist_inertial
ros2 topic echo --once /uav_0/state/twist
ros2 topic echo --once /uav_0/mocap
```

## Inspect and Stop

```bash
tail -n 20 /tmp/x650_ros_hover_track.csv
grep -Ein 'error|failed|traceback|segmentation' /tmp/x650_ros_hover_*.log
tmux kill-session -t x650_ros_hover
```

The verified `(1, -1, 1.5)` m run converged to approximately 1.5 cm 3D tracking error.
