## Fsc Aerial Manipulation-Pegasus-PX4 SITL Launch Scripts

> Full per-script documentation (what each one launches, tmux layout, prerequisites) lives in the Sphinx docs under `docs/source/fsc_lab/index.md`.

### 1. How to launch the simulation:
```
{$PATH_TO_FSC_PEGASUS}/scripts/start_single_drone_sitl.sh {CONFIG_FILE_NAME}
```
- For example:
```
source/fsc_PegasusSimulator/scripts/start_single_drone_sitl.sh longhao_machine
```
### 2. To setup the path for your machine, create a new .conf file in the config folder. 

- In the new .conf file, add the following:
- The directory of the PX4 SITL:
```
PX4_DIR="$HOME/PX4-Autopilot"
```
- The path to the root of fsc_Pegasus repo:
```
FSC_PEGASUS_ROOT="/home/longhao/source/fsc_PegasusSimulator"
```
- The path to Issac sim python launch script:
```
ISAAC_PY="$HOME/isaacsim/python_r_fsc.sh"
```

### 3. If a scenario crashes or closes uncleanly

The launch scripts kill each other's tmux pane (PX4 <-> Isaac Sim) when either side exits, so a
lingering half shouldn't normally happen. If it does anyway (e.g. a process started outside these
scripts), scan for and clean it up with:
```
./scripts/kill_stale_sim_processes.sh          # scan, ask before killing
./scripts/kill_stale_sim_processes.sh -y        # scan, kill without asking
./scripts/kill_stale_sim_processes.sh --dry-run # scan only
```

### 4. Aerial manipulator control modes

The aerial-manipulator launcher preserves the original direct ROS 2 rotor path and
also supports PX4 Offboard direct-actuator control:

```bash
# Original: controller publishes rotor angular velocity directly to Isaac
./scripts/start_aerial_manipulator.sh longhao_machine direct

# PX4 path: controller -> ActuatorMotors -> PX4 gate -> HIL_ACTUATOR_CONTROLS -> Isaac
./scripts/start_aerial_manipulator.sh longhao_machine px4-offboard
```

The PX4 mode starts PX4 SITL, Micro XRCE-DDS Agent, Isaac Sim, and the whole-body
controller in one tmux session. It automatically prestreams `OffboardControlMode`
and `ActuatorMotors` for one second, requests Offboard mode, waits for PX4 to
confirm it, and then requests arming.

The controller needs a `px4_msgs` ROS 2 overlay. The launcher defaults to:

```bash
$HOME/source/fsc_autopilot_ws/install/setup.bash
```

If yours is elsewhere, add this to the selected machine config:

```bash
PX4_MSGS_SETUP="$HOME/path/to/px4_ros_ws/install/setup.bash"
```

The PX4 mode writes logs to `/tmp/aerial_manip_px4.log`,
`/tmp/aerial_manip_xrce_agent.log`, `/tmp/aerial_manip_isaac.log`, and
`/tmp/aerial_manip_controller.log`. It auto-arms the simulated vehicle; keep the
PX4 mode for SITL only unless you deliberately remove that launcher setting.

### 5. X650 ROS direct-actuator hover test

The free-flight hover fixture launches PX4, the XRCE agent, the calibrated X650
plant, the external `apl20_ros` PX4-style cascade, and a persistent 1.5 m ENU
setpoint:

```bash
./scripts/start_x650_ros_offboard_hover_test.sh longhao_machine
./scripts/start_x650_ros_offboard_hover_test.sh longhao_machine headless
```

Override the target with `X650_HOVER_ALT` and run detached with
`X650_HOVER_TEST_DETACHED=1`. Controller tracking is written to
`/tmp/x650_ros_hover_track.csv`; component logs use the same
`/tmp/x650_ros_hover_*` prefix.
