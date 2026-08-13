#!/usr/bin/env bash
set -euo pipefail

# AERIAL-MANIPULATOR direct-actuator simulation on the T650 parameter set,
# with the ARM COMMANDED OVER ROS 2 by the fsc_open_manipulator POSITION-MODE
# stack (aerial/inverted configuration) instead of 04's in-process hold.
#
# Incremental sibling of start_t650_aerial_manipulator_direct_actuator_sitl.sh
# (which keeps working unchanged): the drone side is identical — external
# fsc_autopilot_ros2 DIRECT law -> PX4 gate -> AM_xfwd plant on T650 motors —
# and what changes is who owns the arm reference:
#
#   fsc_open_manipulator PositionController (the REAL ros2_control plugin,
#       aerial config: home [0, 40, 40, 0] deg, min-jerk moves)
#     -> open_manipulator_x_isaac_bridge/IsaacTopicSystem
#     -> /uav_0/arm/joint_position_commands -> Isaac servo emulation
#     <- /uav_0/arm/joint_states
#
# On top of the base tmux session this launcher adds an "arm" window with two
# panes: the arm ros2_control stack (controller_manager + spawners) and the
# ARM GROUND STATION (custom_gui joint_plot_inverted). Together with the drone
# ground station from the paired fsc_autopilot stack, the running simulation
# has TWO ground stations — one per subsystem, exactly like the real rig.
#
# PAIR WITH fsc_autopilot_ros2/scripts/isaacsim/start_direct_actuation_t650_aerial_manipulator_stack.sh
# (started FIRST — it owns MicroXRCEAgent and brings the drone ground station).
#
# The PX4 profile is SHARED with the 04 launcher on purpose: same plant, same
# saved tune (the per-vehicle-profile rule separates different vehicles, and
# this is the same vehicle).

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

IN_TERM=0
if [[ "${1:-}" == "--in-terminal" ]]; then
  IN_TERM=1
  shift
fi

if [[ $# -ne 1 ]]; then
  echo "ERROR: must provide config name." >&2
  cfg_usage "$0"
  exit 2
fi

CFG_NAME="$1"

if [[ $IN_TERM -eq 0 ]]; then
  open_new_terminal "$0" --in-terminal "$CFG_NAME"
  exit 0
fi

load_machine_config "$0" "$CFG_NAME"

BASE_LAUNCHER="$SCRIPT_DIR/indoor_sim/start_single_drone_x650.sh"
PARAM_SCRIPT="$SCRIPT_DIR/apply_aerial_manipulator_px4_offboard_params.sh"
SESSION="px4_isaac"
PARAM_DELAY="${T650_AERIAL_MANIPULATOR_DIRECT_ACTUATOR_PARAM_DELAY:-8}"

# Arm-stack workspace (fsc_open_manipulator). Defaults reproduce shiqi-desktop;
# override in the machine config. ROSDEPS_SETUP is the root-less ros2_control
# overlay this machine needs (no sudo) — point it at /dev/null where
# ros2_control is apt-installed.
ARM_WS="${FSC_OM_ARM_WS:-$HOME/ros2_ws/src/fsc_open_manipulator}"
ARM_ROSDEPS_SETUP="${FSC_OM_ARM_ROSDEPS_SETUP:-$HOME/ros2_ws/rosdeps/local_setup.bash}"
ARM_ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
ARM_WS_SETUP="$ARM_WS/install/setup.bash"
ARM_STATE_TOPIC="/uav_0/arm/joint_states"
# Display-only workspace datum for the inverted ground station: the paired
# fsc_autopilot stack's standard hover reference is z = 1.2 m.
ARM_GS_MOUNT_HEIGHT="${ARM_GS_MOUNT_HEIGHT:-1.2}"

# Variant hooks consumed by the base launcher (same mechanism as the 04
# launcher; only the Isaac entrypoint and the label differ).
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/05_px4_direct_t650_aerial_manipulator_ros2_arm_hold.py"
export INDOOR_SIM_VEHICLE_LABEL="AM-T650-ARM"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"

[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || {
  echo "ERROR: missing AM-T650 Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2
  exit 1
}
[[ -f "$ARM_WS_SETUP" ]] || {
  echo "ERROR: fsc_open_manipulator workspace is not built: $ARM_WS_SETUP" >&2
  echo "Build it with:" >&2
  echo "  cd $ARM_WS && source $ARM_ROS2_SETUP && source $ARM_ROSDEPS_SETUP \\" >&2
  echo "  && colcon build --base-paths src --packages-select \\" >&2
  echo "     dynamixel_interfaces open_manipulator_x_description open_manipulator_x_bringup \\" >&2
  echo "     open_manipulator_x_custom_controller open_manipulator_x_isaac_bridge custom_gui \\" >&2
  echo "     --symlink-install" >&2
  exit 1
}
if [[ ! "$PARAM_DELAY" =~ ^[0-9]+$ ]]; then
  echo "ERROR: T650_AERIAL_MANIPULATOR_DIRECT_ACTUATOR_PARAM_DELAY must be a non-negative integer." >&2
  exit 2
fi

if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo "ERROR: MicroXRCEAgent is not running." >&2
  echo "Start the agent from the external controller stack before this launcher." >&2
  exit 1
fi

# A wall-clock DDS controller can pause PX4 actuator output during OFFBOARD
# transitions. Disabling Pegasus lockstep prevents both sides waiting forever.
# The tmux-server push mirrors the 04 launcher — see its comment for why the
# export alone is discarded whenever a tmux server already exists.
export PEGASUS_PX4_LOCKSTEP=0
if tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null; then
  echo "Pegasus PX4 lockstep: pushed to the tmux server environment"
else
  echo "Pegasus PX4 lockstep: no tmux server yet; the new session will inherit the export"
fi

echo "Starting AM-T650 direct-actuator SITL with the ROS2 position-mode arm stack."
echo "Plant: AM_xfwd on T650 motors; arm commanded by fsc_open_manipulator"
echo "  PositionController (aerial config, home [0,40,40,0] deg) via"
echo "  open_manipulator_x_isaac_bridge; arm ground station: joint_plot_inverted"
echo "Arm workspace: $ARM_WS"
echo "MicroXRCEAgent: externally owned and detected"

# Environment chain for the arm panes. The workspace was built against the
# root-less rosdeps overlay, and its setup.bash cannot re-source it (the
# overlay is a plain deb extract, not a colcon prefix), so the chain is
# explicit: ROS 2 -> rosdeps overlay (if present) -> arm workspace.
ARM_ENV="source '$ARM_ROS2_SETUP'; if [ -f '$ARM_ROSDEPS_SETUP' ]; then source '$ARM_ROSDEPS_SETUP'; fi; source '$ARM_WS_SETUP'"
ARM_DISPLAY="${DISPLAY:-:0}"

# The base launcher (exec'd below) recreates the tmux session, so the arm
# window is added by a detached helper once the new session exists. The stack
# pane then waits for Isaac's arm joint states before launching
# controller_manager, keeping the hardware-activation timeout budget intact.
(
  sleep 8
  for _ in $(seq 90); do
    tmux has-session -t "$SESSION" 2>/dev/null && break
    sleep 1
  done
  tmux has-session -t "$SESSION" 2>/dev/null || exit 0

  tmux new-window -d -t "$SESSION" -n arm "
$ARM_ENV
echo 'Waiting for Isaac arm joint states on $ARM_STATE_TOPIC ...'
until timeout 5 ros2 topic echo --once $ARM_STATE_TOPIC >/dev/null 2>&1; do
  sleep 2
done
echo 'Isaac arm is reporting; launching the position-mode ros2_control stack.'
ros2 launch open_manipulator_x_isaac_bridge position_control_isaac.launch.py
echo 'Arm ros2_control stack exited.'
exec bash
"

  tmux split-window -h -t "$SESSION:arm" "
$ARM_ENV
export DISPLAY='$ARM_DISPLAY'
echo 'Arm ground station (inverted). It discovers the active controller on its own.'
ros2 run custom_gui joint_plot_inverted --ros-args -r __ns:=/uav_0 \\
  -p torque_constant:=1.81 -p mount_height:=$ARM_GS_MOUNT_HEIGHT
echo 'Arm ground station exited.'
exec bash
"
) &

# Apply the wall-clock DDS timestamp and HIL auto-disarm settings after the PX4
# shell is ready. These changes intentionally remain per-run and are not saved.
"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

# Reuse the validated indoor PX4/Isaac orchestration and cleanup. Passing
# --in-terminal prevents the base launcher from opening a second terminal.
exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
