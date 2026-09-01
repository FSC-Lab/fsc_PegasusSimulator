#!/usr/bin/env bash
set -euo pipefail

# AERIAL-MANIPULATOR direct-actuator simulation on the T650 parameter set for
# the GEOMETRIC + L1-ADAPTIVE controller (Cai et al., Control Engineering
# Practice 164 (2025) 106418: geometric SE(3) baseline with CoM-offset
# compensation + L1 adaptive augmentation), with the ARM COMMANDED OVER ROS 2
# by the fsc_open_manipulator POSITION-MODE stack.
#
# Incremental sibling of start_t650_aerial_manipulator_geometric_direct_actuator_sitl.sh
# (which keeps working unchanged): THE PEGASUS/PX4/ARM SIDE IS IDENTICAL —
# same AM_xfwd plant on T650 motors (05's Isaac entrypoint), same PX4 profile,
# same fsc_open_manipulator ros2_control stack + arm ground station — and the
# ONLY difference is which external controller stack it pairs with:
#
#   fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh
#   (started FIRST — it owns MicroXRCEAgent and brings the drone ground
#   station; node: autopilot_geometric_l1_direct_actuation_node, params:
#   params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim.yaml,
#   service ns: fsc_autopilot_ros2/geometric_l1_direct_actuation)
#   NOTE THE _sim SUFFIX: that config was split sim/hardware on 2026-08-24. The
#   plain ..._t650.yaml beside it is now the HARDWARE config (bench kf/km, no
#   robustness injections) and must never be flown here — only the _sim.yaml
#   carries the +20% kf and +10 mm r_os injections this rig is meant to exercise.
#
# The controller subscribes to /uav_0/fsc_open_manipulator/joint_states for
# the LIVE r_os (CoM offset) term of the paper's control law, which only the
# arm ros2_control stack this launcher brings publishes — so this launcher,
# not the plain 04 hold one, is the intended pairing.
#
# FIRST CAMPAIGN = THE HOVER TEST (the L1 node has never flown): SAFETY
# takeoff to z = 1.2 m from the drone ground station, settle, then enter
# DIRECT from the Controller tab or:
#   ros2 service call /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode \
#     std_srvs/srv/SetBool "{data: true}"
# Abort back to SAFETY with data: false. See the paired stack script's header
# for what to watch on l1_control_debug.
#
# The PX4 profile is SHARED with the 04/05 launchers on purpose: same plant,
# same saved tune (the per-vehicle-profile rule separates different vehicles,
# and this is the same vehicle).

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

# Colcon workspace holding the fsc_open_manipulator repo (at src/, with its
# Dynamixel deps beside it). Defaults reproduce shiqi-desktop; override in the
# machine config. ROSDEPS_SETUP is the root-less ros2_control overlay this
# machine needs (no sudo) — point it at /dev/null where ros2_control is
# apt-installed.
ARM_WS="${FSC_OM_ARM_WS:-$HOME/ros2_ws}"
ARM_ROSDEPS_SETUP="${FSC_OM_ARM_ROSDEPS_SETUP:-$HOME/ros2_ws/rosdeps/local_setup.bash}"
ARM_ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
ARM_WS_SETUP="$ARM_WS/install/setup.bash"
# Topic naming (see docs/docs_aerial_manipulator/Arm Topic Naming.md):
#   /uav_0/fsc_open_manipulator/...  the REAL arm stack — same names on hardware
#   /uav_0/isaacsim_manipulator/...  the simulated servo bus — sim only
# ARM_NS is the ros2_control stack's namespace; the ground station must run in
# the SAME namespace (it resolves every topic/service relatively), so both are
# derived from this one variable.
ARM_NS="uav_0/fsc_open_manipulator"
ARM_STATE_TOPIC="/uav_0/isaacsim_manipulator/joint_states"
# Display-only workspace datum for the inverted ground station: the paired
# fsc_autopilot stack's standard hover reference is z = 1.2 m.
ARM_GS_MOUNT_HEIGHT="${ARM_GS_MOUNT_HEIGHT:-1.2}"

# Variant hooks consumed by the base launcher (same mechanism as the sibling
# geometric launcher; only the label differs — the plant is the same).
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/05_px4_direct_t650_aerial_manipulator_ros2_arm_hold.py"
export INDOOR_SIM_VEHICLE_LABEL="AM-T650-L1"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"
# Fail before physics starts if this plant ever drifts from the mass used by
# the paired geometric+L1 controller YAML.
export PEGASUS_EXPECTED_TOTAL_MASS="3.746170"

[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || {
  echo "ERROR: missing AM-T650 Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2
  exit 1
}
[[ -f "$ARM_WS_SETUP" ]] || {
  echo "ERROR: the arm workspace is not built: $ARM_WS_SETUP" >&2
  echo "Build it with:" >&2
  echo "  cd $ARM_WS && source $ARM_ROS2_SETUP && source $ARM_ROSDEPS_SETUP \\" >&2
  echo "  && colcon build --packages-select \\" >&2
  echo "     dynamixel_interfaces open_manipulator_x_description open_manipulator_x_bringup \\" >&2
  echo "     open_manipulator_x_custom_controller open_manipulator_x_isaac_bridge utils_custom_ground_station \\" >&2
  echo "     --symlink-install" >&2
  exit 1
}
if [[ ! "$PARAM_DELAY" =~ ^[0-9]+$ ]]; then
  echo "ERROR: T650_AERIAL_MANIPULATOR_DIRECT_ACTUATOR_PARAM_DELAY must be a non-negative integer." >&2
  exit 2
fi

if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo "ERROR: MicroXRCEAgent is not running." >&2
  echo "Start the agent from the external controller stack before this launcher:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh" >&2
  exit 1
fi

# MicroXRCEAgent alone is not proof that the matching controller stack is
# active; another launcher can own an agent while publishing incompatible PX4
# setpoints. Require this variant's executable specifically.
controller_ready=0
for _ in $(seq 30); do
  if pgrep -f "autopilot_geometric_l1_direct_actuation_node" >/dev/null 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
if [[ $controller_ready -ne 1 ]]; then
  echo "ERROR: the geometric+L1 aerial-manipulator controller is not running." >&2
  echo "Start its external stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh $CFG_NAME" >&2
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

echo "Starting AM-T650 GEOMETRIC+L1 direct-actuator SITL with the ROS2 position-mode arm stack."
echo "Plant: AM_xfwd on T650 motors; controller: L1-augmented geometric SE(3)"
echo "  (Cai et al., CEP 2025) from the paired fsc_autopilot_ros2 L1 stack;"
echo "  arm commanded by fsc_open_manipulator PositionController (aerial config,"
echo "  home [0,40,40,0] deg) via open_manipulator_x_isaac_bridge;"
echo "  arm ground station: joint_plot_inverted"
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
ros2 launch open_manipulator_x_isaac_bridge position_control_isaac.launch.py namespace:=$ARM_NS
echo 'Arm ros2_control stack exited.'
exec bash
"

  tmux split-window -h -t "$SESSION:arm" "
$ARM_ENV
export DISPLAY='$ARM_DISPLAY'
echo 'Arm ground station (inverted). It discovers the active controller on its own.'
ros2 run utils_custom_ground_station joint_plot_inverted --ros-args -r __ns:=/$ARM_NS \\
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
