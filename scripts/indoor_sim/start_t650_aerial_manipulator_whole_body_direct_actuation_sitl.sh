#!/usr/bin/env bash
set -euo pipefail

# AERIAL-MANIPULATOR WHOLE-BODY direct-actuation simulation on the T650
# parameter set: the coupled airframe+arm impedance + GMO law
# (utils_controller/controller.py, ported to C++ in fsc_autopilot_ros2's
# single_aerial_manipulator_whole_body_direct_actuation fork) computes BOTH
# the four rotor commands and the four arm joint torques, with the arm in
# TORQUE mode through the fsc_open_manipulator stack.
#
# Incremental sibling of start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh
# (which keeps working unchanged). Three things differ:
#
#   * Isaac entrypoint: 06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py
#     (05's plant with the servo emulation replaced by external-effort
#     application + a PD+gravity stale fallback).
#   * The arm ros2_control stack runs the TORQUE bring-up
#     (torque_control_isaac.launch.py: IsaacTopicEffortSystem +
#     ExternalTorqueController) instead of the position one.
#   * It pairs with fsc_autopilot_ros2's
#       scripts/isaacsim/start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh
#     (started FIRST — it owns MicroXRCEAgent and brings the drone ground
#     station; node: autopilot_whole_body_direct_actuation_node, params:
#     params_single_aerial_manipulator_whole_body_direct_actuation_t650.yaml,
#     service ns: fsc_autopilot_ros2/whole_body_direct_actuation).
#
# The whole-body node subscribes /uav_0/fsc_open_manipulator/joint_states
# (arm state into the coupled law) and the ExternalTorqueController's
# smoothed_reference_joint_trajectory (the ONE activation/Home/joint/EE
# minimum-jerk reference its u3 tracks), and publishes joint torques to
# external_torque_controller/joint_torque_command — so BOTH ground stations
# are live: the drone GS flies the base, the arm GS poses the arm.
#
# MATCHED-MODEL VALIDATED BASELINE: restore the paired controller yaml's
# alloc_thrust_coeff to 4.679931e-05, take off in SAFETY to z = 1.2 m from
# the drone ground station, settle, then
# enter DIRECT from the Controller tab or:
#   ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode \
#     std_srvs/srv/SetBool "{data: true}"
# (the service REFUSES the switch until the hover/arm settle gates pass — the
# refusal message says which gate is red). Abort back to SAFETY with
# data: false. NEVER step a reference at the instant of a mode switch — the
# sequencing rule; see Command.md §7.14.
#
# CURRENT STRESS CONFIG (2026-08-22): the paired yaml deliberately believes a
# +15% kf while this Isaac plant keeps the calibrated truth. The preceding
# +20% test failed after 7.7 s. The retuned +15% hover-only case passed two
# clean 90 s guarded runs; keep the SAFETY abort ready for any non-hover task.
# Command.md §7.14.2 records the campaign.
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
# L1 launcher; the Isaac entrypoint and label differ — TORQUE-mode plant).
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py"
export INDOOR_SIM_VEHICLE_LABEL="AM-T650-WB"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"
# Fail before physics starts if this plant ever drifts from the mass used by
# the paired whole-body controller YAML.
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
  echo "Start the agent from the external controller stack before this launcher:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh" >&2
  exit 1
fi

# MicroXRCEAgent alone is not proof that the matching controller stack is
# active; another launcher can own an agent while publishing incompatible PX4
# setpoints. Require this variant's executable specifically.
controller_ready=0
for _ in $(seq 30); do
  if pgrep -f "autopilot_whole_body_direct_actuation_node" >/dev/null 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
if [[ $controller_ready -ne 1 ]]; then
  echo "ERROR: the whole-body aerial-manipulator controller is not running." >&2
  echo "Start its external stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh $CFG_NAME" >&2
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

echo "Starting AM-T650 WHOLE-BODY direct-actuator SITL with the ROS2 TORQUE-mode arm stack."
echo "Plant: AM_xfwd on T650 motors; controller: whole-body impedance + GMO"
echo -e "\033[1;33mNOTICE: paired controller carries a simulation-only +15% kf mismatch, validated for hover only; +20% fails.\033[0m"
echo "  (controller.py's law, C++ port) from the paired fsc_autopilot_ros2 stack;"
echo "  arm torques via fsc_open_manipulator ExternalTorqueController"
echo "  (torque bring-up) through open_manipulator_x_isaac_bridge's effort system;"
echo "  arm ground station: joint_plot_inverted (WB-TORQUE; controller:=external_torque_controller)"
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
echo 'Isaac arm is reporting; launching the TORQUE-mode ros2_control stack.'
ros2 launch open_manipulator_x_isaac_bridge torque_control_isaac.launch.py namespace:=$ARM_NS
echo 'Arm ros2_control stack exited.'
exec bash
"

  tmux split-window -h -t "$SESSION:arm" "
$ARM_ENV
export DISPLAY='$ARM_DISPLAY'
echo 'Arm ground station (inverted, torque mode). simulation:=true — Isaac reports N*m.'
ros2 run custom_gui joint_plot_inverted --ros-args -r __ns:=/$ARM_NS \\
  -p controller:=external_torque_controller -p simulation:=true \\
  -p mount_height:=$ARM_GS_MOUNT_HEIGHT \\
  -p fallback_min_deg:='[-35.0, -80.0, -40.0, -120.0]' \\
  -p fallback_max_deg:='[35.0, 50.0, 50.0, 120.0]'
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
