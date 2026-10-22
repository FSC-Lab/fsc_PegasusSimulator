#!/usr/bin/env bash
set -euo pipefail

# AERIAL-MANIPULATOR direct-actuation simulation on the T650 parameter set for
# the COORDINATED-ADMITTANCE controller (Liu, Lyu, Guo, Wang, Yu, Guo, "A
# coordinated framework of aerial manipulator for safe and compliant physical
# interaction", CEP 146 (2024) 105898): decoupled PD/flatness UAV law +
# momentum wrench estimator in the flight node, admittance filter + adaptive
# weights + kinematic MPC in a separate planner node, the ARM COMMANDED OVER
# ROS 2 by the fsc_open_manipulator POSITION-MODE stack (the paper's servo PD).
# The SIMULATION comparison rig for the box-pushing task (2026-10-10).
#
# Incremental sibling of start_t650_aerial_manipulator_geometric_L1_adaptive_direct_actuation_sitl.sh
# (the other decoupled rig; it keeps working unchanged): the Pegasus/PX4/arm
# side is IDENTICAL -- 06 (or the scene AM_ISAAC_SCENE_SCRIPT names) with the
# arm in position-command mode, the same PX4 profile, the same position-mode
# ros2_control stack + arm ground station -- and the only difference is the
# external controller stack it pairs with:
#
#   fsc_autopilot_ros2/scripts/isaacsim/start_coordinated_admittance_direct_actuation_t650_aerial_manipulator_stack.sh
#   (started FIRST; node: autopilot_coordinated_admittance_direct_actuation_node,
#   params: params_single_aerial_manipulator_coordinated_admittance_direct_actuation_t650_sim*.yaml,
#   service ns: fsc_autopilot_ros2/coordinated_admittance_direct_actuation)
#
# The plant knobs (section 1 of that yaml, a byte copy of the whole-body 4-D
# sim yaml's) are read by the shared lib below, so both rigs fly the same plant.
# WB_SIM_PROFILE=box_pushing (old name push_pull) selects the box-pushing pair (the
# scene wrapper start_t650_aerial_manipulator_coordinated_admittance_box_pushing_sitl.sh sets it).

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

BASE_LAUNCHER="$SCRIPT_DIR/indoor_sim/lib/start_single_drone_x650.sh"
PARAM_SCRIPT="$SCRIPT_DIR/apply_aerial_manipulator_px4_offboard_params.sh"
SESSION="px4_isaac"
PARAM_DELAY="${T650_AERIAL_MANIPULATOR_DIRECT_ACTUATOR_PARAM_DELAY:-8}"

ARM_WS="${FSC_OM_ARM_WS:-$HOME/ros2_ws}"
ARM_ROSDEPS_SETUP="${FSC_OM_ARM_ROSDEPS_SETUP:-$HOME/ros2_ws/rosdeps/local_setup.bash}"
ARM_ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
ARM_WS_SETUP="$ARM_WS/install/setup.bash"
ARM_NS="uav_0/fsc_open_manipulator"
ARM_STATE_TOPIC="/uav_0/isaacsim_manipulator/joint_states"
ARM_GS_MOUNT_HEIGHT="${ARM_GS_MOUNT_HEIGHT:-1.2}"
# the tab the arm ground station opens on (the whole-body launcher's convention, 2026-10-17:
# the valve-turning scene wrapper sets 7 = Valve Turning); unset = the station's own default
ARM_GS_TAB_ARG=""
if [[ -n "${ARM_GS_START_TAB:-}" ]]; then
  [[ "$ARM_GS_START_TAB" =~ ^[0-9]+$ ]] || { echo "ERROR: ARM_GS_START_TAB must be a tab index, got '$ARM_GS_START_TAB'" >&2; exit 1; }
  ARM_GS_TAB_ARG="-p start_tab:=$ARM_GS_START_TAB"
fi
# The Box Pushing tab's AUTOMATIC exit (2026-10-19; the station's
# push_pull_auto_exit_delay_s, 1.0 s after DONE push, <= 0 = by hand only). The
# autonomous drivers (pl_mission.py, run by run_pl.sh / run_pl_cadm.sh) release, open
# and exit by themselves and set ARM_GS_AUTO_EXIT_DELAY=0.0 so the station does not
# race them. Unset = the station's default.
if [[ -n "${ARM_GS_AUTO_EXIT_DELAY:-}" ]]; then
  [[ "$ARM_GS_AUTO_EXIT_DELAY" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "ERROR: ARM_GS_AUTO_EXIT_DELAY must be seconds >= 0, got '$ARM_GS_AUTO_EXIT_DELAY'" >&2; exit 1; }
  [[ "$ARM_GS_AUTO_EXIT_DELAY" == *.* ]] || ARM_GS_AUTO_EXIT_DELAY="$ARM_GS_AUTO_EXIT_DELAY.0"   # a double parameter
  ARM_GS_TAB_ARG="$ARM_GS_TAB_ARG -p push_pull_auto_exit_delay_s:=$ARM_GS_AUTO_EXIT_DELAY"
fi
# The Box Pushing tab's AUTOMATIC Ready To Push + Push (2026-10-21; the station's
# push_pull_auto_ready / push_pull_auto_push, both true by default). The autonomous drivers
# fly those steps themselves and set ARM_GS_AUTO_STEPS=false. Unset = the station's default.
if [[ -n "${ARM_GS_AUTO_STEPS:-}" ]]; then
  [[ "$ARM_GS_AUTO_STEPS" == "true" || "$ARM_GS_AUTO_STEPS" == "false" ]] || { echo "ERROR: ARM_GS_AUTO_STEPS must be true or false, got '$ARM_GS_AUTO_STEPS'" >&2; exit 1; }
  ARM_GS_TAB_ARG="$ARM_GS_TAB_ARG -p push_pull_auto_ready:=$ARM_GS_AUTO_STEPS -p push_pull_auto_push:=$ARM_GS_AUTO_STEPS"
fi
# The aerial position-mode config with arm_planner's mode topic pointed at THIS
# rig's flight node (a copy of position_controller_isaac_aerial.yaml, one line).
ARM_CONFIG="${FSC_CADM_ARM_CONFIG:-$ARM_WS/src/fsc_open_manipulator/open_manipulator_x_isaac_bridge/config/position_controller_isaac_aerial_cadm.yaml}"

export INDOOR_SIM_PEGASUS_SCRIPT="${AM_ISAAC_SCENE_SCRIPT:-$REPO_ROOT/application/robotic_arm/06_px4_t650_aerial_manipulator_free_flight.py}"
export INDOOR_SIM_VEHICLE_LABEL="${AM_ISAAC_SCENE_LABEL:-AM-T650-CADM}"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"
export PEGASUS_ARM_COMMAND_MODE="${PEGASUS_ARM_COMMAND_MODE:-position}"
export PEGASUS_EXPECTED_TOTAL_MASS="3.746170"

# ── PLANT KNOBS FROM THE CONTROLLER YAML (section 1, REALITY MODEL) ─────────
case "${WB_SIM_PROFILE:-mirror}" in
  mirror)              CADM_SUFFIX="_sim.yaml" ;;
  box_pushing|push_pull|push-pull) CADM_SUFFIX="_sim_box_pushing.yaml" ;;
  valve_turning) CADM_SUFFIX="_sim_valve_turning.yaml" ;;   # 2026-10-17: the valve-turning scene wrapper sets it
  *) echo "ERROR: WB_SIM_PROFILE must be mirror, box_pushing or valve_turning for this rig (got '${WB_SIM_PROFILE}')" >&2; exit 1 ;;
esac
WB_SIM_YAML="${WB_SIM_YAML:-$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_coordinated_admittance_direct_actuation_t650$CADM_SUFFIX}"
[[ -r "$WB_SIM_YAML" ]] || { echo "ERROR: plant yaml not readable: $WB_SIM_YAML" >&2; exit 1; }
# the shared lib validates WB_SIM_PROFILE against its own list; "mirror" and
# "box_pushing" (and its old name "push_pull") are all in it
# shellcheck source=/dev/null
source "$SCRIPT_DIR/indoor_sim/lib/am_plant_from_yaml.sh"
echo -e "\033[1;36mPlant knobs read from $(basename "$WB_SIM_YAML"); arm command mode: $PEGASUS_ARM_COMMAND_MODE\033[0m"

[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || { echo "ERROR: missing Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2; exit 1; }
[[ -f "$ARM_WS_SETUP" ]] || { echo "ERROR: the arm workspace is not built: $ARM_WS_SETUP" >&2; exit 1; }
[[ -f "$ARM_CONFIG" ]] || { echo "ERROR: missing arm config $ARM_CONFIG" >&2; exit 1; }
if [[ ! "$PARAM_DELAY" =~ ^[0-9]+$ ]]; then
  echo "ERROR: T650_AERIAL_MANIPULATOR_DIRECT_ACTUATOR_PARAM_DELAY must be a non-negative integer." >&2
  exit 2
fi

if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo "ERROR: MicroXRCEAgent is not running. Start the controller stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_coordinated_admittance_direct_actuation_t650_aerial_manipulator_stack.sh" >&2
  exit 1
fi

controller_ready=0
for _ in $(seq 30); do
  if pgrep -f "autopilot_coordinated_admittance_direct_actuation_node" >/dev/null 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
if [[ $controller_ready -ne 1 ]]; then
  echo "ERROR: the coordinated-admittance controller is not running. Start its stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_coordinated_admittance_direct_actuation_t650_aerial_manipulator_stack.sh $CFG_NAME" >&2
  exit 1
fi
# The planner node is this rig's whole coordination: without it nothing moves in DIRECT.
if ! pgrep -f "coordinated_admittance_planner" >/dev/null 2>&1; then
  echo "ERROR: coordinated_admittance_planner.py is not running (the stack's planner window)." >&2
  exit 1
fi

export PEGASUS_PX4_LOCKSTEP=0
if tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null; then
  echo "Pegasus PX4 lockstep: pushed to the tmux server environment"
else
  echo "Pegasus PX4 lockstep: no tmux server yet; the new session will inherit the export"
fi

echo "Starting AM-T650 COORDINATED-ADMITTANCE (Liu et al. CEP 2024) SITL with the ROS2 position-mode arm stack."
echo "Plant: $(basename "$INDOOR_SIM_PEGASUS_SCRIPT"), arm in $PEGASUS_ARM_COMMAND_MODE-command mode; profile ${WB_SIM_PROFILE:-mirror}"
echo "Arm workspace: $ARM_WS"

ARM_ENV="source '$ARM_ROS2_SETUP'; if [ -f '$ARM_ROSDEPS_SETUP' ]; then source '$ARM_ROSDEPS_SETUP'; fi; source '$ARM_WS_SETUP'"
ARM_DISPLAY="${DISPLAY:-:0}"
JOY_ENV="source '$ARM_ROS2_SETUP'; source '$FSC_AUTOPILOT_WS/install/setup.bash'"

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
echo 'Isaac arm is reporting; launching the position-mode ros2_control stack (pure tracking + arm_planner).'
echo 'arm config: $(basename "$ARM_CONFIG") (arm_planner follows coordinated_admittance_direct_actuation/mode).'
ros2 launch open_manipulator_x_isaac_bridge position_control_isaac.launch.py namespace:=$ARM_NS arm_planner:=true controller_config:=$ARM_CONFIG
echo 'Arm ros2_control stack exited.'
exec bash
"

  tmux split-window -h -t "$SESSION:arm" "
$ARM_ENV
export DISPLAY='$ARM_DISPLAY'
echo 'Arm ground station (inverted).'
ros2 run utils_custom_ground_station joint_plot_inverted --ros-args -r __ns:=/$ARM_NS \\
  -p controller:=position_controller -p torque_constant:=1.81 -p mount_height:=$ARM_GS_MOUNT_HEIGHT $ARM_GS_TAB_ARG \\
  -p fallback_min_deg:='[-35.0, -80.0, -40.0, -120.0]' \\
  -p fallback_max_deg:='[35.0, 50.0, 50.0, 120.0]'
echo 'Arm ground station exited.'
exec bash
"

  if pgrep -f '[l]ib/px4_offboard_control/gamepad_input' >/dev/null 2>&1; then
    echo -e "\033[1;36mgamepad_input already running -- not opening a second joy window.\033[0m"
  elif [[ -e "${PEGASUS_JOY_DEV_NODE:-/dev/input/js0}" ]]; then
    tmux new-window -d -t "$SESSION" -n joy "
$JOY_ENV
ros2 launch px4_offboard_control gamepad_input.launch.py device_id:=${PEGASUS_JOY_DEVICE:-0} ns:=/${ARM_NS%%/*}
exec bash
"
  fi
) &

"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
