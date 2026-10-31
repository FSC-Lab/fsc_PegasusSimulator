#!/usr/bin/env bash
set -euo pipefail

# AERIAL-MANIPULATOR direct-actuation simulation on the T650 parameter set for
# the ARBITRARY CONVERGENCE TIME controller (Chen, Liu, Cai, Shang, Chang, Chao,
# Wang, Shen, "Arbitrary Convergence Time Control of Unmanned Aerial Manipulator
# for Active Interaction and Dynamic Tracking", IEEE TIE 72(12) 2025): ACTNDO +
# TSKFE + the quadrotor's and the manipulator's arbitrary-time controllers in
# ONE flight node, the ARM COMMANDED IN JOINT TORQUE over ROS 2 through the
# fsc_open_manipulator TORQUE-mode stack (the paper's arm law outputs tau_m,
# eq 37). The SIMULATION comparison rig for the box-pushing task (2026-10-24);
# no hardware twin. Independent of the whole-body controller.
#
# The Pegasus/PX4/arm side is the torque-mode one every whole-body rig flies --
# 06 (or the scene AM_ISAAC_SCENE_SCRIPT names) with the arm in effort mode, the
# same PX4 profile, the torque-mode ros2_control stack + arm ground station --
# with the arm config swapped for torque_controller_isaac_aerial_atc.yaml (its
# arm_planner follows THIS rig's mode topic). It pairs with:
#
#   fsc_autopilot_ros2/scripts/isaacsim/start_arbitrary_time_direct_actuation_t650_aerial_manipulator_stack.sh
#   (started FIRST; node: autopilot_arbitrary_time_direct_actuation_node,
#   params: params_single_aerial_manipulator_arbitrary_time_direct_actuation_t650_sim_box_pushing.yaml,
#   service ns: fsc_autopilot_ros2/arbitrary_time_direct_actuation)
#
# The plant knobs (section 1 of that yaml, a byte copy of the whole-body 4-D
# box-pushing sim yaml's) are read by the shared lib below, so both rigs fly the
# same plant. The box scene: start_t650_aerial_manipulator_arbitrary_time_box_pushing_sitl.sh.

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
ARM_GS_TAB_ARG=""
if [[ -n "${ARM_GS_START_TAB:-}" ]]; then
  [[ "$ARM_GS_START_TAB" =~ ^[0-9]+$ ]] || { echo "ERROR: ARM_GS_START_TAB must be a tab index, got '$ARM_GS_START_TAB'" >&2; exit 1; }
  ARM_GS_TAB_ARG="-p start_tab:=$ARM_GS_START_TAB"
fi
# The Box Pushing tab's automatic exit / Ready / Push (the station's defaults): the
# autonomous driver (run_pl_atc.sh) flies those steps itself and sets
# ARM_GS_AUTO_EXIT_DELAY=0.0 ARM_GS_AUTO_STEPS=false. Unset = the station's default.
if [[ -n "${ARM_GS_AUTO_EXIT_DELAY:-}" ]]; then
  [[ "$ARM_GS_AUTO_EXIT_DELAY" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "ERROR: ARM_GS_AUTO_EXIT_DELAY must be seconds >= 0, got '$ARM_GS_AUTO_EXIT_DELAY'" >&2; exit 1; }
  [[ "$ARM_GS_AUTO_EXIT_DELAY" == *.* ]] || ARM_GS_AUTO_EXIT_DELAY="$ARM_GS_AUTO_EXIT_DELAY.0"
  ARM_GS_TAB_ARG="$ARM_GS_TAB_ARG -p push_pull_auto_exit_delay_s:=$ARM_GS_AUTO_EXIT_DELAY"
fi
if [[ -n "${ARM_GS_AUTO_STEPS:-}" ]]; then
  [[ "$ARM_GS_AUTO_STEPS" == "true" || "$ARM_GS_AUTO_STEPS" == "false" ]] || { echo "ERROR: ARM_GS_AUTO_STEPS must be true or false, got '$ARM_GS_AUTO_STEPS'" >&2; exit 1; }
  ARM_GS_TAB_ARG="$ARM_GS_TAB_ARG -p push_pull_auto_ready:=$ARM_GS_AUTO_STEPS -p push_pull_auto_push:=$ARM_GS_AUTO_STEPS"
fi
# The aerial TORQUE-mode config with arm_planner's mode topic pointed at THIS
# rig's flight node (a copy of torque_controller_isaac_aerial.yaml, one line).
ARM_CONFIG="${FSC_ATC_ARM_CONFIG:-$ARM_WS/src/fsc_open_manipulator/open_manipulator_x_isaac_bridge/config/torque_controller_isaac_aerial_atc.yaml}"

export INDOOR_SIM_PEGASUS_SCRIPT="${AM_ISAAC_SCENE_SCRIPT:-$REPO_ROOT/application/robotic_arm/06_px4_t650_aerial_manipulator_free_flight.py}"
export INDOOR_SIM_VEHICLE_LABEL="${AM_ISAAC_SCENE_LABEL:-AM-T650-ATC}"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"
export PEGASUS_ARM_COMMAND_MODE=effort          # the paper's arm law commands joint torques
export PEGASUS_EXPECTED_TOTAL_MASS="3.746170"

# ── PLANT KNOBS FROM THE CONTROLLER YAML (section 1, REALITY MODEL) ─────────
case "${WB_SIM_PROFILE:-box_pushing}" in
  box_pushing|push_pull|push-pull) ATC_SUFFIX="_sim_box_pushing.yaml" ;;
  *) echo "ERROR: WB_SIM_PROFILE must be box_pushing for this rig (got '${WB_SIM_PROFILE}')" >&2; exit 1 ;;
esac
export WB_SIM_PROFILE="${WB_SIM_PROFILE:-box_pushing}"
WB_SIM_YAML="${WB_SIM_YAML:-$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_arbitrary_time_direct_actuation_t650$ATC_SUFFIX}"
[[ -r "$WB_SIM_YAML" ]] || { echo "ERROR: plant yaml not readable: $WB_SIM_YAML" >&2; exit 1; }
# shellcheck source=/dev/null
source "$SCRIPT_DIR/indoor_sim/lib/am_plant_from_yaml.sh"
echo -e "\033[1;36mPlant knobs read from $(basename "$WB_SIM_YAML"); arm command mode: effort (joint torques)\033[0m"
# THE ARM STACK'S FRICTION FEED-FORWARD on the streamed torque (passthrough_auxiliary_terms).
# The shared lib turns it on whenever the plant carries gearbox friction (the whole-body
# rigs' arrangement). For THIS rig it is OFF by default: the paper's ACTNDO already
# estimates the joint friction as part of tau_ext, and the stack's measured-velocity relay
# on top of it is a second, discontinuous compensation -- flight atc_03 (2026-10-24) limit-
# cycled j2/j3 at 0.8 Hz with it on, atc_04 did not with it off (README section 4).
# ATC_ARM_FRICTION_FF=true|false overrides; the plant's gearbox friction is unchanged either way.
if [[ -n "${ATC_ARM_FRICTION_FF:-}" ]]; then
  [[ "$ATC_ARM_FRICTION_FF" == true || "$ATC_ARM_FRICTION_FF" == false ]] || { echo "ERROR: ATC_ARM_FRICTION_FF must be true or false (got '$ATC_ARM_FRICTION_FF')" >&2; exit 2; }
  ARM_PT_CORR="$ATC_ARM_FRICTION_FF"
  echo -e "\033[1;33mArm-side friction feed-forward forced to $ARM_PT_CORR (ATC_ARM_FRICTION_FF); the plant's gearbox friction is unchanged.\033[0m"
else
  ARM_PT_CORR=false
  echo -e "\033[1;36mArm-side friction feed-forward OFF for this rig (the ACTNDO estimates the friction; ATC_ARM_FRICTION_FF=true to turn it on).\033[0m"
fi

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
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_arbitrary_time_direct_actuation_t650_aerial_manipulator_stack.sh" >&2
  exit 1
fi

controller_ready=0
for _ in $(seq 30); do
  if pgrep -f "autopilot_arbitrary_time_direct_actuation_node" >/dev/null 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
if [[ $controller_ready -ne 1 ]]; then
  echo "ERROR: the arbitrary-time controller is not running. Start its stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_arbitrary_time_direct_actuation_t650_aerial_manipulator_stack.sh $CFG_NAME" >&2
  exit 1
fi

export PEGASUS_PX4_LOCKSTEP=0
if tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null; then
  echo "Pegasus PX4 lockstep: pushed to the tmux server environment"
else
  echo "Pegasus PX4 lockstep: no tmux server yet; the new session will inherit the export"
fi

echo "Starting AM-T650 ARBITRARY-TIME (Chen et al. IEEE TIE 2025) SITL with the ROS2 torque-mode arm stack."
echo "Plant: $(basename "$INDOOR_SIM_PEGASUS_SCRIPT"), arm in effort-command mode; profile ${WB_SIM_PROFILE}"
echo "Arm workspace: $ARM_WS; arm config $(basename "$ARM_CONFIG") (arm-side friction compensation $ARM_PT_CORR)"

ARM_ENV="source '$ARM_ROS2_SETUP'; if [ -f '$ARM_ROSDEPS_SETUP' ]; then source '$ARM_ROSDEPS_SETUP'; fi; source '$ARM_WS_SETUP'"
ARM_DISPLAY="${DISPLAY:-:0}"

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
echo 'Isaac arm is reporting; launching the TORQUE-mode ros2_control stack (arm_planner follows arbitrary_time_direct_actuation/mode).'
ros2 launch open_manipulator_x_isaac_bridge torque_control_isaac.launch.py namespace:=$ARM_NS passthrough_corrections:=$ARM_PT_CORR controller_config:=$ARM_CONFIG
echo 'Arm ros2_control stack exited.'
exec bash
"

  tmux split-window -h -t "$SESSION:arm" "
$ARM_ENV
export DISPLAY='$ARM_DISPLAY'
echo 'Arm ground station (inverted, torque mode). simulation:=true -- Isaac reports N*m.'
ros2 run utils_custom_ground_station joint_plot_inverted --ros-args -r __ns:=/$ARM_NS \\
  -p controller:=external_torque_controller -p simulation:=true \\
  -p mount_height:=$ARM_GS_MOUNT_HEIGHT $ARM_GS_TAB_ARG \\
  -p fallback_min_deg:='[-35.0, -80.0, -40.0, -120.0]' \\
  -p fallback_max_deg:='[35.0, 50.0, 50.0, 120.0]'
echo 'Arm ground station exited.'
exec bash
"
) &

"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
