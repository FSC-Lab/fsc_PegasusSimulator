#!/usr/bin/env bash
set -euo pipefail

# AERIAL-MANIPULATOR direct-actuation simulation on the T650, flown by the
# MODULAR ADAPTIVE law of Yadav, Dantu, Pan, Sun, Roy, Baldi, "Modular Adaptive
# Aerial Manipulation Under Unknown Dynamic Coupling Forces", IEEE/ASME
# Trans. Mechatronics 30(4), 2025 (docs/comparison references/). Added
# 2026-09-30 for the SIMULATION comparison against the whole-body L1 impedance
# law; there is no hardware twin and none is planned. Command.md 7.22.
#
# THE PLANT IS IDENTICAL to the 4-D whole-body rig's
# (start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh):
# same Isaac entrypoint (06_px4_t650_aerial_manipulator_free_flight.py),
# same AM_xfwd asset on T650 motors, same PX4 profile, same 3.746170 kg mass
# gate, same TORQUE-mode fsc_open_manipulator stack (the paper's arm module
# commands joint TORQUES, tau_alpha, eq. 25a), same two ground stations. The
# plant knobs come from the modular yaml, whose section 1 is generated as a
# BYTE COPY of the whole-body yaml's (make_modular_yaml.py checks it), so the
# two rigs differ only in the DIRECT law. WB_SIM_PROFILE=mirror|robustness
# selects the same profile pair as the whole-body launchers.
#
# THE CONTROLLER SIDE differs in two checks:
#   * it waits for autopilot_modular_adaptive_direct_actuation_node and refuses
#     to start against a whole-body node (all three share the
#     whole_body_direct_actuation namespace, so a process-name gate is the
#     only thing that tells them apart);
#   * it reads mod_arm_lambda1 OFF THE RUNNING NODE -- a key only the modular
#     node declares -- so a flight is never mislabelled.
#
# PAIR WITH (started FIRST -- it owns MicroXRCEAgent):
#   fsc_autopilot_ros2/scripts/isaacsim/start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh
#
# Operating procedure is the whole-body rig's (same namespace, same service,
# same gates, same planner):
#   1. SAFETY takeoff to z = 1.2 m from the drone ground station, settle.
#   2. Confirm the autopilot pane shows the magenta "DIRECT LAW: MODULAR
#      ADAPTIVE" banner and the cyan LAW CHECK line (the seven places the paper
#      leaves a choice open, and what this run does at each).
#   3. Enter DIRECT from the Controller tab, or:
#        ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode \
#          std_srvs/srv/SetBool "{data: true}"
#      Entry is GATED on a settled hover; the refusal names every red gate.
#   4. Fly the EE trajectory from the arm GS's End-Effector Trajectory tab (or
#      application/robotic_arm/utils/am_compare_cycle.sh modular <tag> -- ...).
#      Internals: <ns>/fsc_autopilot_ros2/whole_body_direct_actuation/modular_control_debug
#      (layout in the client header); once a second the autopilot pane prints
#      rho, |r|, K_hat_0 and zeta of the three modules.
#
# Usage:
#   ./start_t650_aerial_manipulator_modular_adaptive_direct_actuation_sitl.sh <config_name>
#
# Optional:
#   WB_SIM_PROFILE=robustness   the stress plant (default mirror)
#   WB_SIM_YAML=<path>          a different modular yaml (plant knobs read from it)
#   T650_AERIAL_MANIPULATOR_DIRECT_ACTUATOR_PARAM_DELAY=8

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
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/06_px4_t650_aerial_manipulator_free_flight.py"
export INDOOR_SIM_VEHICLE_LABEL="AM-T650-MODULAR"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"
# Fail before physics starts if this plant ever drifts from the mass used by
# the paired whole-body controller YAML.
export PEGASUS_EXPECTED_TOTAL_MASS="3.746170"
case "${WB_SIM_PROFILE:-mirror}" in
  mirror)     _MOD_SUFFIX="_sim.yaml" ;;
  robustness) _MOD_SUFFIX="_sim_robustness.yaml" ;;
  *) echo "ERROR: WB_SIM_PROFILE must be 'mirror' or 'robustness' (got '${WB_SIM_PROFILE}')" >&2; exit 2 ;;
esac
export WB_SIM_YAML="${WB_SIM_YAML:-$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_modular_adaptive_direct_actuation_t650${_MOD_SUFFIX}}"

# -- THE PLANT KNOBS (arm servo model, model-uncertainty injections, gearbox
# friction, arm gravity mismatch and the 2026-09-26 sim-to-real mirror terms)
# are read from the paired controller yaml's REALITY MODEL section by ONE
# library, shared with the decoupled rig. It also settles WHICH yaml:
# WB_SIM_PROFILE=mirror (default, ..._sim.yaml = the experiment mirror) or
# robustness (..._sim_robustness.yaml = the stress plant), WB_SIM_YAML overrides.
# Precedence for every knob: environment > yaml > built-in default.
# shellcheck source=lib/am_plant_from_yaml.sh
source "$SCRIPT_DIR/indoor_sim/lib/am_plant_from_yaml.sh"

# ── EE MARKER CUBE (2026-09-18, user request) ────────────────────────────────
# sim_ee_marker_cube: true welds a cube into the gripper at spawn. Isaac
# publishes it as the mocap body obj_0, the emulator turns that into
# /obj_0/mocap, and the arm ground station draws it as "EE (Meas)" beside the
# forward-kinematics "EE (FK)". NEVER enters any control law -- it is ground
# truth for the end-effector, and an UNMODELLED end-effector payload.
# false = no cube, the plant exactly as before. Same precedence: env > yaml.
if [[ -z "${PEGASUS_EE_MARKER_CUBE:-}" ]]; then
  case "$(yaml_scalar sim_ee_marker_cube)" in
    true|True|TRUE|1) export PEGASUS_EE_MARKER_CUBE=1 ;;
    *)                export PEGASUS_EE_MARKER_CUBE=0 ;;
  esac
fi
if [[ -z "${PEGASUS_EE_MARKER_CUBE_MASS:-}" ]]; then
  V="$(yaml_scalar sim_ee_marker_cube_mass_kg)"
  [[ -n "$V" ]] && export PEGASUS_EE_MARKER_CUBE_MASS="$V"
fi
if [[ "$PEGASUS_EE_MARKER_CUBE" == 1 ]]; then
  echo -e "\033[1;35mEE MARKER CUBE ACTIVE: ${PEGASUS_EE_MARKER_CUBE_MASS:-0.2} kg welded into the gripper, published as obj_0 -> /obj_0/mocap (measured EE pose). Unmodelled by every controller.\033[0m"
else
  echo "EE marker cube off (no end-effector payload, no /obj_0/mocap)."
fi

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
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh" >&2
  exit 1
fi

# MicroXRCEAgent alone is not proof that the matching controller stack is
# active; another launcher can own an agent while publishing incompatible PX4
# setpoints. Require this variant's executable specifically.
# The two whole-body executables differ ONLY in this name -- neither is a
# substring of the other -- so this is what keeps the L1 rig and the GMO rig
# from being flown against each other by accident.
for other in autopilot_whole_body_l1_direct_actuation_node autopilot_whole_body_direct_actuation_node; do
  if pgrep -f "$other" >/dev/null 2>&1; then
    echo "ERROR: '$other' is running -- that is a WHOLE-BODY rig on the same namespace." >&2
    echo "       Stop its stack and start the modular one:" >&2
    echo "  fsc_autopilot_ros2/scripts/isaacsim/start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh $CFG_NAME" >&2
    exit 1
  fi
done
controller_ready=0
for _ in $(seq 30); do
  if pgrep -f "autopilot_modular_adaptive_direct_actuation_node" >/dev/null 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
if [[ $controller_ready -ne 1 ]]; then
  echo "ERROR: the modular adaptive aerial-manipulator controller is not running." >&2
  echo "Start its external stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh $CFG_NAME" >&2
  exit 1
fi
# POSITIVE confirmation off the running node: mod_arm_lambda1 exists only on the
# modular node, so a readable value proves which law is flying.
MOD_UAV_NS="${INDOOR_SIM_UAV_NS:-uav_0}"
MOD_ANSWER="$(
  set +u
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" >/dev/null 2>&1
  source "$FSC_AUTOPILOT_WS/install/setup.bash" >/dev/null 2>&1
  for _ in $(seq 20); do
    ans="$(timeout 8 ros2 param get "/$MOD_UAV_NS/fsc_autopilot_ros2" mod_arm_lambda1 2>/dev/null | tr -d '\r')"
    case "$ans" in *"Double value is"*) echo "$ans" | sed 's/.*is: //'; exit 0 ;; esac
    sleep 1
  done
  echo unknown
)"
if [[ "$MOD_ANSWER" == unknown ]]; then
  echo "ERROR: could not read mod_arm_lambda1 from /$MOD_UAV_NS/fsc_autopilot_ros2 (is the" >&2
  echo "       autopilot workspace at \$FSC_AUTOPILOT_WS=$FSC_AUTOPILOT_WS built, and the node" >&2
  echo "       running under that namespace?). Refusing to guess which law is flying." >&2
  exit 1
fi
echo -e "\033[1;35mRunning node confirms the MODULAR ADAPTIVE law (mod_arm_lambda1 = $MOD_ANSWER).\033[0m"
export PEGASUS_PX4_LOCKSTEP=0
if tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null; then
  echo "Pegasus PX4 lockstep: pushed to the tmux server environment"
else
  echo "Pegasus PX4 lockstep: no tmux server yet; the new session will inherit the export"
fi

case "$PEGASUS_ARM_SERVO_MODEL" in
  ideal) echo -e "\033[1;33mArm servo model: IDEAL - current-loop residual OFF (commanded effort applied exactly).\033[0m" ;;
  *) echo "Arm servo model: CURRENT LOOP on j2/j3 only (residual ${PEGASUS_ARM_CURRENT_NOISE_A:-servo_model default} A rms per joint @ ${PEGASUS_ARM_CURRENT_NOISE_BW_HZ:-5.0} Hz, seed ${PEGASUS_ARM_CURRENT_NOISE_SEED:-0})"
     echo "  counts/N.m: calibrated 2026-09-11 [162.4, 154.0, 150.5, 153.4] (was [160.0, 173.8, 146.7, 160.0])" ;;
esac

echo "Starting AM-T650 MODULAR ADAPTIVE direct-actuator SITL with the ROS2 TORQUE-mode arm stack."
echo -e "\033[1;35mDIRECT law: Yadav et al. (IEEE/ASME TMECH 2025) -- three model-free adaptive sliding-mode modules (base position, attitude, arm joints).\033[0m"
echo "Plant: AM_xfwd on T650 motors -- the 4-D whole-body rig's plant, section 1 of $(basename "$WB_SIM_YAML")"
if [[ "${WB_SIM_PROFILE:-mirror}" == robustness ]]; then
  echo -e "\033[1;33mNOTICE: robustness profile -- the controller carries a simulation-only +17.6% kf mismatch and the config-A plant injections.\033[0m"
else
  echo -e "\033[1;36mNOTICE: mirror profile -- the plant is the one identified from the 0918/0921/0924 flights.\033[0m"
fi
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
# The gamepad node lives in the AUTOPILOT workspace (px4_offboard_control),
# not the arm one, so the joy window gets its own environment. Sourcing the
# wrong overlay here fails with a bare "package not found" several seconds
# after launch, in a detached window nobody is watching.
JOY_ENV="source '$ARM_ROS2_SETUP'; source '$FSC_AUTOPILOT_WS/install/setup.bash'"

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
ros2 launch open_manipulator_x_isaac_bridge torque_control_isaac.launch.py namespace:=$ARM_NS passthrough_corrections:=$ARM_PT_CORR
echo 'Arm ros2_control stack exited.'
exec bash
"

  tmux split-window -h -t "$SESSION:arm" "
$ARM_ENV
export DISPLAY='$ARM_DISPLAY'
echo 'Arm ground station (inverted, torque mode). simulation:=true — Isaac reports N*m.'
ros2 run utils_custom_ground_station joint_plot_inverted --ros-args -r __ns:=/$ARM_NS \\
  -p controller:=external_torque_controller -p simulation:=true \\
  -p mount_height:=$ARM_GS_MOUNT_HEIGHT \\
  -p fallback_min_deg:='[-35.0, -80.0, -40.0, -120.0]' \\
  -p fallback_max_deg:='[35.0, 50.0, 50.0, 120.0]'
echo 'Arm ground station exited.'
exec bash
"

  # ── PS4 REMOTE (2026-09-20) ───────────────────────────────────────────────
  # joy_node + gamepad_input, so the arm station's "PS4 Remote" tab has a
  # gamepad to read. Gated on the device node existing, and non-fatal: a
  # missing pad must not stop a flight that was never going to use one, and
  # the tab is inert until the operator engages it. PEGASUS_JOY_DEVICE is
  # SDL's joystick INDEX, not a /dev path.
  #
  # Skipped when a gamepad_input is ALREADY running (the §7.19.1 order starts
  # the joystick by hand before the stack): a second joy_node on the same pad
  # would publish every stick sample twice on <ns>/rc/input. The bracket keeps
  # pgrep from matching this subshell's own command line.
  if pgrep -f '[l]ib/px4_offboard_control/gamepad_input' >/dev/null 2>&1; then
    echo -e "\033[1;36mgamepad_input already running (started by hand) -- not opening a second joy window.\033[0m"
  elif [[ -e "${PEGASUS_JOY_DEV_NODE:-/dev/input/js0}" ]]; then
    tmux new-window -d -t "$SESSION" -n joy "
$JOY_ENV
echo 'PS4 gamepad: joy_node -> /${ARM_NS%%/*}/rc/input (the PS4 Remote tab reads this).'
echo 'If a stick moves the WRONG pair of numbers on that tab, this connection'
echo 'orders its axes differently -- set ps4_axis_* / ps4_invert_* on the arm'
echo 'ground station rather than editing code.'
ros2 launch px4_offboard_control gamepad_input.launch.py device_id:=${PEGASUS_JOY_DEVICE:-0} ns:=/${ARM_NS%%/*}
echo 'gamepad_input exited -- PS4 Remote will read no data.'
exec bash
"
  else
    echo -e "\033[1;33mNo gamepad at ${PEGASUS_JOY_DEV_NODE:-/dev/input/js0}; the PS4 Remote tab will read 'no data'.\033[0m"
  fi
) &

# Apply the wall-clock DDS timestamp and HIL auto-disarm settings after the PX4
# shell is ready. These changes intentionally remain per-run and are not saved.
"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

# Reuse the validated indoor PX4/Isaac orchestration and cleanup. Passing
# --in-terminal prevents the base launcher from opening a second terminal.
exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
