#!/usr/bin/env bash
set -euo pipefail

# PHYSICAL-INTERACTION VARIANT (2026-09-27) of
# start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh.
# A COPY with exactly two lines changed: the Isaac entrypoint is
# 07_px4_direct_t650_aerial_manipulator_interaction.py (06's plant + an
# end-effector force injector on /uav_0/isaacsim_manipulator/ee_force_cmd,
# ground truth on .../ee_force_state) and the vehicle label. Everything else --
# plant knobs from the paired yaml (WB_SIM_YAML / WB_SIM_PROFILE), the 4-D node
# check, the arm stack, the ground stations -- is the parent's, verbatim.
# Campaign: docs/docs_aerial_manipulator/interaction_20260927/.
#
# ---- the parent's header follows -------------------------------------------

# AERIAL-MANIPULATOR WHOLE-BODY direct-actuation simulation on the T650, flown
# with the L1 ADAPTIVE AUGMENTED DISTURBANCE OBSERVER and the FOUR-DIMENSIONAL
# attribution of the working note's September 2026 revision ("the
# four-dimensional interaction wrench", disturbance_observer_draft.tex).
# Added 2026-09-16; Command.md 7.17.
#
# THE PLANT IS IDENTICAL to start_t650_aerial_manipulator_whole_body_L1_adaptive_6D_direct_actuation_sitl.sh
# (the six-dimensional rig), which is itself identical to the GMO rig's: same
# Isaac entrypoint (06_px4_t650_aerial_manipulator_free_flight.py),
# same AM_xfwd asset on T650 motors, same PX4 profile, same 3.746170 kg mass
# gate, same arm servo model, same plant-uncertainty injection (the standing
# config A: +15 % allocator kf, body mass/inertia x1.10 + 10/10/5 mm CoM shift,
# MN4010 rotor lag, current-loop residual, gearbox friction x1.05, arm mass
# x1.05). The same TORQUE-mode fsc_open_manipulator stack and the same two
# ground stations come up. Only two things differ, both on the CONTROLLER side:
#
#   * it waits for autopilot_whole_body_l1_direct_actuation_node -- the SAME
#     executable as the 6-D rig, because the attribution is a yaml switch --
#     and then READS wb_l1_four_d OFF THE RUNNING NODE, refusing to start
#     against the 6-D stack (a process-name gate cannot tell them apart);
#   * it reads its plant knobs out of the 4-D yaml
#     (params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml),
#     which is the 6-D yaml verbatim plus the wb_l1_four_d block.
#
# WHAT THE 4-D ATTRIBUTION IS. The 6-D design separated the contact wrench from
# the internal disturbance with a metric-weighted projector and a dynamic
# identifier on the four wrench-free directions; in free flight it reported a
# phantom F_hat_y of 0.04-0.13 N. The 4-D design reads the task force off the
# JOINT ROWS of the residual: a platform wrench (thrust deficit, CoM moment,
# gust) has [w]_q = 0 EXACTLY, so
#     F_hat = J_yq^-T ([w_Sigma]_q - w_hat_q),   F_hat_y = (1 - chi) C_x F_hat,
# with w_hat_q the joint-row residual trimmed in free flight and frozen in
# contact, and chi the task's phase flag. In free flight F_hat_y = 0 by
# construction; u3's feedforward becomes C_x T^-T w_hat_int (eq. u3_4). The
# whole change is inside wb_l1_observer.cpp's `four_d` branch, parity-locked
# to l1_observer.py by WbL1ParityTest (8 rollouts, 1e-8).
#
# WHY THE RIG EXISTS. The whole-body law consumes the disturbance estimate in
# three places: -d_hat_t in f_d, -d_hat_r in u_2, and F_hat_y in u_3. The GMO
# supplies all three from one proportional law d_hat = K_o (p - p_hat), and
# that costs two things this rig measures:
#
#   1. ONE GAIN FOR TWO JOBS. K_o sets estimate accuracy AND robustness at
#      once, so it is pinned at 0.5/0.1/0.1 -- raising the body channels to
#      1.0 crashed the rig, the observer booking the 99.7 ms rotor lag as a
#      phantom disturbance. The L1 law separates them: a deadbeat inversion
#      whose accuracy is set by the sample period alone, then an explicit
#      filter C(s) = omega_c/(s+omega_c) that alone decides robustness.
#      omega_c is the direct analogue of K_o -- start matched, then raise.
#   2. NO ATTRIBUTION. A momentum residual measures only the SUM of the
#      internal disturbance and the contact wrench, so the GMO hands u_3 a
#      task force carrying the internal disturbance as a PHANTOM CONTACT
#      FORCE: the controller renders compliance against a force nothing
#      applied. The L1 path separates them on the four directions no wrench
#      can reach. In free flight the true contact wrench is exactly zero, so
#      the reported F_hat_y is an honest, direct measurement of the idea.
#
# Working note: "Decompose the lumped disturbances into end-effector and
# orthogonal components" (2026-08-27). Implementation: the fork's
# wb_l1_observer.{hpp,cpp}; Python reference and every measured number quoted
# in the yaml: extensions/.../robotic_arm/utils_controller/l1_observer.py
# (run it for its self-test). Parity-locked to 1e-8 by WbL1ParityTest.
#
# PAIR WITH (started FIRST -- it owns MicroXRCEAgent):
#   fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh
#
# Operating procedure is UNCHANGED from the GMO rig (same node name, same
# namespace, same service, same gates):
#   1. SAFETY takeoff to z = 1.2 m from the drone ground station, settle.
#   2. Confirm the autopilot pane shows the magenta "DISTURBANCE OBSERVER:
#      L1 ADAPTIVE" banner AND "ATTRIBUTION: FOUR-DIMENSIONAL" -- without the
#      first you are flying the GMO, without the second the 6-D rig.
#   3. Enter DIRECT from the Controller tab, or:
#        ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode \
#          std_srvs/srv/SetBool "{data: true}"
#      Entry is GATED on a settled hover; the refusal names every red gate.
#   4. wb_control_debug: [57] = 1 (L1 live), [88] = 0 (nothing on a bound),
#      [58..61] F_hat_y = 0 in free flight BY CONSTRUCTION, [97..100] the RAW
#      F_hat the collision test reads (the honest phantom number on this
#      rig), [101..104] the joint-row trim w_hat_q, [105] chi (1 = free).
#   5. Abort with data: false. The watchdog reverts at 40 deg / 360 dps.
#
# NEVER step a reference at the instant the arm's torque source switches --
# the sequencing rule; see Command.md 7.14.
#
# The PX4 profile is SHARED with 04/05 and the GMO rig on purpose: same plant,
# same saved tune (the per-vehicle-profile rule separates different VEHICLES,
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
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/07_px4_direct_t650_aerial_manipulator_interaction.py"
export INDOOR_SIM_VEHICLE_LABEL="AM-T650-WB-L1-4D-INT"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"
# Fail before physics starts if this plant ever drifts from the mass used by
# the paired whole-body controller YAML.
export PEGASUS_EXPECTED_TOTAL_MASS="3.746170"

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
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh" >&2
  exit 1
fi

# MicroXRCEAgent alone is not proof that the matching controller stack is
# active; another launcher can own an agent while publishing incompatible PX4
# setpoints. Require this variant's executable specifically.
# The two whole-body executables differ ONLY in this name -- neither is a
# substring of the other -- so this is what keeps the L1 rig and the GMO rig
# from being flown against each other by accident.
controller_ready=0
for _ in $(seq 30); do
  if pgrep -f "autopilot_whole_body_l1_direct_actuation_node" >/dev/null 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
if [[ $controller_ready -ne 1 ]]; then
  echo "ERROR: the whole-body L1 aerial-manipulator controller is not running." >&2
  if pgrep -f "autopilot_whole_body_direct_actuation_node" >/dev/null 2>&1; then
    echo "       The GMO whole-body node IS running -- that is the OTHER rig." >&2
    echo "       Use start_t650_aerial_manipulator_whole_body_GMO_6D_direct_actuation_sitl.sh" >&2
    echo "       for it, or stop it and start the L1 4-D stack instead." >&2
  fi
  echo "Start its external stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh $CFG_NAME" >&2
  exit 1
fi

# THE 6-D AND 4-D RIGS SHARE ONE EXECUTABLE, so the process name above cannot
# tell them apart. Ask the running node which attribution it loaded: the
# parameter is read at node startup from the yaml the stack script chose, so
# this is exactly "which yaml is flying". A wrong answer is a hard refusal --
# every number this launcher's yaml describes (plant knobs, injections) would
# otherwise be attributed to the wrong design. ros2 needs the autopilot
# workspace sourced, which the launcher itself does not carry; do it in a
# subshell so nothing leaks into the panes below.
ATTRIB_UAV_NS="${INDOOR_SIM_UAV_NS:-uav_0}"
FOUR_D_ANSWER="$(
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" >/dev/null 2>&1
  # shellcheck source=/dev/null
  source "$FSC_AUTOPILOT_WS/install/setup.bash" >/dev/null 2>&1
  for _ in $(seq 20); do
    ans="$(timeout 8 ros2 param get "/$ATTRIB_UAV_NS/fsc_autopilot_ros2" wb_l1_four_d 2>/dev/null | tr -d '\r')"
    case "$ans" in *True*|*true*) echo true; exit 0 ;; *False*|*false*) echo false; exit 0 ;; esac
    sleep 1
  done
  echo unknown
)"
case "$FOUR_D_ANSWER" in
  true)
    echo -e "\033[1;35mRunning node confirms wb_l1_four_d = true: the FOUR-DIMENSIONAL attribution is flying.\033[0m" ;;
  false)
    echo "ERROR: the running whole-body L1 node reports wb_l1_four_d = false -- that is the" >&2
    echo "       SIX-dimensional rig (start_whole_body_l1_direct_actuation_..._stack.sh)." >&2
    echo "       This launcher's yaml and label describe the 4-D design; refusing to mislabel" >&2
    echo "       a flight. Stop that stack and start:" >&2
    echo "  fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh $CFG_NAME" >&2
    exit 1 ;;
  *)
    echo "ERROR: could not read wb_l1_four_d from /$ATTRIB_UAV_NS/fsc_autopilot_ros2 (is the" >&2
    echo "       autopilot workspace at \$FSC_AUTOPILOT_WS=$FSC_AUTOPILOT_WS built, and the node" >&2
    echo "       running under that namespace?). Refusing to guess which design is flying." >&2
    exit 1 ;;
esac

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

case "$PEGASUS_ARM_SERVO_MODEL" in
  ideal) echo -e "\033[1;33mArm servo model: IDEAL - current-loop residual OFF (commanded effort applied exactly).\033[0m" ;;
  *) echo "Arm servo model: CURRENT LOOP on j2/j3 only (residual ${PEGASUS_ARM_CURRENT_NOISE_A:-servo_model default} A rms per joint @ ${PEGASUS_ARM_CURRENT_NOISE_BW_HZ:-5.0} Hz, seed ${PEGASUS_ARM_CURRENT_NOISE_SEED:-0})"
     echo "  counts/N.m: calibrated 2026-09-11 [162.4, 154.0, 150.5, 153.4] (was [160.0, 173.8, 146.7, 160.0])" ;;
esac

echo "Starting AM-T650 WHOLE-BODY + L1 ADAPTIVE (4-D attribution) direct-actuator SITL with the ROS2 TORQUE-mode arm stack."
echo -e "\033[1;35mDisturbance observer: L1 ADAPTIVE with the FOUR-DIMENSIONAL attribution on the joint rows. Same law, same gains, same plant as the 6-D rig.\033[0m"
echo "Plant: AM_xfwd on T650 motors; controller: whole-body impedance + L1 augmentation"
if [[ "$WB_SIM_PROFILE" == robustness ]]; then
  echo -e "\033[1;33mNOTICE: robustness profile -- the paired controller carries a simulation-only +17.6% kf mismatch and the config-A plant injections.\033[0m"
else
  echo -e "\033[1;36mNOTICE: mirror profile -- the controller flies the HARDWARE config; the plant is the one identified from the 0918/0921/0924 flights.\033[0m"
fi
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
