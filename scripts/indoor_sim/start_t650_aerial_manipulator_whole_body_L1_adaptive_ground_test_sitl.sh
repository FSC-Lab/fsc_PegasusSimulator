#!/usr/bin/env bash
set -euo pipefail

# GROUND TEST: the AM-T650 whole-body + L1 rig with INERT PROPS.
#
# THE PARALLEL OF start_t650_aerial_manipulator_whole_body_L1_adaptive_direct_actuation_sitl.sh.
# Same Isaac entrypoint, same AM_xfwd asset, same T650 motor map and mass gate,
# same arm servo model, same plant-uncertainty injection, same PX4 profile,
# same L1 controller stack, same TORQUE-mode fsc_open_manipulator stack, same
# two ground stations, same yaml. EXACTLY ONE thing about the world is
# different, and it is on the PLANT side:
#
#   PEGASUS_PLANT_KF_SCALE = PEGASUS_PLANT_KM_SCALE = 0
#
# i.e. the props turn at whatever speed the allocator commands and produce NO
# force and NO yaw moment. The vehicle is seated on its legs at the measured
# resting height (body z = 0.305 m) and stays there. Every wrench acting on
# the body is then the arm reaction, gravity and the floor contact -- which is
# the whole question this rig exists to answer: WHAT DOES THE WHOLE-BODY LAW
# DO IF YOU ENGAGE DIRECT ON THE GROUND AND THEN MOVE THE ARM?
#
# BOTH COEFFICIENTS, NOT JUST k_f. Multirotor.update() applies k_f*w^2 as a
# force on each rotor body and k_m*w^2*rot_dir as ONE summed yaw moment on
# /body -- two different places. Zeroing k_f alone leaves props that torque
# the airframe in yaw while lifting nothing, which is not a physical airframe
# and would quietly confound the yaw channel. 06 warns if only one is zero.
#
# WHAT TO EXPECT, so a normal result is not read as a fault:
#   * The props render STATIONARY. handle_propeller_visual() animates from the
#     applied force, and that force is now exactly 0.0.
#   * The law still commands hover collective. u1 starts at ~m*g and then
#     GROWS: the L1 observer sees a commanded thrust that produces no
#     acceleration and books the whole of it as a disturbance, so d_hat_z
#     walks out to its bound. Watch wb_control_debug[88] -- a railed bound is
#     the expected outcome here, not a bug.
#   * The floor supplies whatever the props do not, so the CoM error stays
#     ~0 and the drift watchdog (0.75 m) will not fire. The TILT watchdog
#     (20 deg) still can, and is the interesting one: it is what catches the
#     vehicle being levered over by its own arm.
#   * In SAFETY the UDE is gated off below ude_height_threshold = 0.35 m and
#     the vehicle rests at 0.305 m, so SAFETY does not wind up on the ground.
#     Only the DIRECT-side observer does.
#
# THE ONE HANDSHAKE THAT CANNOT BE THE SAME, and why. directEntryAllowed()
# refuses DIRECT unless the vehicle is within wb_gate_pos_m = 0.15 m of
# outer_ref_. In the flying rig that gate is satisfied by taking off to z = 1.2
# and settling. Here there is no takeoff, and outer_ref_ is still its default
# (0, 0, 0) against a vehicle seated at z = 0.305 -- so DIRECT would be refused
# for a reason that has nothing to do with the test. This launcher therefore
# adds ONE pane the flying rig does not have: application/robotic_arm/utils/
# wb_ground_reference_hold.py, which republishes a reference AT THE VEHICLE'S
# OWN MEASURED POSE while the node reports SAFETY, and GOES SILENT IN DIRECT
# (a full-rate reference stream in DIRECT drags the whole-body planner out of
# HOLD every tick -- the 2026-08-23 re-plan oscillation). Nothing else about
# the operating procedure changes. Do NOT command a takeoff setpoint here: it
# would open a 0.9 m position error the gate then refuses, and with no thrust
# the vehicle can never close it.
#
# NOT A FLIGHT RIG. The controller believes it is flying, so nothing it
# reports about thrust, disturbance or altitude means what it means in the
# air. Read the ARM and the ATTITUDE; treat the thrust channel as diagnostic
# only.
#
# PAIR WITH (started FIRST -- it owns MicroXRCEAgent), UNCHANGED:
#   fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh
#
# Operating procedure:
#   1. Start the stack above, then this launcher.
#   2. Wait for the "hold captured at (...)" line in the ground-hold pane.
#   3. Arm + OFFBOARD from the drone ground station (or the /uav_0/rc/*
#      services). Do NOT send a takeoff setpoint.
#   4. Confirm the autopilot pane shows the magenta "DISTURBANCE OBSERVER:
#      L1 ADAPTIVE" banner, then enter DIRECT:
#        ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode \
#          std_srvs/srv/SetBool "{data: true}"
#   5. Move the arm from the arm ground station's EE Whole-Body tab (or a
#      joint target), and watch the base.
#   6. Abort with data: false. The watchdog reverts at 20 deg / 360 dps.
#
# The PX4 profile is SHARED with the flying rig on purpose: same vehicle, same
# saved tune (the per-profile rule separates VEHICLES, not scenarios).

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

# ── GROUND-HOLD pane (this rig only) ────────────────────────────────────────
# Seats outer_ref_ on the vehicle's own pose so the DIRECT position gate reads
# ~0 without a takeoff. Needs rclpy AND fsc_autopilot_ros2_msgs, so the pane
# sources the autopilot workspace and PROBES for an interpreter that can
# import both: on some machines `python3` first on PATH is Isaac's 3.11, which
# has neither. Override with FSC_GROUND_HOLD_PYTHON.
GROUND_HOLD_NODE="$REPO_ROOT/application/robotic_arm/utils/wb_ground_reference_hold.py"
GROUND_HOLD_NS="uav_0"
GROUND_HOLD_RATE="${WB_GROUND_HOLD_RATE:-5}"
AUTOPILOT_WS_SETUP="${FSC_AUTOPILOT_WS:-}/install/setup.bash"

# Variant hooks consumed by the base launcher (same mechanism as the sibling
# L1 launcher; the Isaac entrypoint and label differ — TORQUE-mode plant).
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py"
export INDOOR_SIM_VEHICLE_LABEL="AM-T650-WB-L1-GROUND"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"
# Fail before physics starts if this plant ever drifts from the mass used by
# the paired whole-body controller YAML.
export PEGASUS_EXPECTED_TOTAL_MASS="3.746170"

# -- ARM SERVO MODEL: the current loop's residual, in the plant (2026-09-09) ---
# THE SWITCH LIVES HERE, not in the controller YAML: this is a property of the
# PLANT, and the paired fsc_autopilot_ros2 node never sees it -- that mismatch
# is the effect under test. The arm controller now closes a 1.5 Hz software
# CURRENT LOOP around Dynamixel Mode 16, which REMOVES the back-EMF droop this
# block used to carry (j2 torque delivery 93.3 -> 99.4 %, j3 87.5 -> 91.8 %) and
# leaves a zero-mean residual current error in its place. Design and bench
# numbers: fsc_open_manipulator/doc/"Current Loop Design.md" and
# doc/current_error_all.png; implementation in .../robotic_arm/servo_model.py.
#
#   current  ON  - the arm as it is today: residual current noise, band-limited
#                  at 5 Hz, made torque through Kt
#   ideal    OFF - the commanded effort applied exactly, for the A/B
#
# THE RESIDUAL IS PER JOINT (2026-09-11), and the asymmetry is the plant: the
# arm ships current_loop_bandwidth_hz_joints [0.0, 1.5, 1.5, 0.0], because the
# trim helps the two loaded joints and HURTS j1/j4, whose commanded current is
# mostly noise. So the two joints with NO trim are the two NOISIEST -- in-band
# bench rms 11 / 4.4 / 6.8 / 11 mA, which this file's yaml carries as the total
# rms [15.6, 6.2, 9.6, 15.6] mA (a first-order low pass puts half its variance
# below its own corner, so total = sqrt(2) x in-band).
#
# Override without editing this file:
#   PEGASUS_ARM_SERVO_MODEL=ideal <this script> <config>
# The SOURCE OF TRUTH is the paired controller yaml, so one file describes the
# whole run (user request, 2026-09-04). The whole-body NODE never declares these
# keys -- rclcpp ignores them -- they are here for the plant, which is why they
# are prefixed sim_ and why this launcher is what reads them.
# Precedence: environment > yaml > built-in default.
WB_SIM_YAML="${WB_SIM_YAML:-$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml}"
yaml_scalar() {  # $1 = key; prints the value, or nothing if absent/commented
  [[ -r "$WB_SIM_YAML" ]] || return 0
  sed -nE "s/^[[:space:]]*$1:[[:space:]]*([^#[:space:]]+).*/\\1/p" "$WB_SIM_YAML" | head -1
}
if [[ -z "${PEGASUS_ARM_SERVO_MODEL:-}" ]]; then
  case "$(yaml_scalar sim_arm_current_noise_enable)" in
    true|True|TRUE|1)    PEGASUS_ARM_SERVO_MODEL=current ;;
    false|False|FALSE|0) PEGASUS_ARM_SERVO_MODEL=ideal ;;
    "") PEGASUS_ARM_SERVO_MODEL=current
        echo "NOTE: sim_arm_current_noise_enable not found in $WB_SIM_YAML -- defaulting to 'current'." ;;
    *)  echo "ERROR: sim_arm_current_noise_enable must be true or false in $WB_SIM_YAML" >&2; exit 2 ;;
  esac
  if [[ "$PEGASUS_ARM_SERVO_MODEL" != "ideal" ]]; then
    for KV in "PEGASUS_ARM_CURRENT_NOISE_BW_HZ:sim_arm_current_noise_bw_hz" \
              "PEGASUS_ARM_CURRENT_NOISE_SEED:sim_arm_current_noise_seed"; do
      VAR="${KV%%:*}"; KEY="${KV##*:}"
      if [[ -z "${!VAR:-}" ]]; then
        V="$(yaml_scalar "$KEY")"
        [[ -n "$V" ]] && export "$VAR=$V"
      fi
    done
    # The amplitude is PER JOINT: sim_arm_current_noise_a_j1..j4, joined into
    # the "a1,a2,a3,a4" form 06 parses. ALL FOUR OR NONE -- a partial list
    # would silently model three joints, the same rule the counts block uses.
    # The pre-2026-09-11 scalar sim_arm_current_noise_a still works as a
    # fallback, so an older yaml keeps its meaning.
    if [[ -z "${PEGASUS_ARM_CURRENT_NOISE_A:-}" ]]; then
      NOISE_LIST=""
      for J in j1 j2 j3 j4; do
        V="$(yaml_scalar "sim_arm_current_noise_a_$J")"
        [[ -n "$V" ]] || { NOISE_LIST=""; break; }
        NOISE_LIST="${NOISE_LIST:+$NOISE_LIST,}$V"
      done
      [[ -z "$NOISE_LIST" ]] && NOISE_LIST="$(yaml_scalar sim_arm_current_noise_a)"
      [[ -n "$NOISE_LIST" ]] && export PEGASUS_ARM_CURRENT_NOISE_A="$NOISE_LIST"
    fi
  fi
  echo "Arm servo model taken from $(basename "$WB_SIM_YAML")"
fi
export PEGASUS_ARM_SERVO_MODEL="${PEGASUS_ARM_SERVO_MODEL:-current}"

# ── THE ONE PLANT CHANGE: INERT PROPS ───────────────────────────────────────
# Set BEFORE the yaml-backed loop below, which only fills a variable that is
# still EMPTY -- that is the "environment > yaml > built-in" precedence every
# knob on this rig uses, and it is what makes this override the yaml's
# sim_plant_kf_scale without editing a controller config. Both coefficients:
# see the header for why k_f alone is not an airframe.
export PEGASUS_PLANT_KF_SCALE=0.0
export PEGASUS_PLANT_KM_SCALE=0.0
echo -e "\033[1;31mGROUND TEST: ROTORS INERT (k_f = k_m = 0). The vehicle cannot leave the ground.\033[0m"
echo -e "\033[1;33mThe controller is NOT told: it commands hover collective and the observer books the whole of it as a disturbance. That is the measurement.\033[0m"

# ── ROBUSTNESS INJECTION: plant-side model uncertainty ──────────────────────
# Same precedence rule as the servo model: environment > yaml > built-in.
# Absent keys leave the plant nominal, so a yaml without them behaves as before.
for KV in "PEGASUS_PLANT_MASS_SCALE:sim_plant_mass_scale" \
          "PEGASUS_PLANT_INERTIA_SCALE:sim_plant_inertia_scale" \
          "PEGASUS_PLANT_COM_SHIFT_X:sim_plant_com_shift_x" \
          "PEGASUS_PLANT_COM_SHIFT_Y:sim_plant_com_shift_y" \
          "PEGASUS_PLANT_COM_SHIFT_Z:sim_plant_com_shift_z" \
          "PEGASUS_PLANT_KF_SCALE:sim_plant_kf_scale"; do
  VAR="${KV%%:*}"; KEY="${KV##*:}"
  if [[ -z "${!VAR:-}" ]]; then
    V="$(yaml_scalar "$KEY")"
    [[ -n "$V" ]] && export "$VAR=$V"
  fi
done
if [[ -n "${PEGASUS_PLANT_MASS_SCALE:-}${PEGASUS_PLANT_INERTIA_SCALE:-}${PEGASUS_PLANT_COM_SHIFT_X:-}${PEGASUS_PLANT_COM_SHIFT_Y:-}${PEGASUS_PLANT_COM_SHIFT_Z:-}" ]]; then
  echo -e "\033[1;31mMODEL-UNCERTAINTY INJECTION ACTIVE: mass x${PEGASUS_PLANT_MASS_SCALE:-1.0}, inertia x${PEGASUS_PLANT_INERTIA_SCALE:-1.0}, CoM shift (${PEGASUS_PLANT_COM_SHIFT_X:-0},${PEGASUS_PLANT_COM_SHIFT_Y:-0},${PEGASUS_PLANT_COM_SHIFT_Z:-0}) m\033[0m"
fi
if [[ -n "${PEGASUS_PLANT_KF_SCALE:-}" && "${PEGASUS_PLANT_KF_SCALE}" != "1.0" ]]; then
  # A PLANT-side k_f error is seen by SAFETY and DIRECT alike, unlike an
  # alloc_thrust_coeff detune which only DIRECT goes through.
  echo -e "\033[1;31mTHRUST-LOSS INJECTION ACTIVE (PLANT SIDE, both modes): k_f x${PEGASUS_PLANT_KF_SCALE}\033[0m"
  echo -e "\033[1;33mThe PLANT is perturbed; the controller believes the nominal model.\033[0m"
fi

# ── ARM COUNT <-> TORQUE: the digital command path ──────────────────────────
# The real arm command is an int16 Goal PWM register, not a torque. Same
# precedence rule again: environment > yaml > built-in. Absent keys leave the
# arm a perfectly calibrated continuous torque source, i.e. the pre-2026-09-08
# plant, so a yaml without them behaves exactly as before.
if [[ -z "${PEGASUS_ARM_COUNTS_ENABLE:-}" ]]; then
  case "$(yaml_scalar sim_arm_counts_enable)" in
    true|True|TRUE|1)    export PEGASUS_ARM_COUNTS_ENABLE=1 ;;
    false|False|FALSE|0) export PEGASUS_ARM_COUNTS_ENABLE=0 ;;
    "")                  export PEGASUS_ARM_COUNTS_ENABLE=0 ;;
    *)  echo "ERROR: sim_arm_counts_enable must be true or false in $WB_SIM_YAML" >&2; exit 2 ;;
  esac
fi
for KV in "PEGASUS_ARM_COUNTS_NOMINAL:sim_arm_counts_nominal" \
          "PEGASUS_ARM_COUNTS_TRUE:sim_arm_counts_true"; do
  VAR="${KV%%:*}"; KEY="${KV##*:}"
  [[ -n "${!VAR:-}" ]] && continue
  LIST=""
  for J in j1 j2 j3 j4; do
    V="$(yaml_scalar "${KEY}_$J")"
    # All four or none: a partial list would silently model three joints.
    [[ -n "$V" ]] || { LIST=""; break; }
    LIST="${LIST:+$LIST,}$V"
  done
  [[ -n "$LIST" ]] && export "$VAR=$LIST"
done
# Only claim ACTIVE when something actually changes: quantization on, or the
# two calibrations genuinely differ. Setting TRUE to the nominal value is a
# MATCHED run and must not look like an injection -- a banner that fires either
# way is worth nothing.
ARM_COUNTS_MISMATCH=0
if [[ -n "${PEGASUS_ARM_COUNTS_TRUE:-}" ]]; then
  ARM_COUNTS_MISMATCH=$(awk -v a="${PEGASUS_ARM_COUNTS_NOMINAL:-162.4,154.0,150.5,153.4}" \
                            -v b="$PEGASUS_ARM_COUNTS_TRUE" '
    BEGIN { n=split(a,A,","); split(b,B,",");
            for (i=1;i<=n;i++) if ((A[i]-B[i])^2 > 1e-18) { print 1; exit } print 0 }')
fi
if [[ "${PEGASUS_ARM_COUNTS_ENABLE:-0}" == "1" || "$ARM_COUNTS_MISMATCH" == "1" ]]; then
  echo -e "\033[1;31mARM COUNT<->TORQUE PATH ACTIVE: register quantized=${PEGASUS_ARM_COUNTS_ENABLE:-0}, counts/N.m nominal [${PEGASUS_ARM_COUNTS_NOMINAL:-calibrated}] vs true [${PEGASUS_ARM_COUNTS_TRUE:-calibrated}]\033[0m"
  if [[ "$ARM_COUNTS_MISMATCH" == "1" ]]; then
    echo -e "\033[1;33mDelivered torque scales as nominal/true; the controller is not told. Isaac prints the percentage.\033[0m"
  else
    echo -e "\033[1;33mCalibration MATCHED: register quantization only, no gain error.\033[0m"
  fi
else
  echo "Arm count<->torque path off: continuous torque, perfect calibration."
fi


[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || {
  echo "ERROR: missing AM-T650 Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2
  exit 1
}
[[ -f "$GROUND_HOLD_NODE" ]] || {
  echo "ERROR: missing the ground reference-hold node: $GROUND_HOLD_NODE" >&2
  echo "Without it DIRECT entry is refused on the ground (position gate)." >&2
  exit 1
}
[[ -f "$AUTOPILOT_WS_SETUP" ]] || {
  echo "ERROR: the autopilot workspace is not built: $AUTOPILOT_WS_SETUP" >&2
  echo "The ground-hold pane needs fsc_autopilot_ros2_msgs from it." >&2
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
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh" >&2
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
    echo "       Use start_t650_aerial_manipulator_whole_body_direct_actuation_sitl.sh" >&2
    echo "       for it, or stop it and start the L1 stack instead." >&2
  fi
  echo "Start its external stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh $CFG_NAME" >&2
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

case "$PEGASUS_ARM_SERVO_MODEL" in
  ideal) echo -e "\033[1;33mArm servo model: IDEAL - current-loop residual OFF (commanded effort applied exactly).\033[0m" ;;
  *) echo "Arm servo model: CURRENT LOOP on j2/j3 only (residual ${PEGASUS_ARM_CURRENT_NOISE_A:-servo_model default} A rms per joint @ ${PEGASUS_ARM_CURRENT_NOISE_BW_HZ:-5.0} Hz, seed ${PEGASUS_ARM_CURRENT_NOISE_SEED:-0})"
     echo "  counts/N.m: calibrated 2026-09-11 [162.4, 154.0, 150.5, 153.4] (was [160.0, 173.8, 146.7, 160.0])" ;;
esac

echo "Starting AM-T650 WHOLE-BODY + L1 ADAPTIVE GROUND TEST with the ROS2 TORQUE-mode arm stack."
echo -e "\033[1;35mDisturbance observer: L1 ADAPTIVE (not the GMO). Same law, same gains, same plant.\033[0m"
echo "Plant: AM_xfwd on T650 motors with INERT PROPS; controller: whole-body impedance + L1 augmentation"
echo -e "\033[1;33mNOTICE: paired controller carries a simulation-only +15% kf mismatch. The GMO baseline is validated for hover only; +20% fails it -- that is the bar.\033[0m"
echo "  (controller.py's law, C++ port) from the paired fsc_autopilot_ros2 stack;"
echo "  arm torques via fsc_open_manipulator ExternalTorqueController"
echo "  (torque bring-up) through open_manipulator_x_isaac_bridge's effort system;"
echo "  arm ground station: joint_plot_inverted (WB-TORQUE; controller:=external_torque_controller)"
echo "Arm workspace: $ARM_WS"
echo "MicroXRCEAgent: externally owned and detected"
echo "Ground reference hold: $GROUND_HOLD_NODE (tmux window 'ground')"
echo
echo -e "\033[1;36mGROUND-TEST PROCEDURE\033[0m"
echo "  1. wait for 'hold captured at (...)' in the tmux window 'ground'"
echo "  2. arm + OFFBOARD from the drone ground station -- do NOT send a takeoff setpoint"
echo "  3. check the autopilot pane shows the magenta L1 ADAPTIVE banner"
echo "  4. ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode \\"
echo "       std_srvs/srv/SetBool \"{data: true}\""
echo "  5. move the arm from the arm ground station and watch the base"
echo "  6. abort with {data: false}; the watchdog reverts at 20 deg / 360 dps"

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
ros2 run utils_custom_ground_station joint_plot_inverted --ros-args -r __ns:=/$ARM_NS \\
  -p controller:=external_torque_controller -p simulation:=true \\
  -p mount_height:=$ARM_GS_MOUNT_HEIGHT \\
  -p fallback_min_deg:='[-35.0, -80.0, -40.0, -120.0]' \\
  -p fallback_max_deg:='[35.0, 50.0, 50.0, 120.0]'
echo 'Arm ground station exited.'
exec bash
"

  # GROUND HOLD -- its own window, because it is the one pane this rig has
  # that the flying rig does not, and it must be easy to see. It waits for
  # Isaac's odometry so the captured hold is the seated pose rather than a
  # half-initialised first sample.
  tmux new-window -d -t "$SESSION" -n ground "
source '$ARM_ROS2_SETUP'
source '$AUTOPILOT_WS_SETUP'
PY_BIN='${FSC_GROUND_HOLD_PYTHON:-}'
if [ -z \"\$PY_BIN\" ]; then
  for CAND in /usr/bin/python3 python3; do
    command -v \"\$CAND\" >/dev/null 2>&1 || continue
    if \"\$CAND\" -c 'import rclpy, fsc_autopilot_ros2_msgs' >/dev/null 2>&1; then
      PY_BIN=\"\$CAND\"; break
    fi
  done
fi
if [ -z \"\$PY_BIN\" ]; then
  echo 'ERROR: no interpreter here imports both rclpy and fsc_autopilot_ros2_msgs.'
  echo '       Set FSC_GROUND_HOLD_PYTHON to one that can, then rerun.'
  echo '       Without this pane DIRECT entry is refused: position error 0.305 m > gate 0.15 m.'
  exec bash
fi
echo \"Ground reference hold using \$PY_BIN\"
echo 'Waiting for Isaac odometry on /$GROUND_HOLD_NS/state_estimator/local_position/odom ...'
until timeout 5 ros2 topic echo --once /$GROUND_HOLD_NS/state_estimator/local_position/odom >/dev/null 2>&1; do
  sleep 2
done
\"\$PY_BIN\" '$GROUND_HOLD_NODE' --rate $GROUND_HOLD_RATE \\
  --ros-args -r __ns:=/$GROUND_HOLD_NS
echo 'Ground reference hold exited.'
exec bash
"
) &

# Apply the wall-clock DDS timestamp and HIL auto-disarm settings after the PX4
# shell is ready. These changes intentionally remain per-run and are not saved.
"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

# Reuse the validated indoor PX4/Isaac orchestration and cleanup. Passing
# --in-terminal prevents the base launcher from opening a second terminal.
exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
