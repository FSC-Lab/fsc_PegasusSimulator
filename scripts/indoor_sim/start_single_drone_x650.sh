#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
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
  echo "ERROR: must provide config name."
  cfg_usage "$0"
  exit 2
fi

CFG_NAME="$1"

if [[ $IN_TERM -eq 0 ]]; then
  open_new_terminal "$0" --in-terminal "$CFG_NAME"
  exit 0
fi

load_machine_config "$0" "$CFG_NAME"

ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
GROUNDTRUTH_CHECK="$SCRIPT_DIR/verify_groundtruth_ros2.sh"
# Vehicle-variant hooks. The defaults reproduce the validated indoor X650 exactly,
# so nothing changes for existing callers. start_single_drone_t650.sh sets these
# to swap in the MN4010 / 2.95 kg scenario without duplicating this orchestration:
# the two vehicles differ only in the Isaac app script and the PX4 parameter
# profile, while everything below (PX4 boot, param application, ground-truth
# check, tmux layout, cleanup) is identical for both.
PEGASUS_SCRIPT="${INDOOR_SIM_PEGASUS_SCRIPT:-$FSC_PEGASUS_ROOT/application/px4_base/03_px4_single_drone_x650.py}"
VEHICLE_LABEL="${INDOOR_SIM_VEHICLE_LABEL:-X650}"
X650_ASSET="$FSC_PEGASUS_ROOT/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/x650_new.usd"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_BIN="$PX4_BUILD_DIR/bin/px4"
PX4_ROOTFS_BASE="$PX4_BUILD_DIR/rootfs"
# Keep these parameters independent of the indoor Iris and outdoor launchers --
# and, via the override, of each vehicle variant. PX4 `param save`s into this
# directory, so sharing one profile across vehicles would silently carry one
# airframe's saved tune into the other. The override is a directory NAME, not a
# path, so a variant wrapper does not need PX4_DIR resolved before it runs.
PX4_WORK_DIR="$PX4_BUILD_DIR/${INDOOR_SIM_PX4_PROFILE:-rootfs_fsc_indoor_x650}"
SESSION="px4_isaac"
DELAY=2
EV_DELAY_MS="${PX4_INDOOR_EV_DELAY_MS:-0}"
if [[ ! "$EV_DELAY_MS" =~ ^[0-9]+$ ]] || (( 10#$EV_DELAY_MS > 300 )); then
  echo "ERROR: PX4_INDOOR_EV_DELAY_MS must be an integer from 0 to 300 ms." >&2
  exit 2
fi
EV_DELAY_MS="$((10#$EV_DELAY_MS))"
GROUNDTRUTH_LOG="/tmp/indoor_x650_groundtruth.log"
# Baked into the Isaac pane's command line below rather than left to inheritance. A new
# tmux session inherits the environment of the already-running tmux SERVER, not of this
# shell, so an `export` here (e.g. the one start_x650_direct_actuator_sitl.sh does) is
# silently discarded whenever a server is already up -- and lockstep stays enabled, which
# deadlocks the HIL link the moment the external controller enters DIRECT.
# Default 1 keeps stock lockstep behaviour for the baseline flows that source this script.
LOCKSTEP="${PEGASUS_PX4_LOCKSTEP:-1}"
EXPECTED_TOTAL_MASS="${PEGASUS_EXPECTED_TOTAL_MASS:-}"
if [[ -n "$EXPECTED_TOTAL_MASS" ]] &&
   [[ ! "$EXPECTED_TOTAL_MASS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: PEGASUS_EXPECTED_TOTAL_MASS must be a positive decimal number." >&2
  exit 2
fi
# Optional payload mass in kg, folded into the airframe body mass by the Isaac app
# script (currently only application/px4_base/05_px4_single_drone_t650.py reads it).
# Baked into the pane command line below for the same reason as LOCKSTEP: a new tmux
# session inherits the SERVER environment, not this shell's. Empty = bare airframe.
PAYLOAD_MASS="${PEGASUS_PAYLOAD_MASS:-}"
if [[ -n "$PAYLOAD_MASS" ]] &&
   [[ ! "$PAYLOAD_MASS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: PEGASUS_PAYLOAD_MASS must be a non-negative decimal number." >&2
  exit 2
fi

# Arm SERVO MODEL, read by application/robotic_arm/06_*_arm_torque.py only (the
# whole-body TORQUE plant). This is a PLANT property, not a controller gain --
# the arm controller closes a 1.5 Hz software CURRENT LOOP around Dynamixel
# Mode 16, and what it leaves behind is a zero-mean residual current error --
# PER JOINT since 2026-09-11, because the loop runs on j2/j3 only, so the two
# untrimmed joints are the noisy ones (in-band 11 / 4.4 / 6.8 / 11 mA rms). See
# extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/servo_model.py
#   current  (default) the arm as it is today: residual current noise
#   ideal    an exact torque source, for the A/B
# Baked into the pane command line for the same tmux-server reason as LOCKSTEP.
ARM_SERVO_MODEL="${PEGASUS_ARM_SERVO_MODEL:-}"
case "$ARM_SERVO_MODEL" in
  ""|current|ideal) ;;
  pwm|pwm_0903)
    echo "ERROR: PEGASUS_ARM_SERVO_MODEL='$ARM_SERVO_MODEL' is GONE (2026-09-09)." >&2
    echo "       Those modelled the back-EMF droop, which the arm controller's" >&2
    echo "       1.5 Hz current loop removes. Use 'current' (or 'ideal')." >&2
    exit 2 ;;
  *)
    echo "ERROR: PEGASUS_ARM_SERVO_MODEL must be current or ideal (got '$ARM_SERVO_MODEL')." >&2
    exit 2 ;;
esac

# The residual current error itself: rms in AMPS, first-order corner in Hz, and
# an integer seed (or "none" for OS entropy). Empty = servo_model.py's
# bench-measured defaults. EITHER FORM IS ACCEPTED: one number applies to all
# four joints, or "a1,a2,a3,a4" gives each its own -- which is what the arm
# needs since 2026-09-11, because the current loop runs on j2/j3 only
# (current_loop_bandwidth_hz_joints [0, 1.5, 1.5, 0]) and the untrimmed joints
# are the noisy ones.
ARM_CURRENT_NOISE_A="${PEGASUS_ARM_CURRENT_NOISE_A:-}"
ARM_CURRENT_NOISE_BW_HZ="${PEGASUS_ARM_CURRENT_NOISE_BW_HZ:-}"
ARM_CURRENT_NOISE_SEED="${PEGASUS_ARM_CURRENT_NOISE_SEED:-}"
_NUM_RE='[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?'
for _NV in "$ARM_CURRENT_NOISE_A" "$ARM_CURRENT_NOISE_BW_HZ"; do
  if [[ -n "$_NV" \
        && ! "$_NV" =~ ^${_NUM_RE}$ \
        && ! "$_NV" =~ ^${_NUM_RE},${_NUM_RE},${_NUM_RE},${_NUM_RE}$ ]]; then
    echo "ERROR: PEGASUS_ARM_CURRENT_NOISE_A / _BW_HZ must be a non-negative decimal" >&2
    echo "       or four of them as 'a1,a2,a3,a4' (got '$_NV')." >&2
    exit 2
  fi
done
if [[ -n "$ARM_CURRENT_NOISE_SEED" && ! "$ARM_CURRENT_NOISE_SEED" =~ ^([0-9]+|none)$ ]]; then
  echo "ERROR: PEGASUS_ARM_CURRENT_NOISE_SEED must be an integer or 'none' (got '$ARM_CURRENT_NOISE_SEED')." >&2
  exit 2
fi
# RENDERING. Baked into the pane command line for the same tmux-server reason as
# everything else here -- an `export` from a wrapper does NOT reach an Isaac pane
# on an already-running tmux server, which is how this knob silently did nothing.
#
# THIS IS A CONTROL-FIDELITY SETTING ON A SLOW MACHINE, not just a convenience.
# The whole-body direct-actuation rigs run PEGASUS_PX4_LOCKSTEP=0, so the external
# controller runs on the WALL CLOCK while the plant advances at whatever real-time
# factor the box can manage. At RTF f, every millisecond of real transport delay is
# 1/f milliseconds of SIMULATED delay -- and these rigs are delay-margin limited
# (wb_hover_stability.py scores candidates on a ~32 ms margin against a 10.03 1/s
# rotor pole). Measured on fsc-jupiter 2026-09-11: RTF 0.336 WITH rendering, i.e.
# a 3x inflation of every delay, which alone makes the whole-body L1 rig marginal.
# 06 already does world.step(render=not HEADLESS), so this actually removes the
# render pass rather than just hiding the window.
# Check it during any run with docs/.../tools or a sensor_combined timestamp probe.
PEGASUS_HEADLESS_ARG="${PEGASUS_HEADLESS:-0}"
if [[ ! "$PEGASUS_HEADLESS_ARG" =~ ^[01]$ ]]; then
  echo "ERROR: PEGASUS_HEADLESS must be 0 or 1 (got '$PEGASUS_HEADLESS_ARG')." >&2
  exit 2
fi

# Plant-side model-uncertainty injection (whole-body robustness tests). Baked
# into the pane command line, NOT exported: the tmux server keeps its own env.
PLANT_MASS_SCALE="${PEGASUS_PLANT_MASS_SCALE:-1.0}"
PLANT_INERTIA_SCALE="${PEGASUS_PLANT_INERTIA_SCALE:-1.0}"
PLANT_COM_X="${PEGASUS_PLANT_COM_SHIFT_X:-0.0}"
PLANT_COM_Y="${PEGASUS_PLANT_COM_SHIFT_Y:-0.0}"
PLANT_COM_Z="${PEGASUS_PLANT_COM_SHIFT_Z:-0.0}"
PLANT_KF_SCALE="${PEGASUS_PLANT_KF_SCALE:-1.0}"
if [[ ! "$PLANT_KF_SCALE" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: PEGASUS_PLANT_KF_SCALE must be a non-negative decimal (got '$PLANT_KF_SCALE')." >&2
  exit 2
fi
# The yaw-channel twin of the above. k_f and k_m are applied in DIFFERENT
# places (per-rotor force vs one summed body yaw moment), so zeroing k_f alone
# leaves props that torque but do not lift. 0.0 on both = "props removed",
# which is what the ground-test rig uses.
PLANT_KM_SCALE="${PEGASUS_PLANT_KM_SCALE:-1.0}"
if [[ ! "$PLANT_KM_SCALE" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: PEGASUS_PLANT_KM_SCALE must be a non-negative decimal (got '$PLANT_KM_SCALE')." >&2
  exit 2
fi
# Arm count<->torque path: whether the int16 Goal PWM register is modelled, and
# the counts-per-N.m the command chain BELIEVES vs the winding actually HAS.
# Empty count lists = servo_model.py's calibrated value on both sides.
ARM_COUNTS_ENABLE="${PEGASUS_ARM_COUNTS_ENABLE:-0}"
ARM_COUNTS_NOMINAL="${PEGASUS_ARM_COUNTS_NOMINAL:-}"
ARM_COUNTS_TRUE="${PEGASUS_ARM_COUNTS_TRUE:-}"
if [[ ! "$ARM_COUNTS_ENABLE" =~ ^[01]$ ]]; then
  echo "ERROR: PEGASUS_ARM_COUNTS_ENABLE must be 0 or 1 (got '$ARM_COUNTS_ENABLE')." >&2
  exit 2
fi
for _CL in "$ARM_COUNTS_NOMINAL" "$ARM_COUNTS_TRUE"; do
  if [[ -n "$_CL" ]] &&
     [[ ! "$_CL" =~ ^[0-9]+([.][0-9]+)?(,[0-9]+([.][0-9]+)?){3}$ ]]; then
    echo "ERROR: PEGASUS_ARM_COUNTS_* must be four positive numbers 'c1,c2,c3,c4' (got '$_CL')." >&2
    exit 2
  fi
done
command -v tmux >/dev/null 2>&1 || { echo "ERROR: tmux is not installed or not on PATH." >&2; exit 1; }
command -v timeout >/dev/null 2>&1 || { echo "ERROR: timeout is not installed or not on PATH." >&2; exit 1; }
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: missing $PEGASUS_SCRIPT" >&2; exit 1; }
[[ -f "$X650_ASSET" ]] || { echo "ERROR: missing corrected X650 asset: $X650_ASSET" >&2; exit 1; }
[[ -f "$ROS2_SETUP" ]] || { echo "ERROR: missing $ROS2_SETUP" >&2; exit 1; }
[[ -x "$GROUNDTRUTH_CHECK" ]] || { echo "ERROR: missing executable $GROUNDTRUTH_CHECK" >&2; exit 1; }
[[ -x "$ISAAC_PY" ]] || { echo "ERROR: Isaac Sim wrapper is not executable: $ISAAC_PY" >&2; exit 1; }
[[ -x "$PX4_BIN" ]] || {
  echo "ERROR: PX4 SITL is not built: $PX4_BIN" >&2
  echo "Run: make -C $PX4_DIR px4_sitl_default" >&2
  exit 1
}
[[ -d "$PX4_ROOTFS_BASE" ]] || { echo "ERROR: missing $PX4_ROOTFS_BASE" >&2; exit 1; }

if [[ ! -d "$PX4_WORK_DIR" ]]; then
  echo "Initializing persistent indoor $VEHICLE_LABEL PX4 profile: $PX4_WORK_DIR"
  cp -a "$PX4_ROOTFS_BASE" "$PX4_WORK_DIR"
fi

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

tmux new-session -d -s "$SESSION" -n "indoor_x650" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting indoor $VEHICLE_LABEL PX4 SITL with persistent OptiTrack parameters...'
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=iris PX4_UXRCE_DDS_NS=uav_0 \\
PX4_PARAM_EKF2_HGT_REF=3 PX4_PARAM_EKF2_MAG_TYPE=5 \\
PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_EV_CTRL=15 \\
PX4_PARAM_EKF2_EV_DELAY=$EV_DELAY_MS PX4_PARAM_COM_ARM_WO_GPS=1 \\
PX4_PARAM_SYS_HAS_MAG=0 \\
PX4_PARAM_MC_ROLLRATE_K=0.3 PX4_PARAM_MC_PITCHRATE_K=0.3 \\
PX4_PARAM_MC_YAWRATE_K=0.3 PX4_PARAM_MC_ROLL_P=3.25 \\
PX4_PARAM_MC_PITCH_P=3.25 PX4_PARAM_MC_YAW_P=1.4 \\
  \"$PX4_BIN\" -i 0 -w \"$PX4_WORK_DIR\"
echo 'PX4 SITL exited.'
tmux kill-pane -t \"$SESSION:0.1\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"

# Print and persist the complete profile after PX4 reaches its console.
(
  sleep 4
  for parameter in \
    EKF2_HGT_REF EKF2_MAG_TYPE EKF2_GPS_CTRL EKF2_EV_CTRL EKF2_EV_DELAY COM_ARM_WO_GPS \
    SYS_HAS_MAG \
    MC_ROLLRATE_K MC_PITCHRATE_K MC_YAWRATE_K MC_ROLL_P MC_PITCH_P MC_YAW_P; do
    tmux send-keys -t "$SESSION:0.0" "param show $parameter" Enter
    sleep 0.15
  done
  tmux send-keys -t "$SESSION:0.0" "param save" Enter
) &

tmux split-window -h -t "$SESSION":0 "
echo 'Waiting $DELAY sec for PX4...'
sleep $DELAY
echo 'Launching indoor $VEHICLE_LABEL from: $PEGASUS_SCRIPT'
echo 'Asset: $X650_ASSET'
PEGASUS_PX4_LOCKSTEP=$LOCKSTEP PEGASUS_EXPECTED_TOTAL_MASS=$EXPECTED_TOTAL_MASS \
PEGASUS_HEADLESS=$PEGASUS_HEADLESS_ARG \
PEGASUS_PAYLOAD_MASS=$PAYLOAD_MASS PEGASUS_ARM_SERVO_MODEL=$ARM_SERVO_MODEL \
PEGASUS_ARM_CURRENT_NOISE_A=$ARM_CURRENT_NOISE_A \
PEGASUS_ARM_CURRENT_NOISE_BW_HZ=$ARM_CURRENT_NOISE_BW_HZ \
PEGASUS_ARM_CURRENT_NOISE_SEED=$ARM_CURRENT_NOISE_SEED \
PEGASUS_PLANT_MASS_SCALE=$PLANT_MASS_SCALE \
PEGASUS_PLANT_INERTIA_SCALE=$PLANT_INERTIA_SCALE \
PEGASUS_PLANT_COM_SHIFT_X=$PLANT_COM_X PEGASUS_PLANT_COM_SHIFT_Y=$PLANT_COM_Y \
PEGASUS_PLANT_COM_SHIFT_Z=$PLANT_COM_Z \
PEGASUS_PLANT_KF_SCALE=$PLANT_KF_SCALE \
PEGASUS_PLANT_KM_SCALE=$PLANT_KM_SCALE \
PEGASUS_ARM_COUNTS_ENABLE=$ARM_COUNTS_ENABLE \
PEGASUS_ARM_COUNTS_NOMINAL=$ARM_COUNTS_NOMINAL \
PEGASUS_ARM_COUNTS_TRUE=$ARM_COUNTS_TRUE \
  \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo 'Isaac Sim exited.'
tmux kill-pane -t \"$SESSION:0.0\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"

tmux split-window -v -t "$SESSION":0 "
sleep 12
if ! \"$GROUNDTRUTH_CHECK\" \"$ROS2_SETUP\" \"$GROUNDTRUTH_LOG\"; then
  echo 'ERROR: Isaac ground-truth ROS 2 verification failed.' | tee -a \"$GROUNDTRUTH_LOG\"
fi
exec bash
"

tmux select-layout -t "$SESSION":0 tiled
tmux attach-session -t "$SESSION"
