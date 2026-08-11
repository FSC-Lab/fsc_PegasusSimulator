#!/usr/bin/env bash
set -euo pipefail

# Controller-neutral AERIAL-MANIPULATOR direct-actuator simulation on the T650
# parameter set:
#
#   external ROS 2 controller -> PX4 ActuatorMotors gate
#       -> HIL_ACTUATOR_CONTROLS -> AM_realign plant with the T650 motor model
#          (MN4010, lambda=10.0265) + T650 body mass; the arm is PD-held at its
#          home pose [0, 40, 40, 0] deg by the Isaac process itself
#
# The external controller owns MicroXRCEAgent and all OFFBOARD/actuator topics.
# This launcher only configures and starts PX4 SITL plus the simulated plant.
#
# PAIR WITH fsc_autopilot_ros2/scripts/isaacsim/start_direct_actuation_am_t650_stack.sh
# (started FIRST — it owns the agent). Identical orchestration to
# start_t650_direct_actuator_sitl.sh; what differs is the plant:
#
#   * Isaac entrypoint: application/robotic_arm/04_px4_direct_am_t650_hold.py
#     (AM_realign.usda, X650 frame + 4-DOF arm) instead of the bare
#     05_px4_single_drone_t650.py.
#   * TOTAL flying mass 3.746 kg (T650 body 2.95 + AM rotors 0.1595 + arm
#     0.6366) — expected hover command ~0.569, not the bare T650's ~0.503.
#     Static thrust-to-weight 2.71.
#   * The paired stack MUST run the AM yaml
#     (params_single_drone_direct_actuation_am_t650.yaml): its vehicle_mass /
#     thrust map carry this heavier hover point. Flying the bare-T650 yaml
#     against this plant leaves the position loop's feedforward ~7 N short.

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
PARAM_DELAY="${AM_T650_DIRECT_ACTUATOR_PARAM_DELAY:-8}"

# Variant hooks consumed by the base launcher (same mechanism as
# start_single_drone_t650.sh). The PX4 profile is a directory NAME the base
# launcher prefixes with the SITL build dir; the AM gets its own because PX4
# `param save`s into it — sharing the bare-T650 profile would carry one
# plant's saved tune into the other.
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/04_px4_direct_am_t650_hold.py"
export INDOOR_SIM_VEHICLE_LABEL="AM-T650"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"

[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || {
  echo "ERROR: missing AM-T650 Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2
  exit 1
}
if [[ ! "$PARAM_DELAY" =~ ^[0-9]+$ ]]; then
  echo "ERROR: AM_T650_DIRECT_ACTUATOR_PARAM_DELAY must be a non-negative integer." >&2
  exit 2
fi

if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo "ERROR: MicroXRCEAgent is not running." >&2
  echo "Start the agent from the external controller stack before this launcher." >&2
  exit 1
fi

# A wall-clock DDS controller can pause PX4 actuator output during OFFBOARD
# transitions. Disabling Pegasus lockstep prevents both sides waiting forever.
export PEGASUS_PX4_LOCKSTEP=0

# ...but the export alone is not enough (diagnosed live 2026-07-30, see
# start_t650_direct_actuator_sitl.sh for the full account): the base launcher
# starts Isaac inside a NEW tmux session, and a new session inherits the
# environment of the already-running tmux SERVER — not of this shell. Under the
# documented start order the controller stack runs first, so a server always
# exists by now and the export above would be discarded; Isaac would come up
# with lockstep enabled and entering DIRECT would deadlock the PX4/Isaac HIL
# link ("ERROR [simulator_mavlink] poll timeout 0, 111").
if tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null; then
  echo "Pegasus PX4 lockstep: pushed to the tmux server environment"
else
  echo "Pegasus PX4 lockstep: no tmux server yet; the new session will inherit the export"
fi

echo "Starting controller-neutral AM-T650 direct-actuator SITL."
echo "Plant: AM_realign (X650 frame + 4-DOF arm) on T650 motors (MN4010, lambda=10.0265)"
echo "Total mass 3.746 kg (expected hover command ~0.569); arm PD-held at [0,40,40,0] deg"
echo "Pegasus PX4 lockstep: disabled"
echo "MicroXRCEAgent: externally owned and detected"
echo "No controller or actuator publisher will be started by this launcher."

# Apply the wall-clock DDS timestamp and HIL auto-disarm settings after the PX4
# shell is ready. These changes intentionally remain per-run and are not saved.
"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

# Reuse the validated indoor PX4/Isaac orchestration and cleanup. Passing
# --in-terminal prevents the base launcher from opening a second terminal.
exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
