#!/usr/bin/env bash
set -euo pipefail

# Controller-neutral AERIAL-MANIPULATOR BASELINE simulation on the T650
# parameter set:
#
#   external ROS 2 baseline controller -> PX4 attitude/rate/mixer
#       -> HIL_ACTUATOR_CONTROLS -> AM_xfwd plant with the T650 motor model
#          (MN4010, lambda=10.0265) + T650 body mass; the arm is PD-held at its
#          home pose [0, 40, 40, 0] deg by the Isaac process itself
#
# The external controller owns MicroXRCEAgent and all OFFBOARD topics. This
# launcher only configures and starts PX4 SITL plus the simulated plant.
#
# PAIR WITH fsc_autopilot_ros2/scripts/isaacsim/start_baseline_t650_aerial_manipulator_stack_fused.sh
# (started FIRST -- it owns the agent).
#
# THIS IS THE BASELINE (SAFETY) COUNTERPART of
# start_t650_aerial_manipulator_direct_actuator_sitl.sh. The ONLY functional difference is
# LOCKSTEP: this script leaves it ON (the plant default), because lockstep exists
# for exactly this PX4-owns-the-inner-loops path. The direct launcher disables it,
# which is required there and wrong here -- the same split the bare T650 makes
# between start_single_drone_t650.sh and start_t650_direct_actuator_sitl.sh.
#
# Same plant in every other respect:
#   * Isaac entrypoint application/robotic_arm/04_px4_direct_t650_aerial_manipulator_hold.py
#     (its name says "direct" for historical reasons; it is control-agnostic --
#     it only spawns the plant and holds the arm, and reads PEGASUS_PX4_LOCKSTEP).
#   * TOTAL flying mass 3.746 kg (T650 body 2.95 + AM rotors 0.1595 + arm 0.6366)
#     -- expected hover command ~0.569, not the bare T650's ~0.503. T/W 2.71.
#   * Shares the PX4 parameter profile rootfs_fsc_indoor_am_t650 with the direct
#     launcher: same vehicle, same airframe params, and PX4 `param save`s into it.
#   * The paired stack MUST run the AM BASELINE yaml
#     (params_single_vehicle_baseline_t650_aerial_manipulator.yaml): its vehicle_mass carries the
#     arm's weight as feedforward. The bare-T650 baseline yaml declares 2.9 kg
#     against this 3.746 kg plant -- a 29% error the UDE would silently absorb.

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
PARAM_DELAY="${T650_AERIAL_MANIPULATOR_BASELINE_PARAM_DELAY:-8}"

# Variant hooks consumed by the base launcher (same mechanism as
# start_single_drone_t650.sh). The PX4 profile is a directory NAME the base
# launcher prefixes with the SITL build dir; the AM gets its own because PX4
# `param save`s into it — sharing the bare-T650 profile would carry one
# plant's saved tune into the other.
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/04_px4_direct_t650_aerial_manipulator_hold.py"
export INDOOR_SIM_VEHICLE_LABEL="AM-T650"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"

[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || {
  echo "ERROR: missing AM-T650 Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2
  exit 1
}
if [[ ! "$PARAM_DELAY" =~ ^[0-9]+$ ]]; then
  echo "ERROR: T650_AERIAL_MANIPULATOR_BASELINE_PARAM_DELAY must be a non-negative integer." >&2
  exit 2
fi

if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo "ERROR: MicroXRCEAgent is not running." >&2
  echo "Start the agent from the external controller stack before this launcher." >&2
  exit 1
fi

# Lockstep ON, asserted rather than assumed. The base launcher reads
# ${PEGASUS_PX4_LOCKSTEP:-1} in ITS shell and bakes the result onto the Isaac pane
# command line, so an unset variable already yields 1. But the DIRECT launcher
# pushes a global 0 onto the tmux SERVER (tmux setenv -g), which survives that
# session -- so running this script from inside a tmux pane afterwards would
# inherit 0 and silently start the baseline plant with lockstep disabled. Set it
# explicitly and clear the server global so the value cannot depend on where this
# was invoked from.
export PEGASUS_PX4_LOCKSTEP=1
tmux setenv -gu PEGASUS_PX4_LOCKSTEP 2>/dev/null || true

echo "Starting controller-neutral AM-T650 BASELINE SITL."
echo "Plant: AM_xfwd (X650 frame + 4-DOF arm) on T650 motors (MN4010, lambda=10.0265)"
echo "Total mass 3.746 kg (expected hover command ~0.569); arm PD-held at [0,40,40,0] deg"
echo "Pegasus PX4 lockstep: ENABLED (baseline path -- PX4 owns attitude+rate+mixer)"
echo "MicroXRCEAgent: externally owned and detected"
echo "No controller or actuator publisher will be started by this launcher."

# Apply the wall-clock DDS timestamp and HIL auto-disarm settings after the PX4
# shell is ready. These changes intentionally remain per-run and are not saved.
"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

# Reuse the validated indoor PX4/Isaac orchestration and cleanup. Passing
# --in-terminal prevents the base launcher from opening a second terminal.
exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
