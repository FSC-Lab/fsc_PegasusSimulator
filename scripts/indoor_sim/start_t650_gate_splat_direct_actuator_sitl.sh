#!/usr/bin/env bash
set -euo pipefail

# Controller-neutral T650 direct-actuator SITL inside the GATE SPLAT scene:
#
#   external ROS 2 controller -> PX4 ActuatorMotors gate
#       -> HIL_ACTUATOR_CONTROLS -> calibrated T650 Isaac plant
#       -> flying through the reconstructed A2RL x DCL gate (NuRec splat)
#
# Identical in every respect to start_t650_direct_actuator_sitl.sh except that
# the base launcher is the gate-splat variant, so Isaac loads gate_metric.usda
# (metric, gravity-aligned, floor z=0, 1.50 m gate opening centred at
# (0, 0, 1.75), fly-through axis = world X) and the drone carries a forward
# MonocularCamera with its own "Drone Camera" viewport.
#
# The external controller owns MicroXRCEAgent and all OFFBOARD/actuator topics,
# exactly as in the stock launcher. See that script for the T650 plant numbers
# (hover ~0.503, rotor tau 99.7 ms) and for the lockstep rationale copied below.

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

BASE_LAUNCHER="$SCRIPT_DIR/indoor_sim/start_single_drone_t650_gate_splat.sh"
PARAM_SCRIPT="$SCRIPT_DIR/apply_aerial_manipulator_px4_offboard_params.sh"
SESSION="px4_isaac"
PARAM_DELAY="${T650_DIRECT_ACTUATOR_PARAM_DELAY:-8}"

[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
if [[ ! "$PARAM_DELAY" =~ ^[0-9]+$ ]]; then
  echo "ERROR: T650_DIRECT_ACTUATOR_PARAM_DELAY must be a non-negative integer." >&2
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

# The export alone is not enough when a tmux server already exists (the new
# session inherits the SERVER environment, not this shell's). Push the value
# onto the server too; harmless if no server is running yet. Full rationale in
# start_t650_direct_actuator_sitl.sh (diagnosed live 2026-07-30).
if tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null; then
  echo "Pegasus PX4 lockstep: pushed to the tmux server environment"
else
  echo "Pegasus PX4 lockstep: no tmux server yet; the new session will inherit the export"
fi

echo "Starting controller-neutral T650 direct-actuator SITL in the GATE SPLAT scene."
echo "Plant: MN4010 + 15x5\" motors, 2.95 kg (expected hover command ~0.503)"
echo "Scene: ${GATE_SPLAT_USD:-$HOME/Downloads/IMG_4701/gate_metric.usda}"
echo "Pegasus PX4 lockstep: disabled"
echo "MicroXRCEAgent: externally owned and detected"
echo "No controller or actuator publisher will be started by this launcher."

# Apply the wall-clock DDS timestamp and HIL auto-disarm settings after the PX4
# shell is ready. These changes intentionally remain per-run and are not saved.
"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

# Reuse the validated indoor T650 PX4/Isaac orchestration and cleanup. Passing
# --in-terminal prevents the base launcher from opening a second terminal.
exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
