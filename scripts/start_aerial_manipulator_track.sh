#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Launch the aerial-manipulator IN-PROCESS demo (no ROS 2, no PX4, no SITL).
#
# This is the Phase-1 / controller-debug variant of start_aerial_manipulator_hover.sh.
# It runs a SINGLE Isaac Sim process: the whole-body controller runs IN-PROCESS
# as a physics callback at the 250 Hz physics rate (no ROS 2 transport, no
# separate controller node, no 60 Hz render-loop bottleneck).
#
#   ┌ Isaac Sim (only process) ─────────────────────────────────────────┐
#   │  application/robotic_arm/02_aerial_manipulator_track.py               │
#   │    spawns the AM vehicle (AM_realign.usda),                        │
#   │    runs controller_track in-process, and writes rotor input_ref +     │
#   │    arm efforts directly every physics step.                        │
#   └────────────────────────────────────────────────────────────────────┘
#
# Contrast with start_aerial_manipulator_hover.sh, which runs the distributed ROS 2
# version (Isaac pane + controller-node pane) over topics.
#
# Usage:
#   ./scripts/start_aerial_manipulator_track.sh <config_name>
#   e.g. ./scripts/start_aerial_manipulator_track.sh shiqi_machine
#
# Optional: export AM_SWEEP='{"headless":true,"traj_type":"pickplace"}' before
# running to pass a JSON config to the demo (defaults to {} = interactive hover).
# ============================================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

# ---------------------------
# New-terminal relaunch logic
# ---------------------------
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

# ================================
# Entry-point script (fixed relative tail)
# ================================
PEGASUS_SCRIPT_REL="application/robotic_arm/02_aerial_manipulator_track.py"
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"

[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Isaac script not found: $PEGASUS_SCRIPT" >&2; exit 1; }

# The demo imports its modules (x650_rotorcraft_utils, controller_track) via
# the editable-installed `fsc_aerial_manipulation` package, so nothing extra
# needs to go on the path. NOTE: Isaac's python.sh resets PYTHONPATH, so a
# bare-module + PYTHONPATH approach does NOT survive into the process — that is
# why the demo uses fully-qualified package imports instead.

# Log file so the run can be grepped after Ctrl+C.
ISAAC_LOG="/tmp/aerial_manip_inproc.log"

echo "Launching Isaac Sim aerial-manipulator IN-PROCESS demo...  (log: $ISAAC_LOG)"
echo "  script:   $PEGASUS_SCRIPT"
echo "  AM_SWEEP: ${AM_SWEEP:-unset - demo uses empty default}"

# Single process (no tmux needed) — run in this terminal, tee to the log,
# and drop to a shell afterwards so the window stays open.
PYTHONUNBUFFERED=1 "$ISAAC_PY" "$PEGASUS_SCRIPT" 2>&1 | tee "$ISAAC_LOG"
echo "Isaac Sim exited."
exec bash
