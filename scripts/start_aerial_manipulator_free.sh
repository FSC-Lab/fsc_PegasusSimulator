#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Launch the FREE-FLYING aerial-manipulator demo (no ROS 2, no PX4, no SITL).
#
# "free" = the rotors are driven DIRECTLY from the in-process control law: the
# whole-body GMO controller writes backend.input_ref every physics step with no
# autopilot in the loop and no arming/offboard gate. Contrast with
# start_px4_direct_aerial_manipulator_free.sh, which runs the SAME law
# but routes omega through PX4 (ActuatorMotors + offboard gate) and a
# bench-calibrated LAGGED motor model. This rig has ideal instantaneous rotors.
#
# Same single-process rig as start_aerial_manipulator_track.sh (the controller
# runs as a physics callback at the 250 Hz physics rate), but driving
# controller.py + config/free.yaml — the posture-anchor-free control law with the
# generalized-momentum disturbance observer.
#
#   ┌ Isaac Sim (only process) ─────────────────────────────────────────┐
#   │  application/robotic_arm/02_aerial_manipulator_free.py             │
#   │    spawns the AM vehicle (AM_realign.usda),                        │
#   │    runs controller.py + config/free.yaml in-process, writing       │
#   │    rotor input_ref + arm efforts directly every physics step.      │
#   └────────────────────────────────────────────────────────────────────┘
#
# Contrast with start_aerial_manipulator_track.sh (same rig, controller_track.py
# with the joint-space posture anchor and no observer).
#
# Usage:
#   ./scripts/start_aerial_manipulator_free.sh <config_name> [trajectory_mode]
#   e.g. ./scripts/start_aerial_manipulator_free.sh shiqi_machine
#        ./scripts/start_aerial_manipulator_free.sh shiqi_machine circle_drone
#
# The optional 2nd argument picks the trajectory for this run without editing
# the demo (it just exports AM_MODE, which the demo's MODE block reads). Names
# are <shape>_<who> — shape: hover | circle | figure8 | poly, who: drone (arm
# locked at one pose) | whole (whole body moves):
#   hover_drone  circle_drone  figure8_drone  poly_drone
#   hover_whole  circle_whole  figure8_whole  poly_whole
# Omit it to use the demo's in-script MODE. A name that is not in the catalogue
# is rejected at startup with the full list, before Isaac loads the stage.
#
# Optional: export AM_SWEEP='{"headless":true,"traj_type":"hover_drone"}' before
# running to pass a JSON config to the demo (defaults to {} = the in-script
# MODE). AM_SWEEP's traj_type WINS over the argument above — it is the batch /
# gain-sweep path and also carries headless, t_end, gains and log_path.
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

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "ERROR: must provide config name (trajectory mode optional)."
  cfg_usage "$0"
  echo
  echo "Optional 2nd argument — trajectory for this run (default: the demo's MODE):"
  echo "  hover_drone  circle_drone  figure8_drone  poly_drone   (arm locked)"
  echo "  hover_whole  circle_whole  figure8_whole  poly_whole   (whole body)"
  echo "  e.g. ${0} shiqi_machine circle_drone"
  exit 2
fi

CFG_NAME="$1"
TRAJ_MODE="${2:-}"

if [[ $IN_TERM -eq 0 ]]; then
  # forward the mode through the terminal relaunch, if one was given
  open_new_terminal "$0" --in-terminal "$CFG_NAME" ${TRAJ_MODE:+"$TRAJ_MODE"}
  exit 0
fi

# The demo reads AM_MODE (see its MODE block); AM_SWEEP's traj_type still wins
# over it, so a sweep run is unaffected by this argument.
if [[ -n "$TRAJ_MODE" ]]; then
  export AM_MODE="$TRAJ_MODE"
fi

load_machine_config "$0" "$CFG_NAME"

# ================================
# Entry-point script (fixed relative tail)
# ================================
PEGASUS_SCRIPT_REL="application/robotic_arm/02_aerial_manipulator_free.py"
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"

[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Isaac script not found: $PEGASUS_SCRIPT" >&2; exit 1; }

# The demo imports its modules (x650_rotorcraft_utils, controller) via
# the editable-installed `fsc_aerial_manipulation` package, so nothing extra
# needs to go on the path. NOTE: Isaac's python.sh resets PYTHONPATH, so a
# bare-module + PYTHONPATH approach does NOT survive into the process — that is
# why the demo uses fully-qualified package imports instead.

# Log file so the run can be grepped after Ctrl+C.
ISAAC_LOG="/tmp/aerial_manip_gmo.log"

echo "Launching Isaac Sim aerial-manipulator GMO IN-PROCESS demo...  (log: $ISAAC_LOG)"
echo "  script:   $PEGASUS_SCRIPT"
echo "  AM_MODE:  ${AM_MODE:-unset - demo uses its in-script MODE}"
echo "  AM_SWEEP: ${AM_SWEEP:-unset - demo uses empty default}"

# Single process (no tmux needed) — run in this terminal, tee to the log,
# and drop to a shell afterwards so the window stays open.
PYTHONUNBUFFERED=1 "$ISAAC_PY" "$PEGASUS_SCRIPT" 2>&1 | tee "$ISAAC_LOG"
echo "Isaac Sim exited."
exec bash
