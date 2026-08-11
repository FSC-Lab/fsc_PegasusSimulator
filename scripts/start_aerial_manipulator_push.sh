#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Launch the BOX-PUSH aerial-manipulator demo (no ROS 2, no PX4, no SITL).
#
# This is the PHYSICAL-INTERACTION rig, sibling of
# start_aerial_manipulator_pick.sh: instead of grasping, the vehicle PUSHES a
# real physics box a fixed distance across a table with its closed jaws — the
# sliding-friction contact force (~0.6 N, nowhere in the reference model) is
# the disturbance the impedance + GMO are there to carry. Everything else is
# the same single-process rig as start_aerial_manipulator_free.sh (the
# whole-body law runs as a physics callback at the 250 Hz physics rate, rotors
# driven straight from backend.input_ref, no autopilot / arming / offboard
# gate).
#
#   ┌ Isaac Sim (only process) ─────────────────────────────────────────┐
#   │  application/robotic_arm/02_aerial_manipulator_push.py             │
#   │    spawns the AM vehicle (AM_realign.usda) + the push props        │
#   │    (one 0.30 x ~0.6 m table spanning the push line, and a 0.2 kg   │
#   │     box standing on it just ahead of the fingers),                 │
#   │    runs controller.py + config/push.yaml in-process, writing       │
#   │    rotor input_ref + arm/gripper efforts every physics step.       │
#   └────────────────────────────────────────────────────────────────────┘
#
# Default mission (MODE = "push_home" in the demo) — a full GROUND-TO-GROUND
# flight along WORLD +y, the direction the arm faces:
#   takeoff  fly to the task start point with the arm FOLDED at its home pose
#            ([0,40,40,0] deg) — ONE position setpoint, no climb ramp
#   fly-out  translate to above the push start while the arm UNFOLDS to the
#            push pose (q = 0, tool vertical) and the jaws CLOSE — the closed
#            fingers are the pusher, nothing is ever grasped
#   approach descend so the hanging fingers sit BESIDE the box
#   push     fly a STRAIGHT fixed-yaw +y line at constant height: the EE gets
#            a straight-line command, meets the box after a 30 mm gap, and
#            slides it ~0.32 m down the table
#   retreat  back off along −y (contact ends — the "stop pushing"), climb out
#   fly-land translate PAST the table while the arm FOLDS back to home
#   land     vertical descent onto the legs, arm folded, rotors ramped off
# The arm has just TWO setpoints and swaps between them during the transit
# flights — it is FIXED at the push pose whenever contact is possible, so the
# push geometry never changes mid-contact. The folded home pose is REQUIRED to
# land: at q = 0 the finger tips reach 51 mm BELOW ground at the resting body
# height. Switch modes, or to any other trajectory in
# utils_planner.TRAJ_CONFIG, via MODE or AM_SWEEP's traj_type (below).
#
# WHAT YOU WILL SEE (same three-phase structure as the free-flight demo):
#   * a RENDERED run starts PAUSED with the vehicle seated on its legs and the
#     arm already folded — inspect the table/box clearances, then press PLAY in
#     the GUI. Do NOT press STOP: stop->play resets PhysX and undoes the arm's
#     spawn pose.
#   * the terminal carries ONE two-panel frame per second — the flight-phase log
#     on the left, live state (pose / arm / grip / errors / control / GMO
#     estimates / box position, slide distance and tilt) on the right, plus a
#     "gate:" line whenever a phase switch is pending, showing exactly which
#     tolerance is still open. During contact the DIST/force row IS the measured
#     push force the observer is rejecting, and BOX/tilt is the tip-over watch.
#   * takeoff -> task and task -> landing are gated on the MEASURED hover error
#     (0.15 m, 0.20 m/s, held 2 s), never on a clock, so phase times shift a
#     little run to run. The npz records the real boundaries.
#   * once it has settled at the landing setpoint the rotors ramp off over 2 s
#     and the app SAVES THE LOG AND CLOSES ITSELF — no need to kill it.
#
# Contrast with:
#   start_aerial_manipulator_pick.sh  — same rig, grasp/carry/release mission
#                                       (config/pick.yaml, pick_log.npz)
#   start_aerial_manipulator_free.sh  — same law, free flight, no object contact
#                                       (config/free.yaml, gmo_log.npz)
#   start_aerial_manipulator_track.sh — posture-anchor baseline, no observer
#
# Usage:
#   ./scripts/start_aerial_manipulator_push.sh <config_name>
#   e.g. ./scripts/start_aerial_manipulator_push.sh shiqi_machine
#
# Optional: export AM_SWEEP='{"headless":true,"t_end":50.0}' before running to
# pass a JSON config to the demo (defaults to {} = interactive push mission).
# t_end is only a SAFETY STOP now — the demo ends itself after the landing. The
# default mission is 29 s of trajectory after a hover-gated takeoff (~3 s), plus
# the landing descent and a 2 s rotor ramp-off, so a headless run reaches the
# ground at roughly 39 s; give t_end ~50 s of margin if you set one at all.
# A headless run skips the PAUSE and flies immediately.
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
PEGASUS_SCRIPT_REL="application/robotic_arm/02_aerial_manipulator_push.py"
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"

[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Isaac script not found: $PEGASUS_SCRIPT" >&2; exit 1; }

# The demo imports its modules (x650_rotorcraft_utils, controller) via
# the editable-installed `fsc_aerial_manipulation` package, so nothing extra
# needs to go on the path. NOTE: Isaac's python.sh resets PYTHONPATH, so a
# bare-module + PYTHONPATH approach does NOT survive into the process — that is
# why the demo uses fully-qualified package imports instead.

# Log file so the run can be grepped after Ctrl+C.
ISAAC_LOG="/tmp/aerial_manip_push.log"

echo "Launching Isaac Sim aerial-manipulator BOX-PUSH IN-PROCESS demo...  (log: $ISAAC_LOG)"
echo "  script:   $PEGASUS_SCRIPT"
echo "  AM_SWEEP: ${AM_SWEEP:-unset - demo uses empty default}"
echo "  npz:      <fsc_aerial_manipulation>/robotic_arm/results/log/push_log.npz (written on clean exit)"

# Single process (no tmux needed) — run in this terminal, tee to the log,
# and drop to a shell afterwards so the window stays open.
PYTHONUNBUFFERED=1 "$ISAAC_PY" "$PEGASUS_SCRIPT" 2>&1 | tee "$ISAAC_LOG"
echo "Isaac Sim exited."
exec bash
