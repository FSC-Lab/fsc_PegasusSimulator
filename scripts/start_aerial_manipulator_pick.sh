#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Launch the PICK-AND-PLACE aerial-manipulator demo (no ROS 2, no PX4, no SITL).
#
# This is the PHYSICAL-INTERACTION rig: the vehicle actually grasps, carries and
# releases a real physics object, so the controller meets contact forces and an
# UNMODELLED 0.2 kg payload — the disturbance the GMO is there to reject.
# Everything else is the same single-process rig as
# start_aerial_manipulator_free.sh (the whole-body law runs as a physics
# callback at the 250 Hz physics rate, rotors driven straight from
# backend.input_ref, no autopilot / arming / offboard gate).
#
#   ┌ Isaac Sim (only process) ─────────────────────────────────────────┐
#   │  application/robotic_arm/02_aerial_manipulator_pick.py             │
#   │    spawns the AM vehicle (AM_realign.usda) + the pick/place props  │
#   │    (a 0.30 m table at each end, and a tall graspable block         │
#   │     standing on the pick table),                                   │
#   │    runs controller.py + config/pick.yaml in-process, writing       │
#   │    rotor input_ref + arm/gripper efforts every physics step.       │
#   └────────────────────────────────────────────────────────────────────┘
#
# Default mission (MODE = "pickplace_home" in the demo) — a full GROUND-TO-GROUND
# flight along WORLD +y, the direction the arm faces:
#   takeoff  fly to the task start point with the arm FOLDED at its home pose
#            ([0,40,40,0] deg) — ONE position setpoint, no climb ramp
#   fly-out  translate to the pick table while the arm UNFOLDS to the grasp pose
#   pick     descend alongside the block, close the jaws (a constant closing
#            torque, not a position servo), climb out
#   carry    translate to the place table, descend, release, climb out
#   fly-land translate one more leg PAST the place table while the arm FOLDS
#            back to home — it does NOT return to the takeoff point
#   land     vertical descent onto the legs, arm folded, rotors ramped off
# The arm has just TWO setpoints and swaps between them during the transit
# flights, so it only moves while the vehicle translates — never while it is in
# contact with the block. The folded home pose is REQUIRED to land: at q = 0 the
# finger tips reach 51 mm BELOW ground at the resting body height. The jaws grip
# the block's top ~20 mm only — the fingers reach 64 mm BELOW where they pinch,
# so a mid-height grasp would drive the finger tips through the table.
# "pickplace_static" is the same mission without the ground phases (arm fixed at
# q = 0, ends hovering). Switch modes, or to any other trajectory in
# utils_planner.TRAJ_CONFIG, via MODE or AM_SWEEP's traj_type (below).
#
# WHAT YOU WILL SEE (same three-phase structure as the free-flight demo):
#   * a RENDERED run starts PAUSED with the vehicle seated on its legs and the
#     arm already folded — inspect the props, then press PLAY in the GUI. Do NOT
#     press STOP: stop->play resets PhysX and undoes the arm's spawn pose.
#   * the terminal carries ONE two-panel frame per second — the flight-phase log
#     on the left, live state (pose / arm / grip / errors / control / GMO
#     estimates / block position) on the right, plus a "gate:" line whenever a
#     phase switch is pending, showing exactly which tolerance is still open.
#   * takeoff -> task and task -> landing are gated on the MEASURED hover error
#     (0.15 m, 0.20 m/s, held 2 s), never on a clock, so phase times shift a
#     little run to run. The npz records the real boundaries.
#   * once it has settled at the landing setpoint the rotors ramp off over 2 s
#     and the app SAVES THE LOG AND CLOSES ITSELF — no need to kill it.
#
# Contrast with:
#   start_aerial_manipulator_free.sh  — same law, free flight, no object contact
#                                       (config/free.yaml, gmo_log.npz)
#   start_aerial_manipulator_track.sh — posture-anchor baseline, no observer
#
# Usage:
#   ./scripts/start_aerial_manipulator_pick.sh <config_name>
#   e.g. ./scripts/start_aerial_manipulator_pick.sh shiqi_machine
#
# Optional: export AM_SWEEP='{"headless":true,"t_end":55.0}' before running to
# pass a JSON config to the demo (defaults to {} = interactive pick-and-place).
# t_end is only a SAFETY STOP now — the demo ends itself after the landing. The
# default mission is 34 s of trajectory after a hover-gated takeoff (~3 s), plus
# the landing descent and a 2 s rotor ramp-off, so a headless run reaches the
# ground at roughly 44 s; give t_end ~55 s of margin if you set one at all.
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
PEGASUS_SCRIPT_REL="application/robotic_arm/02_aerial_manipulator_pick.py"
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"

[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Isaac script not found: $PEGASUS_SCRIPT" >&2; exit 1; }

# The demo imports its modules (x650_rotorcraft_utils, controller) via
# the editable-installed `fsc_aerial_manipulation` package, so nothing extra
# needs to go on the path. NOTE: Isaac's python.sh resets PYTHONPATH, so a
# bare-module + PYTHONPATH approach does NOT survive into the process — that is
# why the demo uses fully-qualified package imports instead.

# Log file so the run can be grepped after Ctrl+C.
ISAAC_LOG="/tmp/aerial_manip_pick.log"

echo "Launching Isaac Sim aerial-manipulator PICK-AND-PLACE IN-PROCESS demo...  (log: $ISAAC_LOG)"
echo "  script:   $PEGASUS_SCRIPT"
echo "  AM_SWEEP: ${AM_SWEEP:-unset - demo uses empty default}"
echo "  npz:      <fsc_aerial_manipulation>/robotic_arm/results/log/pick_log.npz (written on clean exit)"

# Single process (no tmux needed) — run in this terminal, tee to the log,
# and drop to a shell afterwards so the window stays open.
PYTHONUNBUFFERED=1 "$ISAAC_PY" "$PEGASUS_SCRIPT" 2>&1 | tee "$ISAAC_LOG"
echo "Isaac Sim exited."
exec bash
