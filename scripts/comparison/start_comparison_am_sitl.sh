#!/usr/bin/env bash
set -euo pipefail

# PEGASUS SIDE of the whole-body vs geometric+L1 COMPARISON campaign.
#
#   scripts/comparison/start_comparison_am_sitl.sh <machine-config> <wb|wb_l1|l1>
#
# NOTHING IN THE FLIGHT-VALIDATION PATH IS TOUCHED BY THIS CAMPAIGN.  The two
# production launchers
#   indoor_sim/start_t650_aerial_manipulator_whole_body_direct_actuation_sitl.sh
#   indoor_sim/start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh
# keep working exactly as they are; this is a third, parallel launcher that
# reuses their Isaac entrypoints (06 and 05) unmodified.  Reusing rather than
# copying them is deliberate: the plant is the one thing that MUST be identical
# between the two runs, and two copies of a 700-line plant script would be the
# easiest way to lose that.
#
# WHAT IS DIFFERENT FROM THE PRODUCTION LAUNCHERS
#   * no fsc_open_manipulator ros2_control stack and no ground stations.  The
#     comparison driver owns the arm command on BOTH runs (it publishes the
#     joint reference straight onto the Isaac arm bus for the L1 run, and the
#     whole-body node's arm reference for the WB run), so the arm command path
#     has the same length on both sides and no GUI can perturb a run.
#   * the paired controller stack is the comparison one, which loads the
#     _comparison.yaml -- matched thrust coefficient, no r_os injection.
#
# SAME AS THE PRODUCTION LAUNCHERS, on purpose
#   * the AM_xfwd plant, the T650 motor model, the measured rotor lag, the
#     3.746170 kg total, the PX4 profile and the PX4 offboard parameters.
#
# Order of operations (same as every other AM launcher):
#   1. fsc_autopilot_ros2/scripts/isaacsim/start_comparison_am_stack.sh <wb|l1>
#      (owns MicroXRCEAgent and the control node)
#   2. this script
#   3. application/robotic_arm/comparison_driver.py --controller <wb|l1> --task ...
#
# Full command sequence: docs/docs_aerial_manipulator/Comparison Command.md

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

if [[ $# -ne 2 ]]; then
  echo "ERROR: usage: $0 <machine-config> <wb|wb_l1|l1>" >&2
  cfg_usage "$0"
  exit 2
fi

CFG_NAME="$1"
WHICH="$2"

case "$WHICH" in
  wb)
    PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py"
    LABEL="AM-T650-WB-CMP"
    CONTROLLER_NODE="autopilot_whole_body_direct_actuation_node"
    DESC="whole-body coupled impedance + GMO (arm in TORQUE mode)"
    ;;
  wb_l1)
    # IDENTICAL plant to `wb` -- same Isaac entrypoint, same asset, same
    # injections. Only the controller node differs, which is the point.
    PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py"
    LABEL="AM-T650-WBL1-CMP"
    CONTROLLER_NODE="autopilot_whole_body_l1_direct_actuation_node"
    ;;
  l1)
    PEGASUS_SCRIPT="$REPO_ROOT/application/robotic_arm/05_px4_direct_t650_aerial_manipulator_ros2_arm_hold.py"
    LABEL="AM-T650-L1-CMP"
    CONTROLLER_NODE="autopilot_geometric_l1_direct_actuation_node"
    DESC="geometric SE(3) + L1 adaptive (arm as a POSITION-mode servo)"
    ;;
  *)
    echo "ERROR: second argument must be 'wb', 'wb_l1' or 'l1', got '$WHICH'." >&2
    exit 2
    ;;
esac

if [[ $IN_TERM -eq 0 ]]; then
  open_new_terminal "$0" --in-terminal "$CFG_NAME" "$WHICH"
  exit 0
fi

load_machine_config "$0" "$CFG_NAME"

BASE_LAUNCHER="$SCRIPT_DIR/indoor_sim/start_single_drone_x650.sh"
PARAM_SCRIPT="$SCRIPT_DIR/apply_aerial_manipulator_px4_offboard_params.sh"
SESSION="px4_isaac"
PARAM_DELAY="${T650_AERIAL_MANIPULATOR_DIRECT_ACTUATOR_PARAM_DELAY:-8}"

export INDOOR_SIM_PEGASUS_SCRIPT="$PEGASUS_SCRIPT"
export INDOOR_SIM_VEHICLE_LABEL="$LABEL"
# SHARED with the 04/05/06 launchers on purpose: same vehicle, same saved PX4
# tune.  (The per-vehicle-profile rule separates different vehicles; cloning a
# fresh profile also costs 39 GB on this machine.)
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_am_t650"
# Fail before physics starts if the plant ever drifts from the mass BOTH
# comparison yamls declare.  This is the single number that would silently
# invalidate the whole campaign.
export PEGASUS_EXPECTED_TOTAL_MASS="3.746170"

[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || {
  echo "ERROR: missing Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2
  exit 1
}
if [[ ! "$PARAM_DELAY" =~ ^[0-9]+$ ]]; then
  echo "ERROR: T650_AERIAL_MANIPULATOR_DIRECT_ACTUATOR_PARAM_DELAY must be a non-negative integer." >&2
  exit 2
fi

if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo "ERROR: MicroXRCEAgent is not running." >&2
  echo "Start the comparison controller stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_comparison_am_stack.sh $WHICH" >&2
  exit 1
fi

# An agent alone is not proof the RIGHT controller is up -- the sibling stack
# can own an agent while publishing a different law's setpoints for the same
# vehicle.  Require this run's executable specifically, and refuse if the OTHER
# one is also alive (that would silently mix the two laws into one comparison).
controller_ready=0
for _ in $(seq 30); do
  if pgrep -f "$CONTROLLER_NODE" >/dev/null 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
if [[ $controller_ready -ne 1 ]]; then
  echo "ERROR: '$CONTROLLER_NODE' is not running." >&2
  echo "Start its stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_comparison_am_stack.sh $WHICH" >&2
  exit 1
fi
for other in autopilot_whole_body_direct_actuation_node \
             autopilot_whole_body_l1_direct_actuation_node \
             autopilot_geometric_l1_direct_actuation_node \
             autopilot_drone_geometric_l1_direct_actuation_node \
             autopilot_geometric_direct_actuation_node \
             autopilot_aerial_manipulator_direct_actuation_node \
             autopilot_direct_actuation_node \
             autopilot_sv_baseline_node; do
  [[ "$other" == "$CONTROLLER_NODE" ]] && continue
  if pgrep -f "$other" >/dev/null 2>&1; then
    echo "ERROR: '$other' is ALSO running -- two control nodes must never fly" >&2
    echo "       the same vehicle. Stop it first:  pkill -f $other" >&2
    exit 1
  fi
done

# A wall-clock DDS controller can pause PX4 actuator output during OFFBOARD
# transitions; disabling Pegasus lockstep prevents both sides waiting forever.
# The tmux-server push is required because an export alone is discarded
# whenever a tmux server already exists.
export PEGASUS_PX4_LOCKSTEP=0
if tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null; then
  echo "Pegasus PX4 lockstep: pushed to the tmux server environment"
else
  echo "Pegasus PX4 lockstep: no tmux server yet; the new session will inherit the export"
fi

echo "COMPARISON RUN -- $LABEL"
echo "  controller : $DESC"
echo "  plant      : AM_xfwd.usda on the T650 motor model (identical on both runs)"
echo "  Isaac      : $(basename "$PEGASUS_SCRIPT")"
echo "  arm stack  : NONE (comparison_driver.py owns the arm reference)"
echo "  MicroXRCEAgent: externally owned and detected"

"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
