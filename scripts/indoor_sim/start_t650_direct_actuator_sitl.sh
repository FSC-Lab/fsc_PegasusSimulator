#!/usr/bin/env bash
set -euo pipefail

# Controller-neutral T650 direct-actuator simulation:
#
#   external ROS 2 controller -> PX4 ActuatorMotors gate
#       -> HIL_ACTUATOR_CONTROLS -> calibrated T650 Isaac plant
#
# The external controller owns MicroXRCEAgent and all OFFBOARD/actuator topics.
# This launcher only configures and starts PX4 SITL plus the simulated plant.
#
# Identical in every respect to start_x650_direct_actuator_sitl.sh except that it
# launches the T650 plant: MN4010 + 15x5" motors
# (docs/propeller_testing/MN_4010_15x5_report.pdf) and 2.95 kg total mass,
# against the X650's MN4014 and 3.5 kg. Pair with fsc_autopilot_ros2's T650
# config.
#
# Two T650-specific numbers worth having in hand before flying it:
#   * Expected hover command is ~0.503, not the X650's ~0.480. The MN4010's top
#     rotor speed is lower (730.05 vs 817.59 rad/s) at a nearly identical thrust
#     constant, so the same weight sits higher on the stick.
#   * Rotor lag is LARGER: lambda 10.0265 vs 10.51 1/s, i.e. tau 99.7 vs 95.1 ms.
#     That costs phase margin in every inner loop, PX4's and this repo's alike.

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

BASE_LAUNCHER="$SCRIPT_DIR/indoor_sim/start_single_drone_t650.sh"
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

# ...but the export alone is not enough, and silently was not working. This script
# ends by exec'ing the base launcher, which starts Isaac inside a NEW tmux session,
# and a new session inherits the environment of the already-running tmux SERVER --
# not of this shell. Under the documented start order the controller stack runs
# first, so a server always exists by now and the export above is discarded. Isaac
# then comes up with lockstep enabled (the default), and entering DIRECT deadlocks
# the PX4/Isaac HIL link: PX4 waits for Isaac, Isaac waits for PX4, and the PX4
# console fills with "ERROR [simulator_mavlink] poll timeout 0, 111".
#
# Push the value onto the server as well. If no server is running yet this fails
# harmlessly -- in that case the base launcher creates the first session, which does
# inherit the exported value above, so both paths end up correct.
# Diagnosed live 2026-07-30; see fsc_autopilot_ros2
# docs/direct_actuator_control/progress_and_next_steps.md steps 0 and 3.
if tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null; then
  echo "Pegasus PX4 lockstep: pushed to the tmux server environment"
else
  echo "Pegasus PX4 lockstep: no tmux server yet; the new session will inherit the export"
fi

echo "Starting controller-neutral T650 direct-actuator SITL."
echo "Plant: MN4010 + 15x5\" motors, 2.95 kg (expected hover command ~0.503)"
echo "Pegasus PX4 lockstep: disabled"
echo "MicroXRCEAgent: externally owned and detected"
echo "No controller or actuator publisher will be started by this launcher."

# Apply the wall-clock DDS timestamp and HIL auto-disarm settings after the PX4
# shell is ready. These changes intentionally remain per-run and are not saved.
"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

# Reuse the validated indoor T650 PX4/Isaac orchestration and cleanup. Passing
# --in-terminal prevents the base launcher from opening a second terminal.
exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
