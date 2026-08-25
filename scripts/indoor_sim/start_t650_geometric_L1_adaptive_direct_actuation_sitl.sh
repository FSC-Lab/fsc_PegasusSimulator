#!/usr/bin/env bash
set -euo pipefail

# BARE-T650 direct-actuator simulation for the GEOMETRIC + L1-ADAPTIVE
# controller (Cai et al., Control Engineering Practice 164 (2025) 106418:
# geometric SE(3) baseline + L1 adaptive augmentation) -- the NO-ARM parallel
# of start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh.
#
#   external ROS 2 GEOMETRIC+L1 controller -> PX4 ActuatorMotors gate
#       -> HIL_ACTUATOR_CONTROLS -> calibrated bare-T650 Isaac plant
#
# THE PLANT IS THAT OF start_t650_geometric_direct_actuator_sitl.sh PLUS A
# PAYLOAD -- same x650_new.usd asset, same MN4010 + 15x5" calibration, same
# lockstep-off configuration, but since 2026-08-24 this launcher defaults to
# PEGASUS_PAYLOAD_MASS=0.769 kg (see the PAYLOAD block below), so the body mass
# is 3.719 kg and the total the solver sees is 3.802921 kg. No arm, no arm
# stack, no arm ground station: with r_os identically zero on a bare airframe
# the paired controller carries no arm model at all. This script exists as a
# named parallel so the GEOMETRIC+L1 ROS 2 stack (fsc_autopilot_ros2's
# scripts/isaacsim/start_geometric_l1_direct_actuation_t650_stack.sh, which
# runs the single-drone L1 node autopilot_drone_geometric_l1_direct_actuation_node
# with params_single_drone_geometric_l1_direct_actuation_t650.yaml) has an
# unambiguous Pegasus counterpart, mirroring how every other rig pairs 1:1.
#
# THE ROBUSTNESS TEST LIVES ON THE CONTROLLER SIDE, NOT HERE: the paired yaml
# ships a deliberate +20% thrust-coefficient mismatch (alloc_thrust_coeff
# 5.6159172e-05 against this plant's true 4.679931e-05). Pegasus is NOT
# changed -- the plant keeps the truth, the controller believes the wrong
# number, and the L1 augmentation is what closes the 16.67% physical thrust
# deficit (expect u_L1 thrust, l1_control_debug [16], settling near +7.5 N in a
# settled DIRECT hover with the 769 g payload, +6 N bare -- not near zero; the
# demand is mg*0.2 with the mass matched, so it scales with the payload).
#
# Numbers worth having in hand before flying it:
#   * Expected hover command is ~0.5741 loaded (~0.5025 bare), all four motors
#     symmetric (a front/rear split means an AM yaml got loaded somewhere).
#   * Rotor lag: lambda 10.0265 1/s (tau 99.7 ms). The paired attitude pair
#     (l1geo_kr 2.4 / komega 0.73) is the bare geometric node's sim-flown
#     loop shape against that pole -- do not stiffen casually.
#
# Run order (Command.md section 7.13): the ROS 2 stack FIRST (it owns
# MicroXRCEAgent and the L1 node this launcher requires), then this script,
# then OFFBOARD -> arm -> SAFETY takeoff -> set_direct_mode true. The mode
# service is /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode
# (same name as the AM L1 rig -- frozen-runbook convention).

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
PARAM_DELAY="${T650_GEOMETRIC_L1_DIRECT_ACTUATOR_PARAM_DELAY:-8}"

[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
if [[ ! "$PARAM_DELAY" =~ ^[0-9]+$ ]]; then
  echo "ERROR: T650_GEOMETRIC_L1_DIRECT_ACTUATOR_PARAM_DELAY must be a non-negative integer." >&2
  exit 2
fi

if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo "ERROR: MicroXRCEAgent is not running." >&2
  echo "Start the agent from the external controller stack before this launcher:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_stack.sh" >&2
  exit 1
fi

# MicroXRCEAgent alone is not proof that the matching controller stack is
# active; another launcher can own an agent while publishing incompatible PX4
# setpoints. Require this variant's executable specifically. The DECORATED
# single-drone name is matched on purpose: it is the only pattern that matches
# THIS fork. The decoration is infixed ("autopilot_" + "drone_" + the rest), so
# the two L1 names are mutually non-matching under pgrep -f -- neither is a
# substring of the other. That is what makes this check able to refuse the AM
# stack (3.746 kg arm-laden model) flying this 3.034 kg plant.
controller_ready=0
for _ in $(seq 30); do
  if pgrep -f "autopilot_drone_geometric_l1_direct_actuation_node" >/dev/null 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
if [[ $controller_ready -ne 1 ]]; then
  echo "ERROR: the single-drone geometric+L1 controller is not running." >&2
  echo "Start its external stack first:" >&2
  echo "  fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_stack.sh $CFG_NAME" >&2
  exit 1
fi

# A wall-clock DDS controller can pause PX4 actuator output during OFFBOARD
# transitions. Disabling Pegasus lockstep prevents both sides waiting forever.
export PEGASUS_PX4_LOCKSTEP=0

# ...but the export alone is not enough: this script ends by exec'ing the base
# launcher, which starts Isaac inside a NEW tmux session, and a new session
# inherits the environment of the already-running tmux SERVER -- not of this
# shell. Under the documented start order the controller stack runs first, so
# a server always exists by now and the export above would be discarded,
# deadlocking the PX4/Isaac HIL link on the first DIRECT entry ("poll timeout
# 0, 111"). Push the value onto the server as well; if no server is running
# yet this fails harmlessly and the export covers it. Diagnosed live
# 2026-07-30 -- see start_t650_geometric_direct_actuator_sitl.sh.
if tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null; then
  echo "Pegasus PX4 lockstep: pushed to the tmux server environment"
else
  echo "Pegasus PX4 lockstep: no tmux server yet; the new session will inherit the export"
fi

# PAYLOAD (added 2026-08-24): the 769 g loaded campaign. The paired yaml's
# vehicle_mass, thrust_scaling and idle_thrust are all set for the LOADED plant, so
# the two MUST move together -- flying this launcher with PEGASUS_PAYLOAD_MASS=0
# against the loaded yaml gives the controller a 769 g phantom and it will climb.
# Set PEGASUS_PAYLOAD_MASS=0 ONLY together with restoring the yaml's bare numbers
# (vehicle_mass 3.033921, thrust_scaling 0.040232, idle_thrust 0.203169).
PAYLOAD_MASS="${PEGASUS_PAYLOAD_MASS:-0.769}"
if [[ ! "$PAYLOAD_MASS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: PEGASUS_PAYLOAD_MASS must be a non-negative decimal number." >&2
  exit 2
fi
export PEGASUS_PAYLOAD_MASS="$PAYLOAD_MASS"

echo "Starting bare-T650 direct-actuator SITL (GEOMETRIC+L1-stack pairing)."
if [[ "$PAYLOAD_MASS" == "0" || "$PAYLOAD_MASS" == "0.0" ]]; then
  echo "Plant: MN4010 + 15x5\" motors, 3.034 kg total (expected hover command ~0.5025)"
else
  echo "Plant: MN4010 + 15x5\" motors, airframe 3.033921 kg + PAYLOAD ${PAYLOAD_MASS} kg"
  echo "  -> 3.802921 kg total at 0.769 kg (expected hover command ~0.5741); the Isaac"
  echo "     pane prints the authoritative total -- the yaml's vehicle_mass must match it."
fi
echo "Controller: L1-augmented geometric SE(3) (Cai et al., CEP 2025), no arm model;"
echo "  the paired yaml carries the +20% thrust-coefficient robustness mismatch,"
echo "  so u_L1 thrust settles near +7.5 N loaded (near +6 N bare) in DIRECT hover."
echo "Pegasus PX4 lockstep: disabled"
echo "MicroXRCEAgent: externally owned and detected"
echo "No controller or actuator publisher will be started by this launcher."

# Apply the wall-clock DDS timestamp and HIL auto-disarm settings after the PX4
# shell is ready. These changes intentionally remain per-run and are not saved.
"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &

# Reuse the validated indoor T650 PX4/Isaac orchestration and cleanup. Passing
# --in-terminal prevents the base launcher from opening a second terminal.
exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
