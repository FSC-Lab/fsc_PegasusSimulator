#!/usr/bin/env bash
# X650 + AK40-10 winch + slung payload, PX4 SITL primary, for rehearsing the LAB slung-load stack
# (autopilot_drone_slung_load_node + the lab AK cable node + the ground stations) on one machine.
#
# Plant: application/slungload/px4_x650_slung_load_ak_winch.py (header: masses, cable stopper,
# drum model, sim-only /sim/payload_attached and /sim/disturbance controls).
# Paired ROS stack (start it FIRST; it owns MicroXRCEAgent):
#   fsc_autopilot_ros2 (branch dev_slung_load_hw) scripts/isaacsim/start_x650_slung_load_sim_stack.sh <machine_config>
# One-command wrapper for both: fsc_autopilot_ros2 scripts/isaacsim/slung/run_slung_sim.sh <machine_config>
# (machine config longhao_machine_ddshz: the PX4 SITL with the lab's UXRCE_DDS_SC_HZ / _VA_HZ rates).
# Hardware counterpart: none in this repo (the vehicle runs fsc_autopilot_ros2's slung runbook).
#
# Like start_x650_direct_actuator_sitl.sh: the controller streams PX4 attitude setpoints (SAFETY)
# and actuator_motors (SLUNG) over uXRCE-DDS on the wall clock, so Pegasus lockstep is OFF, the
# plant is held to real time by the pacer (PEGASUS_REALTIME=1), and the per-run PX4 params
# UXRCE_DDS_SYNCT=0, COM_DISARM_LAND=0, COM_DISARM_PRFLT=0 are applied. PX4 profile:
# rootfs_fsc_indoor_x650_slung (separate from the bare X650's saved parameters).
set -euo pipefail

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

BASE_LAUNCHER="$SCRIPT_DIR/indoor_sim/lib/start_single_drone_x650.sh"
PARAM_SCRIPT="$SCRIPT_DIR/apply_aerial_manipulator_px4_offboard_params.sh"
SESSION="px4_isaac"
PARAM_DELAY="${X650_DIRECT_ACTUATOR_PARAM_DELAY:-8}"
[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo "ERROR: MicroXRCEAgent is not running: start the paired stack first" >&2
  echo "       (fsc_autopilot_ros2 scripts/isaacsim/start_x650_slung_load_sim_stack.sh $CFG_NAME)." >&2
  exit 1
fi

# The app comes from THIS checkout (it may be a branch worktree); the extensions and the separately distributed
# assets come from the machine config's FSC_PEGASUS_ROOT, as for every other launcher.
export INDOOR_SIM_PEGASUS_SCRIPT="$(cd -- "$SCRIPT_DIR/.." && pwd)/application/slungload/px4_x650_slung_load_ak_winch.py"
[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || { echo "ERROR: missing $INDOOR_SIM_PEGASUS_SCRIPT" >&2; exit 1; }
export INDOOR_SIM_VEHICLE_LABEL="X650-slung"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_x650_slung"
export PEGASUS_PX4_LOCKSTEP=0
export PEGASUS_REALTIME="${PEGASUS_REALTIME:-1}"
export PEGASUS_HEADLESS="${PEGASUS_HEADLESS:-0}"
# PX4 picks the DDS domain itself (UXRCE_DDS_DOM_ID, default 0), not the agent: match the ROS domain.
# PX4_PARAM_* reaches PX4 through the pane environment (rcS applies every PX4_PARAM_<name>).
export PX4_PARAM_UXRCE_DDS_DOM_ID="${ROS_DOMAIN_ID:-0}"
# Lab uXRCE rates for sensor_combined / vehicle_attitude (the 11-20 flight bag: ~400 Hz). Only a PX4 with the
# UXRCE_DDS_SC_HZ / _VA_HZ patch has them (machine config longhao_machine_ddshz); stock PX4 ignores them
# (~80 Hz). In SITL the IMU is the 250 Hz physics step, so 400 yields 250.
export PX4_PARAM_UXRCE_DDS_SC_HZ="${SLUNG_SIM_PX4_SC_HZ:-400}"
export PX4_PARAM_UXRCE_DDS_VA_HZ="${SLUNG_SIM_PX4_VA_HZ:-400}"
# The plant knobs are read from the environment by the app; a pane inherits the tmux SERVER's
# environment, so push them there as well (start_x650_direct_actuator_sitl.sh's pattern).
for v in SLUNG_SIM_L0 SLUNG_SIM_ATTACHED SLUNG_SIM_DRONE_MASS SLUNG_SIM_PAYLOAD_MASS \
         SLUNG_SIM_DRUM_RADIUS SLUNG_SIM_ROD_LEN ROS_DOMAIN_ID ROS_LOCALHOST_ONLY PX4_PARAM_UXRCE_DDS_DOM_ID \
         PX4_PARAM_UXRCE_DDS_SC_HZ PX4_PARAM_UXRCE_DDS_VA_HZ; do
  if [[ -n "${!v:-}" ]]; then
    tmux setenv -g "$v" "${!v}" 2>/dev/null || true
  else
    tmux setenv -gu "$v" 2>/dev/null || true
  fi
done
tmux setenv -g PEGASUS_PX4_LOCKSTEP 0 2>/dev/null || true

echo "X650 slung-load SITL: lockstep OFF, real-time pacer ${PEGASUS_REALTIME}, headless ${PEGASUS_HEADLESS},"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
"$PARAM_SCRIPT" "$SESSION" "0.0" "$PARAM_DELAY" &
exec "$BASE_LAUNCHER" --in-terminal "$CFG_NAME"
