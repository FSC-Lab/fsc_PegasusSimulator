#!/usr/bin/env bash
# X650 direct-actuator SITL for the CCM (rotor-thrust RCCM) stack: start_x650_direct_actuator_sitl.sh
# (lockstep OFF, the DIRECT wall-clock DDS path, X650 PX4 profile rootfs_fsc_indoor_x650) with the
# Isaac app application/px4_base/05_px4_single_drone_x650.py instead of 03:
#   * total mass 3.25 kg, the weighed X650 with battery, the CCM was trained for (PEGASUS_X650_TOTAL_MASS=<kg>
#     overrides; the controller yamls' vehicle_mass must match, the app prints the truth);
#   * the real-time pacer (machine config SIM_REALTIME=1 / PEGASUS_REALTIME=1) and the
#     real-time I/O of x650_bare_frame_utils (ROS 2 ground truth spun at 10 Hz, mag 50 / GPS 10 Hz);
#   * drag injection: PEGASUS_LINEAR_DRAG / PEGASUS_ANGULAR_DRAG="x,y,z".
# Run the ROS 2 side FIRST (fsc_autopilot_ros2 scripts/isaacsim/start_ccm_rotor_*_x650_stack.sh).
#
# Usage: [PEGASUS_X650_TOTAL_MASS=<kg>] start_x650_ccm_direct_actuator_sitl.sh [--in-terminal] <machine_config>
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

if [[ "${1:-}" != "--in-terminal" ]]; then
  [[ $# -eq 1 ]] || { echo "ERROR: must provide config name." >&2; cfg_usage "$0"; exit 2; }
  open_new_terminal "$0" --in-terminal "$1"
  exit 0
fi

export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/px4_base/05_px4_single_drone_x650.py"
export INDOOR_SIM_VEHICLE_LABEL="X650"
[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || { echo "ERROR: missing $INDOOR_SIM_PEGASUS_SCRIPT" >&2; exit 1; }
echo "X650 CCM SITL: Isaac app 05_px4_single_drone_x650.py, total mass ${PEGASUS_X650_TOTAL_MASS:-3.25} kg"
exec "$SCRIPT_DIR/indoor_sim/archive/start_x650_direct_actuator_sitl.sh" "$@"
