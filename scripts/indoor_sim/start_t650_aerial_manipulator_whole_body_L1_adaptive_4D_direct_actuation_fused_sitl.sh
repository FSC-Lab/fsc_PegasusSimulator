#!/usr/bin/env bash
set -euo pipefail

# AERIAL-MANIPULATOR WHOLE-BODY + L1 ADAPTIVE (4-D attribution) simulation with
# the controller's position/velocity feedback taken from PX4 EKF2's FUSED
# odometry instead of the raw mocap message. Added 2026-09-21 (user request):
# the simulation proof of the hardware workflow
#   fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator_fused.sh
#
# THE PLANT, PX4 PROFILE, ARM STACK AND GROUND STATIONS ARE UNCHANGED. The
# feedback source is chosen entirely on the controller side (which estimator
# executable the stack starts), and EKF2 already fuses the emulated mocap in
# every indoor rig -- EKF2_EV_CTRL=15, EKF2_HGT_REF=3 (vision), no GPS, no mag,
# set by start_single_drone_x650.sh. So this launcher adds ONE thing to
#   start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh
# and then hands over to it: it REFUSES to start unless the fused estimator
# (indoor_state_estimator_node) is the one publishing
# state_estimator/local_position/odom. The two estimators publish the same
# topic and are otherwise indistinguishable from the Isaac side, so without
# this check a "fused" run could silently be a raw-mocap one -- the same
# reason the 4-D launcher reads wb_l1_four_d off the running node.
#
# PAIR WITH (started FIRST -- it owns MicroXRCEAgent):
#   fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh
#
# Operating procedure is the 4-D rig's (Command.md 7.17.1), plus one check
# before arming -- EKF2 must be fusing the emulated mocap:
#   ros2 topic echo /uav_0/fmu/out/estimator_status_flags
#     cs_ev_pos, cs_ev_hgt, cs_ev_vel, cs_ev_yaw, cs_yaw_align all true
# and, in the hover, the fused feedback should sit on the ground truth:
#   ros2 topic echo /uav_0/state_estimator/local_position/odom   (fused)
#   ros2 topic echo /uav_0/mocap                                (Isaac ground truth)
#
# Usage: <this script> <machine_config>     (same as the 4-D launcher)

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
FOUR_D_LAUNCHER="$SCRIPT_DIR/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
FUSED_STACK_HINT="fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh"

[[ -x "$FOUR_D_LAUNCHER" ]] || { echo "ERROR: missing executable $FOUR_D_LAUNCHER" >&2; exit 1; }

# The 4-D launcher re-execs itself in a new terminal when called without
# --in-terminal; the estimator check below runs here, in the calling shell,
# either way, so it is never skipped.
CFG_ARG="${1:-}"
[[ "$CFG_ARG" == "--in-terminal" ]] && CFG_ARG="${2:-}"

# The stack may still be bringing its panes up: wait for the fused estimator
# the same way the 4-D launcher waits for the controller.
fused_ready=0
for _ in $(seq 30); do
  if pgrep -f "indoor_state_estimator_node" >/dev/null 2>&1; then
    fused_ready=1
    break
  fi
  sleep 1
done
if pgrep -f "indoor_mocap_feedback_node" >/dev/null 2>&1; then
  echo "ERROR: indoor_mocap_feedback_node (the RAW-MOCAP estimator) is running." >&2
  echo "       This launcher is the EKF2-FUSED variant; refusing to mislabel the run." >&2
  echo "       Use start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh" >&2
  echo "       for the raw-mocap stack, or stop it and start:" >&2
  echo "  $FUSED_STACK_HINT ${CFG_ARG:-<config>}" >&2
  exit 1
fi
if [[ $fused_ready -ne 1 ]]; then
  echo "ERROR: the EKF2-fused estimator (indoor_state_estimator_node) is not running." >&2
  echo "Start its external stack first:" >&2
  echo "  $FUSED_STACK_HINT ${CFG_ARG:-<config>}" >&2
  exit 1
fi

echo -e "\033[1;36mFEEDBACK: EKF2-FUSED odometry (indoor_state_estimator_node -> Px4FusedOdomBridge). The simulated mocap is perfect, so this tests the fused path, not mocap faults.\033[0m"
echo -e "\033[1;33mBefore arming: cs_ev_pos/hgt/vel/yaw true on /uav_0/fmu/out/estimator_status_flags.\033[0m"

exec "$FOUR_D_LAUNCHER" "$@"
