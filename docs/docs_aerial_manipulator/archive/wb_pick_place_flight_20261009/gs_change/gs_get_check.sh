#!/usr/bin/env bash
# The arm GS Pick & Place panel's 2026-10-10 change -- a Get on the PLACE row, and
# the PICK row as editable boxes bound to the planner's pick_place_pick_point --
# checked offscreen on a PRIVATE ROS domain (77): safe beside nothing else on that
# domain; never run it on the default domain.
#
#   1. (only with OLD_PLANNER_BIN=<a planner binary from before the change>)
#      old:  the tab still loads against it, the Pick boxes stay read-only and show
#            Get's capture from pick_place/info, the Place Get reports the missing
#            service
#   2. get:  the REAL planner (ros2 launch, the HARDWARE pick-and-place yaml):
#            Place Get -> pick_place_place_point x, y, z (yaw kept), z lowered by
#            hand, Pick Get -> pick_place_pick_point, a Pick box edited, the
#            Planned-goal preview, a frozen feed refused
#
# The harness publishes the payload's mocap body itself (/obj_0/mocap) and presses
# the panel's own buttons; see gs_harness/gs_get_harness.cpp.
#
#   gs_get_check.sh <harness binary> [<png prefix>]
# Build first: the station (colcon build --packages-select utils_custom_ground_station),
# then the harness (gs_harness/CMakeLists.txt, into a build dir outside the repo).
# Planner logs go to $GS_GET_LOG_DIR (default: a new mktemp directory).
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${1:?gs_get_harness binary}"; PNG="${2:-$HERE/gs_get}"
LOGS="${GS_GET_LOG_DIR:-$(mktemp -d)}"
export ROS_DOMAIN_ID=77 QT_QPA_PLATFORM=offscreen
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
YAML="$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_place.yaml"
rc=0
PL=""

# the planner in a session of its own, stopped by its process-group id (never by name)
stop_planner() {
  [ -n "$PL" ] || return 0
  kill -INT -- "-$PL" 2>/dev/null
  for _ in $(seq 80); do kill -0 "$PL" 2>/dev/null || break; sleep 0.1; done
  kill -KILL -- "-$PL" 2>/dev/null
  PL=""
}
trap stop_planner EXIT

if [ -n "${OLD_PLANNER_BIN:-}" ]; then
  echo "== 1. a planner from BEFORE the change ($OLD_PLANNER_BIN)"
  NS=/uav_gsold
  setsid "$OLD_PLANNER_BIN" --ros-args -r "__ns:=$NS" --params-file "$YAML" \
    -p "base_com:=[0.0,-0.017854,0.0]" -p "arm_joint_sign:=[-1.0,1.0,1.0,-1.0]" \
    > "$LOGS/planner_old.log" 2>&1 &
  PL=$!
  timeout 90 "$BIN" old "$NS/fsc_open_manipulator" "$PNG" 2> "$LOGS/harness_old.log" | grep GSVAL || rc=1
  stop_planner
fi

echo "== 2. the planner as built now (hardware pick-and-place yaml)"
NS=/uav_gscheck
setsid ros2 launch fsc_trajectory_planner whole_body_trajectory_planner_launch.py \
  uav_prefix:="${NS#/}" params_file:="$YAML" base_com:="[0.0,-0.017854,0.0]" \
  arm_joint_sign:="[-1.0,1.0,1.0,-1.0]" > "$LOGS/planner_new.log" 2>&1 &
PL=$!
timeout 120 "$BIN" get "$NS/fsc_open_manipulator" "$PNG" 2> "$LOGS/harness_new.log" | grep GSVAL || rc=1
stop_planner
echo "logs: $LOGS"
exit $rc
