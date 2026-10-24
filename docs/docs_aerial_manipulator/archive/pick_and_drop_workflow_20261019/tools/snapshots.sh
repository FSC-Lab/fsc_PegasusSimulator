#!/usr/bin/env bash
# The Pick-and-Drop workflow page's arm-GS snapshots (2026-10-19): the REAL planner on the
# HARDWARE pick-and-drop yaml + the REAL Pick & Drop tab (offscreen, gs_harness) + the loopback
# rig (pnd_rig.py), on a PRIVATE ROS domain (77) under /uav_pndshot -- no Isaac, no PX4, safe
# beside a running simulation. Raw panel grabs + widget rectangles land in <out dir>; crop.py
# turns them into the page's pictures.
#   snapshots.sh <gs_pnd_snapshot binary> <out dir> [params yaml]
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
BIN="${1:?gs_pnd_snapshot binary}"; OUT="${2:?out dir}"
YAML="${3:-$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_drop.yaml}"
mkdir -p "$OUT"
export ROS_DOMAIN_ID=77 QT_QPA_PLATFORM=offscreen
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/../../q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
export RCUTILS_LOGGING_BUFFERED_STREAM=0 PYTHONUNBUFFERED=1
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
# hardware launch arguments, as the whole-body hardware stacks pass them
setsid ros2 launch fsc_trajectory_planner whole_body_trajectory_planner_launch.py \
  uav_prefix:=uav_pndshot params_file:="$YAML" base_com:="[0.0,-0.017854,0.0]" \
  arm_joint_sign:="[-1.0,1.0,1.0,-1.0]" > "$OUT/planner.log" 2>&1 &
PL=$!
setsid "$BIN" /uav_pndshot/fsc_open_manipulator "$OUT" > "$OUT/gs.log" 2>&1 &
GS=$!
trap 'kill -TERM -- -$PL -$GS 2>/dev/null; sleep 1; kill -KILL -- -$PL -$GS 2>/dev/null' EXIT
sleep 5
/usr/bin/python3 "$HERE/pnd_rig.py" "$YAML" "$OUT"
rc=$?
echo "--- arm GS log (Drop / gripper lines)"
grep -aE "Drop|GRIPPER|Exit" "$OUT/gs.log" | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-220 | head -30
exit $rc
