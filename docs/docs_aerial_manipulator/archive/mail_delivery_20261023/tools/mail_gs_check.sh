#!/usr/bin/env bash
# The Mail Envelope tab, end to end without Isaac (2026-10-23): the REAL planner on the
# mail-delivery yaml + the REAL tab (offscreen, gs_harness/gs_mail_snapshot) + the loopback
# rig (mail_rig.py), on a PRIVATE ROS domain (77) under /uav_mailshot -- no Isaac, no PX4.
# Panel grabs + widget rectangles land in <out dir>.
#   mail_gs_check.sh <gs_mail_snapshot binary> <out dir> [params yaml] [launch overrides...]
# (the hardware yaml needs the hardware launch arguments:
#    base_com:="[0.0,-0.017854,0.0]" arm_joint_sign:="[-1.0,1.0,1.0,-1.0]"  and MAIL_RIG_SIGN=hw)
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
BIN="${1:?gs_mail_snapshot binary}"; OUT="${2:?out dir}"
YAML="${3:-$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_mail_delivery.yaml}"
shift 3 2>/dev/null || shift $#
mkdir -p "$OUT"
export ROS_DOMAIN_ID=77 QT_QPA_PLATFORM=offscreen
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/../../q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
export RCUTILS_LOGGING_BUFFERED_STREAM=0 PYTHONUNBUFFERED=1
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
setsid ros2 launch fsc_trajectory_planner whole_body_trajectory_planner_launch.py \
  uav_prefix:=uav_mailshot params_file:="$YAML" "$@" > "$OUT/planner.log" 2>&1 &
PL=$!
setsid "$BIN" /uav_mailshot/fsc_open_manipulator "$OUT" > "$OUT/gs.log" 2>&1 &
GS=$!
trap 'kill -TERM -- -$PL -$GS 2>/dev/null; sleep 1; kill -KILL -- -$PL -$GS 2>/dev/null' EXIT
sleep 5
/usr/bin/python3 "$HERE/mail_rig.py" "$YAML" "$OUT"
rc=$?
echo "--- arm GS log (gripper / step lines)"
grep -aE "Insert|Pick|GRIPPER|Exit|Mail" "$OUT/gs.log" | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-220 | head -40
exit $rc
