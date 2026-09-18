#!/usr/bin/env bash
# One matched end-effector-trajectory flight on either aerial-manipulator rig,
# clean slate to npz (2026-09-18, the whole-body vs decoupled comparison).
#
#   am_compare_cycle.sh <wb|decoupled> <run-tag> [machine-config] -- <driver args>
#
#   wb         the whole-body + L1 4-D rig (start_whole_body_l1_4d_..._stack.sh +
#              start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh)
#   decoupled  the geometric+L1 rig of Cai et al. (start_geometric_l1_..._stack.sh +
#              start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh), which since
#              2026-09-18 flies the SAME Isaac plant (06, arm in position-command mode),
#              the SAME planner (fsc_trajectory_planner, one yaml section) and the
#              SAME EE task, converted for it by decoupled_reference_bridge.py.
#
# The mission is am_ee_compare_driver.py's: SAFETY takeoff -> DIRECT -> stability
# gate -> planner shape parameters -> select -> time scale -> Go-to-start -> Start ->
# run -> SAFETY -> land. Everything after `--` goes to the driver (shape, radius,
# lap time, laps, time scale, yaw ...). Output: $AM_CMP_OUT/<which>_<tag>.npz.
set -uo pipefail
WHICH="${1:?usage: am_compare_cycle.sh <wb|decoupled> <run-tag> [machine-config] -- <driver args>}"
TAG="${2:?usage}"
shift 2
CFG="shiqi_machine"
if [[ "${1:-}" != "--" && $# -gt 0 ]]; then CFG="$1"; shift; fi
[[ "${1:-}" == "--" ]] && shift
case "$WHICH" in wb|decoupled) ;; *) echo "first arg must be wb or decoupled"; exit 2;; esac

PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
# shellcheck source=/dev/null
source "$PEG/scripts/config/${CFG}.conf"
AUT="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2"
if [[ "$WHICH" == wb ]]; then
  STACK="$AUT/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh"
  SITL="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
  NODE="autopilot_whole_body_l1_direct_actuation_node"
else
  STACK="$AUT/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh"
  SITL="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh"
  NODE="autopilot_geometric_l1_direct_actuation_node"
fi
OUT="${AM_CMP_OUT:-$PEG/docs/docs_aerial_manipulator/wb_vs_decoupled_20260918}"
LOGS="$OUT/logs"; mkdir -p "$OUT" "$LOGS"

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck source=/dev/null
source "${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/install/setup.bash"
set -u
/usr/bin/python3 -c "from fsc_autopilot_ros2_msgs.msg import WholeBodyReference" 2>/dev/null || {
  echo "FAILED: fsc_autopilot_ros2_msgs is not the built one." >&2; exit 1; }
[[ -d "${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/install/fsc_trajectory_planner" ]] || {
  echo "FAILED: fsc_trajectory_planner not built." >&2; exit 1; }

echo "=== [$WHICH/$TAG] 0. clean slate ==="
"$AUT/scripts/isaacsim/stop_isaacsim_stack.sh" >/dev/null 2>&1
"$PEG/scripts/kill_stale_sim_processes.sh" -y  >/dev/null 2>&1
pkill -f "[w]hole_body_trajectory_planner" 2>/dev/null; pkill -f "[d]ecoupled_reference_bridge" 2>/dev/null
sleep 5

echo "=== [$WHICH/$TAG] 1. controller stack ==="
setsid nohup "$STACK" "$CFG" uav_0 > "$LOGS/stack_$TAG.log" 2>&1 < /dev/null &
for _ in $(seq 60); do pgrep -x MicroXRCEAgent >/dev/null && break; sleep 2; done
pgrep -x MicroXRCEAgent >/dev/null || { echo "FAILED: agent never came up"; tail -20 "$LOGS/stack_$TAG.log"; exit 1; }
for _ in $(seq 60); do pgrep -f "$NODE" >/dev/null && break; sleep 1; done
pgrep -f "$NODE" >/dev/null || { echo "FAILED: $NODE never came up"; tail -30 "$LOGS/stack_$TAG.log"; exit 1; }
sleep 8

echo "=== [$WHICH/$TAG] 2. Pegasus / PX4 / arm ==="
DISPLAY="${DISPLAY:-:0}" setsid nohup "$SITL" --in-terminal "$CFG" > "$LOGS/pegasus_$TAG.log" 2>&1 < /dev/null &

echo "=== [$WHICH/$TAG] 3. waiting for odometry ==="
ok=0
for _ in $(seq 120); do
  if timeout 5 ros2 topic echo --once /uav_0/state_estimator/local_position/odom >/dev/null 2>&1; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: no odometry"; tail -40 "$LOGS/pegasus_$TAG.log"; exit 1; }
echo "=== [$WHICH/$TAG] 3b. waiting for EKF yaw/EV alignment ==="
ok=0
for _ in $(seq 60); do
  f=$(timeout 5 ros2 topic echo --once /uav_0/fmu/out/estimator_status_flags 2>/dev/null)
  if grep -q "cs_yaw_align: true" <<<"$f" && grep -q "cs_ev_pos: true" <<<"$f" && grep -q "cs_ev_yaw: true" <<<"$f"; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: EKF never aligned"; exit 1; }
echo "=== [$WHICH/$TAG] 3c. waiting for the arm stack + planner ==="
ok=0
for _ in $(seq 60); do
  if timeout 5 ros2 topic echo --once /uav_0/fsc_open_manipulator/joint_states >/dev/null 2>&1; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || echo "WARNING: no arm joint states -- DIRECT entry may refuse"
pgrep -f "[w]hole_body_trajectory_planner" >/dev/null || echo "WARNING: no trajectory planner running"
if [[ "$WHICH" == decoupled ]]; then
  pgrep -f "[d]ecoupled_reference_bridge" >/dev/null || echo "WARNING: no reference bridge running"
fi
sleep 10

echo "=== [$WHICH/$TAG] 4. flying ($*) ==="
/usr/bin/python3 "$PEG/application/robotic_arm/utils/am_ee_compare_driver.py" --rig "$WHICH" \
    --out "$OUT/${WHICH}_${TAG}.npz" "$@" > "$LOGS/${WHICH}_${TAG}.log" 2>&1
rc=$?
echo "=== [$WHICH/$TAG] done (driver rc=$rc) ==="
tail -6 "$LOGS/${WHICH}_${TAG}.log"
exit $rc
