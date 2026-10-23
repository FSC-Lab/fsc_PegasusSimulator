#!/usr/bin/env bash
# One automated PS4-teleoperation check in Isaac (2026-10-01): stack -> Pegasus
# (NO real pad window) -> SAFETY takeoff + DIRECT (ps4_teleop_bringup.py up) ->
# synthetic pad through every channel (tools/ps4_pad_check.py) -> land.
#
#   run_check.sh <tag> [decoupled|wb]          (default decoupled)
#
# PREREQUISITE, as two SEPARATE shell calls first (the kill script can take its
# invoking shell with it):
#   ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
#   ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y
#
# Headless by default (PEGASUS_HEADLESS=0 for the window); the arm station still
# opens on DISPLAY (:1 on shiqi-desktop), because the check engages through it.
# `wb` flies the whole-body 4-D rig on its PS4 yaml, for the A/B.
set -uo pipefail
TAG="${1:?usage: run_check.sh <tag> [decoupled|wb]}"
RIG="${2:-decoupled}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../../.." && pwd)"
CFG=shiqi_machine
# shellcheck source=/dev/null
source "$PEG/scripts/config/$CFG.conf"
AUT="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2"
case "$RIG" in
  decoupled)
    STACK="$AUT/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh"
    SITL="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_adaptive_direct_actuation_sitl.sh"
    NODE="autopilot_geometric_l1_direct_actuation_node" ;;
  wb)
    export WB_SIM_YAML="$AUT/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_ps4test.yaml"
    STACK="$AUT/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh"
    SITL="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
    NODE="autopilot_whole_body_l1_direct_actuation_node" ;;
  *) echo "rig must be decoupled or wb" >&2; exit 2 ;;
esac
LOGS="$HERE/logs"; DATA="$HERE/data"; mkdir -p "$LOGS" "$DATA"
export DISPLAY="${DISPLAY:-:1}" PEGASUS_HEADLESS="${PEGASUS_HEADLESS:-1}"
# no real pad on this run: the check publishes rc/input itself
export PEGASUS_JOY_DEV_NODE=/nonexistent/js_disabled_for_the_synthetic_check
PROFILE="$PEG/docs/docs_aerial_manipulator/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
PY="/usr/bin/python3"

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck source=/dev/null
source "${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/install/setup.bash"
set -u
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "=== [$RIG/$TAG] 1. controller stack $(date +%T) ==="
setsid nohup "$STACK" "$CFG" uav_0 > "$LOGS/stack_${RIG}_$TAG.log" 2>&1 < /dev/null &
for _ in $(seq 60); do pgrep -x MicroXRCEAgent >/dev/null && break; sleep 2; done
for _ in $(seq 60); do pgrep -f "$NODE" >/dev/null && break; sleep 1; done
pgrep -f "$NODE" >/dev/null || { echo "FAILED: $NODE never came up"; tail -30 "$LOGS/stack_${RIG}_$TAG.log"; exit 1; }
sleep 8

echo "=== [$RIG/$TAG] 2. Pegasus / PX4 / arm (headless=$PEGASUS_HEADLESS) ==="
setsid nohup "$SITL" --in-terminal "$CFG" > "$LOGS/pegasus_${RIG}_$TAG.log" 2>&1 < /dev/null &

echo "=== [$RIG/$TAG] 3. waiting for odometry, EKF alignment, arm, planner ==="
ok=0
for _ in $(seq 120); do
  if timeout 5 ros2 topic echo --once /uav_0/state_estimator/local_position/odom >/dev/null 2>&1; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: no odometry"; tail -40 "$LOGS/pegasus_${RIG}_$TAG.log"; exit 1; }
ok=0
for _ in $(seq 60); do
  f=$(timeout 5 ros2 topic echo --once /uav_0/fmu/out/estimator_status_flags 2>/dev/null)
  if grep -q "cs_yaw_align: true" <<<"$f" && grep -q "cs_ev_pos: true" <<<"$f" && grep -q "cs_ev_yaw: true" <<<"$f"; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || { echo "FAILED: EKF never aligned"; exit 1; }
ok=0
for _ in $(seq 60); do
  if timeout 5 ros2 topic echo --once /uav_0/fsc_open_manipulator/joint_states >/dev/null 2>&1; then ok=1; break; fi
  sleep 5
done
[ "$ok" = 1 ] || echo "WARNING: no arm joint states"
pgrep -f "[w]hole_body_trajectory_planner" >/dev/null || echo "WARNING: no trajectory planner running"
sleep 10

echo "=== [$RIG/$TAG] 4. SAFETY takeoff + DIRECT $(date +%T) ==="
FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE" "$PY" "$PEG/application/robotic_arm/utils/ps4_teleop_bringup.py" up \
  > "$LOGS/bringup_${RIG}_$TAG.log" 2>&1
rc=$?
cat "$LOGS/bringup_${RIG}_$TAG.log"
[ "$rc" = 0 ] || { echo "FAILED: bring-up rc=$rc"; exit 1; }

echo "=== [$RIG/$TAG] 5. synthetic pad $(date +%T) ==="
FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE" "$PY" "$HERE/tools/ps4_pad_check.py" \
  --out "$DATA/${RIG}_$TAG.npz" 2>&1 | tee "$LOGS/check_${RIG}_$TAG.log"
crc=${PIPESTATUS[0]}

echo "=== [$RIG/$TAG] 6. land $(date +%T) ==="
FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE" "$PY" "$PEG/application/robotic_arm/utils/ps4_teleop_bringup.py" land \
  > "$LOGS/land_${RIG}_$TAG.log" 2>&1
tail -3 "$LOGS/land_${RIG}_$TAG.log"
tmux capture-pane -J -p -t px4_isaac:0.1 -S -5000 > "$LOGS/isaac_pane_${RIG}_$TAG.txt" 2>/dev/null
echo "RTF: $(grep -o 'RTF [0-9.]*' "$LOGS/isaac_pane_${RIG}_$TAG.txt" | awk '{print $2}' | tr '\n' ' ')"
echo "=== [$RIG/$TAG] done (check rc=$crc) ==="
exit "$crc"
