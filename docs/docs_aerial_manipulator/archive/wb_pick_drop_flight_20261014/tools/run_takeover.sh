#!/usr/bin/env bash
# One pilot-takeover test flight in Isaac, headless (2026-10-14):
#
#   run_takeover.sh wb|geo <tag>      (after a clean slate in a SEPARATE call:
#                                      scripts/kill_stale_sim_processes.sh -y)
#
#   wb   whole-body 4-D L1 rig (mirror plant, the same controller code as the hardware stack)
#   geo  decoupled geometric + L1 rig
# Brings the stack and the plain free-flight scene up, takes off and enters DIRECT
# (ps4_teleop_bringup.py up), runs takeover_test.py, lands, and writes
# ../runs/<tag>.{npz,log} + the stack / scene logs. Leaves the simulation running.
set -o pipefail
RIG="${1:?rig: wb or geo}"; TAG="${2:?tag}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$HERE/../runs"; mkdir -p "$RUNS"
PEG="$HOME/fsc_PegasusSimulator"
STACKS="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
case "$RIG" in
  wb)  STACK="$STACKS/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh"
       SCENE="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh"
       DA=whole_body_direct_actuation ;;
  geo) # the decoupled pairing that flies (10-09 campaign): the plain geometric _sim.yaml still carries the old
       # stress plant and deadlocks in the SAFETY takeoff at the 0.4 m UDE gate
       CFG="$HOME/ros2_ws/src/fsc_autopilot_ros2/config"
       export WB_SIM_YAML="$CFG/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim_pick_and_place.yaml"
       export WB_PLANNER_YAML="$CFG/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_place.yaml"
       STACK="$STACKS/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh"
       SCENE="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_pick_and_place_sitl.sh"
       DA=geometric_l1_direct_actuation ;;
  *) echo "rig must be wb or geo" >&2; exit 2 ;;
esac
(setsid nohup bash "$STACK" shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
sleep 25          # let the stack get past its pgrep guards before anything else runs
(setsid nohup env PEGASUS_HEADLESS=1 "$SCENE" shiqi_machine > "$RUNS/$TAG.scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for odometry (Isaac + PX4)"
for i in $(seq 90); do
  timeout 5 ros2 topic echo --no-daemon --once /uav_0/state_estimator/local_position/odom > /dev/null 2>&1 && break; sleep 2
done
echo "[$(date +%T)] odometry up; letting PX4 / the arm stack settle"
sleep 45
BRING="$PEG/application/robotic_arm/utils/ps4_teleop_bringup.py"
echo "[$(date +%T)] take-off + DIRECT"
/usr/bin/python3 "$BRING" up --da "$DA" --hover-z 1.0 2>&1 | tee "$RUNS/$TAG.up.log" || exit 1
echo "[$(date +%T)] takeover test"
/usr/bin/python3 "$HERE/takeover_test.py" --da "$DA" --out "$RUNS/$TAG.npz" 2>&1 | tee "$RUNS/$TAG.log"
echo "[$(date +%T)] land"
/usr/bin/python3 "$BRING" land --da "$DA" 2>&1 | tee -a "$RUNS/$TAG.log"
for p in $(tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index}' 2>/dev/null); do
  tmux capture-pane -J -p -S - -t "$p" 2>/dev/null
done | sed 's/\x1b\[[0-9;]*m//g' | grep -E "LEFT OFFBOARD|OFFBOARD again|Returned to SAFETY|WATCHDOG|ENTERING|Traceback" > "$RUNS/$TAG.panes.log" || true
echo "[$(date +%T)] done"
