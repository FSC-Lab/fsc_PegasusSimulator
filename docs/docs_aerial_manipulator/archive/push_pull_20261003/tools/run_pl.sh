#!/usr/bin/env bash
# One AUTONOMOUS push-and-pull flight in Isaac, headless, whole-body 4-D L1:
#
#   run_pl.sh <tag>        (after a clean slate in a SEPARATE call:
#                           scripts/kill_stale_sim_processes.sh -y)
#
# WB_SIM_PROFILE=push_pull: the planner block and the controller from
# ..._l1_4d_..._sim_push_pull.yaml (make_push_pull_yaml.py). Raw mocap
# odometry (PUSH_FEEDBACK=fused for the EKF2 path). PL_DRIVER_ARGS="..." goes
# to pl_mission.py; the scene's PEGASUS_PUSH_* knobs (env) reach Isaac through
# the launcher. Brings the stack and the scene up in the background, waits for
# the planner, the box and PX4, flies pl_mission.py and writes
# ../runs/<tag>.{npz,log} plus the stack / scene logs. Leaves the simulation
# running: the next clean slate closes it. The scene launcher runs
# --in-terminal (no GUI terminal window), so its output lands in <tag>.scene.log.
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
TAG="${1:?tag}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="${PL_RUNS_DIR:-$HERE/../runs}"; mkdir -p "$RUNS"   # PL_RUNS_DIR: another campaign's runs/
PEG="$HOME/fsc_PegasusSimulator"
STACKS="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim"
FB="${PUSH_FEEDBACK:-raw}"
STACK="$STACKS/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh"
[[ "$FB" == fused ]] && STACK="$STACKS/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"

(WB_SIM_PROFILE=push_pull setsid nohup bash "$STACK" shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"

# the stack refuses to start beside a running planner and checks with pgrep -f:
# a poll naming the planner while that check runs would match it
sleep 20
echo "[$(date +%T)] waiting for the planner's push-and-pull block"
for i in $(seq 60); do
  off=$(ros2 param get --no-daemon --spin-time 3 /uav_0/whole_body_trajectory_planner push_pull_ee_offset 2>/dev/null \
        | sed -nE 's/.*values? (is|are): *(.*)$/\2/p')
  [[ -n "$off" ]] && break; sleep 2
done
[[ -n "$off" ]] || { echo "the planner has no push_pull_ee_offset -- old binary? rebuild fsc_trajectory_planner"; exit 1; }
echo "[$(date +%T)] planner up, EE offset $off"
(setsid nohup env PEGASUS_HEADLESS=1 PUSH_FEEDBACK="$FB" WB_SIM_PROFILE=push_pull \
   "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_push_and_pull_sitl.sh" --in-terminal shiqi_machine \
   > "$RUNS/$TAG.scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for the box (Isaac)"
for i in $(seq 90); do
  timeout 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break; sleep 2
done
echo "[$(date +%T)] box up; letting PX4 / the arm stack settle"
sleep 45
echo "[$(date +%T)] flying"
/usr/bin/python3 "$HERE/pl_mission.py" --out "$RUNS/$TAG.npz" ${PL_DRIVER_ARGS:-} 2>&1 | tee "$RUNS/$TAG.log"
echo "[$(date +%T)] done"
