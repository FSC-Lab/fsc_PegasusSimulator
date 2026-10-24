#!/usr/bin/env bash
# One AUTONOMOUS VALVE-TURNING flight in Isaac, headless, whole-body 4-D L1 (the box push's run_pl.sh, 2026-10-15):
#
#   run_valve.sh <tag>        (after a clean slate in a SEPARATE call:
#                           scripts/kill_stale_sim_processes.sh -y)
#
# WB_SIM_PROFILE=valve_turning: the planner block (push_pull_turn_deg 360) and the controller from
# ..._l1_4d_..._sim_valve_turning.yaml (make_valve_turning_yaml.py). Raw mocap
# odometry (PUSH_FEEDBACK=fused for the EKF2 path). PL_DRIVER_ARGS="..." goes
# to pl_mission.py; the scene's PEGASUS_VALVE_* knobs (env) reach Isaac through
# the launcher. Brings the stack and the scene up in the background, waits for
# the planner, the valve and PX4, flies the box push's pl_mission.py (the same five services; its "push" step is the turn) and writes
# ../runs/<tag>.{npz,log} plus the stack / scene logs. Leaves the simulation
# running: the next clean slate closes it. The scene launcher runs
# --in-terminal (no GUI terminal window), so its output lands in <tag>.scene.log.
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
TAG="${1:?tag}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="${PL_RUNS_DIR:-$HERE/../runs}"; mkdir -p "$RUNS"   # PL_RUNS_DIR: another campaign's runs/
PUSH_TOOLS="$HERE/../../push_pull_20261003/tools"   # pl_mission.py lives with the box push
PEG="$HOME/fsc_PegasusSimulator"
STACKS="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim"
FB="${VALVE_FEEDBACK:-raw}"
STACK="$STACKS/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh"
[[ "$FB" == fused ]] && STACK="$STACKS/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"

(WB_SIM_PROFILE=valve_turning setsid nohup bash "$STACK" shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"

# the stack refuses to start beside a running planner and checks with pgrep -f:
# a poll naming the planner while that check runs would match it
sleep 20
echo "[$(date +%T)] waiting for the planner's valve-turning block"
for i in $(seq 60); do
  off=$(ros2 param get --no-daemon --spin-time 3 /uav_0/whole_body_trajectory_planner push_pull_turn_deg 2>/dev/null \
        | sed -nE 's/.*values? (is|are): *(.*)$/\2/p')
  [[ -n "$off" ]] && break; sleep 2
done
[[ -n "$off" ]] || { echo "the planner has no push_pull_turn_deg -- old binary? rebuild fsc_trajectory_planner"; exit 1; }
echo "[$(date +%T)] planner up, turn $off deg"
# VALVE_WINDOWED=1 (2026-10-18): the scene with its window on DISPLAY :1 (it slows to RTF
# ~0.83 in contact -- the windowed failures); default headless
HL="PEGASUS_HEADLESS=1"; [[ "${VALVE_WINDOWED:-0}" == 1 ]] && HL="DISPLAY=:1"
# ARM_GS_AUTO_EXIT_DELAY=0.0 / ARM_GS_AUTO_STEPS=false (the defaults, 2026-10-22): the arm GS's
# Valve Turning tab runs Ready / Turn / Exit by itself since that day (the box push's automation),
# which would race pl_mission.py's own steps; set them (0.2 / true) with PL_DRIVER_ARGS=--gs-auto
# for the station to fly them, as on hardware, while the driver watches.
(setsid nohup env $HL VALVE_FEEDBACK="$FB" WB_SIM_PROFILE=valve_turning \
   ARM_GS_AUTO_EXIT_DELAY="${ARM_GS_AUTO_EXIT_DELAY:-0.0}" ARM_GS_AUTO_STEPS="${ARM_GS_AUTO_STEPS:-false}" \
   "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_valve_turning_sitl.sh" --in-terminal shiqi_machine \
   > "$RUNS/$TAG.scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for the valve (Isaac)"
for i in $(seq 90); do
  timeout 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break; sleep 2
done
echo "[$(date +%T)] valve up; letting PX4 / the arm stack settle"
sleep 45
echo "[$(date +%T)] flying"
/usr/bin/python3 "$PUSH_TOOLS/pl_mission.py" --out "$RUNS/$TAG.npz" ${PL_DRIVER_ARGS:-} 2>&1 | tee "$RUNS/$TAG.log"
echo "[$(date +%T)] done"
