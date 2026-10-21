#!/usr/bin/env bash
# One AUTONOMOUS box-pushing flight in Isaac, headless, on the COORDINATED-
# ADMITTANCE rig (Liu et al. CEP 2024): the twin of push_pull_20261003/tools/run_pl.sh
# with the stack, the scene wrapper and the driver's controller namespace swapped.
#
#   run_pl_cadm.sh <tag>      (after a clean slate in a SEPARATE call:
#                              scripts/kill_stale_sim_processes.sh -y)
#
# PL_DRIVER_ARGS="..." goes to pl_mission.py (--timetable etc.); PL_RUNS_DIR
# overrides the output directory. Writes <runs>/<tag>.{npz,log,stack.log,scene.log}
# and copies the yaml the run flew beside them. CLOSES THE SIMULATION when the
# driver exits (2026-10-10: a stack left up over a break wrote 27 GB of PX4 ulog
# and a 1.2 GB Isaac kit log in 4 h); PL_KEEP_SIM=1 leaves it running.
set -o pipefail
TAG="${1:?tag}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="${PL_RUNS_DIR:-$HERE/../runs}"; mkdir -p "$RUNS"
PEG="$HOME/fsc_PegasusSimulator"
PL="$PEG/docs/docs_aerial_manipulator/archive/push_pull_20261003/tools"
STACKS="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim"
STACK="$STACKS/start_coordinated_admittance_direct_actuation_t650_aerial_manipulator_stack.sh"
YAML="$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_coordinated_admittance_direct_actuation_t650_sim_box_pushing.yaml"
DA="fsc_autopilot_ros2/coordinated_admittance_direct_actuation"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
cp "$YAML" "$RUNS/$TAG.yaml"
# The box and table the run flies, passed to the scene explicitly and recorded beside the npz like
# box_pushing_campaign_20261016/tools/fly.sh's .args (2026-10-17: the real box -- 500 g, mu 0.25 / 0.25,
# its back face 0.10 m from the table's +y end, centre y 0.28). Before 2026-10-17 this script left
# them to the scene's defaults and wrote no .args.
MASS=${MASS:-0.500}; MU_S=${MU_S:-0.25}; MU_K=${MU_K:-0.25}; BOX_Y0=${BOX_Y0:-0.28}
DIST=$(sed -nE 's/^    push_pull_push_distance:\s*([0-9.]+).*/\1/p' "$YAML" | head -1)
REL=0; [[ " ${PL_DRIVER_ARGS:-} " == *" --release-at-open "* ]] && REL=1
{ echo "MASS=$MASS"; echo "DIST=$DIST"; echo "MU_S=$MU_S"; echo "MU_K=$MU_K"; echo "BOX_Y0=$BOX_Y0"; echo "RELEASE=$REL"; } > "$RUNS/$TAG.args"

(WB_SIM_PROFILE=box_pushing setsid nohup bash "$STACK" shiqi_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"

sleep 20
echo "[$(date +%T)] waiting for the planner's push-and-pull block"
for i in $(seq 60); do
  off=$(ros2 param get --no-daemon --spin-time 3 /uav_0/whole_body_trajectory_planner push_pull_ee_offset 2>/dev/null \
        | sed -nE 's/.*values? (is|are): *(.*)$/\2/p')
  [[ -n "$off" ]] && break; sleep 2
done
[[ -n "$off" ]] || { echo "the planner has no push_pull_ee_offset -- old binary? rebuild fsc_trajectory_planner"; exit 1; }
echo "[$(date +%T)] planner up, EE offset $off"
# ARM_GS_AUTO_EXIT_DELAY=0.0: pl_mission.py owns release / open / exit -- the arm GS's
# automatic exit (2026-10-19) would race it
(setsid nohup env PEGASUS_HEADLESS=1 WB_SIM_PROFILE=box_pushing ARM_GS_AUTO_EXIT_DELAY=0.0 PEGASUS_PUSH_BOX_MASS=$MASS \
   PEGASUS_PUSH_FRICTION_STATIC=$MU_S PEGASUS_PUSH_FRICTION_DYNAMIC=$MU_K \
   PEGASUS_PUSH_BOX_XY="${PEGASUS_PUSH_BOX_XY:-0.675,$BOX_Y0}" \
   "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_coordinated_admittance_box_pushing_sitl.sh" --in-terminal shiqi_machine \
   > "$RUNS/$TAG.scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for the box (Isaac)"
for i in $(seq 90); do
  timeout 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break; sleep 2
done
echo "[$(date +%T)] box up; letting PX4 / the arm stack settle"
sleep 45
echo "[$(date +%T)] flying"
/usr/bin/python3 "$PL/pl_mission.py" --da "$DA" --dbg-topic coordinated_control_debug --dbg-len 56 \
  --hover-dwell "${PL_HOVER_DWELL:-15.0}" --grip-dwell-in "${PL_GRIP_DWELL_IN:-0.3}" --grip-tol "${PL_GRIP_TOL:-0.025}" --grip-axis-tol "${PL_GRIP_AXIS_TOL:-0.004}" ${PL_GRIP_LIVE_BOX:---grip-live-box} --out "$RUNS/$TAG.npz" ${PL_DRIVER_ARGS:-} 2>&1 | tee "$RUNS/$TAG.log"
echo "[$(date +%T)] done"
if [[ "${PL_KEEP_SIM:-0}" != "1" ]]; then
  echo "[$(date +%T)] closing the simulation (PL_KEEP_SIM=1 to keep it)"
  "$STACKS/stop_isaacsim_stack.sh" > /dev/null 2>&1 || true
  "$PEG/scripts/kill_stale_sim_processes.sh" -y > /dev/null 2>&1 || true
  # this run's PX4 ulog (the record is the driver's npz) and Isaac kit log
  find "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs_fsc_indoor_am_t650/log" -mindepth 1 -mmin -60 -delete 2>/dev/null || true
  find "$HOME/isaacsim/kit/logs/Kit/Isaac-Sim Python" -type f -name "kit_*.log" -mmin -60 -size +20M -delete 2>/dev/null || true
  echo "[$(date +%T)] simulation closed"
fi
