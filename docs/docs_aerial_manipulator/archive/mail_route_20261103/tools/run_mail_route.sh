#!/usr/bin/env bash
# One headless MAIL-ROUTE flight in Isaac on fsc_lab_machine (2026-11-03): three letters on three hats in a
# line, posted alternately into the RED and BLUE mailboxes -- whole-body 4-D L1 on the EKF2-FUSED stack, the
# 08 mail-route scene, mail_route_mission.py (the arm GS Mail Route tab's sequence). A COPY of
# mail_3cm_tune_20261031/tools/run_mail_3cm.sh (not touched), on the scene's own default layout.
#
#   run_mail_route.sh <tag>    (after a clean slate, TWO separate calls:
#                                $FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
#                                $FSC_PEGASUS_ROOT/scripts/kill_stale_sim_processes.sh -y)
#
# ROUTE_YAML=<file>                 flight yaml (default the generated ..._sim_mail_route.yaml)
# ROUTE_REALTIME=1|0                Isaac's real-time pacer (default 1)
# ROUTE_SIM_RTF=<x>                 the wall-clock compensation's RTF (default 1.0)
# ROUTE_SCENE_ENV="K=V ..."         any further scene knob (08's PEGASUS_ROUTE_* / PEGASUS_MAIL_* / PEGASUS_PNP_*)
# ROUTE_DRIVER_ARGS="..."           extra mail_route_mission.py arguments (e.g. --order 1,3,2)
# ROUTE_PROBE_ONLY=1                stop after bring-up (Isaac banners -> runs/<tag>.isaac.log)
# ROUTE_BRINGUP_TRIES=<n>          relaunch after an Isaac crash at bring-up, up to n tries in all (default 2)
# ROUTE_TOPICS=<file>               the recorded topics (default the mail study's hw_topics.txt + the envelopes)
# Writes runs/<tag>.{npz,log,stack.log,scene.log,isaac.log,yaml} and runs/<tag>.bag/; leaves the simulation running.
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
TAG="${1:?tag}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$(cd -- "$HERE/.." && pwd)/runs"; mkdir -p "$RUNS"
source "$HERE/../../../../../scripts/config/fsc_lab_machine.conf"
PEG="$FSC_PEGASUS_ROOT"
STACK="$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh"
YAML="${ROUTE_YAML:-$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_mail_route.yaml}"
[[ -f "$YAML" ]] || { echo "ERROR: no yaml $YAML" >&2; exit 2; }
cp "$YAML" "$RUNS/$TAG.yaml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
TIMING=(PEGASUS_REALTIME="${ROUTE_REALTIME:-1}" PEGASUS_SIM_RTF="${ROUTE_SIM_RTF:-1.0}")

(env WB_SIM_PROFILE=mail_delivery WB_SIM_YAML="$YAML" "${TIMING[@]}" \
   setsid nohup bash "$STACK" fsc_lab_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
SCENE=(env PEGASUS_HEADLESS=1 PNP_FEEDBACK=fused WB_SIM_PROFILE=mail_delivery WB_SIM_YAML="$YAML" "${TIMING[@]}"
       ${ROUTE_SCENE_ENV:-}
       "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_mail_route_sitl.sh"
       --in-terminal fsc_lab_machine)

source /opt/ros/humble/setup.bash
source "$FSC_AUTOPILOT_WS/install/setup.bash"
source "$FSC_OM_ARM_WS/install/setup.bash"

# wait on the planner's pick-and-place status TOPIC (the stacks' pgrep -f guards match the node name)
sleep 20
echo "[$(date +%T)] waiting for the planner"
for i in $(seq 60); do
  st=$(timeout -k 2 6 ros2 topic echo --no-daemon --once /uav_0/whole_body_planner/pick_place/status 2>/dev/null | grep -m1 '^data:')
  [[ -n "$st" ]] && break; sleep 2
done
echo "[$(date +%T)] planner up: $st"
(setsid nohup "${SCENE[@]}" > "$RUNS/$TAG.scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for the localization frame (Isaac)"
isaac_died() {
  tmux capture-pane -J -p -S - -t px4_isaac:0.0 2>/dev/null | grep -q 'Isaac Sim exited'
}
for i in $(seq 90); do
  timeout -k 2 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break
  if isaac_died; then
    try=${ROUTE_BRINGUP_TRY:-1}
    echo "[$(date +%T)] Isaac crashed at bring-up (try $try of ${ROUTE_BRINGUP_TRIES:-2})"
    tmux capture-pane -J -p -S - -t px4_isaac:0.0 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' \
      | grep -E 'Fatal|Segmentation|Traceback|Error' | tail -6 | cut -c1-200
    (( try >= ${ROUTE_BRINGUP_TRIES:-2} )) && { echo "giving up"; exit 75; }
    "$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh" > /dev/null 2>&1
    "$FSC_PEGASUS_ROOT/scripts/kill_stale_sim_processes.sh" -y > /dev/null 2>&1
    for p in $(pgrep -f "[k]onsole --noclose -e bash -lc $FSC_PEGASUS_ROOT/scripts/indoor_sim"); do kill "$p" 2>/dev/null; done
    sleep 3
    ROUTE_BRINGUP_TRY=$((try + 1)) exec "$0" "$TAG"
  fi
  sleep 2
done
echo "[$(date +%T)] Isaac up; letting PX4 / the arm stack settle"
sleep 45
dump_isaac() {   # the Isaac pane's own output (banners, envelope monitor, RTF) lives only in tmux
  for p in $(tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index}' 2>/dev/null); do
    tmux capture-pane -J -p -S - -t "$p" 2>/dev/null
  done | sed 's/\x1b\[[0-9;]*m//g' | grep -E "AM-T650|RTF|Traceback|Error" > "$RUNS/$TAG.isaac.log" || true
}
if [[ "${ROUTE_PROBE_ONLY:-0}" == 1 ]]; then
  sleep 30; dump_isaac; echo "[$(date +%T)] probe only: $RUNS/$TAG.isaac.log"; exit 0
fi
mapfile -t TOPICS < <(grep -v '^#' "${ROUTE_TOPICS:-$PEG/docs/docs_aerial_manipulator/archive/mail_3cm_tune_20261031/tools/hw_topics.txt}")
TOPICS+=(/payload_0/state/pose /payload_1/state/pose /payload_2/state/pose /claw_0/state/pose
         /uav_0/whole_body_planner/pick_place/status /uav_0/whole_body_planner/pick_place/info
         /uav_0/whole_body_planner/pick_place/route/drop_now)
(setsid nohup ros2 bag record -o "$RUNS/$TAG.bag" "${TOPICS[@]}" > "$RUNS/$TAG.bag.log" 2>&1 &)
sleep 3
echo "[$(date +%T)] flying (recording $RUNS/$TAG.bag)"
/usr/bin/python3 "$HERE/mail_route_mission.py" --out "$RUNS/$TAG.npz" ${ROUTE_DRIVER_ARGS:-} 2>&1 | tee "$RUNS/$TAG.log"
sleep 3
pkill -INT -f "ros2 bag record -o $RUNS/$TAG.bag" ; sleep 3
dump_isaac
echo "[$(date +%T)] done"
