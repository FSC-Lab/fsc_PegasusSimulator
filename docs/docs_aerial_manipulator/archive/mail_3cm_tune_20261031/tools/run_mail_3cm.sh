#!/usr/bin/env bash
# One headless mail-delivery flight in Isaac on fsc_lab_machine, LAID OUT AS THE 10-31 HARDWARE 3cm RUN
# (flight_wb_l1_4d_mail_delivery_3cm_20261031_170629, 2026-10-31 17:06): whole-body 4-D L1 on the
# EKF2-FUSED stack (the hardware path), the 07 mail-delivery scene, pnp_mission_v2.py --mail (the arm GS
# Mail Envelope tab's sequence). A fsc_lab_machine port of mail_delivery_20261023/tools/run_mail.sh that
# also RECORDS A ROSBAG OF THE HARDWARE RECORDER'S TOPICS (hw_topics.txt), so the sim flight and the
# hardware bag are scored by the same analysis.
#
#   run_mail_3cm.sh <tag>      (after a clean slate, TWO separate calls:
#                                $FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
#                                $FSC_PEGASUS_ROOT/scripts/kill_stale_sim_processes.sh -y)
#
# THE LAYOUT reproduces the hardware run's PLANNER TARGETS (pick_place/info), not the room:
#   pick capture (-0.290, 0.993, 0.860) yaw -3.5  <- pillar centre - 95 mm along the envelope, hat top + 2.1 mm
#   release claw (0.550, 0.285, 1.070)            <- place point (front face + 9 mm, box top - 48.5 mm) + ee_offset -60 mm
#   start (-1.495, -0.016)                        <- Adjust on the ground at the spawn
# The scene's table top is fixed at 0.80 m, so the hardware box top (46 mm higher) is reached by a taller
# box (0.3185 m) with the mouth raised by the same 46 mm. The mouth is 60 mm high (90 mm design, the 3 cm
# insert); MAIL_INSERT_EDGE says which edge the insert moved: both (centred, default) | bottom | top.
#
# MAIL_YAML=<file>                  controller yaml (default the shipped ..._sim_mail_delivery.yaml);
#                                   make_gain_yaml.py writes the candidates; copied to runs/<tag>.yaml
# MAIL_INSERT_EDGE=both|bottom|top  see above
# MAIL_LAYOUT=3cm|4cm              which hardware run's layout (default 3cm)
# MAIL_REALTIME=1|0                 Isaac's real-time pacer (default 1)
# MAIL_SIM_RTF=<x>                  the wall-clock compensation's RTF (default 1.0 -- right only with the pacer
#                                   ON and holding 1.000; with the pacer off pass the measured RTF)
# MAIL_SCENE_ENV="K=V ..."          any further scene knob (07's PEGASUS_MAIL_* / PEGASUS_PNP_*)
# MAIL_DRIVER_ARGS="..."            extra pnp_mission_v2.py arguments
# MAIL_PROBE_ONLY=1                 stop after bring-up (Isaac banners, probes, RTF lines -> runs/<tag>.isaac.log)
# MAIL_BRINGUP_TRIES=<n>           relaunch after an Isaac crash at bring-up (the intermittent libomniclient
#                                   segfault in SimulationApp's startup), up to n tries in all (default 2)
# MAIL_TOPICS=<file>                the recorded topics (default hw_topics.txt; analysis_topics.txt = only what
#                                   tools/analysis reads, a lighter recorder)
# Writes runs/<tag>.{npz,log,stack.log,scene.log,isaac.log,yaml} and runs/<tag>.bag/; leaves the simulation running.
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
TAG="${1:?tag}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$(cd -- "$HERE/.." && pwd)/runs"; mkdir -p "$RUNS"
source "$HERE/../../../../../scripts/config/fsc_lab_machine.conf"
PEG="$FSC_PEGASUS_ROOT"
STACK="$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh"
YAML="${MAIL_YAML:-$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_mail_delivery.yaml}"
[[ -f "$YAML" ]] || { echo "ERROR: no yaml $YAML" >&2; exit 2; }
cp "$YAML" "$RUNS/$TAG.yaml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEG/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"

# MAIL_LAYOUT picks the hardware run whose planner targets the scene reproduces (3cm default; 4cm =
# flight_wb_l1_4d_mail_delivery_4cm_20261031_163410, mouth 50 mm, box 0.2985 m). The mouth is the 90 mm
# design one reduced by the insert; MAIL_INSERT_EDGE moves its centre by half the reduction.
case "${MAIL_LAYOUT:-3cm}" in
  3cm) MH=0.060; MZ0=0.2485; HALF=0.015
       LAYOUT=(PEGASUS_PNP_SPAWN_XY=-1.495,-0.016
               PEGASUS_MAIL_PILLAR_XY=-0.195,0.987 PEGASUS_MAIL_PILLAR_HEIGHT=0.850 PEGASUS_MAIL_ENVELOPE_YAW_DEG=-3.5
               PEGASUS_MAIL_TABLE_FRONT_X=0.601 PEGASUS_MAIL_RED_Y=0.286 PEGASUS_MAIL_BOX_SIZE=0.235,0.125,0.3185) ;;
  4cm) MH=0.050; MZ0=0.2285; HALF=0.020
       LAYOUT=(PEGASUS_PNP_SPAWN_XY=-1.473,-0.004
               PEGASUS_MAIL_PILLAR_XY=-0.195,0.988 PEGASUS_MAIL_PILLAR_HEIGHT=0.850 PEGASUS_MAIL_ENVELOPE_YAW_DEG=-1.8
               PEGASUS_MAIL_TABLE_FRONT_X=0.621 PEGASUS_MAIL_RED_Y=0.283 PEGASUS_MAIL_BOX_SIZE=0.235,0.125,0.2985) ;;
  *) echo "MAIL_LAYOUT must be 3cm or 4cm" >&2; exit 2 ;;
esac
case "${MAIL_INSERT_EDGE:-both}" in
  both)   MZ=$MZ0 ;;
  bottom) MZ=$(awk -v z=$MZ0 -v h=$HALF 'BEGIN{printf "%.4f", z + h}') ;;
  top)    MZ=$(awk -v z=$MZ0 -v h=$HALF 'BEGIN{printf "%.4f", z - h}') ;;
  *) echo "MAIL_INSERT_EDGE must be both, bottom or top" >&2; exit 2 ;;
esac
LAYOUT+=(PEGASUS_MAIL_MOUTH=0.190,$MH,$MZ)
TIMING=(PEGASUS_REALTIME="${MAIL_REALTIME:-1}" PEGASUS_SIM_RTF="${MAIL_SIM_RTF:-1.0}")

(env WB_SIM_PROFILE=mail_delivery WB_SIM_YAML="$YAML" "${TIMING[@]}" \
   setsid nohup bash "$STACK" fsc_lab_machine uav_0 > "$RUNS/$TAG.stack.log" 2>&1 &)
SCENE=(env PEGASUS_HEADLESS=1 PNP_FEEDBACK=fused WB_SIM_PROFILE=mail_delivery WB_SIM_YAML="$YAML" "${TIMING[@]}"
       "${LAYOUT[@]}" ${MAIL_SCENE_ENV:-}
       "$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh"
       --in-terminal fsc_lab_machine)   # --in-terminal: the launcher's own output lands in <tag>.scene.log,
                                        # not in a konsole window on the desktop

source /opt/ros/humble/setup.bash
source "$FSC_AUTOPILOT_WS/install/setup.bash"
source "$FSC_OM_ARM_WS/install/setup.bash"

# the stacks refuse to start beside a running planner and check with pgrep -f on its NODE name, so
# wait on the planner's pick-and-place status TOPIC, whose name does not contain it (run_mail.sh's note)
sleep 20
echo "[$(date +%T)] waiting for the planner"
for i in $(seq 60); do
  st=$(timeout -k 2 6 ros2 topic echo --no-daemon --once /uav_0/whole_body_planner/pick_place/status 2>/dev/null | grep -m1 '^data:')
  [[ -n "$st" ]] && break; sleep 2
done
echo "[$(date +%T)] planner up: $st"
(setsid nohup "${SCENE[@]}" > "$RUNS/$TAG.scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for the localization frame (Isaac)"
isaac_died() {   # the Isaac pane prints this when SimulationApp dies (the scene wrapper's own message)
  tmux capture-pane -J -p -S - -t px4_isaac:0.0 2>/dev/null | grep -q 'Isaac Sim exited'
}
for i in $(seq 90); do
  timeout -k 2 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break
  if isaac_died; then
    try=${MAIL_BRINGUP_TRY:-1}
    echo "[$(date +%T)] Isaac crashed at bring-up (try $try of ${MAIL_BRINGUP_TRIES:-2})"
    tmux capture-pane -J -p -S - -t px4_isaac:0.0 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' \
      | grep -E 'Fatal|Segmentation' | tail -4 | cut -c1-160
    (( try >= ${MAIL_BRINGUP_TRIES:-2} )) && { echo "giving up"; exit 75; }
    "$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh" > /dev/null 2>&1
    "$FSC_PEGASUS_ROOT/scripts/kill_stale_sim_processes.sh" -y > /dev/null 2>&1
    for p in $(pgrep -f "[k]onsole --noclose -e bash -lc $FSC_PEGASUS_ROOT/scripts/indoor_sim"); do kill "$p" 2>/dev/null; done
    sleep 3
    MAIL_BRINGUP_TRY=$((try + 1)) exec "$0" "$TAG"
  fi
  sleep 2
done
echo "[$(date +%T)] Isaac up; letting PX4 / the arm stack settle"
sleep 45
dump_isaac() {   # the Isaac pane's own output (probes, banners, RTF) lives only in tmux
  for p in $(tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index}' 2>/dev/null); do
    tmux capture-pane -J -p -S - -t "$p" 2>/dev/null
  done | sed 's/\x1b\[[0-9;]*m//g' | grep -E "AM-T650|RTF|Traceback|Error" > "$RUNS/$TAG.isaac.log" || true
}
if [[ "${MAIL_PROBE_ONLY:-0}" == 1 ]]; then
  sleep 30; dump_isaac; echo "[$(date +%T)] probe only: $RUNS/$TAG.isaac.log"; exit 0
fi
mapfile -t TOPICS < <(grep -v '^#' "${MAIL_TOPICS:-$HERE/hw_topics.txt}")
(setsid nohup ros2 bag record -o "$RUNS/$TAG.bag" "${TOPICS[@]}" > "$RUNS/$TAG.bag.log" 2>&1 &)
sleep 3
echo "[$(date +%T)] flying (recording $RUNS/$TAG.bag)"
/usr/bin/python3 "$PEG/docs/docs_aerial_manipulator/archive/pick_place_controllers_20261003/tools/pnp_mission_v2.py" \
  --rig wb --out "$RUNS/$TAG.npz" --mail --grip-via-action --grip-axis-tol 0.008 ${MAIL_DRIVER_ARGS:-} 2>&1 | tee "$RUNS/$TAG.log"
sleep 3
pkill -INT -f "ros2 bag record -o $RUNS/$TAG.bag" ; sleep 3
dump_isaac
echo "[$(date +%T)] done"
