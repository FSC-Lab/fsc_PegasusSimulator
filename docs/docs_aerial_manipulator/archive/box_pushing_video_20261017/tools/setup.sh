#!/bin/bash
# Windowed simulation for a presentation video (after a clean slate, in a SEPARATE
# call: scripts/kill_stale_sim_processes.sh -y).
#   setup.sh VIEW.json        view_circle.json | view_figure8.json | view_box_pushing.json | view_valve_turning.json
#                             | view_pick_and_place.json | view_pick_and_drop.json | view_pick_and_place_assembly.json
# WB_SIM_YAML=<path> (inherited by the stack and the scene launcher) flies a variant of the
# profile's yaml; PEGASUS_PNP_* set in this script's environment reach the 07 scene.
# 1. the 4-D whole-body stack, RAW mocap; WB_SIM_PROFILE = mirror (free flight) or
#    box_pushing or valve_turning; its drone GS is replaced by drone_gs_video.py (the video layout)
# 2. the scene through the 4-D launcher with AM_ISAAC_SCENE_SCRIPT = video_scene.py
#    (06 or 07 per the view file): PX4, the torque-mode arm stack, the arm GS on the
#    scene's tab at QT_SCALE_FACTOR 0.8
# 3. the windows laid out (Isaac left, both ground stations right) and the arm GS's
#    page/log splitter dragged down so the tab page gets the height
# Logs: ../logs/.
# REPLAY_TAKE=<take> (2026-10-18): the RECORDING half of a replayed video -- the scene runs
# HEADLESS as record_scene.py (poses + the exported stage -> ../takes/<take>/), only the two
# ground stations are laid out; replay_scene.py renders the Isaac view afterwards.
VIEW="${1:?view json}"
export DISPLAY="${DISPLAY:-:1}"
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml
V="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
L="$V/../logs"; mkdir -p "$L"
PEG=$HOME/fsc_PegasusSimulator
PY=/usr/bin/python3
GEOM_ISAAC="70 27 1244 1400"
GEOM_GS="1316 27 1244 1400"
cp "$VIEW" "$V/video_view.json"
SCENE=$($PY -c "import json,sys; print(json.load(open(sys.argv[1]))['scene'])" "$V/video_view.json")
case "$SCENE" in
  free_flight) PROFILE=mirror;      TAB=2 ;;   # EE Trajectory
  box_pushing) PROFILE=box_pushing; TAB=6 ;;   # Box Pushing
  valve_turning) PROFILE=valve_turning; TAB=7 ;; # Valve Turning (2026-10-17)
  pick_and_place) PROFILE=pick_and_place; TAB=4; PNP_TASK=place ;; # Pick & Place: the paper's set-down mission (2026-10-18)
  pick_and_drop)  PROFILE=pick_and_drop;  TAB=5; PNP_TASK=drop ;;  # Pick & Drop: the latest payload into the tray (2026-10-18)
  pick_and_place_assembly)                                        # Pick & Place with the LATEST payload, set down on
    PROFILE=pick_and_drop; TAB=4; PNP_TASK=place                  # the place pillar's hat (no tray; 2026-10-18, user)
    export WB_SIM_YAML="${WB_SIM_YAML:-$V/pick_and_place_assembly_video_sim.yaml}"
    export PEGASUS_PNP_PAYLOAD="${PEGASUS_PNP_PAYLOAD:-assembly}" PEGASUS_PNP_BIN="${PEGASUS_PNP_BIN:-0}"
    export PEGASUS_PNP_PICK_PILLAR_HEIGHT="${PEGASUS_PNP_PICK_PILLAR_HEIGHT:-0.77}"
    export PEGASUS_PNP_PLACE_PILLAR_HEIGHT="${PEGASUS_PNP_PLACE_PILLAR_HEIGHT:-0.77}"
    export PEGASUS_PNP_PAYLOAD_MASS="${PEGASUS_PNP_PAYLOAD_MASS:-0.190}" ;;
  *) echo "unknown scene $SCENE"; exit 1 ;;
esac
echo "[$(date +%T)] scene $SCENE, stack profile $PROFILE, arm GS tab $TAB"
SCENE_SCRIPT="$V/video_scene.py"; HL=0
if [[ -n "${REPLAY_TAKE:-}" ]]; then
  REC_DIR="$(cd -- "$V/.." && pwd)/takes/$REPLAY_TAKE"; mkdir -p "$REC_DIR"
  $PY -c "import json,sys; json.dump({'scene': sys.argv[1], 'out_dir': sys.argv[2], 'every_steps': 5}, open(sys.argv[3], 'w'))" \
    "$SCENE" "$REC_DIR" "$V/record_cfg.json"
  cp "$V/video_view.json" "$REC_DIR/view.json"
  SCENE_SCRIPT="$V/record_scene.py"; HL=1
  echo "[$(date +%T)] RECORD mode: headless scene, poses + stage -> $REC_DIR"
fi

(WB_SIM_PROFILE=$PROFILE setsid nohup bash $HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0 > "$L/stack.log" 2>&1 < /dev/null &)
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/rosdeps/local_setup.bash 2>/dev/null || true
source $HOME/ros2_ws/install/setup.bash
# the stack refuses to start beside a running planner and checks with pgrep -f:
# nothing here may name the planner while that check runs
sleep 20
for i in $(seq 60); do
  ros2 param get --no-daemon --spin-time 3 /uav_0/whole_body_trajectory_planner ee_traj_lap_time 2>/dev/null \
    | grep -q -E "values? (is|are)" && break; sleep 2
done
echo "[$(date +%T)] planner up"

# the stack's drone GS -> the video layout
for p in $(ps -eo pid=,args= | awk '/single_drone_ground_control\.py/ && !/awk/ {print $1}'); do kill -9 "$p"; done
sleep 1
(setsid nohup $PY "$V/drone_gs_video.py" $GEOM_GS > "$L/drone_gs.log" 2>&1 < /dev/null &)
echo "[$(date +%T)] drone GS (video layout) started"

# the arm GS is started inside the stack's tmux server, whose environment the panes inherit
tmux setenv -g QT_SCALE_FACTOR 0.8
if [[ -n "${PNP_TASK:-}" ]]; then
  # 07 pick-and-place reads PEGASUS_PNP_* from the tmux server's environment (the
  # wrapper script's job, bypassed here): the demonstration from the scene, plus any
  # PEGASUS_PNP_* set in this script's environment; every other knob at its default
  for v in $(tmux show-environment -g 2>/dev/null | sed -nE 's/^-?(PEGASUS_PNP_[A-Z_]+).*/\1/p'); do tmux setenv -gu "$v"; done
  tmux setenv -g PEGASUS_PNP_TASK "$PNP_TASK"
  for v in $(compgen -e | grep '^PEGASUS_PNP_'); do
    tmux setenv -g "$v" "${!v}"; echo "[$(date +%T)] scene knob $v=${!v}"
  done
fi
(AM_ISAAC_SCENE_SCRIPT="$SCENE_SCRIPT" AM_ISAAC_SCENE_LABEL="AM-T650-WB-L1-4D-VIDEO-$SCENE" \
 PEGASUS_EE_MARKER_CUBE=0 WB_SIM_PROFILE=$PROFILE ARM_GS_START_TAB=$TAB PEGASUS_HEADLESS=$HL \
 setsid nohup $PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh \
   --in-terminal shiqi_machine > "$L/scene.log" 2>&1 < /dev/null &)
for i in $(seq 120); do
  timeout 5 ros2 topic echo --no-daemon --once /uav_0/state/pose > /dev/null 2>&1 && break; sleep 2
done
echo "[$(date +%T)] Isaac up"
for i in $(seq 60); do
  $PY "$V/xwin.py" find "Indoor Robotic Arm" > /dev/null 2>&1 && break; sleep 2
done
tmux setenv -gu QT_SCALE_FACTOR
sleep 5
[[ $HL == 1 ]] || $PY "$V/xwin.py" place "Isaac Sim" $GEOM_ISAAC --undecorate
$PY "$V/xwin.py" place "Drone Ground Station" $GEOM_GS --undecorate
$PY "$V/xwin.py" place "Indoor Robotic Arm" $GEOM_GS --undecorate
sleep 3
# the tab page takes the height, the log keeps a few lines (user, 2026-10-17)
PYTHONNOUSERSITE=1 $PY "$V/split.py" 1290
echo "[$(date +%T)] windows placed; letting PX4 / the arm stack settle"
sleep 25
echo "[$(date +%T)] ready"
