#!/usr/bin/env bash
#
# start_vtol_fsc_hil.sh — the VTOL under the EXISTING FSC autopilot framework
#                         (fsc_autopilot_ros2 + fsc_drone_state_estimator_ros2),
#                         flown as a MULTICOPTER (no transition).
#
# This is start_vtol_hil.sh with the direct_actuator *test* node replaced by the
# real FSC flight-control stack. fsc_autopilot_ros2 is a PX4 OFFBOARD controller:
# it consumes a state estimate + a position reference, runs its own attitude loop
# ("apm") and publishes px4_msgs/OffboardControlMode + VehicleRatesSetpoint to PX4
# over uXRCE-DDS. PX4 then closes the RATES and allocates through ITS control
# allocation (CA) on the 4101 tiltrotor airframe: the tilt servos make yaw by
# differential tilt about upright, PX4 never transitions.
#
# ★ 2026-10-28 — THE AIRFRAME FILE DRIVES THIS PATH TOO (FSC_AIRFRAME_ID, default the
# package's airframe_id.cfg). Until today it flew the 4101 literals (CAD arms, KM 0.12)
# and a YAML fitted to an 18 N rotor -- on the current CAD build that YAML asks for ~2x
# the weight -- and since the 2026-09-07 sim re-anchor PX4's upright tilt (-1) landed on
# the nacelles' aft stop. Now:
#   * scripts/fsc_autopilot_config.py derives, from the file + the tank env, PX4's CA set
#     (hubs about the wet CG, per-pair CT, KM = spin x cq, THR_MDL_FAC 1, MPC_THR_HOVER),
#     the autopilot's YAML (the wet mass; thrust_scaling from a replica of PX4's own
#     normalized allocation) and VTOL_TILT_PX4CA (the sim maps PX4's tilt convention onto
#     its own -- vtol_utils.px4ca_tilt_map). Written to $FSC_CFG_DIR.
#   * window 8-params applies the PX4 set on the console once PX4 is up, reads every value
#     back (`param show`) and writes $FSC_CFG_DIR/px4_params.verified only if all match;
#     scripts/fly_fsc_cpp.py refuses to arm without it.
#   * PX4 runs in ITS OWN working dir (rootfs_fsc0): SITL persists parameters, and
#     THR_MDL_FAC 1 would otherwise reach the direct-actuator launchers' motors.
#   * Isaac gets FSC_AIRFRAME_ID + the tank + VTOL_TILT_PX4CA; window 9-bag records -a to
#     $BAG_DIR; GS=0 skips the ground station (a scripted flight).
#
# Node graph (all under /uav_0):
#   0 Micro XRCE-DDS Agent   1 PX4 SITL (4101, rootfs_fsc0)   2 Isaac (03_vtol_px4_hil.py)
#   3 mocap emulator         4 state estimator (indoor)       5 fsc_autopilot
#   6 ground station (GS=1)  7 arm/offboard helper            8 PX4 params   9 bag
#
# Fly it: scripts/fly_fsc_cpp.py (scripted: verify -> reference -> OFFBOARD -> ARM -> legs ->
# land -> disarm), or by hand in window 7: fsc_ref 0 0 1.5; fsc_check; fsc_offboard; fsc_arm.
#
# Usage: [FSC_AIRFRAME_ID=<cfg> VTOL_SLOSH=1 VTOL_TANK=slosh VTOL_TANK_KG=.. ...] \
#        scripts/start_vtol_fsc_hil.sh [--here] <machine-config>       # e.g. pieper_machine
# Each airframe's gains for this path live in its file's [tuning] (fsc_<yaml key>): the CAD airframe's
# (airframe_id.cfg) are yaw 1.5 + posctl k_pos_x/y 0.7, the plan's yaw 1.5 (2026-10-28, CLAUDE.md).
# FSC_YAML_SET="k=v,.." is for an experiment on top of them, recorded in the YAML header and LAUNCH_LOG.

# ============================================================================
#  >>> tunables (override via env) <<<
# ----------------------------------------------------------------------------
PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_quad_tiltrotor}"       # PX4 4101 tiltrotor airframe
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"             # "auto" or numeric ID (4101)
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
UAV_PREFIX="${UAV_PREFIX:-uav_0}"                          # ns for PX4 DDS + all fsc nodes
GS_DIR="${GS_DIR:-$ROS2_WS/src/ros2_ground_station_gui}"   # ground station GUI checkout
GS_MAIN="${GS_MAIN:-single_drone_ground_control.py}"       # single-drone GUI (run from GS_DIR/src)
GAZEBO_WS="${GAZEBO_WS:-$HOME/workspaces/gazebo}"          # overlay ws w/ fsc_autopilot+estimator+msgs
GS="${GS:-1}"                                              # 0 = no ground station (scripted flight)
BAG_DIR="${BAG_DIR:-$HOME/flight_bags/fsc_cpp}"
CTRL_PKG="${CTRL_PKG:-$HOME/envs/fsc_autopilot_ros2-master/px4_vtol_direct_actuator/px4_vtol_direct_actuator}"
AF_FILE="${FSC_AIRFRAME_ID:-$CTRL_PKG/airframe_id.cfg}"
FSC_CFG_DIR="${FSC_CFG_DIR:-$BAG_DIR/config_$(basename "$AF_FILE" .cfg)}"

PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-1}"   # ON here: fly_fsc_cpp.py reads the mapped tilt channels from it before the climb
VTOL_ROS2_STATE=1                                          # REQUIRED: Isaac must publish state for mocap
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

# --- new-terminal handling (default: pop a new terminal; --here to stay) -----
NEW_TERM=1; ARGS=()
for a in "$@"; do
  case "$a" in
    --here|--current-terminal|--in-terminal) NEW_TERM=0 ;;
    --new-terminal) NEW_TERM=1 ;;
    *) ARGS+=("$a") ;;
  esac
done
set -- "${ARGS[@]}"
[[ $# -ne 1 ]] && { echo "ERROR: must provide config name."; cfg_usage "$0"; exit 2; }
CFG_NAME="$1"
if [[ $NEW_TERM -eq 1 ]]; then open_new_terminal "$0" --in-terminal "$CFG_NAME"; exit 0; fi

load_machine_config "$0" "$CFG_NAME"   # -> PX4_DIR, ISAAC_PY, FSC_PEGASUS_ROOT

# --- resolve the Pegasus entrypoint + Isaac python --------------------------
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2; exit 1; }
ISAAC_ROOT="$(cd -- "$(dirname -- "$ISAAC_PY")" && pwd)"
ISAAC_BUNDLED_PY="${ISAAC_ROOT}/kit/python/bin/python3"
[[ -x "$ISAAC_BUNDLED_PY" ]] || { echo "ERROR: Isaac bundled Python not found: $ISAAC_BUNDLED_PY" >&2; exit 1; }
BRIDGE_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

# --- the configuration, DERIVED from the airframe file + the tank --------------
[[ -f "$AF_FILE" ]] || { echo "ERROR: airframe file not found: $AF_FILE" >&2; exit 1; }
# water aboard exactly as scene 03 decides it: VTOL_SLOSH on and VTOL_TANK not "0" (its default "1")
case "${VTOL_SLOSH:-0}" in 0|""|false|False) TANK_KG=0 ;;
  *) if [ "${VTOL_TANK:-1}" = 0 ]; then TANK_KG=0; else TANK_KG="${VTOL_TANK_KG:-0.502}"; fi ;; esac
rm -f "$FSC_CFG_DIR/px4_params.verified"
VTOL_TANK_KG="$TANK_KG" "$HOME/envs/isaac_env/bin/python3" "$SCRIPT_DIR/fsc_autopilot_config.py" \
    --airframe "$AF_FILE" --out "$FSC_CFG_DIR" || { echo "ERROR: fsc_autopilot_config.py failed" >&2; exit 1; }
# shellcheck source=/dev/null
source "$FSC_CFG_DIR/env.sh"                                       # -> VTOL_TILT_PX4CA
AUTOPILOT_PARAMS="${AUTOPILOT_PARAMS:-$FSC_CFG_DIR/autopilot_params.yaml}"
[[ -f "$AUTOPILOT_PARAMS" ]] || { echo "ERROR: autopilot params not found: $AUTOPILOT_PARAMS" >&2; exit 1; }

# --- locate the prebuilt PX4 binary + rootfs --------------------------------
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$PX4_BIN" ]] || { echo "ERROR: PX4 SITL binary not built: $PX4_BIN (run: cd $PX4_DIR && make px4_sitl)" >&2; exit 1; }
if   [[ -d "$PX4_DIR/build/px4_sitl_default/rootfs" ]];     then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/rootfs"
elif [[ -d "$PX4_DIR/build/px4_sitl_default/tmp/rootfs" ]]; then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/tmp/rootfs"
else echo "ERROR: PX4 SITL rootfs not found (build px4_sitl_default)." >&2; exit 1; fi
# ITS OWN working dir: SITL persists parameters (parameters.bson), and this path's THR_MDL_FAC 1 /
# CA geometry must never reach the direct-actuator launchers (rootfs_vtol0), nor theirs this one.
PX4_WORKDIR="$PX4_DIR/build/px4_sitl_default/rootfs_fsc0"
[[ -d "$PX4_WORKDIR" ]] || cp -a "$ROOTFS_BASE" "$PX4_WORKDIR"

# --- resolve the airframe autostart ID from the model name, if "auto" -------
AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
if [[ "$PX4_SYS_AUTOSTART" == "auto" ]]; then
  match="$(ls "$AF_DIR" 2>/dev/null | grep -E "_${PX4_SIM_MODEL}\$" | head -n1 || true)"
  [[ -z "$match" ]] && match="$(ls "$AF_DIR" 2>/dev/null | grep -- "$PX4_SIM_MODEL" | head -n1 || true)"
  [[ -n "$match" ]] || { echo "ERROR: could not resolve autostart ID for '$PX4_SIM_MODEL'." >&2; exit 1; }
  PX4_SYS_AUTOSTART="${match%%_*}"
  echo "[fsc-hil] auto-resolved PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART  (airframe: $match)"
fi

# --- sanity notes (non-fatal) -----------------------------------------------
command -v MicroXRCEAgent >/dev/null 2>&1 || echo "[fsc-hil] WARN: MicroXRCEAgent not on PATH — DDS pane will error."
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || echo "[fsc-hil] WARN: /opt/ros/${ROS_DISTRO}/setup.bash missing."
[[ -f "${ROS2_WS}/install/setup.bash" ]] || echo "[fsc-hil] WARN: ${ROS2_WS}/install/setup.bash missing — build the workspace."
# fsc_autopilot / estimator / msgs live in the gazebo overlay ws; the isaac mocap
# emulator lives in ~/ros2_ws. Accept the package in EITHER install prefix.
for pkg in fsc_autopilot_ros2 fsc_drone_state_estimator_ros2 isaacsim_optitrack_ros2_emulator fsc_autopilot_ros2_msgs; do
  [[ -d "$ROS2_WS/install/$pkg" || -d "$GAZEBO_WS/install/$pkg" ]] || echo "[fsc-hil] WARN: $pkg not built in $ROS2_WS/install or $GAZEBO_WS/install."
done
[[ "$GS" == 1 && ! -f "$GS_DIR/src/$GS_MAIN" ]] && echo "[fsc-hil] WARN: ground station not found: $GS_DIR/src/$GS_MAIN (window 6 will error)."

# ================================
SESSION="vtol_fsc_hil"
PX4_INSTANCE=0
D_PX4=2; D_ISAAC=4; D_EMU=14; D_EST=16; D_AP=18; D_GS=20; D_HELP=8   # staged startup delays [s]
HELPERS="$FSC_PEGASUS_ROOT/scripts/fsc_offboard_helpers.sh"
mkdir -p "$BAG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"

echo "[fsc-hil] model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART ns=$UAV_PREFIX scene=$PEGASUS_SCRIPT_REL"
echo "[fsc-hil] airframe=$AF_FILE  slosh=${VTOL_SLOSH:-0} tank=${VTOL_TANK:-1} water=$TANK_KG kg  config=$FSC_CFG_DIR  VTOL_TILT_PX4CA=$VTOL_TILT_PX4CA"
echo "[fsc-hil] PX4 workdir=$PX4_WORKDIR (its own)  bag=$BAG_DIR/fsc_$STAMP"
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# Shared prelude for the ROS 2 panes: system ROS + BOTH overlay workspaces
# (~/ros2_ws for the isaac mocap emulator; the gazebo ws for fsc_autopilot /
# estimator / msgs), matching ~/.bashrc's source order. Prepend /usr/bin so
# `python3` = system python (the isaac_env venv on PATH has NO PyQt5/numpy, which
# is what silently kills the ground station GUI); clear venv/conda vars too.
ROS_PRELUDE="unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_PRELOAD;
export PATH=\"/usr/bin:\$PATH\";
export ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0};
source /opt/ros/$ROS_DISTRO/setup.bash;
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\";
[ -f \"$GAZEBO_WS/install/setup.bash\" ] && source \"$GAZEBO_WS/install/setup.bash\";"

# Each component gets its OWN full-size tmux WINDOW (not a split pane) — switch
# with Ctrl-b <number> (0..9) or Ctrl-b w for the list.
# ---- Window 0: Micro XRCE-DDS Agent (up before PX4's DDS client) ------------
tmux new-session -d -s "$SESSION" -n "0-dds" "
echo '[pane0] Micro XRCE-DDS Agent  udp4 -p $DDS_AGENT_PORT'
if command -v MicroXRCEAgent >/dev/null 2>&1; then MicroXRCEAgent udp4 -p $DDS_AGENT_PORT
else echo 'ERROR: MicroXRCEAgent not on PATH. Run:  MicroXRCEAgent udp4 -p $DDS_AGENT_PORT'; fi
exec bash
"

# ---- Window 1: PX4 SITL (4101 tiltrotor, HIL — Pegasus is the simulator) ----
tmux new-window -t "$SESSION" -n "1-px4" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo '[pane1] waiting $D_PX4 s for DDS agent...'; sleep $D_PX4
echo '[pane1] PX4 SITL model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART ns=$UAV_PREFIX (TCP $((4560 + PX4_INSTANCE))) workdir=$PX4_WORKDIR'
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$UAV_PREFIX \
  \"$PX4_BIN\" -i $PX4_INSTANCE -w \"$PX4_WORKDIR\"
echo '[pane1] PX4 SITL exited.'; exec bash
"

# ---- Window 2: Isaac Sim + Pegasus (VTOL HIL scene, publishes /uav_0/state/*)
# the airframe file, the tank and the PX4 tilt convention travel INLINE: what Isaac builds is what
# the config above was derived from
tmux new-window -t "$SESSION" -n "2-isaac" "
echo '[pane2] waiting $D_ISAAC s for PX4...'; sleep $D_ISAAC
echo '[pane2] Isaac Sim (VTOL HIL scene, state->mocap ON)...'
unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:$BRIDGE_LIB\"
PYTHONUNBUFFERED=1 PEGASUS_HIL_DEBUG=$PEGASUS_HIL_DEBUG VTOL_ROS2_STATE=$VTOL_ROS2_STATE FSC_AIRFRAME_ID='${FSC_AIRFRAME_ID:-}' \
  VTOL_TILT_PX4CA='$VTOL_TILT_PX4CA' VTOL_SLOSH='${VTOL_SLOSH:-0}' VTOL_TANK='${VTOL_TANK:-1}' \
  VTOL_TANK_KG='${VTOL_TANK_KG:-}' VTOL_TANK_R='${VTOL_TANK_R:-}' VTOL_TANK_F1='${VTOL_TANK_F1:-}' \
  VTOL_TANK_ZETA='${VTOL_TANK_ZETA:-}' VTOL_TANK_DX='${VTOL_TANK_DX:-0}' VTOL_TANK_DZ='${VTOL_TANK_DZ:-0}' \
  PYTHONEXE=\"$ISAAC_BUNDLED_PY\" \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\" 2>&1 | tee \"$BAG_DIR/isaac_$STAMP.log\"
echo '[pane2] Isaac Sim exited.'; exec bash
"

# ---- Window 3: mocap emulator  (/uav_0/state/* -> /uav_0/mocap) -------------
tmux new-window -t "$SESSION" -n "3-mocap" "
$ROS_PRELUDE
echo '[pane3] waiting $D_EMU s for Isaac state topics...'; sleep $D_EMU
echo '[pane3] mocap emulator  bodies=$UAV_PREFIX'
ros2 launch isaacsim_optitrack_ros2_emulator emulator_launch.py bodies:=$UAV_PREFIX
echo '[pane3] emulator exited.'; exec bash
"

# ---- Window 4: state estimator (indoor): mocap -> odom + EKF vision_pose ----
tmux new-window -t "$SESSION" -n "4-estimator" "
$ROS_PRELUDE
echo '[pane4] waiting $D_EST s for mocap...'; sleep $D_EST
echo '[pane4] indoor state estimator  uav_prefix=$UAV_PREFIX'
ros2 launch fsc_drone_state_estimator_ros2 indoor_estimator_launch.py uav_prefix:=$UAV_PREFIX
echo '[pane4] estimator exited.'; exec bash
"

# ---- Window 5: fsc_autopilot (offboard attitude controller) ----------------
tmux new-window -t "$SESSION" -n "5-autopilot" "
$ROS_PRELUDE
echo '[pane5] waiting $D_AP s for estimator + PX4 topics...'; sleep $D_AP
echo '[pane5] fsc_autopilot  ns=/$UAV_PREFIX  params=$AUTOPILOT_PARAMS'
ros2 run fsc_autopilot_ros2 autopilot_sv_baseline_node \
    --ros-args -r __ns:=/$UAV_PREFIX -r __node:=autopilot_sv_baseline_node --params-file \"$AUTOPILOT_PARAMS\"
echo '[pane5] autopilot exited.'; exec bash
"

# ---- Window 6: ground station GUI (reference + arm/offboard commands) -------
if [ "$GS" = 1 ]; then
tmux new-window -t "$SESSION" -n "6-groundstn" "
$ROS_PRELUDE
export DISPLAY=\"\${DISPLAY:-:0}\"
echo '[pane6] waiting $D_GS s for the stack...'; sleep $D_GS
if [ -f \"$GS_DIR/src/$GS_MAIN\" ]; then
  cd \"$GS_DIR\"                       # run from REPO ROOT (GUI reads src/ROS_Node/geofence.json relatively)
  echo '[pane6] ground station GUI ($GS_MAIN) — arm + Offboard + set a position setpoint'
  FSC_AIRFRAME_ID='${FSC_AIRFRAME_ID:-}' VTOL_TANK_KG='${VTOL_TANK_KG:-}' /usr/bin/python3 \"src/$GS_MAIN\"
else
  echo '[pane6] ground station not found at $GS_DIR/src/$GS_MAIN.'
fi
exec bash
"
fi

# ---- Window 7: arm/offboard helper (DDS commands — no pxh console needed) ---
tmux new-window -t "$SESSION" -n "7-helper" "
export FSC_NS=$UAV_PREFIX ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
echo '[win7] arm/offboard helper — loads fsc_* commands (type fsc_help). Fly from here.'
if [ -f \"$HELPERS\" ]; then exec bash --init-file \"$HELPERS\"
else echo '[win7] helper not found: $HELPERS'; exec bash; fi
"

# ---- Window 8: PX4 parameters -- the derived CA set + the HIL relaxations, then READ BACK ----
# (COM_ARM_EKF_HGT/VEL/POS/YAW, which the other launchers still set, do not exist in PX4 v1.16: the
#  read-back's first run refused them, 2026-10-28 -- `param set` errors and nothing is relaxed.)
tmux new-window -t "$SESSION" -n "8-params" "
echo '[8-params] waiting for the PX4 console (pxh>)...'
for i in \$(seq 1 200); do
  tmux capture-pane -p -S -80 -t '$SESSION:1-px4' 2>/dev/null | grep -q 'pxh>' && break; sleep 2
done
sleep 3
{ grep -v '^#' '$FSC_CFG_DIR/px4_params.txt'
  printf '%s\n' 'SIM_BAT_MIN_PCT 100' 'COM_LOW_BAT_ACT 0' 'BAT_LOW_THR 0.05' 'COM_DISARM_LAND 0' 'COM_DISARM_PRFLT 0' \
                'COM_ARM_IMU_ACC 1.0' 'COM_ARM_IMU_GYR 1.0' 'COM_ARM_MAG_ANG -1'; } > '$FSC_CFG_DIR/px4_params.applied'
while read -r name val; do
  [ -n \"\$name\" ] || continue
  tmux send-keys -t '$SESSION:1-px4' \"param set \$name \$val\" Enter; sleep 0.3
done < '$FSC_CFG_DIR/px4_params.applied'
sleep 1
# ★ 2026-10-28: on the CAD airframe EKF2 initialised while the quad was still settling from its spawn height
# and never recovered -- 'vertical velocity unstable' + 'Attitude failure (roll)' from boot, z 7 m, 60 height
# resets, arming denied -- with the aircraft level and still. A restart on the settled aircraft clears it.
tmux send-keys -t '$SESSION:1-px4' 'ekf2 stop' Enter; sleep 1
tmux send-keys -t '$SESSION:1-px4' 'ekf2 start' Enter; sleep 5
tmux send-keys -t '$SESSION:1-px4' 'clear' Enter; sleep 0.5
for name in \$(cut -d' ' -f1 '$FSC_CFG_DIR/px4_params.applied'); do
  tmux send-keys -t '$SESSION:1-px4' \"param show \$name\" Enter; sleep 0.25
done
sleep 2
tmux capture-pane -p -J -S -2000 -t '$SESSION:1-px4' > '$FSC_CFG_DIR/px4_console.txt'
\"$HOME/envs/isaac_env/bin/python3\" \"$SCRIPT_DIR/fsc_autopilot_config.py\" --verify '$FSC_CFG_DIR' \
  && touch '$FSC_CFG_DIR/px4_params.verified'
# PX4's own preflight, polled until it passes (the restarted EKF2 needs a few seconds of sim time); the last
# verdict and its reasons go to px4_preflight.txt -- PX4 refuses to arm on a FAILED one either way.
for i in \$(seq 1 12); do
  tmux send-keys -t '$SESSION:1-px4' 'commander check' Enter; sleep 5
  v=\$(tmux capture-pane -p -J -S -40 -t '$SESSION:1-px4' | grep -E 'Preflight check:|Preflight Fail' | tail -4)
  echo \"\$v\" | grep 'Preflight check:' | tail -1 | grep -q OK && break
done
echo \"\$v\" > '$FSC_CFG_DIR/px4_preflight.txt'; echo \"[8-params] PX4 preflight: \$v\"
exec bash
"

# ---- Window 9: the bag (all topics; kill_hil.sh SIGINTs the recorder) ----
# ★ 2026-10-28: the CAD flight's recorder died mid-session without finalising, and rosbag2's default 100 MB write
# cache held the whole flight in memory -- the db3 ended four minutes before teardown, and the window's output
# (the only trace of why) died with tmux. A 10 MB cache keeps a death to ~25 s of loss, and the recorder's
# output and exit code go to bag_<stamp>.log beside the bag.
tmux new-window -t "$SESSION" -n "9-bag" "
$ROS_PRELUDE
sleep $D_AP
echo '[9-bag] ros2 bag record -a -> $BAG_DIR/fsc_$STAMP'
ros2 bag record -a --max-cache-size 10485760 -o \"$BAG_DIR/fsc_$STAMP\" 2>&1 | tee \"$BAG_DIR/bag_$STAMP.log\"
echo \"[9-bag] recorder exited rc=\${PIPESTATUS[0]} at \$(date +%T)\" | tee -a \"$BAG_DIR/bag_$STAMP.log\"
exec bash
"
echo "fsc_$STAMP af=$AF_FILE cfg=$FSC_CFG_DIR slosh=${VTOL_SLOSH:-0} tank=${VTOL_TANK:-1} water=$TANK_KG tilt_px4ca=$VTOL_TILT_PX4CA yaml_set=${FSC_YAML_SET:-}" >> "$BAG_DIR/LAUNCH_LOG.txt"

tmux select-window -t "$SESSION:7-helper"   # land on the helper
if [ -t 1 ] && [ "${ATTACH:-1}" = 1 ]; then tmux attach-session -t "$SESSION"; fi
