#!/usr/bin/env bash

# The flight controller moved to fsc_autopilot_ros2. Resolve it through the
# workspace symlink so a future move of that repo needs no change here.
: "${FSC_CONTROLLER_DIR:=$HOME/ros2_ws/src/px4_vtol_direct_actuator}"
#
# start_vtol_smc_hil.sh — VTOL HIL with a sliding-mode OUTER loop + switchable INNER loop.
#
#   THE SPLIT (2026-09-06 streamline). Two jobs, two kinds of controller:
#
#     SAFE HOVER  (geometric_hover_node.py, window 6, ACTIVE at start)
#       Goes to the x,y,z the operator types and holds it. A pure MULTICOPTER -- rotors
#       vertical at all times, no tilt schedule, no aero model, no trajectory clock. It
#       reads the RAW reference and generates its own velocity-limited motion profile, so
#       it needs no outer loop. This is where STOP always sends the vehicle, so its
#       behaviour is deliberately boring. Closed-loop tested against the simulator's own
#       equations: tests/test_geometric_hover.py.
#
#     TRAJECTORY / PATH / TRANSITION  (the NMPC nodes, windows 6b..7, STANDBY)
#       Trajectory tracking, path following and the hover<->cruise transition. Several are
#       SMC-fed (window 5) because they were tuned around that smoothing.
#
#   OUTER: smc_outer_node.py -- position -> robust converging velocity reference. Feeds the
#          NMPC nodes ONLY; the safe controller no longer uses it.
#
# CONTROLLERS env selects which nodes actually start (default: geometric alone). Every
# standby node still solves at full rate and competes with Isaac/PX4/GS for CPU, and an
# overrunning control loop is a documented cause of a HIL tumble -- so run what you need:
#     CONTROLLERS="geometric"              (default)
#     CONTROLLERS="geometric nmpc_full"    the safe controller plus the one under test
#     CONTROLLERS=all                      everything (the old behaviour)
#
# Windows:
#   0 dds  1 px4  2 isaac(->mocap)  3 mocap  4 estimator  4b ev-bridge(mocap->px4 EKF EV)
#   5 smc-outer  6 geometric(SAFE, active)  6b transition  6c nmpc-cruise  6d cascade
#   6e so3  6f nmpc6  6g nmpc-full  7 attnmpc  8 ekf-relax/geofence/EV  9 ground-station
#   (6b..7 appear only if named in CONTROLLERS)
#
# Env knobs: CONTROLLERS="geometric ..." (which nodes to run), GEO_VMAX=4 (safe-controller
#            horizontal speed cap m/s), GEO_VZ_UP/GEO_VZ_DN, EV_BRIDGE=1 (mocap->px4 EV),
#            DISABLE_GEOFENCE=1 (GF_* off), SMC_VMAX=20, AUTO_ARM=true, FW_VCRUISE=15.
#
# Usage:  scripts/start_vtol_traj_hil.sh <machine-config>      # e.g. pieper_machine
# ============================================================================
PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_quad_tiltrotor}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
GAZEBO_WS="${GAZEBO_WS:-$HOME/workspaces/gazebo}"   # fsc estimator + msgs live here
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
PX4_UXRCE_DDS_NS="${PX4_UXRCE_DDS_NS:-uav_0}"
AUTO_ARM="${AUTO_ARM:-false}"   # arming is the OPERATOR's (GS: ARM / OFFBOARD / TAKEOFF).
# With auto-arm on, a controller node re-arms whenever it is active -- it silently undoes a
# DISARM and arms the aircraft without anyone asking.  Export AUTO_ARM=true to restore it.
EV_BRIDGE="${EV_BRIDGE:-1}"                # 1 = feed mocap to PX4 as external vision (EKF2 EV)
DISABLE_GEOFENCE="${DISABLE_GEOFENCE:-1}"  # 1 = turn off PX4 geofence (GF_*) so it can fly far
CTRL_METHOD="${CTRL_METHOD:-se3}"          # se3 (geometric PID+FF); loop uses SE(3) attitude
PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-1}"
VTOL_ROS2_STATE=1
# 1 = spawn the FREE-SURFACE TANK model (VTOLSlosh) instead of the baseline VTOL.
# Default 0, so this launcher is byte-identical to before unless it is asked for.
# Set it through scripts/start_slosh_hil.sh, which also VERIFIES that the model
# actually came up -- a stack that silently flies the baseline while you believe
# you are flying the tank is the failure this whole gate exists to prevent.
VTOL_SLOSH="${VTOL_SLOSH:-0}"
VTOL_TANK="${VTOL_TANK:-1}"          # 1 dead mass (phase 2) | slosh (phase 3) | 0 no tank
# The controller needs rclpy/px4_msgs -> the isaac_env venv python.
NODE_PY="${NODE_PY:-$HOME/envs/isaac_env/bin/python3}"
GS_DIR="${GS_DIR:-$ROS2_WS/src/ros2_ground_station_gui}"
# ============================================================================

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

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

PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2; exit 1; }
DA_DIR="${FSC_CONTROLLER_DIR}/px4_vtol_direct_actuator"
GEO_NODE="${DA_DIR}/geometric_hover_node.py"      # the SAFE controller: go to x,y,z and hold
TRAJ_NODE="${GEO_NODE}"                           # (compat alias)
SMC_NODE="${DA_DIR}/smc_outer_node.py"            # outer-loop sliding-mode (-> reference_smc)
ATT_NODE="${DA_DIR}/nmpc_attitude_node.py"        # inner loop B (attitude-NMPC, PX4 inner)
EV_NODE="${DA_DIR}/mocap_to_px4_ev.py"            # mocap -> PX4 external vision (fixes EKF yaw for B)
TRANS_NODE="${DA_DIR}/aero_transition_node.py"    # aero-aware tilt transition (hover<->cruise)
NMPC_CRUISE_NODE="${DA_DIR}/nmpc_cruise_node.py"  # in-loop transition NMPC (co-optimizes tilt+thrust)
CASCADE_NODE="${DA_DIR}/nmpc_so3_cascade_node.py" # SO(3) NMPC (async tilt-FF) + geometric pos inner -> fly+land
SO3_NODE="${DA_DIR}/nmpc_so3_transition_node.py"  # the AVTOL SO(3) NMPC IN-LOOP (JIT'd, 25 Hz) -> 90-deg cruise
NMPC6_NODE="${DA_DIR}/nmpc6_cruise_node.py"       # 6-STATE pitch-moment longitudinal NMPC -> genuine 90-deg WING-BORNE
FULL_NODE="${DA_DIR}/nmpc_full_smc_node.py"       # FULL-STATE (19) 6-DOF NMPC + outer positional SMC -> 90-deg wing-borne, lateral IN the solve
# CASCADE: 6-state longitudinal NMPC outer (bg thread) + geometric inner -> roll/yaw at
# mocap rate. CASC_NODE_PY selects a VARIANT of it (nmpc_cascade_tank_node.py: same
# node name and controller_id, one schedule function replaced) -- the tank stack
# sets it; unset means the base node, unchanged.
CASC_NODE="${CASC_NODE_PY:-${DA_DIR}/nmpc_cascade_node.py}"
REF_TOPIC="/${PX4_UXRCE_DDS_NS}/fsc_autopilot_ros2/position_controller/reference"
# ---- WHICH CONTROLLERS TO RUN ----------------------------------------------------------
# Every standby controller still SOLVES at its full rate; seven of them were competing with
# Isaac + PX4 + the GS for CPU, and a controller loop that overruns its period is a
# documented cause of a HIL tumble (the 19-state solve measured 22 ms idle vs 48-96 ms under
# load). So the launcher now starts only what you ask for.
#   CONTROLLERS="geometric"                 <- the default: the safe hover controller alone
#   CONTROLLERS="geometric nmpc_full"       <- plus the one you are actually testing
#   CONTROLLERS=all                         <- the old behaviour (everything)
# Names: geometric, nmpc_cruise, nmpc6, nmpc_full, so3, cascade, transition, attnmpc, vtol_nmpc
CONTROLLERS="${CONTROLLERS:-geometric nmpc_full cascade}"   # nmpc_full is NOT optional any more:
# the ground station's "Run Full Auto Mission" configures /nmpc_full_smc_control and ABORTS
# if it cannot.  With the old `geometric`-only default the node never started, so the button
# took off, failed every parameter set, and held altitude -- which reads as "nothing happens".
want() {
  [[ "$CONTROLLERS" == all ]] && return 0
  [[ " $CONTROLLERS " == *" $1 "* ]]
}
SMC_VMAX="${SMC_VMAX:-20.0}"                      # cruise horizontal speed cap [m/s]
[[ "$SMC_VMAX" == *.* ]] || SMC_VMAX="${SMC_VMAX}.0"   # force DOUBLE (rclpy rejects int for a double param)
SMC_LEAD="${SMC_LEAD:-30.0}"                      # SMC setpoint lead distance [m] (reset_dist): lead FAR
[[ "$SMC_LEAD" == *.* ]] || SMC_LEAD="${SMC_LEAD}.0"   # so the inner NMPC can build to wing-borne speed
FW_VCRUISE="${FW_VCRUISE:-15.0}"                  # AVTOL-scaled profile cruise speed (full wing-borne >13.3)
[[ "$FW_VCRUISE" == *.* ]] || FW_VCRUISE="${FW_VCRUISE}.0"
AVTOL_ENV="${AVTOL_REF_DIR:+AVTOL_REF_DIR=\"$AVTOL_REF_DIR\" }"   # pass through if set (default path works)
# ★ THE WATER MASS GOES TO EVERY PANE. VTOL_TANK_KG (0.1 / 0.502 / 0.7) is read by the plant
# (slosh.default_tank) AND by every controller (so3.M = airframe + it). A pane does not inherit
# this shell, so it is written INLINE on each command below; a pane it does not reach flies
# a different aircraft from the others. start_slosh_hil.sh checks both banners against it.
TANK_KG="${VTOL_TANK_KG:-0.502}"
SO3_VC="${SO3_VC:-18.2}"   # NOTE: Vc=15 trims at alpha 6.7-12 (nose-high); alpha=4 needs 18.2 m/s + corridor retuning (see docs);  [[ "$SO3_VC"  == *.* ]] || SO3_VC="${SO3_VC}.0"     # force DOUBLE for rclpy
SO3_BC="${SO3_BC:-90.0}";  [[ "$SO3_BC"  == *.* ]] || SO3_BC="${SO3_BC}.0"
SO3_TF="${SO3_TF:-24.0}";  [[ "$SO3_TF"  == *.* ]] || SO3_TF="${SO3_TF}.0"
SO3_LAT="${SO3_LAT:-0.15}"; [[ "$SO3_LAT" == *.* ]] || SO3_LAT="${SO3_LAT}.0"
# FULL-MISSION mode on the SO(3) node (climb->transition->cruise->back->land at a target). Off by
# default. SO3_MISSION=1 flies it self-contained (no SMC/GS needed); offline-validated to vc~14.
SO3_MISSION="${SO3_MISSION:-0.0}"; [[ "$SO3_MISSION" == *.* ]] || SO3_MISSION="${SO3_MISSION}.0"
MISSION_TX="${MISSION_TX:-200.0}"; [[ "$MISSION_TX" == *.* ]] || MISSION_TX="${MISSION_TX}.0"
MISSION_TY="${MISSION_TY:-0.0}";   [[ "$MISSION_TY" == *.* ]] || MISSION_TY="${MISSION_TY}.0"
MISSION_VC="${MISSION_VC:-13.0}";  [[ "$MISSION_VC" == *.* ]] || MISSION_VC="${MISSION_VC}.0"
MISSION_CAP="${MISSION_CAP:-55.0}"; [[ "$MISSION_CAP" == *.* ]] || MISSION_CAP="${MISSION_CAP}.0"
MISSION_ZC="${MISSION_ZC:-15.0}";  [[ "$MISSION_ZC" == *.* ]] || MISSION_ZC="${MISSION_ZC}.0"
# 6-STATE wing-borne NMPC (genuine 90-deg pure-puller cruise). STANDBY; activate with
# active_controller=nmpc6_cruise. Take off on geometric first, then engage at cruise altitude.
NMPC6_VC="${NMPC6_VC:-16.0}";  [[ "$NMPC6_VC"  == *.* ]] || NMPC6_VC="${NMPC6_VC}.0"
NMPC6_CAP="${NMPC6_CAP:-90.0}"; [[ "$NMPC6_CAP" == *.* ]] || NMPC6_CAP="${NMPC6_CAP}.0"
NMPC6_TTR="${NMPC6_TTR:-18.0}"; [[ "$NMPC6_TTR" == *.* ]] || NMPC6_TTR="${NMPC6_TTR}.0"
NMPC6_LAT="${NMPC6_LAT:-0.10}"
NMPC6_ALT="${NMPC6_ALT:-15.0}"; [[ "$NMPC6_ALT" == *.* ]] || NMPC6_ALT="${NMPC6_ALT}.0"  # climb-to alt before transition
# FULL-STATE NMPC + outer SMC (roll/yaw/pitch/tilt all inside one solve). STANDBY;
# activate with active_controller=nmpc_full. Env knobs mirror the nmpc6 ones.
FULL_VC="${FULL_VC:-16.0}";  [[ "$FULL_VC"  == *.* ]] || FULL_VC="${FULL_VC}.0"
FULL_CAP="${FULL_CAP:-90.0}"; [[ "$FULL_CAP" == *.* ]] || FULL_CAP="${FULL_CAP}.0"
FULL_TTR="${FULL_TTR:-18.0}"; [[ "$FULL_TTR" == *.* ]] || FULL_TTR="${FULL_TTR}.0"
FULL_ALT="${FULL_ALT:-15.0}"; [[ "$FULL_ALT" == *.* ]] || FULL_ALT="${FULL_ALT}.0"
FULL_RATE="${FULL_RATE:-10.0}"; [[ "$FULL_RATE" == *.* ]] || FULL_RATE="${FULL_RATE}.0"
FULL_LAT="${FULL_LAT:-0.12}"
for f in "$TRAJ_NODE" "$SMC_NODE" "$ATT_NODE" "$EV_NODE" "$TRANS_NODE" "$NMPC_CRUISE_NODE" "$NMPC6_NODE" "$FULL_NODE"; do
  [[ -f "$f" ]] || { echo "ERROR: node not found: $f" >&2; exit 1; }
done

ISAAC_ROOT="$(cd -- "$(dirname -- "$ISAAC_PY")" && pwd)"
ISAAC_BUNDLED_PY="${ISAAC_ROOT}/kit/python/bin/python3"
[[ -x "$ISAAC_BUNDLED_PY" ]] || { echo "ERROR: Isaac bundled Python not found: $ISAAC_BUNDLED_PY" >&2; exit 1; }
BRIDGE_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$PX4_BIN" ]] || { echo "ERROR: PX4 SITL binary not built: $PX4_BIN" >&2; exit 1; }
if [[ -d "$PX4_DIR/build/px4_sitl_default/rootfs" ]]; then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/rootfs"
elif [[ -d "$PX4_DIR/build/px4_sitl_default/tmp/rootfs" ]]; then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/tmp/rootfs"
else echo "ERROR: PX4 SITL rootfs not found." >&2; exit 1; fi
PX4_WORKDIR="$PX4_DIR/build/px4_sitl_default/rootfs_vtol0"
[[ -d "$PX4_WORKDIR" ]] || cp -a "$ROOTFS_BASE" "$PX4_WORKDIR"

AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
if [[ "$PX4_SYS_AUTOSTART" == "auto" ]]; then
  match="$(ls "$AF_DIR" 2>/dev/null | grep -E "_${PX4_SIM_MODEL}\$" | head -n1 || true)"
  [[ -z "$match" ]] && match="$(ls "$AF_DIR" 2>/dev/null | grep -- "$PX4_SIM_MODEL" | head -n1 || true)"
  [[ -n "$match" ]] && PX4_SYS_AUTOSTART="${match%%_*}" || { echo "ERROR: cannot resolve autostart for '$PX4_SIM_MODEL'." >&2; exit 1; }
fi

SESSION="vtol_smc_hil"
D_PX4=2; D_ISAAC=4; D_MOCAP=14; D_EST=16; D_CTRL=18; D_RELAX=12; D_GS=24; PX4_INSTANCE=0
# The SO(3) node loads a 32 MB JIT solver cache (mmap). Building it at D_CTRL=18 s -- while
# Isaac is still loading -- SIGBUS-crashed the mmap under transient memory pressure (a manual
# respawn AFTER Isaac settled always worked). Build it LATE, after Isaac is up.
D_SO3="${D_SO3:-105}"
echo "[SMC] model=$PX4_SIM_MODEL ns=$PX4_UXRCE_DDS_NS  SMC outer loop -> A:geometric(active) / B:attnmpc  (GS switch)"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
# no other controller may share the PX4 setpoint/actuator topics
pkill -9 -f autopilot_sv_baseline 2>/dev/null || true

# shared prelude for the ROS 2 windows (system ROS + overlays), matching ~/.bashrc order
ROS_PRELUDE="unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_PRELOAD;
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
source /opt/ros/$ROS_DISTRO/setup.bash;
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\";
[ -f \"$GAZEBO_WS/install/setup.bash\" ] && source \"$GAZEBO_WS/install/setup.bash\";"

# ---- 0: DDS agent ----------------------------------------------------------
tmux new-session -d -s "$SESSION" -n "0-dds" "
echo '[0-dds] Micro XRCE-DDS Agent udp4 -p $DDS_AGENT_PORT'
command -v MicroXRCEAgent >/dev/null 2>&1 && MicroXRCEAgent udp4 -p $DDS_AGENT_PORT || echo 'ERROR: MicroXRCEAgent not on PATH.'
exec bash
"
# ---- 1: PX4 ----------------------------------------------------------------
tmux new-window -t "$SESSION" -n "1-px4" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo '[1-px4] waiting $D_PX4 s...'; sleep $D_PX4
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS \
  \"$PX4_BIN\" -i $PX4_INSTANCE -w \"$PX4_WORKDIR\"
echo '[1-px4] exited.'; exec bash
"
# ---- 2: Isaac (state -> mocap) ---------------------------------------------
tmux new-window -t "$SESSION" -n "2-isaac" "
echo '[2-isaac] waiting $D_ISAAC s...'; sleep $D_ISAAC
unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:$BRIDGE_LIB\"
PEGASUS_HIL_DEBUG=$PEGASUS_HIL_DEBUG PEGASUS_PROP_DEBUG=\${PEGASUS_PROP_DEBUG:-} VTOL_ROS2_STATE=$VTOL_ROS2_STATE VTOL_SLOSH=$VTOL_SLOSH VTOL_TANK=$VTOL_TANK VTOL_TANK_KG=$TANK_KG VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_CG_DX=${VTOL_CG_DX:-0} VTOL_GPS_HZ=\${VTOL_GPS_HZ:-} VTOL_GPS_SIGMA=\${VTOL_GPS_SIGMA:-} VTOL_GPS_VSIGMA=\${VTOL_GPS_VSIGMA:-} VTOL_GPS_WALK=\${VTOL_GPS_WALK:-} PYTHONEXE=\"$ISAAC_BUNDLED_PY\" \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo '[2-isaac] exited.'; exec bash
"
# ---- 3: mocap emulator -----------------------------------------------------
tmux new-window -t "$SESSION" -n "3-mocap" "
echo '[3-mocap] waiting $D_MOCAP s...'; sleep $D_MOCAP
$ROS_PRELUDE
ros2 launch isaacsim_optitrack_ros2_emulator emulator_launch.py bodies:=$PX4_UXRCE_DDS_NS
echo '[3-mocap] exited.'; exec bash
"
# ---- 4: estimator (mocap -> odom, so the GS knows the vehicle position) -----
tmux new-window -t "$SESSION" -n "4-estimator" "
echo '[4-estimator] waiting $D_EST s...'; sleep $D_EST
$ROS_PRELUDE
ros2 launch fsc_drone_state_estimator_ros2 indoor_estimator_launch.py uav_prefix:=$PX4_UXRCE_DDS_NS
echo '[4-estimator] exited.'; exec bash
"
# ---- 4b: mocap -> PX4 external vision (EKF2 EV) so the PX4 EKF gets a correct yaw.
#          This is what makes inner-loop B (attitude-NMPC) viable in HIL. Harmless to A
#          (A uses mocap directly); it also steadies the EKF so arming is more reliable. ---
if [[ "$EV_BRIDGE" == 1 ]]; then
tmux new-window -t "$SESSION" -n "4b-ev" "
echo '[4b-ev] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[4b-ev] mocap -> /$PX4_UXRCE_DDS_NS/fmu/in/vehicle_visual_odometry (EKF2 EV)...'
\"$NODE_PY\" \"$EV_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS
echo '[4b-ev] exited.'; exec bash
"
fi
# ---- 5: SMC outer loop (target -> converging setpoint + robust velocity) ---
tmux new-window -t "$SESSION" -n "5-smc" "
echo '[5-smc] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[5-smc] outer-loop sliding-mode: $REF_TOPIC -> ${REF_TOPIC}_smc (vmax=$SMC_VMAX, lead=$SMC_LEAD)'
\"$NODE_PY\" \"$SMC_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p vmax:=$SMC_VMAX \
    -p reset_dist:=$SMC_LEAD -p amax:=2.5
echo '[5-smc] exited.'; exec bash
"
# ---- 6: THE SAFE CONTROLLER -- geometric hover (ACTIVE at start) ------------
#      Goes to the x,y,z the operator types and holds it. Pure multicopter: the rotors
#      are vertical at all times. It reads the RAW reference and does its OWN
#      velocity-limited ramp, so it is NOT SMC-fed any more -- one reference topic, one
#      publisher on it. (The SMC still feeds the NMPC nodes that were tuned around it.)
tmux new-window -t "$SESSION" -n "6-geometric" "
echo '[6-geometric] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[6-geometric] SAFE hover controller (raw reference, ACTIVE at start)...'
VTOL_TANK_KG=$TANK_KG VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_TANK_DZ=${VTOL_TANK_DZ:-0} \"$NODE_PY\" \"$GEO_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
    -p controller_id:=geometric -p active_at_start:=true \
    -p v_max:=${GEO_VMAX:-4.0} -p vz_up:=${GEO_VZ_UP:-2.0} -p vz_down:=${GEO_VZ_DN:-1.0} -p state_source:=${GEO_STATE:-mocap}
echo '[6-geometric] exited.'; exec bash
"
# ---- 6b: AERO-AWARE TILT TRANSITION (hover<->cruise), STANDBY at start. Activate with
#          active_controller=transition. Subtracts modeled wing lift/drag -> holds altitude
#          through the transition + tilts to cruise (offline-validated). SMC-fed like A. ----
if want transition; then
# DISABLED BY DEFAULT (RUN_TRANSITION_C=1 to enable): this standby node was observed
# CO-PUBLISHING actuators at its 200 Hz rate during D/E flights (HIL bags fwD1/fwE2 --
# a constant 45-deg tilt + zero elevator stream fighting the active controller until the
# vehicle porpoised/crashed; the stream stopped the moment this node was killed). The
# fixed-wing transition role is covered by D (nmpc_cruise, profile-driven).
tmux new-window -t "$SESSION" -n "6b-transition" "
echo '[6b-transition] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[6b-transition] aero-aware tilt transition (STANDBY; activate: active_controller=transition)...'
\"$NODE_PY\" \"$TRANS_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
    -p controller_id:=transition -p active_at_start:=false \
    -r ${REF_TOPIC}:=${REF_TOPIC}_smc
echo '[6b-transition] exited.'; exec bash
"
fi
# ---- 6c: IN-LOOP TRANSITION NMPC (hover<->cruise), STANDBY at start. Activate with
#          active_controller=nmpc_cruise. CO-OPTIMIZES tilt+thrust (longitudinal NMPC) +
#          gentle rate/saturation-limited attitude/lateral loop. Offline-validated: holds
#          altitude within 0.23 m through a full 0->16 m/s transition. SMC-fed like A. ----
if want nmpc_cruise; then
tmux new-window -t "$SESSION" -n "6c-nmpc-cruise" "
echo '[6c-nmpc-cruise] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[6c-nmpc-cruise] D: FIXED-WING transition NMPC (STANDBY; activate: active_controller=nmpc_cruise)...'
$AVTOL_ENV\"$NODE_PY\" \"$NMPC_CRUISE_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
    -p controller_id:=nmpc_cruise -p active_at_start:=false -p vmax_fwd:=${NMPC_VMAX:-6.0} -p a_fwd:=1.5 \
    -p pitch_rate_deg_s:=15.0 -p cruise_trim_deg:=${CRUISE_TRIM:-3.0} \
    -p use_profile:=${USE_PROFILE:-1.0} -p profile_v_cruise:=$FW_VCRUISE \
    -r ${REF_TOPIC}:=${REF_TOPIC}_smc
    # NB: reads reference_smc (SMC-fed). The SMC's velocity-limited ramp is REQUIRED for stability --
    # driving the raw step directly (even with the NMPC's internal ramp) tumbled it at the cruise onset.
    # The SMC leads far ($SMC_LEAD) so the inner NMPC can still build to wing-borne speed.
    # NB: reads reference_smc (SMC-fed) so the reference is SMOOTH (velocity-limited) -- a raw step
    # trajectory (GS Play / a far target) destabilised it (fly-away). SMC smoothing = stable tracking.
echo '[6c-nmpc-cruise] exited.'; exec bash
"
fi
# ---- 6d: SO(3) CASCADE fly-to-point + LAND (STANDBY at start). Activate with
#          active_controller=so3_cascade (GS button 'E: SO(3) Cascade'). OUTER = SO(3) NMPC async
#          (tilt feed-forward); INNER = fast geometric tiltrotor position control + wing-lift FF.
#          Reads the RAW GS reference directly (NOT SMC-fed) -> the GS Fly-to-point mission drives it. ----
if want so3_cascade; then
tmux new-window -t "$SESSION" -n "6d-cascade" "
echo '[6d-cascade] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[6d-cascade] E: SO(3) cascade FIXED-WING fly+land (STANDBY; activate: active_controller=so3_cascade)...'
$AVTOL_ENV\"$NODE_PY\" \"$CASCADE_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=false \
    -p controller_id:=so3_cascade -p active_at_start:=false \
    -p cruise_speed:=${CASC_VC:-12.0} -p beta_ff_gain:=${CASC_BFF:-0.0} -p wing_ff_gain:=${CASC_WFF:-0.8} \
    -p fixed_wing:=${CASC_FW:-false} -p profile_v_cruise:=$FW_VCRUISE
    # CASC_FW=false default: E's PROFILE fixed-wing branch is EXPERIMENTAL -- its traverse-engage
    # transient toppled the vehicle across 5 HIL attempts (bags fwE2..fwE7: mode-flap -> backward
    # slide -> self-trapped tilt; each root cause fixed but the engage is still not robust).
    # E defaults to its VALIDATED multicopter tilt-for-forward fly-to-point (precise landing),
    # improved this session by the Mission lead-clamp/z-ramp/clock-freeze. CASC_FW=true to research
    # the fixed-wing branch. The VALIDATED fixed-wing fly-to-point is D (nmpc_cruise).
echo '[6d-cascade] exited.'; exec bash
"
fi
# ---- 6e: the AVTOL SO(3) transition NMPC IN-LOOP (user's derivation, JIT-compiled -> 25 Hz).
#          STANDBY; activate with active_controller=so3_transition. Flies its own hover->90deg
#          pure-puller transition path from the engage point (does NOT read the reference topic).
#          First-ever run JIT-compiles ~5 min then caches (~/.cache/fsc_so3_nmpc_*).
#          SO3_LAT: start HIGH and walk down live (under-compensation is the fatal direction). ----
if want so3; then
tmux new-window -t "$SESSION" -n "6e-so3" "
echo '[6e-so3] waiting $D_SO3 s (build the 32MB JIT solver AFTER Isaac settles -- see D_SO3 note)...'; sleep $D_SO3
$ROS_PRELUDE
echo '[6e-so3] AVTOL SO(3) NMPC in-loop (STANDBY; activate: active_controller=so3_transition)...'
for try in 1 2 3 4 5; do
  $AVTOL_ENV\"$NODE_PY\" \"$SO3_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
      -p controller_id:=so3_transition -p active_at_start:=false \
      -p Vcruise:=$SO3_VC -p beta_cruise_deg:=$SO3_BC -p transition_tf:=$SO3_TF \
      -p latency_s:=$SO3_LAT \
      -p mission_mode:=$SO3_MISSION -p mission_target_x:=$MISSION_TX -p mission_target_y:=$MISSION_TY \
      -p mission_vcruise:=$MISSION_VC -p mission_cap_deg:=$MISSION_CAP -p mission_zcruise:=$MISSION_ZC \
      -r ${REF_TOPIC}:=${REF_TOPIC}_smc
  # SMC-FED POSITION: read the SMC outer loop's set-point (reference_smc) as the along-path
  # DESTINATION. The SO(3) NMPC follows its tilt/attitude/velocity transition schedule to reach it.
  code=\$?
  [ \$code -eq 0 ] && break   # clean exit (Ctrl-C / teardown)
  echo \"[6e-so3] node exited code \$code -- likely SIGBUS on the cache mmap; retry \$try after 10s...\"; sleep 10
done
echo '[6e-so3] exited.'; exec bash
"
fi
# ---- 6f: 6-STATE pitch-moment NMPC -> genuine 90-deg WING-BORNE cruise (STANDBY at start) -----
#          The ONLY controller that cracks the rotor->wing handoff: it co-optimizes the pitch
#          MOMENT (state theta,q; qdot=My/Iyy; elevator in-loop). Offline (true plant, 100ms lat):
#          sustained tilt=90, AoA+6.4, rotors unloaded, altitude flat. Take off on geometric, then
#          activate with active_controller=nmpc6_cruise. Env: NMPC6_VC/CAP/TTR/LAT.
if want nmpc6; then
tmux new-window -t "$SESSION" -n "6f-nmpc6" "
echo '[6f-nmpc6] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[6f-nmpc6] 6-STATE wing-borne NMPC (STANDBY; activate: active_controller=nmpc6_cruise)...'
$AVTOL_ENV\"$NODE_PY\" \"$NMPC6_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
    -p controller_id:=nmpc6_cruise -p active_at_start:=false \
    -p vcruise:=$NMPC6_VC -p cap_deg:=$NMPC6_CAP -p ttr:=$NMPC6_TTR -p latency_s:=$NMPC6_LAT \
    -p cruise_alt:=$NMPC6_ALT
echo '[6f-nmpc6] exited.'; exec bash
"
fi
# ---- 6g: FULL-STATE (19) 6-DOF NMPC + outer positional SMC -> 90-deg wing-borne (STANDBY) -----
#          Supersedes 6f for robustness: roll/yaw/pitch/tilt are ALL allocated INSIDE one solve
#          (the 6-state node's hand-rolled lateral PD was fragile at 90 deg). Offline vs the true
#          plant @100ms latency: sustained tilt=90, AoA+6.4, alt flat, roll/yaw->0, and it RECOVERS
#          from combined roll+yaw disturbances that crashed 6f. SMC-fed (reference_smc) for the
#          destination. 20 Hz (solve 22 ms mean / 31 max). Activate: active_controller=nmpc_full.
if want nmpc_full; then
tmux new-window -t "$SESSION" -n "6g-full" "
echo '[6g-full] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[6g-full] FULL-STATE NMPC + SMC (STANDBY; activate: active_controller=nmpc_full)...'
VTOL_TANK_KG=$TANK_KG VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_TANK_DZ=${VTOL_TANK_DZ:-0} $AVTOL_ENV\"$NODE_PY\" \"$FULL_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
    -p controller_id:=nmpc_full -p active_at_start:=false \
    -p vcruise:=$FULL_VC -p cap_deg:=$FULL_CAP -p ttr:=$FULL_TTR -p cruise_alt:=$FULL_ALT \
    -p rate_hz:=$FULL_RATE -p latency_s:=$FULL_LAT \
    -r ${REF_TOPIC}:=${REF_TOPIC}_smc
echo '[6g-full] exited.'; exec bash
"
fi
# ---- 6h: CASCADE -- 6-state longitudinal NMPC outer + geometric inner (STANDBY at start) ----
#      No rate_hz: the inner loop runs on every mocap sample and the outer NMPC runs in a
#      background thread at outer_hz (solve is 9-12 ms on the bench vs the 19-state's 27-93).
#      kR stays at the node default 3.0 -- softer wins at EVERY latency measured offline, and the
#      latency ceiling is the attitude loop's, not the solver's.
if want cascade; then
tmux new-window -t "$SESSION" -n "6h-cascade" "
echo '[6h-cascade] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[6h-cascade] CASCADE NMPC + geometric (STANDBY; activate: active_controller=cascade)...'
VTOL_TANK_KG=$TANK_KG VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_TANK_DZ=${VTOL_TANK_DZ:-0} $AVTOL_ENV\"$NODE_PY\" \"$CASC_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
    -p controller_id:=cascade -p active_at_start:=false \
    -p vcruise:=${CASC_VC:-16.0} -p cap_deg:=90.0 -p ttr:=${CASC_TTR:-10.0} \
    -p cruise_alt:=${CASC_ALT:-30.0} -p outer_hz:=${CASC_OUTER_HZ:-20.0} -p latency_s:=${CASC_LAT:-0.049} -p l1_on:=${CASC_L1:-true} -p state_source:=${CASC_STATE:-mocap} \
    -r ${REF_TOPIC}:=${REF_TOPIC}_smc
echo '[6h-cascade] exited.'; exec bash
"
fi
# ---- 6i: ADAPTIVE CASCADE -- the same cascade with the SO(3) STESO + the slosh predictor inside
#      its L1 (nmpc_cascade_steso_node.py), as its OWN controller (id cascade_adaptive, node
#      nmpc_cascade_control_adaptive, guidance on /uav_0/cascade_adaptive/*). STANDBY beside 6h:
#      the GS's "Adaptive NMPC + Geo" button / ctrl choice selects it. ADAPTIVE=0 skips it.
#      Starts 15 s after 6h: both load the same nmpc6 JIT cache file, and parallel loads of one
#      cache have core-dumped casadi before (the 16-seed sweep note).
ADAPTIVE_NODE="${ADAPTIVE_NODE_PY:-${DA_DIR}/nmpc_cascade_steso_node.py}"
if want cascade && [ "${ADAPTIVE:-1}" != "0" ] && [ -f "$ADAPTIVE_NODE" ]; then
tmux new-window -t "$SESSION" -n "6i-adaptive" "
echo '[6i-adaptive] waiting $((D_CTRL + 15)) s...'; sleep $((D_CTRL + 15))
$ROS_PRELUDE
echo '[6i-adaptive] ADAPTIVE cascade: NMPC + geometric with STESO + slosh predictor (STANDBY; activate: active_controller=cascade_adaptive)...'
VTOL_TANK_KG=$TANK_KG VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_TANK_DZ=${VTOL_TANK_DZ:-0} CASC_EST=${CASC_EST:-steso} STESO_L=${STESO_L:-} STESO_LF=${STESO_LF:-} STESO_MODE=${STESO_MODE:-} STESO_EPS=${STESO_EPS:-} LESO_WO=${LESO_WO:-} LESO_WOF=${LESO_WOF:-} MDOB_K=${MDOB_K:-} MDOB_KF=${MDOB_KF:-} IMU_WC=${IMU_WC:-} IMU_DR=${IMU_DR:-} IMU_DELAY=${IMU_DELAY:-} SLOSH_PRED=${SLOSH_PRED:-} SLOSH_PRED_LAG=${SLOSH_PRED_LAG:-} SLOSH_PRED_CANCEL=${SLOSH_PRED_CANCEL:-} SLOSH_PRED_OUTER=${SLOSH_PRED_OUTER:-} SLOSH_PRED_UMAX=${SLOSH_PRED_UMAX:-} SLOSH_PRED_Q=${SLOSH_PRED_Q:-} SLOSH_PRED_RF=${SLOSH_PRED_RF:-} SLOSH_PRED_RM=${SLOSH_PRED_RM:-} $AVTOL_ENV\"$NODE_PY\" \"$ADAPTIVE_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
    -p controller_id:=cascade_adaptive -p active_at_start:=false \
    -p vcruise:=${CASC_VC:-16.0} -p cap_deg:=90.0 -p ttr:=${CASC_TTR:-10.0} \
    -p cruise_alt:=${CASC_ALT:-30.0} -p outer_hz:=${CASC_OUTER_HZ:-20.0} -p latency_s:=${CASC_LAT:-0.049} -p l1_on:=true -p state_source:=${CASC_STATE:-mocap} \
    -r ${REF_TOPIC}:=${REF_TOPIC}_smc
echo '[6i-adaptive] exited.'; exec bash
"
fi
# ---- 7: inner loop B -- attitude-NMPC + embedded SMC (STANDBY at start) -----
#         hover_throttle=0.50 (0.70 over-scaled -> climb-away, see diagnosis).
if want attnmpc; then
tmux new-window -t "$SESSION" -n "7-attnmpc" "
echo '[7-attnmpc] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[7-attnmpc] B: attitude-NMPC + SMC (PX4 inner loop, STANDBY at start)...'
\"$NODE_PY\" \"$ATT_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
    -p controller_id:=attnmpc -p active_at_start:=false -p hover_throttle:=0.50 \
    -p smc_vmax:=$SMC_VMAX
echo '[7-attnmpc] exited.'; exec bash
"
fi
# ---- 8: EKF arming-check relaxation (the HIL EKF flags 'Attitude failure' on ~half
#         the boots; the controllers use MOCAP for state, so relax the preflight so
#         they can arm). Sends the params to the PX4 console once it is up.
tmux new-window -t "$SESSION" -n "8-relax" "
echo '[8-relax] waiting $D_RELAX s for PX4...'; sleep $D_RELAX
for p in 'COM_ARM_EKF_HGT 3.0' 'COM_ARM_EKF_VEL 1.0' 'COM_ARM_EKF_POS 1.0' \
         'COM_ARM_EKF_YAW 1.0' 'COM_ARM_IMU_ACC 1.0' 'COM_ARM_IMU_GYR 1.0' 'COM_ARM_MAG_ANG -1'; do
  tmux send-keys -t \"$SESSION:1-px4\" \"param set \$p\" Enter; sleep 0.4
done
if [ \"$DISABLE_GEOFENCE\" = 1 ]; then
  echo '[8-relax] disabling geofence (GF_*) so it can fly far / to 20 m/s...'
  for p in 'GF_ACTION 0' 'GF_MAX_HOR_DIST 0' 'GF_MAX_VER_DIST 0' 'GF_HMAX 0' 'GF_PREDICT 0'; do
    tmux send-keys -t \"$SESSION:1-px4\" \"param set \$p\" Enter; sleep 0.3
  done
fi
if [ \"$EV_BRIDGE\" = 1 ] && [ \"${EKF_SOURCE:-ev}\" = ev ]; then
  echo '[8-relax] enabling EKF2 external vision (mocap yaw for inner-loop B), then ekf2 restart...'
  for p in 'EKF2_EV_CTRL 11' 'EKF2_HGT_REF 3' 'EKF2_GPS_CTRL 0' 'EKF2_EV_DELAY 50'; do
    tmux send-keys -t \"$SESSION:1-px4\" \"param set \$p\" Enter; sleep 0.3
  done
  tmux send-keys -t \"$SESSION:1-px4\" 'ekf2 stop'  Enter; sleep 0.8
  tmux send-keys -t \"$SESSION:1-px4\" 'ekf2 start' Enter; sleep 0.5
  echo '[8-relax] EV on. Verify: /$PX4_UXRCE_DDS_NS/fmu/out/vehicle_attitude yaw should track mocap.'
elif [ \"$EV_BRIDGE\" = 1 ]; then
  # EKF_SOURCE=gps: GPS position/velocity + baro, vision YAW only from the mocap bridge. Set EXPLICITLY
  # (PX4 SITL persists parameters in build/.../parameters.bson, so a previous run's EV override
  # would otherwise survive the restart -- it did, 2026-09-15 19:45), then ekf2 restart.
  echo '[8-relax] EKF_SOURCE=${EKF_SOURCE:-ev}: EKF2 on GPS position/velocity + baro, vision yaw from the bridge -- the INS the state_source=px4 cascade flies on'
  for p in 'EKF2_GPS_CTRL 7' 'EKF2_EV_CTRL 8' 'EKF2_HGT_REF 1' 'EKF2_BARO_CTRL 1' 'EKF2_EV_DELAY 50'; do
    tmux send-keys -t \"$SESSION:1-px4\" \"param set \$p\" Enter; sleep 0.3
  done
  tmux send-keys -t \"$SESSION:1-px4\" 'ekf2 stop'  Enter; sleep 0.8
  tmux send-keys -t \"$SESSION:1-px4\" 'ekf2 start' Enter; sleep 0.5
fi
echo '[8-relax] arming checks relaxed. (commander arm -f here if a controller wont arm)'
exec bash
"
# ---- 9: ground station (inner-loop A/B switch + Trajectory tab / missions) ---
tmux new-window -t "$SESSION" -n "9-groundstn" "
echo '[9-groundstn] waiting $D_GS s...'; sleep $D_GS
export PATH=\"/usr/bin:\$PATH\"
$ROS_PRELUDE
cd \"$GS_DIR\" || { echo 'GS_DIR not found: $GS_DIR'; exec bash; }
echo '[9-groundstn] GS: Trajectory tab -> A/B inner-loop switch + Fly+Land mission (VTOL_TANK_KG=${VTOL_TANK_KG:-0.502})...'
VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} /usr/bin/python3 src/single_drone_ground_control.py
echo '[9-groundstn] exited.'; exec bash
"

# ---- 10: ROSBAG -- record EVERY run, automatically ----------------------------------------
# A flight with no bag cannot be diagnosed afterwards, and this project has repeatedly re-flown
# (~9 min each time) purely to capture data it had already produced.  So recording is NOT
# opt-in: every launch gets one.
# The name is TIMESTAMPED so runs never collide -- with a fixed name `ros2 bag record` refuses
# ("Output folder already exists") and you silently get NO recording, which has happened here.
#   BAG_DIR  where bags go        (default ~/flight_bags; /tmp is wiped on reboot)
#   BAG      this run's name      (default vtol_<date>_<time>)
#   BAG_ALL  0 = flight-critical topics only instead of everything
# STOP IT WITH Ctrl-C, NEVER kill -9: a SIGKILLed bag has no metadata.yaml and is UNREADABLE.
tmux new-window -t "$SESSION" -n "10-bag" "
echo '[10-bag] waiting $D_GS s...'; sleep $D_GS
$ROS_PRELUDE
mkdir -p \"${BAG_DIR:-$HOME/flight_bags}\" && cd \"${BAG_DIR:-$HOME/flight_bags}\" || { echo '[10-bag] bag dir failed'; exec bash; }
BAGNAME=\"${BAG:-vtol_\$(date +%Y%m%d_%H%M%S)}\"
echo \"[10-bag] recording -> ${BAG_DIR:-/tmp/bags}/\$BAGNAME\"
echo '[10-bag] Ctrl-C stops it cleanly; a kill -9 leaves an UNREADABLE bag.'
if [ \"${BAG_ALL:-1}\" = 1 ]; then
  ros2 bag record -a -o \"\$BAGNAME\"
else
  ros2 bag record -o \"\$BAGNAME\" \\
    /uav_0/mocap /uav_0/fmu/in/actuator_motors /uav_0/fmu/in/actuator_servos \\
    /uav_0/ground_station/active_controller \\
    /uav_0/fsc_autopilot_ros2/position_controller/reference \\
    /uav_0/fsc_autopilot_ros2/position_controller/reference_smc \\
    /uav_0/fmu/out/vehicle_status_v1 /uav_0/fmu/out/vehicle_odometry
fi
echo '[10-bag] recorder exited.'; exec bash
"

tmux select-window -t "$SESSION:9-groundstn"
tmux attach-session -t "$SESSION"
