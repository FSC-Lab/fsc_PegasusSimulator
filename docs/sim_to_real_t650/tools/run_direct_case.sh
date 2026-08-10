#!/usr/bin/env bash
# One closed-loop DIRECT-actuation case: the real fsc_autopilot_ros2 direct-actuation stack
# + PX4 SITL + the IsaacSim T650 plant, driven through a recorded reference sequence.
#
#   run_direct_case.sh <ref_npz> <out_npz> <tag>
#
# Start order is the one documented by the operator and matters: the ROS 2 stack first
# (it owns the MicroXRCEAgent, which must be listening before PX4 boots or the /fmu/*
# topics never appear), then Pegasus/PX4 SITL, then the driver. The driver requests
# OFFBOARD, arms, and switches to DIRECT itself -- SIMULATION ONLY.
set -uo pipefail

SCRATCH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "$1" in /*) REFS="$1";; *) REFS="$SCRATCH/$1";; esac
case "$2" in /*) OUT="$2";;  *) OUT="$SCRATCH/$2";; esac
TAG="$3"

PEGASUS_ROOT="/home/fsc-jupiter/Source/fsc_PegasusSimulator"
AUTOPILOT_WS="/home/fsc-jupiter/Workspaces/fsc_autopilot_ws"
AUTOPILOT_SRC="$AUTOPILOT_WS/src/fsc_autopilot_ros2"
STOP="$AUTOPILOT_SRC/scripts/isaacsim/stop_isaacsim_stack.sh"
STACK="$AUTOPILOT_SRC/scripts/isaacsim/start_direct_actuation_t650_stack.sh"
SIM="$PEGASUS_ROOT/scripts/indoor_sim/start_t650_direct_actuator_sitl.sh"
CFG="fsc_lab_machine"
NS="uav_0"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$AUTOPILOT_WS/install/setup.bash"
LOGDIR="$SCRATCH/logs/$TAG"
STACK_SESSION="fsc_direct_actuation_t650_stack"

mkdir -p "$LOGDIR"
[[ -f "$REFS" ]] || { echo "ERROR: no such reference file: $REFS" >&2; exit 1; }

cleanup() {
  echo "[case] cleanup"
  bash "$STOP" >>"$LOGDIR/stop.log" 2>&1
  tmux kill-session -t "$STACK_SESSION" 2>/dev/null
  tmux kill-session -t px4_isaac 2>/dev/null
  pkill -f '0[0-9]_px4_single_drone_t650' 2>/dev/null
  pkill -x px4 2>/dev/null
  sleep 3
  pkill -9 -f '0[0-9]_px4_single_drone_t650' 2>/dev/null
  pkill -9 -x px4 2>/dev/null
  pkill -x MicroXRCEAgent 2>/dev/null
  sleep 1
}
trap cleanup EXIT
cleanup
sleep 2

# --- 1. ROS 2 direct-actuation stack (owns the DDS agent) ---
echo "[case] starting stack: $STACK"
DISPLAY=:0 setsid bash "$STACK" "$CFG" "$NS" >"$LOGDIR/stack.log" 2>&1 &
for i in $(seq 1 90); do
  tmux has-session -t "$STACK_SESSION" 2>/dev/null && break
  sleep 1
done
tmux has-session -t "$STACK_SESSION" 2>/dev/null || {
  echo "ERROR: stack session never appeared" >&2; exit 1; }
echo "[case] stack session up after ${i}s"
sleep 10

# --- 2. Pegasus / PX4 SITL ---
echo "[case] starting sim: $SIM"
DISPLAY=:0 setsid bash "$SIM" --in-terminal "$CFG" >"$LOGDIR/sim_launcher.log" 2>&1 &
# Readiness is checked on the DDS topics, not by grepping the tmux pane: capture-pane
# truncates at the pane width, so the "Simulator connected" banner is not reliably
# greppable. A vehicle_status message arriving is the condition that actually matters --
# it means PX4 booted AND the uXRCE-DDS bridge is up.
set +u; source "$ROS2_SETUP"; source "$WS_SETUP"; set -u
# NOTE: do not use `ros2 topic echo --once` here. It subscribes RELIABLE by default while
# every /fmu/out/* topic is BEST_EFFORT, so it blocks forever even on a healthy link. The
# topic merely EXISTING is the real signal: /fmu/out/* is only advertised once PX4 has
# booted and the uXRCE-DDS bridge has connected.
LINKED=0
for i in $(seq 1 120); do
  if ros2 topic list 2>/dev/null | grep -q "^/$NS/fmu/out/vehicle_status_v1$"; then
    echo "[case] PX4 topics advertised after ~$((i*3))s"; LINKED=1; break
  fi
  sleep 3
done
[[ $LINKED -eq 1 ]] || { echo "ERROR: PX4 status topic never appeared" >&2; exit 1; }
sleep 20

# --- 3. drive the recorded reference sequence ---
echo "[case] driving $REFS -> $OUT"
set +u; source "$ROS2_SETUP"; source "$WS_SETUP"; set -u
/usr/bin/python3 "$SCRATCH/stack_driver_direct.py" \
  --refs "$REFS" --out "$OUT" --namespace "/$NS" 2>&1 | tee "$LOGDIR/driver.log"

echo "[case] done -> $OUT"
