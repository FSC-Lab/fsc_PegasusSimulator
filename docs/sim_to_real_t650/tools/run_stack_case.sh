#!/usr/bin/env bash
# One closed-loop case: PX4 SITL + IsaacSim T650 plant + the real fsc_autopilot_ros2
# baseline stack, driven through a recorded reference sequence.
#
#   run_stack_case.sh <ref_npz> <out_npz> <tag>
#
# Start order matters. MicroXRCEAgent must be listening before PX4 boots, otherwise the
# /fmu/* topics never appear (fsc_PegasusSimulator/CLAUDE.md). The stack script starts an
# agent of its own in pane 0; that one finds the port taken and drops to a shell, which is
# harmless.
set -uo pipefail

SCRATCH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "$1" in /*) REFS="$1";; *) REFS="$SCRATCH/$1";; esac
case "$2" in /*) OUT="$2";;  *) OUT="$SCRATCH/$2";; esac
TAG="$3"

PEGASUS_ROOT="/home/fsc-jupiter/Source/fsc_PegasusSimulator"
AUTOPILOT_WS="/home/fsc-jupiter/Workspaces/fsc_autopilot_ws"
SIM_LAUNCHER="$PEGASUS_ROOT/scripts/indoor_sim/start_single_drone_t650.sh"
STACK_LAUNCHER="$AUTOPILOT_WS/src/fsc_autopilot_ros2/scripts/isaacsim/start_baseline_t650_stack_fused.sh"
CFG="fsc_lab_machine"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$AUTOPILOT_WS/install/setup.bash"
LOGDIR="$SCRATCH/logs/$TAG"

ISAAC_PAT='[0-9]5_px4_single_drone_t650'
mkdir -p "$LOGDIR"
[[ -f "$REFS" ]] || { echo "ERROR: no such reference file: $REFS" >&2; exit 1; }

cleanup() {
  echo "[case] cleanup"
  tmux kill-session -t fsc_baseline_t650_stack 2>/dev/null
  tmux kill-session -t px4_isaac 2>/dev/null
  pkill -f "$ISAAC_PAT" 2>/dev/null
  pkill -x px4 2>/dev/null
  sleep 3
  pkill -9 -f "$ISAAC_PAT" 2>/dev/null
  pkill -9 -x px4 2>/dev/null
  pkill -f "[i]ndoor_state_estimator_node" 2>/dev/null
  pkill -f "[a]utopilot_sv_baseline_node" 2>/dev/null
  pkill -x MicroXRCEAgent 2>/dev/null
  sleep 1
}
trap cleanup EXIT
cleanup
sleep 2

# --- 1. DDS agent, before PX4 ---
MicroXRCEAgent udp4 -p 8888 >"$LOGDIR/xrce.log" 2>&1 &
sleep 2

# --- 2. PX4 SITL + IsaacSim T650 plant (the user-specified launcher) ---
echo "[case] starting $SIM_LAUNCHER"
DISPLAY=:0 setsid bash "$SIM_LAUNCHER" --in-terminal "$CFG" >"$LOGDIR/sim_launcher.log" 2>&1 &
for i in $(seq 1 180); do
  if tmux has-session -t px4_isaac 2>/dev/null && \
     tmux capture-pane -p -t px4_isaac:0.0 2>/dev/null | grep -q "Simulator connected"; then
    echo "[case] PX4<->Isaac HIL link up after ${i}s"; break
  fi
  sleep 1
done
echo "[case] waiting for EKF2 / ground-truth topics to settle"
sleep 30

# --- 3. the fsc_autopilot baseline stack (controller + estimator + mocap emulator + vrc) ---
echo "[case] starting $STACK_LAUNCHER"
DISPLAY=:0 setsid bash "$STACK_LAUNCHER" "$CFG" >"$LOGDIR/stack_launcher.log" 2>&1 &
sleep 35
tmux list-panes -t fsc_baseline_t650_stack -F '#{pane_index} #{pane_title}' \
  >"$LOGDIR/stack_panes.txt" 2>&1
for p in 0 1 2 3 4 5; do
  echo "----- pane $p -----" >>"$LOGDIR/stack_panes.txt"
  tmux capture-pane -p -t "fsc_baseline_t650_stack:stack.$p" >>"$LOGDIR/stack_panes.txt" 2>&1
done

# --- 4. drive the recorded reference sequence ---
echo "[case] driving reference sequence from $REFS"
bash -c "source '$ROS2_SETUP'; source '$WS_SETUP'; \
  exec /usr/bin/python3 '$SCRATCH/stack_driver.py' --refs '$REFS' --out '$OUT'" \
  2>&1 | tee "$LOGDIR/driver.log"

echo "[case] done"
sleep 2
