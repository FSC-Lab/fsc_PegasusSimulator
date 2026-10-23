#!/usr/bin/env bash
# One data point of the fused-feedback simulation campaign (2026-09-21).
#
#   run_fused.sh <l1_4d|l1_4d_fused> <tag> [machine-config]
#
# Flies the standard whole-body mission (wb_l1_tune_cycle.sh, the 7.15.5 steps +
# compatible trajectories) on the 4-D rig with either the raw-mocap stack
# (l1_4d) or the EKF2-fused stack (l1_4d_fused), and records the controller's
# feedback beside Isaac's ground truth with tools/fused_feedback_recorder.py.
# Same plant, same yaml, same mission, same driver -- only the estimator differs.
#
# Outputs, in this directory:
#   <which>_<tag>.npz      the campaign driver's log (odom + wb_control_debug)
#   fb_<which>_<tag>.npz   feedback vs ground truth
#   logs/                  stack / Pegasus / driver logs
set -uo pipefail
WHICH="${1:?usage: run_fused.sh <l1_4d|l1_4d_fused> <tag> [machine-config]}"
TAG="${2:?usage: run_fused.sh <l1_4d|l1_4d_fused> <tag> [machine-config]}"
CFG="${3:-shiqi_machine}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../../.." && pwd)"
mkdir -p "$HERE/logs"

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck source=/dev/null
source "$HOME/ros2_ws/install/setup.bash"
set -u

# PRE-CLEAN, then clear Fast DDS shared-memory state nobody holds. Killing live
# participants (the harness's own clean slate does it with kill) leaves their
# port mutexes behind in /dev/shm, and a later participant that draws the same
# port can no longer receive over shared memory. Doing the kill HERE, followed
# by the SHM cleanup, means the stack below starts on a clean /dev/shm; the
# harness's clean slate then finds nothing to kill.
"$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh" >/dev/null 2>&1
"$PEG/scripts/kill_stale_sim_processes.sh" -y >/dev/null 2>&1
sleep 5
timeout 30 fastdds shm clean >/dev/null 2>&1 || true
for f in /dev/shm/sem.fastrtps_port*_mutex; do
  [[ -e "$f" ]] || continue
  fuser -s "$f" 2>/dev/null || rm -f "$f"
done

# THE RECORDER RUNS ON UDP ONLY. On shiqi-desktop (2026-09-21) a participant
# started from a shell received PX4's agent-bridged topics but NOTHING from the
# ROS nodes over Fast DDS's default shared-memory transport (run A's recorder
# came back empty; `ros2 topic echo` hung the same way), while a UDP-only
# participant saw everything at once. Only the recorder gets the profile, so the
# stack under test keeps its default transport.
# The ROS 2 daemon wedges the same way and stalls the harness's `ros2 topic echo`
# checks -- restart it per run.
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true
FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/tools/fastdds_udp_only.xml" \
/usr/bin/python3 "$HERE/tools/fused_feedback_recorder.py" \
    --out "$HERE/fb_${WHICH}_${TAG}.npz" > "$HERE/logs/recorder_${WHICH}_${TAG}.log" 2>&1 &
REC=$!

WB_L1_OUT="$HERE" WB_L1_MISSION=standard \
  WB_L1_CLIENT_DDS_PROFILE="$HERE/tools/fastdds_udp_only.xml" \
  "$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh" "$WHICH" "$TAG" "$CFG"
rc=$?

kill -TERM "$REC" 2>/dev/null
wait "$REC"
tail -2 "$HERE/logs/recorder_${WHICH}_${TAG}.log"
exit $rc
