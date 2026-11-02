#!/usr/bin/env bash
# One closed-loop case of the JGCD paper's controller on the Isaac Sim slung-load plant, no PX4:
#   Isaac (application/slungload/jgcd_x650_winch_direct.py, headless, paced to real time)
#   + ROS 2 controller node (JGCD2025-sim/ros/jgcd_controller_node.py).
#
#   scripts/isaacsim/run_jgcd_case.sh <config> <out_dir> <case> [controller args...]
#   e.g. scripts/isaacsim/run_jgcd_case.sh longhao_machine ~/source/JGCD2025-sim/results/isaac \
#            transfer3_fixed --scenario transfer3 --length fixed
#
# Env: JGCD_DURATION (Isaac sim seconds, default 16), JGCD_LOCKSTEP (0/1), JGCD_RTF (1.0),
#      JGCD_PHYSICS_HZ (250), PEGASUS_HEADLESS (1), JGCD_SIM_ROOT (~/source/JGCD2025-sim).
# Writes <out_dir>/<case>.npz (controller log), <case>_isaac.npz (plant truth + timing) and logs.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCRIPT_DIR/common_config.sh"
[[ $# -ge 3 ]] || { echo "usage: $0 <config> <out_dir> <case> [controller args...]"; exit 2; }
load_machine_config "$0" "$1"
OUT_DIR="$2"; CASE="$3"; shift 3
mkdir -p "$OUT_DIR"
JGCD_SIM_ROOT="${JGCD_SIM_ROOT:-$HOME/source/JGCD2025-sim}"
export LD_LIBRARY_PATH="$HOME/source/acados/lib:${LD_LIBRARY_PATH:-}"

APP="$FSC_PEGASUS_ROOT/application/slungload/jgcd_x650_winch_direct.py"
# A leftover Isaac instance (e.g. a lockstep run whose controller died) would keep publishing
# jgcd/state into the next case: clear stale instances and isolate each case on its own domain.
pkill -KILL -f "$APP" 2>/dev/null || true
pkill -KILL -f "ros/jgcd_controller_node.py" 2>/dev/null || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( (RANDOM % 90) + 10 ))}"

PEGASUS_HEADLESS="${PEGASUS_HEADLESS:-1}" JGCD_DURATION="${JGCD_DURATION:-16}" \
JGCD_LOCKSTEP="${JGCD_LOCKSTEP:-0}" JGCD_RTF="${JGCD_RTF:-1.0}" JGCD_PHYSICS_HZ="${JGCD_PHYSICS_HZ:-250}" \
JGCD_LOG="$OUT_DIR/${CASE}_isaac.npz" \
  setsid "$ISAAC_PY" "$APP" > "$OUT_DIR/${CASE}_isaac.log" 2>&1 &
ISAAC_PID=$!

# hard limit on the controller: a callback that never returns (e.g. a hung solver) must not stall a campaign
( cd "$JGCD_SIM_ROOT" && OMP_NUM_THREADS=1 timeout -k 10 "${JGCD_CTRL_TIMEOUT:-300}" .venv-ros/bin/python ros/jgcd_controller_node.py \
    --out "$OUT_DIR/${CASE}.npz" "$@" ) > "$OUT_DIR/${CASE}_ctrl.log" 2>&1 || true
# Isaac stops by itself after JGCD_DURATION; give it time to write its log
for _ in $(seq 1 120); do kill -0 "$ISAAC_PID" 2>/dev/null || break; sleep 1; done
# $ISAAC_PID is the python_r_fsc.sh wrapper (own session via setsid): end the whole group
kill -TERM -- "-$ISAAC_PID" 2>/dev/null || true
sleep 2
kill -KILL -- "-$ISAAC_PID" 2>/dev/null || true
grep -h "SUMMARY\|DIVERGED\|done (" "$OUT_DIR/${CASE}_isaac.log" "$OUT_DIR/${CASE}_ctrl.log" || true
