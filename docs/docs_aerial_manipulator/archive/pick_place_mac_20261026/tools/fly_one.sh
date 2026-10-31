#!/usr/bin/env bash
# One headless pick-and-place flight on one rig, wrapped in clean slates (2026-10-26).
#   fly_one.sh <wb|geo|mod> <tag> [extra env assignments for run_pnp.sh / the scene, e.g. PEGASUS_ARM_SPIN_DRAIN=1]
# The flight's files land in archive/pick_place_top_hat_20261008/runs/<tag>.*; this
# script copies them into ../runs/. Closes the simulation at the end.
set -o pipefail
RIG="$1"; TAG="$2"; shift 2
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$HOME/fsc_PegasusSimulator"
TOOLS="$PEG/docs/docs_aerial_manipulator/archive/pick_place_top_hat_20261008/tools"
SRC="$PEG/docs/docs_aerial_manipulator/archive/pick_place_top_hat_20261008/runs"
OUT="$HERE/../runs"; mkdir -p "$OUT"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$PEG/scripts/kill_stale_sim_processes.sh"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
declare -A ARGS=([wb]="--hook-place --timetable" [geo]="--hook-place --grip-via-action --grip-axis-tol 0.003 --timetable" [mod]="--hook-place --timetable")
for f in "$SRC/$TAG".*; do [[ -e "$f" ]] && { mkdir -p "$SRC/stale"; mv "$f" "$SRC/stale/"; }; done
clean
echo "[$(date +%T)] $RIG $TAG ($*)"
env "$@" PNP_DRIVER_ARGS="${PNP_DRIVER_ARGS:-${ARGS[$RIG]}}" bash "$TOOLS/run_pnp.sh" "$RIG" "$TAG" > "$SRC/$TAG.runner.log" 2>&1
echo "[$(date +%T)] rc=$?"
cp "$SRC/$TAG".* "$OUT/" 2>/dev/null
clean
echo "[$(date +%T)] closed"
