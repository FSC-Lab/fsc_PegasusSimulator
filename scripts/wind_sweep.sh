#!/usr/bin/env bash
# Airspeed and wind (hardware-transfer piece 4, 2026-09-15): the turning mission in a steady wind and
# in turbulence, N seeds per cell, the IMU-force-channel cascade (imu-mdob) WITH the pitot
# (--airspeed) -- the plant's wind through the gate's --wind/--turb, the controller's WindEstimator.
#
#   bash scripts/wind_sweep.sh <tag> [seed_lo seed_hi] [cells...]
#   env: JOBS=8 WIND_DIR=~/flight_bags/wind NOPITOT=1 (the flown controller, blind to the wind)
#   default cells: still, head3 head5 (wind -x: a headwind on the first leg), cross3 cross5 (+y),
#   crossm5 (-y), diag5 (-3.5,+3.5), turb05 turb10, head5_turb05
#   A tailwind on the FIRST leg (wind +x) is not a cell: the transition is flown downwind then, and
#   the pitot reads nothing in reversed flow -- the operational rule is "transition into the wind"
#   (piece 5 interlocks it); the mission's turns and orbit cover every other wind direction.
# Every run must start with a WARM JIT cache for its mode (nmpc6's build lock serialises a fresh key,
# so a cold sweep costs one build, not sixteen poisoned solvers). Scores: scripts/wind_metrics.py.
set -u
TAG=$1; shift
LO=1; HI=16
if [ $# -ge 2 ] && [[ "$1" =~ ^[0-9]+$ ]] && [[ "$2" =~ ^[0-9]+$ ]]; then LO=$1; HI=$2; shift 2; fi
ROOT=/home/pieper/envs/fsc_PegasusSimulator
WROOT=${WIND_DIR:-$HOME/flight_bags/wind}
D=$WROOT/$TAG; mkdir -p "$D"
JOBS=${JOBS:-8}
CTRL=${FSC_CONTROLLER_DIR:-$HOME/ros2_ws/src/px4_vtol_direct_actuator}   # the package root (controller_path.py's)
cd "$(dirname "$(realpath "$CTRL")")"                                      # the controller REPO root, as est_sweep.sh
if ! git diff --quiet -- px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_node.py px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_imu_node.py px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_so3_transition_node.py; then
  echo "!! the controller tree is DIRTY -- refusing to sweep on it"; git status --short; exit 1
fi
N=px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_imu_node.py
PY=~/envs/isaac_env/bin/python3
export VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} VTOL_TANK_R=${VTOL_TANK_R:-0.10} VTOL_TANK_DX=${VTOL_TANK_DX:-0.3}
FAITHFUL="--ttr 10 --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1 --noise_rate 1.0 --l1 --est mdob"
MISSION="--dur 260 --every 1.0"
GUIDE="20:wp 500 0 500 400 100 400;150:rth"
PITOT=${NOPITOT:+}; [ -z "${NOPITOT:-}" ] && PITOT="--airspeed"
git -C "$ROOT" rev-parse --short HEAD > "$D/sim_commit"; git rev-parse --short HEAD > "$D/ctrl_commit"
declare -A CELL=(
  [still]=""
  [head3]="--wind -3,0,0" [head5]="--wind -5,0,0" [head7]="--wind -7,0,0"
  [cross3]="--wind 0,3,0" [cross5]="--wind 0,5,0" [crossm5]="--wind 0,-5,0"
  [diag5]="--wind -3.5,3.5,0"
  [turb05]="--turb 0.5" [turb10]="--turb 1.0"
  [head5_turb05]="--wind -5,0,0 --turb 0.5"
)
CELLS=${*:-"still head3 head5 cross3 cross5 crossm5 diag5 turb05 turb10 head5_turb05"}
echo "cells=[$CELLS] seeds $LO-$HI pitot=${PITOT:-NONE} tank_kg=$VTOL_TANK_KG R=$VTOL_TANK_R dx=$VTOL_TANK_DX" > "$D/config"
run_one() {  # cell seed
  local c=$1 sd=$2 name="${1}_s${2}"
  for attempt in 1 2; do     # a process that died at start (a JIT-load crash) leaves no summary: run it again once
    timeout 1800 $PY $N --test ${CELL[$c]} $PITOT $FAITHFUL $MISSION --guide "$GUIDE" --seed "$sd" > "$D/$name.txt" 2>&1
    grep -q "worst |roll|" "$D/$name.txt" && break
    echo "  !! $name produced no summary (attempt $attempt)"; sleep 3
  done
  echo "  done $name: $(grep -o 'solve fails [0-9/]*' "$D/$name.txt" | tail -1)  $(grep -c GROUND "$D/$name.txt") ground"
}
n=0
for c in $CELLS; do
  [ -n "${CELL[$c]+x}" ] || { echo "unknown cell $c"; exit 2; }
  for sd in $(seq "$LO" "$HI"); do
    run_one "$c" "$sd" &
    n=$((n + 1)); if [ $((n % JOBS)) -eq 0 ]; then wait; fi
  done
done
wait
echo "sweep $TAG done -> $D"; $PY "$ROOT/scripts/wind_metrics.py" "$D" | tee "$D/summary.txt"
