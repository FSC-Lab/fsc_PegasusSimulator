#!/bin/bash
# airframe_ctrl_gate.sh -- every flown controller of the cascade family, offline, on ONE airframe file
# (FSC_AIRFRAME_ID) and the tank the environment describes: the straight mission + back-transition
# (ke_sweep.sh's line, SEEDS seeds) and the turning mission in still air (wind_sweep.sh's flags, TSEEDS
# seeds). Written 2026-10-28 for "do the controllers work with the plan's airframe"; rerun it on the
# CFD file. It runs on a DIRTY controller tree on purpose (wind_sweep.sh refuses one) and records the
# tree state in OUT.
#   FSC_AIRFRAME_ID=<cfg> VTOL_TANK_KG=.. [VTOL_TANK_R/F1/ZETA/DX/DZ] bash scripts/airframe_ctrl_gate.sh [variant..]
#   variants: cascade tank adaptive imu indi aindi   (default: all)
#   OUT=~/flight_bags/ctrl_gate/<cfg basename>    score: ke_metrics.py OUT/straight/<v>, wind_metrics.py OUT/turns/<v>
#   LATS="0.05 0.065 0.08" adds the transport-delay sweep (LAT_VARIANTS, default "imu indi"; OUT/latency/summary.txt)
#   The airframe file's [tuning] section (if any) is what the variants fly: the gains come from the file.
set -u
ROOT=/home/pieper/envs/fsc_PegasusSimulator
CTRL=$HOME/envs/fsc_autopilot_ros2-master
PKG=px4_vtol_direct_actuator/px4_vtol_direct_actuator
PY=$HOME/envs/isaac_env/bin/python3
: "${FSC_AIRFRAME_ID:?set FSC_AIRFRAME_ID to the airframe file}"
export FSC_AIRFRAME_ID
export VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} VTOL_TANK_R=${VTOL_TANK_R:-0.10} VTOL_TANK_DX=${VTOL_TANK_DX:-0.3}
OUT=${OUT:-$HOME/flight_bags/ctrl_gate/$(basename "$FSC_AIRFRAME_ID" .cfg)}
SEEDS=${SEEDS:-8}; TSEEDS=${TSEEDS:-4}
mkdir -p "$OUT/straight" "$OUT/turns"
BASE="--l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh"
IMU="--est mdob --noise_acc 0.02"
# variant -> node + its launcher default (start_vtol_smc_hil.sh / start_slosh_hil.sh windows 6h..6k)
declare -A NODE=( [cascade]=nmpc_cascade_node.py [tank]=nmpc_cascade_tank_node.py
                  [adaptive]=nmpc_cascade_steso_node.py [imu]=nmpc_cascade_imu_node.py
                  [indi]=nmpc_cascade_indi_node.py [aindi]=nmpc_cascade_indi_node.py )
declare -A FLAGS=( [cascade]="$BASE" [tank]="$BASE" [adaptive]="$BASE --est steso"
                   [imu]="$BASE $IMU" [indi]="$BASE $IMU --indi_mode full --outer_fadapt 0"
                   [aindi]="$BASE $IMU --indi_mode full --outer_fadapt 1" )
VARIANTS=${*:-"cascade tank adaptive imu indi aindi"}
GUIDE="20:wp 500 0 500 400 100 400;150:rth"
TFLAGS="--outer_lag 0.03 --noise_att 0.1 --noise_rate 1.0 --airspeed --dur 260 --every 1.0"
cd "$CTRL"
{ echo "airframe=$FSC_AIRFRAME_ID af=$($PY -c "import sys; sys.path.insert(0,'$PKG'); import airframe_id as a; print(a.AIRFRAME.digest())")"
  env | grep -E '^VTOL_TANK' | sort | tr '\n' ' '; echo
  echo "ctrl=$(git rev-parse --short HEAD) sim=$(git -C "$ROOT" rev-parse --short HEAD)"
  git status --short; } > "$OUT/config"
for v in $VARIANTS; do
  [ -n "${NODE[$v]+x}" ] || { echo "unknown variant $v"; exit 2; }
  echo "[ctrl_gate] warm $v (${NODE[$v]})"     # one load first: a new airframe / fadapt is a new JIT key
  timeout 1200 $PY $PKG/${NODE[$v]} --test ${FLAGS[$v]} --dur 1 > "$OUT/warm_$v.log" 2>&1 \
    || echo "[ctrl_gate] !! warm $v exited $? (see $OUT/warm_$v.log)"
done
for v in $VARIANTS; do
  echo "[ctrl_gate] straight $v"
  KE_DIR=$OUT/straight KE_NODE=${NODE[$v]} bash "$ROOT/scripts/ke_sweep.sh" "$v" 1 "$SEEDS" ${FLAGS[$v]} > /dev/null 2>&1 &
done
wait
for v in $VARIANTS; do
  echo "[ctrl_gate] turns $v"
  mkdir -p "$OUT/turns/$v"
  for sd in $(seq 1 "$TSEEDS"); do
    ( timeout 1800 $PY $PKG/${NODE[$v]} --test ${FLAGS[$v]} $TFLAGS --guide "$GUIDE" --seed "$sd" \
        > "$OUT/turns/$v/still_s$sd.txt" 2>&1 ) &
  done
done
wait
# the transport-delay margin (2026-10-28): LATS="0.05 0.065 0.08" runs the straight mission at each
# --lat (the compensation held at the flown 0.049) for LAT_VARIANTS, LSEEDS seeds; ke_sweep's summary
# line per cell in $OUT/latency/summary.txt. Off unless LATS is set.
if [ -n "${LATS:-}" ]; then
  mkdir -p "$OUT/latency"
  for v in ${LAT_VARIANTS:-imu indi}; do
    for L in $LATS; do
      echo "[ctrl_gate] latency $v $L"
      KE_DIR=$OUT/latency KE_NODE=${NODE[$v]} bash "$ROOT/scripts/ke_sweep.sh" "${v}_lat$L" 1 "${LSEEDS:-3}" \
        --lat "$L" $(echo ${FLAGS[$v]} | sed 's/--lat [0-9.]*//') > /dev/null 2>&1 &
    done
  done
  wait
fi
echo "CTRL_GATE_DONE -> $OUT"
