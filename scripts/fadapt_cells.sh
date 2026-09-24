#!/bin/bash
# THE OUTER'S FORCE ADAPTATION (2026-09-23 night, "Adaptive MPC + INDI"): does it buy back the
# model-mismatch cells the robustness table fails? INDI full on the flown straight line (ke_sweep's
# mission: transition, cruise, back-transition at 60 s), adaptation off vs on, the same seeds, the
# PLANT perturbed and the controller at nominal. Score with scripts/fadapt_metrics.py.
#   bash scripts/fadapt_cells.sh [seed_lo seed_hi]      OUT=~/flight_bags/fadapt/cells
set -u
LO=${1:-1}; HI=${2:-4}
ROOT=/home/pieper/envs/fsc_PegasusSimulator
OUT=${OUT:-$HOME/flight_bags/fadapt/cells}; mkdir -p "$OUT"
LINE="--est mdob --l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh --noise_acc 0.02 --indi_mode full"
# name | plant-side flags (the robustness table's cells, scripts/cells_mn4014.txt and env1)
CELLS=(
  "nominal|"
  "j0_0.62|--plant_J0 0.62"
  "j0_0.71|--plant_J0 0.71"
  "lift_1.15|--plant_aero_scale lift=1.15"
  "lift_1.30|--plant_aero_scale lift=1.30"
  "thrust_0.80|--plant_thrust_scale 0.80"
)
git -C "$ROOT" rev-parse --short HEAD > "$OUT/sim_commit"
git -C "$HOME/envs/fsc_autopilot_ros2-master" rev-parse --short HEAD > "$OUT/ctrl_commit"
for c in "${CELLS[@]}"; do
  name=${c%%|*}; flags=${c#*|}
  # off and on for one cell in parallel (8 processes: 2 x the seeds); both solvers are cached by then
  KE_DIR="$OUT" KE_NODE=nmpc_cascade_indi_node.py bash "$ROOT/scripts/ke_sweep.sh" "${name}_off" "$LO" "$HI" $flags --outer_fadapt 0 $LINE > /dev/null 2>&1 &
  KE_DIR="$OUT" KE_NODE=nmpc_cascade_indi_node.py bash "$ROOT/scripts/ke_sweep.sh" "${name}_on" "$LO" "$HI" $flags --outer_fadapt 1 $LINE > /dev/null 2>&1 &
  wait
  echo "  done $name"
done
echo "FADAPT_DONE -> $OUT"
