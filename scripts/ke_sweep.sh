#!/usr/bin/env bash
# 16-seed (or any range) full-mission sweep of the TANK cascade on the offline gate, seeds in
# parallel, one trace per seed, one summary line. This is what every knife-edge / two-mode table
# in CLAUDE.md was produced with (2026-09-13).
#
#   VTOL_TANK_KG=0.7 bash scripts/ke_sweep.sh <tag> <seed_lo> <seed_hi> [extra harness args...]
#   e.g.  bash scripts/ke_sweep.sh kg07 1 16 --ttr 10 --lat 0.03 --comp 0.049 --slosh
#
# Extra args go FIRST on the harness line (argf takes the first occurrence, so they override the
# sweep's own --ttr 20 / noise / --decel_at 60 / --y0 10 / --outer_lag 0.03 defaults -- the last
# is the outer thread's solve->publish delay, which the node compensates and the gate, since
# 2026-09-13 evening, models and compensates the same way). Traces + summary.txt land in
# $KE_DIR (default ~/flight_bags/ke_sweeps/<tag>). Score with scripts/ke_metrics.py <dir>.
# KE_NODE=nmpc_cascade_steso_node.py + `--est leso|mdob --l1` sweeps the estimator variant.
# 16 parallel casadi processes loading one JIT cache have core-dumped a seed at START (no
# output at all) -- re-run such seeds serially before reading the count.
TAG=$1; LO=$2; HI=$3; shift 3
KE_ROOT=${KE_DIR:-$HOME/flight_bags/ke_sweeps}
D=$KE_ROOT/$TAG
mkdir -p "$D"
cd /home/pieper/envs/fsc_autopilot_ros2-master
# KE_NODE=nmpc_cascade_steso_node.py runs the estimator variant instead (its extra flags -- --est,
# --slosh_pred, the knobs -- go in the extra args; 2026-09-15, the estimator comparison's regression)
N=px4_vtol_direct_actuator/px4_vtol_direct_actuator/${KE_NODE:-nmpc_cascade_tank_node.py}
PY=~/envs/isaac_env/bin/python3
for sd in $(seq $LO $HI); do
  ( timeout 1500 $PY $N --test "$@" --ttr 20 --noise_att 0.1 --noise_rate 1.0 --dur 85 --decel_at 60 --y0 10 --outer_lag 0.03 --every 0.1 --seed $sd > $D/s$sd.txt 2>&1 ) &
done
wait
ok=0; line=""
for sd in $(seq $LO $HI); do
  f=$(grep -oE 'final: z=[0-9.]+' $D/s$sd.txt | grep -oE '[0-9.]+$')
  mz=$(awk '$1 ~ /^[0-9.]+$/ && $1+0>=60 {if(mz==""||$2+0<mz)mz=$2+0} END{print mz}' $D/s$sd.txt)
  case "$f" in 2[0-9].*|3[0-9].*) ok=$((ok+1));; esac
  line="$line s$sd:$f/min$mz"
done
echo "$TAG [VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} $*]: $ok/$((HI-LO+1)) ($line)" | tee -a "$KE_ROOT/summary.txt"
