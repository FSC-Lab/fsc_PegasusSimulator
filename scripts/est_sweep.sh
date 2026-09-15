#!/usr/bin/env bash
# The residual-estimator comparison's closed-loop sweep (piece 4, 2026-09-15): the cascade
# variant on the offline gate, faithful line, one run per (estimator kind, predictor on/off,
# seed), traces + per-tick logs in $EST_DIR/<tag>/, then scripts/est_metrics.py <dir>.
#
#   bash scripts/est_sweep.sh <tag> <case> <seed_lo> <seed_hi> [extra harness args...]
#   case: turns  -- the offset 100 mm tank (VTOL_TANK_R 0.10, VTOL_TANK_DX 0.3) on the turning
#                   mission: legs -> orbit -> RTH stop (the case with a slosh moment to cancel)
#         hover  -- 30 s hover with a 5 cm CG error the controller is not told about
#   env: KINDS="steso leso mdob mdob22" PREDS="off on" JOBS=12 EST_DIR=~/flight_bags/est_sweeps
#   mdob22 = the momentum observer at its equal-LAG knob (K 22, 1.96x the noise floor).
# Extra args go FIRST on the harness line (argf takes the first occurrence). Remember: 16 parallel
# casadi processes loading one JIT cache have core-dumped a seed at start -- JOBS 12 by default,
# and est_metrics.py flags runs with no summary line.
set -u
TAG=$1; CASE=$2; LO=$3; HI=$4; shift 4
ROOT=/home/pieper/envs/fsc_PegasusSimulator
EST_ROOT=${EST_DIR:-$HOME/flight_bags/est_sweeps}
D=$EST_ROOT/$TAG; mkdir -p "$D"
KINDS=${KINDS:-"steso leso mdob mdob22"}; PREDS=${PREDS:-"off on"}; JOBS=${JOBS:-12}
cd /home/pieper/envs/fsc_autopilot_ros2-master
if ! git diff --quiet -- px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_steso_node.py px4_vtol_direct_actuator/px4_vtol_direct_actuator/so3_steso.py px4_vtol_direct_actuator/px4_vtol_direct_actuator/slosh_predictor.py; then
  echo "!! the controller tree is DIRTY (an audit agent once wrote into it) -- refusing to sweep on it"; git status --short; exit 1
fi
N=px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_steso_node.py
PY=~/envs/isaac_env/bin/python3
FAITHFUL="--ttr 10 --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1 --noise_rate 1.0 --l1"
case "$CASE" in
  turns) export VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} VTOL_TANK_R=${VTOL_TANK_R:-0.10} VTOL_TANK_DX=${VTOL_TANK_DX:-0.3}
         GUIDE="20:wp 500 0 500 400 100 400;150:rth"; MISSION="--dur 215 --every 1.0 --pred_score" ;;
  hover) export VTOL_TANK_KG=${VTOL_TANK_KG:-0.502}; unset VTOL_TANK_R VTOL_TANK_DX
         GUIDE=""; MISSION="--hover_only --plant_cg_dx 0.05 --dur 30 --every 1.0" ;;
  *) echo "case must be turns|hover"; exit 2 ;;
esac
git -C "$ROOT" rev-parse --short HEAD > "$D/sim_commit"; git rev-parse --short HEAD > "$D/ctrl_commit"
echo "$CASE kinds=[$KINDS] preds=[$PREDS] seeds $LO-$HI extra=[$*] tank_kg=$VTOL_TANK_KG R=${VTOL_TANK_R:-} dx=${VTOL_TANK_DX:-}" > "$D/config"
run_one() {  # kind pred seed
  local kind=$1 pred=$2 sd=$3 est=$1 extra="" node=$N; shift 3   # the rest of "$@" = the extra harness args
  [ "$kind" = mdob22 ] && { est=mdob; extra="--mdob_k 22"; }
  # imu-<moment kind>: the IMU-force-residual copy (nmpc_cascade_imu_node) with that moment observer
  case "$kind" in imu-*) est=${kind#imu-}; node=px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_imu_node.py ;; esac
  local name="${kind}_p${pred}_s${sd}"
  timeout 1800 $PY $node --test "$@" $extra --est "$est" --slosh_pred "$pred" $FAITHFUL $MISSION ${GUIDE:+--guide "$GUIDE"} --seed "$sd" \
      --log "$D/$name.csv" > "$D/$name.txt" 2>&1
  echo "  done $name: $(grep -o 'solve fails [0-9/]*' "$D/$name.txt" | tail -1)"
}
n=0
for sd in $(seq "$LO" "$HI"); do for kind in $KINDS; do for pred in $PREDS; do
  run_one "$kind" "$pred" "$sd" "$@" &
  n=$((n + 1)); if [ $((n % JOBS)) -eq 0 ]; then wait; fi
done; done; done
wait
echo "sweep $TAG done -> $D"; $PY "$ROOT/scripts/est_metrics.py" "$D" | tee "$D/summary.txt"
