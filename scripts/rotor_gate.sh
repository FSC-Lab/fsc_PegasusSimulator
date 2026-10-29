#!/bin/bash
# THE ROTOR TRADE'S GATE (2026-09-25): fly each candidate motor + propeller as its OWN airframe file
# (scripts/rotor_trade.py --cfg writes them: the propulsion lines and the mass / inertia moved, plant
# and controller matched), on the flown law (imu-mdob + L1, the offset 100 mm tank):
#   * the straight mission with the back-transition (ke_sweep, 8 seeds)  -> corridor / BT / fails / stop
#   * the turning mission in still air, 0.5 and 1.0 m/s turbulence, and 1.0 m/s at an 18 m/s cruise
#     (wind_sweep, 8 seeds)  -> the envelope where thrust headroom at speed showed before
#   bash scripts/rotor_gate.sh [tags...]     CFG=~/flight_bags/rotor_trade/cfg  OUT=~/flight_bags/rotor_trade
# Score: python3 scripts/ke_metrics.py <OUT>/straight/<tag> ; python3 scripts/wind_metrics.py <OUT>/wind/<tag>
set -u
ROOT=/home/pieper/envs/fsc_PegasusSimulator
CFG=${CFG:-$HOME/flight_bags/rotor_trade/cfg}
OUT=${OUT:-$HOME/flight_bags/rotor_trade}
TAGS=${*:-"ref a16x8 d4014_16x8 c14x10 b15x8"}
WIND_TAGS=${WIND_TAGS:-"ref a16x8 d4014_16x8 c14x10"}
WIND_CELLS=${WIND_CELLS:-"still turb05 turb10 turb10_vc18"}
SEEDS=${SEEDS:-8}
export VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} VTOL_TANK_R=${VTOL_TANK_R:-0.10} VTOL_TANK_DX=${VTOL_TANK_DX:-0.3}   # the offset 100 mm tank unless the caller sets another (the plan: 0.8 kg, DZ)
PY=~/envs/isaac_env/bin/python3
CTRL=$HOME/envs/fsc_autopilot_ros2-master
LINE="--est mdob --l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh --noise_acc 0.02"
mkdir -p "$OUT/straight" "$OUT/wind"
# one warm JIT build per airframe file, serially (a fresh key per file: the digest is in the cache key)
for t in $TAGS; do
  echo "[rotor_gate] warm $t"
  ( cd "$CTRL" && FSC_AIRFRAME_ID=$CFG/$t.cfg timeout 900 $PY px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_imu_node.py \
      --test $LINE --dur 1 > "$OUT/warm_$t.log" 2>&1 ) || echo "[rotor_gate] !! warm $t failed"
  grep -m1 -E "af=|airframe" "$OUT/warm_$t.log" | cut -c1-160
done
for t in $TAGS; do
  echo "[rotor_gate] straight $t"
  FSC_AIRFRAME_ID=$CFG/$t.cfg KE_DIR=$OUT/straight KE_NODE=nmpc_cascade_imu_node.py \
    bash "$ROOT/scripts/ke_sweep.sh" "$t" 1 "$SEEDS" $LINE > /dev/null 2>&1
done
for t in $WIND_TAGS; do
  echo "[rotor_gate] wind $t"
  FSC_AIRFRAME_ID=$CFG/$t.cfg WIND_DIR=$OUT/wind JOBS=10 bash "$ROOT/scripts/wind_sweep.sh" "$t" 1 "$SEEDS" $WIND_CELLS > "$OUT/wind_$t.log" 2>&1
done
echo "ROTOR_GATE_DONE -> $OUT"
