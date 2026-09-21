#!/bin/bash
# A/B the flown "Adaptive NMPC + Geo" node (nmpc_cascade_imu_node) against its INDI variant
# (nmpc_cascade_indi_node) on the gate, case by case; the record of 2026-09-21 in CLAUDE.md is
# this script's output. One case = both nodes in parallel on the flown gate line, the same seed;
# outputs in $OUT/<case>_{imu,indi}.txt and one summary line per run.
#
#   bash scripts/indi_ab.sh hover|mission|model|turns|final   [OUT=dir]
#
# The variable flags go BEFORE the base line: the gate's argf takes the FIRST occurrence.
set -u
CTRL=${CTRL:-$HOME/envs/fsc_autopilot_ros2-master}
OUT=${OUT:-$HOME/flight_bags/indi_ab}; mkdir -p "$OUT"
PY=${PY:-$HOME/envs/isaac_env/bin/python3}
D=px4_vtol_direct_actuator/px4_vtol_direct_actuator
BASE="--test --est mdob --ttr 10 --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1 --noise_rate 1.0 --noise_acc 0.02 --l1"
GUIDE="20:wp 500 0 500 400 100 400;150:rth"
cd "$CTRL" || exit 1
git diff --quiet || { echo "controller tree is dirty -- commit first (a gate on an unrecorded tree is not a record)"; exit 1; }
run() {  # name, extra flags...
  local name=$1; shift
  timeout 900 $PY $D/nmpc_cascade_imu_node.py "$@" $BASE > "$OUT/${name}_imu.txt" 2>&1 &
  timeout 900 $PY $D/nmpc_cascade_indi_node.py "$@" $BASE > "$OUT/${name}_indi.txt" 2>&1 &
  wait
  for v in imu indi; do
    local f="$OUT/${name}_$v.txt"
    local ydiag; ydiag=$(awk '$1+0>0 && NF>10' "$f" | grep -o "mot=\[[^]]*\]" | tr -d 'mot=[]' | awk -F, '{d=($1+$4)-($2+$3); s+=d; ss+=d*d; n++} END {if(n>0){m=s/n; printf "%.3f", sqrt(ss/n-m*m)} else print "-"}')
    local turn; turn=$(awk '$1+0>=120 && $1+0<=145 && NF>10 {r=$9+0; s+=r; ss+=r*r; n++} $1+0>=30 && $1+0<=60 && NF>10 {if(-$9>pk)pk=-$9} END {if(n>0){m=s/n; printf "orbit roll %+.1f sd %.2f ", m, sqrt(ss/n-m*m)}; if(pk>0) printf "turn peak bank %.1f", pk}' "$f")
    printf "%-22s %-5s | %s | ydiag sd %s | %s %s\n" "$name" "$v" "$(grep -o 'worst |roll| *[0-9.]* deg *worst |yaw err| *[0-9.]* deg *solve fails [0-9/]*' "$f" | head -1)" "$ydiag" \
      "$(grep -o 'worst displacement from the held point [0-9.]* m\|GROUND at t=[0-9.]*\|RTH STOP t=[0-9.]*: [0-9.]* m' "$f" | head -1)" "$turn"
  done
}
case "${1:-help}" in
  hover)
    run hover_still --hover_only --dur 30 --every 2.0 --seed 3
    run hover_cg5 --plant_cg_dx 0.05 --hover_only --dur 30 --every 2.0 --seed 3
    run hover_cg5_rpy --plant_cg_dx 0.05 --indi_axes rpy --hover_only --dur 30 --every 2.0 --seed 3
    run hover_diagwind --wind -3.5,3.5,0 --airspeed --hover_only --dur 30 --every 2.0 --seed 3
    run hover_cross6 --wind 0,6,0 --airspeed --hover_only --dur 30 --every 2.0 --seed 3
    run hover_noise3 --noise_rate 3.0 --hover_only --dur 30 --every 2.0 --seed 3
    run hover_lat05 --lat 0.05 --comp 0.049 --plant_cg_dx 0.026 --hover_only --dur 30 --every 2.0 --seed 3
    ;;
  mission)
    run straight_s1 --dur 90 --every 1.0 --seed 1
    run straightBT_s1 --decel_at 60 --dur 100 --every 1.0 --seed 1
    run straight_lat05 --lat 0.05 --comp 0.049 --plant_cg_dx 0.026 --dur 90 --every 1.0 --seed 1
    run straight_lat08 --lat 0.08 --dur 90 --every 1.0 --seed 1
    run straight_spin18 --tau_spin 0.18 --l1_tau_spin 0.067 --dur 90 --every 1.0 --seed 1
    ;;
  model)
    run straight_cq05 --plant_cq 0.0095 --dur 90 --every 1.0 --seed 1
    run straight_cq2x --plant_cq 0.038 --dur 90 --every 1.0 --seed 1
    run straight_lat15 --plant_aero_scale lat=1.5 --dur 90 --every 1.0 --seed 1
    run straight_ctrl06 --plant_aero_scale ctrl=0.6 --dur 90 --every 1.0 --seed 1
    run straight_inertia06 --plant_inertia_scale 0.6 --dur 90 --every 1.0 --seed 1
    ;;
  turns)
    export VTOL_TANK_KG=0.502 VTOL_TANK_R=0.10 VTOL_TANK_DX=0.3
    for sd in 1 2 3; do run turns_s$sd --dur 260 --every 1.0 --seed $sd --guide "$GUIDE"; done
    run turns_cross5 --airspeed --wind 0,5,0 --dur 260 --every 1.0 --seed 1 --guide "$GUIDE"
    run turns_diag5 --airspeed --wind -3.5,3.5,0 --dur 260 --every 1.0 --seed 1 --guide "$GUIDE"
    run turns_yawfull_s2 --indi_yaw_fade 0 --dur 260 --every 1.0 --seed 2 --guide "$GUIDE"    # the unfaded yaw INDI: crashes
    run turns_rpy_s2 --indi_axes rpy --dur 260 --every 1.0 --seed 2 --guide "$GUIDE"          # pitch INDI: crashes the corridor
    ;;
  final)
    # the identity: --indi_off must reproduce the IMU node's reference hash (CLAUDE.md)
    timeout 600 $PY $D/nmpc_cascade_indi_node.py --indi_off $BASE --dur 8 --every 1.0 --seed 3 --log "$OUT/identity.csv" > "$OUT/identity.txt" 2>&1
    md5sum "$OUT/identity.csv"
    ;;
  *) sed -n 2,9p "$0"; exit 1 ;;
esac
