#!/bin/bash
# Which SHAPING terms of the reference have a per-inner-loop optimum?
#
# The flare was measured (scripts/ref_probe.py, 2026-09-22): the same commanded peak is
# delivered 83% by the geometric law and 111% by INDI, because at 16 m/s the WING supplies
# 85% of the pitch stiffness the reference is tracked with and INDI inverts it away.
# That argument is not special to the flare -- it applies to every shaping term whose value
# was chosen against how well ONE loop follows it.  This sweeps the other candidates and asks
# whether the OPTIMUM moves, not merely whether each still flies:
#   ttr          the forward speed ramp -- sets the corridor's AoA peak
#   decel_rate   the back-transition's speed ramp (documented boundary: 2.5 crashes)
#   th_slew_bt   the reference PITCH ramp rate, whose stated reason is that the OUTER
#                solver overshoots a step (an outer property, so the prediction is NO move)
# INDI runs with its own shipped reference (nmpc_cascade_indi_ref, flare 8) throughout.
S=${OUT:-$HOME/flight_bags/ref_series2}; mkdir -p "$S"
IP=~/envs/isaac_env/bin/python3
HERE=$(cd "$(dirname "$0")" && pwd)
BASE="--est mdob --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1 --noise_rate 1.0 --noise_acc 0.02 --l1"
run () {  # tag kind flags...
  local tag=$1 kind=$2; shift 2
  [ -s "$S/$tag.csv" ] && return
  timeout 1200 $IP "$HERE/ref_probe.py" $kind "$S/$tag.csv" -- $BASE "$@" > "$S/$tag.log" 2>&1
}
for s in 1 2; do
  for k in geo indi; do
    for t in 6 8 10 14;    do run "ttr${t}_${k}_s$s"   $k --ttr $t  --seed $s --decel_at 60 --dur 90 & done; wait
    for d in 1.5 2.0 2.5;  do run "dec${d}_${k}_s$s"   $k --ttr 10 --decel_rate $d --seed $s --decel_at 60 --dur 95 & done; wait
    for w in 4 8 16;       do run "ths${w}_${k}_s$s"   $k --ttr 10 --th_slew_bt $w --seed $s --decel_at 60 --dur 90 & done; wait
  done
done
echo CELLS_DONE
