#!/bin/bash
# Can each inner loop fly the FULL reference, and does the reference have to change for INDI?
S=${OUT:-$HOME/flight_bags/ref_series}; mkdir -p "$S"
IP=~/envs/isaac_env/bin/python3
BASE="--est mdob --ttr 10 --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1 --noise_rate 1.0 --noise_acc 0.02 --l1"
run () {  # run <tag> <kind> <extra flags...>
  local tag=$1 kind=$2; shift 2
  [ -s "$S/rs_$tag.csv" ] && return
  timeout 1200 $IP $(dirname "$0")/ref_probe.py $kind $S/rs_$tag.csv -- $BASE "$@" > $S/rs_$tag.log 2>&1
}
# 1. the full reference (straight mission + back-transition), 6 seeds, both loops
for s in 1 2 3 4 5 6; do
  for k in geo indi; do run "full_${k}_s$s" $k --seed $s --decel_at 60 --dur 90; done
done
# 2. the FLARE reference for INDI: the cap the 2026-09-13 piece tuned against the geometric loop
for s in 1 2 3; do
  for c in 8 7 6; do run "flare_indi${c}_s$s" indi --seed $s --decel_at 60 --dur 90 --aoa_cap $c; done
  run "flare_geo8_s$s" geo --seed $s --decel_at 60 --dur 90 --aoa_cap 8
done
# 3. the HOVER HOLD's gain: the geometric loop's standing pitch error was part of its braking
for k in geo indi; do
  for h in 0.5 0.8 1.2; do run "hold_${k}_k$h" $k --seed 1 --hover_only --dur 30 --hold_k $h; done
done
echo "SERIES DONE"
